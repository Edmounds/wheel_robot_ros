from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node

from gesture_control.srv import SetBodyHeight

from .gesture_classifier import HandState
from .qnn_backend import QualcommHandBackend


@dataclass(frozen=True)
class GestureDecision:
    gesture: str
    stable: bool


class GestureControlNode(Node):
    def __init__(self) -> None:
        super().__init__("gesture_control_node")

        self.declare_parameter("camera_path", "/dev/video2")
        self.declare_parameter("backend", "qnn")
        self.declare_parameter("qualcomm_assets_dir", "")
        self.declare_parameter("qualcomm_model_dir", "")
        self.declare_parameter("detection_model", "m_handDetctor_w8a8.qnn216.ctx.bin")
        self.declare_parameter("landmark_model", "m_handLandmark_w8a8.qnn216.ctx.bin")
        self.declare_parameter("anchors_file", "anchors_palm.npy")
        self.declare_parameter("min_detection_score", 0.75)
        self.declare_parameter("min_hand_score", 0.4)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("height_service_name", "/gesture_control/set_body_height")
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("gesture_stable_frames", 4)
        self.declare_parameter("gesture_event_cooldown_sec", 1.2)
        self.declare_parameter("lost_hand_timeout_sec", 0.5)
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("max_angular_speed", 0.60)
        self.declare_parameter("forward_speed", 0.15)
        self.declare_parameter("turn_angular_speed", 0.45)
        self.declare_parameter("fail_on_backend_error", False)
        self.declare_parameter("show_debug_image", False)

        self._cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )
        self._height_client = self.create_client(
            SetBodyHeight,
            self.get_parameter("height_service_name").value,
        )

        self._gesture_history: Deque[str] = deque(
            maxlen=max(1, int(self.get_parameter("gesture_stable_frames").value))
        )
        self._last_event_times: dict[str, float] = {}
        self._last_seen_hand_time = time.monotonic()
        self._last_stable_gesture: Optional[str] = None
        self._active_mode = "stop"
        self._backend = None
        self._capture = None
        self._cv2 = None

        if not self._start_backend():
            return
        if not self._open_camera():
            self._close_backend()
            return

        period = 1.0 / max(1.0, float(self.get_parameter("control_rate_hz").value))
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info("gesture control node started")

    def destroy_node(self) -> bool:
        self._publish_stop()
        if self._capture is not None:
            self._capture.release()
        self._close_backend()
        if self._cv2 is not None and bool(self.get_parameter("show_debug_image").value):
            self._cv2.destroyAllWindows()
        return super().destroy_node()

    def _start_backend(self) -> bool:
        backend_name = str(self.get_parameter("backend").value).strip().lower()
        if backend_name != "qnn":
            self.get_logger().error(f"unsupported gesture backend: {backend_name}")
            return False

        assets_dir = str(self.get_parameter("qualcomm_assets_dir").value).strip()
        model_dir = str(self.get_parameter("qualcomm_model_dir").value).strip()
        default_assets_dir = (
            Path(get_package_share_directory("gesture_control")) / "assets" / "qualcomm_hand"
        )
        resolved_assets_dir = Path(assets_dir).expanduser() if assets_dir else default_assets_dir
        resolved_model_dir = (
            Path(model_dir).expanduser() if model_dir else resolved_assets_dir / "models"
        )

        self._backend = QualcommHandBackend(
            model_dir=str(resolved_model_dir),
            assets_dir=str(resolved_assets_dir),
            detection_model=str(self.get_parameter("detection_model").value),
            landmark_model=str(self.get_parameter("landmark_model").value),
            anchors_file=str(self.get_parameter("anchors_file").value),
            min_detection_score=float(self.get_parameter("min_detection_score").value),
            min_hand_score=float(self.get_parameter("min_hand_score").value),
        )

        try:
            self._backend.start()
            return True
        except Exception as exc:  # noqa: BLE001 - ROS node must report backend setup failures.
            message = f"failed to start Qualcomm QNN hand backend: {exc}"
            if bool(self.get_parameter("fail_on_backend_error").value):
                raise RuntimeError(message) from exc
            self.get_logger().error(message)
            self.get_logger().error("node stays alive, but no camera control loop was started")
            self._backend = None
            return False

    def _open_camera(self) -> bool:
        import cv2

        self._cv2 = cv2
        camera_path = self._parse_camera_path(str(self.get_parameter("camera_path").value))
        self._capture = cv2.VideoCapture(camera_path)
        if not self._capture.isOpened():
            self.get_logger().error(f"cannot open camera: {camera_path}")
            return False
        return True

    def _on_timer(self) -> None:
        if self._capture is None or self._backend is None:
            return

        ok, frame = self._capture.read()
        if not ok:
            self.get_logger().warn("failed to read frame from camera")
            self._stop_if_hand_lost()
            return

        try:
            hands = self._backend.detect(frame)
        except Exception as exc:  # noqa: BLE001 - keep robot control node alive.
            self.get_logger().error(f"gesture detection failed: {exc}")
            self._enter_stop(clear_gesture=True)
            return

        if not hands:
            self._stop_if_hand_lost()
            self._show_debug(frame, None)
            return

        hand = max(hands, key=lambda item: item.score)
        self._last_seen_hand_time = time.monotonic()
        decision = self._stable_gesture(hand.gesture)
        self._apply_control(decision)
        self._show_debug(frame, hand)

    def _stable_gesture(self, gesture: str) -> GestureDecision:
        self._gesture_history.append(gesture)
        if len(self._gesture_history) < self._gesture_history.maxlen:
            return GestureDecision(gesture=gesture, stable=False)
        stable = all(item == gesture for item in self._gesture_history)
        return GestureDecision(gesture=gesture, stable=stable)

    def _apply_control(self, decision: GestureDecision) -> None:
        gesture = decision.gesture

        if not decision.stable:
            self._publish_active_mode()
            return

        previous_stable_gesture = self._last_stable_gesture
        self._last_stable_gesture = gesture

        if gesture == "Open_Palm":
            self._enter_stop()
            return

        if gesture == "Pointing_Up":
            self._active_mode = "forward"
            self._publish_forward()
            return

        if gesture == "Closed_Fist":
            self._active_mode = "turn"
            self._publish_turn()
            return

        if gesture == "Thumb_Up":
            if previous_stable_gesture != gesture:
                self._call_height_service(1, gesture)
            self._enter_stop()
            return

        if gesture == "Victory":
            if previous_stable_gesture != gesture:
                self._call_height_service(0, gesture)
            self._enter_stop()
            return

        self._publish_active_mode()

    def _publish_active_mode(self) -> None:
        if self._active_mode == "forward":
            self._publish_forward()
        elif self._active_mode == "turn":
            self._publish_turn()

    def _publish_forward(self) -> None:
        twist = Twist()
        max_linear_speed = abs(float(self.get_parameter("max_linear_speed").value))
        twist.linear.x = self._clamp(
            float(self.get_parameter("forward_speed").value),
            -max_linear_speed,
            max_linear_speed,
        )
        self._cmd_vel_pub.publish(twist)

    def _publish_turn(self) -> None:
        twist = Twist()
        max_angular_speed = abs(float(self.get_parameter("max_angular_speed").value))
        twist.angular.z = self._clamp(
            float(self.get_parameter("turn_angular_speed").value),
            -max_angular_speed,
            max_angular_speed,
        )
        self._cmd_vel_pub.publish(twist)

    def _publish_stop(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _enter_stop(self, clear_gesture: bool = False) -> None:
        self._active_mode = "stop"
        if clear_gesture:
            self._gesture_history.clear()
            self._last_stable_gesture = None
        self._publish_stop()

    def _stop_if_hand_lost(self) -> None:
        timeout = float(self.get_parameter("lost_hand_timeout_sec").value)
        if time.monotonic() - self._last_seen_hand_time >= timeout:
            self._enter_stop(clear_gesture=True)

    def _call_height_service(self, command: int, gesture: str) -> None:
        now = time.monotonic()
        cooldown = float(self.get_parameter("gesture_event_cooldown_sec").value)
        if now - self._last_event_times.get(gesture, 0.0) < cooldown:
            return
        if not self._height_client.service_is_ready():
            self.get_logger().warn(
                f"height service is not ready: {self.get_parameter('height_service_name').value}"
            )
            return

        self._last_event_times[gesture] = now
        request = SetBodyHeight.Request()
        request.command = int(command)
        future = self._height_client.call_async(request)
        future.add_done_callback(
            lambda done: self._log_height_response(done, command)
        )

    def _log_height_response(self, future, command: int) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS async service callback.
            self.get_logger().warn(f"height command {command} failed: {exc}")
            return
        if response.ok:
            self.get_logger().info(f"height command {command} accepted: {response.message}")
        else:
            self.get_logger().warn(f"height command {command} rejected: {response.message}")

    def _show_debug(self, frame, hand: Optional[HandState]) -> None:
        if self._cv2 is None or not bool(self.get_parameter("show_debug_image").value):
            return
        if hand is not None:
            height, width = frame.shape[:2]
            point = (int(hand.wrist_x * width), int(hand.wrist_y * height))
            self._cv2.circle(frame, point, 8, (0, 255, 0), -1)
            self._cv2.putText(
                frame,
                hand.gesture,
                (max(0, point[0] - 40), max(30, point[1] - 20)),
                self._cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                self._cv2.LINE_AA,
            )
        self._cv2.imshow("gesture_control", frame)
        self._cv2.waitKey(1)

    def _close_backend(self) -> None:
        if self._backend is not None:
            self._backend.close()
            self._backend = None

    @staticmethod
    def _parse_camera_path(value: str):
        stripped = value.strip()
        if stripped.isdigit():
            return int(stripped)
        return stripped

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return min(max(value, low), high)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GestureControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
