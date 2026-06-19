#!/usr/bin/env python3

import os
import re
import threading
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from cartographer_ros_msgs.srv import WriteState
from nav2_msgs.srv import SaveMap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


def sanitize_name(name: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
    return sanitized or "map"


def package_data_dir(package_name: str, relative_dir: str) -> Path:
    package_share = Path(get_package_share_directory(package_name))
    workspace_root = package_share.parents[3]
    candidates = (
        workspace_root / package_name / relative_dir,
        workspace_root / "src" / package_name / relative_dir,
        package_share / relative_dir,
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return package_share / relative_dir


class CartographerSaveMap(Node):
    def __init__(self) -> None:
        super().__init__("cartographer_save_map")
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter("maps_dir", str(package_data_dir("navigation", "map")))
        self.declare_parameter("map_name", "default")
        self.declare_parameter("write_state_service", "/write_state")
        self.declare_parameter("save_map_service", "/map_saver/save_map")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("service_timeout_sec", 15.0)
        self.declare_parameter("free_thresh", 0.25)
        self.declare_parameter("occupied_thresh", 0.65)

        self.write_state_client = self.create_client(
            WriteState,
            self.get_parameter("write_state_service").value,
            callback_group=self.callback_group,
        )
        self.save_map_client = self.create_client(
            SaveMap,
            self.get_parameter("save_map_service").value,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            Trigger,
            "/cartographer/save_map",
            self.handle_save_map,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Cartographer save-map service ready on /cartographer/save_map")

    def handle_save_map(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            map_name = sanitize_name(str(self.get_parameter("map_name").value))
            maps_dir = Path(os.path.expanduser(str(self.get_parameter("maps_dir").value)))
            output_dir = maps_dir / map_name
            output_dir.mkdir(parents=True, exist_ok=True)

            pbstream_path = output_dir / f"{map_name}.pbstream"
            map_prefix = output_dir / "map"

            timeout_sec = float(self.get_parameter("service_timeout_sec").value)
            self._wait_for_service(self.write_state_client, timeout_sec)
            self._wait_for_service(self.save_map_client, timeout_sec)

            write_request = WriteState.Request()
            write_request.filename = str(pbstream_path)
            write_request.include_unfinished_submaps = True
            write_result = self._call_service(self.write_state_client, write_request, timeout_sec)
            status = getattr(write_result, "status", None)
            if status is not None and getattr(status, "code", 0) != 0:
                message = getattr(status, "message", "unknown WriteState error")
                raise RuntimeError(f"Cartographer WriteState failed: {message}")
            if not pbstream_path.exists() or pbstream_path.stat().st_size == 0:
                raise RuntimeError(f"Cartographer did not create pbstream: {pbstream_path}")

            save_request = SaveMap.Request()
            save_request.map_topic = str(self.get_parameter("map_topic").value)
            save_request.map_url = str(map_prefix)
            save_request.image_format = "pgm"
            save_request.map_mode = "trinary"
            save_request.free_thresh = float(self.get_parameter("free_thresh").value)
            save_request.occupied_thresh = float(self.get_parameter("occupied_thresh").value)
            save_result = self._call_service(self.save_map_client, save_request, timeout_sec)
            if not getattr(save_result, "result", False):
                raise RuntimeError("Nav2 map_saver reported failure")

            yaml_path = output_dir / "map.yaml"
            pgm_path = output_dir / "map.pgm"
            if not yaml_path.exists():
                raise RuntimeError(f"map_saver did not create YAML: {yaml_path}")
            if not pgm_path.exists():
                raise RuntimeError(f"map_saver did not create image: {pgm_path}")

            response.success = True
            response.message = (
                f"saved map '{map_name}': pbstream={pbstream_path}, yaml={yaml_path}"
            )
        except Exception as exc:  # noqa: BLE001 - surface service failures to the caller
            self.get_logger().error("Failed to save map: %s", exc)
            response.success = False
            response.message = str(exc)
        return response

    def _wait_for_service(self, client, timeout_sec: float) -> None:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"service is not available: {client.srv_name}")

    def _call_service(self, client, request, timeout_sec: float):
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_sec):
            raise RuntimeError(f"service call timed out: {client.srv_name}")
        return future.result()


def main() -> None:
    rclpy.init()
    node = CartographerSaveMap()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
