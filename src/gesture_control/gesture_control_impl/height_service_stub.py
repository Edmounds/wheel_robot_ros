from __future__ import annotations

import rclpy
from rclpy.node import Node

from gesture_control.srv import SetBodyHeight


class HeightServiceStub(Node):
    def __init__(self) -> None:
        super().__init__("height_service_stub_node")
        self.declare_parameter("height_service_name", "/gesture_control/set_body_height")
        service_name = str(self.get_parameter("height_service_name").value)
        self._service = self.create_service(SetBodyHeight, service_name, self._handle_request)
        self.get_logger().info(f"height service stub ready: {service_name}")

    def _handle_request(self, request, response):
        if request.command not in (0, 1):
            response.ok = False
            response.message = "command must be 1 for raise or 0 for lower"
            return response

        action = "raise" if request.command == 1 else "lower"
        self.get_logger().info(f"received body height command: {request.command} ({action})")
        response.ok = True
        response.message = f"stub accepted {action}"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeightServiceStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
