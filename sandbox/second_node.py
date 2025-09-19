#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class SecondNode(Node):
    def __init__(self):
        super().__init__("node2")
        self.get_logger().info("node2: ohai")
        self.create_timer(0.25, self.heartbeat)
        self._count = 0

    def heartbeat(self):
        self.get_logger().info(f"node2: heartbeat U={self._count}")
        self._count += 1


def main(args: list[str] | None = None):
    rclpy.init(args=args)
    node = SecondNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
