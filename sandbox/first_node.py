import rclpy
from rclpy.node import Node


def main(args: list[str] | None = None):
    rclpy.init(args=args)
    node = Node("node1")
    node.get_logger().info("hello world")
    rclpy.spin(node)  # keep alive until ctrl+c
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
