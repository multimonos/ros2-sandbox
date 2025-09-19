import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from typing import Final

NUMBER_TOPIC: Final = "number"


class NumberPublisher(Node):
    def __init__(self):
        super().__init__("number_pub")

        self.number = 3

        self.get_logger().info(f"topic={NUMBER_TOPIC}")

        queue_size = 10
        self.publisher = self.create_publisher(Int64, NUMBER_TOPIC, queue_size)

        self.create_timer(1, self.publish)

    def publish(self):
        msg = Int64()
        msg.data = self.number
        self.publisher.publish(msg)


def main(args: list[str] | None = None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
