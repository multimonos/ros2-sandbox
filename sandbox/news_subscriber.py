import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class RobotNewsSubscriber(Node):
    def __init__(self):
        super().__init__("news_sub")

        self.count = 0
        self.topic = "robonews"
        queue_size = 10
        self.subscriber = self.create_subscription(
            String, self.topic, self.on_message, queue_size
        )

        self.get_logger().info("news: started")
        self.get_logger().info(f"news: topic={self.topic}")

    def on_message(self, msg: String):
        self.get_logger().info(f"data={msg.data}, cnt={self.count}")
        self.count += 1


def main(args: list[str] | None = None):
    rclpy.init(args=args)
    node = RobotNewsSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
