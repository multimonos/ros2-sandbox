import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class RobotNewsStation(Node):
    def __init__(self):
        super().__init__("news_pub")

        self.count = 0
        self.topic = "robonews"

        self.get_logger().info("news: started")
        self.get_logger().info(f"news: topic={self.topic}")

        # create published
        queue_size = 10
        self.publisher = self.create_publisher(String, self.topic, queue_size)

        # publish news regularly
        self.create_timer(0.25, self.publish_news)

    def publish_news(self):
        msg = String()
        msg.data = f"article : id={self.count}"

        self.publisher.publish(msg)

        self.count += 1


def main(args: list[str] | None = None):
    rclpy.init(args=args)
    node = RobotNewsStation()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
