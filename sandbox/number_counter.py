from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from typing import Final

NUMBER_TOPIC: Final = "number"
NUMBER_COUNT_TOPIC: Final = "number_count"


class NumberCounter(Node):
    def __init__(self):
        super().__init__("number_counter")

        self.count: int = 0
        self.queue_size = 10

        self.get_logger().info(f"topic={NUMBER_COUNT_TOPIC}")

        self.subscriber = self.create_subscription(
            Int64, NUMBER_TOPIC, self.on_number, self.queue_size
        )

        self.publisher = self.create_publisher(Int64, NUMBER_COUNT_TOPIC, self.queue_size)

        self.reset_svc = self.create_service(SetBool, "number_counter_reset", self.on_reset)

    def on_number(self, msg: Int64):
        """consume a number from /number"""
        self.count += msg.data
        self.get_logger().info(f"cnt: {self.count}")
        self.send_count(self.count)

    def send_count(self, cnt: int):
        """publish a value to /number_counter"""
        acc = Int64()
        acc.data = cnt
        self.publisher.publish(acc)

    def on_reset(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """reset number counter service"""
        self.get_logger().info(f"reset: {request.data}")

        if request.data:
            self.count = 0

        response.success = request.data
        return response


def main(args: list[str] | None = None):
    rclpy.init(args=args)
    node = NumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
