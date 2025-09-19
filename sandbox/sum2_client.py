from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from example_interfaces.srv import AddTwoInts


class Sum2Client(Node):
    def __init__(self):
        super().__init__("sum2_client")
        self.get_logger().info("started")

        service_name = "sum2ints"
        self.client = self.create_client(AddTwoInts, service_name)
        self.result: int | None = None

    def sum(self, a: int, b: int) -> Future:
        # wait for the service
        while not self.client.wait_for_service(1.0):
            self.get_logger().info(f"waiting for {self.client.service_name} ...")
        self.get_logger().info(f"found {self.client.service_name}")

        # create request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # send async
        future = self.client.call_async(request)

        # not sure this is really required if we are returning a future
        future.add_done_callback(self.on_complete)

        return future

    def on_complete(self, future: Future) -> None:
        self.get_logger().info("got future")
        try:
            response: AddTwoInts.Response | None = future.result()

            if response is not None:
                self.result = response.sum
                self.get_logger().info(f"response: {response}")

        except Exception as e:
            self.get_logger().error(f"{e}")


def main(args: list[str] | None = None):
    rclpy.init(args=args)

    node = Sum2Client()

    fut = node.sum(5, 7)
    rclpy.spin_until_future_complete(node, fut)

    response = fut.result()

    if response:
        node.get_logger().info(f"sum={response.sum}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
