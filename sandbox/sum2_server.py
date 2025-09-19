import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class Sum2Server(Node):
    def __init__(self):
        super().__init__("sum2_server")

        self.server = self.create_service(AddTwoInts, "sum2_server", self.on_request)
        self.get_logger().info("started")

    def on_request(
        self, request: AddTwoInts.Request, response: AddTwoInts.Response
    ) -> AddTwoInts.Response:
        """sum two integers"""
        response.sum = request.a + request.b
        self.get_logger().info(f"sum: {request.a} + {request.b} = {response.sum}")
        return response


def main(args: list[str] | None = None):
    rclpy.init(args=args)
    node = Sum2Server()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
