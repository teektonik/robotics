from full_name_interfaces.srv import FullName
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(FullName, 'create_full_name', self.create_full_name_callback)

    def create_full_name_callback(self, request, response):
        response.full_name = request.a + " " + request.b + " " + request.c
        self.get_logger().info('Incoming request\nlast name: %s \nname: %s \nfirst name: %s' % (request.a, request.b, request.c))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()