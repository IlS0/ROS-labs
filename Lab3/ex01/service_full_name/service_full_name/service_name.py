from full_name_message.srv import FullNameSumService

import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(FullNameSumService, 'summ_full_name', self.summ_full_name_callback)

    def summ_full_name_callback(self, request, response):
        response.full_name = request.last_name + ' ' + request.name + ' ' + request.first_name
        return response


def main():
    rclpy.init()

    service = Service()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()