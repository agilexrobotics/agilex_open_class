import rclpy
from rclpy.node import Node
from limo_msgs.srv import LimoSrv
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

class LimoClient(Node):

    def __init__(self):
        super().__init__('limo_client')
        self.client = self.create_client(LimoSrv, 'limo_srv')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        request = LimoSrv.Request()
        request.x = 0.5
        request.y = 0.0
        request.z = 0.0

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = LimoClient()
    response = node.send_request()

    if response is not None:
        node.get_logger().info(f'success.data: {response.success}')
    else:
        node.get_logger().error('Failed to call service limo_srv')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
