import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

from web_ui.firebase_config import init_firebase, get_reference


class StatusProcessNode(Node):
    def __init__(self):
        super().__init__('status_process_node')
        self.get_logger().info("Status Process Node 시작")

        init_firebase(self.get_logger())

        # Firebase references
        self.status_ref = get_reference('/robot_status/dsr_status')
        self.process_ref = get_reference('/robot_status/dsr_process')

        # /dsr01/status 토픽 구독 (String)
        self.create_subscription(
            String,
            '/dsr01/status',
            self.status_callback,
            10
        )

        # /dsr01/process 토픽 구독 (Int32)
        self.create_subscription(
            Int32,
            '/dsr01/process',
            self.process_callback,
            10
        )

        self.get_logger().info('Subscribed to /dsr01/status and /dsr01/process')

    def status_callback(self, msg: String):
        """status 토픽 콜백"""
        try:
            self.status_ref.update({
                'value': msg.data,
                'last_update': time.time()
            })
        except Exception as e:
            self.get_logger().error(f'Status update failed: {e}')

    def process_callback(self, msg: Int32):
        """process 토픽 콜백"""
        try:
            self.process_ref.update({
                'value': msg.data,
                'last_update': time.time()
            })
        except Exception as e:
            self.get_logger().error(f'Process update failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = StatusProcessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
