import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from web_ui.firebase_config import init_firebase, get_reference


class ErrorFirebaseNode(Node):
    def __init__(self):
        super().__init__('error_firebase_node')
        self.get_logger().info("Error Firebase Node 시작")

        init_firebase(self.get_logger())

        self.error_ref = get_reference('/robot_status/error')

        # 초기 상태 설정
        self.error_ref.update({
            'message': '',
            'has_error': False,
            'last_update': time.time()
        })

        self.create_subscription(
            String,
            '/dsr01/error',
            self.error_callback,
            10
        )

        self.get_logger().info('Error topic subscription ready')

    def error_callback(self, msg: String):
        error_message = msg.data
        has_error = bool(error_message and error_message.strip())

        self.get_logger().warn(f'Error 수신: {error_message}')

        self.error_ref.update({
            'message': error_message,
            'has_error': has_error,
            'last_update': time.time()
        })


def main(args=None):
    rclpy.init(args=args)
    node = ErrorFirebaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
