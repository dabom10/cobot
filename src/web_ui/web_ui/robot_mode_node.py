import time

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetRobotMode

from web_ui.firebase_config import init_firebase, get_reference


class RobotModeNode(Node):
    def __init__(self):
        super().__init__('robot_mode_node')
        self.get_logger().info("Robot Mode Node 시작")

        init_firebase(self.get_logger())

        self.mode_ref = get_reference('/robot_status/robot_mode')

        self.client = self.create_client(
            GetRobotMode,
            '/dsr01/system/get_robot_mode'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_robot_mode service...')

        self.get_logger().info('get_robot_mode service ready')

        # 2초마다 로봇 모드 조회
        self.timer = self.create_timer(2.0, self.query_robot_mode)

        # 초기 조회
        self.query_robot_mode()

    def query_robot_mode(self):
        """GetRobotMode 서비스 호출"""
        req = GetRobotMode.Request()

        future = self.client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response is not None and response.success:
                # robot_mode: 0=MANUAL, 1=AUTONOMOUS, 2=MEASURE
                mode_names = {
                    0: 'MANUAL',
                    1: 'AUTONOMOUS',
                    2: 'MEASURE'
                }
                mode_name = mode_names.get(response.robot_mode, 'UNKNOWN')

                mode_data = {
                    'robot_mode': mode_name,
                    'robot_mode_code': response.robot_mode,
                    'success': response.success,
                    'last_update': time.time()
                }

                self.mode_ref.update(mode_data)
                self.get_logger().info(f'Robot Mode: {mode_name}')
        except Exception as e:
            self.get_logger().error(f'GetRobotMode failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
