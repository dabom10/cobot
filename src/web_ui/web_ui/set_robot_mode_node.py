import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import SetRobotMode

from web_ui.firebase_config import init_firebase, get_reference


class SetRobotModeNode(Node):
    def __init__(self):
        super().__init__('set_robot_mode_node')
        self.get_logger().info("Set Robot Mode Node 시작")

        init_firebase(self.get_logger())

        # Firebase command reference
        self.command_ref = get_reference('/robot_command/set_robot_mode')

        self.client = self.create_client(
            SetRobotMode,
            '/dsr01/system/set_robot_mode'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_robot_mode service...')

        self.get_logger().info('set_robot_mode service ready')

        # Firebase 리스너 설정
        self.command_ref.listen(self.on_command)

    def on_command(self, event):
        """Firebase에서 command 변경 감지"""
        if event.data is None:
            return

        command = event.data
        if command.get('execute') == True:
            mode = command.get('mode', 0)
            mode_names = {0: 'MANUAL', 1: 'AUTONOMOUS', 2: 'MEASURE'}
            self.get_logger().info(f'Set Robot Mode 명령 수신: {mode_names.get(mode, "UNKNOWN")}')
            self.execute_set_robot_mode(mode)
            # 실행 후 execute 플래그 초기화
            self.command_ref.update({'execute': False})

    def execute_set_robot_mode(self, mode):
        """SetRobotMode 서비스 호출"""
        req = SetRobotMode.Request()
        req.robot_mode = int(mode)

        future = self.client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Set Robot Mode 성공')
                self.command_ref.update({'status': 'success'})
            else:
                self.get_logger().error('Set Robot Mode 실패')
                self.command_ref.update({'status': 'failed'})
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.command_ref.update({'status': f'error: {e}'})


def main(args=None):
    rclpy.init(args=args)
    node = SetRobotModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
