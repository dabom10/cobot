import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveHome

from web_ui.firebase_config import init_firebase, get_reference


class MoveHomeNode(Node):
    def __init__(self):
        super().__init__('move_home_node')
        self.get_logger().info("Move Home Node 시작")

        init_firebase(self.get_logger())

        # Firebase command reference
        self.command_ref = get_reference('/robot_command/move_home')

        self.client = self.create_client(
            MoveHome,
            '/dsr01/motion/move_home'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_home service...')

        self.get_logger().info('move_home service ready')

        # Firebase 리스너 설정
        self.command_ref.listen(self.on_command)

    def on_command(self, event):
        """Firebase에서 command 변경 감지"""
        if event.data is None:
            return

        command = event.data
        if command.get('execute') == True:
            self.get_logger().info('Move Home 명령 수신')
            self.execute_move_home(command)
            # 실행 후 execute 플래그 초기화
            self.command_ref.update({'execute': False})

    def execute_move_home(self, command):
        """MoveHome 서비스 호출"""
        req = MoveHome.Request()

        # target: 0 = Mechanical home (0,0,0,0,0,0), 1 = User home
        req.target = int(command.get('target', 1))

        future = self.client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Move Home 성공')
                self.command_ref.update({'status': 'success'})
            else:
                self.get_logger().error(f'Move Home 실패: res={response.res}')
                self.command_ref.update({'status': 'failed'})
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.command_ref.update({'status': f'error: {e}'})


def main(args=None):
    rclpy.init(args=args)
    node = MoveHomeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
