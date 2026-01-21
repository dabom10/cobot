import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveLine

from web_ui.firebase_config import init_firebase, get_reference


class MoveLineNode(Node):
    def __init__(self):
        super().__init__('move_line_node')
        self.get_logger().info("Move Line Node 시작")

        init_firebase(self.get_logger())

        # Firebase command reference
        self.command_ref = get_reference('/robot_command/move_line')

        self.client = self.create_client(
            MoveLine,
            '/dsr01/motion/move_line'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            pass
        #self.get_logger().info('Waiting for move_line service...')

        self.get_logger().info('move_line service ready')

        # Firebase 리스너 설정
        self.command_ref.listen(self.on_command)

    def on_command(self, event):
        """Firebase에서 command 변경 감지"""
        if event.data is None:
            return

        command = event.data
        if command.get('execute') == True:
            self.get_logger().info('Move Line 명령 수신')
            self.execute_move_line(command)
            # 실행 후 execute 플래그 초기화
            self.command_ref.update({'execute': False})

    def execute_move_line(self, command):
        """MoveLine 서비스 호출"""
        req = MoveLine.Request()

        # pos: 목표 위치 [x, y, z, rx, ry, rz]
        #pos = command.get('pos', [400.0, 0.0, 300.0, 0.0, 180.0, 0.0])
        pos = command.get('pos', [0.0, 0.0, -300.0, 0.0, 0.0, 0.0])
        req.pos = [float(v) for v in pos]

        # 속도, 가속도 설정
        vel = command.get('vel', 100.0)
        acc = command.get('acc', 100.0)
        req.vel = [float(vel), float(vel)]
        req.acc = [float(acc), float(acc)]

        req.time = float(command.get('time', 0.0))
        req.radius = float(command.get('radius', 0.0))
        req.ref = int(command.get('ref', 0))  # DR_BASE
        req.mode = int(command.get('mode', 0))  # ABSOLUTE
        req.blend_type = int(command.get('blend_type', 0))
        req.sync_type = int(command.get('sync_type', 0))  # SYNC

        future = self.client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Move Line 성공')
                self.command_ref.update({'status': 'success'})
            else:
                self.get_logger().error('Move Line 실패')
                self.command_ref.update({'status': 'failed'})
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.command_ref.update({'status': f'error: {e}'})


def main(args=None):
    rclpy.init(args=args)
    node = MoveLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
