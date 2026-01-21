import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveCircle
from std_msgs.msg import Float64MultiArray

from web_ui.firebase_config import init_firebase, get_reference


class MoveCircleNode(Node):
    def __init__(self):
        super().__init__('move_circle_node')
        self.get_logger().info("Move Circle Node 시작")

        init_firebase(self.get_logger())

        # Firebase command reference
        self.command_ref = get_reference('/robot_command/move_circle')

        self.client = self.create_client(
            MoveCircle,
            '/dsr01/motion/move_circle'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            pass
            #self.get_logger().info('Waiting for move_circle service...')

        self.get_logger().info('move_circle service ready')

        # Firebase 리스너 설정
        self.command_ref.listen(self.on_command)

    def on_command(self, event):
        """Firebase에서 command 변경 감지"""
        if event.data is None:
            return

        command = event.data
        if command.get('execute') == True:
            self.get_logger().info('Move Circle 명령 수신')
            self.execute_move_circle(command)
            # 실행 후 execute 플래그 초기화
            self.command_ref.update({'execute': False})

    def execute_move_circle(self, command):
        """MoveCircle 서비스 호출"""
        req = MoveCircle.Request()

        # pos: 2개의 위치 (중간점, 끝점)
        # 기본값 설정 (사용자가 Firebase에서 설정 가능)
        pos1 = command.get('pos1', [400.0, 100.0, 300.0, 0.0, 180.0, 0.0])
        pos2 = command.get('pos2', [400.0, -100.0, 300.0, 0.0, 180.0, 0.0])

        pos1_msg = Float64MultiArray()
        pos1_msg.data = [float(v) for v in pos1]
        pos2_msg = Float64MultiArray()
        pos2_msg.data = [float(v) for v in pos2]
        req.pos = [pos1_msg, pos2_msg]

        # 속도, 가속도 설정
        req.vel = [float(command.get('vel', 100.0)), float(command.get('vel', 100.0))]
        req.acc = [float(command.get('acc', 100.0)), float(command.get('acc', 100.0))]

        req.time = float(command.get('time', 0.0))
        req.radius = float(command.get('radius', 0.0))
        req.ref = int(command.get('ref', 0))  # DR_BASE
        req.mode = int(command.get('mode', 0))  # ABSOLUTE
        req.angle1 = float(command.get('angle1', 0.0))
        req.angle2 = float(command.get('angle2', 0.0))
        req.blend_type = int(command.get('blend_type', 0))
        req.sync_type = int(command.get('sync_type', 0))  # SYNC

        future = self.client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Move Circle 성공')
                self.command_ref.update({'status': 'success'})
            else:
                self.get_logger().error('Move Circle 실패')
                self.command_ref.update({'status': 'failed'})
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.command_ref.update({'status': f'error: {e}'})


def main(args=None):
    rclpy.init(args=args)
    node = MoveCircleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
