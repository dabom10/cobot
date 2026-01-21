import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import ServoOff

from web_ui.firebase_config import init_firebase, get_reference


class ServoOffNode(Node):
    def __init__(self):
        super().__init__('servo_off_node')
        self.get_logger().info("Servo Off Node 시작")

        init_firebase(self.get_logger())

        # Firebase command reference
        self.command_ref = get_reference('/robot_command/servo_off')

        self.client = self.create_client(
            ServoOff,
            '/dsr01/system/servo_off'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            pass

        self.get_logger().info('servo_off service ready')

        # Firebase 리스너 설정
        self.command_ref.listen(self.on_command)

    def on_command(self, event):
        """Firebase에서 command 변경 감지"""
        if event.data is None:
            return

        command = event.data
        if command.get('execute') == True:
            self.get_logger().info('Servo Off 명령 수신')
            self.execute_servo_off()
            # 실행 후 execute 플래그 초기화
            self.command_ref.update({'execute': False})

    def execute_servo_off(self):
        """ServoOff 서비스 호출"""
        req = ServoOff.Request()

        future = self.client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Servo Off 성공')
                self.command_ref.update({'status': 'success'})
            else:
                self.get_logger().error(f'Servo Off 실패: {response.message}')
                self.command_ref.update({'status': 'failed'})
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.command_ref.update({'status': f'error: {e}'})


def main(args=None):
    rclpy.init(args=args)
    node = ServoOffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
