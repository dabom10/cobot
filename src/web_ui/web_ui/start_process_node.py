import rclpy
from rclpy.node import Node
from web_ui.firebase_config import init_firebase, get_reference
import subprocess
import threading


class StartProcessNode(Node):
    def __init__(self):
        super().__init__('start_process_node')
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 공정시작 노드가 가동되었습니다.")

        # 1. 파이어베이스 초기화
        init_firebase(self.get_logger())
        self.command_ref = get_reference('robot_command/start_process')
        self.get_logger().info("✅ Firebase 레퍼런스: robot_command/start_process")

        # 2. 상태 관리 변수
        self.last_execute_state = False
        self.check_count = 0
        self.process = None

        # 3. Firebase 폴링용 타이머 (0.5초마다 체크)
        self.firebase_check_timer = self.create_timer(0.5, self.check_firebase)

        # 초기 상태 설정
        try:
            self.command_ref.update({'execute': False, 'status': 'ready'})
            self.get_logger().info("✅ Firebase 초기 상태 설정 완료")
        except Exception as e:
            self.get_logger().error(f"❌ Firebase 초기 상태 설정 실패: {e}")

        self.get_logger().info("✅ 공정시작 명령을 대기합니다.")
        self.get_logger().info("=" * 60)

    def check_firebase(self):
        """주기적으로 Firebase의 값을 체크"""
        self.check_count += 1

        try:
            data = self.command_ref.get()

            # 매 10번째마다 상태 출력
            if self.check_count % 10 == 0:
                self.get_logger().info(f"[체크 #{self.check_count}] Firebase 데이터: {data}")

            if data is None:
                return

            execute = data.get('execute', False)

            # execute가 False → True로 변경되었을 때만 반응
            if execute and not self.last_execute_state:
                self.get_logger().info("=" * 60)
                self.get_logger().info('🔔🔔🔔 공정시작 명령 감지!')
                self.get_logger().info("=" * 60)

                # 상태 업데이트
                self.command_ref.update({'execute': False, 'status': 'running'})

                # ros2 run robot robot 실행
                self.run_process()

            self.last_execute_state = execute

        except Exception as e:
            self.get_logger().error(f'❌ Firebase 체크 오류: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def run_process(self):
        """ros2 run robot reboot_a_v2 명령어 실행"""
        def execute_command():
            try:
                self.get_logger().info("🚀 ros2 run robot reboot_a_v2 실행 중...")

                # ROS2 환경을 포함하여 bash로 실행
                cmd = "source /opt/ros/humble/setup.bash && source ~/cobot/install/setup.bash && ros2 run robot reboot_a_v2"
                self.get_logger().info(f"📌 실행 명령어: {cmd}")

                # subprocess로 명령어 실행 (shell=True로 bash 환경에서 실행)
                self.process = subprocess.Popen(
                    cmd,
                    shell=True,
                    executable='/bin/bash',
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )

                self.get_logger().info(f"📌 프로세스 시작됨 (PID: {self.process.pid})")

                # 프로세스 완료 대기
                stdout, stderr = self.process.communicate()

                self.get_logger().info(f"📌 프로세스 종료 (returncode: {self.process.returncode})")
                if stdout:
                    self.get_logger().info(f"📌 stdout: {stdout[:500]}")
                if stderr:
                    self.get_logger().warn(f"📌 stderr: {stderr[:500]}")

                if self.process.returncode == 0:
                    self.get_logger().info("✅ 공정 완료!")
                    self.command_ref.update({'status': 'success'})
                else:
                    self.get_logger().error(f"❌ 공정 실패 (returncode: {self.process.returncode})")
                    self.command_ref.update({'status': 'failed'})

            except Exception as e:
                self.get_logger().error(f"❌ 명령어 실행 오류: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                self.command_ref.update({'status': 'failed'})

        # 별도 스레드에서 실행 (노드 블로킹 방지)
        self.get_logger().info("📌 스레드 시작...")
        thread = threading.Thread(target=execute_command)
        thread.start()
        self.get_logger().info("📌 스레드 시작 완료")


def main(args=None):
    rclpy.init(args=args)
    node = StartProcessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C 감지")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
