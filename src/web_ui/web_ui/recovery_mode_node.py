import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from web_ui.firebase_config import init_firebase, get_reference

class FirebasePeriodicPublisher(Node):
    def __init__(self):
        super().__init__('firebase_periodic_publisher')
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 파이어베이스 브릿지 노드가 가동되었습니다.")

        # 1. 파이어베이스 초기화
        init_firebase(self.get_logger())
        self.command_ref = get_reference('dsr01/user_cmd')
        self.get_logger().info("✅ Firebase 레퍼런스: dsr01/user_cmd")

        # 2. 퍼블리셔 생성 (topic list에 바로 표시됨)
        self.publisher = self.create_publisher(Int32, '/dsr01/user_cmd', 10)
        self.get_logger().info("✅ Publisher 생성: /dsr01/user_cmd")
        
        # 3. 상태 관리 변수
        self.timer = None
        self.is_publishing = False
        self.last_execute_state = False
        self.check_count = 0

        # 4. Firebase 폴링용 타이머 (0.5초마다 체크)
        self.firebase_check_timer = self.create_timer(0.5, self.check_firebase)
        
        # 초기 상태 설정
        try:
            self.command_ref.update({'execute': False, 'status': 'ready'})
            self.get_logger().info("✅ Firebase 초기 상태 설정 완료")
        except Exception as e:
            self.get_logger().error(f"❌ Firebase 초기 상태 설정 실패: {e}")
        
        self.get_logger().info("✅ /dsr01/user_cmd 토픽이 등록되었습니다. 명령을 대기합니다.")
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
                if self.check_count % 10 == 0:
                    self.get_logger().warn(f"[체크 #{self.check_count}] ⚠️ Firebase 데이터가 None")
                return
            
            execute = data.get('execute', False)
            
            # execute 상태 변화 감지
            if execute != self.last_execute_state:
                self.get_logger().info(f"🔄 execute 상태 변경: {self.last_execute_state} → {execute}")
            
            # execute가 False → True로 변경되었을 때만 반응
            if execute and not self.last_execute_state:
                self.get_logger().info("=" * 60)
                self.get_logger().info('🔔🔔🔔 버튼 클릭 감지! 주기적 발행을 시작합니다.')
                self.get_logger().info("=" * 60)
                self.start_timer()
                
                # 상태 업데이트
                self.command_ref.update({'execute': False, 'status': '발행 중'})
                self.get_logger().info("✅ Firebase 상태 업데이트 완료")
            
            self.last_execute_state = execute
            
        except Exception as e:
            self.get_logger().error(f'❌ Firebase 체크 오류: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def start_timer(self):
        """주기적 발행 시작"""
        if self.timer is None:
            self.get_logger().info("🚀 타이머 시작: 0.1초마다 Int32(0) 발행")
            # 0.1초(10Hz)마다 0을 퍼블리시
            self.timer = self.create_timer(0.1, self.timer_callback)
            self.is_publishing = True
            self.publish_count = 0
        else:
            self.get_logger().warn("⚠️ 타이머가 이미 실행 중입니다")

    def timer_callback(self):
        """실제 메시지 발행"""
        msg = Int32()
        msg.data = 0
        self.publisher.publish(msg)
        
        self.publish_count += 1
        # 처음 5번과 이후 50번마다 로그
        if self.publish_count <= 5 or self.publish_count % 50 == 0:
            self.get_logger().info(f'📤 Published #{self.publish_count}: data={msg.data}')

    def stop_timer(self):
        """발행 중지 (필요시 사용)"""
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            self.is_publishing = False
            self.command_ref.update({'status': 'stopped'})
            self.get_logger().info("🛑 타이머 중지됨")

def main(args=None):
    rclpy.init(args=args)
    node = FirebasePeriodicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C 감지")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()