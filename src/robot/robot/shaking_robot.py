#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
쉐이킹 공정 전담 노드
- capping_robot 노드로부터 캡핑 완료 신호 수신 후 쉐이킹 실행
- 쉐이킹 완료 후 capping_robot 노드에 완료 신호 전송
- 총 2사이클 실행: 캡핑1 → 쉐이킹1 → 캡핑2 → 쉐이킹2 → 완료
"""

import time
import rclpy
from rclpy.node import Node
import DR_init
from std_msgs.msg import String, Int32

# ========================================
# 로봇 설정 및 전역 변수
# ========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v2"

# 속도 설정
VELX = [80, 80]
ACCX = [100, 100]
VELJ = 60
ACCJ = 60
VEL_SHAKE = 60
ACC_SHAKE = 60

# 그리퍼 및 핀
ON, OFF = 1, 0
SAFE_Z_OFFSET = 100

# 쉐이킹 반복 횟수
ITERATION = 5
# 각 관절별 흔들림 진폭
AMP_J3 = 10
AMP_J4 = 15
AMP_J5 = 20
AMP_J6 = 20

# 로봇 관절 한계치 (안전 여유 2도 제외)
joint_limits = [
    (-360 + 2, 360 - 2),  # J1
    (-95 + 2, 95 - 2),    # J2
    (-135 + 2, 135 - 2),  # J3
    (-360 + 2, 360 - 2),  # J4
    (-135 + 2, 135 - 2),  # J5
    (-360 + 2, 360 - 2)   # J6
]

# ========================================
# 좌표 데이터
# ========================================
POS_PICK = [
    [322.7, 8.10, 87.41, 19.83, -179.47, 19.28],
    [321.10, 8.29, 80.98, 178.85, 179.24, 178.52]
]
POS_PLACE = [
    [266.1, -386.71, 200.94, 92.46, 162.31, 92.86],
    [392.69, -381.66, 186.38, 91.56, 162.08, 91.85]
]
POS_AIR = [305.31, -343.97, 390.04, 114.58, -179.02, 115.44]
J_READY = [0, 0, 90, 0, 90, 0]
J_SHAKE_START = [2.98, 17, 63.38, -24.65, 52.10, -1.4]


class ShakingRobotNode(Node):
    def __init__(self):
        super().__init__("shaking_robot_node", namespace=ROBOT_ID)

        # 퍼블리셔
        self.status_pub = self.create_publisher(String, "status", 10)
        self.process_pub = self.create_publisher(Int32, "process", 10)
        self.shaking_done_pub = self.create_publisher(Int32, "shaking_done", 10)

        # 서브스크라이버 - 캡핑 완료 신호 수신
        self.capping_done_sub = self.create_subscription(
            Int32, "capping_done", self.capping_done_callback, 10
        )

        # 상태 변수
        self.current_cycle = -1  # 아직 시작 안함
        self.total_cycles = 2
        self.running = False

        self.get_logger().info("쉐이킹 로봇 노드 초기화 완료 - 캡핑 완료 신호 대기 중")

    def log(self, status_text, progress_val):
        """상태와 진행률 동시 발행"""
        s_msg = String()
        s_msg.data = status_text
        self.status_pub.publish(s_msg)

        p_msg = Int32()
        p_msg.data = int(progress_val)
        self.process_pub.publish(p_msg)
        self.get_logger().info(f"[STATUS] {status_text} | [PROGRESS] {progress_val}%")

    @staticmethod
    def grip():
        from DSR_ROBOT2 import set_digital_output, wait
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.8)

    @staticmethod
    def release():
        from DSR_ROBOT2 import set_digital_output, wait
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.8)

    def capping_done_callback(self, msg):
        """캡핑 완료 신호 수신"""
        cycle = msg.data
        self.get_logger().info(f"캡핑 완료 신호 수신 (사이클: {cycle})")

        if not self.running:
            self.current_cycle = cycle
            self.running = True
            self.run_shaking_cycle()

    def shaking_process(self, idx):
        """쉐이킹 공정 세부 로직"""
        from DSR_ROBOT2 import movel, movej, get_current_posj, posx, DR_MV_MOD_REL, wait

        base_progress = idx * 50
        self.log(f"쉐이킹 공정을 시작합니다.. (사이클 : {idx+1} 회)", base_progress + 30)

        # 캡핑 완료 후 바로 시작하므로 그리퍼가 병을 잡고 있음
        wait(6)
        movel(posx([0, 0, 100, 0, 0, 0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        # j6 초기화 (캡핑하면서 돌아간 줄 풀기)
        curr_j = get_current_posj()
        self.get_logger().info(f"현재 관절 위치: {curr_j}")
        curr_j[5] = -180.0
        movej(curr_j, vel=VELJ, acc=ACCJ)
        self.get_logger().info("돌아간 줄 뽑기 끝")

        # Shaking 동작
        def get_safe_joint(base_j, offsets):
            """현재 각도에 오프셋을 더한 뒤 한계치 넘지 않도록 보정"""
            safe_target = []
            for i in range(6):
                target_val = base_j[i] + offsets[i]
                min_limit, max_limit = joint_limits[i]
                if target_val < min_limit:
                    target_val = min_limit
                if target_val > max_limit:
                    target_val = max_limit
                safe_target.append(target_val)
            return safe_target

        start_j = get_current_posj()
        movej(J_SHAKE_START, vel=VELJ, acc=ACCJ)
        self.get_logger().info("쉐이킹 시작 위치 도달")

        for i in range(ITERATION):
            self.get_logger().info(f"{i+1}번째 쉐이킹 동작 실행 중...")

            # 위로 이동 오프셋
            offsets_up = [0, 0, -AMP_J3, -AMP_J4, -AMP_J5, -AMP_J6]
            target_up = get_safe_joint(start_j, offsets_up)

            # 아래로 이동 오프셋
            offsets_down = [0, 0, AMP_J3, AMP_J4, AMP_J5, AMP_J6]
            target_down = get_safe_joint(start_j, offsets_down)

            # 마지막 루프 여부에 따른 블렌딩 반경 설정
            is_last = (i == ITERATION - 1)
            r_val = 0 if is_last else 15

            movej(target_up, vel=VEL_SHAKE, acc=ACC_SHAKE, radius=r_val)
            movej(target_down, vel=VEL_SHAKE, acc=ACC_SHAKE, radius=r_val)

        self.get_logger().info("쉐이킹 동작 완료")

        # Place
        self.get_logger().info("place 시작")
        place_pos = POS_PLACE[idx]
        movel(POS_AIR, vel=VELX, acc=ACCX)
        movel(posx(place_pos), vel=VELX, acc=ACCX)
        self.release()
        movel(posx([0, 0, SAFE_Z_OFFSET, 0, 0, 0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        self.log(f"쉐이킹 동작 완료 (사이클 : {idx+1} 회)", base_progress + 50)

    def run_shaking_cycle(self):
        """현재 사이클의 쉐이킹 공정 실행"""
        idx = self.current_cycle
        self.get_logger().info(f"쉐이킹 사이클 {idx+1} 시작")

        self.shaking_process(idx)

        # 쉐이킹 완료 신호 전송
        done_msg = Int32()
        done_msg.data = idx
        self.shaking_done_pub.publish(done_msg)
        self.get_logger().info(f"쉐이킹 완료 신호 전송 (사이클: {idx})")

        # 다음 캡핑 대기 상태로 전환
        self.running = False

    def run(self):
        """메인 실행 함수 - 캡핑 완료 신호 대기"""
        self.get_logger().info("쉐이킹 노드 대기 중...")

        # 이벤트 루프 - 캡핑 완료 신호 대기
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = ShakingRobotNode()
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import release_force, release_compliance_ctrl

    release_force(time=0.0)
    release_compliance_ctrl()

    try:
        time.sleep(1.0)
        node.run()
    except KeyboardInterrupt:
        print("사용자 중단")
    except Exception as e:
        print(f"에러 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
