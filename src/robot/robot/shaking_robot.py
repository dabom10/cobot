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
VELX = [150, 150]
ACCX = [150, 150]
VELJ = 60
ACCJ = 60
VEL_SHAKE = 150
ACC_SHAKE = 150

# 그리퍼 및 핀
ON, OFF = 1, 0
SAFE_Z_OFFSET = 100

# ========================================
# 좌표 데이터
# ========================================
POS_PICK = [
    [322.7, 8.10, 87.41, 19.83, -179.47, 19.28],
    [321.10, 8.29, 80.98, 178.85, 179.24, 178.52]
]
POS_PLACE = [
    [266.1, -386.71, 210.94, 92.46, 162.31, 92.86], # z 10 up
    [392.69, -381.66, 196.38, 91.56, 162.08, 91.85] # z 10 up
]
POS_AIR = [328, -215, 456, 176, -176, 151]
J_READY = [0, 0, 90, 0, 90, 0]
J_MIX_1 = [0, 10, 80,  45,  45, 90]
J_MIX_2 = [0, 10, 80, -45, -45, -90]
J_SHAKE_START = [2.98, 17, 63.38, -24.65, 52.10, -180.0]


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
        self.target_cycle = -1
        self.total_cycles = 2
        self.trigger_shaking = False

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
        
        self.target_cycle = cycle
        self.trigger_shaking = True

    def shaking_process(self, idx):
        """쉐이킹 공정 세부 로직"""
        from DSR_ROBOT2 import (movel, movej, get_current_posj, DR_BASE, DR_TOOL, posx,
                                 DR_MV_MOD_REL, wait, move_periodic, set_tcp,
                                 release_compliance_ctrl, release_force, set_robot_mode,
                                 ROBOT_MODE_AUTONOMOUS)

        self.get_logger().info("로봇 상태 초기화 중...")
        
        # 노드 간 제어권 전환 안정화를 위한 짧은 대기
        wait(1.0)
        
        try:
            # 안전을 위해 이전 공정의 힘 제어 및 설정 초기화
            for i in range(3):
                try:
                    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
                    break
                except:
                    wait(0.5)
            
            self.get_logger().info("힘 제어 및 TCP 초기화 실행")
            release_force(time=0.0)
            release_compliance_ctrl()
            set_tcp(ROBOT_TCP)
        except Exception as e:
            self.get_logger().error(f"로봇 초기화 실패: {e}")
            return

        base_progress = idx * 50
        self.log(f"쉐이킹 공정을 시작합니다.. (사이클 : {idx+1} 회)", base_progress + 30)
        
        self.get_logger().info("6초 대기 중...")
        wait(6)  # 사용자 요청 유지

        self.get_logger().info("병 들어올리기 시작")
        movel(posx([0, 0, 100, 0, 0, 0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        # j6 초기화 (캡핑하면서 돌아간 줄 풀기)
        curr_j = get_current_posj()
        if curr_j is None: return
        self.get_logger().info(f"현재 관절 위치: {curr_j}")
        curr_j[5] = -180.0
        self.get_logger().info("J6 관절 꼬임 해제 중...")
        movej(curr_j, vel=VELJ, acc=ACCJ)
        self.get_logger().info("돌아간 줄 뽑기 끝")

        # 2. Shaking 동작 실행
        self.get_logger().info("쉐이킹(Periodic Move) 시작")
        for _ in range(2):
            movej(J_MIX_1, vel=VEL_SHAKE, acc=ACC_SHAKE, radius=20)
            movej(J_MIX_2, vel=VEL_SHAKE, acc=ACC_SHAKE, radius=20)
            # movej(J_READY, vel=VEL_SHAKE, acc=ACC_SHAKE, radius=20)
            movej([0,0,90,0,82,0], vel=VEL_SHAKE, acc=ACC_SHAKE, radius=20)
            self.get_logger().info("주기적 흔들림 동작 중...")
            move_periodic(amp=[0,0,40,0,0,120], period=0.4, repeat=1, ref=DR_BASE)
            move_periodic(amp=[0,0,0,40,40,0], period=0.35, repeat=1, ref=DR_TOOL)

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
        idx = self.target_cycle
        self.get_logger().info(f"쉐이킹 사이클 {idx+1} 시작")

        self.shaking_process(idx)

        # 쉐이킹 완료 신호 전송
        done_msg = Int32()
        done_msg.data = idx
        self.shaking_done_pub.publish(done_msg)
        self.get_logger().info(f"쉐이킹 완료 신호 전송 (사이클: {idx})")

    def run(self):
        """메인 실행 함수 - 캡핑 완료 신호 대기"""
        self.get_logger().info("쉐이킹 노드 실행 루프 시작 - 신호 대기 중")
        
        # 이벤트 루프 - 캡핑 완료 신호 대기 및 쉐이킹 실행
        while rclpy.ok():
            if self.trigger_shaking:
                self.trigger_shaking = False
                self.run_shaking_cycle()
            
            # 신호 대기 중 메시지 처리를 위해 spin_once 호출
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
