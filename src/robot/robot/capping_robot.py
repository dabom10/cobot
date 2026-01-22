#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
캡핑 공정 전담 노드
- 캡핑 공정 완료 후 shaking_robot 노드에 신호 전송
- shaking_robot 노드로부터 완료 신호 수신 후 다음 사이클 진행
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
VELX_FAST = [100, 100]
ACCX_FAST = [120, 120]
VELX_SLOW = [30, 30]
ACCX_SLOW = [60, 60]
VELJ = 60
ACCJ = 60

# 그리퍼 및 핀
ON, OFF = 1, 0
SAFE_Z_OFFSET = 100

# ========================================
# 좌표 데이터
# ========================================
BOTTLE_POSITIONS = [
    [436.33, 247.01, 58.5, 29.08, 180, 29.02],
    [208.26, 245.89, 55.62, 168.87, 180, 167.49]
]
BOTTLE_TARGETS = [
    [325.7, 11.10, 105.41, 19.83, 180, 19.28],
    [325.7, 7.10, 105.53, 19.83, 180, 19.28]
]
BOTTLE_TARGETS_GOS = [
    [319.7, 6.10, 119.41, 19.83, 180, 19.28],
    [318.7, 5.10, 99.53, 19.83, 180, 19.28]
]
CAP_POSITIONS = [
    [569.72, 238.92, 82.52, 130.85, 180, 130.39],
    [569.72, 238.92, 59.52, 130.85, 180, 130.39]
]

J_READY = [0, 0, 90, 0, 90, 0]
POS_AIR = [305.31, -343.97, 390.04, 114.58, -179.02, 115.44]
POS_HOME_BEFORE = [317.34, -307.11, 344.5, 125.41, -170.49, 127.82]

# 캡핑 회전 상수
J6_START, J6_END = -270.0, 90.0
TOTAL_DOWN = 5.0


class CappingRobotNode(Node):
    def __init__(self):
        super().__init__("capping_robot_node", namespace=ROBOT_ID)

        # 퍼블리셔
        self.status_pub = self.create_publisher(String, "status", 10)
        self.process_pub = self.create_publisher(Int32, "process", 10)
        self.capping_done_pub = self.create_publisher(Int32, "capping_done", 10)

        # 서브스크라이버 - 쉐이킹 완료 신호 수신
        self.shaking_done_sub = self.create_subscription(
            Int32, "shaking_done", self.shaking_done_callback, 10
        )

        # 상태 변수
        self.current_cycle = 0
        self.total_cycles = 2
        self.waiting_for_shaking = False
        self.all_done = False

        self.get_logger().info("캡핑 로봇 노드 초기화 완료")

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

    def initialize(self):
        from DSR_ROBOT2 import set_tool, set_tcp, movej, set_robot_mode, ROBOT_MODE_AUTONOMOUS
        try:
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        except:
            pass
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        movej(J_READY, vel=VELJ, acc=ACCJ)
        self.log("캡핑 시스템 준비 완료", 0)

    def shaking_done_callback(self, msg):
        """쉐이킹 완료 신호 수신"""
        cycle = msg.data
        self.get_logger().info(f"쉐이킹 완료 신호 수신 (사이클: {cycle})")

        if self.waiting_for_shaking and cycle == self.current_cycle:
            self.waiting_for_shaking = False
            self.current_cycle += 1

            if self.current_cycle < self.total_cycles:
                # 다음 캡핑 사이클 실행
                self.run_capping_cycle()
            else:
                # 모든 사이클 완료
                self.finish_all()

    def capping_process(self, idx):
        """캡핑 공정 세부 로직"""
        from DSR_ROBOT2 import (posx, movel, movej, wait, DR_MV_MOD_REL, get_current_posj,
                                 task_compliance_ctrl, set_stiffnessx, set_desired_force,
                                 check_force_condition, DR_AXIS_Z, release_force, get_tool_force,
                                 release_compliance_ctrl, DR_FC_MOD_REL, posj, get_current_posx,
                                 is_done_bolt_tightening)

        base_progress = idx * 50
        self.log(f"캡핑 공정을 시작합니다.. (사이클 : {idx+1} 회)", base_progress + 5)

        # 1. 병 이동
        self.release()
        bottle_pos = BOTTLE_POSITIONS[idx]
        target_pos = BOTTLE_TARGETS[idx]

        movel(posx([bottle_pos[0], bottle_pos[1], bottle_pos[2]+70, bottle_pos[3], bottle_pos[4], bottle_pos[5]]), vel=VELX_FAST, acc=ACCX_FAST)
        movel(posx(bottle_pos), vel=VELX_SLOW, acc=ACCX_SLOW)
        self.grip()
        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        movel(posx([target_pos[0], target_pos[1], target_pos[2] + 70,
                    target_pos[3], target_pos[4], target_pos[5]]), vel=VELX, acc=ACCX)

        movel(posx([target_pos[0], target_pos[1], target_pos[2] - 20,
                    target_pos[3], target_pos[4], target_pos[5]]), vel=VELX, acc=ACCX)
        self.release()
        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        # 2. 뚜껑 조립
        cap_p = CAP_POSITIONS[idx]
        target_go = BOTTLE_TARGETS_GOS[idx]

        # J6 초기화
        curr_j = get_current_posj()
        curr_j[5] = J6_START
        movej(curr_j, vel=VELJ, acc=ACCJ)

        movel(posx([cap_p[0], cap_p[1], cap_p[2]+80, cap_p[3], cap_p[4], cap_p[5]]), vel=VELX, acc=ACCX)
        movel(posx(cap_p), vel=VELX_SLOW, acc=ACCX_SLOW)
        self.grip()
        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        movel(posx([target_go[0], target_go[1], target_go[2]+30, target_go[3], target_go[4], target_go[5]]), vel=VELX, acc=ACCX)
        movel(posx([target_go[0], target_go[1], target_go[2]-(idx*10), target_go[3], target_go[4], target_go[5]]), vel=[20,20], acc=ACCX_SLOW)

        # 힘 제어 누르기
        for i in range(2):
            force_list = [-60, -50]
            f_list = [65, 60]   # [55, 50]
            self.get_logger().info(f"뚜껑 누르기 : {i}")
            if i == 1:
                rot = posj(0,0,0,0,0,90)
                movej(rot, vel=60, acc=60, mod=DR_MV_MOD_REL)

            if i == 0:
                self.release()
                wait(0.3)
                movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            self.grip()

            task_compliance_ctrl()
            set_stiffnessx([100, 100, 50, 100, 100, 100])
            self.get_logger().info("순응 제어 설정 완료")

            down_grip_control = len(BOTTLE_TARGETS_GOS)
            little_down = posx([0,0,down_grip_control*15-75,0,0,0])
            movel(little_down, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            self.get_logger().info("힘 제어 시작")
            set_desired_force([0, 0, force_list[i], 0, 0, 0], [0, 0, 5, 0, 0, 0], mod=DR_FC_MOD_REL)

            while True:
                obj_ok = check_force_condition(DR_AXIS_Z, min=f_list[i], max=120)
                if not obj_ok:
                    self.get_logger().info("뚜껑 누르기 감지")
                    break
                continue

            if not obj_ok:
                release_force(time=0.0)
                self.get_logger().info("힘제어 설정 off")
                movel(posx([0,0,70,0,0,0]), vel=60, acc=60, mod=DR_MV_MOD_REL)
                release_compliance_ctrl()

        # 회전 조이기
        self.get_logger().info("회전 조이기 시작")
        self.release()

        curr_j = get_current_posj()
        curr_j[5] = J6_START
        movej(curr_j, vel=VELJ, acc=ACCJ)
        self.get_logger().info(f"J6 초기화 완료: {J6_START}도")

        down = posx([0, 0, -94, 0, 0, 0])
        movel(down, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        self.get_logger().info(f"누른 후 다운 : {get_current_posx()}")
        start_j = get_current_posj()
        start_j6 = start_j[5]
        self.get_logger().info("while 시작")
        self.grip()

        # 슬립 감지 변수
        slip_count = 0
        SLIP_THRESHOLD = 2
        MX_SLIP_VALUE = 0.7
        spin_count = 0

        while True:
            current_force = get_tool_force()
            mx = current_force[3]
            spin_count += 1

            self.get_logger().info(f"[Force] Fx={current_force[0]:.2f}, Fy={current_force[1]:.2f}, Fz={current_force[2]:.2f} (N)")
            self.get_logger().info(f"[Torque] Mx={mx:.2f}, My={current_force[4]:.2f}, Mz={current_force[5]:.2f} (Nm)")

            if is_done_bolt_tightening(m=0.7, timeout=5, axis=DR_AXIS_Z):
                slip_count += 1
                self.get_logger().info(f"[볼트체결 감지] ({slip_count}/{SLIP_THRESHOLD})")
                if slip_count >= SLIP_THRESHOLD and spin_count > 10:
                    self.get_logger().info("뚜껑 조이기 완료 (볼트체결 감지)")
                    break
            else:
                slip_count = 0

            self.get_logger().info("while 진입")
            current_j = get_current_posj()
            target_j6 = start_j6 + 360/20
            start_j6 = target_j6
            if target_j6 > 90:
                target_j6 = 90
            elif target_j6 < -270:
                target_j6 = -270

            target_j = posj([
                current_j[0], current_j[1], current_j[2],
                current_j[3], current_j[4], target_j6
            ])

            movej(target_j, vel=50, acc=80)
            movel(posx([0,0,-(TOTAL_DOWN/20),0,0,0]), vel=[10,30], acc=[30,50], mod=DR_MV_MOD_REL)

            if start_j6 >= 90:
                self.get_logger().info("뚜껑 조이기 회전 제한 도달")
                break

        self.log(f"캡핑 공정 완료 (사이클 : {idx+1} 회)", base_progress + 25)

    def run_capping_cycle(self):
        """현재 사이클의 캡핑 공정 실행"""
        idx = self.current_cycle
        self.get_logger().info(f"캡핑 사이클 {idx+1} 시작")

        self.capping_process(idx)

        # 캡핑 완료 신호 전송
        done_msg = Int32()
        done_msg.data = idx
        self.capping_done_pub.publish(done_msg)
        self.get_logger().info(f"캡핑 완료 신호 전송 (사이클: {idx})")

        # 쉐이킹 완료 대기 상태로 전환
        self.waiting_for_shaking = True

    def finish_all(self):
        """모든 공정 완료 후 홈 위치로 복귀"""
        from DSR_ROBOT2 import movel, movej, posx

        self.get_logger().info("모든 사이클 완료, 홈 위치로 복귀")
        movej(POS_AIR, vel=VELJ, acc=ACCJ)
        movel(posx(POS_HOME_BEFORE), vel=VELX, acc=ACCX)
        movej(J_READY, vel=VELJ, acc=ACCJ)
        self.log("all_process_completed", 100)
        self.all_done = True

    def run(self):
        """메인 실행 함수"""
        self.initialize()
        self.run_capping_cycle()

        # 이벤트 루프 - 쉐이킹 완료 신호 대기
        while rclpy.ok() and not self.all_done:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = CappingRobotNode()
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import release_force, get_tcp, release_compliance_ctrl
    print(f"엔드이펙터 - Gripper : {get_tcp()}")

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
