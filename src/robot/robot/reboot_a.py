#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
상태 머신 기반 로봇 제어 스크립트
- 0신호: 멈춤 → 놓기 → 홈 → 1신호 대기 → 다음 단계로
- 단계 1~3에서 0: 에러 위치(4번)로
- 단계 4에서 0: 두번째 병(5번)으로
- 단계 5~7에서 0: 종료
"""

import time
import threading
import rclpy
import DR_init
from rclpy.node import Node
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
VELX_FAST = [150, 150]
ACCX_FAST = [120, 120]
VELX_SLOW = [30, 30]
ACCX_SLOW = [60, 60]
VELJ = 60
ACCJ = 60
VEL_SHAKE = 300
ACC_SHAKE = 350

# 그리퍼 및 핀
ON, OFF = 1, 0
SAFE_Z_OFFSET = 100

# ========================================
# 좌표 데이터
# ========================================
BOTTLE_POSITIONS = [
    [436.33, 247.01, 58.5, 29.08, 180, 29.02],      # 첫번째 병
    [208.26, 245.89, 55.62, 168.87, 180, 167.49]    # 두번째 병
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
    [569.72, 238.92, 82.52, 130.85, 180, 130.39],   # 첫번째 뚜껑
    [569.72, 238.92, 59.52, 130.85, 180, 130.39]    # 두번째 뚜껑
]

# 에러 위치 (4번 단계에서 집을 위치)
ERROR_POSITION = [318.21, 247.01, 58.5, 29.08, 180, 29.02]
POS_HOME_BEFORE = [317.34, -307.11, 344.5, 125.41, -170.49, 127.82]

POS_PLACE = [
    [266.1, -386.71, 210.94, 92.46, 162.31, 92.86],   # z 10 up
    [392.69, -381.66, 196.38, 91.56, 162.08, 91.85]   # z 10 up
]
POS_AIR = [328, -215, 456, 176, -176, 151]
J_READY = [0, 0, 90, 0, 90, 0]

J6_START = -270.0
TOTAL_DOWN = 5.0


class IntegratedSystem:
    def __init__(self, node):
        self.node = node
        self.status_pub = node.create_publisher(String, "status", 10)
        self.process_pub = node.create_publisher(Int32, "process", 10)
        self.sub = node.create_subscription(Int32, "/dsr01/user_cmd", self.listener_callback, 10)

        # 상태 제어용 변수
        self.user_cmd = 1
        self.stop_requested = False
        self.restart_requested = False
        self.is_running = True
        self._lock = threading.Lock()

        # ROS 콜백 처리용 스레드
        self.spin_thread = threading.Thread(target=self._spin_thread, daemon=True)
        self.spin_thread.start()

    def _spin_thread(self):
        while self.is_running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def listener_callback(self, msg):
        number = msg.data
        with self._lock:
            self.user_cmd = number

        print(f"[CALLBACK] user_cmd = {number}")

        if number == 0:
            with self._lock:
                self.stop_requested = True
                self.restart_requested = False
            print("[STOP] 멈춤 신호 수신!")

        elif number == 1:
            with self._lock:
                if self.stop_requested:
                    print("[RESTART] 재시작 신호 수신!")
                    self.restart_requested = True

    def check_stop(self):
        with self._lock:
            return self.stop_requested

    def wait_for_restart(self):
        """1신호 대기"""
        print("\n" + "=" * 50)
        print("[대기] 재시작 신호(user_cmd=1)를 기다리는 중...")
        print("=" * 50 + "\n")

        while self.is_running and rclpy.ok():
            with self._lock:
                if self.restart_requested:
                    self.stop_requested = False
                    self.restart_requested = False
                    print("[재시작] 다음 단계로 진행!")
                    return True
            time.sleep(0.1)
        return False

    def reset_flags(self):
        with self._lock:
            self.stop_requested = False
            self.restart_requested = False
            self.user_cmd = 1

    def log(self, status_text, progress_val):
        s_msg = String()
        s_msg.data = status_text
        self.status_pub.publish(s_msg)
        p_msg = Int32()
        p_msg.data = int(progress_val)
        self.process_pub.publish(p_msg)
        print(f"[STATUS] {status_text} | [PROGRESS] {progress_val}%")

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

    def safe_recovery(self):
        """안전 복구 - 그리퍼 해제 후 홈 위치로"""
        from DSR_ROBOT2 import movej, movel, posx, release_force, release_compliance_ctrl, DR_MV_MOD_REL

        print("[RECOVERY] 복구 시작...")

        try:
            release_force(time=0.0)
            release_compliance_ctrl()
        except:
            pass

        self.release()
        time.sleep(0.5)

        try:
            movel(posx([0, 0, 100, 0, 0, 0]), vel=VELX_SLOW, acc=ACCX_SLOW, mod=DR_MV_MOD_REL)
        except:
            pass

        try:
            movej(J_READY, vel=VELJ, acc=ACCJ)
        except:
            pass

        print("[RECOVERY] 홈 위치 복구 완료!")

    def initialize(self):
        from DSR_ROBOT2 import set_tool, set_tcp, movej, set_robot_mode, ROBOT_MODE_AUTONOMOUS
        try:
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        except:
            pass
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        movej(J_READY, vel=VELJ, acc=ACCJ)
        self.log("시스템 준비 완료", 0)

    # ========================================
    # 단계별 함수
    # ========================================

    def step_1_pick_bottle_1(self):
        """1단계: 첫번째 병 잡고 지정된 위치에 놓기"""
        from DSR_ROBOT2 import posx, movel, DR_MV_MOD_REL

        self.log("1단계: 첫번째 병 집기", 10)
        bottle = BOTTLE_POSITIONS[0]
        target = BOTTLE_TARGETS[0]

        self.release()
        if self.check_stop(): return False

        movel(posx([bottle[0], bottle[1], bottle[2]+70] + bottle[3:]), vel=VELX_FAST, acc=ACCX_FAST)
        if self.check_stop(): return False

        movel(posx(bottle), vel=VELX_SLOW, acc=ACCX_SLOW)
        self.grip()
        if self.check_stop(): return False

        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        if self.check_stop(): return False

        movel(posx([target[0], target[1], target[2]+70] + target[3:]), vel=VELX, acc=ACCX)
        if self.check_stop(): return False

        movel(posx([target[0], target[1], target[2]-20] + target[3:]), vel=VELX, acc=ACCX)
        self.release()
        if self.check_stop(): return False

        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        self.log("1단계 완료: 첫번째 병 배치", 15)
        return True

    def step_2_pick_cap_1(self):
        """2단계: 첫번째 뚜껑 잡고 병 위에 놓기"""
        from DSR_ROBOT2 import posx, movel, movej, get_current_posj, DR_MV_MOD_REL

        self.log("2단계: 첫번째 뚜껑 집기", 20)
        cap = CAP_POSITIONS[0]
        target_go = BOTTLE_TARGETS_GOS[0]

        if self.check_stop(): return False
        curr_j = get_current_posj()
        curr_j[5] = J6_START
        movej(curr_j, vel=VELJ, acc=ACCJ)

        if self.check_stop(): return False
        movel(posx([cap[0], cap[1], cap[2]+80] + cap[3:]), vel=VELX, acc=ACCX)

        if self.check_stop(): return False
        movel(posx(cap), vel=VELX_SLOW, acc=ACCX_SLOW)
        self.grip()

        if self.check_stop(): return False
        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        if self.check_stop(): return False
        movel(posx([target_go[0], target_go[1], target_go[2]+30] + target_go[3:]), vel=VELX, acc=ACCX)

        if self.check_stop(): return False
        movel(posx(target_go), vel=[20,20], acc=ACCX_SLOW)

        self.log("2단계 완료: 첫번째 뚜껑 배치", 25)
        return True

    def step_3_capping_shaking_1(self):
        """3단계: 첫번째 캡핑 + 쉐이킹"""
        from DSR_ROBOT2 import (posx, movel, movej, move_periodic, get_current_posj, wait, DR_MV_MOD_REL, DR_BASE,
                                 task_compliance_ctrl, set_stiffnessx, set_desired_force,
                                 check_force_condition, DR_AXIS_Z, release_force,
                                 release_compliance_ctrl, DR_FC_MOD_REL, posj,
                                 is_done_bolt_tightening, get_tool_force)

        self.log("3단계: 첫번째 캡핑", 30)
        target_go = BOTTLE_TARGETS_GOS[0]

        # 힘 제어 누르기
        for i in range(2):
            if self.check_stop(): return False

            force_list = [-80, -70]
            f_list = [30, 30]

            if i == 1:
                movej(posj(0,0,0,0,0,90), vel=60, acc=60, mod=DR_MV_MOD_REL)

            if i == 0:
                self.release()
                wait(0.3)
                if self.check_stop(): return False
                movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            self.grip()

            task_compliance_ctrl()
            set_stiffnessx([100, 100, 50, 100, 100, 100])
            movel(posx([0,0,-45,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            set_desired_force([0, 0, force_list[i], 0, 0, 0], [0, 0, 5, 0, 0, 0], mod=DR_FC_MOD_REL)

            print(f"[DEBUG 3단계] 뚜껑 누르기 {i+1}회차 - 목표 힘: {force_list[i]}N, 감지 기준: {f_list[i]}N")
            force_check_count = 0
            start_time = time.time()
            while True:
                if self.check_stop():
                    release_force(time=0.0)
                    release_compliance_ctrl()
                    return False

                # 디버깅: 현재 힘 출력
                current_force = get_tool_force()
                force_check_count += 1
                if force_check_count % 10 == 0:  # 10회마다 출력
                    print(f"[FORCE 3단계] Fx={current_force[0]:.2f}, Fy={current_force[1]:.2f}, Fz={current_force[2]:.2f} | Mx={current_force[3]:.2f}, My={current_force[4]:.2f}, Mz={current_force[5]:.2f}")

                if not check_force_condition(DR_AXIS_Z, min=f_list[i], max=150):
                    print(f"[FORCE 3단계] 뚜껑 누르기 감지! Fz={current_force[2]:.2f}N")
                    break

                if time.time() - start_time > 3:
                    print("[ERROR] 뚜껑 누르기 타임아웃!")
                    break

            release_force(time=0.0)
            release_compliance_ctrl()
            if self.check_stop(): return False
            movel(posx([0,0,70,0,0,0]), vel=60, acc=60, mod=DR_MV_MOD_REL)

        # 회전 조이기
        if self.check_stop(): return False
        self.release()
        curr_j = get_current_posj()
        curr_j[5] = J6_START
        movej(curr_j, vel=VELJ, acc=ACCJ)

        if self.check_stop(): return False
        movel(posx([0, 0, -94, 0, 0, 0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        start_j = get_current_posj()
        start_j6 = start_j[5]
        self.grip()

        slip_count = 0
        spin_count = 0
        print("[DEBUG 3단계] 회전 조이기 시작")

        while True:
            if self.check_stop(): return False

            current_force = get_tool_force()
            spin_count += 1
            print(f"[FORCE 3단계 회전] Fz={current_force[2]:.2f}N | Mx={current_force[3]:.2f}Nm | spin={spin_count}")

            if is_done_bolt_tightening(m=0.7, timeout=5, axis=DR_AXIS_Z):
                slip_count += 1
                print(f"[볼트체결 감지] ({slip_count}/2)")
                if slip_count >= 2 and spin_count > 12:
                    print("[DEBUG 3단계] 뚜껑 조이기 완료!")
                    break
            else:
                slip_count = 0

            current_j = get_current_posj()
            target_j6 = start_j6 + 360/20
            start_j6 = target_j6
            if target_j6 > 90: target_j6 = 90
            elif target_j6 < -270: target_j6 = -270

            if self.check_stop(): return False
            movej(posj([current_j[0], current_j[1], current_j[2], current_j[3], current_j[4], target_j6]), vel=50, acc=80)
            movel(posx([0,0,-(TOTAL_DOWN/20),0,0,0]), vel=[10,30], acc=[30,50], mod=DR_MV_MOD_REL)

            if start_j6 >= 90:
                print("[DEBUG 3단계] 회전 제한 도달")
                break

        # 쉐이킹
        self.log("3단계: 쉐이킹", 40)
        wait(6)
        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        # j6 초기화
        curr_j = get_current_posj()
        print(f"[DEBUG 3단계] 현재 조인트: {curr_j}")
        curr_j[5] = -180.0
        movej(curr_j, vel=VELJ, acc=ACCJ)  # 캡핑하면서 돌아간 줄 풀기
        print('[DEBUG 3단계] 돌아간 줄 뽑기 끝')

        # Shaking

        J_MIX_1 = [0, 10, 80, 45, 45, 90]
        J_MIX_2 = [0, 10, 80, -45, -45, -90]
        for _ in range(2):
            if self.check_stop(): return False
            movej(J_MIX_1, vel=VEL_SHAKE, acc=ACC_SHAKE)
            movej(J_MIX_2, vel=VEL_SHAKE, acc=ACC_SHAKE)
            movej([0,0,90,0,90,0], vel=VEL_SHAKE, acc=ACC_SHAKE)
            move_periodic(amp=[0,0,40,0,0,120], period=0.4, repeat=1, ref=DR_BASE)
            move_periodic(amp=[0,0,0,40,40,0], period=0.35, repeat=1, ref=DR_BASE)
        print("[DEBUG 3단계] 쉐이킹 동작 완료")

        # Place
        print('[DEBUG 3단계] place 시작')
        if self.check_stop(): return False
        movel(POS_AIR, vel=VELX, acc=ACCX)
        movel(posx(POS_PLACE[0]), vel=VELX, acc=ACCX)
        self.release()
        movel(posx([0,0,SAFE_Z_OFFSET,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        self.log("3단계 완료: 첫번째 캡핑+쉐이킹", 45)
        return True

    def step_4_pick_error(self):
        """4단계: 에러 위치로 가서 집기"""
        from DSR_ROBOT2 import posx, movel, DR_MV_MOD_REL

        self.log("4단계: 에러 위치 집기", 50)
        error_pos = ERROR_POSITION

        if self.check_stop(): return False
        movel(posx([error_pos[0], error_pos[1], error_pos[2]+70] + error_pos[3:]), vel=VELX_FAST, acc=ACCX_FAST)

        if self.check_stop(): return False
        movel(posx(error_pos), vel=VELX_SLOW, acc=ACCX_SLOW)
        self.grip()

        if self.check_stop(): return False
        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        # 에러 물체 처리 (놓기)
        self.release()

        self.log("4단계 완료: 에러 위치 처리", 55)
        return True

    def step_5_pick_bottle_2(self):
        """5단계: 두번째 병 잡고 지정된 위치에 놓기"""
        from DSR_ROBOT2 import posx, movel, DR_MV_MOD_REL

        self.log("5단계: 두번째 병 집기", 60)
        bottle = BOTTLE_POSITIONS[1]
        target = BOTTLE_TARGETS[1]

        self.release()
        if self.check_stop(): return False

        movel(posx([bottle[0], bottle[1], bottle[2]+70] + bottle[3:]), vel=VELX_FAST, acc=ACCX_FAST)
        if self.check_stop(): return False

        movel(posx(bottle), vel=VELX_SLOW, acc=ACCX_SLOW)
        self.grip()
        if self.check_stop(): return False

        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        if self.check_stop(): return False

        movel(posx([target[0], target[1], target[2]+70] + target[3:]), vel=VELX, acc=ACCX)
        if self.check_stop(): return False

        movel(posx([target[0], target[1], target[2]-20] + target[3:]), vel=VELX, acc=ACCX)
        self.release()
        if self.check_stop(): return False

        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        self.log("5단계 완료: 두번째 병 배치", 65)
        return True

    def step_6_pick_cap_2(self):
        """6단계: 두번째 뚜껑 잡고 병 위에 놓기"""
        from DSR_ROBOT2 import posx, movel, movej, get_current_posj, DR_MV_MOD_REL

        self.log("6단계: 두번째 뚜껑 집기", 70)
        cap = CAP_POSITIONS[1]
        target_go = BOTTLE_TARGETS_GOS[1]

        if self.check_stop(): return False
        curr_j = get_current_posj()
        curr_j[5] = J6_START
        movej(curr_j, vel=VELJ, acc=ACCJ)

        if self.check_stop(): return False
        movel(posx([cap[0], cap[1], cap[2]+80] + cap[3:]), vel=VELX, acc=ACCX)

        if self.check_stop(): return False
        movel(posx(cap), vel=VELX_SLOW, acc=ACCX_SLOW)
        self.grip()

        if self.check_stop(): return False
        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        if self.check_stop(): return False
        movel(posx([target_go[0], target_go[1], target_go[2]+30] + target_go[3:]), vel=VELX, acc=ACCX)

        if self.check_stop(): return False
        movel(posx([target_go[0], target_go[1], target_go[2]-10] + target_go[3:]), vel=[20,20], acc=ACCX_SLOW)

        self.log("6단계 완료: 두번째 뚜껑 배치", 75)
        return True

    def step_7_capping_shaking_2(self):
        """7단계: 두번째 캡핑 + 쉐이킹"""
        from DSR_ROBOT2 import (posx, movel, movej, get_current_posj, wait, DR_MV_MOD_REL,
                                 task_compliance_ctrl, set_stiffnessx, set_desired_force,
                                 check_force_condition, DR_AXIS_Z, release_force,move_periodic,DR_BASE,
                                 release_compliance_ctrl, DR_FC_MOD_REL, posj,
                                 is_done_bolt_tightening, get_tool_force)

        self.log("7단계: 두번째 캡핑", 80)

        # 힘 제어 누르기
        for i in range(2):
            if self.check_stop(): return False

            force_list = [-80, -70]
            f_list = [30, 30]

            if i == 1:
                movej(posj(0,0,0,0,0,90), vel=60, acc=60, mod=DR_MV_MOD_REL)

            if i == 0:
                self.release()
                wait(0.3)
                if self.check_stop(): return False
                movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            self.grip()

            task_compliance_ctrl()
            set_stiffnessx([100, 100, 50, 100, 100, 100])
            movel(posx([0,0,-45,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            set_desired_force([0, 0, force_list[i], 0, 0, 0], [0, 0, 5, 0, 0, 0], mod=DR_FC_MOD_REL)

            print(f"[DEBUG 7단계] 뚜껑 누르기 {i+1}회차 - 목표 힘: {force_list[i]}N, 감지 기준: {f_list[i]}N")
            force_check_count = 0
            start_time = time.time()
            while True:
                if self.check_stop():
                    release_force(time=0.0)
                    release_compliance_ctrl()
                    return False

                # 디버깅: 현재 힘 출력
                current_force = get_tool_force()
                force_check_count += 1
                if force_check_count % 10 == 0:  # 10회마다 출력
                    print(f"[FORCE 7단계] Fx={current_force[0]:.2f}, Fy={current_force[1]:.2f}, Fz={current_force[2]:.2f} | Mx={current_force[3]:.2f}, My={current_force[4]:.2f}, Mz={current_force[5]:.2f}")

                if not check_force_condition(DR_AXIS_Z, min=f_list[i], max=150):
                    print(f"[FORCE 7단계] 뚜껑 누르기 감지! Fz={current_force[2]:.2f}N")
                    break
                else:
                    pass
                print("진행 시간 :",time.time() - start_time)
                if time.time() - start_time > 3:
                    print("[ERROR] 뚜껑 누르기 타임아웃!")
                    break
                else:
                    pass

            release_force(time=0.0)
            release_compliance_ctrl()
            if self.check_stop(): return False
            movel(posx([0,0,70,0,0,0]), vel=60, acc=60, mod=DR_MV_MOD_REL)

        # 회전 조이기
        if self.check_stop(): return False
        self.release()
        curr_j = get_current_posj()
        curr_j[5] = J6_START
        movej(curr_j, vel=VELJ, acc=ACCJ)

        if self.check_stop(): return False
        movel(posx([0, 0, -94, 0, 0, 0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        start_j = get_current_posj()
        start_j6 = start_j[5]
        self.grip()

        slip_count = 0
        spin_count = 0

        while True:
            if self.check_stop(): return False

            current_force = get_tool_force()
            spin_count += 1
            print(f"[FORCE 7단계 회전] Fz={current_force[2]:.2f}N | Mx={current_force[3]:.2f}Nm | spin={spin_count}")

            if is_done_bolt_tightening(m=0.7, timeout=5, axis=DR_AXIS_Z):
                slip_count += 1
                print(f"[볼트체결 감지] ({slip_count}/2)")
                if slip_count >= 2 and spin_count > 12:
                    break
            else:
                slip_count = 0

            current_j = get_current_posj()
            target_j6 = start_j6 + 360/20
            start_j6 = target_j6
            if target_j6 > 90: target_j6 = 90
            elif target_j6 < -270: target_j6 = -270

            if self.check_stop(): return False
            movej(posj([current_j[0], current_j[1], current_j[2], current_j[3], current_j[4], target_j6]), vel=50, acc=80)
            movel(posx([0,0,-(TOTAL_DOWN/20),0,0,0]), vel=[10,30], acc=[30,50], mod=DR_MV_MOD_REL)

            if start_j6 >= 90: break

        # 쉐이킹
        self.log("7단계: 쉐이킹", 90)
        wait(6)
        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        # j6 초기화
        curr_j = get_current_posj()
        print(f"[DEBUG 7단계] 현재 조인트: {curr_j}")
        curr_j[5] = -180.0
        movej(curr_j, vel=VELJ, acc=ACCJ)  # 캡핑하면서 돌아간 줄 풀기
        print('[DEBUG 7단계] 돌아간 줄 뽑기 끝')

        # Shaking
        J_MIX_1 = [0, 10, 80, 45, 45, 90]
        J_MIX_2 = [0, 10, 80, -45, -45, -90]
        for _ in range(4):
            if self.check_stop(): return False
            movej(J_MIX_1, vel=VEL_SHAKE, acc=ACC_SHAKE)
            movej(J_MIX_2, vel=VEL_SHAKE, acc=ACC_SHAKE)
            movej([0,0,90,0,90,0], vel=VEL_SHAKE, acc=ACC_SHAKE)
            move_periodic(amp=[0,0,40,0,0,120], period=0.4, repeat=1, ref=DR_BASE)
            move_periodic(amp=[0,0,0,40,40,0], period=0.35, repeat=1, ref=DR_BASE)            
        print("[DEBUG 7단계] 쉐이킹 동작 완료")

        # Place
        print('[DEBUG 7단계] place 시작')
        if self.check_stop(): return False
        movel(POS_AIR, vel=VELX, acc=ACCX)
        movel(posx(POS_PLACE[1]), vel=VELX, acc=ACCX)
        self.release()
        movel(posx([0,0,SAFE_Z_OFFSET,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        self.log("7단계 완료: 두번째 캡핑+쉐이킹", 95)
        return True

    # ========================================
    # 메인 실행 루프
    # ========================================

    def run(self):
        """상태 머신 기반 실행"""
        from DSR_ROBOT2 import movej,movel

        self.initialize()
        current_step = 1

        while current_step <= 7 and self.is_running:
            print(f"\n{'='*50}")
            print(f"[현재 단계: {current_step}]")
            print('='*50)

            success = False

            if current_step == 1:
                success = self.step_1_pick_bottle_1()
            elif current_step == 2:
                success = self.step_2_pick_cap_1()
            elif current_step == 3:
                success = self.step_3_capping_shaking_1()
            elif current_step == 4:
                success = self.step_4_pick_error()
            elif current_step == 5:
                success = self.step_5_pick_bottle_2()
            elif current_step == 6:
                success = self.step_6_pick_cap_2()
            elif current_step == 7:
                success = self.step_7_capping_shaking_2()

            if success:
                # 성공: 다음 단계로
                current_step += 1
            else:
                # 실패(0신호): 복구 후 1신호 대기
                print(f"[멈춤] 단계 {current_step}에서 멈춤 감지")
                self.safe_recovery()

                if self.wait_for_restart():
                    # 다음 단계 결정
                    if current_step in [1, 2, 3]:
                        print("[점프] → 4단계(에러 위치)로 이동")
                        current_step = 4
                    elif current_step == 4:
                        print("[점프] → 5단계(두번째 병)로 이동")
                        current_step = 5
                    else:  # 5, 6, 7
                        print("[종료] 프로그램 종료")
                        from DSR_ROBOT2 import movej
                        movej(POS_AIR, vel=VELJ, acc=ACCJ)
                        movel(POS_HOME_BEFORE, vel=VELX, acc=ACCX) # 빼고도 구조물에 안걸리는지 확인 필요함->걸려.....^^
                        movej(J_READY, vel=VELJ, acc=ACCJ)                        
                        break
                    
                else:
                    print("[종료] 재시작 신호 없음")
                    break

        # 완료
        print("\n" + "=" * 50)
        print("[완료] 전체 공정 완료!")
        print("=" * 50)

        movej(J_READY, vel=VELJ, acc=ACCJ)
        self.log("all_process_completed", 100)

    def shutdown(self):
        self.is_running = False
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("integrated_automation_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    from DSR_ROBOT2 import release_force, get_tcp, release_compliance_ctrl

    print(f"엔드이펙터 - Gripper : {get_tcp()}")

    system = IntegratedSystem(node)

    release_force(time=0.0)
    release_compliance_ctrl()

    try:
        time.sleep(1.0)
        system.run()
    except KeyboardInterrupt:
        print("사용자 중단 (Ctrl+C)")
    except Exception as e:
        print(f"에러 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        system.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()