#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
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
VELX = [150, 150] # [60, 60]
ACCX = [150, 150]
VELX_FAST = [180, 180]
ACCX_FAST = [180, 180]
VELX_SLOW = [30, 30]
ACCX_SLOW = [60, 60]
VELJ = 60
ACCJ = 60
VEL_SHAKE = 150
ACC_SHAKE = 150
MOVE_DIST = 40 # 위아래 이동 거리
SNAP_J5 = 20
SNAP_J6 = 15

# 그리퍼 및 핀
ON, OFF = 1, 0
SAFE_Z_OFFSET = 100


# [쉐이킹 for문 내 변수 모음]
# 쉐이킹 반복 횟수
ITERATION = 5
# 각 관절별 흔들림 진폭
AMP_J3 = 10 
AMP_J4 = 15
AMP_J5 = 20 
AMP_J6 = 20
TILT_ANGLE = 45 # 기울기 각도

# 로봇 관절 한계치 (입력해주신 값 기준, 안전 여유 2도 제외)
joint_limits = [
            (-360 + 2, 360 - 2), # J1
            (-95 + 2, 95 - 2),   # J2
            (-135 + 2, 135 - 2), # J3
            (-360 + 2, 360 - 2), # J4
            (-135 + 2, 135 - 2), # J5
            (-360 + 2, 360 - 2)  # J6
        ]

# ========================================
# 좌표 데이터 (제공된 리스트 활용)
# ========================================
# [캡핑 관련]
BOTTLE_POS_2 = [347.21, -96.46, 50.73, 19.45, 179.44, 20.02]
BOTTLE_POSITIONS = [
    [436.33, 247.01, 58.5, 29.08, 180, 29.02],   # BOTTLE_POS_1
    [208.26, 245.89, 55.62, 168.87, 180, 167.49]  # BOTTLE_POS_3
]
BOTTLE_TARGETS = [
    [325.7, 11.10, 105.41, 19.83, 180, 19.28],   # TARGET 1
    [325.7, 7.10, 105.53, 19.83, 180, 19.28]     # TARGET 3
]
BOTTLE_TARGETS_GOS = [
    [319.7, 6.10, 119.41, 19.83, 180, 19.28],
    [318.7+3, 5.10, 99.53, 19.83, 180, 19.28] # [319.7, 6.10, 99.53, 19.83, 180, 19.28] 
]
CAP_POSITIONS = [
    [569.72, 238.92, 82.52, 130.85, 180, 130.39], # CAP 2
    [569.72, 238.92, 59.52, 130.85, 180, 130.39]  # CAP 3
]

# [쉐이킹 관련]
POS_PICK = [
    [322.7, 8.10, 87.41, 19.83, -179.47, 19.28],
    [321.10, 8.29, 80.98, 178.85, 179.24, 178.52]
]
POS_PLACE = [
    [266.1, -386.71, 210.94, 92.46, 162.31, 92.86], # z 10 up
    [392.69, -381.66, 196.38, 91.56, 162.08, 91.85] # z 10 up
]
POS_AIR = [328, -215, 456, 176, -176, 151] # x: 261.31 -> 300.31로 변경
J_READY = [0, 0, 90, 0, 90, 0]
J_MIX_1 = [0, 10, 80,  45,  45, 90]
J_MIX_2 = [0, 10, 80, -45, -45, -90]
POS_HOME_BEFORE = [317.34, -307.11, 344.5, 125.41, -170.49, 127.82] # 마지막 동작에서 movel -> movej로 디버깅해서 괜찮을 수도 있음
J_SHAKE_START=[2.98, 17, 63.38, -24.65, 52.10, -180.0]


# 캡핑 회전 상수
J6_START, J6_END = -270.0, 90.0
TOTAL_DOWN = 5.0

class IntegratedSystem:
    def __init__(self, node):
        self.node = node
        self.status_pub = node.create_publisher(String, "status", 10)
        self.process_pub = node.create_publisher(Int32, "process", 10)
        # node.create_subscriber(String, "command", self.move_home, 10)

    def log(self, status_text, progress_val):
        """상태와 진행률 동시 발행"""
        s_msg = String()
        s_msg.data = status_text
        self.status_pub.publish(s_msg)
        
        p_msg = Int32()
        p_msg.data = int(progress_val)
        self.process_pub.publish(p_msg)
        print(f"[STATUS] {status_text} | [PROGRESS] {progress_val}%")

    # 홈 위치 이동 함수
    def move_home(self):
        from DSR_ROBOT2 import movej, movel
        movej(J_READY, vel=VELJ, acc=ACCJ)

    @staticmethod
    def grip():
        from DSR_ROBOT2 import set_digital_output, wait
        set_digital_output(1, ON); set_digital_output(2, OFF)
        wait(0.8)

    @staticmethod
    def release():
        from DSR_ROBOT2 import set_digital_output, wait
        set_digital_output(1, OFF); set_digital_output(2, ON)
        wait(0.8)

    def initialize(self):
        from DSR_ROBOT2 import set_tool, set_tcp, movej, wait, set_robot_mode, ROBOT_MODE_AUTONOMOUS
        try:
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        except: pass
        set_tool(ROBOT_TOOL); set_tcp(ROBOT_TCP)
        movej(J_READY, vel=VELJ, acc=ACCJ)
        self.log("시스템 준비 완료", 0)

    def capping_process(self, idx, base_progress):
        """캡핑 공정 세부 로직"""
        from DSR_ROBOT2 import (posx, movel, movej, wait, DR_MV_MOD_REL, get_current_posj, 
                                 task_compliance_ctrl, set_stiffnessx, set_desired_force, 
                                 check_force_condition, DR_AXIS_Z, DR_AXIS_C, release_force, get_tool_force,
                                 release_compliance_ctrl, DR_FC_MOD_REL, posj,get_current_posx, is_done_bolt_tightening)

        self.log(f"캡핑 공정을 시작합니다.. (사이클 : {idx+1} 회)", base_progress + 5)
        # 1. 병 이동
        self.release()
        bottle_pos = BOTTLE_POSITIONS[idx]
        target_pos = BOTTLE_TARGETS[idx]
        # if idx == 1:
        #     try:    
        #         movel(posx(BOTTLE_POS_2), vel=VELX_FAST, acc=ACCX_FAST)
        #     except:
        #         self.move_home()
        #     wait(2)  # 두 번째 병 대기 시간

        movel(posx([bottle_pos[0], bottle_pos[1], bottle_pos[2]+70, bottle_pos[3], bottle_pos[4], bottle_pos[5]]), vel=VELX_FAST, acc=ACCX_FAST)
        movel(posx(bottle_pos), vel=VELX_SLOW, acc=ACCX_SLOW)
        self.grip()
        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        # movel(posx(target_pos), vel=VELX, acc=ACCX)
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

        # # 힘 제어 누르기
        # print("뚜껑 누르기")
        # self.release(); wait(0.3)
        # movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        # self.grip()

        # task_compliance_ctrl()
        # set_stiffnessx([100, 100, 50, 100, 100, 100])
        # print("순응 제어 설정 완료")    
        
        # down_grip_control = len(BOTTLE_TARGETS_GOS)
        # little_down = posx([0,0,down_grip_control*15-80,0,0,0])
        # movel(little_down, vel=60, acc=60, mod=DR_MV_MOD_REL)  # 아래로 약간 하강
        # print('힘제어 시작')
        # set_desired_force([0, 0, -60, 0, 0, 0], [0, 0, 5, 0, 0, 0], mod=DR_FC_MOD_REL)
        
        # while True:
        #     obj_ok = check_force_condition(DR_AXIS_Z, min=50, max=100) 
        #     if not obj_ok:  # 힘 감지 (뚜껑 눌림)
        #             # tp_log("뚜껑 누르기 완료")
        #             print(obj_ok)
        #             print('뚜껑 누르기 감지')
        #             break
        #     continue

        # if not obj_ok: # 검출 됐다면
        #         release_force(time=0.0) # 아래로 더이상 내려가지 않게 force 끔

        #         print('힘제어 설정 off')

        #         movel(posx([0,0,70,0,0,0]), vel=60, acc=60, mod=DR_MV_MOD_REL)  # 위로 위치 조정
        #         release_compliance_ctrl()   
        
        # 힘 제어 누르기
        for i in range(2):
            force_list = [-70, -60]
            f_list = [50, 50]
            print("뚜껑 누르기 : ", [i])
            if i == 1 :
                # 그리퍼 90도 회전
                rot = posj(0,0,0,0,0,90)
                movej(rot, vel=60, acc=60, mod=DR_MV_MOD_REL)
        
            if i == 0 :
                self.release()
                wait(0.3)
                movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            self.grip()

            task_compliance_ctrl()
            set_stiffnessx([100, 100, 50, 100, 100, 100])
            print("순응 제어 설정 완료")    
            
            down_grip_control = len(BOTTLE_TARGETS_GOS)
            little_down = posx([0,0,down_grip_control*15-75,0,0,0])
            movel(little_down, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)  # 아래로 약간 하강
            print('힘 제어 시작')
            set_desired_force([0, 0, force_list[i], 0, 0, 0], [0, 0, 5, 0, 0, 0], mod=DR_FC_MOD_REL)
            while True:
                obj_ok = check_force_condition(DR_AXIS_Z, min=f_list[i], max=150) 
                if not obj_ok:  # 힘 감지 (뚜껑 눌림)
                        # tp_log("뚜껑 누르기 완료")
                        print(obj_ok)
                        print('뚜껑 누르기 감지')
                        break
                continue

            if not obj_ok: # 검출 됐다면
                release_force(time=0.0) # 아래로 더이상 내려가지 않게 force 끔  
                print('힘제어 설정 off')   
                release_compliance_ctrl() 
                movel(posx([0,0,70,0,0,0]), vel=60, acc=60, mod=DR_MV_MOD_REL)  # 위로 위치 조정
                # release_compliance_ctrl()       

        # 회전 조이기
        print('회전 조이기 시작')
        self.release()

        # [수정] 회전 조이기 전 J6를 -180도로 초기화
        # 두 번째 병에서도 충분한 회전 범위(-180 ~ 180)를 확보하기 위함
        # 이전에는 첫 번째 병 후 J6가 180 근처여서 두 번째 병에서 조금만 돌고 멈추는 문제 발생
        curr_j = get_current_posj()
        curr_j[5] = J6_START  # J6_START = -180.0
        movej(curr_j, vel=VELJ, acc=ACCJ)
        print(f"[DEBUG] J6 초기화 완료: {J6_START}도")

        down = posx([0, 0, -94, 0, 0, 0])
        movel(down, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        # print(f"누른 후 다운 : {get_current_posx()}")
        start_j = get_current_posj()
        start_j6 = start_j[5]  # 이제 항상 -180에서 시작       
        print('while 시작')
        self.grip()

        # [슬립 감지] Mx 기반 슬립 감지 변수
        # Mx > 0.6 이면 슬립 상태로 판단 (그리퍼만 헛돌고 캡이 안 돌아감)
        # 3회 연속 감지 시 종료하여 오탐 방지
        slip_count = 0
        SLIP_THRESHOLD = 2      # 연속 감지 횟수
        MX_SLIP_VALUE = 0.7     # 슬립 판단 임계값 (Nm)
        spin_count = 0 

        while True:
            current_force = get_tool_force()
            mx = current_force[3]  # Mx 토크 값
            spin_count += 1
            # [디버깅] 힘(N)과 토크(Nm) 6개 값 모두 출력
            print(f"[Force] Fx={current_force[0]:.2f}, Fy={current_force[1]:.2f}, Fz={current_force[2]:.2f} (N)")
            print(f"[Torque] Mx={mx:.2f}, My={current_force[4]:.2f}, Mz={current_force[5]:.2f} (Nm)")

            # [슬립 감지] Mx가 임계값 초과 시 슬립 카운트 증가
            # if mx > MX_SLIP_VALUE:
            #     slip_count += 1
            #     print(f"[슬립 감지] Mx={mx:.2f} > {MX_SLIP_VALUE} ({slip_count}/{SLIP_THRESHOLD})")
            #     if slip_count >= SLIP_THRESHOLD and spin_count > 5:  # 연속 감지 및 최소 회전 후 종료
            #         print('뚜껑 조이기 완료 (슬립 감지)')
            #         break
            #     elif is_done_bolt_tightening():  # 추가 조건: 볼트 조임 완료 감지 함수
            #         print('뚜껑 조이기 완료 (볼트 조임 완료 감지)')
            #         break
            # else:
            #     slip_count = 0  # 조건 불만족 시 리셋

            if is_done_bolt_tightening(m=0.7, timeout=5, axis=DR_AXIS_Z):
                slip_count += 1
                print(f"[볼트체결 감지] ({slip_count}/{SLIP_THRESHOLD})")
                if slip_count >= SLIP_THRESHOLD and spin_count > 12:
                    print('뚜껑 조이기 완료 (볼트체결 감지)')
                    break
            else:
                slip_count = 0

            print('while 진입')
            current_j = get_current_posj()
            target_j6 = start_j6 + 360/20
            start_j6 = target_j6
            if target_j6 > 90:      # J6_END
                target_j6 = 90
            elif target_j6 < -270:  # J6_START
                target_j6 = -270

            target_j = posj([
                current_j[0], current_j[1], current_j[2],
                current_j[3], current_j[4], target_j6
            ])

            movej(target_j, vel=50, acc=80)
            movel(posx([0,0,-(TOTAL_DOWN/20),0,0,0]), vel=[10,30], acc=[30,50], mod=DR_MV_MOD_REL)

            # 안전장치: 회전 제한 도달 시 종료
            if start_j6 >= 90:
                print('뚜껑 조이기 회전 제한 도달')
                break              
            
        # self.release()
        # movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        self.log(f"캡핑 공정 완료 (사이클 : {idx+1} 회)", base_progress + 25)

    def shaking_process(self, idx, base_progress):
        """쉐이킹 공정 세부 로직"""
        from DSR_ROBOT2 import movel, movej, get_current_posj, DR_BASE, DR_TOOL, posx, DR_MV_MOD_REL, wait, move_periodic

        self.log(f"쉐이킹 공정을 시작합니다.. (사이클 : {idx+1} 회)", base_progress + 30)
        
        # 1. Pick (캡핑된 병 위치)
        # pick_pos = POS_PICK[idx] # 캡핑하고 바로 시작하는거라 필요없음
        # movej(J_READY, vel=VELJ, acc=ACCJ) # 지움
        # movel(posx(pick_pos), vel=VELX, acc=ACCX)
        # self.grip()
        # movel(posx([0,0,SAFE_Z_OFFSET,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL) # 어차피 shaking 할때 홈 위치로 가서 이 부분 없어도 괜찮음
        
        wait(6)
        movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)

        # j6 초기화 
        curr_j = get_current_posj()
        print(curr_j)
        curr_j[5] = -180.0
        movej(curr_j, vel=VELJ, acc=ACCJ) # 캡핑하면서 돌아간 줄 풀기
        print('돌아간 줄 뽑기 끝')

        # 2. Shaking

        J_SHAKE = [0, 0, 90, 0, 0, -180]
        J_SHAKE_2 = [0, 0, 90, 0, 0, -180]

        for _ in range(4):
            movej(J_SHAKE, vel=VEL_SHAKE, acc=ACC_SHAKE)
            movej(J_SHAKE_2, vel=VEL_SHAKE, acc=ACC_SHAKE)
        # # def get_safe_joint(base_j, offsets):
        # for _ in range(2):
        #     # movej(J_READY, vel=60, acc=ACC_SHAKE) # 갑자기 슉 가는 부분(해결함)
        #     movej(J_MIX_1, vel=VEL_SHAKE, acc=ACC_SHAKE)
        #     movej(J_MIX_2, vel=VEL_SHAKE, acc=ACC_SHAKE)
        #     movej(J_READY, vel=VEL_SHAKE, acc=ACC_SHAKE)
        #     move_periodic(amp=[0,0,40,0,0,120], period=0.4, repeat=1, ref=DR_BASE)
        #     move_periodic(amp=[0,0,0,40,40,0], period=0.35, repeat=1, ref=DR_TOOL)
        #     """현재 각도에 오프셋을 더한 뒤 한계치 넘지 않도록 보정"""
        #     safe_target = []
        #     for i in range(6):
        #         target_val = base_j[i] + offsets[i]
        #         min_limit, max_limit = joint_limits[i]
        #         if target_val < min_limit: target_val = min_limit
        #         if target_val > max_limit: target_val = max_limit
        #         safe_target.append(target_val)
        #     return safe_target
        

        # # movel(posx([505.31, 43.97, 340.04, 114.58, -179.02, 115.44]), vel=VEL_SHAKE, acc=ACC_SHAKE)  # 쉐이킹 시작 위치로 이동 -> 필요없음
        # movej(J_SHAKE_START, vel=VELJ, acc=ACCJ)
        # print("쉐이킹 시작 위치 도달")
        # # movel(posx([0,0,100,0,0,0]), vel=100, acc=100, mod=DR_MV_MOD_REL)
        # start_j_normal = get_current_posj()    
        
        # for i in range(ITERATION):
        #     print(f"기본 쉐이킹 동작 시작...")

        #     # shake 1) 위로 이동 오프셋 
        #     offsets_up = [0, 0, -AMP_J3, -AMP_J4, -AMP_J5, -AMP_J6]
        #     target_up = get_safe_joint(start_j_normal, offsets_up)
        #     # shake 2) 아래로 이동 오프셋
        #     offsets_down = [0, 0, AMP_J3, AMP_J4, AMP_J5, AMP_J6]
        #     target_down = get_safe_joint(start_j_normal, offsets_down)

        #     movej(target_up, vel=VEL_SHAKE, acc=ACC_SHAKE, radius=5)
        #     movej(target_down, vel=VEL_SHAKE, acc=ACC_SHAKE, radius=5)

        # ## --- [모션 2: 가로(좌우 비틀기) 쉐이킹] ---
        # # J4를 90도 돌리지 않고, 현재 상태에서 J4와 J6만 교차로 흔듭니다.
        # print("2. 좌우 비틀기(가로) 쉐이킹 시작")
        
        # for i in range(ITERATION):
        #     # J3(팔꿈치), J5(스냅)는 고정(0)하고 J4와 J6만 반대 방향으로 흔듦
        #     # 이렇게 하면 병이 제자리에서 좌우로 빠르게 회전하며 가로 쉐이킹 효과를 냅니다.
        #     target_left = get_safe_joint(start_j_normal, [0, 0, 0, -30, 0, 30])
        #     target_right = get_safe_joint(start_j_normal, [0, 0, 0, 30, 0, -30])
            
        #     movej(target_left, vel=VEL_SHAKE + 50, acc=ACC_SHAKE + 200, radius=5)
        #     movej(target_right, vel=VEL_SHAKE + 50, acc=ACC_SHAKE + 200, radius=5)
        print("쉐이킹 동작 완료")
        
        # 3. Place
        print('place 시작')
        place_pos = POS_PLACE[idx]
        movel(POS_AIR, vel=VELX, acc=ACCX)
        movel(posx(place_pos), vel=VELX, acc=ACCX)
        self.release()
        movel(posx([0,0,SAFE_Z_OFFSET,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        
        self.log(f"쉐이킹 동작 완료 (사이클 : {idx+1} 회)", base_progress + 50)

    def run(self):
        """전체 2사이클 실행 루프"""
        from DSR_ROBOT2 import movel, movej
        self.initialize()
        for i in range(2):
            base_p = i * 50
            # self.capping_process(i, base_p)
            self.shaking_process(i, base_p)
        
        from DSR_ROBOT2 import movej
        movej(POS_AIR, vel=VELJ, acc=ACCJ)
        movel(POS_HOME_BEFORE, vel=VELX, acc=ACCX) # 빼고도 구조물에 안걸리는지 확인 필요함->걸려.....^^
        movej(J_READY, vel=VELJ, acc=ACCJ)
        self.log("all_process_completed", 100)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("integrated_automation_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL #[305.31, -343.97, 390.04, 114.58, -179.02, 115.44]
    from DSR_ROBOT2 import release_force, get_tcp, release_compliance_ctrl

    if get_tcp() != ROBOT_TCP:
        print(f"엔드이펙터 - Gripper 오류: {get_tcp()} != {ROBOT_TCP}")
        node.destroy_node()
        rclpy.shutdown()

    print(f"엔드이펙터 - Gripper : {get_tcp()}")

    system = IntegratedSystem(node)
    
    release_force(time=0.0)
    release_compliance_ctrl()

    try:
        time.sleep(1.0)
        system.run()
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