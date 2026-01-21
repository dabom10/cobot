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
VELX = [60, 60]
ACCX = [100, 100]
VELX_FAST = [100, 100]
ACCX_FAST = [120, 120]
VELX_SLOW = [30, 30]
ACCX_SLOW = [60, 60]
VELJ = 60
ACCJ = 60
VEL_SHAKE = 400
ACC_SHAKE = 400

# 그리퍼 및 핀
ON, OFF = 1, 0
SAFE_Z_OFFSET = 100

# ========================================
# 좌표 데이터 (제공된 리스트 활용)
# ========================================
# [캡핑 관련]
BOTTLE_POSITIONS = [
    [436.33, 247.01, 58.5, 29.08, 177.43, 29.02],   # BOTTLE_POS_1
    [208.26, 245.89, 55.62, 168.87, 179.6, 167.49]  # BOTTLE_POS_3
]
BOTTLE_TARGETS = [
    [325.7, 11.10, 105.41, 19.83, -179.47, 19.28],   # TARGET 1
    [325.7, 7.10, 105.53, 19.83, -179.47, 19.28]     # TARGET 3
]
BOTTLE_TARGETS_GOS = [
    [319.7, 6.10, 119.41, 19.83, -179.47, 19.28],
    [319.7, 6.10, 99.53, 19.83, -179.47, 19.28]
]
CAP_POSITIONS = [
    [569.72, 238.92, 82.52, 130.85, -177.92, 130.39], # CAP 2
    [569.72, 238.92, 59.52, 130.85, -177.92, 130.39]  # CAP 3
]

# [쉐이킹 관련]
POS_PICK = [
    [322.7, 8.10, 87.41, 19.83, -179.47, 19.28],
    [321.10, 8.29, 80.98, 178.85, 179.24, 178.52]
]
POS_PLACE = [
    [266.1, -386.71, 200.94, 92.46, 162.31, 92.86],
    [392.69, -381.66, 186.38, 91.56, 162.08, 91.85]
]
POS_AIR = [305.31, -343.97, 390.04, 114.58, -179.02, 115.44] # x: 261.31 -> 300.31로 변경
J_READY = [0, 0, 90, 0, 90, 0]
J_MIX_1 = [0, 10, 80,  45,  45,  90]
J_MIX_2 = [0, 10, 80, -45, -45, -90]
POS_HOME_BEFORE = [317.34, -307.11, 344.5, 125.41, -170.49, 127.82] # 마지막 동작에서 movel -> movej로 디버깅해서 괜찮을 수도 있음

# 캡핑 회전 상수
J6_START, J6_END = -180.0, 180.0
TOTAL_DOWN = 5.0

class IntegratedSystem:
    def __init__(self, node):
        self.node = node
        self.status_pub = node.create_publisher(String, "status", 10)
        self.process_pub = node.create_publisher(Int32, "process", 10)

    def log(self, status_text, progress_val):
        """상태와 진행률 동시 발행"""
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
                                 check_force_condition, DR_AXIS_Z, DR_AXIS_C, release_force, 
                                 release_compliance_ctrl, DR_FC_MOD_REL, posj,get_current_posx)

        self.log(f"캡핑 공정을 시작합니다.. (사이클 : {idx+1} 회)", base_progress + 5)
        
        # 1. 병 이동
        self.release()
        bottle_pos = BOTTLE_POSITIONS[idx]
        target_pos = BOTTLE_TARGETS[idx]
        
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
            force_list = [-60, -50]
            f_list = [55, 50]
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
            little_down = posx([0,0,down_grip_control*15-80,0,0,0])
            movel(little_down, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)  # 아래로 약간 하강
            print('힘 제어 시작')
            set_desired_force([0, 0, force_list[i], 0, 0, 0], [0, 0, 5, 0, 0, 0], mod=DR_FC_MOD_REL)
            while True:
                obj_ok = check_force_condition(DR_AXIS_Z, min=f_list[i], max=100) 
                if not obj_ok:  # 힘 감지 (뚜껑 눌림)
                        # tp_log("뚜껑 누르기 완료")
                        print(obj_ok)
                        print('뚜껑 누르기 감지')
                        break
                continue

            if not obj_ok: # 검출 됐다면
                
                    release_force(time=0.0) # 아래로 더이상 내려가지 않게 force 끔  
                    print('힘제어 설정 off')    
                    movel(posx([0,0,70,0,0,0]), vel=60, acc=60, mod=DR_MV_MOD_REL)  # 위로 위치 조정
                    release_compliance_ctrl()       

        # 회전 조이기
        print('회전 조이기 시작')
        self.release()
        down = posx([0, 0, -94, 0, 0, 0])
        movel(down, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        print(f"누른 후 다운 : {get_current_posx()}")        
        start_j = get_current_posj()
        start_j6 = start_j[5]       
        print('while 시작')
        self.grip()
        while True:
            print('while 진입')
            # count += 1
            current_j = get_current_posj()
            target_j6 = start_j6 + 360/20 
            start_j6 = target_j6 
            if target_j6 > 180:
                target_j6 = 180
            elif target_j6 < -180:
                target_j6 = -180
            
            target_j = posj([
                current_j[0], current_j[1], current_j[2],
                current_j[3], current_j[4], target_j6
            ])
            # target_j = posj([
            #     current_j[0], current_j[1], current_j[2],
            #     current_j[3], current_j[4], J6_END + 60
            # ])
            movej(target_j, vel=50, acc=80)
            movel(posx([0,0,-(TOTAL_DOWN/20),0,0,0]), vel=[10,30], acc=[30,50], mod=DR_MV_MOD_REL)
            if not check_force_condition(DR_AXIS_C, min=20, max=70):
                print('뚜껑 조이기 완료')
                break            # if not check_force_condition(DR_AXIS_C, min=20, max=70):
            
            elif start_j6 >= 180: # 안전장치
                print('뚜껑 조이기 회전 제한 도달')
                break
            # elif count >= 20: # 안전장치
            #     print('뚜껑 조이기 시도 횟수 제한 도달')
            #     break
            else:
                continue              
            
        # self.release()
        # movel(posx([0,0,100,0,0,0]), vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        self.log(f"캡핑 공정 완료 (사이클 : {idx+1} 회)", base_progress + 25)

    def shaking_process(self, idx, base_progress):
        """쉐이킹 공정 세부 로직"""
        from DSR_ROBOT2 import movel, movej, move_periodic, get_current_posj, DR_BASE, DR_TOOL, posx, DR_MV_MOD_REL, wait

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
        curr_j[5] = 0
        movej(curr_j, vel=VELJ, acc=ACCJ)

        # 2. Shaking
        for _ in range(2):
            movej(J_READY, vel=60, acc=ACC_SHAKE) # 갑자기 슉 가는 부분(해결함)
            movej(J_MIX_1, vel=VEL_SHAKE, acc=ACC_SHAKE)
            movej(J_MIX_2, vel=VEL_SHAKE, acc=ACC_SHAKE)
            movej(J_READY, vel=VEL_SHAKE, acc=ACC_SHAKE)
            move_periodic(amp=[0,0,40,0,0,120], period=0.4, repeat=1, ref=DR_BASE)
            move_periodic(amp=[0,0,0,40,40,0], period=0.35, repeat=1, ref=DR_TOOL)

        # 3. Place
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
            self.capping_process(i, base_p)
            self.shaking_process(i, base_p)
        
        from DSR_ROBOT2 import movej
        movej(POS_AIR, vel=VELJ, acc=ACCJ)
        movel(POS_HOME_BEFORE, vel=VELX, acc=ACCX) # 빼고도 구조물에 안걸리는지 확인 필요함->걸려.....^^
        movej(J_READY, vel=VELJ, acc=ACCJ)
        self.log("모든 프로세스가 완료되었습니다.", 100)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("integrated_automation_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    system = IntegratedSystem(node)

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