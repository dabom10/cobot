#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
병 이동 및 뚜껑 조립 자동화 시스템
수정 버전 v6 - 2024.01.19

수정 사항:
- [v2] posx[0] → cap_pos[0] 오타 수정
- [v2] movej() 인자 누락 수정
- [v2] 회전 조이기: movel로 Z하강 + Rz회전 동시 수행
- [v2] 클래스 메서드에 @staticmethod 추가
- [v3] J6 범위 수정: -355° → -350°, +355° → +350° (로봇 한계 ±360° 고려)
- [v3] wait 시간 증가로 알람 3509 방지
- [v3] 초기화 안정성 개선 (서비스 연결 대기)
- [v4] movel vel/acc를 리스트 형태로 수정 [선속도, 각속도]
- [v4] set_velx/set_accx 글로벌 속도 설정 추가
- [v5] set_robot_mode(ROBOT_MODE_AUTONOMOUS) 추가 (필수!)
- [v6] 홈 위치로 movej 이동 후 movel 실행 (로봇 상태 안정화)
- [v6] set_robot_mode를 try-except로 감싸서 에러 무시
"""

import time


import rclpy
import DR_init

# ========================================
# 로봇 설정 상수
# ========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v2"

# 이동 속도 및 가속도 (movel용 - [선속도 mm/s, 각속도 deg/s])
VELX = [60, 60]      # 기본 속도
ACCX = [100, 100]    # 기본 가속도
VELX_SLOW = [30, 30]  # 느린 속도 (정밀 작업용)
ACCX_SLOW = [60, 60]
VELOCITY = 60
ACC = 60
VELOCITY2 = 100
ACC2 = 100

# movej용 속도 (스칼라)
VELJ = 60
ACCJ = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ========================================
# 위치 좌표 (티칭으로 실제 좌표 입력 필요!)
# ========================================

# 병 초기 위치 (4개)
BOTTLE_POS_1 = [436.33, 247.01, 55.5+3, 29.08, 177.43, 29.02]
BOTTLE_POS_3 = [208.26, 245.89, 55.62, 168.87, 179.6, 167.49]

BOTTLE_POSITIONS = [BOTTLE_POS_1, BOTTLE_POS_3]

# 병 작업 위치
BOTTLE_TARGET1 = [325.7, 11.10, 105.41, 19.83, -179.47, 19.28]
BOTTLE_TARGET_GO1 = [319.7, 6.10, 119.41, 19.83, -179.47, 19.28]

BOTTLE_TARGET3 = [325.7, 7.10, 105.41+0.12, 19.83, -179.47, 19.28]
BOTTLE_TARGET_GO3 = [319.7, 6.10, 99.41+0.12, 19.83, -179.47, 19.28]

BOTTLE_TARGETS = [BOTTLE_TARGET1, BOTTLE_TARGET3]
BOTTLE_TARGETS_GOS = [BOTTLE_TARGET_GO1, BOTTLE_TARGET_GO3]

# 뚜껑 초기 위치 (4개)
CAP_POS_2 = [569.72, 238.92, 82.52, 130.85, -177.92, 130.39]
CAP_POS_3 = [569.72, 238.92, 59.52, 130.85, -177.92, 130.39]

CAP_POSITIONS = [CAP_POS_2, CAP_POS_3]

# J6 회전 범위 (로봇 한계: ±360°, 여유 10° 확보)
J6_START = -180.0  # 시작 각도 (기존 -355 → -350으로 수정)
J6_END = 180.0     # 끝 각도 (기존 +355 → +350으로 수정)
J6_ROTATION = J6_END - J6_START  # 총 700도 (약 1.94바퀴)

# 나사 피치 (1회전당 하강 거리, mm)
SCREW_PITCH = 2.5
# 총 하강 거리 계산: (700도 / 360도) * 2.5mm ≈ 4.86mm → 여유있게 5mm
TOTAL_DOWN = 5.0

# 그리퍼 설정
GRIPPER_PIN = 1
ON = 1
OFF = 0


class Caping:
    """병 뚜껑 조립 자동화 클래스"""

    @staticmethod
    def grip():
        """그리퍼 닫기"""
        from DSR_ROBOT2 import set_digital_output, wait
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)
        print("  [그리퍼] 닫힘")

    @staticmethod
    def release():
        """그리퍼 열기"""
        from DSR_ROBOT2 import set_digital_output, wait
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.5)
        print("  [그리퍼] 열림")

    @staticmethod
    def initialize_robot():
        """로봇 초기화"""
        from DSR_ROBOT2 import (
            set_tool, set_tcp, set_velx, set_accx,
            set_robot_mode, ROBOT_MODE_AUTONOMOUS,
            movej, posj, wait
        )

        # 서비스 안정화 대기
        print("  [초기화] 서비스 안정화 대기...")
        wait(1.0)

        # 로봇 모드 설정 (에뮬레이터에서는 warning 발생 가능)
        try:
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            print("  [로봇 모드] AUTONOMOUS 설정 완료")
        except Exception as e:
            print(f"  [로봇 모드] 설정 warning (무시 가능): {e}")

        # Tool/TCP 설정 (재시도 로직 포함)
        for attempt in range(3):
            try:
                set_tool(ROBOT_TOOL)
                wait(0.3)
                set_tcp(ROBOT_TCP)
                wait(0.3)
                print(f"  [Tool/TCP] 설정 완료 (시도 {attempt+1})")
                break
            except Exception as e:
                print(f"  [Tool/TCP] 설정 실패 (시도 {attempt+1}): {e}")
                wait(0.5)

        # 글로벌 속도 설정 (movel 기본값)
        set_velx(VELX[0], VELX[1])
        set_accx(ACCX[0], ACCX[1])

        print("#" * 50)
        print("로봇 초기화 완료:")
        print(f"  ROBOT_ID: {ROBOT_ID}")
        print(f"  ROBOT_MODEL: {ROBOT_MODEL}")
        print(f"  ROBOT_TCP: {ROBOT_TCP}")
        print(f"  ROBOT_TOOL: {ROBOT_TOOL}")
        print(f"  VELX: {VELX} (mm/s, deg/s)")
        print(f"  ACCX: {ACCX} (mm/s², deg/s²)")
        print(f"  J6 회전 범위: {J6_START}° → {J6_END}° (총 {J6_ROTATION}°)")
        print("#" * 50)

        # 홈 위치로 이동 (movej - 로봇 상태 안정화)
        print("\n[홈 위치로 이동 중...]")
        Caping.release()
        home = posj(0, 0, 90, 0, 90, 0)
        movej(home, vel=VELJ, acc=ACCJ)
        wait(1.0)
        print("[홈 위치 도착]\n")

    @staticmethod
    def set_j6_to_start():
        """J6를 시작 위치(-350도)로 설정"""
        from DSR_ROBOT2 import get_current_posj, movej, wait

        print(f"  [J6 초기화] {J6_START}도로 이동")
        current_j = get_current_posj()
        current_j[5] = J6_START
        movej(current_j, vel=VELJ, acc=ACCJ)
        wait(1.0)  # 모션 완료 대기 시간 증가 (0.5 → 1.0)

    @staticmethod
    def press_cap():
        from DSR_ROBOT2 import posx, movel, wait, DR_MV_MOD_REL, DR_AXIS_C, movej, posj
        from DSR_ROBOT2 import task_compliance_ctrl, set_stiffnessx, set_desired_force, check_force_condition, DR_AXIS_Z, DR_FC_MOD_REL, release_force, release_compliance_ctrl

        print("4. 그리퍼 열기")
        Caping.release()
        wait(0.5)
        # 위로 올리고 그립 닫기 
        up = posx([0, 0, 100, 0, 0, 0])

        movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        wait(0.3)
        
        Caping.grip()

        print("5. 뚜껑 누르기")
                
        task_compliance_ctrl()  # compliance on
        set_stiffnessx([100.00, 100.00, 50.00, 100.00, 100.00, 100.00],time=0.0)  
        print("순응 제어 설정 완료")    
        # compliance([x,y,z,a,b,c], 대기 시간값) 강성 설정
        down_grip_control = len(BOTTLE_TARGETS_GOS)
        little_down = posx([0,0,down_grip_control*15-80,0,0,0])
        movel(little_down, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)  # 아래로 약간 하강 (5)

        print('힘제어 시작')
        set_desired_force([0.00, 0.00, -55, 0.00, 0.00, 0.00],[0,0,5,0,0,0],time=0.0,mod=DR_FC_MOD_REL)  # force 제어

        while True: # 하강하면서 물체가 검출이 되는지 계속 체크
            #print('힘 체크 시작')
            obj_ok = check_force_condition(DR_AXIS_Z, min=50, max=100)  # z축 기준 / min~max사이의 힘이 발생하면 0 리턴

        #print(obj_ok)
            if not obj_ok:  # 힘 감지 (뚜껑 눌림)
                    # tp_log("뚜껑 누르기 완료")
                    print(obj_ok)
                    print('뚜껑 누르기 감지')
                    break
            continue

        if not obj_ok: # 검출 됐다면
                release_force(time=0.0) # 아래로 더이상 내려가지 않게 force 끔

                print('힘제어 설정 off')

                movel(posx([0,0,70,0,0,0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)  # 위로 위치 조정?
                release_compliance_ctrl()   # compliance 종료
                # 로봇 민감도 25%로 미리 설정해두었음

        wait(0.5)        
        Caping.release()   # 그리퍼 열기

    @staticmethod
    def move_bottle(bottle_idx):
        """병 이동"""
        from DSR_ROBOT2 import posx, movel, wait, DR_MV_MOD_REL

        print(f"\n{'='*50}")
        print(f"병 {bottle_idx+1} 이동")
        print(f"{'='*50}")

        try:
            bottle_pos = BOTTLE_POSITIONS[bottle_idx]
            BOTTLE_TARGET = BOTTLE_TARGETS[bottle_idx]

            Caping.release()

            # 1. 병 위치 접근
            print("1. 병 위치 접근")
            
            approach_pos = posx([bottle_pos[0], bottle_pos[1], bottle_pos[2]+70,
                                bottle_pos[3], bottle_pos[4], bottle_pos[5]])
            movel(approach_pos, vel=VELX, acc=ACCX)
            wait(0.5)

            # 2. 병 위치 도착
            print("2. 병 위치 도착")
            movel(posx(bottle_pos), vel=VELX_SLOW, acc=ACCX_SLOW)
            wait(0.5)

            # 3. 파지
            print("3. 파지")
            Caping.grip()

            # 4. 들어올리기
            print("4. 들어올리기")
            up = posx([0, 0, 100, 0, 0, 0])
            movel(up, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            wait(0.5)

            # 5. 작업 위치 접근
            print("5. 작업 위치 접근")
            target_approach = posx([BOTTLE_TARGET[0], BOTTLE_TARGET[1], BOTTLE_TARGET[2] + 70,
                                   BOTTLE_TARGET[3], BOTTLE_TARGET[4], BOTTLE_TARGET[5]])
            movel(target_approach, vel=VELX, acc=ACCX)
            wait(0.5)

            # 6. 작업 위치 도착
            find_target = posx([BOTTLE_TARGET[0], BOTTLE_TARGET[1], BOTTLE_TARGET[2] - 20,
                                   BOTTLE_TARGET[3], BOTTLE_TARGET[4], BOTTLE_TARGET[5]])            
            print("6. 작업 위치 도착")
            movel(find_target, vel=VELX_SLOW, acc=ACCX_SLOW)
            wait(0.5)

            # 7. 내려놓기
            print("7. 내려놓기")
            Caping.release()

            # 8. 복귀
            print("8. 복귀")
            movel(up, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            wait(0.5)

            print(f"병 {bottle_idx+1} 이동 완료\n")
            return True

        except Exception as e:
            print(f"에러: {e}")
            import traceback
            traceback.print_exc()
            return False

    @staticmethod
    def assemble_cap(cap_idx):
        """뚜껑 조립 - 수정 버전"""
        from DSR_ROBOT2 import (
            posx, movel, movej, wait,
            amovel, amovej, mwait, check_force_condition,
            get_current_posx, get_current_posj,DR_AXIS_C,
            DR_MV_MOD_REL
        )

        print(f"\n{'='*50}")
        print(f"병 {cap_idx+1} 뚜껑 조립")
        print(f"{'='*50}")

        try:
            cap_pos = CAP_POSITIONS[cap_idx]
            BOTTLE_TARGET = BOTTLE_TARGETS[cap_idx]
            Caping.release()

            # 0. J6를 시작 위치(-350도)로 설정
            print(f"0. J6 초기 위치 설정 ({J6_START}도)")
            Caping.set_j6_to_start()

            # 1. 뚜껑 픽업
            print("1. 뚜껑 위치 접근")
            # [수정] posx[0] → cap_pos[0]
            cap_approach = posx([cap_pos[0], cap_pos[1], cap_pos[2] + 80,
                                cap_pos[3], cap_pos[4], cap_pos[5]])
            movel(cap_approach, vel=VELX, acc=ACCX)
            wait(0.5)

            print("2. 뚜껑 위치 도착")
            movel(posx(cap_pos), vel=VELX_SLOW, acc=ACCX_SLOW)
            wait(0.5)

            print("3. 뚜껑 파지")
            Caping.grip()

            # Caping.release()
            print("4. 뚜껑 들어올리기")
            up = posx([0, 0, 100, 0, 0, 0])
            movel(up, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            # wait(0.5)
            BOTTLE_TARGET_GO = BOTTLE_TARGETS_GOS[cap_idx]
            # 2. 병 위로 이동
            print("5. 병 위로 이동")
            cap_above = posx([BOTTLE_TARGET_GO[0], BOTTLE_TARGET_GO[1], BOTTLE_TARGET_GO[2] + 30,
                             BOTTLE_TARGET_GO[3], BOTTLE_TARGET_GO[4], BOTTLE_TARGET_GO[5]])
            # 20 -> 40으로 조정

            movel(cap_above, vel=VELX, acc=ACCX)
            wait(0.5)

            # 3. 뚜껑 올려놓기
            print("6. 뚜껑 올려놓기")
            cap_on = posx([BOTTLE_TARGET_GO[0], BOTTLE_TARGET_GO[1], BOTTLE_TARGET_GO[2] - (cap_idx * 10),
                          BOTTLE_TARGET_GO[3], BOTTLE_TARGET_GO[4], BOTTLE_TARGET_GO[5]])
            movel(cap_on, vel=[20, 20], acc=ACCX_SLOW)
            wait(0.5)

            # 4. 누르기
            Caping.press_cap()

            print("down")
            down = posx([0, 0, -94, 0, 0, 0])
            movel(down, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            print(f"누른 후 다운 : {get_current_posx()}")
            wait(0.5)            

            # 6. 재파지
            print("10. 뚜껑 재파지")
            Caping.grip()
            wait(0.5)

            # ============================================
            # 7. 회전 조이기 (J6 회전 + Z 하강)
            # 작은 스텝으로 나눠서 거의 동시에 실행
            # ============================================
            print("11. 회전 조이기 (J6 회전 + Z 하강)")

            from DSR_ROBOT2 import posj

            # 회전량 설정
            TOTAL_ROTATION = 700.0    # 총 회전량 (J6: -350° → +350°)
            NUM_STEPS = 20            # 스텝 수 (많을수록 부드러움)
            ROTATION_PER_STEP = TOTAL_ROTATION / NUM_STEPS
            DOWN_PER_STEP = TOTAL_DOWN / NUM_STEPS

            print(f"    → 총 회전: {TOTAL_ROTATION}도 (약 {TOTAL_ROTATION/360:.1f}바퀴)")
            print(f"    → 총 하강: {TOTAL_DOWN}mm")
            print(f"    → {NUM_STEPS}스텝으로 실행")

            # 시작 위치 기록
            start_j = get_current_posj()
            start_j6 = start_j[5]
            print(f"    → 시작 J6: {start_j6:.1f}°")

            while True:
                # 현재 조인트 위치
                current_j = get_current_posj()
                # """ 
                # """
                # J6 목표 (누적 회전)
            
                # target_j6 = start_j6 + (ROTATION_PER_STEP * (step + 1))
                target_j6 = start_j6 + (700/20)
                start_j6 = target_j6

                # J6 범위 체크 (-350 ~ +350)
                if target_j6 > 330:
                    target_j6 = 330
                elif target_j6 < -350:
                    target_j6 = -350

                # 목표 조인트 위치
                target_j = posj([
                    current_j[0], current_j[1], current_j[2],
                    current_j[3], current_j[4], target_j6
                ])

                # Z 하강 목표
                down_target = posx([0, 0, -DOWN_PER_STEP, 0, 0, 0])

                # J6 회전 (빠르게)
                movej(target_j, vel=50, acc=80)

                # Z 하강 (빠르게)
                movel(down_target, vel=[10, 30], acc=[30, 50], mod=DR_MV_MOD_REL)

                # 진행 상황 (5스텝마다 출력)
                # if (step + 1) % 5 == 0:
                    # print(f"    [{step+1}/{NUM_STEPS}] J6={target_j6:.1f}°")
                
                close_cap = check_force_condition(DR_AXIS_C, min=20, max=70)  # z축 기준 / 30~100사이의 힘이 발생하면 True(1) 리턴
                print(close_cap)
                if not close_cap:  # 힘 감지 (뚜껑 눌림)
                    print(close_cap)
                    print('뚜껑 잠김')
                    break
                if target_j6 >= J6_END:
                    print("최대 회전 도달")
                    break
                else:
                    continue                

            # 최종 위치 확인
            final_j = get_current_posj()
            print(f"    → 완료! J6: {start_j6:.1f}° → {final_j[5]:.1f}° (회전: {final_j[5]-start_j6:.1f}°)")

            # 8. 완료 - 그리퍼 열고 복귀
            print("12. 작업 완료 - 복귀")
            Caping.release()
            movel(up, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            wait(0.5)

            print(f"병 {cap_idx+1} 뚜껑 조립 완료!\n")
            return True

        except Exception as e:
            print(f"에러: {e}")
            import traceback
            traceback.print_exc()
            return False

    @staticmethod
    def run_full_process():
        """전체 프로세스 - 4개 병 이동 + 조립"""
        from DSR_ROBOT2 import wait

        print("\n" + "="*50)
        print("전체 작업 시작 (4개 병)")
        print("="*50 + "\n")

        for i in range(len(BOTTLE_POSITIONS)):
            print(f"\n>>> 병 {i+1}/{len(BOTTLE_POSITIONS)} 작업\n")

            if not Caping.move_bottle(i):
                print(f"[중단] 병 {i+1} 이동 실패")
                break
            wait(1.0)

            if not Caping.assemble_cap(i):
                print(f"[중단] 병 {i+1} 조립 실패")
                break
            wait(1.0)

        print("\n" + "="*50)
        print("전체 작업 완료!")
        print("="*50)

    @staticmethod
    def run_single_test(bottle_idx):
        """단일 병 테스트"""
        from DSR_ROBOT2 import wait

        if bottle_idx >= len(BOTTLE_POSITIONS):
            print(f"[에러] 잘못된 병 번호: {bottle_idx+1}")
            return

        print(f"\n>>> 병 {bottle_idx+1} 단독 테스트\n")

        if Caping.move_bottle(bottle_idx):
            wait(1.0)
            Caping.assemble_cap(bottle_idx)

    @staticmethod
    def run_bottle_only_test(bottle_idx):
        """병 이동만 테스트 (뚜껑 조립 없이)"""
        if bottle_idx >= len(BOTTLE_POSITIONS):
            print(f"[에러] 잘못된 병 번호: {bottle_idx+1}")
            return

        print(f"\n>>> 병 {bottle_idx+1} 이동 단독 테스트\n")
        Caping.move_bottle(bottle_idx)

    @staticmethod
    def run_cap_only_test(cap_idx):
        """뚜껑 조립만 테스트 (병 이동 없이)"""
        if cap_idx >= len(CAP_POSITIONS):
            print(f"[에러] 잘못된 뚜껑 번호: {cap_idx+1}")
            return

        print(f"\n>>> 뚜껑 {cap_idx+1} 조립 단독 테스트\n")
        Caping.assemble_cap(cap_idx)

    @staticmethod
    def run_rotation_only_test():
        """회전만 테스트 (J6 회전 + Z 하강)"""
        from DSR_ROBOT2 import (
            posj, posx, movej, movel, wait,
            get_current_posx, get_current_posj,check_force_condition,
            DR_MV_MOD_REL,DR_AXIS_C
        )

        print("\n" + "="*50)
        print("회전 테스트 (J6 회전 + Z 하강)")
        print("="*50)

        try:
            # 회전량 설정
            TOTAL_ROTATION = 700.0    # 총 회전량 (J6: -350° → +350°)
            NUM_STEPS = 20            # 스텝 수
            ROTATION_PER_STEP = TOTAL_ROTATION / NUM_STEPS
            DOWN_PER_STEP = TOTAL_DOWN / NUM_STEPS

            print(f"\n1. 회전 설정")
            print(f"   → 총 회전: {TOTAL_ROTATION}도 (약 {TOTAL_ROTATION/360:.1f}바퀴)")
            print(f"   → 총 하강: {TOTAL_DOWN}mm")
            print(f"   → {NUM_STEPS}스텝으로 실행")

            # 시작 위치 확인
            Caping.set_j6_to_start()
            start_pos = get_current_posx()[0]
            start_j = get_current_posj()
            start_j6 = start_j[5]
            print(f"\n2. 시작 위치:")
            print(f"   Task: Z={start_pos[2]:.1f}mm")
            print(f"   Joint: J6={start_j6:.1f}°")

            print(f"\n3. 회전 시작\n")


            while True:
                # 현재 조인트 위치
                current_j = get_current_posj()
                # """ 
                # """
                # J6 목표 (누적 회전)
            
                # target_j6 = start_j6 + (ROTATION_PER_STEP * (step + 1))
                target_j6 = start_j6 + (700/20)
                start_j6 = target_j6

                # J6 범위 체크 (-350 ~ +350)
                if target_j6 > 330:
                    target_j6 = 330
                elif target_j6 < -350:
                    target_j6 = -350

                # 목표 조인트 위치
                target_j = posj([
                    current_j[0], current_j[1], current_j[2],
                    current_j[3], current_j[4], target_j6
                ])

                # Z 하강 목표
                down_target = posx([0, 0, -DOWN_PER_STEP, 0, 0, 0])

                # J6 회전 (빠르게)
                movej(target_j, vel=50, acc=80)

                # Z 하강 (빠르게)
                movel(down_target, vel=[10, 30], acc=[30, 50], mod=DR_MV_MOD_REL)

                # 진행 상황 (5스텝마다 출력)
                # if (step + 1) % 5 == 0:
                    # print(f"    [{step+1}/{NUM_STEPS}] J6={target_j6:.1f}°")
                
                close_cap = check_force_condition(DR_AXIS_C, min=20, max=70)  # z축 기준 / 30~100사이의 힘이 발생하면 True(1) 리턴
                print(close_cap)
                if not close_cap:  # 힘 감지 (뚜껑 눌림)
                    print(close_cap)
                    print('뚜껑 잠김')
                    break
                if target_j6 >= J6_END:
                    print("최대 회전 도달")
                    break                
                else:
                    continue  
            print("\n4. 회전 테스트 완료!")

            # 최종 위치 확인
            final_pos = get_current_posx()[0]
            final_j = get_current_posj()
            print(f"\n   최종 위치:")
            print(f"   Task: Z={final_pos[2]:.1f}mm")
            print(f"   Joint: J6={final_j[5]:.1f}°")
            print(f"\n   변화량:")
            print(f"   Z:  {final_pos[2] - start_pos[2]:.1f}mm")
            print(f"   J6: {final_j[5] - start_j6:.1f}°")

            return True

        except Exception as e:
            print(f"[에러] 회전 테스트 실패: {e}")
            import traceback
            traceback.print_exc()
            return False

    @staticmethod
    def run_force_rotation_test():
        """Force Control + J6 회전 테스트"""
        from DSR_ROBOT2 import (
            posj, movej, wait,
            get_current_posj, get_current_posx,
            task_compliance_ctrl, set_desired_force, release_force,
            DR_FC_MOD_ABS
        )

        print("\n" + "="*50)
        print("Force Control + J6 회전 테스트")
        print("="*50)
        print("\n※ 주의: 시뮬레이터에서는 Force Control이")
        print("   제대로 동작하지 않을 수 있습니다.\n")

        try:
            # 회전량 설정
            TOTAL_ROTATION = 700.0    # 총 회전량
            NUM_STEPS = 10            # 스텝 수
            ROTATION_PER_STEP = TOTAL_ROTATION / NUM_STEPS
            FORCE_Z = -5.0            # Z축 힘 (N) - 아래로 누르는 힘

            print(f"1. 설정")
            print(f"   → 총 회전: {TOTAL_ROTATION}도")
            print(f"   → Z축 힘: {FORCE_Z}N")
            print(f"   → {NUM_STEPS}스텝으로 실행")

            # 시작 위치 확인
            start_j = get_current_posj()
            start_pos = get_current_posx()[0]
            start_j6 = start_j[5]
            print(f"\n2. 시작 위치:")
            print(f"   Z={start_pos[2]:.1f}mm, J6={start_j6:.1f}°")

            # 3. Force Control 활성화
            print(f"\n3. Force Control 활성화...")
            print(f"   → 컴플라이언스 모드 설정")
            task_compliance_ctrl([3000, 3000, 3000, 200, 200, 200])
            wait(0.5)

            print(f"   → Z축 힘 설정: {FORCE_Z}N")
            # set_desired_force(fd, dir, time, mod)
            # fd: [Fx, Fy, Fz, Tx, Ty, Tz]
            # dir: [0,0,1,0,0,0] = Z축 방향으로 힘 적용
            set_desired_force([0, 0, FORCE_Z, 0, 0, 0], [0, 0, 1, 0, 0, 0], 0, DR_FC_MOD_ABS)
            wait(0.5)

            # 4. J6 회전 (Force 유지하면서)
            print(f"\n4. J6 회전 시작 (Force 유지)\n")

            for step in range(NUM_STEPS):
                current_j = get_current_posj()

                # J6 목표 (누적 회전)
                target_j6 = start_j6 + (ROTATION_PER_STEP * (step + 1))

                # J6 범위 체크
                if target_j6 > 350:
                    target_j6 = 350
                elif target_j6 < -350:
                    target_j6 = -350

                # 목표 조인트 위치
                target_j = posj([
                    current_j[0], current_j[1], current_j[2],
                    current_j[3], current_j[4], target_j6
                ])

                # J6 회전
                movej(target_j, vel=30, acc=50)

                # 진행 상황 출력
                if (step + 1) % 2 == 0:
                    curr_j = get_current_posj()
                    curr_pos = get_current_posx()[0]
                    print(f"   [{step+1}/{NUM_STEPS}] J6={curr_j[5]:.1f}°, Z={curr_pos[2]:.1f}mm")

            # 5. Force Control 해제
            print(f"\n5. Force Control 해제...")
            release_force()
            wait(0.5)

            # 최종 위치 확인
            final_j = get_current_posj()
            final_pos = get_current_posx()[0]
            print(f"\n6. 완료!")
            print(f"   최종 위치: Z={final_pos[2]:.1f}mm, J6={final_j[5]:.1f}°")
            print(f"   변화량: Z={final_pos[2]-start_pos[2]:.1f}mm, J6={final_j[5]-start_j6:.1f}°")

            return True

        except Exception as e:
            print(f"\n[에러] Force 테스트 실패: {e}")
            import traceback
            traceback.print_exc()

            # 에러 시 Force Control 해제 시도
            try:
                release_force()
            except:
                pass

            return False

    @staticmethod
    def run_torque_stop_rotation_test():
        """토크 감지 시 회전 정지 테스트 (check_force_condition 사용)"""
        from DSR_ROBOT2 import (
            posj, amovej, movej, wait, mwait,
            get_current_posj, get_current_posx,
            check_force_condition, stop,
            DR_AXIS_C, DR_FC_MOD_ABS
        )

        print("\n" + "="*50)
        print("토크 감지 시 회전 정지 테스트")
        print("="*50)
        print("\n※ 회전축(C/Rz)에서 토크가 감지되면 정지합니다.")
        print("※ 시뮬레이터에서는 토크 감지가 안될 수 있습니다.\n")

        try:
            # 설정
            TORQUE_THRESHOLD = 3.0    # 토크 임계값 (Nm) - 낮을수록 민감
            ROTATION_TARGET = 700.0   # 목표 회전량 (도)

            print(f"1. 설정")
            print(f"   → 토크 임계값: {TORQUE_THRESHOLD} Nm")
            print(f"   → 목표 회전: {ROTATION_TARGET}도")

            # J6를 시작 위치로
            Caping.set_j6_to_start()

            start_j = get_current_posj()
            start_j6 = start_j[5]
            target_j6 = start_j6 + ROTATION_TARGET

            # J6 범위 체크
            if target_j6 > 350:
                target_j6 = 350

            print(f"\n2. 시작 위치: J6={start_j6:.1f}°")
            print(f"   목표 위치: J6={target_j6:.1f}°")

            # 목표 조인트 위치
            target_j = posj([
                start_j[0], start_j[1], start_j[2],
                start_j[3], start_j[4], target_j6
            ])

            # 3. 비동기 회전 시작
            print(f"\n3. 회전 시작 (비동기)")
            print(f"   토크 모니터링 중...\n")

            # amovej로 비동기 회전 시작
            amovej(target_j, vel=20, acc=40)

            # 4. 토크 모니터링 루프
            torque_detected = False
            max_check_time = 30.0  # 최대 모니터링 시간 (초)
            check_interval = 0.05  # 체크 간격 (초)
            elapsed = 0.0

            while elapsed < max_check_time:
                # check_force_condition(axis, min, max, ref)
                # axis: DR_AXIS_C = 회전축 (Rz/C)
                # 토크가 min~max 범위를 벗어나면 True 반환
                #
                # 방법 1: check_force_condition 사용
                # - 특정 축의 힘/토크가 조건을 만족하는지 확인
                # - True 반환 시 조건 만족 (토크 감지)

                torque_check = check_force_condition(
                    DR_AXIS_C,           # 축: C (회전축/Rz)
                    -TORQUE_THRESHOLD,   # min: -3 Nm
                    TORQUE_THRESHOLD,    # max: +3 Nm
                    DR_FC_MOD_ABS        # 절대값 모드
                )

                # check_force_condition이 True면 범위 내, False면 범위 초과
                # → 토크가 임계값을 초과하면 True가 아닌 False 반환
                # 주의: API 동작 확인 필요 - 반대일 수 있음

                current_j = get_current_posj()

                # 0.5초마다 상태 출력
                if int(elapsed * 10) % 5 == 0:
                    print(f"   [{elapsed:.1f}s] J6={current_j[5]:.1f}°, 토크체크={torque_check}")

                # 토크 감지 시 (check_force_condition 결과에 따라 조건 조정 필요)
                # 일반적으로 True = 조건 만족 (범위 내), False = 조건 불만족 (범위 초과)
                if not torque_check:  # 범위 초과 = 토크 감지
                    print(f"\n   ★ 토크 감지! J6={current_j[5]:.1f}°에서 정지")
                    stop(0)  # 즉시 정지 (0=급정지)
                    torque_detected = True
                    break

                # 목표 도달 체크
                if abs(current_j[5] - target_j6) < 1.0:
                    print(f"\n   목표 도달! J6={current_j[5]:.1f}°")
                    break

                wait(check_interval)
                elapsed += check_interval

            # 5. 결과
            final_j = get_current_posj()
            print(f"\n4. 완료!")
            print(f"   최종 위치: J6={final_j[5]:.1f}°")
            print(f"   회전량: {final_j[5] - start_j6:.1f}°")

            if torque_detected:
                print(f"   결과: 토크 감지로 조기 정지")
            else:
                print(f"   결과: 정상 완료 (토크 미감지)")

            return True

        except Exception as e:
            print(f"\n[에러] 토크 감지 테스트 실패: {e}")
            import traceback
            traceback.print_exc()

            # 에러 시 정지 시도
            try:
                from DSR_ROBOT2 import stop
                stop(0)
            except:
                pass

            return False

    @staticmethod
    def run_coordinate_system_test():
        """좌표계 사용 테스트 (기본 좌표계 + 사용자 좌표계)"""
        from DSR_ROBOT2 import (
            posx, movel, movej, posj, wait,
            get_current_posx, get_current_posj,
            set_ref_coord, set_user_cart_coord,
            DR_BASE, DR_WORLD, DR_TOOL,
            DR_MV_MOD_REL, DR_MV_MOD_ABS
        )

        print("\n" + "="*50)
        print("좌표계 사용 테스트")
        print("="*50)
        print("\n좌표계 종류:")
        print("  - DR_BASE: 로봇 베이스 좌표계 (기본)")
        print("  - DR_WORLD: 월드 좌표계")
        print("  - DR_TOOL: 툴 좌표계")
        print("  - 사용자 정의 좌표계: set_user_cart_coord()로 설정\n")

        try:
            # 1. 현재 위치 확인
            print("1. 현재 위치 확인")
            current_pos = get_current_posx()[0]
            print(f"   현재 위치: X={current_pos[0]:.1f}, Y={current_pos[1]:.1f}, Z={current_pos[2]:.1f}")

            # 2. 기본 좌표계 설정 (DR_BASE)
            print("\n2. 기본 좌표계 설정: DR_BASE")
            set_ref_coord(DR_BASE)
            print("   → 이후 모든 movel은 기본적으로 베이스 좌표계 사용")

            # 3. 베이스 좌표계로 이동
            print("\n3. 베이스 좌표계로 상대 이동 (Z +50mm)")
            move_up = posx([0, 0, 50, 0, 0, 0])
            movel(move_up, vel=VELX_SLOW, acc=ACCX_SLOW, mod=DR_MV_MOD_REL)
            # ref 파라미터 없음 → 기본 좌표계(DR_BASE) 사용
            wait(0.5)

            new_pos = get_current_posx()[0]
            print(f"   이동 후: Z={new_pos[2]:.1f}mm (변화: {new_pos[2]-current_pos[2]:.1f}mm)")

            # 4. 특정 동작에서 툴 좌표계 사용
            print("\n4. 특정 동작에서 툴 좌표계 사용")
            print("   movel(..., ref=DR_TOOL)로 툴 좌표계 지정")

            # 툴 좌표계 기준 Z방향 이동 (툴 끝단 기준 앞으로)
            tool_move = posx([0, 0, -30, 0, 0, 0])
            movel(tool_move, vel=VELX_SLOW, acc=ACCX_SLOW, mod=DR_MV_MOD_REL, ref=DR_TOOL)
            wait(0.5)

            final_pos = get_current_posx()[0]
            print(f"   이동 후: X={final_pos[0]:.1f}, Y={final_pos[1]:.1f}, Z={final_pos[2]:.1f}")

            # 5. 사용자 좌표계 설정 예제
            print("\n5. 사용자 좌표계 설정 방법 (참고)")
            print("   # 사용자 좌표계 정의 (원점 위치 + 방향)")
            print("   user_coord = posx([100, 200, 0, 0, 0, 45])  # 원점(100,200,0), Z축 45도 회전")
            print("   set_user_cart_coord(user_coord, ref=DR_BASE)")
            print("")
            print("   # 사용자 좌표계로 이동")
            print("   movel(target, vel=vel, acc=acc, ref=DR_USER)  # DR_USER=사용자 좌표계")

            # 6. 원위치 복귀
            print("\n6. 원위치 복귀")
            original = posx([current_pos[0], current_pos[1], current_pos[2],
                           current_pos[3], current_pos[4], current_pos[5]])
            movel(original, vel=VELX, acc=ACCX)
            wait(0.5)

            print("\n좌표계 테스트 완료!")
            print("\n※ 핵심 요약:")
            print("   - set_ref_coord(DR_BASE): 기본 좌표계 설정")
            print("   - movel(..., ref=DR_TOOL): 특정 명령에서 다른 좌표계 사용")
            print("   - 기본 좌표계를 변경해도 ref= 파라미터로 오버라이드 가능")

            return True

        except Exception as e:
            print(f"\n[에러] 좌표계 테스트 실패: {e}")
            import traceback
            traceback.print_exc()
            return False

    @staticmethod
    def get_current_position():
        """현재 위치 확인"""
        from DSR_ROBOT2 import get_current_posx, get_current_posj

        try:
            pos_x = get_current_posx()[0]
            pos_j = get_current_posj()

            print(f"\n현재 태스크 위치:")
            print(f"  X: {pos_x[0]:.2f} mm")
            print(f"  Y: {pos_x[1]:.2f} mm")
            print(f"  Z: {pos_x[2]:.2f} mm")
            print(f"  Rx: {pos_x[3]:.2f}°")
            print(f"  Ry: {pos_x[4]:.2f}°")
            print(f"  Rz: {pos_x[5]:.2f}°")

            print(f"\n현재 관절 위치:")
            for i, j in enumerate(pos_j):
                print(f"  J{i+1}: {j:.2f}°")

            return pos_x, pos_j
        except Exception as e:
            print(f"[에러] 위치 조회 실패: {e}")
            return None, None


def main(args=None):
    """메인 함수"""
    import time

    rclpy.init(args=args)
    node = rclpy.create_node("bottle_cap_assembly", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 서비스 연결 대기 (초기화 안정성 개선)
        print("서비스 연결 대기 중...")
        time.sleep(2.0)  # 컨트롤러 초기화 대기

        # 노드 설정 후 DSR_ROBOT2 import
        from DSR_ROBOT2 import get_tool
        print(f"엔드 이펙터 설정 : {get_tool()}")

        # 로봇 초기화
        Caping.initialize_robot()

        # 메뉴
        print("\n" + "="*60)
        print("실행 모드 선택:")
        print("="*60)
        print("  1. 단일 테스트 (병 이동 + 뚜껑 조립)")
        print("  2. 전체 실행 (4개 병 이동 + 조립)")
        print("  3. 병 이동만 테스트")
        print("  4. 뚜껑 조립만 테스트")
        print("  5. 현재 위치 확인")
        print("  6. 회전만 테스트 (J6 회전 + Z 하강)")
        print("  7. Force Control 테스트 (힘 제어 + J6 회전)")
        print("  8. 토크 감지 회전 테스트 (토크 감지 시 정지)")
        print("  9. 좌표계 사용 테스트 (기본/사용자 좌표계)")
        print("="*60)

        choice = input("선택 (1-9): ").strip()

        if choice == "1":
            bottle_num = int(input("병 번호 (1-4): ").strip()) - 1
            Caping.run_single_test(bottle_num)
        elif choice == "2":
            Caping.run_full_process()
        elif choice == "3":
            bottle_num = int(input("병 번호 (1-4): ").strip()) - 1
            Caping.run_bottle_only_test(bottle_num)
        elif choice == "4":
            cap_num = int(input("뚜껑 번호 (1-4): ").strip()) - 1
            Caping.run_cap_only_test(cap_num)
        elif choice == "5":
            Caping.get_current_position()
        elif choice == "6":
            Caping.run_rotation_only_test()
        elif choice == "7":
            Caping.run_force_rotation_test()
        elif choice == "8":
            Caping.run_torque_stop_rotation_test()
        elif choice == "9":
            Caping.run_coordinate_system_test()
        else:
            print("[에러] 잘못된 선택")
        
        # 쉐이킹

    except KeyboardInterrupt:
        print("\n\n[중단] 사용자가 중단했습니다")
    except Exception as e:
        print(f"\n[에러] 예외 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[종료] 프로그램 종료")
        rclpy.shutdown()


if __name__ == "__main__":
    main()