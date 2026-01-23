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
VELX = [150, 150]
ACCX = [150, 150]
VELX_FAST = [180, 180]
ACCX_FAST = [180, 180]
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
    [321.7, 5.10, 99.53, 19.83, 180, 19.28] # 318.7 + 3
]
CAP_POSITIONS = [
    [569.72, 238.92, 82.52, 130.85, 180, 130.39],
    [569.72, 238.92, 59.52, 130.85, 180, 130.39]
]

J_READY = [0, 0, 90, 0, 90, 0]
POS_AIR = [328, -215, 456, 176, -176, 151]
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
        self.trigger_next_cycle = False
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
        from DSR_ROBOT2 import set_tool, set_tcp, movej, set_robot_mode, ROBOT_MODE_AUTONOMOUS, get_tcp, wait, release_force, release_compliance_ctrl
        
        self.get_logger().info("로봇 드라이버 연결 및 모드 설정 중...")
        # 자율 모드 설정 시도 (드라이버 준비 대기 포함 최대 5회 재시도)
        for i in range(5):
            try:
                set_robot_mode(ROBOT_MODE_AUTONOMOUS)
                release_force(time=0.0)
                release_compliance_ctrl()
                break
            except:
                self.get_logger().warn(f"로봇 모드 설정 재시도 중... ({i+1}/5)")
                time.sleep(1.0)

        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        wait(0.5) # 설정 반영 대기
        
        self.get_logger().info("초기 위치(J_READY)로 이동...")
        movej(J_READY, vel=VELJ, acc=ACCJ)
        self.get_logger().info(f"엔드이펙터 설정 확인 - TCP: {get_tcp()}")
        self.log("캡핑 시스템 준비 완료", 0)

    def shaking_done_callback(self, msg):
        """쉐이킹 완료 신호 수신"""
        cycle = msg.data
        self.get_logger().info(f"쉐이킹 완료 신호 수신 (사이클: {cycle})")

        if self.waiting_for_shaking and cycle == self.current_cycle:
            self.waiting_for_shaking = False
            self.trigger_next_cycle = True
            self.get_logger().info("다음 사이클 트리거 활성화")

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
            force_list = [-70, -60]
            f_list = [50, 50]
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

            # 가상 모드 대응: 힘 감지 루프에 타임아웃 및 대기 시간 추가
            start_wait = time.time()
            while True:
                obj_ok = check_force_condition(DR_AXIS_Z, min=f_list[i], max=150)
                if not obj_ok:
                    self.get_logger().info("뚜껑 누르기 감지")
                    break
                
                if time.time() - start_wait > 2.0: # 2초 이상 감지 안되면 가상 모드로 간주하고 통과
                    self.get_logger().info("힘 제어 대기 타임아웃 (가상 모드 대응)")
                    break
                wait(0.1) # CPU 점유 방지 및 드라이버 통신 허용

            if not obj_ok:
                release_force(time=0.0)
                self.get_logger().info("힘제어 설정 off")
                release_compliance_ctrl()
                movel(posx([0,0,70,0,0,0]), vel=60, acc=60, mod=DR_MV_MOD_REL)

        # 회전 조이기
        self.get_logger().info("회전 조이기 시작")
        self.release()

        curr_j = get_current_posj()
        curr_j[5] = J6_START
        movej(curr_j, vel=VELJ, acc=ACCJ)
        self.get_logger().info(f"J6 초기화 완료: {J6_START}도")

        # [수정] 이전 단계에서 70mm 상승했으므로, 다시 캡에 닿으려면 최소 -75mm 이상 하강해야 함
        down = posx([0, 0, -94, 0, 0, 0])
        movel(down, vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
        curr_pos, _ = get_current_posx()
        self.get_logger().info(f"누른 후 다운 위치 : {curr_pos}")
        
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
                if slip_count >= SLIP_THRESHOLD and spin_count > 12:
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
        self.log("모든 공정이 완료되었습니다.", 100)
        self.all_done = True

    def run(self):
        """메인 실행 함수"""
        self.get_logger().info("캡핑 노드 실행 루프 시작")
        self.initialize()
        self.run_capping_cycle()

        # 이벤트 루프 - 쉐이킹 완료 신호 대기 및 다음 사이클 제어
        while rclpy.ok() and not self.all_done:
            if self.trigger_next_cycle:
                self.trigger_next_cycle = False
                self.current_cycle += 1
                
                if self.current_cycle < self.total_cycles:
                    self.run_capping_cycle()
                else:
                    self.finish_all()
            
            # 로봇이 대기 중일 때 메시지 처리를 위해 spin_once 호출
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = CappingRobotNode()
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
