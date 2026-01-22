#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
로봇 제어권 복구 스크립트
- 티치팬던트에서 제어권을 가져온 후 코드에서 다시 제어권 획득
- 서보 온 및 엔드이펙터 설정 복구
- 사용법: ros2 run robot recover
"""

import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v2"


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("recover_node", namespace=ROBOT_ID)

    DR_init.__dsr__node = node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    from DSR_ROBOT2 import (
        set_robot_mode, ROBOT_MODE_AUTONOMOUS,
        set_tool, set_tcp, get_tcp,
        release_force, release_compliance_ctrl
    )

    print("=" * 50)
    print("로봇 제어권 복구 스크립트")
    print("=" * 50)

    try:
        # 1. 자율 모드로 전환 (제어권 획득 + 서보 온)
        print("\n[1/4] 자율 모드 전환 중...")
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        print("      -> 자율 모드 전환 완료 (서보 온)")

        # 2. 툴 설정
        print(f"\n[2/4] 툴 설정 중... ({ROBOT_TOOL})")
        set_tool(ROBOT_TOOL)
        print(f"      -> 툴 설정 완료")

        # 3. TCP 설정
        print(f"\n[3/4] TCP 설정 중... ({ROBOT_TCP})")
        set_tcp(ROBOT_TCP)
        print(f"      -> TCP 설정 완료")

        # 4. 힘 제어 해제
        print("\n[4/4] 힘 제어 해제 중...")
        release_force(time=0.0)
        release_compliance_ctrl()
        print("      -> 힘 제어 해제 완료")

        # 확인
        current_tcp = get_tcp()
        print("\n" + "=" * 50)
        print("복구 완료!")
        print(f"현재 TCP: {current_tcp}")

        if current_tcp == ROBOT_TCP:
            print("상태: 정상")
        else:
            print(f"경고: TCP가 {ROBOT_TCP}가 아닙니다!")
        print("=" * 50)

    except Exception as e:
        print(f"\n에러 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()