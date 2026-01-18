import rclpy
import DR_init

def main(args=None):
    rclpy.init(args=args)

    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "m0609"

    node = rclpy.create_node("shake_pick_place_node", namespace=ROBOT_ID)

    # Doosan 필수 등록
    DR_init.__dsr__node = node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    from DSR_ROBOT2 import (
        movej, movel, move_periodic,
        set_tool, set_tcp, set_digital_output,
        wait, DR_BASE, DR_TOOL,
        ON, OFF
    )

    # -------------------------------
    # Gripper
    # -------------------------------
    def grip():
        print("그리퍼 닫기 (Grip)")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(1.0)

    def release():
        print("그리퍼 열기 (Release)")
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1.0)

    try:
        set_tool("Tool Weight")
        set_tcp("GripperDA_v1")

        VELOCITY = 300
        ACC = 300

        POS_PICK  = [332.46, 8.53, 84.44, 175.74, 179.82, 175.36]
        POS_PLACE = [252.68, -365.13, 199.45, 89.47, 165.36, 94.89]

        SAFE_Z_OFFSET = 500

        J_READY = [0, 0, 90, 0, 90, 0]
        J_MIX_1 = [0, 10, 80,  45,  45,  90]
        J_MIX_2 = [0, 10, 80, -45, -45, -90]

        # ===============================
        # Step 1. Pick
        # ===============================
        print("1. 물체 집기 시작")
        release()

        movej(J_READY, vel=120, acc=120)

        pick_up = [
            POS_PICK[0],
            POS_PICK[1],
            POS_PICK[2] + SAFE_Z_OFFSET,
            POS_PICK[3],
            POS_PICK[4],
            POS_PICK[5]
        ]

        movej(pick_up, vel=120, acc=120)
        movel(POS_PICK, vel=80, acc=80)
        grip()
        movel(pick_up, vel=80, acc=80)

        # ===============================
        # Step 2. Shaking
        # ===============================
        print("2. 쉐이킹 시작")

        for i in range(2):
            print(f"  - 믹싱 사이클 {i+1}/2")

            movej(J_READY, vel=VELOCITY, acc=ACC)

            print("    [동작] J4, J5 대각 타격")
            movej(J_MIX_1, vel=VELOCITY, acc=ACC)
            movej(J_MIX_2, vel=VELOCITY, acc=ACC)

            print("    [동작] J6 회전 + Z 타격")
            movej(J_READY, vel=VELOCITY, acc=ACC)
            move_periodic(
                amp=[0, 0, 40, 0, 0, 120],
                period=0.4,
                atime=0.05,
                repeat=1,
                ref=DR_BASE
            )

            print("    [동작] 툴 기준 쉐이킹")
            move_periodic(
                amp=[0, 0, 0, 40, 40, 0],
                period=0.35,
                atime=0.05,
                repeat=1,
                ref=DR_TOOL
            )

        # ===============================
        # Step 3. Place
        # ===============================
        print("3. 목적지 이동 및 놓기")

        POS_AIR = [261.31, -343.97, 329.04, 114.58, -179.02, 115.44]
        place_up = [
            POS_PLACE[0],
            POS_PLACE[1],
            POS_PLACE[2] + SAFE_Z_OFFSET,
            POS_PLACE[3],
            POS_PLACE[4],
            POS_PLACE[5]
        ]

        movej(POS_AIR, vel=80, acc=80)
        # movej(place_up, vel=80, acc=80)
        movel(place_up, vel=80, acc=80)
        movel(POS_PLACE, vel=80, acc=80)
        release()
        movel(place_up, vel=80, acc=80)
        

        movej(J_READY, vel=40, acc=40)
        print("모든 공정 완료")

    except Exception as e:
        print(f"[ERROR] {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
