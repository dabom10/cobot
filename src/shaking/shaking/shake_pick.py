import rclpy
import DR_init


# ===============================
# Global Parameters
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# Motion
VELOCITY = 500
ACC = 500
SAFE_Z_OFFSET = 100

# Joint Pose
J_READY = [0, 0, 90, 0, 90, 0]
J_MIX_1 = [0, 10, 80,  45,  45,  90]
J_MIX_2 = [0, 10, 80, -45, -45, -90]

# Task Pose (Raw)
POS_PICK = [322.7, 8.10, 87.41, 19.83, -179.47, 19.28]

POS_AIR = [261.31, -343.97, 400.04, 114.58, -179.02, 115.44]

POS_PLACE = [
    [251.1, -386.71, 215.94, 92.46, 162.31, 92.86], # first place
    [399.69, -381.66, 186.38, 91.56, 162.08, 91.85] # second place
]

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("shake_pick_place_node", namespace=ROBOT_ID)

    # Doosan 필수 등록
    DR_init.__dsr__node = node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    from DSR_ROBOT2 import (
        posx,
        movej, movel, move_periodic,
        set_tool, set_tcp, set_digital_output,
        wait, DR_BASE, DR_TOOL, DR_MV_MOD_REL,
        ON, OFF
    )

    # ===============================
    # IO Control
    # ===============================
    def grip():
        print("[IO] Gripper Close")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(1.0)

    def release():
        print("[IO] Gripper Open")
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1.0)

    # ===============================
    # Pick
    # ===============================
    def pick(p):
        print("[STEP] Pick")

        release()
        movej(J_READY, vel=120, acc=120)

        pick_up = posx(0, 0, SAFE_Z_OFFSET, 0, 0, 0)
        pick_target = posx(*p)

        # Z 확보
        # movel(pick_up, vel=80, acc=80, mod=DR_MV_MOD_REL)

        # Pick 위치 이동
        movel(pick_target, vel=80, acc=80)
        grip()

        # 다시 Z 확보
        movel(pick_up, vel=80, acc=80, mod=DR_MV_MOD_REL)

    # ===============================
    # Shaking
    # ===============================
    def shaking(cycle=2):
        print("[STEP] Shaking")

        for i in range(cycle):
            print(f"  - Cycle {i+1}/{cycle}")

            movej(J_READY, vel=VELOCITY, acc=ACC)

            movej(J_MIX_1, vel=VELOCITY, acc=ACC)
            movej(J_MIX_2, vel=VELOCITY, acc=ACC)

            movej(J_READY, vel=VELOCITY, acc=ACC)
            move_periodic(
                amp=[0, 0, 40, 0, 0, 120],
                period=0.4,
                atime=0.05,
                repeat=1,
                ref=DR_BASE
            )

            move_periodic(
                amp=[0, 0, 0, 40, 40, 0],
                period=0.35,
                atime=0.05,
                repeat=1,
                ref=DR_TOOL
            )

    # ===============================
    # Place
    # ===============================
    def place(p):
        print("[STEP] Place")

        place_up = posx(0, 0, SAFE_Z_OFFSET, 0, 0, 0)
        place_target = posx(*p)

        # 1. 현재 위치 기준 Z 상승 & place 위치 위로 이동
        movel(place_up, vel=80, acc=80, mod=DR_MV_MOD_REL)
        movel(POS_AIR, vel=80, acc=80)

        # 2. Place 위치로 이동
        movel(place_target, vel=80, acc=80)

        # 3 Release
        release()

        # 4. 다시 Z 상승
        movel(place_up, vel=80, acc=80, mod=DR_MV_MOD_REL)

    try:
        # Tool / TCP
        set_tool("Tool Weight")
        set_tcp("GripperDA_v2")

        # ===============================
        # Main Flow
        # ===============================
        pick(POS_PICK)
        shaking(cycle=2)
        place(POS_PLACE[1])

        movel(POS_AIR, vel=120, acc=120)
        movej(J_READY, vel=120, acc=120)
        print("[DONE] All Process Completed")

    except Exception as e:
        print(f"[ERROR] {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
