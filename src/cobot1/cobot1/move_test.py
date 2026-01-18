import rclpy
import math
import DR_init

from rclpy.node import Node
from sensor_msgs.msg import JointState

# ======================
# 로봇 설정
# ======================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 20   # ⚠️ 처음엔 무조건 저속
ACC = 20

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class JointStateMoveNode():

    def __init__(self):
        #super().__init__("joint_state_move", namespace=ROBOT_ID)

        node = rclpy.create_node("joint_state_move", namespace=ROBOT_ID)

        # DR_init에 노드 등록
        DR_init.__dsr__node = self

        self.get_logger().info("Initializing robot...")
        self.initialize_robot()

        self.latest_joint = None

        self.create_subscription(
            JointState,
            "/dsr01/joint_states_pub",   # ⚠️ 로봇용 토픽과 분리
            self.joint_callback,
            10
        )

        # 주기적으로 movej 실행
        self.create_timer(0.5, self.move_timer)

    def initialize_robot(self):
        from DSR_ROBOT2 import set_tool, set_tcp
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)

    def joint_callback(self, msg: JointState):
        if len(msg.position) != 6:
            self.get_logger().warn("Invalid joint length")
            return

        # rad → deg
        self.latest_joint = [math.degrees(p) for p in msg.position]

    def move_timer(self):
        from DSR_ROBOT2 import posj
        if self.latest_joint is None:
            return

        from DSR_ROBOT2 import movej

        try:
            movej(
                posj(self.latest_joint),
                vel=VELOCITY,
                acc=ACC
            )
            self.get_logger().info(f"movej: {self.latest_joint}")
        except Exception as e:
            self.get_logger().error(f"movej failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMoveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
