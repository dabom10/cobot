import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import DR_init
import sys

def listener_callback(msg):
    from DSR_ROBOT2 import movej, posj, ROBOT_MODE_AUTONOMOUS
    print('받음:', msg.position)
    msg.position=[0.0,0.0,0.0,0.0,0.0]
    movej(posj(msg.position),50, 50)
    

def main(args=None):
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "m0609"
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    #ROS 2와 네트워크를 통해 통신할 수 있게 설정 
    rclpy.init(args=args)
    node = rclpy.create_node('example_py', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

	#로봇 제어를 위한 핵심 기능 구현
	#DSR_ROBOT2가져오기는 ROS 2 노드( DR_init.__dsr__node)를 초기화한 후 사용해야 함.
    from DSR_ROBOT2 import movej, posj, set_robot_mode, ROBOT_MODE_AUTONOMOUS

	#로봇의 작동 모드 및 속도 설정
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # node = Node('joint_state_ps_node')

    pub = node.create_publisher(
        JointState,
        '/dsr01/joint_states',
        10
    )

    sub = node.create_subscription(
        JointState,
        '/dsr01/joint_states',
        listener_callback,
        10
    )

    def timer_callback():
        msg = JointState()
        msg.name = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        ]
        #msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #pub.publish(msg)
        sub

    timer = node.create_timer(0.1, timer_callback)

    node.get_logger().info('script-style pub/sub started')

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
