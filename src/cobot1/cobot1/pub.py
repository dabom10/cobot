import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')

        self.publisher_ = self.create_publisher(
            JointState,
            '/dsr01/joint_states_pub',
            10
        )

        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.start_time = self.get_clock().now()

        self.get_logger().info('JointState Publisher has been started.')

    def timer_callback(self):
        msg = JointState()

        # header
        msg.header.stamp = self.get_clock().now().to_msg()

        # joint names (예시: 6축 로봇)
        msg.name = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        ]

        # position 값 (간단한 sin 파형 예제)
        # t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 90.0]

        # 선택사항 (안 써도 됨)
        msg.velocity = []
        msg.effort = []

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing positions: {msg.position}')


def main(args=None):
    rclpy.init(args=args)

    node = JointStatePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
