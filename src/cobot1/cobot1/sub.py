import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('joint_state_subscriber')

        self.subscription = self.create_subscription(
            JointState,
            '/dsr01/joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info('JointState Subscriber has been started.')

    def listener_callback(self, msg: JointState):
        self.get_logger().info(
            f'Joint Names: {msg.name}\n'
            f'Positions: {msg.position}\n'
            f'Velocities: {msg.velocity}\n'
            f'Efforts: {msg.effort}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = JointStateSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
