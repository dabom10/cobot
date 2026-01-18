import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetCurrentPose


def main(args=None):
    rclpy.init(args=args)

    node = Node('get_current_pose_client')

    client = node.create_client(
        GetCurrentPose,
        '/dsr01/system/get_current_pose'
    )

    # 서비스 서버 대기
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('get_current_pose 서비스 대기 중...')

    request = GetCurrentPose.Request()

    # 👉 interface show 결과에 따라 필요하면 설정
    # 보통 0 = BASE, 1 = WORLD

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        res = future.result()
        node.get_logger().info(f'현재 포즈: {res.pos}')
        node.get_logger().info(f'성공 여부: {res.success}')
    else:
        node.get_logger().error('서비스 호출 실패')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
