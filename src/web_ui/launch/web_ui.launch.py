from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Joint Firebase Node - 조인트, TCP, 로봇 상태를 Firebase로 전송
        Node(
            package='web_ui',
            executable='joint_firebase',
            name='joint',
            output='screen',
        ),

        # Robot State Firebase Node - 로봇 연결 상태, Description을 Firebase로 전송
        Node(
            package='web_ui',
            executable='robot_state_firebase',
            name='robot_state',
            output='screen',
        ),

        # TCP Firebase Node - TCP 위치 (x, y, z, rx, ry, rz)를 Firebase로 전송
        Node(
            package='web_ui',
            executable='tcp_firebase',
            name='tcp',
            output='screen',
        ),

        # Move Circle Node - Firebase에서 명령 수신하여 원형 이동 실행
        Node(
            package='web_ui',
            executable='move_circle',
            name='move_circle',
            output='screen',
        ),

        # Move Line Node - Firebase에서 명령 수신하여 직선 이동 실행
        Node(
            package='web_ui',
            executable='move_line',
            name='move_line',
            output='screen',
        ),

        # Tool Node - 현재 Tool 조회 및 설정
        Node(
            package='web_ui',
            executable='tool',
            name='tool',
            output='screen',
        ),

        # Move Home Node - Firebase에서 명령 수신하여 홈 위치로 이동
        Node(
            package='web_ui',
            executable='move_home',
            name='move_home',
            output='screen',
        ),

        # Status Process Node - /dsr01/status, /dsr01/process 토픽 구독
        Node(
            package='web_ui',
            executable='status_process',
            name='status_process',
            output='screen',
        ),

        # Firebase Writer Node - 작업 완료 수 등 기타 데이터 전송
        # Node(
        #     package='web_ui',
        #     executable='firebase_writer',
        #     name='firebase_writer_node',
        #     output='screen',
        # ),
    ])
