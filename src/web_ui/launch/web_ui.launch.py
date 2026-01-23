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

        # Error Firebase Node - /dsr01/error 토픽 구독
        Node(
            package='web_ui',
            executable='error_firebase',
            name='error_firebase',
            output='screen',
        ),

        # recovery_mode Node - Firebase에서 명령 수신하여 서보 오프 실행
        Node(
            package='web_ui',
            executable='recovery_mode',
            name='recovery_mode',
            output='screen',
        ),

        # Robot Mode Node - 로봇 모드 정보 조회
        Node(
            package='web_ui',
            executable='robot_mode',
            name='robot_mode',
            output='screen',
        ),

        # Set Robot Mode Node - 로봇 모드 변경
        Node(
            package='web_ui',
            executable='set_robot_mode',
            name='set_robot_mode',
            output='screen',
        ),
        Node(
            package='web_ui',
            executable='firebase_periodic_publisher',
            name='firebase_periodic_publisher',
            output='screen',
        )
    ])
