#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
로봇 공정 통합 Launch 파일
- 통합 로봇 노드 실행 (캡핑 + 쉐이킹)
- 캡핑 → 쉐이킹 순서로 2사이클 자동 실행
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch 설명 생성"""

    # 통합 로봇 노드 (캡핑 + 쉐이킹 모두 처리)
    robot_node = Node(
        package='robot',
        executable='robot',
        name='integrated_automation_node',
        namespace='dsr01',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_node,
    ])
