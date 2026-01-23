import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'web_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dongchanseo',
    maintainer_email='ahwkt46@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #'firebase_listener = web_ui.firebase_listener_node:main',
            #'firebase_writer = web_ui.firebase_writer_node:main',
            'joint_firebase = web_ui.joint_firebase_node:main',
            'robot_state_firebase = web_ui.robot_state_firebase_node:main',
            'tcp_firebase = web_ui.tcp_firebase_node:main',
            'move_circle = web_ui.move_circle_node:main',
            'move_line = web_ui.move_line_node:main',
            'tool = web_ui.tool_node:main',
            'move_home = web_ui.move_home_node:main',
            'status_process = web_ui.status_process_node:main',
            'error_firebase = web_ui.error_firebase_node:main',
            'servo_off = web_ui.servo_off_node:main',
            'robot_mode = web_ui.robot_mode_node:main',
            'set_robot_mode = web_ui.set_robot_mode_node:main',
            'recovery_mode = web_ui.recovery_mode_node:main',
            'firebase_periodic_publisher = web_ui.firebase_periodic_publisher:main',
        ],
    },
)
