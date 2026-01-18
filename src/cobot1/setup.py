from setuptools import find_packages, setup

package_name = 'cobot1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_basic = cobot1.move_basic:main',
            'move_periodic = cobot1.move_periodic:main',
            'grip_test = cobot1.grip_test:main',
            'sub = cobot1.sub:main',
            'pub = cobot1.pub:main',
            'robo_pub_test = cobot1.robo_pub_test:main',
            'get_pose = cobot1.get_pose:main',
            'move_test = cobot1.move_test:main',
        ],
    },
)
