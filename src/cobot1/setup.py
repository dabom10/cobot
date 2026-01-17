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
    maintainer='dabom',
    maintainer_email='dabom@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	# 'move_periodic = cobot1.move_periodic:main',
        	'move_basic = cobot1.move_basic:main',
        	'shake = cobot1.shake_v2:main',
        	'grip = cobot1.grip:main',
        	'release = cobot1.release:main',
        ],
    },
)
