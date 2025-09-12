from setuptools import setup
import os
from glob import glob

package_name = 'keyboard_teleop_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
             ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
             ('share/' + package_name, ['package.xml']),
             (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
             (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
             (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
         ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toycar',
    maintainer_email='codrutmihailaza@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_listener = keyboard_teleop_pkg.cmd_vel_listener:main',
            'cmd_vel_bridge = keyboard_teleop_pkg.cmd_vel_bridge:main',
            'odom_bridge = keyboard_teleop_pkg.odom_bridge:main',
            'tf_publisher_node = keyboard_teleop_pkg.tf_publisher_node:main',
            'keyboard_teleop = keyboard_teleop_pkg.keyboard_teleop:main',
            'navigation_node = keyboard_teleop_pkg.navigation_node:main',
            'exploration_node = keyboard_teleop_pkg.exploration_node:main',
            'imu_node = keyboard_teleop_pkg.imu_node:main',
            'arduino_odometry_node = keyboard_teleop_pkg.arduino_odometry:main',
            'imu2 = keyboard_teleop_pkg.imu2:main'
        ],
    },
)

