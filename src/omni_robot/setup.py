from setuptools import setup
import os
from glob import glob

package_name = 'omni_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Omnidirectional robot with sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_controller = omni_robot.dummy_controller:main',
            'target_detector = omni_robot.target_detector:main',
            'target_behavior_layer = omni_robot.target_behavior_layer:main',
            'target_nav_bridge = omni_robot.target_nav_bridge:main',
            'cmd_vel_bridge = omni_robot.cmd_vel_bridge:main',
            'scan_bridge = omni_robot.scan_bridge:main',
            'odom_tf_bridge = omni_robot.odom_tf_bridge:main',
        ],
    },
)