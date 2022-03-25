from setuptools import setup
import os
import glob

package_name = 'carla_visual_navigation_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	        'local_planner_pid = carla_visual_navigation_agent.local_planner_pid:main',
	        'wheel_odometry_sensor = carla_visual_navigation_agent.wheel_odometry_sensor:main',
	        'localization_sensor_synchronizer = carla_visual_navigation_agent.localization_sensor_synchronizer:main',
            'travelled_distance_sensor = carla_visual_navigation_agent.travelled_distance_sensor:main',
            'path_tracking_sensor = carla_visual_navigation_agent.path_tracking_sensor:main',
            'crash_monitor_node = carla_visual_navigation_agent.crash_monitor_node:main',
        ],
    },
)
