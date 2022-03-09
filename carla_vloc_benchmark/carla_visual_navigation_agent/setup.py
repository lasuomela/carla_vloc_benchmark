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
            'ad_agent = carla_visual_navigation_agent.ad_agent:main',
            'local_planner = carla_visual_navigation_agent.local_planner:main',
	    'local_planner_pid = carla_visual_navigation_agent.local_planner_pid:main',
	    'odometry_noise_simulator = carla_visual_navigation_agent.odometry_noise_node:main',
            'testing_waypoints = carla_visual_navigation_agent.testing_waypoints:main'
        ],
    },
)
