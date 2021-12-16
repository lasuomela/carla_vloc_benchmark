from setuptools import setup
import os
from glob import glob

package_name = 'carla_visual_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'utils'), glob('utils/*.sh')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xosc')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'catalogs'), glob('config/catalogs/*.xosc')),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
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
            'image_capture_node = carla_visual_navigation.image_capture_node:main',
            'scenario_batch_executor_node = carla_visual_navigation.scenario_batch_executor_node:main',
            'place_recognition_evaluator_node = carla_visual_navigation.place_recognition_evaluator_node:main',
            'visual_localizer_evaluator_node = carla_visual_navigation.visual_localizer_evaluator_node:main'
        ],
    },
)
