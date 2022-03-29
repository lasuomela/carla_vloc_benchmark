import os
import glob
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


package_name = "carla_visual_navigation"

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='scenario_dir'
    	),
    	launch.actions.DeclareLaunchArgument(
            name='repetitions'
    	),
	launch.actions.DeclareLaunchArgument(
            name='scenario_timeout',
            default_value='0.0'
    	),
        launch_ros.actions.Node(
            package=package_name,
            executable='scenario_batch_executor_node',
            name='scenario_batch_executor_node',
            output='screen',
            emulate_tty=True,
            parameters=[
            	{
            		'scenario_dir': launch.substitutions.LaunchConfiguration('scenario_dir')
            	},
            	{
    			'repetitions': launch.substitutions.LaunchConfiguration('repetitions')
            	},
            	{
    			'scenario_timeout': launch.substitutions.LaunchConfiguration('scenario_timeout')
            	},
            	]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
