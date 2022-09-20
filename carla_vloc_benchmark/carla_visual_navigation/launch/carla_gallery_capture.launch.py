import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

package_name='carla_visual_navigation'

def generate_launch_description():
    ld = launch.LaunchDescription([
    	launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='5'
        ),
        launch.actions.DeclareLaunchArgument(
	name='avoid_risk',
    	default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
	name='image_save_path',
    	default_value='/image-gallery'
        ),
        launch.actions.DeclareLaunchArgument(
	name='camera_topic',
    	default_value='/carla/ego_vehicle/rgb_front/image'
        ),
        launch.actions.DeclareLaunchArgument(
	name='odometry_topic',
    	default_value='/carla/ego_vehicle/odometry'
        ),
        launch.actions.DeclareLaunchArgument(
	name='image_density',
    	default_value='2.0'
        ),
        launch.actions.DeclareLaunchArgument(
	name='base_frame',
    	default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
	name='sensor_frame',
    	default_value='ego_vehicle/rgb_front'
        ),
        launch.actions.DeclareLaunchArgument(
	name='synchrony_diff',
    	default_value='0.01'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kp_lateral',
            default_value='0.9'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Ki_lateral',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kd_lateral',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kp_longitudinal',
            default_value='0.206'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Ki_longitudinal',
            default_value='0.0206'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kd_longitudinal',
            default_value='0.515'
        ),
        launch.actions.DeclareLaunchArgument(
            name='control_time_step',
            default_value= '0.05'
        ),
#        launch.actions.IncludeLaunchDescription(
#    		launch.launch_description_sources.PythonLaunchDescriptionSource(
#        	os.path.join(get_package_share_directory(
#   	 	'carla_ad_agent'), 'carla_ad_agent.launch.py')
#	    ),
#	    launch_arguments={
#		'avoid_risk': launch.substitutions.LaunchConfiguration('avoid_risk'),
#	    }.items()
#	),
#        launch.actions.IncludeLaunchDescription(
#            launch.launch_description_sources.PythonLaunchDescriptionSource(
#                os.path.join(get_package_share_directory(
#                    'carla_waypoint_publisher'), 'carla_waypoint_publisher.launch.py')
#            ),
#            launch_arguments={
#                'host': launch.substitutions.LaunchConfiguration('host'),
#                'port': launch.substitutions.LaunchConfiguration('port'),
#                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
#                'role_name': 'ego_vehicle'
#            }.items()
#        ),
        launch_ros.actions.Node(
            package='carla_visual_navigation_agent',
            executable='local_planner_pid',
            name=['local_planner_', launch.substitutions.LaunchConfiguration('role_name')],
            output='screen',
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                },
                {
                    'odometry_topic': '/carla/ego_vehicle/odometry'
                },
                {
                    'Kp_longitudinal': launch.substitutions.LaunchConfiguration('Kp_longitudinal')
                },
                {
                    'Ki_longitudinal': launch.substitutions.LaunchConfiguration('Ki_longitudinal')
                },
                {
                    'Kd_longitudinal': launch.substitutions.LaunchConfiguration('Kd_longitudinal')
                },
                {
                    'control_time_step': launch.substitutions.LaunchConfiguration('control_time_step')
                },
            ]
        ),
        launch_ros.actions.Node(
            package=package_name,
            executable='image_capture_node',
            output='screen',
            emulate_tty=True,
            parameters=[
            	{
                    'image_save_path': launch.substitutions.LaunchConfiguration('image_save_path')
                },
                {
                    'image_node': launch.substitutions.LaunchConfiguration('camera_topic')
                },
                {
                    'odometry_node': launch.substitutions.LaunchConfiguration('odometry_topic')
                },
                {
                    'image_density': launch.substitutions.LaunchConfiguration('image_density')
                },
                {
                    'base_frame': launch.substitutions.LaunchConfiguration('base_frame')
                },
                {
                    'sensor_frame': launch.substitutions.LaunchConfiguration('sensor_frame')
                },
                {
                    'synchrony_diff': launch.substitutions.LaunchConfiguration('synchrony_diff')
                }
        	]
        )
        ])
    return ld
	
if __name__ == '__main__':
    generate_launch_description()
	
     
