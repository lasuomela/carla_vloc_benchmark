import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

package_name='carla_visual_navigation'

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
	name='avoid_risk',
    	default_value='False'
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
	name='pose_publish_topic',
    	default_value='/carla/ego_vehicle/place_reg_loc'
        ),
        launch.actions.DeclareLaunchArgument(
	name='place_reg_markers_topic',
    	default_value='/carla/ego_vehicle/place_reg_markers'
        ),
        launch.actions.DeclareLaunchArgument(
	name='global_extractor_name',
    	default_value='netvlad'
        ),
        launch.actions.DeclareLaunchArgument(
	name='gallery_global_descriptor_path',
    	default_value='/image-gallery/example_dir/outputs/netvlad+superpoint_aachen+superglue/global-feats-netvlad.h5'
        ),
        launch.actions.DeclareLaunchArgument(
	name='image_gallery_path',
    	default_value='/image-gallery/example_dir/'
        ),
        launch.actions.DeclareLaunchArgument(
	name='localization_frequence',
    	default_value='2.0'
        ),
        launch.actions.DeclareLaunchArgument(
	name='top_k_matches',
    	default_value='4'
        ),
        launch.actions.DeclareLaunchArgument(
	name='synchrony_diff',
    	default_value='0.01'
        ),
        launch.actions.DeclareLaunchArgument(
	name='compensate_sensor_offset',
    	default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
	name='base_frame',
    	default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
	name='sensor_frame',
    	default_value='ego_vehicle/rgb_front'
        ),
        launch.actions.IncludeLaunchDescription(
    		launch.launch_description_sources.PythonLaunchDescriptionSource(
        	os.path.join(get_package_share_directory(
   	 	'carla_ad_agent'), 'carla_ad_agent.launch.py')
	    ),
	    launch_arguments={
		'avoid_risk': launch.substitutions.LaunchConfiguration('avoid_risk'),
	    }.items()
	),
        launch.actions.IncludeLaunchDescription(
    		launch.launch_description_sources.PythonLaunchDescriptionSource(
        	os.path.join(get_package_share_directory(
   	 	'visual_robot_localization'), 'place_recognition.launch.py')
	    ),
	    launch_arguments={
                    'camera_topic': launch.substitutions.LaunchConfiguration('camera_topic'),
                    'pose_publish_topic': launch.substitutions.LaunchConfiguration('pose_publish_topic'),
                    'extractor_conf': launch.substitutions.LaunchConfiguration('global_extractor_name'),
                    'gallery_db_path': launch.substitutions.LaunchConfiguration('gallery_global_descriptor_path'),
                    'image_gallery_path': launch.substitutions.LaunchConfiguration('image_gallery_path'),
                    'localization_frequence': launch.substitutions.LaunchConfiguration('localization_frequence'),
                    'top_k_matches': launch.substitutions.LaunchConfiguration('top_k_matches'),
	    }.items()
	),
        launch_ros.actions.Node(
            package=package_name,
            executable='place_recognition_evaluator_node',
            output='screen',
            emulate_tty=True,
            parameters=[
        	{
                    'odometry_topic': launch.substitutions.LaunchConfiguration('odometry_topic')
                },
                {
                    'place_reg_pose_topic': launch.substitutions.LaunchConfiguration('pose_publish_topic')
                },
                {
                    'place_reg_markers_topic': launch.substitutions.LaunchConfiguration('place_reg_markers_topic')
                },
                {
                    'synchrony_diff': launch.substitutions.LaunchConfiguration('synchrony_diff')
                },
                {
                    'compensate_sensor_offset': launch.substitutions.LaunchConfiguration('compensate_sensor_offset')
                },
                {
                    'base_frame': launch.substitutions.LaunchConfiguration('base_frame')
                },
                {
                    'sensor_frame': launch.substitutions.LaunchConfiguration('sensor_frame')
                },
        	]
        )
        ])
    return ld
	
if __name__ == '__main__':
    generate_launch_description()
	
     
