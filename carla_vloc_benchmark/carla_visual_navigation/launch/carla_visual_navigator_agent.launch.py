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
	name='pose_publish_topic',
    	default_value='/carla/ego_vehicle/visual_pose_estimate'
        ),
        launch.actions.DeclareLaunchArgument(
    	name='camera_topic',
    	default_value='/carla/ego_vehicle/rgb_front/image'
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
    	name='global_extractor_name',
    	default_value='netvlad'
        ),
	launch.actions.DeclareLaunchArgument(
    	name='local_extractor_name',
    	default_value='superpoint_aachen'
        ),
	launch.actions.DeclareLaunchArgument(
    	name='local_matcher_name',
    	default_value='superglue'
        ),
        launch.actions.DeclareLaunchArgument(
    	name='gallery_global_descriptor_path',
    	default_value='/image-gallery/example_dir/outputs/netvlad+superpoint_aachen+superglue/global-feats-netvlad.h5'
        ),
        launch.actions.DeclareLaunchArgument(
    	name='gallery_local_descriptor_path',
    	default_value='/image-gallery/example_dir/outputs/netvlad+superpoint_aachen+superglue/feats-superpoint-n4096-r1024.h5'
        ),
        launch.actions.DeclareLaunchArgument(
    	name='image_gallery_path',
    	default_value='/image-gallery/example_dir/'
        ),
        launch.actions.DeclareLaunchArgument(
    	name='gallery_sfm_path',
    	default_value='/image-gallery/example_dir/outputs/netvlad+superpoint_aachen+superglue/sfm_netvlad+superpoint_aachen+superglue'
        ),
        launch.actions.DeclareLaunchArgument(
    	name='compensate_sensor_offset',
    	default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
    	name='localization_frequence',
    	default_value='2.0'
        ),
        launch.actions.DeclareLaunchArgument(
    	name='top_k_matches',
    	default_value='3'
        ),
        launch.actions.DeclareLaunchArgument(
    	name='ransac_thresh',
    	default_value='12'
        ),
        launch.actions.DeclareLaunchArgument(
    	name='odometry_topic',
    	default_value='/carla/ego_vehicle/odometry'
        ),
        launch.actions.DeclareLaunchArgument(
    	name='true_pose_publish_topic',
    	default_value='/carla/ego_vehicle/true_pose'
        ),
        launch.actions.IncludeLaunchDescription(
    		launch.launch_description_sources.PythonLaunchDescriptionSource(
        	os.path.join(get_package_share_directory(
   	 	'carla_visual_navigation_agent'), 'carla_ad_agent.launch.py')
	    ),
	    launch_arguments={
		'avoid_risk': launch.substitutions.LaunchConfiguration('avoid_risk'),
		'visual_pose_topic': launch.substitutions.LaunchConfiguration('pose_publish_topic'),
	    }.items()
	),
        launch.actions.IncludeLaunchDescription(
		launch.launch_description_sources.PythonLaunchDescriptionSource(
		os.path.join(get_package_share_directory(
 		'visual_robot_localization'), 'visual_pose_estimator.launch.py')
	    ),
	    launch_arguments={
		'pose_publish_topic': launch.substitutions.LaunchConfiguration('pose_publish_topic'),
		'camera_topic': launch.substitutions.LaunchConfiguration('camera_topic'),
		'base_frame': launch.substitutions.LaunchConfiguration('base_frame'),
            	'global_extractor_name': launch.substitutions.LaunchConfiguration('global_extractor_name'),
     		'local_extractor_name': launch.substitutions.LaunchConfiguration('local_extractor_name'),
            	'local_matcher_name': launch.substitutions.LaunchConfiguration('local_matcher_name'),
		'gallery_global_descriptor_path': launch.substitutions.LaunchConfiguration('gallery_global_descriptor_path'),
		'gallery_local_descriptor_path': launch.substitutions.LaunchConfiguration('gallery_local_descriptor_path'),
		'image_gallery_path': launch.substitutions.LaunchConfiguration('image_gallery_path'),
		'gallery_sfm_path': launch.substitutions.LaunchConfiguration('gallery_sfm_path'),
		'compensate_sensor_offset': launch.substitutions.LaunchConfiguration('compensate_sensor_offset'),
		'localization_frequence': launch.substitutions.LaunchConfiguration('localization_frequence'),
		'top_k_matches': launch.substitutions.LaunchConfiguration('top_k_matches'),
		'ransac_thresh': launch.substitutions.LaunchConfiguration('ransac_thresh')
	    }.items()
	),
	launch_ros.actions.Node(
            package=package_name,
            executable='filtered_visual_localizer_evaluator_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'odometry_topic': launch.substitutions.LaunchConfiguration('odometry_topic')
                },
                {
                    'true_pose_publish_topic': launch.substitutions.LaunchConfiguration('true_pose_publish_topic')
                },
                {
                    'visual_pose_topic': '/carla/ego_vehicle/best_vloc_estimate'
                }
            ]),
    	launch_ros.actions.Node(
            package=package_name,
            executable='travelled_distance_sensor',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': True
                }
            ]),
        launch_ros.actions.Node(
            package=package_name,
            executable='path_tracking_sensor',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': True
                }
            ]),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{'use_sim_time': True}, '/opt/carla_vloc_benchmark/src/carla_visual_navigation_agent/config/ekf.yaml'],
           ),
        launch_ros.actions.Node(
            package='carla_visual_navigation',
            executable='vehicle_reinit_node',
            name='vehicle_reinit_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        launch_ros.actions.Node(
            package='carla_visual_navigation',
            executable='crash_monitor_node',
            name='crash_monitor_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
