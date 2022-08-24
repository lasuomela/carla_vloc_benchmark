import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='planner_odometry_topic',
            default_value='/odometry/filtered'
        ),
        launch.actions.DeclareLaunchArgument(
        name='ekf_config_path',
        default_value='/opt/carla_vloc_benchmark/src/carla_visual_navigation_agent/config/ekf.yaml'
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
                    'odometry_topic': launch.substitutions.LaunchConfiguration('planner_odometry_topic')
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
        package='carla_visual_navigation_agent',
        executable='localization_sensor_synchronizer',
        name='localization_sensor_synchronizer',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            },
            {
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            },
            ],
	   ),
    launch_ros.actions.Node(
        package='carla_visual_navigation_agent',
        executable='crash_monitor_node',
        name='crash_monitor_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            },
            {
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }],
            ),
    launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            },
            launch.substitutions.LaunchConfiguration('ekf_config_path')
            ],
        ),
    launch_ros.actions.Node(
        package='carla_visual_navigation_agent',
        executable='wheel_odometry_sensor',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': True
            },
            {
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            },
        ]),
    launch_ros.actions.Node(
        package='carla_visual_navigation_agent',
        executable='travelled_distance_sensor',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': True
            },
            {
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            },
        ]),
    launch_ros.actions.Node(
        package='carla_visual_navigation_agent',
        executable='path_tracking_sensor',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': True
            },
            {
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            },
        ]),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
