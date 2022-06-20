import os
import glob
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

# String with message to publish on topic /carla/available/scenarios
# The .xosc files found are shown in the rviz scenario runner GUI 
def get_scenarios(package_name):
    scenario_dir = os.path.join(get_package_share_directory(package_name), 'config')
    scenario_paths = glob.glob(scenario_dir+'/*.xosc')
    scenario_names = [os.path.basename(path).replace('.xosc', '') for path in scenario_paths]
    ros_topic_msg_string = "{ 'scenarios': ["
    for i, (s_path, s_name) in enumerate(zip(scenario_paths, scenario_names)):
        if i != 0:
            ros_topic_msg_string = ros_topic_msg_string + ","
        ros_topic_msg_string = ros_topic_msg_string + "{ " + "'name': '{}', 'scenario_file': '{}'".format(s_name, s_path) + "}"
    ros_topic_msg_string = ros_topic_msg_string + "] }"
    return ros_topic_msg_string
    
def get_scenarios_from_path(scenario_dir):
    scenario_paths = glob.glob(scenario_dir+'/*.xosc')
    scenario_names = [os.path.basename(path).replace('.xosc', '') for path in scenario_paths]
    ros_topic_msg_string = "{ 'scenarios': ["
    for i, (s_path, s_name) in enumerate(zip(scenario_paths, scenario_names)):
        if i != 0:
            ros_topic_msg_string = ros_topic_msg_string + ","
        ros_topic_msg_string = ros_topic_msg_string + "{ " + "'name': '{}', 'scenario_file': '{}'".format(s_name, s_path) + "}"
    ros_topic_msg_string = ros_topic_msg_string + "] }"
    return ros_topic_msg_string

package_name = "carla_visual_navigation"
gui_scenarios_path = get_scenarios(package_name)
rviz2_config_path = "config/visual_navigation_ros2.rviz"
objects_config_path = os.path.join(get_package_share_directory(package_name), "config/objects.json")


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
        # Specify the town to load as argument for ros2 launch (town:=$town_name$)
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='None'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='5'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='scenario_runner_path',
            default_value=os.environ.get('SCENARIO_RUNNER_PATH')
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_config',
            default_value=objects_config_path
        ),
        launch_ros.actions.Node(
            package='carla_twist_to_control',
            executable='carla_twist_to_control',
            name='carla_twist_to_control',
            remappings=[
                (
                    ["/carla/",
                        launch.substitutions.LaunchConfiguration('role_name'), "/vehicle_control_cmd"],
                    ["/carla/",
                        launch.substitutions.LaunchConfiguration('role_name'), "/vehicle_control_cmd_manual"]
                )
            ],
            parameters=[
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                }
            ]
        ),
        launch.actions.ExecuteProcess(
            cmd=["ros2", "topic", "pub", "/carla/available_scenarios",
                 "carla_ros_scenario_runner_types/CarlaScenarioList", get_scenarios_from_path(gui_scenarios_path)],
            name='topic_pub_vailable_scenarios',
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'synchronous_mode': 'True', # make sure the time labels for collected images are correct
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
            ),
            launch_arguments={
                'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_config'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'spawn_point_ego_vehicle': {"x": 2.0, "y": 0.0, "z": 2.0, "roll": 0.0, "pitch": 0.0, "yaw": 270.0} # The default value causes bridge to crash -> inject some value
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_scenario_runner'), 'carla_ros_scenario_runner.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'scenario_runner_path': launch.substitutions.LaunchConfiguration('scenario_runner_path'),
                'wait_for_ego': 'True'
            }.items()
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            remappings=[
                (
                    "carla/ego_vehicle/spectator_pose",
                    "/carla/ego_vehicle/rgb_view/control/set_transform"
                )
            ],
            arguments=[
                '-d', os.path.join(get_package_share_directory(package_name), rviz2_config_path)],
            on_exit=launch.actions.Shutdown()
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
