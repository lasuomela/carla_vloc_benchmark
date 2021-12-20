#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/opt/carla/setup.bash"
source "/opt/carla-ros-bridge/install/setup.bash"
source "/opt/carla_vloc_benchmark/install/setup.bash"
source "/opt/visual_robot_localization/install/setup.bash"
exec "$@"

