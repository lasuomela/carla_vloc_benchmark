#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPT_DIR=$(dirname "${SCRIPT}")

ROS_DISTRO="foxy"
IMAGE_NAME="carla-vloc-benchmark:0.0.1"

# Build carla-ros-bridge
cd ${SCRIPT_DIR}/../third_party/ros-bridge/docker
./build.sh

# Build visual_robot_localization
TEMP_IMAGE_NAME="carla_ros_bridge_vloc:0.0.1"
cd ${SCRIPT_DIR}/../visual_robot_localization/docker
./build.sh -b "carla-ros-bridge:$ROS_DISTRO" -t $TEMP_IMAGE_NAME

# Build the benchmark
cd ${SCRIPT_DIR}
docker build \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    --build-arg BASE_IMAGE=$TEMP_IMAGE_NAME \
    -t $IMAGE_NAME \
    -f Dockerfile ${SCRIPT_DIR}/..
