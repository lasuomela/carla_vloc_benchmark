#!/bin/bash
SCRIPT=$(readlink -f "$0")
SCRIPT_DIR=$(dirname "$SCRIPT")
CARLA_VERSION=$(cat ${SCRIPT_DIR}/../third_party/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION)

docker run \
  -u carla \
  --privileged \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  --net=host \
  carlasim/carla:$CARLA_VERSION \
  ./CarlaUE4.sh

