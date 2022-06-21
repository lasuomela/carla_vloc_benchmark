#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CARLA_VERSION=$(cat ${SCRIPT_DIR}/../third_party/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION)
docker pull carlasim/carla:$CARLA_VERSION

# Update nvidia repository key
# Fix for the carla 0.9.12 image missing libraries: libomp5 xdg-user-dirs xdg-utils
docker run -u root -i -t --privileged -v /tmp/.X11-unix:/tmp/.X11-unix --entrypoint /bin/bash carlasim/carla:0.9.12 -c "apt-key del 7fa2af80 && apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub && apt update && apt install libomp5 xdg-user-dirs xdg-utils"
CONTAINER_ID=`docker ps -lq`
docker commit $CONTAINER_ID carlasim/carla:0.9.12
