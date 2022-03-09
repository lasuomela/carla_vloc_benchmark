#!/bin/bash
SCRIPT=$(readlink -f "$0")
SCRIPT_DIR=$(dirname "$SCRIPT")
CARLA_VERSION=$(cat ${SCRIPT_DIR}/../third_party/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION)

# The 0.9.12 CLI has changes (-vulkan -RenderOffScreen). The prebuilt also has missing dependencies, addressed in build-carla.sh
if [[ "$CARLA_VERSION" == "0.9.12" ]]; then
docker run \
 -u carla \
 -p 2000-2002:2000-2002 \
 --net=host \
 -i \
 -t \
 --gpus all \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 --entrypoint /bin/bash carlasim/carla:0.9.12 -c "unset SDL_VIDEODRIVER && ./CarlaUE4.sh -vulkan -RenderOffScreen -nosound -quality-level=Epic /Game/Carla/Maps/Town10HD"
 
else

docker run \
 --rm \
 -u carla \
 -p 2000-2002:2000-2002 \
 --runtime=nvidia \
 -e SDL_VIDEODRIVER=offscreen \
 --gpus='all,"capabilities=graphics,utility,display,video,compute"'  \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -it \
 carlasim/carla:$CARLA_VERSION \
 ./CarlaUE4.sh -opengl -nosound -quality-level=Epic /Game/Carla/Maps/Town01
fi
