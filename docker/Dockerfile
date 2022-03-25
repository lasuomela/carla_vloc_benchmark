ARG ROS_DISTRO
ARG BASE_IMAGE
FROM $BASE_IMAGE

# Scenario runner and dependencies
COPY ./third_party/scenario_runner /opt/third_party/scenario_runner
WORKDIR /opt/third_party/scenario_runner
RUN /bin/bash -c "pip3 install -r requirements.txt"
ENV SCENARIO_RUNNER_PATH="/opt/third_party/scenario_runner/"

# robot_localization and dependencies
COPY ./third_party/robot_localization /opt/third_party/robot_localization
WORKDIR /opt/third_party/robot_localization
RUN /bin/bash -c "chmod +x install_additional_dependencies.sh; \
		   bash install_additional_dependencies.sh ${ROS_DISTRO}"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; \
		  colcon build"

# General package dependencies
COPY ./requirements.txt /opt/
COPY ./install_dependencies.sh /opt/
WORKDIR /opt/
RUN /bin/bash -c "chmod +x install_dependencies.sh; \
		   bash install_dependencies.sh"

# Build the package itself
COPY ./carla_vloc_benchmark /opt/carla_vloc_benchmark/src
WORKDIR /opt/carla_vloc_benchmark/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; \
		  colcon build" 

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /
