ARG ROS_DISTRO
ARG BASE_IMAGE
FROM $BASE_IMAGE

# Scenario runner and dependencies
COPY ./third_party/scenario_runner /opt/third_party/scenario_runner
WORKDIR /opt/third_party/scenario_runner
RUN /bin/bash -c "pip3 install -r requirements.txt"
ENV SCENARIO_RUNNER_PATH="/opt/third_party/scenario_runner/"

# Required:
# source ros-bridge carla visual navigation
