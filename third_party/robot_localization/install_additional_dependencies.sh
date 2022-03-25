#!/bin/bash
# ROS distro from command line argument
ROS_DISTRO=$1
apt-get update && apt-get install -y \
	ros-${ROS_DISTRO}-diagnostic-updater \
      	libgeographic-dev \
      	geographiclib-doc \
      	geographiclib-tools \
      	ros-${ROS_DISTRO}-geographic-msgs
