#!/bin/bash
# Activate the ROS environment for our project, then kick off clion.

# EDIT THIS: set to your installation
CLION_DIR=clion
source ros_env.sh && /opt/${CLION_DIR}/bin/clion.sh &
