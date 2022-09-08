#!/bin/bash
# NOTE: you shouldn't call this script, you should source it. This script
# assumes that you've created a Catkin worspace somewhere that points back
# to this ROS package.

. "${HOME}"/.bashrc

# cd to this dir (so that we can source it from anywhere).
cd "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" > /dev/null

# Figure out what ROS distribution we're running.
IFS='/' read -r -a tokens <<< `find /opt/ros -name env.sh`
DISTRO=${tokens[3]}

# Setup the base ROS environment.
. /opt/ros/${DISTRO}/setup.bash

# Load this project's environment.
CATKIN_WS=../../mock_rover_ws
. ${CATKIN_WS}/devel/setup.bash

# Activate our virtualenv.
#. ~/venv/cmd_vel_filter/bin/activate

export ROS_MASTER_URI=http://127.0.0.1:11311
