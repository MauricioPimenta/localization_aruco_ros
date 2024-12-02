#!/bin/bash

# Set the SSH_CONFIG environment variable to point to your custom SSH config file
export SSH_CONFIG=~/Documents/LIMO_CBA/ssh2Limo/limo_ssh_config

# Check if arguments are passed
if [ "$#" -lt 2 ]; then
	echo "Usage: $0 <package> <launch_file> [args]"
	exit 1
fi

# Get the package and launch file from the arguments
PACKAGE=$1
LAUNCH_FILE=$2

# Shift the arguments so that $@ contains only the remaining arguments
shift 2

LIMO_ROS_SOURCE_PATH=/opt/ros/melodic/setub.bash
LIMO_WS_SOURCE_PATH=~/agilex_ws/devel/setup.bash
# Set the ROSLAUNCH_SSH environment variable to point to the SSH config file
export ROSLAUNCH_SSH="ssh -F $SSH_CONFIG -t 'source $LIMO_ROS_SOURCE_PATH && source $LIMO_WS_SOURCE_PATH && exec'"

# Run the roslaunch command with the environment variable set
roslaunch $PACKAGE $LAUNCH_FILE "$@"