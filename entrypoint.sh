#!/bin/bash

# Get workspace path from command line argument or use default
WS_PATH=${1:-/root/ws_offboard}

echo "Using workspace path: $WS_PATH"

source /root/.bashrc

# Setup ROS environment
source /opt/ros/jazzy/setup.bash

# Change to workspace directory
cd $WS_PATH

# Build the workspace
colcon build

# Source the workspace setup
source $WS_PATH/install/local_setup.bash

# Launch the offboard relay
ros2 launch offboard_companion offboard_relay_sim.launch.py