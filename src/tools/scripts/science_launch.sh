#!/usr/bin/env bash
export ROS_DOMAIN_ID=3
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WORKSPACE_DIR="$( realpath "$SCRIPT_DIR/../../../../.." )"
echo "Workspace directory: $WORKSPACE_DIR"
source "$WORKSPACE_DIR/install/setup.bash"
ros2 launch science_bringup athena_science.launch.py
