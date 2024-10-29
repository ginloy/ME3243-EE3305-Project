#!/bin/bash

# Navigate to the workspace directory (optional, if you want to make sure the script runs from a specific directory)
cd "$(dirname "$0")" || exit 1

# Launches project
. install/setup.bash
ros2 launch ee3305_bringup project.launch.py
