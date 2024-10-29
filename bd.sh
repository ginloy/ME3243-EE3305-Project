#!/bin/bash

# Navigate to the workspace directory (optional, if you want to make sure the script runs from a specific directory)
cd "$(dirname "$0")" || exit 1

# Run the colcon build command with --symlink-install
colcon build --symlink-install
