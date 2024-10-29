#!/bin/bash

# Navigate to the workspace directory (optional, if you want to make sure the script runs from a specific directory)
cd "$(dirname "$0")" || exit 1

# Removes the built file and rebuild using bd.sh
rm -rf build install log
. bd.sh
