#!/bin/bash

# Run test generator

WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

# Check if workspace is built
if [ ! -d "$WORKSPACE_DIR/install" ]; then
  echo "ERROR: Workspace not built! Please run build.sh first."
  exit 1
fi

source "$WORKSPACE_DIR/install/setup.bash"

ros2 launch task_executor test_generator.launch.py "$@"