#!/bin/bash

set -e

WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

echo "Building task offloading system..."
echo "Workspace: $WORKSPACE_DIR"

cd "$WORKSPACE_DIR"

# Detect OS and source ROS 2 accordingly
if [[ "$OSTYPE" == "darwin"* ]]; then
  # macOS
  if [ -f "/opt/homebrew/opt/ros-humble-desktop/setup.bash" ]; then
    echo "Sourcing ROS 2 Humble from Homebrew..."
    source /opt/homebrew/opt/ros-humble-desktop/setup.bash
  elif [ -f "$HOME/ros2_humble/install/setup.bash" ]; then
    echo "Sourcing ROS 2 Humble from local installation..."
    source $HOME/ros2_humble/install/setup.bash
  else
    echo "ERROR: ROS 2 Humble not found on macOS!"
    echo "Please install ROS 2 Humble via Homebrew or from source."
    echo ""
    echo "Option 1: Install via Homebrew (recommended):"
    echo "  brew install ros-humble-desktop"
    echo ""
    echo "Option 2: Build from source:"
    echo "  See https://docs.ros.org/en/humble/Installation/Alternatives/macOS-Development-Setup.html"
    exit 1
  fi
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
  # Linux (Ubuntu)
  if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Sourcing ROS 2 Humble from system installation..."
    source /opt/ros/humble/setup.bash
  else
    echo "ERROR: ROS 2 Humble not found!"
    echo "Please install ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html"
    exit 1
  fi
else
  echo "ERROR: Unsupported OS: $OSTYPE"
  exit 1
fi

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Build completed successfully!"
echo ""
echo "To use the system, source the workspace:"
echo "  source $WORKSPACE_DIR/install/setup.bash"