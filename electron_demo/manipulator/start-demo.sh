#!/bin/bash

# Script to start the Two-Joint Manipulator Demo with ROS2 environment

echo "🤖 Starting Two-Joint Manipulator Demo..."

# Source ROS2 environment
echo "📁 Sourcing ROS2 environment..."
source /home/minggang/Download/ros2-linux/setup.bash

# Verify ROS2 is sourced
echo "✅ ROS_DISTRO: $ROS_DISTRO"

# Start the Electron demo
echo "🚀 Starting manipulator demo..."
echo "🎮 Use the control panel to interact with the robot arm"
npm start