#!/bin/bash
echo "🚀 Starting AMR Fleet Management Backend (Production)"

# Check if ROS2 is available
if command -v ros2 &> /dev/null; then
    echo "🤖 ROS2 environment detected"
    source /opt/ros/${ROS_DISTRO}/setup.bash 2>/dev/null || echo "⚠️  Could not source ROS2 setup"
fi

# Start with PM2
npm run pm2:start
