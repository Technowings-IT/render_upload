#!/bin/bash

# ROS2 Map Saver Script
# Usage: ./save_map.sh [map_name] [directory]
# Default map name: "map"
# Default directory: "/home/piros/my_map"

MAP_NAME=${1:-"map"}
MAP_DIR=${2:-"/home/piros/my_map"}

echo "🗺️ Starting ROS2 map save process..."
echo "📂 Directory: $MAP_DIR"
echo "📝 Map name: $MAP_NAME"

# Create directory if it doesn't exist
mkdir -p "$MAP_DIR"

# Change to the target directory
cd "$MAP_DIR" || {
    echo "❌ Error: Cannot access directory $MAP_DIR"
    exit 1
}

# Source ROS2 environment
echo "🔧 Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Check if nav2_map_server is available
if ! ros2 pkg list | grep -q nav2_map_server; then
    echo "❌ Error: nav2_map_server package not found"
    exit 1
fi

# Execute the map saver
echo "💾 Saving map..."
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME"

# Check if files were created
if [[ -f "${MAP_NAME}.yaml" && -f "${MAP_NAME}.pgm" ]]; then
    echo "✅ Map saved successfully!"
    echo "📄 YAML file: ${MAP_DIR}/${MAP_NAME}.yaml"
    echo "🖼️  PGM file: ${MAP_DIR}/${MAP_NAME}.pgm"
    
    # Display file sizes
    echo "📊 File sizes:"
    ls -lh "${MAP_NAME}.yaml" "${MAP_NAME}.pgm"
    
    exit 0
else
    echo "❌ Error: Map files were not created"
    exit 1
fi