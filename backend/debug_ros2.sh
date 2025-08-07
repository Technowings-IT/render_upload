#!/bin/bash
# ==========================================
# ROS2 Debug and Testing Script
# ==========================================

echo "🔧 ROS2 Debug and Testing Script"
echo "================================="
echo "Timestamp: $(date)"
echo "User: $(whoami)"
echo "Working Directory: $(pwd)"
echo ""

# ==========================================
# 1. ENVIRONMENT CHECK
# ==========================================
echo "🌍 CHECKING ROS2 ENVIRONMENT..."
echo "--------------------------------"

# Check if ROS2 is installed
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 command found"
    ros2 --version
else
    echo "❌ ROS2 command not found"
    echo "Please install ROS2 or check your installation"
    exit 1
fi

# Source environment files
echo ""
echo "📦 Sourcing ROS2 environment files..."
source /opt/ros/jazzy/setup.bash
if [ -f /fleet-management-system/ros_ws/install/local_setup.bash ]; then
    source /fleet-management-system/ros_ws/install/local_setup.bash
    echo "✅ Sourced workspace setup"
else
    echo "⚠️ Workspace setup not found: /fleet-management-system/ros_ws/install/local_setup.bash"
fi

# Check environment variables
echo ""
echo "🔍 Environment Variables:"
echo "ROS_DISTRO: ${ROS_DISTRO:-'NOT SET'}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-'0 (default)'}"
echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-'NOT SET'}"
echo "AMENT_PREFIX_PATH: ${AMENT_PREFIX_PATH:-'NOT SET'}"

# ==========================================
# 2. PACKAGE CHECK
# ==========================================
echo ""
echo "📦 CHECKING REQUIRED PACKAGES..."
echo "--------------------------------"

required_packages=("nav2_bringup" "slam_toolbox" "AMR")

for package in "${required_packages[@]}"; do
    if ros2 pkg list | grep -q "^$package$"; then
        echo "✅ Package found: $package"
    else
        echo "❌ Package NOT found: $package"
    fi
done

# ==========================================
# 3. WORKSPACE CHECK
# ==========================================
echo ""
echo "🏗️ CHECKING WORKSPACE..."
echo "------------------------"

workspace_dir="/fleet-management-system/ros_ws"
if [ -d "$workspace_dir" ]; then
    echo "✅ Workspace directory exists: $workspace_dir"
    
    # Check for build artifacts
    if [ -d "$workspace_dir/install" ]; then
        echo "✅ Install directory exists"
    else
        echo "❌ Install directory missing - workspace not built?"
    fi
    
    if [ -d "$workspace_dir/build" ]; then
        echo "✅ Build directory exists"
    else
        echo "⚠️ Build directory missing"
    fi
    
    if [ -d "$workspace_dir/src" ]; then
        echo "✅ Source directory exists"
        echo "   Packages in src:"
        find "$workspace_dir/src" -name "package.xml" -exec dirname {} \; | sed 's|.*/||' | sort | sed 's/^/     - /'
    else
        echo "❌ Source directory missing"
    fi
else
    echo "❌ Workspace directory not found: $workspace_dir"
fi

# ==========================================
# 4. LAUNCH FILE CHECK
# ==========================================
echo ""
echo "🚀 CHECKING LAUNCH FILES..."
echo "---------------------------"

launch_files=(
    "AMR ros2_control_robot.launch.py"
    "nav2_bringup bringup_launch.py"
    "slam_toolbox online_async_launch.py"
)

for launch_cmd in "${launch_files[@]}"; do
    if ros2 launch $launch_cmd --show-args &>/dev/null; then
        echo "✅ Launch file accessible: $launch_cmd"
    else
        echo "❌ Launch file NOT accessible: $launch_cmd"
    fi
done

# ==========================================
# 5. MAP FILES CHECK
# ==========================================
echo ""
echo "🗺️ CHECKING MAP FILES..."
echo "------------------------"

map_dir="/home/piros/fleet-management-system/ros_ws/src/AMR/maps"
if [ -d "$map_dir" ]; then
    echo "✅ Map directory exists: $map_dir"
    
    # List map files
    echo "   Available maps:"
    for yaml_file in "$map_dir"/*.yaml; do
        if [ -f "$yaml_file" ]; then
            map_name=$(basename "$yaml_file" .yaml)
            pgm_file="$map_dir/$map_name.pgm"
            if [ -f "$pgm_file" ]; then
                echo "     ✅ $map_name (YAML + PGM)"
            else
                echo "     ⚠️ $map_name (YAML only, missing PGM)"
            fi
        fi
    done
    
    if ! ls "$map_dir"/*.yaml 1> /dev/null 2>&1; then
        echo "     ⚠️ No YAML map files found"
    fi
else
    echo "❌ Map directory not found: $map_dir"
fi

# ==========================================
# 6. CONFIG FILES CHECK
# ==========================================
echo ""
echo "⚙️ CHECKING CONFIG FILES..."
echo "---------------------------"

config_files=(
    "/home/piros/fleet-management-system/ros_ws/src/AMR/config/nav2_params1.yaml"
    "/home/piros/fleet-management-system/ros_ws/src/AMR/config/mapper_params_online_async.yaml"
)

for config_file in "${config_files[@]}"; do
    if [ -f "$config_file" ]; then
        echo "✅ Config file exists: $(basename "$config_file")"
    else
        echo "❌ Config file missing: $config_file"
    fi
done

# ==========================================
# 7. PERMISSIONS CHECK
# ==========================================
echo ""
echo "🔐 CHECKING PERMISSIONS..."
echo "-------------------------"

# Check if we can write to log directories
log_dirs=(
    "/tmp"
    "$HOME/.ros"
    "/home/piros/fleet-management-system/storage/logs"
)

for log_dir in "${log_dirs[@]}"; do
    if [ -d "$log_dir" ] && [ -w "$log_dir" ]; then
        echo "✅ Can write to: $log_dir"
    elif [ -d "$log_dir" ]; then
        echo "⚠️ Directory exists but not writable: $log_dir"
    else
        echo "⚠️ Directory doesn't exist: $log_dir"
    fi
done

# ==========================================
# 8. RUNNING PROCESSES CHECK
# ==========================================
echo ""
echo "🔄 CHECKING RUNNING ROS2 PROCESSES..."
echo "------------------------------------"

if pgrep -f "ros2" > /dev/null; then
    echo "🔄 Active ROS2 processes found:"
    pgrep -af "ros2" | sed 's/^/     /'
else
    echo "✅ No ROS2 processes currently running"
fi

# Check for specific nodes
echo ""
echo "🔍 Checking for specific ROS2 nodes..."
if timeout 5 ros2 node list &>/dev/null; then
    echo "✅ ROS2 daemon is responding"
    nodes=$(ros2 node list 2>/dev/null)
    if [ -n "$nodes" ]; then
        echo "   Active nodes:"
        echo "$nodes" | sed 's/^/     /'
    else
        echo "   No nodes currently running"
    fi
else
    echo "⚠️ ROS2 daemon not responding or no nodes running"
fi

# ==========================================
# 9. NETWORK CHECK
# ==========================================
echo ""
echo "🌐 CHECKING NETWORK CONFIGURATION..."
echo "-----------------------------------"

echo "Network interfaces:"
ip addr show | grep -E "^[0-9]+:|inet " | sed 's/^/   /'

echo ""
echo "Checking multicast (for ROS2 discovery):"
if command -v ss &> /dev/null; then
    multicast_sockets=$(ss -u -a | grep -c 224.0.0 || echo "0")
    echo "   Multicast sockets: $multicast_sockets"
else
    echo "   ss command not available, cannot check multicast"
fi

# ==========================================
# 10. QUICK FUNCTIONALITY TEST
# ==========================================
echo ""
echo "🧪 QUICK FUNCTIONALITY TEST..."
echo "-----------------------------"

echo "Testing basic ROS2 commands..."

# Test ros2 topic list
if timeout 3 ros2 topic list &>/dev/null; then
    echo "✅ ros2 topic list - OK"
else
    echo "⚠️ ros2 topic list - Timeout or error"
fi

# Test ros2 node list
if timeout 3 ros2 node list &>/dev/null; then
    echo "✅ ros2 node list - OK"
else
    echo "⚠️ ros2 node list - Timeout or error"
fi

# ==========================================
# 11. RECOMMENDATIONS
# ==========================================
echo ""
echo "💡 RECOMMENDATIONS AND NEXT STEPS..."
echo "======================================"

# Provide specific recommendations based on what we found
recommendations=()

if ! command -v ros2 &> /dev/null; then
    recommendations+=("Install ROS2 Jazzy or check installation")
fi

if [ ! -d "/fleet-management-system/ros_ws/install" ]; then
    recommendations+=("Build your workspace: cd /fleet-management-system/ros_ws && colcon build")
fi

if ! ros2 pkg list | grep -q "AMR"; then
    recommendations+=("The 'AMR' package is not found - check your workspace build")
fi

if [ ! -d "/home/piros/fleet-management-system/ros_ws/src/AMR/maps" ]; then
    recommendations+=("Create maps directory and add map files")
fi

if [ ${#recommendations[@]} -eq 0 ]; then
    echo "✅ No critical issues found! Your ROS2 setup looks good."
    echo ""
    echo "🚀 You can now try running:"
    echo "   Backend: node app.js"
    echo "   Frontend: Send WebSocket commands to start SLAM/Navigation"
else
    echo "⚠️ Issues found that need attention:"
    printf '   - %s\n' "${recommendations[@]}"
fi

echo ""
echo "📝 For debugging script execution, check logs in:"
echo "   - /home/piros/fleet-management-system/storage/logs/"
echo "   - ~/.ros/log/"
echo ""
echo "🔧 Debug completed at $(date)"