#!/bin/bash
# test_system.sh - Quick test script for AMR Fleet Management System

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log() {
    echo -e "${GREEN}[$(date +'%H:%M:%S')] $1${NC}"
}

info() {
    echo -e "${BLUE}[$(date +'%H:%M:%S')] $1${NC}"
}

warn() {
    echo -e "${YELLOW}[$(date +'%H:%M:%S')] $1${NC}"
}

error() {
    echo -e "${RED}[$(date +'%H:%M:%S')] ERROR: $1${NC}"
    exit 1
}

# Check if we're in the right directory
if [[ "$(basename "$PWD")" != "AMR-fleet-management" ]]; then
  error "Please run this script from the AMR-fleet-management root directory"
fi
# Function to check if a port is in use
check_port() {
    if netstat -tuln | grep -q ":$1 "; then
        return 0  # Port is in use
    else
        return 1  # Port is free
    fi
}

# Function to wait for a service to be ready
wait_for_service() {
    local url=$1
    local timeout=${2:-30}
    local count=0
    
    while [ $count -lt $timeout ]; do
        if curl -s "$url" > /dev/null 2>&1; then
            return 0
        fi
        sleep 1
        ((count++))
    done
    return 1
}

# Kill any existing processes
cleanup() {
    log "Cleaning up existing processes..."
    pkill -f "node.*app.js" 2>/dev/null || true
    pkill -f "flutter.*run" 2>/dev/null || true
    pkill -f "ros2.*topic.*pub" 2>/dev/null || true
    sleep 2
}

# Start mock AMR (ROS2 simulator)
start_mock_AMR() {
    log "Starting mock AMR simulator..."
    
    # Check if ROS2 is available
    if ! command -v ros2 &> /dev/null; then
        error "ROS2 not found. Please install ROS2 Jazzy first."
    fi
    
    # Source ROS2
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0
    
    # Start mock odometry publisher
    nohup ros2 topic pub /AMR_001/odom nav_msgs/msg/Odometry "{
        header: {
            stamp: {sec: 0, nanosec: 0},
            frame_id: 'map'
        },
        pose: {
            pose: {
                position: {x: 1.0, y: 2.0, z: 0.0},
                orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
            }
        },
        twist: {
            twist: {
                linear: {x: 0.1, y: 0.0, z: 0.0},
                angular: {x: 0.0, y: 0.0, z: 0.1}
            }
        }
    }" --rate 2 > /dev/null 2>&1 &
    
    # Start mock battery publisher
    nohup ros2 topic pub /AMR_001/battery_state sensor_msgs/msg/BatteryState "{
        voltage: 12.5,
        current: 2.0,
        charge: 45.0,
        capacity: 50.0,
        percentage: 85.0,
        power_supply_status: 2,
        power_supply_health: 1,
        power_supply_technology: 1,
        present: true
    }" --rate 0.5 > /dev/null 2>&1 &
    
    # Start mock robot state publisher
    nohup ros2 topic pub /AMR_001/robot_state std_msgs/msg/String "{
        data: '{\"mode\": \"auto\", \"status\": \"ok\", \"errors\": []}'
    }" --rate 0.2 > /dev/null 2>&1 &
    
    info "Mock AMR started with topics:"
    info "  - /AMR_001/odom (odometry data)"
    info "  - /AMR_001/battery_state (battery info)"
    info "  - /AMR_001/robot_state (robot status)"
    
    sleep 2
}

# Start backend server
start_backend() {
    log "Starting backend server..."
    
    cd AMR-fleet-backend
    
    # Check if node_modules exists
    if [[ ! -d "node_modules" ]]; then
        warn "node_modules not found. Running npm install..."
        npm install
    fi
    
    # Create .env if it doesn't exist
    if [[ ! -f ".env" ]]; then
        log "Creating .env file..."
        cat > .env << EOF
NODE_ENV=development
PORT=3000
ROS_DOMAIN_ID=0
LOG_LEVEL=info
MAX_LINEAR_SPEED=1.0
MAX_ANGULAR_SPEED=1.0
BATTERY_LOW_THRESHOLD=20
EOF
    fi
    
    # Source ROS2 for backend
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0
    
    # Start backend in background
    nohup npm run dev > ../backend.log 2>&1 &
    BACKEND_PID=$!
    echo $BACKEND_PID > backend.pid
    
    cd ..
    
    # Wait for backend to be ready
    info "Waiting for backend to start..."
    if wait_for_service "http://localhost:3000/api/control/health" 15; then
        log "Backend server started successfully (PID: $BACKEND_PID)"
    else
        error "Backend failed to start. Check backend.log for details."
    fi
}

# Start Flutter frontend
start_frontend() {
    log "Starting Flutter frontend..."
    
    cd AMR_fleet_management
    
    # Check if Flutter is available
    if ! command -v flutter &> /dev/null; then
        error "Flutter not found. Please install Flutter first."
    fi
    
    # Get dependencies if needed
    if [[ ! -d ".dart_tool" ]]; then
        log "Getting Flutter dependencies..."
        flutter pub get
    fi
    
    # Enable web platform
    flutter config --enable-web > /dev/null 2>&1 || true
    
    # Start Flutter web app
    info "Starting Flutter web application..."
    nohup flutter run -d chrome --web-port=8080 > ../frontend.log 2>&1 &
    FRONTEND_PID=$!
    echo $FRONTEND_PID > frontend.pid
    
    cd ..
    
    log "Flutter frontend starting (PID: $FRONTEND_PID)"
    info "Web app will open automatically in Chrome"
    info "If it doesn't open, go to: http://localhost:8080"
}

# Display system status
show_status() {
    echo ""
    log "=== SYSTEM STATUS ==="
    
    # Backend status
    if curl -s http://localhost:3000/api/control/health > /dev/null; then
        echo -e "✅ Backend Server: ${GREEN}Running${NC} (http://localhost:3000)"
    else
        echo -e "❌ Backend Server: ${RED}Not Running${NC}"
    fi
    
    # Frontend status
    if check_port 8080; then
        echo -e "✅ Flutter Frontend: ${GREEN}Running${NC} (http://localhost:8080)"
    else
        echo -e "❌ Flutter Frontend: ${RED}Not Running${NC}"
    fi
    
    # ROS2 topics
    if source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && ros2 topic list | grep -q AMR_001; then
        echo -e "✅ Mock AMR: ${GREEN}Running${NC}"
        echo "   ROS2 Topics:"
        source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && ros2 topic list | grep AMR_001 | sed 's/^/     /'
    else
        echo -e "❌ Mock AMR: ${RED}Not Running${NC}"
    fi
    
    echo ""
    log "=== NEXT STEPS ==="
    echo "1. Open the Flutter app in your browser: http://localhost:8080"
    echo "2. Go to 'Devices' tab and click '+' to add a device"
    echo "3. Use these settings:"
    echo "   - Device ID: AMR_001"
    echo "   - Device Name: Test AMR"
    echo "   - IP Address: 127.0.0.1"
    echo "4. Click 'Connect Device'"
    echo "5. Go to Dashboard to see your connected AMR"
    echo "6. Try the Control interface to test the joystick"
    echo ""
    echo "To stop the system: $0 stop"
    echo "To check logs: tail -f backend.log frontend.log"
}

# Stop all services
stop_services() {
    log "Stopping all services..."
    
    # Stop backend
    if [[ -f "AMR-fleet-backend/backend.pid" ]]; then
        kill $(cat AMR-fleet-backend/backend.pid) 2>/dev/null || true
        rm AMR-fleet-backend/backend.pid
    fi
    
    # Stop frontend
    if [[ -f "AMR_fleet_management/frontend.pid" ]]; then
        kill $(cat AMR_fleet_management/frontend.pid) 2>/dev/null || true
        rm AMR_fleet_management/frontend.pid
    fi
    
    # Stop all related processes
    cleanup
    
    log "All services stopped"
}

# Main script logic
case "${1:-start}" in
    start)
        log "Starting AMR Fleet Management System Test..."
        cleanup
        start_mock_AMR
        start_backend
        start_frontend
        sleep 5
        show_status
        ;;
    stop)
        stop_services
        ;;
    status)
        show_status
        ;;
    *)
        echo "Usage: $0 {start|stop|status}"
        exit 1
        ;;
esac