#!/bin/bash
# launch.sh - Main launch script for AGV Fleet Management System

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
BACKEND_DIR="agv-fleet-backend"
FRONTEND_DIR="agv_fleet_management"
DEFAULT_ROS_DOMAIN_ID=0

print_banner() {
    echo -e "${BLUE}"
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║                 AGV Fleet Management System                   ║"
    echo "║                        Launch Script                          ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_usage() {
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  setup           Setup the entire system (first-time installation)"
    echo "  start           Start all services (backend + frontend)"
    echo "  stop            Stop all services"
    echo "  backend         Start only backend server"
    echo "  frontend        Start only frontend application"
    echo "  agv-sim         Start AGV simulator"
    echo "  status          Check system status"
    echo "  logs            Show system logs"
    echo "  clean           Clean build artifacts and logs"
    echo "  help            Show this help message"
    echo ""
    echo "Options:"
    echo "  --domain-id ID  Set ROS2 domain ID (default: $DEFAULT_ROS_DOMAIN_ID)"
    echo "  --dev           Run in development mode"
    echo "  --docker        Use Docker containers"
    echo "  --verbose       Enable verbose output"
}

log() {
    echo -e "${GREEN}[$(date +'%Y-%m-%d %H:%M:%S')] $1${NC}"
}

warn() {
    echo -e "${YELLOW}[$(date +'%Y-%m-%d %H:%M:%S')] WARNING: $1${NC}"
}

error() {
    echo -e "${RED}[$(date +'%Y-%m-%d %H:%M:%S')] ERROR: $1${NC}"
    exit 1
}

check_dependencies() {
    log "Checking system dependencies..."
    
    # Check ROS2
    if ! command -v ros2 &> /dev/null; then
        error "ROS2 not found. Please install ROS2 Jazzy."
    fi
    
    # Check Node.js
    if ! command -v node &> /dev/null; then
        error "Node.js not found. Please install Node.js 16+."
    fi
    
    # Check Flutter (optional)
    if ! command -v flutter &> /dev/null; then
        warn "Flutter not found. Frontend features will be limited."
    fi
    
    # Check Docker (if using docker mode)
    if [[ "$USE_DOCKER" == "true" ]] && ! command -v docker &> /dev/null; then
        error "Docker not found. Please install Docker."
    fi
    
    log "Dependencies check completed ✓"
}

setup_system() {
    log "Setting up AGV Fleet Management System..."
    
    check_dependencies
    
    # Setup backend
    if [ -d "$BACKEND_DIR" ]; then
        log "Setting up backend..."
        cd "$BACKEND_DIR"
        
        # Source ROS2
        source /opt/ros/jazzy/setup.bash
        
        # Install dependencies
        npm install
        
        # Build rclnodejs
        log "Building rclnodejs (this may take a while)..."
        npx rclnodejs-cli build
        
        # Create storage directories
        mkdir -p storage/{maps,logs,uploads}
        
        # Copy example config
        if [ ! -f ".env" ] && [ -f ".env.example" ]; then
            cp .env.example .env
            warn "Please configure .env file with your settings"
        fi
        
        cd ..
    else
        warn "Backend directory '$BACKEND_DIR' not found"
    fi
    
    # Setup frontend
    if [ -d "$FRONTEND_DIR" ]; then
        log "Setting up frontend..."
        cd "$FRONTEND_DIR"
        
        # Get Flutter dependencies
        flutter pub get
        
        # Run code generation
        flutter pub run build_runner build --delete-conflicting-outputs
        
        cd ..
    else
        warn "Frontend directory '$FRONTEND_DIR' not found"
    fi
    
    log "Setup completed successfully! ✓"
    log "Run '$0 start' to launch the system"
}

start_backend() {
    log "Starting backend server..."
    
    if [ ! -d "$BACKEND_DIR" ]; then
        error "Backend directory '$BACKEND_DIR' not found"
    fi
    
    cd "$BACKEND_DIR"
    
    # Source ROS2
    source /opt/ros/jazzy/setup.bash
    
    # Set ROS domain ID
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE
    
    if [[ "$USE_DOCKER" == "true" ]]; then
        log "Starting backend with Docker..."
        if [[ "$DEV_MODE" == "true" ]]; then
            docker-compose -f docker-compose.dev.yml up -d agv-backend-dev
        else
            docker-compose up -d agv-backend
        fi
    else
        # Check if already running
        if pgrep -f "node.*app.js" > /dev/null; then
            warn "Backend appears to be already running"
            return 0
        fi
        
        if [[ "$DEV_MODE" == "true" ]]; then
            npm run dev &
        else
            npm start &
        fi
        
        # Save PID
        echo $! > backend.pid
    fi
    
    cd ..
    log "Backend server started ✓"
}

start_frontend() {
    log "Starting frontend application..."
    
    if [ ! -d "$FRONTEND_DIR" ]; then
        error "Frontend directory '$FRONTEND_DIR' not found"
    fi
    
    cd "$FRONTEND_DIR"
    
    # Check if already running
    if pgrep -f "flutter.*run" > /dev/null; then
        warn "Frontend appears to be already running"
        cd ..
        return 0
    fi
    
    if [[ "$DEV_MODE" == "true" ]]; then
        flutter run -d chrome &
    else
        flutter run --release -d chrome &
    fi
    
    # Save PID
    echo $! > frontend.pid
    
    cd ..
    log "Frontend application started ✓"
}

start_agv_simulator() {
    log "Starting AGV simulator..."
    
    # Source ROS2
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE
    
    # Start TurtleBot3 simulator as AGV simulator
    if command -v ros2 launch turtlebot3_gazebo &> /dev/null; then
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
        echo $! > agv_sim.pid
        log "AGV simulator started ✓"
    else
        warn "TurtleBot3 simulator not found. Install with:"
        warn "sudo apt install ros-jazzy-turtlebot3*"
    fi
}

stop_services() {
    log "Stopping all services..."
    
    if [[ "$USE_DOCKER" == "true" ]]; then
        cd "$BACKEND_DIR" 2>/dev/null || true
        docker-compose down
        cd ..
    else
        # Stop backend
        if [ -f "$BACKEND_DIR/backend.pid" ]; then
            kill $(cat "$BACKEND_DIR/backend.pid") 2>/dev/null || true
            rm "$BACKEND_DIR/backend.pid"
        fi
        
        # Stop frontend
        if [ -f "$FRONTEND_DIR/frontend.pid" ]; then
            kill $(cat "$FRONTEND_DIR/frontend.pid") 2>/dev/null || true
            rm "$FRONTEND_DIR/frontend.pid"
        fi
        
        # Stop AGV simulator
        if [ -f "agv_sim.pid" ]; then
            kill $(cat "agv_sim.pid") 2>/dev/null || true
            rm "agv_sim.pid"
        fi
        
        # Kill any remaining processes
        pkill -f "node.*app.js" 2>/dev/null || true
        pkill -f "flutter.*run" 2>/dev/null || true
    fi
    
    log "All services stopped ✓"
}

show_status() {
    log "System Status:"
    echo ""
    
    # Backend status
    if pgrep -f "node.*app.js" > /dev/null || docker ps | grep agv-backend > /dev/null 2>&1; then
        echo -e "Backend Server:     ${GREEN}Running ✓${NC}"
    else
        echo -e "Backend Server:     ${RED}Stopped ✗${NC}"
    fi
    
    # Frontend status
    if pgrep -f "flutter.*run" > /dev/null; then
        echo -e "Frontend App:       ${GREEN}Running ✓${NC}"
    else
        echo -e "Frontend App:       ${RED}Stopped ✗${NC}"
    fi
    
    # ROS2 status
    if command -v ros2 &> /dev/null; then
        echo -e "ROS2:               ${GREEN}Available ✓${NC}"
        if ros2 node list 2>/dev/null | grep -q fleet_management; then
            echo -e "ROS2 Node:          ${GREEN}Connected ✓${NC}"
        else
            echo -e "ROS2 Node:          ${YELLOW}Not connected${NC}"
        fi
    else
        echo -e "ROS2:               ${RED}Not available ✗${NC}"
    fi
    
    # Docker status (if applicable)
    if [[ "$USE_DOCKER" == "true" ]]; then
        if docker ps | grep agv- > /dev/null 2>&1; then
            echo -e "Docker Services:    ${GREEN}Running ✓${NC}"
        else
            echo -e "Docker Services:    ${RED}Stopped ✗${NC}"
        fi
    fi
    
    echo ""
    echo "ROS Domain ID: $ROS_DOMAIN_ID_VALUE"
    echo "Mode: $([ "$DEV_MODE" == "true" ] && echo "Development" || echo "Production")"
}

show_logs() {
    log "Showing system logs..."
    
    if [[ "$USE_DOCKER" == "true" ]]; then
        cd "$BACKEND_DIR" 2>/dev/null || true
        docker-compose logs -f
    else
        # Show backend logs
        if [ -f "$BACKEND_DIR/logs/app.log" ]; then
            echo -e "${BLUE}=== Backend Logs ===${NC}"
            tail -n 50 "$BACKEND_DIR/logs/app.log"
        fi
        
        # Show ROS2 logs
        echo -e "${BLUE}=== ROS2 Logs ===${NC}"
        ros2 log view 2>/dev/null || echo "No ROS2 logs available"
    fi
}

clean_system() {
    log "Cleaning build artifacts and logs..."
    
    # Clean backend
    if [ -d "$BACKEND_DIR" ]; then
        cd "$BACKEND_DIR"
        rm -rf node_modules/.cache
        rm -rf logs/*
        rm -f *.pid
        cd ..
    fi
    
    # Clean frontend
    if [ -d "$FRONTEND_DIR" ]; then
        cd "$FRONTEND_DIR"
        flutter clean
        rm -f *.pid
        cd ..
    fi
    
    # Clean Docker
    if [[ "$USE_DOCKER" == "true" ]]; then
        docker system prune -f
    fi
    
    log "Cleanup completed ✓"
}

# Parse command line arguments
COMMAND=""
ROS_DOMAIN_ID_VALUE=$DEFAULT_ROS_DOMAIN_ID
DEV_MODE="false"
USE_DOCKER="false"
VERBOSE="false"

while [[ $# -gt 0 ]]; do
    case $1 in
        setup|start|stop|backend|frontend|agv-sim|status|logs|clean|help)
            COMMAND="$1"
            shift
            ;;
        --domain-id)
            ROS_DOMAIN_ID_VALUE="$2"
            shift 2
            ;;
        --dev)
            DEV_MODE="true"
            shift
            ;;
        --docker)
            USE_DOCKER="true"
            shift
            ;;
        --verbose)
            VERBOSE="true"
            set -x
            shift
            ;;
        *)
            error "Unknown option: $1"
            ;;
    esac
done

# Main execution
print_banner

case $COMMAND in
    setup)
        setup_system
        ;;
    start)
        start_backend
        sleep 3
        start_frontend
        log "System started successfully! ✓"
        log "Access the web interface at: http://localhost:3000"
        ;;
    stop)
        stop_services
        ;;
    backend)
        start_backend
        ;;
    frontend)
        start_frontend
        ;;
    agv-sim)
        start_agv_simulator
        ;;
    status)
        show_status
        ;;
    logs)
        show_logs
        ;;
    clean)
        clean_system
        ;;
    help|"")
        print_usage
        ;;
    *)
        error "Unknown command: $COMMAND"
        ;;
esac

---

# Additional utility scripts

# agv_monitor.sh - System monitoring script
#!/bin/bash

# AGV System Monitor
INTERVAL=5

while true; do
    clear
    echo "AGV Fleet Management - System Monitor"
    echo "======================================"
    echo "Time: $(date)"
    echo ""
    
    # System resources
    echo "System Resources:"
    echo "CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
    echo "Memory: $(free | grep Mem | awk '{printf("%.1f%%", $3/$2 * 100.0)}')"
    echo "Disk: $(df -h / | awk 'NR==2{printf "%s", $5}')"
    echo ""
    
    # ROS2 topics
    echo "Active ROS2 Topics:"
    ros2 topic list 2>/dev/null | head -10
    echo ""
    
    # Connected devices
    echo "Network devices:"
    nmap -sn 192.168.1.0/24 2>/dev/null | grep "Nmap scan report" | wc -l
    echo ""
    
    sleep $INTERVAL
done

---

# agv_deploy.sh - Deployment script
#!/bin/bash

# AGV Fleet Management Deployment Script

DEPLOY_USER="agv"
DEPLOY_HOST="your-server.com"
DEPLOY_PATH="/opt/agv-fleet"

log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"
}

deploy_backend() {
    log "Deploying backend to $DEPLOY_HOST..."
    
    # Build and upload
    tar -czf agv-backend.tar.gz agv-fleet-backend/
    scp agv-backend.tar.gz $DEPLOY_USER@$DEPLOY_HOST:$DEPLOY_PATH/
    
    # Remote deployment
    ssh $DEPLOY_USER@$DEPLOY_HOST << EOF
        cd $DEPLOY_PATH
        tar -xzf agv-backend.tar.gz
        cd agv-fleet-backend
        npm install --production
        pm2 restart agv-backend || pm2 start app.js --name agv-backend
EOF
    
    log "Backend deployment completed"
}

deploy_frontend() {
    log "Building and deploying frontend..."
    
    cd agv_fleet_management
    flutter build web
    
    # Upload to web server
    rsync -avz build/web/ $DEPLOY_USER@$DEPLOY_HOST:$DEPLOY_PATH/web/
    
    log "Frontend deployment completed"
}

case $1 in
    backend)
        deploy_backend
        ;;
    frontend)
        deploy_frontend
        ;;
    all)
        deploy_backend
        deploy_frontend
        ;;
    *)
        echo "Usage: $0 {backend|frontend|all}"
        exit 1
        ;;
esac