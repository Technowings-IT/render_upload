#!/bin/bash
# setup.sh - Automated setup script for AGV Fleet Management System

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
PROJECT_NAME="agv-fleet-management"
BACKEND_DIR="agv-fleet-backend"
FRONTEND_DIR="agv_fleet_management"
NODE_VERSION="18"
FLUTTER_CHANNEL="stable"
ROS_DISTRO="jazzy"

# System detection
OS=$(lsb_release -si 2>/dev/null || echo "Unknown")
OS_VERSION=$(lsb_release -sr 2>/dev/null || echo "Unknown")
ARCH=$(uname -m)

print_banner() {
    echo -e "${BLUE}"
    echo "╔═══════════════════════════════════════════════════════════════════════════════╗"
    echo "║                                                                               ║"
    echo "║                    AGV Fleet Management System                                ║"
    echo "║                         Automated Setup Script                                ║"
    echo "║                                                                               ║"
    echo "║  This script will install and configure the complete AGV fleet management     ║"
    echo "║  system including ROS2, Node.js, Flutter, and all dependencies.               ║"
    echo "║                                                                               ║"
    echo "╚═══════════════════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
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

info() {
    echo -e "${CYAN}[$(date +'%Y-%m-%d %H:%M:%S')] INFO: $1${NC}"
}

print_system_info() {
    echo -e "${PURPLE}System Information:${NC}"
    echo "OS: $OS $OS_VERSION"
    echo "Architecture: $ARCH"
    echo "User: $(whoami)"
    echo "Home: $HOME"
    echo "PWD: $(pwd)"
    echo ""
}

check_prerequisites() {
    log "Checking system prerequisites..."
    
    # Check if running as root
    if [[ $EUID -eq 0 ]]; then
        error "This script should not be run as root. Please run as a regular user."
    fi
    
    # Check Ubuntu version
    if [[ "$OS" != "Ubuntu" ]]; then
        warn "This script is designed for Ubuntu. Other distributions may not work correctly."
    fi
    
    if [[ "$OS_VERSION" < "22.04" ]]; then
        warn "Ubuntu 22.04 LTS or newer is recommended for ROS2 Jazzy support."
    fi
    
    # Check internet connection
    if ! ping -c 1 google.com &> /dev/null; then
        error "Internet connection required for installation."
    fi
    
    log "Prerequisites check completed ✓"
}

update_system() {
    log "Updating system packages..."
    
    sudo apt update
    sudo apt upgrade -y
    
    # Install essential packages
    sudo apt install -y \
        curl \
        wget \
        git \
        build-essential \
        software-properties-common \
        apt-transport-https \
        ca-certificates \
        gnupg \
        lsb-release \
        unzip \
        tar \
        python3 \
        python3-pip \
        python3-dev \
        cmake \
        pkg-config \
        libnss3-dev \
        libgconf-2-4 \
        libxss1 \
        libasound2-dev
    
    log "System update completed ✓"
}

install_ros2() {
    log "Installing ROS2 $ROS_DISTRO..."
    
    # Check if ROS2 is already installed
    if command -v ros2 &> /dev/null; then
        warn "ROS2 is already installed"
        return 0
    fi
    
    # Add ROS2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Add ROS2 repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Update package list
    sudo apt update
    
    # Install ROS2
    sudo apt install -y ros-$ROS_DISTRO-desktop
    
    # Install development tools
    sudo apt install -y ros-dev-tools
    
    # Install additional ROS2 packages for AGV
    sudo apt install -y \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup \
        ros-$ROS_DISTRO-robot-localization \
        ros-$ROS_DISTRO-slam-toolbox \
        ros-$ROS_DISTRO-tf2-tools \
        ros-$ROS_DISTRO-rviz2 \
        ros-$ROS_DISTRO-rqt*
    
    # Source ROS2 in bashrc
    if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    fi
    
    # Source for current session
    source /opt/ros/$ROS_DISTRO/setup.bash
    
    log "ROS2 $ROS_DISTRO installation completed ✓"
}

install_nodejs() {
    log "Installing Node.js $NODE_VERSION..."
    
    # Check if Node.js is already installed with correct version
    if command -v node &> /dev/null; then
        current_version=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
        if [[ "$current_version" -ge "$NODE_VERSION" ]]; then
            warn "Node.js v$current_version is already installed"
            return 0
        fi
    fi
    
    # Install Node.js from NodeSource
    curl -fsSL https://deb.nodesource.com/setup_${NODE_VERSION}.x | sudo -E bash -
    sudo apt-get install -y nodejs
    
    # Verify installation
    node_version=$(node --version)
    npm_version=$(npm --version)
    
    log "Node.js $node_version and npm $npm_version installed ✓"
}

install_flutter() {
    log "Installing Flutter..."
    
    # Check if Flutter is already installed
    if command -v flutter &> /dev/null; then
        warn "Flutter is already installed"
        flutter --version
        return 0
    fi
    
    # Download Flutter SDK
    cd $HOME
    if [[ ! -d "flutter" ]]; then
        git clone https://github.com/flutter/flutter.git -b $FLUTTER_CHANNEL
    fi
    
    # Add Flutter to PATH
    if ! grep -q 'export PATH="$PATH:$HOME/flutter/bin"' ~/.bashrc; then
        echo 'export PATH="$PATH:$HOME/flutter/bin"' >> ~/.bashrc
    fi
    
    # Export for current session
    export PATH="$PATH:$HOME/flutter/bin"
    
    # Run Flutter doctor
    flutter doctor --android-licenses || true
    flutter doctor
    
    log "Flutter installation completed ✓"
}

install_docker() {
    log "Installing Docker..."
    
    # Check if Docker is already installed
    if command -v docker &> /dev/null; then
        warn "Docker is already installed"
        return 0
    fi
    
    # Install Docker
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    
    # Add user to docker group
    sudo usermod -aG docker $USER
    
    # Install Docker Compose
    sudo curl -L "https://github.com/docker/compose/releases/latest/download/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    sudo chmod +x /usr/local/bin/docker-compose
    
    # Create docker group and add user
    sudo groupadd docker || true
    sudo usermod -aG docker $USER
    
    log "Docker installation completed ✓"
    warn "Please log out and back in to use Docker without sudo"
}

create_project_structure() {
    log "Creating project structure..."
    
    # Create main project directory
    mkdir -p $PROJECT_NAME
    cd $PROJECT_NAME
    
    # Create backend structure
    mkdir -p $BACKEND_DIR/{routes,websocket,ros/utils,middleware,models,services,utils,storage/{maps,logs,uploads},tests/{unit,integration,fixtures},bin,config}
    
    # Create scripts directory
    mkdir -p scripts
    
    # Create configs directory
    mkdir -p configs/{systemd,nginx,udev}
    
    # Create documentation directory
    mkdir -p docs
    
    log "Project structure created ✓"
}

setup_backend() {
    log "Setting up backend..."
    
    cd $BACKEND_DIR
    
    # Initialize npm project if package.json doesn't exist
    if [[ ! -f "package.json" ]]; then
        info "Creating package.json..."
        # Here you would copy the package.json content we created earlier
        cat > package.json << 'EOF'
{
  "name": "agv-fleet-backend",
  "version": "1.0.0",
  "description": "Backend server for AGV Fleet Management System with ROS2 integration",
  "main": "app.js",
  "scripts": {
    "start": "node app.js",
    "dev": "nodemon app.js",
    "setup": "npm install && npm run setup-ros && npm run create-dirs",
    "setup-ros": "npx rclnodejs-cli build",
    "create-dirs": "mkdir -p storage/maps storage/logs storage/uploads"
  },
  "dependencies": {
    "express": "^4.18.2",
    "cors": "^2.8.5",
    "body-parser": "^1.20.2",
    "ws": "^8.14.2",
    "rclnodejs": "^0.21.0",
    "helmet": "^7.0.0",
    "morgan": "^1.10.0",
    "dotenv": "^16.3.1"
  },
  "devDependencies": {
    "nodemon": "^3.0.1"
  }
}
EOF
    fi
    
    # Install dependencies
    npm install
    
    # Create .env file if it doesn't exist
    if [[ ! -f ".env" ]]; then
        cp .env.example .env 2>/dev/null || cat > .env << 'EOF'
NODE_ENV=development
PORT=3000
ROS_DOMAIN_ID=0
LOG_LEVEL=info
EOF
    fi
    
    # Build rclnodejs
    info "Building rclnodejs (this may take a while)..."
    source /opt/ros/$ROS_DISTRO/setup.bash
    npx rclnodejs-cli build || warn "rclnodejs build failed - you may need to build it manually"
    
    cd ..
    log "Backend setup completed ✓"
}

setup_frontend() {
    log "Setting up frontend..."
    
    # Initialize Flutter project if it doesn't exist
    if [[ ! -d "$FRONTEND_DIR" ]]; then
        flutter create $FRONTEND_DIR
    fi
    
    cd $FRONTEND_DIR
    
    # Get dependencies
    flutter pub get
    
    # Enable web support
    flutter config --enable-web
    
    cd ..
    log "Frontend setup completed ✓"
}

create_launch_script() {
    log "Creating launch script..."
    
    cat > launch.sh << 'EOF'
#!/bin/bash
# AGV Fleet Management System Launch Script

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

log() {
    echo -e "${GREEN}[$(date +'%H:%M:%S')] $1${NC}"
}

error() {
    echo -e "${RED}[$(date +'%H:%M:%S')] ERROR: $1${NC}"
    exit 1
}

case $1 in
    start)
        log "Starting AGV Fleet Management System..."
        
        # Start backend
        cd agv-fleet-backend
        source /opt/ros/jazzy/setup.bash
        npm start &
        BACKEND_PID=$!
        echo $BACKEND_PID > backend.pid
        cd ..
        
        # Start frontend (web)
        cd agv_fleet_management
        flutter run -d chrome &
        FRONTEND_PID=$!
        echo $FRONTEND_PID > frontend.pid
        cd ..
        
        log "System started! Backend PID: $BACKEND_PID, Frontend PID: $FRONTEND_PID"
        ;;
    stop)
        log "Stopping AGV Fleet Management System..."
        
        # Stop backend
        if [[ -f "agv-fleet-backend/backend.pid" ]]; then
            kill $(cat agv-fleet-backend/backend.pid) 2>/dev/null || true
            rm agv-fleet-backend/backend.pid
        fi
        
        # Stop frontend
        if [[ -f "agv_fleet_management/frontend.pid" ]]; then
            kill $(cat agv_fleet_management/frontend.pid) 2>/dev/null || true
            rm agv_fleet_management/frontend.pid
        fi
        
        log "System stopped"
        ;;
    status)
        echo "System Status:"
        if pgrep -f "node.*app.js" > /dev/null; then
            echo "Backend: Running"
        else
            echo "Backend: Stopped"
        fi
        
        if pgrep -f "flutter.*run" > /dev/null; then
            echo "Frontend: Running"
        else
            echo "Frontend: Stopped"
        fi
        ;;
    *)
        echo "Usage: $0 {start|stop|status}"
        exit 1
        ;;
esac
EOF
    
    chmod +x launch.sh
    log "Launch script created ✓"
}

install_additional_tools() {
    log "Installing additional development tools..."
    
    # Install PM2 for process management
    sudo npm install -g pm2
    
    # Install useful development tools
    sudo npm install -g nodemon eslint prettier
    
    # Install VS Code (optional)
    if ! command -v code &> /dev/null; then
        wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
        sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
        sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
        sudo apt update
        sudo apt install -y code
        
        # Install useful VS Code extensions
        code --install-extension dart-code.dart-code
        code --install-extension dart-code.flutter
        code --install-extension ms-vscode.cpptools
        code --install-extension ms-python.python
    fi
    
    log "Additional tools installed ✓"
}

create_desktop_entry() {
    log "Creating desktop entry..."
    
    cat > ~/.local/share/applications/agv-fleet-management.desktop << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=AGV Fleet Management
Comment=AGV Fleet Management System
Exec=/bin/bash -c "cd $(pwd) && ./launch.sh start"
Icon=applications-engineering
Terminal=true
Categories=Development;Engineering;
EOF
    
    log "Desktop entry created ✓"
}

run_tests() {
    log "Running system tests..."
    
    # Test ROS2
    if command -v ros2 &> /dev/null; then
        info "✓ ROS2 is available"
        source /opt/ros/$ROS_DISTRO/setup.bash
        ros2 doctor --report > /dev/null && info "✓ ROS2 environment is healthy"
    else
        warn "✗ ROS2 not found"
    fi
    
    # Test Node.js
    if command -v node &> /dev/null; then
        info "✓ Node.js $(node --version) is available"
    else
        warn "✗ Node.js not found"
    fi
    
    # Test Flutter
    if command -v flutter &> /dev/null; then
        info "✓ Flutter is available"
        flutter doctor --version > /dev/null && info "✓ Flutter environment is ready"
    else
        warn "✗ Flutter not found"
    fi
    
    # Test Docker
    if command -v docker &> /dev/null; then
        info "✓ Docker is available"
    else
        warn "✗ Docker not found"
    fi
    
    log "System tests completed ✓"
}

cleanup() {
    log "Cleaning up temporary files..."
    rm -f get-docker.sh
    log "Cleanup completed ✓"
}

print_completion_message() {
    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                    INSTALLATION COMPLETED!                    ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}🎉 AGV Fleet Management System has been successfully installed!${NC}"
    echo ""
    echo -e "${YELLOW}Next steps:${NC}"
    echo "1. Open a new terminal or run: source ~/.bashrc"
    echo "2. Navigate to the project directory: cd $PROJECT_NAME"
    echo "3. Start the system: ./launch.sh start"
    echo "4. Open your browser and go to: http://localhost:3000"
    echo ""
    echo -e "${YELLOW}Important notes:${NC}"
    echo "• Make sure to configure your AGV devices in the .env file"
    echo "• Set the correct ROS_DOMAIN_ID for your network"
    echo "• Check the documentation in the docs/ directory"
    echo "• For production deployment, see configs/ directory"
    echo ""
    echo -e "${CYAN}For support, visit: https://github.com/yourusername/agv-fleet-management${NC}"
    echo ""
}

# Main installation process
main() {
    print_banner
    print_system_info
    
    # Ask for confirmation
    echo -e "${YELLOW}This script will install:${NC}"
    echo "• ROS2 Jazzy"
    echo "• Node.js $NODE_VERSION"
    echo "• Flutter"
    echo "• Docker & Docker Compose"
    echo "• AGV Fleet Management System"
    echo ""
    read -p "Do you want to continue? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Installation cancelled."
        exit 0
    fi
    
    # Run installation steps
    check_prerequisites
    update_system
    install_ros2
    install_nodejs
    install_flutter
    install_docker
    create_project_structure
    setup_backend
    setup_frontend
    create_launch_script
    install_additional_tools
    create_desktop_entry
    run_tests
    cleanup
    print_completion_message
}

# Handle interrupts
trap 'echo -e "\n${RED}Installation interrupted!${NC}"; exit 1' INT TERM

# Run main function
main "$@"