# AMR Fleet Management System

A comprehensive fleet management system for Automated Guided Vehicles (AMRs) built with Flutter frontend, Node.js backend, and ROS2 integration.

## 🏗️ Architecture

```
┌─────────────────┐    WebSocket/HTTP   ┌─────────────────┐    ROS2    ┌─────────────────┐
│  Flutter App    │ ◄─────────────────► │  Node.js Server │ ◄────────► │  AMR Fleet      │
│  (Frontend)     │                     │  (Backend)      │            │  (ROS2 Nodes)   │
└─────────────────┘                     └─────────────────┘            └─────────────────┘
```

## 🚀 Features

### Frontend (Flutter)
- Real-time AMR monitoring dashboard
- WebSocket-based live updates
- Manual AMR control with joystick
- Mission planning and waypoint management
- Fleet statistics and analytics
- Dark/Light theme support
- Responsive design for mobile and tablet

### Backend (Node.js + ROS2)
- ROS2 integration with rclnodejs
- WebSocket server for real-time communication
- RESTful API for AMR control
- Modular architecture with separate concerns
- AMR discovery and network scanning
- Safety features and emergency stop
- Mission management and queue system

### AMR Integration
- Position and orientation tracking
- Battery monitoring
- Status updates (idle, moving, charging, error)
- Velocity control (linear and angular)
- Emergency stop capabilities
- Diagnostic information

## 📋 Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 (recommended)
- **ROS2**: jazzy
- **Node.js**: v16.0.0 or later
- **Python**: 3.8 or later
- **Flutter**: 3.0.0 or later

### Hardware Requirements
- Network connection to AMRs
- WiFi router for AMR communication
- Computer/server with minimum 4GB RAM

## 🛠️ Installation

### 1. ROS2 Setup

```bash
# Install ROS2 Humble (Ubuntu 22.04)
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional packages
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-pip
sudo rosdep init
rosdep update
```

### 2. Backend Setup

```bash
# Clone repository
git clone <your-repository-url>
cd AMR-fleet-management/backend

# Install Node.js dependencies
npm install

# Install Python dependencies for ROS2
pip3 install rclpy

# Environment configuration
cp .env.example .env
# Edit .env file with your configuration

# Set ROS2 environment
export ROS_DOMAIN_ID=0
export ROS_NAMESPACE=/fleet
```

### 3. Frontend Setup

```bash
# Navigate to frontend directory
cd ../frontend

# Install Flutter dependencies
flutter pub get

# Configure WebSocket URL in services/websocket_service.dart
# Update serverUrl to your backend server IP
```

## ⚙️ Configuration

### Backend Configuration

Edit the `.env` file:

```bash
# Basic configuration
NODE_ENV=development
PORT=3000
ROS_DOMAIN_ID=0

# AMR Configuration
AMR_IDS=AMR_001,AMR_002,AMR_003

# Network
CORS_ORIGIN=*
```

### AMR Network Configuration

Ensure all AMRs are connected to the same WiFi network:

```bash
# Example network configuration
Network: AMR_Fleet_WiFi
IP Range: 192.168.1.100-200
Gateway: 192.168.1.1
```

### ROS2 Topic Configuration

Default topic structure for each AMR:
```
/AMR_001/pose          # Position updates
/AMR_001/status        # Status updates  
/AMR_001/battery_state # Battery information
/AMR_001/cmd_vel       # Velocity commands
/AMR_001/mission       # Mission commands
```

## 🚀 Running the System

### 1. Start ROS2 Environment

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Set domain ID (must match backend configuration)
export ROS_DOMAIN_ID=0

# Verify ROS2 is working
ros2 topic list
```

### 2. Start Backend Server

```bash
cd backend
npm start

# For development (with auto-reload)
npm run dev
```

Expected output:
```
🚀 AMR Fleet Management Server running on port 3000
📡 WebSocket server ready for connections
✅ ROS2 initialization complete
📤 Publishers created for 3 AMRs
📥 Subscribers created for 3 AMRs
```

### 3. Start Flutter App

```bash
cd frontend

# For mobile development
flutter run

# For web development
flutter run -d chrome

# For desktop development
flutter run -d linux
```

## 📱 Using the Application

### Dashboard Features

1. **Connection Status**: Shows real-time connection to backend
2. **AMR Selection**: Dropdown to select which AMR to monitor
3. **AMR Information Card**: 
   - Current status (Active, Idle, Charging, Error)
   - Battery percentage and charging status
   - Current position coordinates
   - Speed information
   - Active mission progress

4. **Quick Actions**:
   - **Start**: Navigate to control page
   - **Stop**: Send stop command to AMR
   - **E-Stop**: Emergency stop for selected AMR
   - **Emergency Stop All**: Stop all AMRs immediately

5. **Navigation Menu**:
   - **View Map**: See AMR positions on map
   - **Fleet Status**: Analytics and statistics
   - **Settings**: Configuration options
   - **Refresh**: Reconnect to server

### Control Interface

- **Virtual Joystick**: Manual control of AMR movement
- **Direction Buttons**: Precise directional control
- **Speed Control**: Adjust movement speed
- **Emergency Stop**: Quick access emergency stop

### Mission Planning

- **Waypoint Addition**: Add GPS coordinates for AMR navigation
- **Mission Assignment**: Send waypoint sequences to AMRs
- **Priority Settings**: Set mission priority levels
- **Progress Monitoring**: Track mission completion

## 🔧 API Reference

### REST API Endpoints

#### AMR Management
```http
GET    /api/AMRs                    # Get all AMRs
GET    /api/AMRs/:AMRId             # Get specific AMR
POST   /api/AMRs/:AMRId/command     # Send command
POST   /api/AMRs/:AMRId/move        # Move with velocity
POST   /api/AMRs/:AMRId/stop        # Stop AMR
POST   /api/AMRs/:AMRId/emergency-stop  # Emergency stop
POST   /api/AMRs/:AMRId/mission     # Assign mission
POST   /api/AMRs/:AMRId/goto        # Go to position
POST   /api/AMRs/:AMRId/mode        # Set mode
```

#### Fleet Operations
```http
GET    /api/fleet/stats             # Fleet statistics
POST   /api/fleet/emergency-stop    # Stop all AMRs
GET    /api/health                  # System health
GET    /api/discover                # Discover AMRs
```

### WebSocket Events

#### Client → Server
```javascript
socket.emit('get_AMR_status', AMRId);
socket.emit('send_command', {AMRId, command, params});
socket.emit('assign_mission', {AMRId, waypoints, priority});
socket.emit('emergency_stop', AMRId);
socket.emit('emergency_stop_all');
```

#### Server → Client
```javascript
socket.on('AMR_data', (AMRList) => {});
socket.on('AMR_position_update', (update) => {});
socket.on('AMR_status_update', (update) => {});
socket.on('AMR_battery_update', (update) => {});
socket.on('command_response', (response) => {});
```

## 🐛 Troubleshooting

### Common Issues

#### 1. Backend won't start
```bash
# Check ROS2 environment
echo $ROS_DISTRO
source /opt/ros/humble/setup.bash

# Check Node.js version
node --version  # Should be 16+

# Check dependencies
npm install
```

#### 2. No AMRs detected
```bash
# Check ROS2 topics
ros2 topic list | grep AMR

# Check network connectivity
ping 192.168.1.101  # Your AMR IP

# Check ROS2 domain ID
echo $ROS_DOMAIN_ID
```

#### 3. Flutter app can't connect
```bash
# Verify backend is running
curl http://localhost:3000/health

# Check IP address in Flutter code
# Update serverUrl in websocket_service.dart

# Check network connectivity
ping [backend-server-ip]
```

#### 4. WebSocket connection fails
- Ensure backend server is running
- Check firewall settings (port 3000)
- Verify CORS configuration
- Check network connectivity between devices

### Debug Commands

```bash
# Check ROS2 communication
ros2 topic echo /AMR_001/pose
ros2 topic pub /AMR_001/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Backend logs
npm run dev  # Shows detailed logs

# Flutter logs
flutter logs
```

## 📊 Monitoring & Analytics

### Built-in Metrics
- AMR connection status
- Battery levels and charging status
- Position tracking and movement history
- Mission completion rates
- Error frequency and types
- System performance metrics

### Health Checks
```bash
# System health
curl http://localhost:3000/health

# AMR discovery
curl http://localhost:3000/api/discover

# Fleet statistics
curl http://localhost:3000/api/fleet/stats
```

## 🔒 Security Considerations

### Network Security
- Use WPA3 encryption for WiFi network
- Implement VPN for remote access
- Regular security updates

### API Security
- Enable authentication in production
- Use HTTPS in production
- Implement rate limiting
- API key authentication

### ROS2 Security
- Use ROS2 security features (SROS2)
- Limit topic access permissions
- Network segmentation

## 📈 Performance Optimization

### Backend Optimization
- Use clustering for multiple CPU cores
- Implement Redis for session management
- Database optimization (if using databases)
- Message queue for high-traffic scenarios

### Frontend Optimization
- Implement WebSocket connection pooling
- Use state management (Provider/Bloc)
- Optimize widget rebuilds
- Image and asset optimization

## 🤝 Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

- **Documentation**: Check this README and inline code comments
- **Issues**: Report bugs in GitHub Issues
- **Questions**: Use GitHub Discussions
- **Email**: [agrawalvidit656@gmail.com]

## 🗺️ Roadmap

### Phase 1 (Current)
- ✅ Basic AMR control and monitoring
- ✅ WebSocket communication
- ✅ Flutter dashboard
- ✅ ROS2 integration

### Phase 2 (Planned)
- [ ] Advanced mission planning
- [ ] Map visualization
- [ ] Fleet optimization algorithms
- [ ] Database integration
- [ ] User authentication

### Phase 3 (Future)
- [ ] Machine learning integration
- [ ] Predictive maintenance
- [ ] Advanced analytics
- [ ] Multi-site management
- [ ] Mobile app enhancements

---

**Happy AMR Managing! 🤖🚀**
