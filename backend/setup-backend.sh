#!/bin/bash

# Backend Setup Script for AGV Fleet Management System

echo "🚀 Setting up AGV Fleet Management Backend..."

# Create directory structure
echo "📁 Creating directory structure..."

# Create main directories
mkdir -p utils
mkdir -p storage/maps
mkdir -p storage/logs  
mkdir -p storage/uploads
mkdir -p storage/backups

# Create log files
touch storage/logs/app.log
touch storage/logs/error.log
touch storage/logs/ros.log

# Create initial data files if they don't exist
echo '[]' > storage/devices.json
echo '{}' > storage/orders.json

# Set permissions
chmod +x setup-backend.sh
chmod -R 755 storage/

echo "✅ Directory structure created"

# Install dependencies
echo "📦 Installing dependencies..."
npm install

# Check ROS2 environment
echo "🤖 Checking ROS2 environment..."
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 found: $(ros2 --version)"
    
    # Check if ROS_DOMAIN_ID is set
    if [ -z "$ROS_DOMAIN_ID" ]; then
        echo "⚠️  ROS_DOMAIN_ID not set. Setting to 0"
        export ROS_DOMAIN_ID=0
        echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
    fi
    
    # Test ROS2 topics
    echo "📡 Checking ROS2 topics..."
    timeout 5 ros2 topic list > storage/logs/ros_topics.log 2>&1
    if [ $? -eq 0 ]; then
        echo "✅ ROS2 topics accessible"
        cat storage/logs/ros_topics.log
    else
        echo "⚠️  ROS2 topics not accessible (running in simulation mode)"
    fi
else
    echo "⚠️  ROS2 not found. Backend will run in simulation mode."
fi

# Create environment file
echo "🔧 Creating environment configuration..."
cat > .env << EOF
# AGV Fleet Management Environment Configuration
NODE_ENV=development
PORT=3000
HOST=0.0.0.0

# ROS2 Configuration
ROS_DOMAIN_ID=0
ENABLE_SIMULATION=false
DEBUG_ROS=false

# Security
ENABLE_AUTH=false
JWT_SECRET=your-secret-key-change-this

# Logging
LOG_LEVEL=info

# Performance
MAX_PAYLOAD_SIZE=50mb
EOF

echo "✅ Environment file created"

# Create ecosystem config for PM2 (optional)
echo "🔄 Creating PM2 ecosystem config..."
cat > ecosystem.config.js << EOF
module.exports = {
  apps: [{
    name: 'agv-fleet-backend',
    script: 'app.js',
    instances: 1,
    autorestart: true,
    watch: false,
    max_memory_restart: '1G',
    env: {
      NODE_ENV: 'development',
      PORT: 3000
    },
    env_production: {
      NODE_ENV: 'production',
      PORT: 3000
    },
    error_file: './storage/logs/pm2-error.log',
    out_file: './storage/logs/pm2-out.log',
    log_file: './storage/logs/pm2-combined.log',
    time: true
  }]
};
EOF

echo "✅ PM2 ecosystem config created"

# Create startup scripts
echo "📜 Creating startup scripts..."

# Development startup
cat > start-dev.sh << EOF
#!/bin/bash
echo "🚀 Starting AGV Fleet Management Backend (Development)"

# Check if ROS2 is available
if command -v ros2 &> /dev/null; then
    echo "🤖 ROS2 environment detected"
    source /opt/ros/\${ROS_DISTRO}/setup.bash 2>/dev/null || echo "⚠️  Could not source ROS2 setup"
fi

# Start with nodemon for development
npm run dev
EOF

# Production startup
cat > start-prod.sh << EOF
#!/bin/bash
echo "🚀 Starting AGV Fleet Management Backend (Production)"

# Check if ROS2 is available
if command -v ros2 &> /dev/null; then
    echo "🤖 ROS2 environment detected"
    source /opt/ros/\${ROS_DISTRO}/setup.bash 2>/dev/null || echo "⚠️  Could not source ROS2 setup"
fi

# Start with PM2
npm run pm2:start
EOF

chmod +x start-dev.sh start-prod.sh

echo "✅ Startup scripts created"

# Create systemd service file (optional)
echo "🔧 Creating systemd service template..."
cat > agv-fleet-backend.service << EOF
[Unit]
Description=AGV Fleet Management Backend
After=network.target

[Service]
Type=simple
User=\$USER
WorkingDirectory=/path/to/your/agv-fleet-backend
Environment=NODE_ENV=production
Environment=PORT=3000
Environment=ROS_DOMAIN_ID=0
ExecStart=/usr/bin/node app.js
Restart=always
RestartSec=10

# Logging
StandardOutput=append:/path/to/your/agv-fleet-backend/storage/logs/systemd.log
StandardError=append:/path/to/your/agv-fleet-backend/storage/logs/systemd-error.log

[Install]
WantedBy=multi-user.target
EOF

echo "✅ Systemd service template created (edit paths before using)"

# Create health check script
echo "🏥 Creating health check script..."
cat > health-check.sh << EOF
#!/bin/bash

echo "🏥 AGV Fleet Management Backend Health Check"

# Check if server is running
if curl -f http://localhost:3000/api/health >/dev/null 2>&1; then
    echo "✅ Backend server is healthy"
    
    # Get detailed status
    curl -s http://localhost:3000/api/health | jq '.' 2>/dev/null || echo "Response received but jq not available for formatting"
else
    echo "❌ Backend server is not responding"
    exit 1
fi

# Check ROS2 topics if available
if command -v ros2 &> /dev/null; then
    echo ""
    echo "🤖 ROS2 Topics:"
    timeout 3 ros2 topic list 2>/dev/null || echo "⚠️  ROS2 topics not accessible"
fi

# Check storage
echo ""
echo "💾 Storage Status:"
echo "Maps: $(ls -1 storage/maps/ 2>/dev/null | wc -l) files"
echo "Logs: $(ls -1 storage/logs/ 2>/dev/null | wc -l) files"
echo "Storage size: $(du -sh storage/ 2>/dev/null | cut -f1)"
EOF

chmod +x health-check.sh

echo "✅ Health check script created"

# Final instructions
echo ""
echo "🎉 Backend setup completed!"
echo ""
echo "Next steps:"
echo "1. Review and edit .env file with your settings"
echo "2. If using systemd, edit agv-fleet-backend.service with correct paths"
echo "3. Start development server: ./start-dev.sh"
echo "4. Or start production server: ./start-prod.sh"
echo "5. Check health: ./health-check.sh"
echo ""
echo "📖 Available commands:"
echo "  npm start          - Start production server"
echo "  npm run dev        - Start development server with auto-reload"
echo "  npm run pm2:start  - Start with PM2"
echo "  npm run health-check - Check server health"
echo ""
echo "🌐 After starting, visit:"
echo "  http://localhost:3000/api/health - Health check"
echo "  http://localhost:3000/api/devices - Connected devices"
echo ""