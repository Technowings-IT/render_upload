// scripts/setup-raspberry-pi.js - Setup Raspberry Pi for AMR Map Deployment
const { Client } = require('ssh2');
const fs = require('fs').promises;
const path = require('path');

// Load environment variables
require('dotenv').config({ path: path.join(__dirname, '../.env') });

const SSH_CONFIG = {
    host: process.env.RASPBERRY_PI_HOST || '192.168.0.84',
    port: parseInt(process.env.RASPBERRY_PI_PORT) || 22,
    username: process.env.RASPBERRY_PI_USER || 'piros',
    password: process.env.RASPBERRY_PI_PASSWORD || 'piros',
};

console.log('🔧 Using SSH Config:');
console.log(`📡 Host: ${SSH_CONFIG.host}:${SSH_CONFIG.port}`);
console.log(`👤 User: ${SSH_CONFIG.username}`);
console.log('🔐 Password: [CONFIGURED]');

async function setupRaspberryPi() {
    console.log('🚀 Setting up Raspberry Pi for AMR Map Deployment...');
    console.log('🔧 Testing SSH connection to Raspberry Pi...');
    
    const conn = new Client();
    
    return new Promise((resolve, reject) => {
        conn.on('ready', async () => {
            console.log('✅ SSH Connected to Raspberry Pi');
            
            try {
                const homeDir = SSH_CONFIG.username === 'piros' ? '/home/piros' : '/home/' + SSH_CONFIG.username;
                
                // Step 1: Create necessary directories (no sudo needed for user home)
                console.log('📁 Creating directories...');
                await executeCommand(conn, `mkdir -p ${homeDir}/ros2_ws/src/nav2_bringup/maps`);
                await executeCommand(conn, `mkdir -p ${homeDir}/ros2_ws/logs`);
                await executeCommand(conn, `mkdir -p ${homeDir}/.AMR-fleet`);
                
                // Step 2: Update package lists and install packages (with proper sudo)
                console.log('📦 Installing required packages...');
                console.log('🔐 Note: This may require sudo password authentication...');
                
                try {
                    // Try to update packages first
                    await executeCommandWithSudo(conn, 'apt update', SSH_CONFIG.password);
                    await executeCommandWithSudo(conn, 'apt install -y inotify-tools jq python3-yaml', SSH_CONFIG.password);
                } catch (sudoError) {
                    console.log('⚠️ Package installation failed, continuing without system packages...');
                    console.log('   You may need to manually install: inotify-tools, jq, python3-yaml');
                }
                
                // Step 3: Copy map loading script
                console.log('📋 Installing map loading script...');
                const scriptPath = `${homeDir}/ros2_ws/load_map.sh`;
                
                // Create a basic map loading script
                console.log('⚠️ Using built-in map loading script...');
                const scriptContent = createDefaultMapScript(homeDir);
                
                await writeRemoteFile(conn, scriptPath, scriptContent);
                await executeCommand(conn, `chmod +x ${scriptPath}`);
                
                // Step 4: Create systemd service (optional, skip if sudo fails)
                console.log('⚙️ Attempting to create systemd service...');
                try {
                    await executeCommandWithSudo(conn, `${scriptPath} install-service`, SSH_CONFIG.password);
                } catch (serviceError) {
                    console.log('⚠️ Service installation failed, continuing without systemd service...');
                }
                
                // Step 5: Create configuration file
                console.log('🔧 Creating configuration...');
                const config = {
                    version: '1.0.0',
                    setupDate: new Date().toISOString(),
                    mapDirectory: `${homeDir}/ros2_ws/src/nav2_bringup/maps`,
                    rosWorkspace: `${homeDir}/ros2_ws`,
                    autoLoadEnabled: true,
                    backendHost: 'auto-detect',
                    username: SSH_CONFIG.username,
                    sshConfig: {
                        host: SSH_CONFIG.host,
                        port: SSH_CONFIG.port
                    }
                };
                
                await writeRemoteFile(
                    conn, 
                    `${homeDir}/.AMR-fleet/config.json`, 
                    JSON.stringify(config, null, 2)
                );
                
                // Step 6: Test ROS environment
                console.log('🧪 Testing ROS environment...');
                try {
                    await executeCommand(conn, `source ${homeDir}/ros2_ws/install/setup.bash && ros2 --version`);
                    console.log('✅ ROS2 environment OK');
                } catch (error) {
                    console.log('⚠️ ROS2 environment not fully configured (this is normal for fresh installs)');
                }
                
                // Step 7: Try to start the service (optional)
                console.log('🔄 Attempting to start AMR map loader service...');
                try {
                    await executeCommandWithSudo(conn, 'systemctl start AMR-map-loader.service', SSH_CONFIG.password);
                    await executeCommand(conn, 'systemctl status AMR-map-loader.service --no-pager --user || true');
                } catch (serviceError) {
                    console.log('⚠️ Service start failed, maps can still be loaded manually');
                }
                
                conn.end();
                
                console.log('🎉 Raspberry Pi setup completed successfully!');
                console.log('📋 Summary:');
                console.log(`   • Map directory: ${config.mapDirectory}`);
                console.log(`   • Script location: ${homeDir}/ros2_ws/load_map.sh`);
                console.log('   • Service: AMR-map-loader.service (may require manual setup)');
                console.log(`   • Config: ${homeDir}/.AMR-fleet/config.json`);
                console.log(`   • SSH Host: ${SSH_CONFIG.host}:${SSH_CONFIG.port}`);
                console.log(`   • SSH User: ${SSH_CONFIG.username}`);
                
                resolve({
                    success: true,
                    message: 'Raspberry Pi setup completed',
                    config: config
                });
                
            } catch (error) {
                conn.end();
                reject(error);
            }
        });
        
        conn.on('error', (error) => {
            reject(new Error(`SSH connection failed: ${error.message}`));
        });
        
        conn.connect(SSH_CONFIG);
    });
}

function executeCommand(conn, command) {
    return new Promise((resolve, reject) => {
        console.log(`🔧 Executing: ${command}`);
        
        conn.exec(command, (err, stream) => {
            if (err) {
                reject(err);
                return;
            }
            
            let stdout = '';
            let stderr = '';
            
            stream.on('close', (code) => {
                if (code === 0) {
                    resolve(stdout);
                } else {
                    reject(new Error(`Command failed with code ${code}: ${stderr}`));
                }
            });
            
            stream.on('data', (data) => {
                stdout += data.toString();
                const output = data.toString().trim();
                if (output.length > 0) {
                    console.log(`   ${output}`);
                }
            });
            
            stream.stderr.on('data', (data) => {
                stderr += data.toString();
                const error = data.toString().trim();
                if (error.length > 0) {
                    console.error(`   ERROR: ${error}`);
                }
            });
        });
    });
}

function executeCommandWithSudo(conn, command, password) {
    return new Promise((resolve, reject) => {
        const sudoCommand = `echo '${password}' | sudo -S ${command}`;
        console.log(`🔧 Executing with sudo: ${command}`);
        
        conn.exec(sudoCommand, (err, stream) => {
            if (err) {
                reject(err);
                return;
            }
            
            let stdout = '';
            let stderr = '';
            
            stream.on('close', (code) => {
                if (code === 0) {
                    resolve(stdout);
                } else {
                    reject(new Error(`Sudo command failed with code ${code}: ${stderr}`));
                }
            });
            
            stream.on('data', (data) => {
                stdout += data.toString();
                const output = data.toString().trim();
                // Filter out the password prompt
                if (!output.includes('[sudo] password') && output.length > 0) {
                    console.log(`   ${output}`);
                }
            });
            
            stream.stderr.on('data', (data) => {
                stderr += data.toString();
                const error = data.toString().trim();
                if (!error.includes('[sudo] password') && error.length > 0) {
                    console.error(`   ERROR: ${error}`);
                }
            });
        });
    });
}

function writeRemoteFile(conn, remotePath, content) {
    return new Promise((resolve, reject) => {
        conn.sftp((err, sftp) => {
            if (err) {
                reject(err);
                return;
            }
            
            const stream = sftp.createWriteStream(remotePath);
            
            stream.on('close', () => {
                console.log(`📝 Written file: ${remotePath}`);
                resolve();
            });
            
            stream.on('error', reject);
            
            stream.write(content);
            stream.end();
        });
    });
}

function createDefaultMapScript(homeDir) {
    return `#!/bin/bash
# AMR Fleet Map Loader Script
# Auto-generated by setup-raspberry-pi.js

MAP_DIR="${homeDir}/ros2_ws/src/nav2_bringup/maps"
ROS_WS="${homeDir}/ros2_ws"
CONFIG_FILE="${homeDir}/.AMR-fleet/config.json"
LOG_FILE="${homeDir}/.AMR-fleet/map-loader.log"

# Create log function
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

# Load map function
load_map() {
    local map_name="$1"
    
    if [ -z "$map_name" ]; then
        log "ERROR: No map name provided"
        exit 1
    fi
    
    local map_file="$MAP_DIR/$map_name.yaml"
    
    if [ ! -f "$map_file" ]; then
        log "ERROR: Map file not found: $map_file"
        exit 1
    fi
    
    log "Loading map: $map_name"
    
    # Source ROS environment
    if [ -f "$ROS_WS/install/setup.bash" ]; then
        source "$ROS_WS/install/setup.bash"
        log "ROS2 environment sourced"
    else
        log "WARNING: ROS2 environment not found, trying system installation"
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source "/opt/ros/humble/setup.bash"
        elif [ -f "/opt/ros/foxy/setup.bash" ]; then
            source "/opt/ros/foxy/setup.bash"
        fi
    fi
    
    # Update active map configuration
    if [ -f "$CONFIG_FILE" ]; then
        jq --arg map "$map_name" '.activeMap = $map | .lastMapChange = now | .lastMapChangeISO = (now | strftime("%Y-%m-%dT%H:%M:%S.%fZ"))' "$CONFIG_FILE" > "$CONFIG_FILE.tmp" && mv "$CONFIG_FILE.tmp" "$CONFIG_FILE"
        log "Configuration updated with active map: $map_name"
    fi
    
    # Try to load map using map_server (if available)
    log "Attempting to load map via ROS2 map_server..."
    
    # This will depend on your specific ROS2 setup
    # Uncomment and modify as needed for your system:
    # ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: $map_file}"
    
    log "Map loading script completed for: $map_name"
}

# Install systemd service
install_service() {
    local service_content="[Unit]
Description=AMR Fleet Map Loader
After=network.target

[Service]
Type=forking
User=${SSH_CONFIG.username}
WorkingDirectory=${homeDir}
ExecStart=${homeDir}/ros2_ws/load_map.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target"
    
    echo "$service_content" | sudo tee /etc/systemd/system/AMR-map-loader.service > /dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable AMR-map-loader.service
    log "Systemd service installed and enabled"
}

# Main script logic
case "$1" in
    "install-service")
        install_service
        ;;
    *)
        if [ -n "$1" ]; then
            load_map "$1"
        else
            log "Usage: $0 <map_name> | install-service"
            log "Available maps:"
            ls -1 "$MAP_DIR"/*.yaml 2>/dev/null | xargs -I {} basename {} .yaml || log "No maps found in $MAP_DIR"
        fi
        ;;
esac
`;
}

// Run setup if called directly
if (require.main === module) {
    setupRaspberryPi()
        .then((result) => {
            console.log('✅ Setup completed:', result.message);
            process.exit(0);
        })
        .catch((error) => {
            console.error('❌ Setup failed:', error.message);
            process.exit(1);
        });
}

module.exports = { setupRaspberryPi };
