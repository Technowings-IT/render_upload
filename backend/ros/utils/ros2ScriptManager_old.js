// ros/utils/ros2ScriptManager.js - Enhanced version with direct SSH execution to Pi
const { spawn, exec } = require('child_process');
const fs = require('fs').promises;
const path = require('path');
const EventEmitter = require('events');
const { Client } = require('ssh2');

class ROS2ScriptManager extends EventEmitter {
    constructor() {
        super();
        this.activeProcesses = new Map();
        this.processStatus = {
            slam: 'stopped',
            navigation: 'stopped',
            robot_control: 'stopped'
        };
        this.processPIDs = {}; // Store PIDs from Pi
        this.logFile = path.join(__dirname, '../../storage/logs/ros_script_manager.log');
        this.tempScriptDir = path.join(__dirname, '../../temp');
        this.mapDir = path.join(__dirname, '../../maps');
        
        // Raspberry Pi SSH configuration
        this.piConfig = {
            host: '192.168.0.84',
            username: 'piros',
            password: 'piros',
            port: 22
        };
        
        // Pi script file paths
        this.piScriptPaths = {
            robot_control: '/home/piros/scripts/robot_control.sh',
            slam: '/home/piros/scripts/slam.sh',
            navigation: '/home/piros/scripts/nav2.sh',
            kill: '/home/piros/scripts/kill.sh',
        };
        
        // Ensure directories exist
        this.initializeDirectories();
    }

    // Method to update Pi configuration
    updatePiConfig(newConfig) {
        this.piConfig = {
            ...this.piConfig,
            ...newConfig
        };
        console.log(`🔧 Pi configuration updated:`, this.piConfig);
        return this.piConfig;
    }

    async initializeDirectories() {
        try {
            await fs.mkdir(this.tempScriptDir, { recursive: true });
            await fs.mkdir(this.mapDir, { recursive: true });
            await fs.mkdir(path.dirname(this.logFile), { recursive: true });
        } catch (error) {
            console.error('Failed to create directories:', error);
        }
    }

    async log(message) {
        const timestamp = new Date().toISOString();
        const logMessage = `[${timestamp}] ${message}\n`;
        try {
            await fs.appendFile(this.logFile, logMessage);
        } catch (error) {
            console.error('Failed to write to log file:', error);
        }
        console.log(`🤖 ROS2Manager: ${message}`);
    }

    // Direct script execution on Pi (same method as direct_start.js)
    async executeScriptOnPi(scriptType, options = {}) {
        this.log(`Starting executeScriptOnPi for ${scriptType} with options: ${JSON.stringify(options)}`);
        
        return new Promise((resolve, reject) => {
            const conn = new Client();
            
            conn.on('ready', () => {
                this.log(`✅ SSH connected for ${scriptType}`);
                
                // Prepare script arguments based on type
                let scriptName, args = '';
                
                switch (scriptType) {
                    case 'robot_control':
                        scriptName = 'robot_control.sh';
                        break;
                        
                    case 'slam':
                        scriptName = 'slam.sh';
                        args = options.mapName || ('new_slam_map_' + Date.now());
                        break;
                        
                    case 'navigation':
                        scriptName = 'nav2.sh';
                        args = options.mapPath || '/home/piros/fleet-management-system/ros_ws/src/amr/maps/map_1750065869.yaml params_file:=/home/piros/fleet-management-system/ros_ws/src/amr/config/nav2_params1.yaml use_sim_time:=False';
                        break;
                        
                    case 'kill':
                        scriptName = 'kill.sh';
                        break;
                        
                    default:
                        conn.end();
                        return reject(new Error(`Unknown script type: ${scriptType}`));
                }
                
                // Execute script in background with nohup (same as direct_start.js)
                const command = `cd /home/piros/scripts && nohup bash ${scriptName} ${args} > /tmp/${scriptName.replace('.sh', '')}.log 2>&1 & echo $!`;
                this.log(`📤 Executing: ${command}`);
                
                conn.exec(command, (err, stream) => {
                    if (err) {
                        conn.end();
                        return reject(err);
                    }
                    
                    let output = '';
                    
                    stream.on('data', (data) => {
                        output += data.toString();
                        this.log(`📥 ${scriptName} output: ${data.toString().trim()}`);
                    });
                    
                    stream.on('close', (code) => {
                        conn.end();
                        const pid = output.trim();
                        this.log(`🏁 ${scriptName} finished with code ${code}, PID: ${pid}`);
                        
                        // Store PID for process tracking
                        if (pid && pid !== '') {
                            this.processPIDs[scriptType] = pid;
                        }
                        
                        // Update process status
                        if (scriptType !== 'kill') {
                            this.processStatus[scriptType] = 'running';
                        }
                        
                        resolve({ 
                            script: scriptName, 
                            code, 
                            pid,
                            status: scriptType !== 'kill' ? 'running' : 'stopped'
                        });
                    });
                });
            });
            
            conn.on('error', (err) => {
                this.log(`SSH connection error for ${scriptType}: ${err.message}`);
                this.processStatus[scriptType] = 'stopped';
                reject(err);
            });
            
            conn.connect(this.piConfig);
        });
    }

    // Start robot control
    async startRobotControl() {
        try {
            this.log('Starting robot control script...');
            this.processStatus.robot_control = 'starting';
            
            const result = await this.executeScriptOnPi('robot_control');
            this.processStatus.robot_control = 'running';
            this.emit('robot_control_started', result);
            
            this.log(`Robot control started successfully: PID ${result.pid}`);
            return {
                success: true,
                message: 'Robot control started successfully',
                pid: result.pid
            };
        } catch (error) {
            this.processStatus.robot_control = 'stopped';
            this.log(`Failed to start robot control: ${error.message}`);
            throw error;
        }
    }

    // Start SLAM
    async startSLAM(options = {}) {
        try {
            this.log('Starting SLAM...');
            this.processStatus.slam = 'starting';
            
            // Auto-start robot control if not running
            if (this.processStatus.robot_control !== 'running') {
                this.log('Auto-starting robot control for SLAM...');
                try {
                    await this.startRobotControl();
                    // Wait a moment for robot control to initialize
                    await new Promise(resolve => setTimeout(resolve, 2000));
                } catch (error) {
                    this.log(`Failed to auto-start robot control: ${error.message}`);
                }
            }
            
            const result = await this.executeScriptOnPi('slam', options);
            this.processStatus.slam = 'running';
            this.emit('slam_started', result);
            
            this.log(`SLAM started successfully: PID ${result.pid}`);
            return {
                success: true,
                message: 'SLAM started successfully',
                pid: result.pid
            };
        } catch (error) {
            this.processStatus.slam = 'stopped';
            this.log(`Failed to start SLAM: ${error.message}`);
            throw error;
        }
    }

    // Start Navigation
    async startNavigation(mapPath, options = {}) {
        try {
            this.log(`Starting navigation with map: ${mapPath}`);
            this.processStatus.navigation = 'starting';
            
            // Auto-start robot control if not running
            if (this.processStatus.robot_control !== 'running') {
                this.log('Auto-starting robot control for navigation...');
                try {
                    await this.startRobotControl();
                    // Wait a moment for robot control to initialize
                    await new Promise(resolve => setTimeout(resolve, 2000));
                } catch (error) {
                    this.log(`Failed to auto-start robot control: ${error.message}`);
                }
            }
            
            const result = await this.executeScriptOnPi('navigation', { 
                ...options, 
                mapPath 
            });
            this.processStatus.navigation = 'running';
            this.emit('navigation_started', result);
            
            this.log(`Navigation started successfully: PID ${result.pid}`);
            return {
                success: true,
                message: 'Navigation started successfully',
                pid: result.pid
            };
        } catch (error) {
            this.processStatus.navigation = 'stopped';
            this.log(`Failed to start navigation: ${error.message}`);
            throw error;
        }
    }

    // Kill all ROS processes using kill.sh script
    async killAllRosProcesses() {
        try {
            this.log('Executing kill.sh script to stop all ROS processes...');
            
            const result = await this.executeScriptOnPi('kill');
            
            // Reset all process statuses
            this.processStatus.robot_control = 'stopped';
            this.processStatus.slam = 'stopped';
            this.processStatus.navigation = 'stopped';
            this.processPIDs = {};
            
            this.emit('emergency_stop');
            this.log('All ROS processes stopped via kill.sh script');
            
            return {
                success: true,
                message: 'All ROS processes stopped successfully'
            };
        } catch (error) {
            this.log(`Failed to execute kill script: ${error.message}`);
            throw error;
        }
    }

    // Get process status
    getProcessStatus() {
        return {
            processes: this.processStatus,
            pids: this.processPIDs,
            timestamp: new Date().toISOString()
        };
    }

    // Handle script commands (for WebSocket integration)
    async handleScriptCommand(command, options = {}) {
        this.log(`Handling script command: ${command} with options: ${JSON.stringify(options)}`);
        
        try {
            switch (command) {
                case 'start_robot_control':
                    return await this.startRobotControl();
                    
                case 'start_slam':
                    return await this.startSLAM(options);
                    
                case 'start_navigation':
                    return await this.startNavigation(options.mapPath, options);
                    
                case 'stop_all_scripts':
                    return await this.killAllRosProcesses();
                    
                default:
                    throw new Error(`Unknown script command: ${command}`);
            }
        } catch (error) {
            this.log(`Script command failed: ${error.message}`);
            throw error;
        }
    }
}

module.exports = ROS2ScriptManager;
