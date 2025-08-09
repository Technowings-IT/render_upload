// ros/utils/ros2ScriptManager.js - Enhanced version with SSH execution to Pi
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
            robot_launch: 'stopped',
            slam: 'stopped',
            navigation: 'stopped',
            kill_robot: 'stopped',
            kill_all: 'stopped',
            robot_control: 'stopped' // Keep for backward compatibility
        };
        this.processPIDs = {}; // Add this to store PIDs
        this.logFile = path.join(__dirname, '../../storage/logs/ros_script_manager.log');
        this.tempScriptDir = path.join(__dirname, '../../temp');
        this.mapDir = path.join(__dirname, '../../maps');
        
        // Raspberry Pi SSH configuration - can be updated via updatePiConfig
        this.piConfig = {
            host: '192.168.208.29',
            username: 'piros',
            password: 'piros',
            port: 22
        };
        
        // Pi script file paths - Updated for 5 robot control buttons
        this.piScriptPaths = {
            robot_launch: '/home/piros/scripts/robot_launch.sh',
            slam: '/home/piros/scripts/slam.sh',
            navigation: '/home/piros/scripts/nav2.sh',
            kill_robot: '/home/piros/scripts/kill_robot.sh',
            kill_all: '/home/piros/scripts/kill_all.sh'
        };
        
        // Track SSH connections
        this.sshConnections = new Map();
        
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

    // Diagnostic method to check Pi status
    async checkPiStatus() {
        const status = {
            piReachable: false,
            sshConnectable: false,
            scriptsExist: false,
            rosInstalled: false,
            diagnostics: [],
            recommendations: []
        };

        try {
            // Test basic ping connectivity
            const { spawn } = require('child_process');
            const pingResult = await new Promise((resolve) => {
                const ping = spawn('ping', ['-c', '1', '-W', '5', this.piConfig.host]);
                ping.on('close', (code) => {
                    resolve(code === 0);
                });
            });

            status.piReachable = pingResult;
            if (pingResult) {
                status.diagnostics.push(`✅ Pi is reachable at ${this.piConfig.host}`);
            } else {
                status.diagnostics.push(`❌ Pi is not reachable at ${this.piConfig.host}`);
                status.recommendations.push('Check Pi power and network connection');
            }

            // Test SSH connectivity
            try {
                await this.testSSHConnectivity();
                status.sshConnectable = true;
                status.diagnostics.push('✅ SSH connection successful');

                // Test if scripts exist
                try {
                    const scriptsResult = await this.checkScriptsExist();
                    status.scriptsExist = scriptsResult;
                    if (scriptsResult) {
                        status.diagnostics.push('✅ Required scripts found on Pi');
                    } else {
                        status.diagnostics.push('❌ Required scripts missing on Pi');
                        status.recommendations.push('Deploy scripts to Pi: /home/piros/scripts/');
                    }
                } catch (error) {
                    status.diagnostics.push(`❌ Cannot check scripts: ${error.message}`);
                }

            } catch (sshError) {
                status.diagnostics.push(`❌ SSH connection failed: ${sshError.message}`);
                status.recommendations.push('Check SSH service is running on Pi');
                status.recommendations.push('Verify SSH credentials (username: piros, password: piros)');
            }

        } catch (error) {
            status.diagnostics.push(`❌ Status check failed: ${error.message}`);
        }

        return status;
    }

    async checkScriptsExist() {
        return new Promise((resolve, reject) => {
            const conn = new Client();
            
            conn.on('ready', () => {
                const checkCommand = 'ls -la /home/piros/scripts/slam.sh /home/piros/scripts/nav2.sh 2>/dev/null | wc -l';
                
                conn.exec(checkCommand, (err, stream) => {
                    if (err) {
                        conn.end();
                        return reject(err);
                    }
                    
                    let output = '';
                    stream.on('close', (code) => {
                        conn.end();
                        const scriptCount = parseInt(output.trim());
                        resolve(scriptCount >= 3); // At least 3 scripts should exist
                    });
                    
                    stream.on('data', (data) => {
                        output += data.toString();
                    });
                });
            });
            
            conn.on('error', (err) => {
                reject(err);
            });
            
            conn.connect(this.piConfig);
        });
    }

    // ==========================================
    // RASPBERRY PI SSH EXECUTION METHODS
    // ==========================================

    // Test SSH connectivity to Pi
    async testSSHConnectivity() {
        return new Promise((resolve, reject) => {
            const conn = new Client();
            const timeout = setTimeout(() => {
                conn.end();
                reject(new Error('SSH connectivity test failed - Pi may be unreachable or SSH service down'));
            }, 10000); // 10 second timeout
            
            conn.on('ready', () => {
                clearTimeout(timeout);
                this.log('SSH connectivity test successful');
                // Test a simple command
                conn.exec('echo "SSH test successful"', (err, stream) => {
                    if (err) {
                        conn.end();
                        return reject(new Error(`SSH command test failed: ${err.message}`));
                    }
                    
                    stream.on('close', (code) => {
                        conn.end();
                        if (code === 0) {
                            resolve(true);
                        } else {
                            reject(new Error(`SSH test command failed with code ${code}`));
                        }
                    });
                });
            });
            
            conn.on('error', (err) => {
                clearTimeout(timeout);
                this.log(`SSH connectivity test failed: ${err.message}`);
                reject(new Error(`SSH connectivity test failed - Pi may be unreachable or SSH service down`));
            });
            
            conn.connect(this.piConfig);
        });
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
                    case 'robot_launch':
                        scriptName = 'robot_launch.sh';
                        args = ''; // Robot launch typically doesn't need arguments
                        break;
                        
                    case 'slam':
                        scriptName = 'slam.sh';
                        args = options.mapName || ('new_slam_map_' + Date.now());
                        break;
                        
                    case 'navigation':
                        scriptName = 'nav2.sh';
                        args = options.mapPath || '/home/piros/fleet-management-system/ros_ws/src/AMR/maps/map_1750065869.yaml params_file:=/home/piros/fleet-management-system/ros_ws/src/AMR/config/nav2_params1.yaml use_sim_time:=False';
                        break;
                        
                    case 'kill_robot':
                        scriptName = 'kill_robot.sh';
                        args = ''; // Kill robot script doesn't need arguments
                        break;
                        
                    case 'kill_all':
                    case 'kill': // Keep backward compatibility
                        scriptName = 'kill_all.sh';
                        args = ''; // Kill all script doesn't need arguments
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
            let result;
            switch (command) {
                // ✅ NEW: 5 Specific Robot Control Commands
                case 'start_robot':
                case 'execute_robot_launch':
                    result = await this.executeSpecificScript({
                        script_name: 'robot_launch.sh',
                        script_path: '/home/piros/scripts/robot_launch.sh',
                        action: 'start_robot'
                    });
                    break;
                    
                case 'start_slam_button':
                case 'execute_slam':
                    result = await this.executeSpecificScript({
                        script_name: 'slam.sh',
                        script_path: '/home/piros/scripts/slam.sh',
                        action: 'start_slam',
                        map_name: options.map_name || `slam_map_${Date.now()}`
                    });
                    break;
                    
                case 'start_nav_button':
                case 'execute_nav2':
                    result = await this.executeSpecificScript({
                        script_name: 'nav2.sh',
                        script_path: '/home/piros/scripts/nav2.sh',
                        action: 'start_navigation'
                    });
                    break;
                    
                case 'stop_robot_button':
                case 'execute_kill_robot':
                    result = await this.executeSpecificScript({
                        script_name: 'kill_robot.sh',
                        script_path: '/home/piros/scripts/kill_robot.sh',
                        action: 'stop_robot'
                    });
                    break;
                    
                case 'stop_pid_button':
                case 'execute_kill_all':
                    result = await this.executeSpecificScript({
                        script_name: 'kill_all.sh',
                        script_path: '/home/piros/scripts/kill_all.sh',
                        action: 'stop_all_processes'
                    });
                    break;
                
                // ✅ EXISTING: Backward compatibility commands
                case 'start_robot_control':
                    result = await this.startRobotControl();
                    break;
                    
                case 'start_slam':
                    result = await this.startSLAM(options);
                    break;
                    
                case 'start_navigation':
                    result = await this.startNavigation(options.mapPath, options);
                    break;
                    
                case 'stop_robot_control':
                    result = await this.stopProcess('robot_control');
                    break;
                    
                case 'stop_slam':
                    result = await this.stopProcess('slam');
                    break;
                    
                case 'stop_navigation':
                    result = await this.stopProcess('navigation');
                    break;
                    
                case 'stop_all_scripts':
                    result = await this.killAllRosProcesses();
                    break;
                    
                case 'kill_all_processes':
                    result = await this.killAllRosProcesses();
                    break;
                    
                // ✅ NEW: Handle specific script execution
                case 'execute_script':
                    result = await this.executeSpecificScript(options);
                    break;
                    
                case 'emergency_stop':
                    result = await this.emergencyStop();
                    break;
                    
                case 'get_status':
                    result = this.getProcessStatus();
                    break;
                    
                case 'get_detailed_status':
                    result = this.getDetailedStatus();
                    break;
                    
                case 'check_pi_status':
                    result = await this.checkPiStatus();
                    break;
                    
                case 'test_ssh_connectivity':
                    result = await this.testSSHConnectivity();
                    break;
                    
                case 'restart_robot_control':
                    result = await this.restart('robot_control', options);
                    break;
                    
                case 'restart_slam':
                    result = await this.restart('slam', options);
                    break;
                    
                case 'restart_navigation':
                    result = await this.restart('navigation', options);
                    break;
                    
                default:
                    throw new Error(`Unknown script command: ${command}`);
            }
            
            this.log(`✅ Script command '${command}' executed successfully`);
            return result;
        } catch (error) {
            this.log(`Script command failed: ${error.message}`);
            throw error;
        }
    }

    // ✅ NEW: Execute specific script directly on Pi
    async executeSpecificScript(options) {
        const { script_name, script_path, action, map_name } = options;
        
        if (!script_name) {
            throw new Error('script_name is required');
        }
        
        this.log(`🎯 Executing specific script: ${script_name} on Pi`);
        this.log(`📂 Script path: ${script_path || 'default'}`);
        this.log(`🎬 Action: ${action || 'run_only_this_script'}`);
        
        try {
            const result = await this.executePiScript(script_name, script_path, {
                action,
                map_name,
                force_kill_others: false // Only run this script, don't kill others
            });
            
            // Update process status based on script
            switch (script_name) {
                case 'robot_launch.sh':
                    this.processStatus.robot_launch = 'running';
                    this.processStatus.robot_control = 'running'; // For compatibility
                    break;
                case 'slam.sh':
                    this.processStatus.slam = 'running';
                    break;
                case 'nav2.sh':
                    this.processStatus.navigation = 'running';
                    break;
                case 'kill_robot.sh':
                    this.processStatus.kill_robot = 'stopped';
                    this.processStatus.robot_launch = 'stopped';
                    this.processStatus.robot_control = 'stopped'; // For compatibility
                    break;
                case 'kill_all.sh':
                case 'kill.sh': // Backward compatibility
                    // Kill all script stops everything
                    this.processStatus.robot_launch = 'stopped';
                    this.processStatus.slam = 'stopped';
                    this.processStatus.navigation = 'stopped';
                    this.processStatus.kill_robot = 'stopped';
                    this.processStatus.kill_all = 'stopped';
                    this.processStatus.robot_control = 'stopped'; // For compatibility
                    break;
            }
            
            return {
                success: true,
                message: `${script_name} executed successfully on Pi`,
                script_name,
                script_path: script_path || `./scripts/${script_name}`,
                action: action || 'run_only_this_script',
                output: result.stdout || '',
                timestamp: new Date().toISOString()
            };
            
        } catch (error) {
            this.log(`❌ Failed to execute ${script_name}: ${error.message}`);
            throw new Error(`Failed to execute ${script_name}: ${error.message}`);
        }
    }

        // ✅ NEW: Execute script directly on Pi via SSH
    async executePiScript(scriptName, scriptPath, options = {}) {
        return new Promise((resolve, reject) => {
            const conn = new Client();
            const timeout = setTimeout(() => {
                conn.end();
                reject(new Error(`Script execution timeout for ${scriptName}`));
            }, 30000); // 30 second timeout
            
            conn.on('ready', () => {
                this.log(`📡 SSH connected for ${scriptName} execution`);
                
                // ✅ FIXED: Always use absolute paths from piScriptPaths first
                let actualPath;
                const scriptKey = scriptName.replace('.sh', '');
                
                // Priority: 1. Predefined paths, 2. Absolute path, 3. Default absolute path
                if (this.piScriptPaths[scriptKey]) {
                    actualPath = this.piScriptPaths[scriptKey];
                    this.log(`🎯 Using predefined path: ${actualPath}`);
                } else if (scriptPath && scriptPath.startsWith('/')) {
                    actualPath = scriptPath;
                    this.log(`🎯 Using provided absolute path: ${actualPath}`);
                } else {
                    actualPath = `/home/piros/scripts/${scriptName}`;
                    this.log(`🎯 Using default absolute path: ${actualPath}`);
                }
                
                // ✅ UPDATED: Build command for all 5 script types
                let command;
                switch (scriptName) {
                    case 'robot_launch.sh':
                        // Run robot launch in background with nohup
                        command = `cd /home/piros/scripts && nohup bash ${actualPath} > /tmp/robot_launch_output.log 2>&1 & echo $!`;
                        break;
                    case 'slam.sh':
                        const mapName = options.map_name || `slam_map_${Date.now()}`;
                        // Run SLAM in background with nohup
                        command = `cd /home/piros/scripts && nohup bash ${actualPath} "${mapName}" > /tmp/slam_output.log 2>&1 & echo $!`;
                        break;
                    case 'nav2.sh':
                        // Run nav2 in background with nohup to prevent it from stopping when SSH closes
                        command = `cd /home/piros/scripts && nohup bash ${actualPath} > /tmp/nav2_output.log 2>&1 & echo $!`;
                        break;
                    case 'kill_robot.sh':
                        // Kill robot script should run immediately, not in background
                        command = `chmod +x ${actualPath} && ${actualPath}`;
                        break;
                    case 'kill_all.sh':
                    case 'kill.sh': // Backward compatibility
                        // Kill all script should run immediately, not in background
                        command = `chmod +x ${actualPath} && ${actualPath}`;
                        break;
                    default:
                        // Default: run in background
                        command = `cd /home/piros/scripts && nohup bash ${actualPath} > /tmp/${scriptName}_output.log 2>&1 & echo $!`;
                }
                
                this.log(`🚀 Executing command: ${command}`);
                
                conn.exec(command, (err, stream) => {
                    if (err) {
                        clearTimeout(timeout);
                        conn.end();
                        return reject(new Error(`Failed to execute ${scriptName}: ${err.message}`));
                    }
                    
                    let stdout = '';
                    let stderr = '';
                    
                    stream.on('data', (data) => {
                        stdout += data.toString();
                        this.log(`📄 ${scriptName} stdout: ${data.toString().trim()}`);
                    });
                    
                    stream.stderr.on('data', (data) => {
                        stderr += data.toString();
                        this.log(`⚠️ ${scriptName} stderr: ${data.toString().trim()}`);
                    });
                    
                    stream.on('close', (code) => {
                        clearTimeout(timeout);
                        conn.end();
                        
                        this.log(`✅ ${scriptName} execution completed with code: ${code}`);
                        
                        // For background processes, store PID if available
                        if (scriptName !== 'kill.sh' && scriptName !== 'kill_all.sh' && scriptName !== 'kill_robot.sh') {
                            const pid = stdout.trim();
                            if (pid && !isNaN(pid)) {
                                this.processPIDs[scriptKey] = pid;
                                this.log(`🆔 Stored PID ${pid} for ${scriptName}`);
                            }
                        }
                        
                        if (code === 0) {
                            resolve({
                                success: true,
                                stdout,
                                stderr,
                                exitCode: code,
                                pid: stdout.trim(),
                                timestamp: new Date().toISOString()
                            });
                        } else {
                            reject(new Error(`${scriptName} failed with exit code ${code}: ${stderr || stdout}`));
                        }
                    });
                });
            });
            
            conn.on('error', (err) => {
                clearTimeout(timeout);
                this.log(`❌ SSH connection error for ${scriptName}: ${err.message}`);
                reject(new Error(`SSH connection failed: ${err.message}`));
            });
            
            // Connect to Pi
            conn.connect(this.piConfig);
        });
    }

    // ==========================================
    // ENHANCED PROCESS VERIFICATION METHODS
    // ==========================================

    // New method to verify process is actually running
    async verifyProcessRunning(scriptType, existingConn = null) {
        return new Promise((resolve, reject) => {
            const useConn = existingConn || new Client();
            const shouldCloseConn = !existingConn;
            
            const executeCheck = () => {
                // Check if process is running by looking for it in process list
                const checkCommand = `ps aux | grep "${scriptType}" | grep -v grep || echo "NO_PROCESS_FOUND"`;
                
                useConn.exec(checkCommand, (err, stream) => {
                    if (err) {
                        if (shouldCloseConn) useConn.end();
                        return reject(new Error(`Failed to verify process: ${err.message}`));
                    }
                    
                    let output = '';
                    
                    stream.on('data', (data) => {
                        output += data.toString();
                    });
                    
                    stream.on('close', (code) => {
                        if (shouldCloseConn) useConn.end();
                        
                        this.log(`Process verification output: ${output.trim()}`);
                        
                        if (output.includes('NO_PROCESS_FOUND')) {
                            reject(new Error(`${scriptType} process not found in process list`));
                        } else {
                            resolve({
                                running: true,
                                processes: output.trim().split('\n').filter(line => line.trim())
                            });
                        }
                    });
                });
            };
            
            if (existingConn) {
                executeCheck();
            } else {
                useConn.on('ready', executeCheck);
                useConn.on('error', (err) => {
                    reject(new Error(`SSH connection failed for verification: ${err.message}`));
                });
                useConn.connect(this.piConfig);
            }
        });
    }

    // Helper method to verify script is running
    async verifyScriptRunning(scriptType, sessionName) {
        return new Promise((resolve, reject) => {
            const conn = new Client();
            
            conn.on('ready', () => {
                // Check if screen session exists
                const checkCommand = `screen -ls | grep ${sessionName}`;
                
                conn.exec(checkCommand, (err, stream) => {
                    if (err) {
                        conn.end();
                        return reject(new Error(`Failed to verify ${scriptType}: ${err.message}`));
                    }
                    
                    let output = '';
                    
                    stream.on('close', (code) => {
                        conn.end();
                        
                        if (code === 0 && output.includes(sessionName)) {
                            resolve();
                        } else {
                            reject(new Error(`${scriptType} screen session not found`));
                        }
                    });
                    
                    stream.on('data', (data) => {
                        output += data.toString();
                    });
                });
            });
            
            conn.connect(this.piConfig);
        });
    }

    async stopScriptOnPi(scriptType) {
        return new Promise((resolve, reject) => {
            const conn = new Client();
            
            conn.on('ready', () => {
                this.log(`SSH connection established for stopping ${scriptType}`);
                
                // Build kill commands
                let killCommands = [];
                
                // Kill by PID if we have it
                if (this.processPIDs && this.processPIDs[scriptType]) {
                    killCommands.push(`kill ${this.processPIDs[scriptType]} 2>/dev/null || true`);
                }
                
                // Kill by process name as backup
                switch (scriptType) {
                    case 'slam':
                        killCommands.push('pkill -f "slam_toolbox" || true');
                        killCommands.push('pkill -f "slam.sh" || true');
                        break;
                    case 'navigation':
                        killCommands.push('pkill -f "nav2_bringup" || true');
                        killCommands.push('pkill -f "nav2.sh" || true');
                        break;
                    
                }
                
                const killCommand = killCommands.join(' ; ');
                
                this.log(`Stopping ${scriptType} on Pi: ${killCommand}`);
                
                conn.exec(killCommand, (err, stream) => {
                    if (err) {
                        conn.end();
                        return reject(new Error(`Failed to stop ${scriptType}: ${err.message}`));
                    }
                    
                    let stdout = '';
                    let stderr = '';
                    
                    stream.on('close', (code) => {
                        this.log(`Stop command for ${scriptType} finished with code ${code}`);
                        
                        // Clean up stored PID
                        if (this.processPIDs && this.processPIDs[scriptType]) {
                            delete this.processPIDs[scriptType];
                        }
                        
                        this.processStatus[scriptType] = 'stopped';
                        this.emit(`${scriptType}_stopped`);
                        
                        conn.end();
                        resolve({
                            success: true,
                            message: `${scriptType} stopped successfully on Pi`,
                            stdout,
                            stderr
                        });
                    });
                    
                    stream.on('data', (data) => {
                        stdout += data.toString();
                    });
                    
                    stream.stderr.on('data', (data) => {
                        stderr += data.toString();
                    });
                });
            });
            
            conn.connect(this.piConfig);
        });
    }

    // ==========================================
    // ENHANCED PROCESS MANAGEMENT FOR PI
    // ==========================================

    async stopProcess(processName) {
        if (!this.processStatus[processName] || this.processStatus[processName] === 'stopped') {
            return { success: false, message: `${processName} is not running` };
        }

        try {
            await this.log(`Stopping ${processName} on Raspberry Pi...`);
            
            // Stop the process on Pi
            const result = await this.stopScriptOnPi(processName);
            
            this.processStatus[processName] = 'stopped';
            
            await this.log(`${processName} stopped successfully on Pi`);
            this.emit(`${processName}_stopped`, 0);
            
            return { 
                success: true, 
                message: `${processName} stopped successfully on Raspberry Pi`,
                piResult: result
            };

        } catch (error) {
            await this.log(`Failed to stop ${processName} on Pi: ${error.message}`);
            throw error;
        }
    }

    async stopAll() {
        const stopPromises = Object.keys(this.processStatus).map(
            processName => this.stopProcess(processName).catch(error => ({
                processName,
                success: false,
                error: error.message
            }))
        );
        
        const results = await Promise.all(stopPromises);
        return results;
    }

    async emergencyStop() {
        await this.log('EMERGENCY STOP - Killing all ROS2 processes on Pi');
        
        return new Promise((resolve, reject) => {
            const conn = new Client();
            
            conn.on('ready', () => {
                // Kill all ROS2 processes and scripts (removed robot_control references)
                const emergencyCommand = `
                    pkill -f "ros2"; 
                    pkill -f "slam_toolbox"; 
                    pkill -f "nav2_bringup"; 
                    pkill -f "slam.sh"; 
                    pkill -f "nav2.sh"
                `;
                
                conn.exec(emergencyCommand, (err, stream) => {
                    if (err) {
                        conn.end();
                        return reject(err);
                    }
                    
                    stream.on('close', (code) => {
                        conn.end();
                        
                        // Reset process statuses
                        this.processStatus.slam = 'stopped';
                        this.processStatus.navigation = 'stopped';
                        
                        // Close any active SSH connections
                        this.sshConnections.clear();
                        
                        this.emit('emergency_stop');
                        
                        resolve({ 
                            success: true, 
                            message: 'Emergency stop completed on Raspberry Pi' 
                        });
                    });
                    
                    stream.on('data', (data) => {
                        this.log(`Emergency stop output: ${data.toString().trim()}`);
                    });
                });
            });
            
            conn.on('error', (err) => {
                this.log(`Emergency stop SSH error: ${err.message}`);
                reject(err);
            });
            
            conn.connect(this.piConfig);
        });
    }

    // ==========================================
    // UTILITY METHODS
    // ==========================================

    async validateAndResolveMapPath(mapPath) {
        // If it's already a full path, validate it exists
        if (path.isAbsolute(mapPath)) {
            try {
                await fs.access(mapPath);
                return mapPath;
            } catch (error) {
                throw new Error(`Map file not found: ${mapPath}`);
            }
        }
        
        // If it's just a name, look in the maps directory
        const baseName = path.basename(mapPath, '.yaml');
        const yamlPath = path.join(this.mapDir, `${baseName}.yaml`);
        const pgmPath = path.join(this.mapDir, `${baseName}.pgm`);
        
        try {
            await fs.access(yamlPath);
            await fs.access(pgmPath);
            return yamlPath;
        } catch (error) {
            throw new Error(`Map files not found: ${baseName}.yaml and ${baseName}.pgm must exist in ${this.mapDir}`);
        }
    }

    getStatus() {
        return {
            processes: { ...this.processStatus },
            activeCount: Object.values(this.processStatus).filter(status => status === 'running').length,
            activeProcesses: Object.keys(this.processStatus).filter(key => this.processStatus[key] === 'running'),
            sshConnections: this.sshConnections.size,
            timestamp: new Date().toISOString()
        };
    }

    getDetailedStatus() {
        const status = this.getStatus();
        
        return {
            ...status,
            piConfig: {
                host: this.piConfig.host,
                username: this.piConfig.username,
                port: this.piConfig.port
            },
            piScriptPaths: this.piScriptPaths,
            logFile: this.logFile,
            tempScriptDir: this.tempScriptDir,
            mapDir: this.mapDir
        };
    }

    async restart(processName, options = {}) {
        await this.stopProcess(processName);
        
        // Wait a bit before restarting
        await new Promise(resolve => setTimeout(resolve, 2000));
        
        switch (processName) {
            case 'slam':
                return await this.startSLAM(options);
            case 'navigation':
                if (!options.mapPath) {
                    throw new Error('Navigation restart requires mapPath in options');
                }
                return await this.startNavigation(options.mapPath, options);
            case 'robot_control':
                return await this.startRobotControl();
            default:
                throw new Error(`Unknown process: ${processName}`);
        }
    }

    // ==========================================
    // MAP CONVERSION INTEGRATION
    // ==========================================

    async convertAndPrepareMap(jsonMapData, mapName) {
        try {
            await this.log(`Converting map: ${mapName}`);
            
            // Call Python conversion script
            const pythonScript = path.join(__dirname, '../../scripts/map_converter.py');
            const tempJsonPath = path.join(this.tempScriptDir, `${mapName}_temp.json`);
            
            // Write temporary JSON file
            await fs.writeFile(tempJsonPath, JSON.stringify(jsonMapData, null, 2));
            
            // Run Python converter
            const result = await this.runPythonConverter(pythonScript, tempJsonPath, this.mapDir, mapName);
            
            // Cleanup temp file
            await fs.unlink(tempJsonPath);
            
            await this.log(`Map conversion completed: ${mapName}`);
            return result;
            
        } catch (error) {
            await this.log(`Map conversion failed: ${error.message}`);
            throw error;
        }
    }

    runPythonConverter(pythonScript, inputFile, outputDir, mapName) {
        return new Promise((resolve, reject) => {
            const converterProcess = spawn('python3', [
                pythonScript,
                inputFile,
                '-o', outputDir,
                '-n', mapName
            ]);

            let stdout = '';
            let stderr = '';

            converterProcess.stdout.on('data', (data) => {
                stdout += data.toString();
            });

            converterProcess.stderr.on('data', (data) => {
                stderr += data.toString();
            });

            converterProcess.on('close', (code) => {
                if (code === 0) {
                    resolve({
                        success: true,
                        pgmPath: path.join(outputDir, `${mapName}.pgm`),
                        yamlPath: path.join(outputDir, `${mapName}.yaml`),
                        output: stdout
                    });
                } else {
                    reject(new Error(`Python converter failed with code ${code}: ${stderr}`));
                }
            });

            converterProcess.on('error', (error) => {
                reject(new Error(`Failed to start Python converter: ${error.message}`));
            });
        });
    }

    // ==========================================
    // DEPLOYMENT TO RASPBERRY PI
    // ==========================================

    async deployMapToRaspberryPi(mapName, piAddress = '192.168.208.29') {
        try {
            await this.log(`Deploying map ${mapName} to Raspberry Pi: ${piAddress}`);
            
            const yamlPath = path.join(this.mapDir, `${mapName}.yaml`);
            const pgmPath = path.join(this.mapDir, `${mapName}.pgm`);
            
            // Validate files exist
            await fs.access(yamlPath);
            await fs.access(pgmPath);
            
            // Copy files to Pi using SCP
            const piMapDir = '/home/piros/fleet-management-system/ros_ws/src/AMR/maps/';
            const targetYamlPath = `${piMapDir}${mapName}.yaml`;
            const targetPgmPath = `${piMapDir}${mapName}.pgm`;
            
            // Use SCP to copy files
            await this.runCommand(`scp ${yamlPath} piros@${piAddress}:${targetYamlPath}`);
            await this.runCommand(`scp ${pgmPath} piros@${piAddress}:${targetPgmPath}`);
            
            await this.log(`Map deployed successfully to Pi: ${mapName}`);
            
            return {
                success: true,
                message: `Map ${mapName} deployed to Raspberry Pi`,
                targetFiles: {
                    yaml: targetYamlPath,
                    pgm: targetPgmPath
                }
            };
            
        } catch (error) {
            await this.log(`Map deployment failed: ${error.message}`);
            throw error;
        }
    }

    runCommand(command) {
        return new Promise((resolve, reject) => {
            exec(command, (error, stdout, stderr) => {
                if (error) {
                    reject(error);
                } else {
                    resolve({ stdout, stderr });
                }
            });
        });
    }

    // ==========================================
    // SSH HELPER FUNCTION
    // ==========================================

    /**
     * Execute SSH command helper function
     */
    executeSSHCommand(conn, command) {
        return new Promise((resolve, reject) => {
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
                });
                
                stream.stderr.on('data', (data) => {
                    stderr += data.toString();
                });
            });
        });
    }

    // ==========================================
    // ROS2 MAP SAVER METHODS
    // ==========================================

    /**
     * Execute ROS2 map saver using enhanced save_map.sh script on Raspberry Pi
     */
    async executeROS2MapSaver({ deviceId, mapName, directory = '/home/piros/my_map', format = 'pgm' }) {
        this.log(`Starting enhanced ROS2 map save for device: ${deviceId}, map: ${mapName}`);
        
        return new Promise((resolve, reject) => {
            const conn = new Client();
            
            // Add timeout for the entire operation
            const saveTimeout = setTimeout(() => {
                this.log(`Map save timeout for ${mapName} - closing connection`);
                conn.end();
                resolve({
                    success: false,
                    error: `Map save timeout for ${mapName}`
                });
            }, 45000); // 45 second timeout for map saving
            
            conn.on('ready', async () => {
                try {
                    clearTimeout(saveTimeout);
                    this.log('SSH connection established for enhanced ROS2 map saver');
                    
                    // First, ensure the save_map.sh script exists on Pi
                    await this.ensureSaveMapScriptOnPi(conn);
                    
                    // Create directory if it doesn't exist
                    await this.executeSSHCommand(conn, `mkdir -p ${directory}`);
                    
                    // Execute the enhanced save_map.sh script
                    const scriptCommand = `/home/piros/scripts/save_map.sh "${mapName}" "${directory}"`;
                    
                    this.log(`Executing enhanced map save script: ${scriptCommand}`);
                    
                    const result = await this.executeSSHCommand(conn, scriptCommand);
                    
                    // Check if files were created
                    const yamlFile = `${directory}/${mapName}.yaml`;
                    const pgmFile = `${directory}/${mapName}.pgm`;
                    
                    const filesCheck = await this.executeSSHCommand(conn, `ls -la "${yamlFile}" "${pgmFile}" 2>/dev/null || echo "FILES_NOT_FOUND"`);
                    
                    conn.end();
                    
                    if (filesCheck.includes('FILES_NOT_FOUND')) {
                        this.log(`Map save failed - files not found: ${yamlFile}, ${pgmFile}`);
                        resolve({
                            success: false,
                            error: 'Map files were not created by save script',
                            scriptOutput: result
                        });
                    } else {
                        this.log(`Map save successful - files created: ${yamlFile}, ${pgmFile}`);
                        resolve({
                            success: true,
                            message: `Map "${mapName}" saved successfully`,
                            command: scriptCommand,
                            files: {
                                yaml: yamlFile,
                                pgm: pgmFile
                            },
                            directory: directory,
                            scriptOutput: result,
                            filesInfo: filesCheck
                        });
                    }
                    
                } catch (error) {
                    clearTimeout(saveTimeout);
                    conn.end();
                    this.log(`Enhanced map save error: ${error.message}`);
                    resolve({
                        success: false,
                        error: `Enhanced ROS2 map saver failed: ${error.message}`
                    });
                }
            });
            
            conn.on('error', (error) => {
                clearTimeout(saveTimeout);
                this.log(`SSH connection error during map save: ${error.message}`);
                resolve({
                    success: false,
                    error: `SSH connection failed: ${error.message}`
                });
            });
            
            conn.connect(this.piConfig);
        });
    }

    /**
     * Ensure save_map.sh script exists on Raspberry Pi
     */
    async ensureSaveMapScriptOnPi(existingConn = null) {
        const useConn = existingConn;
        const scriptPath = '/home/piros/scripts/save_map.sh';
        
        try {
            // Check if script exists
            const checkResult = await this.executeSSHCommand(useConn, `ls -la ${scriptPath} 2>/dev/null || echo "SCRIPT_NOT_FOUND"`);
            
            if (checkResult.includes('SCRIPT_NOT_FOUND')) {
                this.log('save_map.sh not found on Pi, uploading...');
                
                // Create scripts directory
                await this.executeSSHCommand(useConn, 'mkdir -p /home/piros/scripts');
                
                // Read local script content
                const fs = require('fs').promises;
                const localScriptPath = path.join(__dirname, '../../scripts/save_map.sh');
                const scriptContent = await fs.readFile(localScriptPath, 'utf8');
                
                // Upload script content (using echo to write file)
                const escapedContent = scriptContent.replace(/'/g, "'\"'\"'");
                await this.executeSSHCommand(useConn, `echo '${escapedContent}' > ${scriptPath}`);
                
                // Make script executable
                await this.executeSSHCommand(useConn, `chmod +x ${scriptPath}`);
                
                this.log('save_map.sh uploaded and made executable on Pi');
            } else {
                this.log('save_map.sh already exists on Pi');
            }
            
            return true;
        } catch (error) {
            this.log(`Error ensuring save_map.sh on Pi: ${error.message}`);
            throw error;
        }
    }

    /**
     * List ROS2 saved maps on Raspberry Pi
     */
    async listROS2SavedMaps({ deviceId, directory = '/tmp/saved_maps' }) {
        return new Promise((resolve, reject) => {
            const conn = new Client();
            
            conn.on('ready', async () => {
                try {
                    this.log(`Listing ROS2 saved maps in: ${directory}`);
                    
                    // List YAML files (map metadata)
                    const listCommand = `find ${directory} -name "*.yaml" -type f -exec ls -la {} + 2>/dev/null || echo "NO_MAPS_FOUND"`;
                    const result = await this.executeSSHCommand(conn, listCommand);
                    
                    conn.end();
                    
                    if (result.includes('NO_MAPS_FOUND')) {
                        resolve([]);
                        return;
                    }
                    
                    // Parse the file list
                    const maps = result.split('\n')
                        .filter(line => line.trim() && line.includes('.yaml'))
                        .map(line => {
                            const parts = line.trim().split(/\s+/);
                            const filePath = parts[parts.length - 1];
                            const fileName = path.basename(filePath, '.yaml');
                            
                            return {
                                name: fileName,
                                yamlFile: filePath,
                                pgmFile: filePath.replace('.yaml', '.pgm'),
                                size: parts[4],
                                modified: `${parts[5]} ${parts[6]} ${parts[7]}`,
                                directory: directory
                            };
                        });
                    
                    resolve(maps);
                    
                } catch (error) {
                    conn.end();
                    console.error('❌ Error listing ROS2 saved maps:', error);
                    resolve([]);
                }
            });
            
            conn.on('error', (error) => {
                console.error('❌ SSH error listing ROS2 maps:', error);
                resolve([]);
            });
            
            conn.connect(this.piConfig);
        });
    }

    /**
     * Load ROS2 saved map data for editing
     */
    async loadROS2SavedMap({ deviceId, mapName, directory = '/tmp/saved_maps' }) {
        return new Promise((resolve, reject) => {
            const conn = new Client();
            
            conn.on('ready', async () => {
                try {
                    this.log(`Loading ROS2 saved map: ${mapName}`);
                    
                    const yamlFile = `${directory}/${mapName}.yaml`;
                    const pgmFile = `${directory}/${mapName}.pgm`;
                    
                    // Read YAML metadata
                    const yamlContent = await this.executeSSHCommand(conn, `cat ${yamlFile}`);
                    
                    // Download PGM file (simplified - in production, you'd want to properly transfer binary data)
                    const pgmInfo = await this.executeSSHCommand(conn, `file ${pgmFile} && wc -c ${pgmFile}`);
                    
                    conn.end();
                    
                    // Convert to MapData format (simplified conversion)
                    const mapData = this.convertROS2MapToMapData(yamlContent, pgmInfo, mapName);
                    
                    resolve({
                        success: true,
                        mapData: mapData
                    });
                    
                } catch (error) {
                    conn.end();
                    resolve({
                        success: false,
                        error: `Failed to load ROS2 map: ${error.message}`
                    });
                }
            });
            
            conn.on('error', (error) => {
                resolve({
                    success: false,
                    error: `SSH connection failed: ${error.message}`
                });
            });
            
            conn.connect(this.piConfig);
        });
    }

    /**
     * Convert ROS2 map data to MapData format
     */
    convertROS2MapToMapData(yamlContent, pgmInfo, mapName) {
        // Parse YAML content to extract map parameters
        const lines = yamlContent.split('\n');
        let resolution = 0.05;
        let origin = [-25.0, -25.0, 0.0];
        let width = 1000;
        let height = 1000;
        
        lines.forEach(line => {
            if (line.includes('resolution:')) {
                resolution = parseFloat(line.split(':')[1].trim());
            }
            if (line.includes('origin:')) {
                const originStr = line.split(':')[1].trim();
                origin = originStr.replace(/[\[\]]/g, '').split(',').map(v => parseFloat(v.trim()));
            }
        });
        
        // Create MapData structure compatible with your system
        return {
            deviceId: 'ros2_saved',
            timestamp: new Date().toISOString(),
            info: {
                resolution: resolution,
                width: width,
                height: height,
                origin: {
                    position: { x: origin[0], y: origin[1], z: origin[2] || 0.0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1 }
                },
                originOrientation: { x: 0, y: 0, z: 0, w: 1 }
            },
            occupancyData: [], // Would need to process PGM file for actual data
            shapes: [],
            version: 1,
            metadata: {
                source: 'ros2_map_saver',
                originalMapName: mapName,
                loadedAt: new Date().toISOString()
            }
        };
    }

    // ==========================================
    // STOP ALL SCRIPTS METHOD
    // ==========================================

    async stopAllScripts() {
        const results = [];
        const scriptsToStop = ['robot_control', 'slam', 'navigation'];
        
        try {
            await this.log('Stopping all scripts on Raspberry Pi...');
            
            // Stop each script individually first
            for (const scriptType of scriptsToStop) {
                if (this.processStatus[scriptType] === 'running') {
                    try {
                        const result = await this.stopScriptOnPi(scriptType);
                        results.push({ scriptType, success: true, result });
                        await this.log(`${scriptType} stopped successfully`);
                    } catch (error) {
                        results.push({ scriptType, success: false, error: error.message });
                        await this.log(`Failed to stop ${scriptType}: ${error.message}`);
                    }
                }
            }
            
            // Then do a comprehensive kill as backup
            await this.killAllRosProcesses();
            
            // Update all statuses
            scriptsToStop.forEach(scriptType => {
                this.processStatus[scriptType] = 'stopped';
                this.emit(`${scriptType}_stopped`);
            });
            
            this.emit('all_scripts_stopped', results);
            
            return {
                success: true,
                message: 'All scripts stopped successfully',
                details: results
            };
            
        } catch (error) {
            await this.log(`Error stopping all scripts: ${error.message}`);
            throw error;
        }
    }

    // ==========================================
    // CLEANUP AND MONITORING
    // ==========================================

    async cleanup() {
        await this.log('Cleaning up ROS2 Script Manager...');
        
        // Stop all processes
        await this.stopAll();
        
        // Close all SSH connections
        for (const [scriptType, { conn }] of this.sshConnections.entries()) {
            try {
                conn.end();
            } catch (error) {
                this.log(`Error closing SSH connection for ${scriptType}: ${error.message}`);
            }
        }
        this.sshConnections.clear();
        
        // Clean up temporary files
        try {
            const tempFiles = await fs.readdir(this.tempScriptDir);
            const jsonFiles = tempFiles.filter(file => file.endsWith('.json'));
            
            for (const file of jsonFiles) {
                await fs.unlink(path.join(this.tempScriptDir, file));
            }
            
            await this.log(`Cleaned up ${jsonFiles.length} temporary files`);
        } catch (error) {
            await this.log(`Error cleaning temporary files: ${error.message}`);
        }
        
        // Remove all listeners
        this.removeAllListeners();
    }

    isHealthy() {
        const healthStatus = {
            healthy: true,
            issues: [],
            processes: this.getStatus(),
            timestamp: new Date().toISOString()
        };
        
        // Check for any issues
        const runningProcesses = Object.values(this.processStatus).filter(status => status === 'running').length;
        if (runningProcesses === 0) {
            healthStatus.issues.push('No active ROS2 processes');
        }
        
        // Check if directories exist
        try {
            require('fs').accessSync(this.tempScriptDir);
            require('fs').accessSync(this.mapDir);
        } catch (error) {
            healthStatus.issues.push('Required directories missing');
            healthStatus.healthy = false;
        }
        
        if (healthStatus.issues.length > 0) {
            healthStatus.healthy = false;
        }
        
        return healthStatus;
    }
}

module.exports = ROS2ScriptManager;
