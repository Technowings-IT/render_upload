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
            slam: 'stopped',
            navigation: 'stopped',
            robot_control: 'stopped'
        };
        this.processPIDs = {}; // Add this to store PIDs
        this.logFile = path.join(__dirname, '../../storage/logs/ros_script_manager.log');
        this.tempScriptDir = path.join(__dirname, '../../temp');
        this.mapDir = path.join(__dirname, '../../maps');
        
        // Raspberry Pi SSH configuration - can be updated via updatePiConfig
        this.piConfig = {
            host: '192.168.0.84',
            username: 'piros',
            password: 'piros',
            port: 22
        };
        
        // Pi script file paths
        this.piScriptPaths = {
            slam: '/home/piros/scripts/slam.sh',
            navigation: '/home/piros/scripts/nav2.sh',
            robot_control: '/home/piros/scripts/robot_control.sh'
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
                const checkCommand = 'ls -la /home/piros/scripts/slam.sh /home/piros/scripts/nav2.sh /home/piros/scripts/robot_control.sh 2>/dev/null | wc -l';
                
                conn.exec(checkCommand, (err, stream) => {
                    if (err) {
                        conn.end();
                        return reject(err);
                    }
                    
                    let output = '';
                    stream.on('close', (code) => {
                        conn.end();
                        const scriptCount = parseInt(output.trim());
                        resolve(scriptCount >= 3); // All 3 scripts should exist
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

    async executeScriptOnPi(scriptType, options = {}) {
        // Test SSH connectivity first
        try {
            await this.testSSHConnectivity();
        } catch (error) {
            this.log(`SSH connectivity check failed: ${error.message}`);
            throw error;
        }
        
        return new Promise((resolve, reject) => {
            const conn = new Client();
            
            conn.on('ready', () => {
                this.log(`SSH connection established with Pi for ${scriptType}`);
                
                // Use nohup instead of screen (more universally available)
                let command;
                
                switch (scriptType) {
                    case 'slam':
                        const slamScript = options.mapName 
                            ? `bash slam.sh ${options.mapName}` 
                            : 'bash slam.sh';
                        command = `cd /home/piros/scripts && nohup ${slamScript} > /tmp/${scriptType}.log 2>&1 & echo $!`;
                        break;
                        
                    case 'navigation':
                        const navScript = options.mapPath 
                            ? `bash nav2.sh ${options.mapPath}` 
                            : 'bash nav2.sh';
                        command = `cd /home/piros/scripts && nohup ${navScript} > /tmp/${scriptType}.log 2>&1 & echo $!`;
                        break;
                        
                    case 'robot_control':
                        command = `cd /home/piros/scripts && nohup bash robot_control.sh > /tmp/${scriptType}.log 2>&1 & echo $!`;
                        break;
                        
                    default:
                        conn.end();
                        return reject(new Error(`Unknown script type: ${scriptType}`));
                }
                
                this.log(`Executing on Pi: ${command}`);
                
                conn.exec(command, (err, stream) => {
                    if (err) {
                        conn.end();
                        return reject(new Error(`Script execution failed: ${err.message}`));
                    }
                    
                    let stdout = '';
                    let stderr = '';
                    
                    stream.on('close', (code) => {
                        this.log(`Pi script ${scriptType} launch finished with code ${code}`);
                        this.log(`Pi script ${scriptType} stdout: ${stdout}`);
                        this.log(`Pi script ${scriptType} stderr: ${stderr}`);
                        
                        conn.end();
                        
                        if (code === 0) {
                            // Extract PID from stdout if available
                            const pidMatch = stdout.trim().match(/\d+$/);
                            const pid = pidMatch ? pidMatch[0] : null;
                            
                            this.processStatus[scriptType] = 'running';
                            this.emit(`${scriptType}_started`);
                            
                            // Store PID for later killing
                            if (pid) {
                                this.processPIDs = this.processPIDs || {};
                                this.processPIDs[scriptType] = pid;
                            }
                            
                            resolve({
                                success: true,
                                message: `${scriptType} started successfully on Pi`,
                                stdout,
                                stderr,
                                exitCode: code,
                                pid: pid
                            });
                        } else {
                            this.processStatus[scriptType] = 'stopped';
                            reject(new Error(`Pi script failed with code ${code}: ${stderr || stdout}`));
                        }
                    });
                    
                    stream.on('data', (data) => {
                        const output = data.toString();
                        stdout += output;
                        this.log(`Pi ${scriptType} output: ${output.trim()}`);
                    });
                    
                    stream.stderr.on('data', (data) => {
                        const error = data.toString();
                        stderr += error;
                        this.log(`Pi ${scriptType} error: ${error.trim()}`);
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
                    case 'robot_control':
                        killCommands.push('pkill -f "ros2_control_robot" || true');
                        killCommands.push('pkill -f "robot_control.sh" || true');
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
    // UPDATED CONTROL METHODS FOR PI EXECUTION
    // ==========================================

    async startSLAM(options = {}) {
        if (this.processStatus.slam === 'running') {
            throw new Error('SLAM is already running');
        }

        try {
            await this.log('Starting SLAM process on Raspberry Pi...');
            
            // First ensure robot control is running
            if (this.processStatus.robot_control !== 'running') {
                await this.log('Starting robot control first for SLAM...');
                try {
                    await this.startRobotControl();
                    // Wait for robot control to initialize
                    await new Promise(resolve => setTimeout(resolve, 5000));
                } catch (robotControlError) {
                    await this.log(`Failed to start robot control for SLAM: ${robotControlError.message}`);
                    // Continue anyway - SLAM might work without robot control in simulation
                    await this.log('Continuing SLAM startup without robot control...');
                }
            }
            
            this.processStatus.slam = 'starting';
            this.emit('slam_starting', options);
            
            // Execute slam.sh on Pi (now only starts SLAM)
            const result = await this.executeScriptOnPi('slam', options);
            
            this.processStatus.slam = 'running';
            
            await this.log('SLAM started successfully on Pi');
            this.emit('slam_started', { options, piResult: result });
            
            return { 
                success: true, 
                message: 'SLAM started successfully on Raspberry Pi',
                piResult: result
            };

        } catch (error) {
            await this.log(`Failed to start SLAM on Pi: ${error.message}`);
            this.processStatus.slam = 'stopped';
            
            // Provide more specific error messages
            let userMessage = error.message;
            if (error.message.includes('SSH connectivity test failed')) {
                userMessage = 'Cannot connect to Raspberry Pi. Please check:\n' +
                             '• Pi is powered on and connected to network\n' +
                             '• SSH service is running on Pi\n' +
                             '• Network connectivity between backend and Pi\n' +
                             '• Pi IP address is correct (192.168.0.84)';
            } else if (error.message.includes('SLAM is already running')) {
                userMessage = 'SLAM is already running. Stop it first before starting again.';
            }
            
            this.emit('slam_error', userMessage);
            throw new Error(userMessage);
        }
    }

    async startNavigation(mapPath, options = {}) {
        if (this.processStatus.navigation === 'running') {
            throw new Error('Navigation is already running');
        }

        try {
            await this.log(`Starting Navigation on Raspberry Pi with map: ${mapPath}`);
            
            // First ensure robot control is running
            if (this.processStatus.robot_control !== 'running') {
                await this.log('Starting robot control first for navigation...');
                try {
                    await this.startRobotControl();
                    // Wait for robot control to initialize
                    await new Promise(resolve => setTimeout(resolve, 5000));
                } catch (robotControlError) {
                    await this.log(`Failed to start robot control for navigation: ${robotControlError.message}`);
                    // Continue anyway - navigation might work without robot control in simulation
                    await this.log('Continuing navigation startup without robot control...');
                }
            }
            
            this.processStatus.navigation = 'starting';
            this.emit('navigation_starting', { mapPath, options });
            
            // Execute nav2.sh on Pi (now only starts navigation)
            const result = await this.executeScriptOnPi('navigation', { 
                ...options, 
                mapPath 
            });
            
            this.processStatus.navigation = 'running';
            
            await this.log('Navigation started successfully on Pi');
            this.emit('navigation_started', { mapPath, options, piResult: result });
            
            return { 
                success: true, 
                message: 'Navigation started successfully on Raspberry Pi',
                mapPath,
                piResult: result
            };

        } catch (error) {
            await this.log(`Failed to start Navigation on Pi: ${error.message}`);
            this.processStatus.navigation = 'stopped';
            
            // Provide more specific error messages
            let userMessage = error.message;
            if (error.message.includes('SSH connectivity test failed')) {
                userMessage = 'Cannot connect to Raspberry Pi. Please check:\n' +
                             '• Pi is powered on and connected to network\n' +
                             '• SSH service is running on Pi\n' +
                             '• Network connectivity between backend and Pi\n' +
                             '• Pi IP address is correct (192.168.0.84)';
            } else if (error.message.includes('Navigation is already running')) {
                userMessage = 'Navigation is already running. Stop it first before starting again.';
            } else if (error.message.includes('mapPath') || error.message.includes('mapName')) {
                userMessage = 'Map file is required for navigation. Please select a valid map.';
            }
            
            this.emit('navigation_error', userMessage);
            throw new Error(userMessage);
        }
    }

    async startRobotControl() {
        if (this.processStatus.robot_control === 'running') {
            return { success: true, message: 'Robot control already running' };
        }

        try {
            await this.log('Starting Robot Control on Raspberry Pi...');
            
            this.processStatus.robot_control = 'starting';
            this.emit('robot_control_starting');
            
            // Execute robot_control.sh on Pi
            const result = await this.executeScriptOnPi('robot_control');
            
            this.processStatus.robot_control = 'running';
            
            await this.log('Robot Control started successfully on Pi');
            this.emit('robot_control_started', { piResult: result });
            
            return { 
                success: true, 
                message: 'Robot control started successfully on Raspberry Pi',
                piResult: result
            };

        } catch (error) {
            await this.log(`Failed to start Robot Control on Pi: ${error.message}`);
            this.processStatus.robot_control = 'stopped';
            this.emit('robot_control_error', error.message);
            throw error;
        }
    }

    // ==========================================
    // PROCESS MANAGEMENT FOR PI
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
    // SCRIPT CREATION METHODS (REMOVED - USING EXISTING FILES)
    // ==========================================
    
    // These methods are no longer needed since we're using existing slam.sh and nav2.sh files
    // createSLAMScript(options = {}) - REMOVED
    // createNavigationScript(mapPath, options = {}) - REMOVED
    // createRobotControlScript() - REMOVED
    // writeTemporaryScript(processName, scriptContent) - REMOVED

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

    async deployMapToRaspberryPi(mapName, piAddress = '192.168.0.84') {
        try {
            await this.log(`Deploying map ${mapName} to Raspberry Pi: ${piAddress}`);
            
            const yamlPath = path.join(this.mapDir, `${mapName}.yaml`);
            const pgmPath = path.join(this.mapDir, `${mapName}.pgm`);
            
            // Validate files exist
            await fs.access(yamlPath);
            await fs.access(pgmPath);
            
            // Copy files to Pi using SCP
            const piMapDir = '/home/piros/fleet-management-system/ros_ws/src/amr/maps/';
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
    // COMPREHENSIVE KILL METHOD
    // ==========================================

    async killAllRosProcesses() {
        return new Promise((resolve, reject) => {
            const conn = new Client();
            
            conn.on('ready', () => {
                this.log('SSH connection established for killing all ROS processes');
                
                // Comprehensive kill command for all ROS processes
                const killCommands = [
                    // Kill screen sessions
                    'screen -wipe',
                    'screen -ls | grep robot_control_session | cut -d. -f1 | awk \'{print $1}\' | xargs -r kill',
                    'screen -ls | grep slam_session | cut -d. -f1 | awk \'{print $1}\' | xargs -r kill',
                    'screen -ls | grep navigation_session | cut -d. -f1 | awk \'{print $1}\' | xargs -r kill',
                    
                    // Kill ROS processes by name
                    'pkill -f "ros2_control_robot"',
                    'pkill -f "slam_toolbox"',
                    'pkill -f "nav2_bringup"',
                    'pkill -f "robot_control.sh"',
                    'pkill -f "slam.sh"',
                    'pkill -f "nav2.sh"',
                    
                    // Kill any remaining ros2 launch processes
                    'pkill -f "ros2 launch"',
                    
                    // Kill any ros2 nodes
                    'pkill -f "ros2 run"',
                    
                    // Force kill any remaining ROS processes
                    'pkill -9 -f ros2 || true'
                ].join(' ; ');
                
                this.log(`Executing kill commands: ${killCommands}`);
                
                conn.exec(killCommands, (err, stream) => {
                    if (err) {
                        conn.end();
                        return reject(new Error(`Failed to kill processes: ${err.message}`));
                    }
                    
                    let stdout = '';
                    let stderr = '';
                    
                    stream.on('close', (code) => {
                        this.log(`Kill commands finished with code ${code}`);
                        this.log(`Kill stdout: ${stdout}`);
                        this.log(`Kill stderr: ${stderr}`);
                        
                        conn.end();
                        
                        // Always resolve since some processes might not exist
                        resolve({
                            success: true,
                            message: 'Process kill commands executed',
                            stdout,
                            stderr,
                            exitCode: code
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
            
            conn.on('error', (err) => {
                this.log(`SSH connection error for kill operation: ${err.message}`);
                reject(err);
            });
            
            conn.connect(this.piConfig);
        });
    }
}

module.exports = ROS2ScriptManager;