// websocket/messageHandler.js - Enhanced WebSocket message handling with ROS2 integration
const rosConnection = require('../ros/utils/ros_connection');
const storageManager = require('../ros/utils/storageManager');
const ROS2ScriptManager = require('../ros/utils/ros2ScriptManager');
const path = require('path');
const fs = require('fs').promises;
const { spawn } = require('child_process');

class EnhancedMessageHandler {
    constructor() {
        // Initialize ROS2 Script Manager
        this.ros2Manager = null; // Initialize as null, create on demand
        
        // Track active commands
        this.activeCommands = new Map();
        this.commandHistory = [];
        
        // Store clients reference
        this.clients = null;
    }

    // Initialize ROS2 manager on demand
    initializeROS2Manager() {
        if (!this.ros2Manager) {
            console.log('🔧 Initializing ROS2 Script Manager...');
            this.ros2Manager = new ROS2ScriptManager();
            this.setupROS2EventHandlers();
        }
        return this.ros2Manager;
    }

    setupROS2EventHandlers() {
        // Forward ROS2 events to WebSocket clients
        this.ros2Manager.on('slam_started', (data) => {
            this.broadcastToClients({
                type: 'ros2_status_update',
                data: {
                    process: 'slam',
                    status: 'started',
                    details: data,
                    timestamp: new Date().toISOString()
                }
            });
        });

        this.ros2Manager.on('navigation_started', (data) => {
            this.broadcastToClients({
                type: 'ros2_status_update',
                data: {
                    process: 'navigation',
                    status: 'started',
                    details: data,
                    timestamp: new Date().toISOString()
                }
            });
        });

        this.ros2Manager.on('slam_stopped', (code) => {
            this.broadcastToClients({
                type: 'ros2_status_update',
                data: {
                    process: 'slam',
                    status: 'stopped',
                    exitCode: code,
                    timestamp: new Date().toISOString()
                }
            });
        });

        this.ros2Manager.on('navigation_stopped', (code) => {
            this.broadcastToClients({
                type: 'ros2_status_update',
                data: {
                    process: 'navigation',
                    status: 'stopped',
                    exitCode: code,
                    timestamp: new Date().toISOString()
                }
            });
        });

        this.ros2Manager.on('emergency_stop', () => {
            this.broadcastToClients({
                type: 'emergency_stop_activated',
                data: {
                    message: 'Emergency stop activated - All ROS2 processes stopped',
                    timestamp: new Date().toISOString()
                }
            });
        });

        // Output and error forwarding
        this.ros2Manager.on('slam_output', (output) => {
            this.broadcastToClients({
                type: 'ros2_output',
                data: { process: 'slam', output, timestamp: new Date().toISOString() }
            });
        });

        this.ros2Manager.on('navigation_output', (output) => {
            this.broadcastToClients({
                type: 'ros2_output',
                data: { process: 'navigation', output, timestamp: new Date().toISOString() }
            });
        });
    }

    async handleMessage(ws, message, clients) {
        this.clients = clients; // Store clients for broadcasting
        
        try {
            const parsedMessage = typeof message === 'string' ? JSON.parse(message) : message;
            const commandId = this.generateCommandId();
            
            // Debug logging
            console.log(`📨 Received message type: "${parsedMessage.type}"`);
            console.log(`📋 Message details:`, {
                type: parsedMessage.type,
                deviceId: parsedMessage.deviceId,
                command: parsedMessage.command,
                hasData: !!parsedMessage.data,
                hasOptions: !!parsedMessage.options
            });
            
            // Log command
            this.logCommand(commandId, parsedMessage);
            
            switch (parsedMessage.type) {
                // ==========================================
                // SCRIPT CONTROL COMMANDS (PRIORITY)
                // ==========================================
                case 'script_command':
                    await this.handleScriptCommand(ws, parsedMessage, commandId);
                    break;

                case 'get_script_status':
                    await this.handleGetScriptStatus(ws, parsedMessage, commandId);
                    break;

                // ==========================================
                // ROS2 SCRIPT CONTROL COMMANDS
                // ==========================================
                case 'start_slam':
                    await this.handleStartSLAM(ws, parsedMessage.data, commandId);
                    break;
                    
                case 'start_navigation':
                    await this.handleStartNavigation(ws, parsedMessage.data, commandId);
                    break;
                    
                case 'stop_ros2_process':
                    await this.handleStopProcess(ws, parsedMessage.data, commandId);
                    break;

                case 'emergency_stop':
                    await this.handleEmergencyStop(ws, commandId);
                    break;
                    
                case 'get_ros2_status':
                    await this.handleGetROS2Status(ws, commandId);
                    break;

                case 'pi_diagnostics':
                    await this.handlePiDiagnostics(ws, commandId);
                    break;

                case 'update_pi_config':
                    await this.handleUpdatePiConfig(ws, parsedMessage.data, commandId);
                    break;

                case 'restart_ros2_process':
                    await this.handleRestartProcess(ws, parsedMessage.data, commandId);
                    break;
                    
                // ==========================================
                // MAP MANAGEMENT COMMANDS
                // ==========================================
                case 'upload_map':
                    await this.handleMapUpload(ws, parsedMessage.data, commandId);
                    break;

                case 'convert_map':
                    await this.handleMapConversion(ws, parsedMessage.data, commandId);
                    break;

                case 'deploy_map':
                    await this.handleMapDeployment(ws, parsedMessage.data, commandId);
                    break;
                    
                // ==========================================
                // ORDER MANAGEMENT COMMANDS
                // ==========================================
                case 'create_order':
                    await this.handleCreateOrder(ws, parsedMessage.data, commandId);
                    break;
                    
                case 'get_orders':
                    await this.handleGetOrders(ws, commandId);
                    break;
                    
                case 'update_order_status':
                    await this.handleUpdateOrderStatus(ws, parsedMessage.data, commandId);
                    break;
                    
                // ==========================================
                // EXISTING MESSAGE TYPES
                // ==========================================
                case 'control_command':
                    this.handleControlCommand(ws, parsedMessage);
                    break;
                
                case 'joystick_control':
                    this.handleJoystickControl(ws, parsedMessage);
                    break;
                
                case 'mapping_command':
                    this.handleMappingCommand(ws, parsedMessage);
                    break;
                
                case 'map_edit':
                    this.handleMapEdit(ws, parsedMessage);
                    break;
                
                case 'order_command':
                    this.handleOrderCommand(ws, parsedMessage);
                    break;
                
                case 'device_command':
                    this.handleDeviceCommand(ws, parsedMessage);
                    break;
                
                case 'subscribe':
                    this.handleSubscription(ws, parsedMessage);
                    break;
                
                case 'unsubscribe':
                    this.handleUnsubscription(ws, parsedMessage);
                    break;
                
                case 'request_data':
                    this.handleDataRequest(ws, parsedMessage);
                    break;

                case 'ping':
                    this.handlePing(ws, parsedMessage);
                    break;

                case 'pong':
                    this.handlePong(ws, parsedMessage);
                    break;
                    
                default:
                    console.log(`❓ Unknown message type: "${parsedMessage.type}"`);
                    console.log('📋 Available handlers:', [
                        'script_command', 'get_script_status', 'start_slam', 'start_navigation',
                        'stop_ros2_process', 'emergency_stop', 'get_ros2_status', 'restart_ros2_process',
                        'upload_map', 'convert_map', 'deploy_map', 'create_order', 'get_orders',
                        'update_order_status', 'control_command', 'joystick_control', 'mapping_command',
                        'map_edit', 'order_command', 'device_command', 'subscribe', 'unsubscribe',
                        'request_data', 'ping', 'pong'
                    ]);
                    await this.handleOtherMessages(ws, parsedMessage);
            }
            
        } catch (error) {
            console.error('❌ Message handling error:', error);
            console.error('📋 Error stack:', error.stack);
            this.sendErrorResponse(ws, error.message, commandId);
        }
    }

    // ==========================================
    // SCRIPT CONTROL HANDLERS
    // ==========================================
    async handleScriptCommand(ws, message, commandId) {
        try {
            console.log(`🤖 Processing script command:`, {
                commandId,
                type: message.type,
                deviceId: message.deviceId,
                command: message.command,
                options: message.options
            });
            
            const { deviceId, command, options = {} } = message;
            
            if (!deviceId) {
                throw new Error('deviceId is required for script commands');
            }
            
            if (!command) {
                throw new Error('command is required for script commands');
            }
            
            this.setCommandActive(commandId, 'script_command');
            
            // Initialize ROS2 manager if needed
            let ros2Manager;
            try {
                ros2Manager = this.initializeROS2Manager();
            } catch (error) {
                throw new Error(`Failed to initialize ROS2 manager: ${error.message}`);
            }
            
            // Use the new integrated handleScriptCommand method
            let result;
            
            try {
                result = await ros2Manager.handleScriptCommand(command, options);
                
                // Broadcast appropriate status based on command
                switch (command) {
                    case 'start_robot_control':
                        this.broadcastScriptStatus(deviceId, 'robot_control', 'running', 'Robot control started successfully');
                        break;
                    case 'start_slam':
                        this.broadcastScriptStatus(deviceId, 'slam', 'running', 'SLAM started successfully');
                        break;
                    case 'start_navigation':
                        this.broadcastScriptStatus(deviceId, 'navigation', 'running', 'Navigation started successfully');
                        break;
                    case 'stop_all_scripts':
                        this.broadcastScriptStatus(deviceId, 'all_scripts', 'stopped', 'All scripts stopped successfully');
                        ['robot_control', 'slam', 'navigation'].forEach(scriptType => {
                            this.broadcastScriptStatus(deviceId, scriptType, 'stopped', `${scriptType} stopped`);
                        });
                        break;
                }
                
            } catch (error) {
                console.error('❌ Script command failed:', error);
                throw error;
            }
            
            this.setCommandCompleted(commandId, result);
            
            const response = {
                type: 'script_command_response',
                commandId: commandId,
                success: true,
                data: result,
                command: command,
                deviceId: deviceId,
                timestamp: new Date().toISOString(),
                ...(result.warnings && { warnings: result.warnings }),
                ...(result.note && { note: result.note })
            };
            
            ws.send(JSON.stringify(response));
            
            // Log completion with details
            if (result.warnings && result.warnings.length > 0) {
                console.log(`⚠️ Script command ${command} completed with warnings:`, result.warnings);
                console.log(`ℹ️ Script status: ${result.message}`);
            } else {
                console.log(`✅ Script command ${command} completed successfully`);
            }
            
        } catch (error) {
            this.setCommandFailed(commandId, error);
            console.error('❌ Error executing script command:', error);
            console.error('📋 Error details:', {
                message: error.message,
                stack: error.stack,
                command: message.command,
                deviceId: message.deviceId
            });
            
            const errorResponse = {
                type: 'script_command_response',
                commandId: commandId,
                success: false,
                error: error.message,
                command: message.command || 'unknown',
                deviceId: message.deviceId || 'unknown',
                timestamp: new Date().toISOString()
            };
            
            ws.send(JSON.stringify(errorResponse));
        }
    }

    async handleGetScriptStatus(ws, message, commandId) {
        try {
            const { deviceId } = message;
            
            // Initialize ROS2 manager if needed
            const ros2Manager = this.initializeROS2Manager();
            
            const status = ros2Manager.getDetailedStatus();
            
            ws.send(JSON.stringify({
                type: 'script_status_response',
                commandId: commandId,
                success: true,
                deviceId: deviceId,
                data: status,
                timestamp: new Date().toISOString()
            }));
            
        } catch (error) {
            console.error('❌ Error getting script status:', error);
            
            ws.send(JSON.stringify({
                type: 'script_status_response',
                commandId: commandId,
                success: false,
                error: error.message,
                deviceId: message.deviceId || 'unknown',
                timestamp: new Date().toISOString()
            }));
        }
    }

    broadcastScriptStatus(deviceId, scriptType, status, message) {
        try {
            console.log(`📡 Broadcasting script status: ${scriptType} - ${status}`);
            
            const statusMessage = {
                type: 'broadcast',
                topic: 'script_status',
                data: {
                    type: 'script_status_update',
                    deviceId: deviceId,
                    script: scriptType,
                    status: status,
                    message: message,
                    timestamp: new Date().toISOString()
                }
            };
            
            this.broadcastToClients(statusMessage);
            
        } catch (error) {
            console.error('❌ Error broadcasting script status:', error);
        }
    }

    // ==========================================
    // ROS2 SCRIPT CONTROL HANDLERS
    // ==========================================

    async handleStartSLAM(ws, data, commandId) {
        try {
            console.log(`🗺️ Starting SLAM mode - Command ID: ${commandId}`);
            
            this.setCommandActive(commandId, 'start_slam');
            
            const options = {
                mapName: data?.mapName || 'new_slam_map',
                useSimTime: data?.useSimTime || false
            };
            
            const ros2Manager = this.initializeROS2Manager();
            const result = await ros2Manager.startSLAM(options);
            
            this.setCommandCompleted(commandId, result);
            
            ws.send(JSON.stringify({
                type: 'slam_command_response',
                commandId: commandId,
                success: true,
                data: result,
                message: 'SLAM mode started successfully',
                timestamp: new Date().toISOString()
            }));
            
            // Broadcast status update
            this.broadcastROS2Status();
            
        } catch (error) {
            this.setCommandFailed(commandId, error);
            console.error('❌ Error starting SLAM:', error);
            
            ws.send(JSON.stringify({
                type: 'slam_command_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    async handleStartNavigation(ws, data, commandId) {
        try {
            console.log(`🚀 Starting Navigation mode - Command ID: ${commandId}`);
            
            this.setCommandActive(commandId, 'start_navigation');
            
            // Validate required data
            if (!data?.mapPath && !data?.mapName) {
                throw new Error('Either mapPath or mapName must be provided for navigation');
            }
            
            const mapPath = data.mapPath || data.mapName;
            const options = {
                useSimTime: data?.useSimTime || false,
                navParamsFile: data?.navParamsFile
            };
            
            const ros2Manager = this.initializeROS2Manager();
            const result = await ros2Manager.startNavigation(mapPath, options);
            
            this.setCommandCompleted(commandId, result);
            
            ws.send(JSON.stringify({
                type: 'navigation_command_response',
                commandId: commandId,
                success: true,
                data: result,
                message: 'Navigation mode started successfully',
                timestamp: new Date().toISOString()
            }));
            
            // Broadcast status update
            this.broadcastROS2Status();
            
        } catch (error) {
            this.setCommandFailed(commandId, error);
            console.error('❌ Error starting Navigation:', error);
            
            ws.send(JSON.stringify({
                type: 'navigation_command_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    async handleStopProcess(ws, data, commandId) {
        try {
            const { processName } = data;
            console.log(`🛑 Stopping ROS2 process: ${processName} - Command ID: ${commandId}`);
            
            this.setCommandActive(commandId, 'stop_process');
            
            const validProcesses = ['slam', 'navigation', 'robot_control', 'all'];
            if (!validProcesses.includes(processName)) {
                throw new Error(`Invalid process name. Valid options: ${validProcesses.join(', ')}`);
            }
            
            const ros2Manager = this.initializeROS2Manager();
            
            let result;
            if (processName === 'all') {
                result = await ros2Manager.stopAll();
            } else {
                result = await ros2Manager.stopProcess(processName);
            }
            
            this.setCommandCompleted(commandId, result);
            
            ws.send(JSON.stringify({
                type: 'stop_process_response',
                commandId: commandId,
                success: true,
                data: result,
                processName: processName,
                timestamp: new Date().toISOString()
            }));
            
            // Broadcast status update
            this.broadcastROS2Status();
            
        } catch (error) {
            this.setCommandFailed(commandId, error);
            console.error('❌ Error stopping process:', error);
            
            ws.send(JSON.stringify({
                type: 'stop_process_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    async handleEmergencyStop(ws, commandId) {
        try {
            console.log(`🚨 EMERGENCY STOP activated - Command ID: ${commandId}`);
            
            this.setCommandActive(commandId, 'emergency_stop');
            
            const ros2Manager = this.initializeROS2Manager();
            const result = await ros2Manager.emergencyStop();
            
            this.setCommandCompleted(commandId, result);
            
            ws.send(JSON.stringify({
                type: 'emergency_stop_response',
                commandId: commandId,
                success: true,
                data: result,
                message: 'Emergency stop completed',
                timestamp: new Date().toISOString()
            }));
            
            // Broadcast emergency stop to all clients
            this.broadcastToClients({
                type: 'emergency_stop_activated',
                data: {
                    activatedBy: commandId,
                    timestamp: new Date().toISOString()
                }
            });
            
        } catch (error) {
            this.setCommandFailed(commandId, error);
            console.error('❌ Error during emergency stop:', error);
            
            ws.send(JSON.stringify({
                type: 'emergency_stop_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    async handleGetROS2Status(ws, commandId) {
        try {
            const ros2Manager = this.initializeROS2Manager();
            const status = ros2Manager.getDetailedStatus();
            
            ws.send(JSON.stringify({
                type: 'ros2_status_response',
                commandId: commandId,
                success: true,
                data: status,
                timestamp: new Date().toISOString()
            }));
            
        } catch (error) {
            console.error('❌ Error getting ROS2 status:', error);
            
            ws.send(JSON.stringify({
                type: 'ros2_status_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    async handlePiDiagnostics(ws, commandId) {
        try {
            console.log('🔍 Running Pi diagnostics...');
            const ros2Manager = this.initializeROS2Manager();
            const diagnostics = await ros2Manager.checkPiStatus();
            
            console.log('📋 Pi diagnostics completed:', diagnostics);
            
            ws.send(JSON.stringify({
                type: 'pi_diagnostics_response',
                commandId: commandId,
                success: true,
                data: diagnostics,
                timestamp: new Date().toISOString()
            }));
            
        } catch (error) {
            console.error('❌ Error running Pi diagnostics:', error);
            
            ws.send(JSON.stringify({
                type: 'pi_diagnostics_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    async handleUpdatePiConfig(ws, data, commandId) {
        try {
            console.log('🔧 Updating Pi configuration...', data);
            const ros2Manager = this.initializeROS2Manager();
            const updatedConfig = ros2Manager.updatePiConfig(data);
            
            // Test the new configuration
            const testResult = await ros2Manager.testSSHConnectivity();
            
            ws.send(JSON.stringify({
                type: 'pi_config_update_response',
                commandId: commandId,
                success: true,
                data: {
                    config: updatedConfig,
                    connectionTest: testResult
                },
                message: 'Pi configuration updated successfully',
                timestamp: new Date().toISOString()
            }));
            
        } catch (error) {
            console.error('❌ Error updating Pi configuration:', error);
            
            ws.send(JSON.stringify({
                type: 'pi_config_update_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    async handleRestartProcess(ws, data, commandId) {
        try {
            const { processName, options = {} } = data;
            console.log(`🔄 Restarting ROS2 process: ${processName} - Command ID: ${commandId}`);
            
            this.setCommandActive(commandId, 'restart_process');
            
            const ros2Manager = this.initializeROS2Manager();
            const result = await ros2Manager.restart(processName, options);
            
            this.setCommandCompleted(commandId, result);
            
            ws.send(JSON.stringify({
                type: 'restart_process_response',
                commandId: commandId,
                success: true,
                data: result,
                processName: processName,
                timestamp: new Date().toISOString()
            }));
            
            // Broadcast status update
            this.broadcastROS2Status();
            
        } catch (error) {
            this.setCommandFailed(commandId, error);
            console.error('❌ Error restarting process:', error);
            
            ws.send(JSON.stringify({
                type: 'restart_process_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    // ==========================================
    // MAP MANAGEMENT HANDLERS
    // ==========================================

    async handleMapUpload(ws, data, commandId) {
        try {
            console.log(`📂 Processing map upload - Command ID: ${commandId}`);
            
            this.setCommandActive(commandId, 'map_upload');
            
            const { mapData, mapName, deployToPi = false } = data;
            
            if (!mapData || !mapName) {
                throw new Error('mapData and mapName are required');
            }
            
            // Convert JSON map to PGM+YAML format
            const conversionResult = await this.ros2Manager.convertAndPrepareMap(mapData, mapName);
            
            let deploymentResult = null;
            if (deployToPi) {
                deploymentResult = await this.ros2Manager.deployMapToRaspberryPi(mapName);
            }
            
            const result = {
                conversion: conversionResult,
                deployment: deploymentResult,
                mapName: mapName
            };
            
            this.setCommandCompleted(commandId, result);
            
            ws.send(JSON.stringify({
                type: 'map_upload_response',
                commandId: commandId,
                success: true,
                data: result,
                message: deployToPi ? 'Map converted and deployed to Pi' : 'Map converted successfully',
                timestamp: new Date().toISOString()
            }));
            
        } catch (error) {
            this.setCommandFailed(commandId, error);
            console.error('❌ Error uploading map:', error);
            
            ws.send(JSON.stringify({
                type: 'map_upload_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    async handleMapConversion(ws, data, commandId) {
        try {
            console.log(`🔄 Converting map - Command ID: ${commandId}`);
            
            this.setCommandActive(commandId, 'map_conversion');
            
            const { mapData, mapName } = data;
            
            if (!mapData || !mapName) {
                throw new Error('mapData and mapName are required for conversion');
            }
            
            const result = await this.ros2Manager.convertAndPrepareMap(mapData, mapName);
            
            this.setCommandCompleted(commandId, result);
            
            ws.send(JSON.stringify({
                type: 'map_conversion_response',
                commandId: commandId,
                success: true,
                data: result,
                message: 'Map converted to PGM+YAML format',
                timestamp: new Date().toISOString()
            }));
            
        } catch (error) {
            this.setCommandFailed(commandId, error);
            console.error('❌ Error converting map:', error);
            
            ws.send(JSON.stringify({
                type: 'map_conversion_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    async handleMapDeployment(ws, data, commandId) {
        try {
            console.log(`🚀 Deploying map to Pi - Command ID: ${commandId}`);
            
            this.setCommandActive(commandId, 'map_deployment');
            
            const { mapName, piAddress } = data;
            
            if (!mapName) {
                throw new Error('mapName is required for deployment');
            }
            
            const result = await this.ros2Manager.deployMapToRaspberryPi(mapName, piAddress);
            
            this.setCommandCompleted(commandId, result);
            
            ws.send(JSON.stringify({
                type: 'map_deployment_response',
                commandId: commandId,
                success: true,
                data: result,
                message: 'Map deployed to Raspberry Pi successfully',
                timestamp: new Date().toISOString()
            }));
            
        } catch (error) {
            this.setCommandFailed(commandId, error);
            console.error('❌ Error deploying map:', error);
            
            ws.send(JSON.stringify({
                type: 'map_deployment_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    // ==========================================
    // ORDER MANAGEMENT HANDLERS
    // ==========================================

    async handleCreateOrder(ws, data, commandId) {
        try {
            console.log(`📋 Creating order - Command ID: ${commandId}`);
            
            const ordersPath = path.join(__dirname, '../storage/orders.json');
            let orders = [];
            
            try {
                const ordersData = await fs.readFile(ordersPath, 'utf8');
                orders = JSON.parse(ordersData);
            } catch (error) {
                // File doesn't exist, start with empty array
            }
            
            const newOrder = {
                id: `order_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
                ...data,
                status: 'pending',
                createdAt: new Date().toISOString(),
                updatedAt: new Date().toISOString(),
                commandId: commandId
            };
            
            orders.push(newOrder);
            await fs.mkdir(path.dirname(ordersPath), { recursive: true });
            await fs.writeFile(ordersPath, JSON.stringify(orders, null, 2));
            
            ws.send(JSON.stringify({
                type: 'create_order_response',
                commandId: commandId,
                success: true,
                data: newOrder,
                message: 'Order created successfully',
                timestamp: new Date().toISOString()
            }));
            
            // Broadcast to all clients
            this.broadcastToClients({
                type: 'new_order_created',
                data: newOrder
            });
            
        } catch (error) {
            console.error('❌ Error creating order:', error);
            
            ws.send(JSON.stringify({
                type: 'create_order_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    async handleGetOrders(ws, commandId) {
        try {
            const ordersPath = path.join(__dirname, '../storage/orders.json');
            let orders = [];
            
            try {
                const ordersData = await fs.readFile(ordersPath, 'utf8');
                orders = JSON.parse(ordersData);
            } catch (error) {
                // File doesn't exist, return empty array
            }
            
            ws.send(JSON.stringify({
                type: 'orders_response',
                commandId: commandId,
                success: true,
                data: orders,
                count: orders.length,
                timestamp: new Date().toISOString()
            }));
            
        } catch (error) {
            console.error('❌ Error getting orders:', error);
            
            ws.send(JSON.stringify({
                type: 'orders_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    async handleUpdateOrderStatus(ws, data, commandId) {
        try {
            const { orderId, status, details } = data;
            console.log(`📝 Updating order ${orderId} status to ${status} - Command ID: ${commandId}`);
            
            const ordersPath = path.join(__dirname, '../storage/orders.json');
            let orders = [];
            
            try {
                const ordersData = await fs.readFile(ordersPath, 'utf8');
                orders = JSON.parse(ordersData);
            } catch (error) {
                throw new Error('Orders file not found');
            }
            
            const orderIndex = orders.findIndex(order => order.id === orderId);
            if (orderIndex === -1) {
                throw new Error('Order not found');
            }
            
            orders[orderIndex] = {
                ...orders[orderIndex],
                status: status,
                updatedAt: new Date().toISOString(),
                ...(details && { details })
            };
            
            await fs.writeFile(ordersPath, JSON.stringify(orders, null, 2));
            
            ws.send(JSON.stringify({
                type: 'update_order_response',
                commandId: commandId,
                success: true,
                data: orders[orderIndex],
                message: 'Order status updated successfully',
                timestamp: new Date().toISOString()
            }));
            
            // Broadcast to all clients
            this.broadcastToClients({
                type: 'order_status_updated',
                data: orders[orderIndex]
            });
            
        } catch (error) {
            console.error('❌ Error updating order status:', error);
            
            ws.send(JSON.stringify({
                type: 'update_order_response',
                commandId: commandId,
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            }));
        }
    }

    // ==========================================
    // EXISTING CONTROL COMMAND HANDLERS
    // ==========================================

    handleControlCommand(ws, message) {
        const { deviceId, command, data } = message;
        
        if (!deviceId) {
            return this.sendError(ws, 'Device ID is required for control commands');
        }
        
        console.log(`🎮 Control command: ${command} for device ${deviceId}`);
        
        let result;
        
        switch (command) {
            case 'move':
                result = this.handleMoveCommand(deviceId, data);
                break;
            
            case 'stop':
                result = rosConnection.emergencyStop();
                break;
            
            case 'goal':
                result = rosConnection.publishGoal(data.x, data.y, data.orientation || 0);
                break;
            
            case 'start_mapping':
                result = rosConnection.startMapping();
                break;
            
            case 'stop_mapping':
                result = rosConnection.stopMapping();
                break;
            
            default:
                return this.sendError(ws, `Unknown control command: ${command}`);
        }
        
        // Send response to client
        ws.send(JSON.stringify({
            type: 'control_response',
            deviceId: deviceId,
            command: command,
            result: result,
            timestamp: new Date().toISOString()
        }));
        
        // Broadcast to other subscribers
        this.broadcastToClients({
            type: 'control_events',
            data: {
                type: `${command}_executed`,
                deviceId: deviceId,
                data: data,
                result: result,
                timestamp: new Date().toISOString()
            }
        });
    }

    handleMoveCommand(deviceId, data) {
        const { linear, angular, deadman } = data;
        
        if (deadman !== undefined) {
            // Joystick-style control with deadman switch
            return rosConnection.publishJoystick(angular || 0, linear || 0, deadman);
        } else {
            // Direct velocity control
            return rosConnection.publishVelocity(linear || 0, angular || 0);
        }
    }

    // ==========================================
    // JOYSTICK CONTROL HANDLER
    // ==========================================

    handleJoystickControl(ws, message) {
        const { deviceId, x, y, deadman } = message;
        
        if (!deviceId) {
            return this.sendError(ws, 'Device ID is required for joystick control');
        }
        
        // Validate joystick input
        if (typeof x !== 'number' || typeof y !== 'number') {
            return this.sendError(ws, 'Invalid joystick input: x and y must be numbers');
        }
        
        // Clamp values to safe range
        const clampedX = Math.max(-1, Math.min(1, x));
        const clampedY = Math.max(-1, Math.min(1, y));
        
        const result = rosConnection.publishJoystick(clampedX, clampedY, deadman);
        
        // Send immediate response (for real-time control)
        ws.send(JSON.stringify({
            type: 'joystick_response',
            deviceId: deviceId,
            result: result,
            timestamp: new Date().toISOString()
        }));
        
        // Broadcast to other clients (less frequently to avoid spam)
        if (!global.lastJoystickBroadcast || Date.now() - global.lastJoystickBroadcast > 200) {
            this.broadcastToClients({
                type: 'control_events',
                data: {
                    type: 'joystick_update',
                    deviceId: deviceId,
                    data: { x: clampedX, y: clampedY, deadman },
                    timestamp: new Date().toISOString()
                }
            });
            
            global.lastJoystickBroadcast = Date.now();
        }
    }

    // ==========================================
    // BASIC MESSAGE HANDLERS
    // ==========================================

    handlePing(ws, message) {
        ws.send(JSON.stringify({
            type: 'pong',
            timestamp: new Date().toISOString(),
            originalTimestamp: message.timestamp
        }));
    }

    handlePong(ws, message) {
        console.log('🏓 Received pong from client');
    }

    // ==========================================
    // UTILITY METHODS
    // ==========================================

    generateCommandId() {
        return `cmd_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    }

    logCommand(commandId, message) {
        const logEntry = {
            commandId,
            type: message.type,
            timestamp: new Date().toISOString(),
            data: message.data
        };
        
        this.commandHistory.push(logEntry);
        
        // Keep only last 100 commands
        if (this.commandHistory.length > 100) {
            this.commandHistory.shift();
        }
        
        console.log(`📝 Command logged: ${commandId} - ${message.type}`);
    }

    setCommandActive(commandId, type) {
        this.activeCommands.set(commandId, {
            type,
            status: 'active',
            startTime: new Date().toISOString()
        });
    }

    setCommandCompleted(commandId, result) {
        if (this.activeCommands.has(commandId)) {
            this.activeCommands.set(commandId, {
                ...this.activeCommands.get(commandId),
                status: 'completed',
                endTime: new Date().toISOString(),
                result
            });
        }
    }

    setCommandFailed(commandId, error) {
        if (this.activeCommands.has(commandId)) {
            this.activeCommands.set(commandId, {
                ...this.activeCommands.get(commandId),
                status: 'failed',
                endTime: new Date().toISOString(),
                error: error.message
            });
        }
    }

    broadcastROS2Status() {
        try {
            if (!this.ros2Manager) {
                console.warn('⚠️ ROS2 manager not initialized, cannot broadcast status');
                return;
            }
            
            const status = this.ros2Manager.getStatus();
            
            // Broadcast to all connected WebSocket clients
            this.broadcastToClients({
                type: 'ros2_status_update',
                data: status,
                timestamp: new Date().toISOString()
            });
        } catch (error) {
            console.error('❌ Error broadcasting ROS2 status:', error);
        }
    }

    sendError(ws, message) {
        this.sendErrorResponse(ws, message);
    }

    broadcastToClients(message) {
        try {
            // Use the stored clients reference
            if (this.clients) {
                this.clients.forEach(client => {
                    try {
                        if (client.readyState === 1) { // WebSocket.OPEN
                            client.send(JSON.stringify(message));
                        }
                    } catch (error) {
                        console.error('❌ Error sending to client:', error);
                    }
                });
            } else if (global.webSocketInstance && global.webSocketInstance.broadcastToSubscribers) {
                global.webSocketInstance.broadcastToSubscribers(message);
            }
        } catch (error) {
            console.error('❌ Error in broadcastToClients:', error);
        }
    }

    // ==========================================
    // PLACEHOLDER HANDLERS (implement as needed)
    // ==========================================

    handleMappingCommand(ws, message) {
        console.log('🗺️ Mapping command received:', message);
        ws.send(JSON.stringify({
            type: 'mapping_response',
            success: true,
            message: 'Mapping command processed',
            timestamp: new Date().toISOString()
        }));
    }

    handleMapEdit(ws, message) {
        console.log('✏️ Map edit command received:', message);
        ws.send(JSON.stringify({
            type: 'map_edit_response',
            success: true,
            message: 'Map edit processed',
            timestamp: new Date().toISOString()
        }));
    }

    handleOrderCommand(ws, message) {
        console.log('📋 Order command received:', message);
        ws.send(JSON.stringify({
            type: 'order_response',
            success: true,
            message: 'Order command processed',
            timestamp: new Date().toISOString()
        }));
    }

    handleDeviceCommand(ws, message) {
        console.log('📱 Device command received:', message);
        ws.send(JSON.stringify({
            type: 'device_response',
            success: true,
            message: 'Device command processed',
            timestamp: new Date().toISOString()
        }));
    }

    handleSubscription(ws, message) {
        console.log('📡 Subscription request:', message);
        ws.send(JSON.stringify({
            type: 'subscription_response',
            success: true,
            message: 'Subscription processed',
            timestamp: new Date().toISOString()
        }));
    }

    handleUnsubscription(ws, message) {
        console.log('📡 Unsubscription request:', message);
        ws.send(JSON.stringify({
            type: 'unsubscription_response',
            success: true,
            message: 'Unsubscription processed',
            timestamp: new Date().toISOString()
        }));
    }

    handleDataRequest(ws, message) {
        console.log('📊 Data request:', message);
        ws.send(JSON.stringify({
            type: 'data_response',
            success: true,
            message: 'Data request processed',
            timestamp: new Date().toISOString()
        }));
    }

    async handleOtherMessages(ws, message) {
        console.log('❓ Unknown message type:', message.type);
        this.sendError(ws, `Unknown message type: ${message.type}`);
    }

    // ==========================================
    // CLEANUP
    // ==========================================

    async cleanup() {
        console.log('🧹 Cleaning up Enhanced Message Handler...');
        
        // Stop ROS2 manager
        if (this.ros2Manager) {
            await this.ros2Manager.cleanup();
            this.ros2Manager = null;
        }
        
        // Clear active commands
        this.activeCommands.clear();
        this.commandHistory = [];
        
        console.log('✅ Enhanced Message Handler cleanup completed');
    }
}

module.exports = EnhancedMessageHandler;