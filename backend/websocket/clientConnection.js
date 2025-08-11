// websocket/clientConnection.js - FIXED WebSocket with proper mapping command handling
const WebSocket = require('ws');
const config = require('../config');

let wss;
const connectedClients = new Map();
const clientSubscriptions = new Map();
const pingTimers = new Map();

function initializeWebSocketServer(server) {
    wss = new WebSocket.Server({ 
        server,
        perMessageDeflate: config.WEBSOCKET.COMPRESSION,
        clientTracking: true,
        maxPayload: 16 * 1024,
        skipUTF8Validation: false,
    });

    wss.on('connection', (ws, req) => {
        const clientId = generateClientId();
        const clientInfo = {
            id: clientId,
            ws: ws,
            connectedAt: new Date(),
            lastHeartbeat: new Date(),
            lastPong: new Date(),
            subscribedTopics: new Set(),
            ipAddress: req.connection.remoteAddress || req.socket.remoteAddress,
            userAgent: req.headers['user-agent'],
            isAlive: true,
            subscriptions: {
                devices: new Set(),
                topics: new Set(),
                events: new Set()
            }
        };

        connectedClients.set(clientId, clientInfo);
        clientSubscriptions.set(clientId, new Set());

        console.log(`📱 Client ${clientId} connected from ${clientInfo.ipAddress}. Total clients: ${connectedClients.size}`);

        setupWebSocketHandlers(ws, clientId, clientInfo);
        startPingPong(clientId);
        sendWelcomeMessage(clientId);
        sendInitialData(clientId);
    });

    setInterval(cleanupConnections, 30000);
    console.log('✅ WebSocket server initialized with enhanced stability');
}

function setupWebSocketHandlers(ws, clientId, clientInfo) {
    ws.on('message', (data) => {
        try {
            clientInfo.lastHeartbeat = new Date();
            const message = JSON.parse(data);
            handleClientMessage(clientId, message);
        } catch (error) {
            console.error(`❌ Error parsing message from client ${clientId}:`, error);
            sendToClient(clientId, {
                type: 'error',
                message: 'Invalid message format',
                details: error.message,
                timestamp: new Date().toISOString()
            });
        }
    });

    ws.on('pong', (data) => {
        const client = connectedClients.get(clientId);
        if (client) {
            client.isAlive = true;
            client.lastPong = new Date();
            console.log(`💓 Pong received from ${clientId}`);
        }
    });

    ws.on('close', (code, reason) => {
        console.log(`📱 Client ${clientId} disconnected (code: ${code}, reason: ${reason})`);
        stopPingPong(clientId);
        connectedClients.delete(clientId);
        clientSubscriptions.delete(clientId);
    });

    ws.on('error', (error) => {
        console.error(`❌ WebSocket error for client ${clientId}:`, error);
        stopPingPong(clientId);
        connectedClients.delete(clientId);
        clientSubscriptions.delete(clientId);
    });
}

function startPingPong(clientId) {
    const pingTimer = setInterval(() => {
        const client = connectedClients.get(clientId);
        if (!client) {
            clearInterval(pingTimer);
            return;
        }

        if (!client.isAlive) {
            console.log(`💔 Client ${clientId} failed ping test, terminating`);
            client.ws.terminate();
            connectedClients.delete(clientId);
            clientSubscriptions.delete(clientId);
            clearInterval(pingTimer);
            return;
        }

        client.isAlive = false;
        try {
            if (client.ws.readyState === WebSocket.OPEN) {
                client.ws.ping('ping');
                console.log(`🏓 Ping sent to ${clientId}`);
            }
        } catch (error) {
            console.error(`❌ Error sending ping to ${clientId}:`, error);
            connectedClients.delete(clientId);
            clientSubscriptions.delete(clientId);
            clearInterval(pingTimer);
        }
    }, config.WEBSOCKET.PING_INTERVAL);

    pingTimers.set(clientId, pingTimer);
}

function stopPingPong(clientId) {
    const pingTimer = pingTimers.get(clientId);
    if (pingTimer) {
        clearInterval(pingTimer);
        pingTimers.delete(clientId);
    }
}
// ADD THIS NEW FUNCTION to clientConnection.js (after the existing handlers)

function handleScriptCommand(clientId, message) {
    try {
        console.log(`🤖 Script command from ${clientId}:`, message);
        const { deviceId, command, options = {} } = message;
        
        if (!deviceId) {
            throw new Error('Missing deviceId in script command');
        }
        
        if (!command) {
            throw new Error('Missing command in script request');
        }

        // Import and use the message handler
        const EnhancedMessageHandler = require('./messageHandler');
        const messageHandler = new EnhancedMessageHandler();
        
        // Create a mock WebSocket object for the handler
        const ws = {
            send: (data) => {
                const client = connectedClients.get(clientId);
                if (client && client.ws.readyState === 1) {
                    client.ws.send(data);
                }
            }
        };
        
        // Call the enhanced message handler
        messageHandler.handleScriptCommand(ws, message, generateCommandId())
            .catch(error => {
                console.error('❌ Error in script command handler:', error);
                sendToClient(clientId, {
                    type: 'error',
                    message: 'Script command failed',
                    details: error.message,
                    timestamp: new Date().toISOString()
                });
            });
        
    } catch (error) {
        console.error('❌ Error handling script command:', error);
        sendToClient(clientId, {
            type: 'error',
            message: 'Failed to process script command',
            details: error.message,
            timestamp: new Date().toISOString()
        });
    }
}

// ADD THIS HELPER FUNCTION if it doesn't exist
function generateCommandId() {
    return `cmd_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}
// ✅ FIXED: Enhanced message handling with proper string matching
function handleClientMessage(clientId, message) {
    const client = connectedClients.get(clientId);
    if (!client) return;

    console.log(`📨 Message from ${clientId}: ${message.type}`);

    switch (message.type) {
        case 'ping':
            sendToClient(clientId, {
                type: 'pong',
                timestamp: new Date().toISOString()
            });
            break;
            
        case 'pong':
            console.log(`🏓 Application pong received from ${clientId}`);
            break;
            
        case 'heartbeat':
            handleHeartbeat(clientId);
            break;
            
        case 'device_connect':
            handleDeviceConnectMessage(clientId, message);
            break;
            
        case 'subscribe':
            handleSubscription(clientId, message);
            break;
            
        case 'unsubscribe':
            handleUnsubscription(clientId, message);
            break;
        case 'script_command':
            handleScriptCommand(clientId, message);
            break;
            
        // ✅ FIXED: Use direct string matching instead of config constants
        case 'joystick_control':
            handleJoystickControl(clientId, message);
            break;
            
        case 'mapping_command':  // ✅ FIXED: Direct string matching
            handleMappingCommand(clientId, message);
            break;
            
        case 'control_command':
            handleControlCommand(clientId, message);
            break;
            
        case 'device_discovery':
            handleDeviceDiscovery(clientId, message);
            break;
            
        // ✅ NEW: Handle request_data to prevent "Unknown message type" errors
        case 'request_data':
            handleDataRequest(clientId, message);
            break;
            
        default:
            console.log(`❓ Unknown message type from ${clientId}: ${message.type}`);
            sendToClient(clientId, {
                type: 'error',
                message: `Unknown message type: ${message.type}`,
                timestamp: new Date().toISOString()
            });
    }
}

function handleDeviceConnectMessage(clientId, message) {
    try {
        const { deviceId, deviceInfo } = message;
        
        if (!deviceId) {
            sendToClient(clientId, {
                type: 'error',
                message: 'Device ID required for connection',
                timestamp: new Date().toISOString()
            });
            return;
        }
        
        const client = connectedClients.get(clientId);
        if (client) {
            client.connectedDeviceId = deviceId;
            client.deviceInfo = deviceInfo;
        }
        
        sendToClient(clientId, {
            type: 'device_connected',
            deviceId: deviceId,
            status: 'success',
            timestamp: new Date().toISOString()
        });
        
        console.log(`🔌 Client ${clientId} connected to device ${deviceId}`);
        
    } catch (error) {
        console.error('❌ Error handling device connect:', error);
        sendToClient(clientId, {
            type: 'error',
            message: 'Failed to connect to device',
            details: error.message,
            timestamp: new Date().toISOString()
        });
    }
}

function handleSubscription(clientId, message) {
    try {
        const { topics, deviceId } = message;
        const client = connectedClients.get(clientId);
        
        if (client && topics) {
            topics.forEach(topic => {
                client.subscribedTopics.add(topic);
                
                if (deviceId) {
                    const deviceTopic = `${deviceId}.${topic}`;
                    client.subscribedTopics.add(deviceTopic);
                }
            });
            
            sendToClient(clientId, {
                type: 'subscription_confirmed',
                topics: topics,
                deviceId: deviceId,
                timestamp: new Date().toISOString()
            });
            
            console.log(`📡 Client ${clientId} subscribed to: ${topics.join(', ')}`);
        }
    } catch (error) {
        console.error('❌ Error handling subscription:', error);
        sendToClient(clientId, {
            type: 'error',
            message: 'Failed to process subscription',
            details: error.message,
            timestamp: new Date().toISOString()
        });
    }
}

function handleUnsubscription(clientId, message) {
    try {
        const { topics } = message;
        const client = connectedClients.get(clientId);
        
        if (client && topics) {
            topics.forEach(topic => {
                client.subscribedTopics.delete(topic);
            });
            
            sendToClient(clientId, {
                type: 'unsubscription_confirmed',
                topics: topics,
                timestamp: new Date().toISOString()
            });
        }
    } catch (error) {
        console.error('❌ Error handling unsubscription:', error);
    }
}

// ✅ FIXED: Enhanced joystick control handling with max speed support
function handleJoystickControl(clientId, message) {
    try {
        console.log(`🕹️ Joystick control from ${clientId}:`, message);
        const { deviceId, x, y, deadman, maxLinearSpeed, maxAngularSpeed } = message;
        
        if (!deviceId) {
            throw new Error('Missing deviceId in joystick control');
        }

        // Validate joystick parameters
        if (typeof x !== 'number' || typeof y !== 'number' || typeof deadman !== 'boolean') {
            throw new Error('Invalid joystick parameters');
        }

        // Check if device exists
        const device = global.connectedDevices?.find(d => d.id === deviceId);
        if (!device) {
            console.warn(`⚠️ Device ${deviceId} not found in connected devices`);
            // Don't throw error, just warn and continue for now
        }
        
        // Import ROS connection for publishing
        const rosConnection = require('../ros/utils/ros_connection');
        
        let result;
        if (rosConnection && typeof rosConnection.publishJoystick === 'function') {
            // ✅ CRITICAL FIX: Pass max speeds from UI to ROS publisher
            result = rosConnection.publishJoystick(x, y, deadman, maxLinearSpeed, maxAngularSpeed);
        } else {
            console.warn('⚠️ ROS connection not available, simulating joystick command');
            result = { success: true, simulated: true };
        }
        
        // Send result back to client
        sendToClient(clientId, {
            type: 'joystick_result',
            deviceId: deviceId,
            result: result,
            timestamp: new Date().toISOString()
        });
        
        // Broadcast to other subscribers
        broadcastToSubscribers('control_events', {
            type: 'joystick_command',
            deviceId: deviceId,
            data: { x, y, deadman, maxLinearSpeed, maxAngularSpeed },
            result: result,
            clientId: clientId,
            timestamp: new Date().toISOString()
        }, clientId);
        
    } catch (error) {
        console.error('❌ Error handling joystick control:', error);
        sendToClient(clientId, {
            type: 'error',
            message: 'Failed to process joystick command',
            details: error.message,
            timestamp: new Date().toISOString()
        });
    }
}

// ✅ FIXED: Enhanced mapping command handling with better error handling
function handleMappingCommand(clientId, message) {
    try {
        console.log(`🗺️ Mapping command from ${clientId}:`, message);
        const { deviceId, command, options = {} } = message;

        // Validate required fields
        if (!deviceId) {
            throw new Error('Missing deviceId in mapping command');
        }
        
        if (!command) {
            throw new Error('Missing command in mapping request');
        }

        // Validate command type
        const validCommands = ['start', 'stop', 'pause', 'resume', 'save'];
        if (!validCommands.includes(command)) {
            throw new Error(`Invalid mapping command: ${command}. Valid commands: ${validCommands.join(', ')}`);
        }

        // Check if device exists (warn but don't fail)
        const device = global.connectedDevices?.find(d => d.id === deviceId);
        if (!device) {
            console.warn(`⚠️ Device ${deviceId} not found in connected devices, but continuing...`);
        }

        // Try to get ROS connection
        let rosConnection;
        let result;
        
        try {
            rosConnection = require('../ros/utils/ros_connection');
        } catch (requireError) {
            console.warn('⚠️ ROS connection module not found, simulating mapping command');
            result = { 
                success: true, 
                simulated: true, 
                command: command,
                message: `Simulated ${command} mapping for device ${deviceId}`
            };
        }

        if (rosConnection && !result) {
            // Execute the mapping command
            switch (command) {
                case 'start':
                    if (typeof rosConnection.startMapping === 'function') {
                        result = rosConnection.startMapping(deviceId);
                    } else {
                        result = { success: true, simulated: true, command: 'start' };
                    }
                    break;
                case 'stop':
                    if (typeof rosConnection.stopMapping === 'function') {
                        result = rosConnection.stopMapping(deviceId);
                    } else {
                        result = { success: true, simulated: true, command: 'stop' };
                    }
                    break;
                case 'pause':
                    if (typeof rosConnection.pauseMapping === 'function') {
                        result = rosConnection.pauseMapping(deviceId);
                    } else {
                        result = { success: true, simulated: true, command: 'pause' };
                    }
                    break;
                case 'resume':
                    if (typeof rosConnection.resumeMapping === 'function') {
                        result = rosConnection.resumeMapping(deviceId);
                    } else {
                        result = { success: true, simulated: true, command: 'resume' };
                    }
                    break;
                case 'save':
                    if (typeof rosConnection.saveMap === 'function') {
                        // Pass options to saveMap function for enhanced map saving
                        console.log(`💾 Saving map with options:`, options);
                        result = rosConnection.saveMap(deviceId, options);
                    } else {
                        result = { success: true, simulated: true, command: 'save' };
                    }
                    break;
                default:
                    throw new Error(`Unhandled mapping command: ${command}`);
            }
        }

        // Ensure we have a result
        if (!result) {
            result = { 
                success: false, 
                error: 'No result from mapping operation',
                command: command 
            };
        }

        console.log(`✅ Mapping command ${command} executed for device ${deviceId}:`, result);

        // Send result back to client
        sendToClient(clientId, {
            type: 'mapping_result',
            deviceId: deviceId,
            command: command,
            result: result,
            success: result.success !== false,
            timestamp: new Date().toISOString()
        });

        // Broadcast to other subscribers
        broadcastToSubscribers('mapping_events', {
            type: `mapping_${command}`,
            deviceId: deviceId,
            command: command,
            result: result,
            clientId: clientId,
            timestamp: new Date().toISOString()
        }, clientId);

    } catch (error) {
        console.error('❌ Error handling mapping command:', error);
        
        const errorResponse = {
            type: 'error',
            subType: 'mapping_command_failed',
            message: 'Failed to process mapping command',
            details: error.message,
            deviceId: message.deviceId || 'unknown',
            command: message.command || 'unknown',
            timestamp: new Date().toISOString()
        };
        
        sendToClient(clientId, errorResponse);
        
        // Also broadcast the error to mapping event subscribers
        broadcastToSubscribers('mapping_events', {
            type: 'mapping_error',
            error: errorResponse,
            clientId: clientId,
            timestamp: new Date().toISOString()
        }, clientId);
    }
}

// ✅ NEW: General control command handler
function handleControlCommand(clientId, message) {
    try {
        console.log(`🎮 Control command from ${clientId}:`, message);
        const { deviceId, command, data } = message;

        if (!deviceId) {
            throw new Error('Missing deviceId in control command');
        }

        if (!command) {
            throw new Error('Missing command in control request');
        }

        // Check if device exists
        const device = global.connectedDevices?.find(d => d.id === deviceId);
        if (!device) {
            console.warn(`⚠️ Device ${deviceId} not found in connected devices`);
        }

        let result = { success: true, command: command, data: data || {} };

        // Try to get ROS connection
        try {
            const rosConnection = require('../ros/utils/ros_connection');
            
            switch (command) {
                case 'move':
                    if (rosConnection.publishMovement) {
                        result = rosConnection.publishMovement(data.linear || 0, data.angular || 0);
                    }
                    break;
                case 'stop':
                    if (rosConnection.publishMovement) {
                        result = rosConnection.publishMovement(0, 0);
                    }
                    break;
                case 'goal':
                    if (rosConnection.publishGoal) {
                        result = rosConnection.publishGoal(data.x, data.y, data.orientation);
                    }
                    break;
                default:
                    result = { success: true, simulated: true, command: command };
            }
        } catch (rosError) {
            console.warn('⚠️ ROS connection error:', rosError.message);
            result = { success: true, simulated: true, command: command };
        }

        // Send result back to client
        sendToClient(clientId, {
            type: 'control_result',
            deviceId: deviceId,
            command: command,
            result: result,
            timestamp: new Date().toISOString()
        });

        // Broadcast to other subscribers
        broadcastToSubscribers('control_events', {
            type: 'control_command',
            deviceId: deviceId,
            command: command,
            data: data,
            result: result,
            clientId: clientId,
            timestamp: new Date().toISOString()
        }, clientId);

    } catch (error) {
        console.error('❌ Error handling control command:', error);
        sendToClient(clientId, {
            type: 'error',
            message: 'Failed to process control command',
            details: error.message,
            timestamp: new Date().toISOString()
        });
    }
}

function handleDeviceDiscovery(clientId, message) {
    try {
        const availableDevices = global.connectedDevices || [];
        
        sendToClient(clientId, {
            type: 'device_discovery_response',
            devices: availableDevices,
            server: {
                ip: '192.168.128.29',
                port: 3000,
                websocket_port: 3000,
                capabilities: ['mapping', 'navigation', 'remote_control'],
                ros2_status: 'connected'
            },
            timestamp: new Date().toISOString()
        });
        
        console.log(`🔍 Sent device discovery response to ${clientId}: ${availableDevices.length} devices`);
        
    } catch (error) {
        console.error('❌ Error handling device discovery:', error);
        sendToClient(clientId, {
            type: 'error',
            message: 'Failed to process device discovery',
            details: error.message,
            timestamp: new Date().toISOString()
        });
    }
}

// ✅ NEW: Handle data requests to prevent errors
function handleDataRequest(clientId, message) {
    try {
        const { requestType, deviceId } = message;
        
        console.log(`📊 Data request from ${clientId}: ${requestType} for device ${deviceId || 'any'}`);
        
        // Handle different request types
        switch (requestType) {
            case 'device_status':
                // Send device status if available
                const devices = global.connectedDevices || [];
                const device = deviceId ? devices.find(d => d.id === deviceId) : null;
                
                sendToClient(clientId, {
                    type: 'device_status_response',
                    deviceId: deviceId,
                    status: device ? 'online' : 'unknown',
                    data: device || {},
                    timestamp: new Date().toISOString()
                });
                break;
                
            case 'map':
                // Send current map data if available
                sendToClient(clientId, {
                    type: 'map_data_response',
                    deviceId: deviceId,
                    mapData: global.currentMapData || null,
                    timestamp: new Date().toISOString()
                });
                break;
                
            case 'global_costmap':
            case 'local_costmap':
                // Send costmap data if available
                sendToClient(clientId, {
                    type: `${requestType}_response`,
                    deviceId: deviceId,
                    costmapData: global[`${requestType}Data`] || null,
                    timestamp: new Date().toISOString()
                });
                break;
                
            default:
                sendToClient(clientId, {
                    type: 'data_request_result',
                    requestType: requestType,
                    success: false,
                    message: `Unsupported request type: ${requestType}`,
                    timestamp: new Date().toISOString()
                });
        }
        
    } catch (error) {
        console.error('❌ Error handling data request:', error);
        sendToClient(clientId, {
            type: 'error',
            message: 'Failed to process data request',
            details: error.message,
            timestamp: new Date().toISOString()
        });
    }
}

function handleHeartbeat(clientId) {
    const client = connectedClients.get(clientId);
    if (client) {
        client.lastHeartbeat = new Date();
        sendToClient(clientId, {
            type: 'heartbeat_ack',
            timestamp: new Date().toISOString()
        });
    }
}

function sendWelcomeMessage(clientId) {
    sendToClient(clientId, {
        type: 'connection',
        status: 'connected',
        clientId: clientId,
        timestamp: new Date().toISOString(),
        serverInfo: {
            version: '1.0.0',
            capabilities: ['real_time_tracking', 'map_editing', 'order_management', 'device_discovery'],
            pingInterval: config.WEBSOCKET.PING_INTERVAL,
            heartbeatInterval: config.WEBSOCKET.HEARTBEAT_INTERVAL,
        },
        availableTopics: [
            'real_time_data',
            'control_events', 
            'mapping_events',
            'order_events',
            'device_events',
            'map_events'
        ]
    });
}

function sendInitialData(clientId) {
    try {
        const initialData = {
            devices: global.connectedDevices || [],
            liveData: global.liveData || {},
            systemHealth: {
                connectedDevices: (global.connectedDevices || []).length,
                activeOrders: Object.values(global.deviceOrders || {})
                    .flat()
                    .filter(order => order.status === 'active').length,
                timestamp: new Date().toISOString()
            }
        };

        sendToClient(clientId, {
            type: 'initial_data',
            devices: initialData.devices,
            liveData: initialData.liveData,
            systemHealth: initialData.systemHealth,
            timestamp: new Date().toISOString()
        });

        console.log(`📊 Sent initial data to ${clientId}: ${initialData.devices.length} devices`);

    } catch (error) {
        console.error(`❌ Error sending initial data to ${clientId}: ${error}`);
    }
}

function sendToClient(clientId, message) {
    const client = connectedClients.get(clientId);
    if (client && client.ws.readyState === WebSocket.OPEN) {
        try {
            client.ws.send(JSON.stringify(message));
        } catch (error) {
            console.error(`❌ Error sending message to client ${clientId}:`, error);
            connectedClients.delete(clientId);
            clientSubscriptions.delete(clientId);
        }
    }
}

function broadcastToSubscribers(topic, message, excludeClientId = null) {
    const timestamp = new Date().toISOString();
    const broadcastMessage = {
        type: 'broadcast',
        topic: topic,
        data: message,
        timestamp: timestamp
    };

    let sentCount = 0;
    
    connectedClients.forEach((client, clientId) => {
        if (clientId === excludeClientId) return;
        
        if (client.subscribedTopics.has(topic)) {
            sendToClient(clientId, broadcastMessage);
            sentCount++;
        }
    });

    if (sentCount > 0) {
        console.log(`📡 Broadcasted ${topic} to ${sentCount} subscribers`);
    }
}

function broadcastToAll(message, excludeClientId = null) {
    const timestamp = new Date().toISOString();
    const broadcastMessage = {
        ...message,
        timestamp: timestamp
    };

    let sentCount = 0;
    
    connectedClients.forEach((client, clientId) => {
        if (clientId === excludeClientId) return;
        sendToClient(clientId, broadcastMessage);
        sentCount++;
    });

    console.log(`📡 Broadcasted message to ${sentCount} clients`);
}

function cleanupConnections() {
    const now = new Date();
    const clientsToRemove = [];

    connectedClients.forEach((client, clientId) => {
        try {
            if (client.ws.readyState !== WebSocket.OPEN) {
                console.log(`🧹 Removing client with closed connection: ${clientId}`);
                clientsToRemove.push(clientId);
            } else {
                const timeSinceLastHeartbeat = now - client.lastHeartbeat;
                if (timeSinceLastHeartbeat > 120000) { // 2 minutes
                    console.log(`🧹 Removing stale client: ${clientId} (${timeSinceLastHeartbeat}ms since last heartbeat)`);
                    clientsToRemove.push(clientId);
                    try {
                        client.ws.terminate();
                    } catch (e) {
                        // Ignore termination errors
                    }
                }
            }
        } catch (error) {
            console.error(`❌ Error checking client ${clientId}: ${error}`);
            clientsToRemove.push(clientId);
        }
    });

    clientsToRemove.forEach(clientId => {
        stopPingPong(clientId);
        connectedClients.delete(clientId);
        clientSubscriptions.delete(clientId);
    });

    if (clientsToRemove.length > 0) {
        console.log(`🧹 Cleaned up ${clientsToRemove.length} dead connections`);
    }
}

function generateClientId() {
    return `client_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}

function getConnectionStats() {
    const stats = {
        totalClients: connectedClients.size,
        clientsByTopic: {},
        averageConnectionTime: 0,
        topicSubscriptions: {}
    };

    const now = new Date();
    let totalConnectionTime = 0;
    
    connectedClients.forEach((client) => {
        totalConnectionTime += now - client.connectedAt;
        
        client.subscribedTopics.forEach(topic => {
            if (!stats.topicSubscriptions[topic]) {
                stats.topicSubscriptions[topic] = 0;
            }
            stats.topicSubscriptions[topic]++;
        });
    });

    if (connectedClients.size > 0) {
        stats.averageConnectionTime = totalConnectionTime / connectedClients.size;
    }

    return stats;
}

module.exports = {
    initializeWebSocketServer,
    sendToClient,
    broadcastToSubscribers,
    broadcastToAll,
    getConnectedClients: () => connectedClients,
    getConnectionStats
};