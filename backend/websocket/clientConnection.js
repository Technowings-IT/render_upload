// websocket/clientConnection.js - FIXED WebSocket with stable connection
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
        startPingPong(clientId); // ✅ FIXED: Enable ping-pong
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
                timestamp: new Date().toISOString()
            });
        }
    });

    // ✅ FIXED: Proper pong handling
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

// ✅ FIXED: Uncommented and improved ping-pong mechanism
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

// ✅ FIXED: Enhanced message handling with device-specific support
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
            
        case config.WEBSOCKET.MESSAGE_TYPES.HEARTBEAT:
            handleHeartbeat(clientId);
            break;
            
        // ✅ FIXED: Device-specific WebSocket handling
        case 'device_connect':
            handleDeviceConnectMessage(clientId, message);
            break;
            
        case 'subscribe':
            handleSubscription(clientId, message);
            break;
            
        case 'unsubscribe':
            handleUnsubscription(clientId, message);
            break;
            
        case config.WEBSOCKET.MESSAGE_TYPES.JOYSTICK_CONTROL:
            handleJoystickControl(clientId, message);
            break;
            
        case config.WEBSOCKET.MESSAGE_TYPES.MAPPING_COMMAND:
            handleMappingCommand(clientId, message);
            break;
            
        case 'device_discovery':
            handleDeviceDiscovery(clientId, message);
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

// ✅ NEW: Device-specific connection handling
function handleDeviceConnectMessage(clientId, message) {
    try {
        const { deviceId, deviceInfo } = message;
        
        if (!deviceId) {
            sendToClient(clientId, {
                type: 'error',
                message: 'Device ID required for connection'
            });
            return;
        }
        
        // Store device association with client
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
            message: 'Failed to connect to device'
        });
    }
}

// ✅ NEW: Enhanced subscription handling
function handleSubscription(clientId, message) {
    try {
        const { topics, deviceId } = message;
        const client = connectedClients.get(clientId);
        
        if (client && topics) {
            topics.forEach(topic => {
                client.subscribedTopics.add(topic);
                
                // Device-specific subscriptions
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

// ✅ NEW: Joystick control via WebSocket
function handleJoystickControl(clientId, message) {
    try {
        const { deviceId, x, y, deadman } = message;
        
        // Import ROS connection for publishing
        const rosConnection = require('../ros/utils/ros_connection');
        const result = rosConnection.publishJoystick(x, y, deadman);
        
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
            data: { x, y, deadman },
            result: result,
            clientId: clientId
        }, clientId);
        
    } catch (error) {
        console.error('❌ Error handling joystick control:', error);
        sendToClient(clientId, {
            type: 'error',
            message: 'Failed to process joystick command'
        });
    }
}

// ✅ NEW: Mapping command via WebSocket
function handleMappingCommand(clientId, message) {
    try {
        const { deviceId, command } = message;
        const rosConnection = require('../ros/utils/ros_connection');
        
        let result;
        switch (command) {
            case 'start':
                result = rosConnection.startMapping();
                break;
            case 'stop':
                result = rosConnection.stopMapping();
                break;
            default:
                throw new Error(`Unknown mapping command: ${command}`);
        }
        
        sendToClient(clientId, {
            type: 'mapping_result',
            deviceId: deviceId,
            command: command,
            result: result,
            timestamp: new Date().toISOString()
        });
        
        broadcastToSubscribers('mapping_events', {
            type: `mapping_${command}`,
            deviceId: deviceId,
            result: result,
            clientId: clientId
        }, clientId);
        
    } catch (error) {
        console.error('❌ Error handling mapping command:', error);
        sendToClient(clientId, {
            type: 'error',
            message: 'Failed to process mapping command'
        });
    }
}

// ✅ NEW: Device discovery response
function handleDeviceDiscovery(clientId, message) {
    try {
        const availableDevices = global.connectedDevices || [];
        
        sendToClient(clientId, {
            type: 'device_discovery_response',
            devices: availableDevices,
            server: {
                ip: '192.168.253.79', // Your backend IP
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
    }
}

function handleHeartbeat(clientId) {
    const client = connectedClients.get(clientId);
    if (client) {
        client.lastHeartbeat = new Date();
        sendToClient(clientId, {
            type: config.WEBSOCKET.MESSAGE_TYPES.HEARTBEAT_ACK,
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
                // ✅ FIXED: More lenient timeout (2 minutes instead of aggressive timeouts)
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