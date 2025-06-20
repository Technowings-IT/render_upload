// websocket/clientConnection.js - Enhanced WebSocket Server for Real-time Communication
const WebSocket = require('ws');
const config = require('../config');

let wss;
const connectedClients = new Map();
const clientSubscriptions = new Map(); // Track what each client is subscribed to

function initializeWebSocketServer(server) {
    wss = new WebSocket.Server({ 
        server,
        perMessageDeflate: config.WEBSOCKET.COMPRESSION
    });

    wss.on('connection', (ws, req) => {
        const clientId = generateClientId();
        const clientInfo = {
            id: clientId,
            ws: ws,
            connectedAt: new Date(),
            lastHeartbeat: new Date(),
            subscribedTopics: new Set(),
            ipAddress: req.connection.remoteAddress,
            userAgent: req.headers['user-agent'],
            subscriptions: {
                devices: new Set(),
                topics: new Set(),
                events: new Set()
            }
        };

        connectedClients.set(clientId, clientInfo);
        clientSubscriptions.set(clientId, new Set());
        
        console.log(`📱 Client ${clientId} connected from ${clientInfo.ipAddress}. Total clients: ${connectedClients.size}`);

        // Send welcome message with capabilities
        sendToClient(clientId, {
            type: 'connection',
            status: 'connected',
            clientId: clientId,
            timestamp: new Date().toISOString(),
            capabilities: {
                realTimeTracking: true,
                mapEditing: true,
                orderManagement: true,
                joystickControl: true
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

        // Handle incoming messages
        ws.on('message', (data) => {
            try {
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

        // Handle client disconnect
        ws.on('close', () => {
            console.log(`📱 Client ${clientId} disconnected`);
            connectedClients.delete(clientId);
            clientSubscriptions.delete(clientId);
        });

        // Handle WebSocket errors
        ws.on('error', (error) => {
            console.error(`❌ WebSocket error for client ${clientId}:`, error);
            connectedClients.delete(clientId);
            clientSubscriptions.delete(clientId);
        });

        // Set up heartbeat
        setupHeartbeat(clientId);
        
        // Send initial data if devices are connected
        sendInitialData(clientId);
    });

    // Periodic cleanup of dead connections
    setInterval(cleanupConnections, 60000); // Every minute
    
    console.log('✅ WebSocket server initialized');
}

function handleClientMessage(clientId, message) {
    const client = connectedClients.get(clientId);
    if (!client) return;

    // Update last heartbeat on any message
    client.lastHeartbeat = new Date();

    switch (message.type) {
        case config.WEBSOCKET.MESSAGE_TYPES.HEARTBEAT:
            handleHeartbeat(clientId);
            break;

        case config.WEBSOCKET.MESSAGE_TYPES.SUBSCRIBE:
            handleSubscription(clientId, message);
            break;

        case config.WEBSOCKET.MESSAGE_TYPES.UNSUBSCRIBE:
            handleUnsubscription(clientId, message);
            break;

        case config.WEBSOCKET.MESSAGE_TYPES.CONTROL_COMMAND:
            handleControlCommand(clientId, message);
            break;

        case config.WEBSOCKET.MESSAGE_TYPES.MAP_EDIT:
            handleMapEdit(clientId, message);
            break;

        case config.WEBSOCKET.MESSAGE_TYPES.ORDER_COMMAND:
            handleOrderCommand(clientId, message);
            break;

        case 'request_data':
            handleDataRequest(clientId, message);
            break;

        case 'ping':
            sendToClient(clientId, {
                type: 'pong',
                timestamp: new Date().toISOString()
            });
            break;

        default:
            sendToClient(clientId, {
                type: 'error',
                message: `Unknown message type: ${message.type}`,
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
            timestamp: new Date().toISOString(),
            clientInfo: {
                connectedFor: Date.now() - client.connectedAt.getTime(),
                subscriptions: client.subscribedTopics.size
            }
        });
    }
}

function handleSubscription(clientId, message) {
    const client = connectedClients.get(clientId);
    if (!client) return;

    const { topic, deviceId, options } = message;

    if (topic) {
        client.subscribedTopics.add(topic);
        
        // Handle device-specific subscriptions
        if (deviceId) {
            client.subscriptions.devices.add(deviceId);
        }

        // Add to topic-based subscriptions
        if (!clientSubscriptions.has(topic)) {
            clientSubscriptions.set(topic, new Set());
        }
        clientSubscriptions.get(topic).add(clientId);

        sendToClient(clientId, {
            type: 'subscription_ack',
            topic: topic,
            deviceId: deviceId,
            status: 'subscribed',
            timestamp: new Date().toISOString()
        });

        console.log(`📡 Client ${clientId} subscribed to ${topic}${deviceId ? ` for device ${deviceId}` : ''}`);

        // Send initial data for this subscription
        sendInitialDataForTopic(clientId, topic, deviceId);
    }
}

function handleUnsubscription(clientId, message) {
    const client = connectedClients.get(clientId);
    if (!client) return;

    const { topic, deviceId } = message;

    if (topic) {
        client.subscribedTopics.delete(topic);
        
        if (deviceId) {
            client.subscriptions.devices.delete(deviceId);
        }

        // Remove from topic-based subscriptions
        if (clientSubscriptions.has(topic)) {
            clientSubscriptions.get(topic).delete(clientId);
        }

        sendToClient(clientId, {
            type: 'unsubscription_ack',
            topic: topic,
            deviceId: deviceId,
            status: 'unsubscribed',
            timestamp: new Date().toISOString()
        });

        console.log(`📡 Client ${clientId} unsubscribed from ${topic}${deviceId ? ` for device ${deviceId}` : ''}`);
    }
}

function handleControlCommand(clientId, message) {
    const { deviceId, command, data } = message;
    
    if (!deviceId || !command) {
        sendToClient(clientId, {
            type: 'error',
            message: 'Device ID and command are required',
            timestamp: new Date().toISOString()
        });
        return;
    }

    // Validate device exists
    const device = global.connectedDevices?.find(d => d.id === deviceId);
    if (!device) {
        sendToClient(clientId, {
            type: 'error',
            message: 'Device not found or not connected',
            timestamp: new Date().toISOString()
        });
        return;
    }

    // Forward to appropriate handler based on command type
    try {
        let result;
        const publishers = require('../ros/utils/publishers');

        switch (command) {
            case 'move':
                result = publishers.publishVelocity(deviceId, data.linear || 0, data.angular || 0);
                break;
            case 'stop':
                result = publishers.emergencyStop(deviceId);
                break;
            case 'goal':
                result = publishers.publishGoal(deviceId, data.x, data.y, data.orientation || 0);
                break;
            default:
                throw new Error(`Unknown command: ${command}`);
        }

        // Acknowledge command execution
        sendToClient(clientId, {
            type: 'command_ack',
            deviceId,
            command,
            result,
            timestamp: new Date().toISOString()
        });

        // Broadcast command execution to other clients
        broadcastToSubscribers('control_events', {
            type: 'control_command_executed',
            deviceId,
            command,
            data,
            executedBy: clientId,
            timestamp: new Date().toISOString()
        }, clientId); // Exclude the sender

    } catch (error) {
        sendToClient(clientId, {
            type: 'error',
            message: `Command execution failed: ${error.message}`,
            timestamp: new Date().toISOString()
        });
    }
}

function handleMapEdit(clientId, message) {
    const { deviceId, editType, editData } = message;
    
    if (!deviceId || !editType) {
        sendToClient(clientId, {
            type: 'error',
            message: 'Device ID and edit type are required',
            timestamp: new Date().toISOString()
        });
        return;
    }

    try {
        // Initialize map if it doesn't exist
        if (!global.deviceMaps) {
            global.deviceMaps = {};
        }
        if (!global.deviceMaps[deviceId]) {
            global.deviceMaps[deviceId] = {
                shapes: [],
                annotations: [],
                metadata: { deviceId, createdAt: new Date().toISOString() }
            };
        }

        let result = { success: true };

        switch (editType) {
            case 'add_shape':
                const newShape = {
                    id: `shape_${Date.now()}_${Math.random().toString(36).substr(2, 6)}`,
                    ...editData,
                    createdAt: new Date().toISOString(),
                    createdBy: clientId
                };
                global.deviceMaps[deviceId].shapes.push(newShape);
                result.shape = newShape;
                break;

            case 'update_shape':
                const shapeIndex = global.deviceMaps[deviceId].shapes.findIndex(s => s.id === editData.id);
                if (shapeIndex !== -1) {
                    Object.assign(global.deviceMaps[deviceId].shapes[shapeIndex], editData, {
                        updatedAt: new Date().toISOString(),
                        updatedBy: clientId
                    });
                    result.shape = global.deviceMaps[deviceId].shapes[shapeIndex];
                } else {
                    throw new Error('Shape not found');
                }
                break;

            case 'delete_shape':
                const deleteIndex = global.deviceMaps[deviceId].shapes.findIndex(s => s.id === editData.id);
                if (deleteIndex !== -1) {
                    result.deletedShape = global.deviceMaps[deviceId].shapes.splice(deleteIndex, 1)[0];
                } else {
                    throw new Error('Shape not found');
                }
                break;

            case 'add_annotation':
                const annotation = {
                    id: `annotation_${Date.now()}_${Math.random().toString(36).substr(2, 6)}`,
                    ...editData,
                    createdAt: new Date().toISOString(),
                    createdBy: clientId
                };
                global.deviceMaps[deviceId].annotations.push(annotation);
                result.annotation = annotation;
                break;

            default:
                throw new Error(`Unknown edit type: ${editType}`);
        }

        // Acknowledge edit
        sendToClient(clientId, {
            type: 'map_edit_ack',
            deviceId,
            editType,
            result,
            timestamp: new Date().toISOString()
        });

        // Broadcast edit to other clients
        broadcastToSubscribers('map_events', {
            type: 'map_edited',
            deviceId,
            editType,
            editData: result,
            editedBy: clientId,
            timestamp: new Date().toISOString()
        }, clientId);

    } catch (error) {
        sendToClient(clientId, {
            type: 'error',
            message: `Map edit failed: ${error.message}`,
            timestamp: new Date().toISOString()
        });
    }
}

function handleOrderCommand(clientId, message) {
    const { deviceId, orderAction, orderData } = message;
    
    // This would typically interface with the order management system
    // For now, just acknowledge and broadcast
    sendToClient(clientId, {
        type: 'order_ack',
        deviceId,
        orderAction,
        timestamp: new Date().toISOString()
    });

    broadcastToSubscribers('order_events', {
        type: 'order_command_received',
        deviceId,
        orderAction,
        orderData,
        issuedBy: clientId,
        timestamp: new Date().toISOString()
    }, clientId);
}

function handleDataRequest(clientId, message) {
    const { requestType, deviceId } = message;
    
    try {
        let data = null;

        switch (requestType) {
            case 'device_status':
                if (deviceId) {
                    const device = global.connectedDevices?.find(d => d.id === deviceId);
                    const liveData = global.liveData?.[deviceId];
                    data = { device, liveData };
                } else {
                    data = {
                        devices: global.connectedDevices || [],
                        liveData: global.liveData || {}
                    };
                }
                break;

            case 'map_data':
                if (deviceId) {
                    data = global.deviceMaps?.[deviceId];
                } else {
                    data = global.deviceMaps || {};
                }
                break;

            case 'orders':
                if (deviceId) {
                    data = global.deviceOrders?.[deviceId] || [];
                } else {
                    data = global.deviceOrders || {};
                }
                break;

            default:
                throw new Error(`Unknown request type: ${requestType}`);
        }

        sendToClient(clientId, {
            type: 'data_response',
            requestType,
            deviceId,
            data,
            timestamp: new Date().toISOString()
        });

    } catch (error) {
        sendToClient(clientId, {
            type: 'error',
            message: `Data request failed: ${error.message}`,
            timestamp: new Date().toISOString()
        });
    }
}

function sendInitialData(clientId) {
    // Send current device list
    sendToClient(clientId, {
        type: 'initial_data',
        devices: global.connectedDevices || [],
        timestamp: new Date().toISOString()
    });
}

function sendInitialDataForTopic(clientId, topic, deviceId) {
    try {
        let data = null;

        switch (topic) {
            case 'real_time_data':
                if (deviceId && global.liveData?.[deviceId]) {
                    data = global.liveData[deviceId];
                } else if (!deviceId) {
                    data = global.liveData || {};
                }
                break;

            case 'device_events':
                data = {
                    connectedDevices: global.connectedDevices || [],
                    count: (global.connectedDevices || []).length
                };
                break;

            case 'map_events':
                if (deviceId && global.deviceMaps?.[deviceId]) {
                    data = global.deviceMaps[deviceId];
                } else if (!deviceId) {
                    data = global.deviceMaps || {};
                }
                break;

            case 'order_events':
                if (deviceId && global.deviceOrders?.[deviceId]) {
                    data = global.deviceOrders[deviceId];
                } else if (!deviceId) {
                    data = global.deviceOrders || {};
                }
                break;
        }

        if (data) {
            sendToClient(clientId, {
                type: 'initial_topic_data',
                topic,
                deviceId,
                data,
                timestamp: new Date().toISOString()
            });
        }

    } catch (error) {
        console.error(`❌ Error sending initial data for topic ${topic}:`, error);
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
        if (clientId === excludeClientId) return; // Skip excluded client
        
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

function setupHeartbeat(clientId) {
    const client = connectedClients.get(clientId);
    if (!client) return;

    const heartbeatInterval = setInterval(() => {
        if (!connectedClients.has(clientId)) {
            clearInterval(heartbeatInterval);
            return;
        }

        const now = new Date();
        const timeSinceLastHeartbeat = now - client.lastHeartbeat;

        if (timeSinceLastHeartbeat > config.WEBSOCKET.CONNECTION_TIMEOUT) {
            console.log(`💔 Client ${clientId} timed out (${timeSinceLastHeartbeat}ms since last heartbeat)`);
            client.ws.terminate();
            connectedClients.delete(clientId);
            clientSubscriptions.delete(clientId);
            clearInterval(heartbeatInterval);
        } else {
            // Send periodic heartbeat
            sendToClient(clientId, {
                type: 'heartbeat',
                timestamp: now.toISOString(),
                serverStats: {
                    connectedClients: connectedClients.size,
                    uptime: process.uptime()
                }
            });
        }
    }, config.WEBSOCKET.HEARTBEAT_INTERVAL);
}

function cleanupConnections() {
    const now = new Date();
    const clientsToRemove = [];

    connectedClients.forEach((client, clientId) => {
        if (client.ws.readyState !== WebSocket.OPEN) {
            clientsToRemove.push(clientId);
        } else {
            // Check for stale connections
            const timeSinceLastHeartbeat = now - client.lastHeartbeat;
            if (timeSinceLastHeartbeat > config.WEBSOCKET.CONNECTION_TIMEOUT * 2) {
                console.log(`🧹 Cleaning up stale connection: ${clientId}`);
                clientsToRemove.push(clientId);
                client.ws.terminate();
            }
        }
    });

    clientsToRemove.forEach(clientId => {
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

// Get statistics about connected clients
function getConnectionStats() {
    const stats = {
        totalClients: connectedClients.size,
        clientsByTopic: {},
        averageConnectionTime: 0,
        topicSubscriptions: {}
    };

    // Calculate average connection time
    const now = new Date();
    let totalConnectionTime = 0;
    
    connectedClients.forEach((client) => {
        totalConnectionTime += now - client.connectedAt;
        
        // Count subscriptions by topic
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

// Export functions for external use
module.exports = {
    initializeWebSocketServer,
    sendToClient,
    broadcastToSubscribers,
    broadcastToAll,
    getConnectedClients: () => connectedClients,
    getConnectionStats
};