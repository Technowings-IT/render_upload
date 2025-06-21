// websocket/messageHandler.js - Enhanced WebSocket message handling for AGV control
const rosConnection = require('../ros/utils/ros_connection');
const storageManager = require('../ros/utils/storageManager');

// Enhanced message handler that integrates with your WebSocket clientConnection.js
function enhanceWebSocketHandlers(existingHandler) {
    return function handleClientMessage(clientId, message) {
        const client = this.connectedClients?.get(clientId);
        if (!client) return;

        try {
            // Call existing handler first for basic messages
            if (existingHandler) {
                existingHandler.call(this, clientId, message);
            }

            // Handle AGV-specific control messages
            switch (message.type) {
                case 'control_command':
                    handleControlCommand(clientId, message);
                    break;
                
                case 'joystick_control':
                    handleJoystickControl(clientId, message);
                    break;
                
                case 'mapping_command':
                    handleMappingCommand(clientId, message);
                    break;
                
                case 'map_edit':
                    handleMapEdit(clientId, message);
                    break;
                
                case 'order_command':
                    handleOrderCommand(clientId, message);
                    break;
                
                case 'device_command':
                    handleDeviceCommand(clientId, message);
                    break;
                
                case 'subscribe':
                    handleSubscription(clientId, message);
                    break;
                
                case 'unsubscribe':
                    handleUnsubscription(clientId, message);
                    break;
                
                case 'request_data':
                    handleDataRequest(clientId, message);
                    break;
                
                default:
                    // Let existing handler deal with unknown types
                    break;
            }
        } catch (error) {
            console.error(`❌ Error handling message from ${clientId}:`, error);
            sendToClient(clientId, {
                type: 'error',
                message: 'Failed to process message',
                error: error.message,
                timestamp: new Date().toISOString()
            });
        }
    };
}

// === CONTROL COMMAND HANDLERS ===

function handleControlCommand(clientId, message) {
    const { deviceId, command, data } = message;
    
    if (!deviceId) {
        return sendError(clientId, 'Device ID is required for control commands');
    }
    
    console.log(`🎮 Control command from ${clientId}: ${command} for device ${deviceId}`);
    
    let result;
    
    switch (command) {
        case 'move':
            result = handleMoveCommand(deviceId, data);
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
            return sendError(clientId, `Unknown control command: ${command}`);
    }
    
    // Send response to client
    sendToClient(clientId, {
        type: 'control_response',
        deviceId: deviceId,
        command: command,
        result: result,
        timestamp: new Date().toISOString()
    });
    
    // Broadcast to other subscribers
    broadcastToSubscribers('control_events', {
        type: `${command}_executed`,
        deviceId: deviceId,
        data: data,
        result: result,
        initiatedBy: clientId,
        timestamp: new Date().toISOString()
    });
}

function handleMoveCommand(deviceId, data) {
    const { linear, angular, deadman } = data;
    
    if (deadman !== undefined) {
        // Joystick-style control with deadman switch
        return rosConnection.publishJoystick(angular || 0, linear || 0, deadman);
    } else {
        // Direct velocity control
        return rosConnection.publishVelocity(linear || 0, angular || 0);
    }
}

// === JOYSTICK CONTROL HANDLER ===

function handleJoystickControl(clientId, message) {
    const { deviceId, x, y, deadman } = message;
    
    if (!deviceId) {
        return sendError(clientId, 'Device ID is required for joystick control');
    }
    
    // Validate joystick input
    if (typeof x !== 'number' || typeof y !== 'number') {
        return sendError(clientId, 'Invalid joystick input: x and y must be numbers');
    }
    
    // Clamp values to safe range
    const clampedX = Math.max(-1, Math.min(1, x));
    const clampedY = Math.max(-1, Math.min(1, y));
    
    const result = rosConnection.publishJoystick(clampedX, clampedY, deadman);
    
    // Send immediate response (for real-time control)
    sendToClient(clientId, {
        type: 'joystick_response',
        deviceId: deviceId,
        result: result,
        timestamp: new Date().toISOString()
    });
    
    // Broadcast to other clients (less frequently to avoid spam)
    if (!global.lastJoystickBroadcast || Date.now() - global.lastJoystickBroadcast > 200) {
        broadcastToSubscribers('control_events', {
            type: 'joystick_update',
            deviceId: deviceId,
            data: { x: clampedX, y: clampedY, deadman },
            timestamp: new Date().toISOString()
        }, clientId); // Exclude sender
        
        global.lastJoystickBroadcast = Date.now();
    }
}

// === MAPPING COMMAND HANDLERS ===

function handleMappingCommand(clientId, message) {
    const { deviceId, action, data } = message;
    
    let result;
    
    switch (action) {
        case 'start':
            result = rosConnection.startMapping();
            break;
        
        case 'stop':
            result = rosConnection.stopMapping();
            break;
        
        case 'save':
            result = handleSaveMap(deviceId, data);
            break;
        
        case 'status':
            result = {
                success: true,
                data: rosConnection.getMappingStatus(deviceId)
            };
            break;
        
        default:
            return sendError(clientId, `Unknown mapping action: ${action}`);
    }
    
    sendToClient(clientId, {
        type: 'mapping_response',
        deviceId: deviceId,
        action: action,
        result: result,
        timestamp: new Date().toISOString()
    });
    
    // Broadcast mapping events
    broadcastToSubscribers('mapping_events', {
        type: `mapping_${action}`,
        deviceId: deviceId,
        result: result,
        timestamp: new Date().toISOString()
    });
}

async function handleSaveMap(deviceId, data) {
    try {
        const { mapName, includeShapes } = data || {};
        const liveData = rosConnection.getLiveData(deviceId);
        
        if (!liveData.map) {
            return { success: false, error: 'No live map data available' };
        }
        
        const mapData = {
            ...liveData.map,
            name: mapName || `Map_${deviceId}_${new Date().toISOString()}`,
            savedAt: new Date().toISOString(),
            shapes: includeShapes ? (global.deviceMaps?.[deviceId]?.shapes || []) : []
        };
        
        global.deviceMaps[deviceId] = mapData;
        await storageManager.saveMap(deviceId);
        
        return {
            success: true,
            message: 'Map saved successfully',
            mapName: mapData.name
        };
        
    } catch (error) {
        console.error('❌ Error saving map:', error);
        return {
            success: false,
            error: error.message
        };
    }
}

// === MAP EDITING HANDLERS ===

function handleMapEdit(clientId, message) {
    const { deviceId, editType, editData } = message;
    
    if (!deviceId) {
        return sendError(clientId, 'Device ID is required for map editing');
    }
    
    let result;
    
    switch (editType) {
        case 'add_shape':
            result = handleAddShape(deviceId, editData);
            break;
        
        case 'update_shape':
            result = handleUpdateShape(deviceId, editData);
            break;
        
        case 'delete_shape':
            result = handleDeleteShape(deviceId, editData);
            break;
        
        case 'add_annotation':
            result = handleAddAnnotation(deviceId, editData);
            break;
        
        default:
            return sendError(clientId, `Unknown map edit type: ${editType}`);
    }
    
    sendToClient(clientId, {
        type: 'map_edit_response',
        deviceId: deviceId,
        editType: editType,
        result: result,
        timestamp: new Date().toISOString()
    });
    
    // Broadcast map changes
    if (result.success) {
        broadcastToSubscribers('map_events', {
            type: 'map_edited',
            deviceId: deviceId,
            editType: editType,
            editData: editData,
            timestamp: new Date().toISOString()
        }, clientId);
    }
}

function handleAddShape(deviceId, shapeData) {
    try {
        const { type, name, points, sides, color } = shapeData;
        
        const newShape = {
            id: `shape_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
            type: type || 'waypoint',
            name: name || 'Unnamed Shape',
            points: points || [],
            sides: sides || { left: '', right: '', front: '', back: '' },
            color: color || '#FF0000FF',
            createdAt: new Date().toISOString()
        };
        
        // Initialize map if not exists
        if (!global.deviceMaps[deviceId]) {
            global.deviceMaps[deviceId] = {
                deviceId: deviceId,
                shapes: [],
                annotations: [],
                version: 1
            };
        }
        
        global.deviceMaps[deviceId].shapes.push(newShape);
        global.deviceMaps[deviceId].version = (global.deviceMaps[deviceId].version || 0) + 1;
        
        // Save asynchronously
        storageManager.saveMap(deviceId).catch(console.error);
        
        return {
            success: true,
            message: 'Shape added successfully',
            shape: newShape
        };
        
    } catch (error) {
        return {
            success: false,
            error: error.message
        };
    }
}

function handleUpdateShape(deviceId, editData) {
    try {
        const { id, ...updates } = editData;
        const mapData = global.deviceMaps[deviceId];
        
        if (!mapData || !mapData.shapes) {
            return { success: false, error: 'No map data found' };
        }
        
        const shapeIndex = mapData.shapes.findIndex(s => s.id === id);
        if (shapeIndex === -1) {
            return { success: false, error: 'Shape not found' };
        }
        
        mapData.shapes[shapeIndex] = {
            ...mapData.shapes[shapeIndex],
            ...updates,
            updatedAt: new Date().toISOString()
        };
        
        mapData.version = (mapData.version || 0) + 1;
        
        // Save asynchronously
        storageManager.saveMap(deviceId).catch(console.error);
        
        return {
            success: true,
            message: 'Shape updated successfully',
            shape: mapData.shapes[shapeIndex]
        };
        
    } catch (error) {
        return {
            success: false,
            error: error.message
        };
    }
}

function handleDeleteShape(deviceId, editData) {
    try {
        const { id } = editData;
        const mapData = global.deviceMaps[deviceId];
        
        if (!mapData || !mapData.shapes) {
            return { success: false, error: 'No map data found' };
        }
        
        const initialLength = mapData.shapes.length;
        mapData.shapes = mapData.shapes.filter(s => s.id !== id);
        
        if (mapData.shapes.length === initialLength) {
            return { success: false, error: 'Shape not found' };
        }
        
        mapData.version = (mapData.version || 0) + 1;
        
        // Save asynchronously
        storageManager.saveMap(deviceId).catch(console.error);
        
        return {
            success: true,
            message: 'Shape deleted successfully'
        };
        
    } catch (error) {
        return {
            success: false,
            error: error.message
        };
    }
}

function handleAddAnnotation(deviceId, annotationData) {
    try {
        const { type, position, text, color } = annotationData;
        
        const newAnnotation = {
            id: `annotation_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
            type: type || 'text',
            position: position || { x: 0, y: 0 },
            text: text || '',
            color: color || '#000000FF',
            createdAt: new Date().toISOString()
        };
        
        // Initialize map if not exists
        if (!global.deviceMaps[deviceId]) {
            global.deviceMaps[deviceId] = {
                deviceId: deviceId,
                shapes: [],
                annotations: [],
                version: 1
            };
        }
        
        if (!global.deviceMaps[deviceId].annotations) {
            global.deviceMaps[deviceId].annotations = [];
        }
        
        global.deviceMaps[deviceId].annotations.push(newAnnotation);
        global.deviceMaps[deviceId].version = (global.deviceMaps[deviceId].version || 0) + 1;
        
        // Save asynchronously
        storageManager.saveMap(deviceId).catch(console.error);
        
        return {
            success: true,
            message: 'Annotation added successfully',
            annotation: newAnnotation
        };
        
    } catch (error) {
        return {
            success: false,
            error: error.message
        };
    }
}

// === SUBSCRIPTION HANDLERS ===

function handleSubscription(clientId, message) {
    const { topic, deviceId } = message;
    const client = global.connectedClients?.get(clientId);
    
    if (!client) return;
    
    client.subscribedTopics.add(topic);
    
    if (deviceId) {
        client.subscriptions.devices.add(deviceId);
    }
    
    sendToClient(clientId, {
        type: 'subscription_ack',
        topic: topic,
        deviceId: deviceId,
        timestamp: new Date().toISOString()
    });
    
    // Send initial data for the subscribed topic
    sendInitialDataForTopic(clientId, topic, deviceId);
    
    console.log(`📡 Client ${clientId} subscribed to topic: ${topic}${deviceId ? ` for device: ${deviceId}` : ''}`);
}

function handleUnsubscription(clientId, message) {
    const { topic, deviceId } = message;
    const client = global.connectedClients?.get(clientId);
    
    if (!client) return;
    
    client.subscribedTopics.delete(topic);
    
    if (deviceId) {
        client.subscriptions.devices.delete(deviceId);
    }
    
    sendToClient(clientId, {
        type: 'unsubscription_ack',
        topic: topic,
        deviceId: deviceId,
        timestamp: new Date().toISOString()
    });
    
    console.log(`📡 Client ${clientId} unsubscribed from topic: ${topic}${deviceId ? ` for device: ${deviceId}` : ''}`);
}

// === DATA REQUEST HANDLERS ===

function handleDataRequest(clientId, message) {
    const { requestType, deviceId } = message;
    
    let responseData;
    
    switch (requestType) {
        case 'devices':
            responseData = global.connectedDevices || [];
            break;
        
        case 'device_status':
            if (deviceId) {
                responseData = {
                    device: global.connectedDevices?.find(d => d.id === deviceId),
                    liveData: rosConnection.getLiveData(deviceId),
                    mappingStatus: rosConnection.getMappingStatus(deviceId)
                };
            }
            break;
        
        case 'map':
            if (deviceId) {
                responseData = {
                    savedMap: global.deviceMaps?.[deviceId],
                    liveMap: rosConnection.getLiveData(deviceId)?.map
                };
            }
            break;
        
        case 'orders':
            responseData = deviceId ? global.deviceOrders?.[deviceId] : global.deviceOrders;
            break;
        
        case 'ros_status':
            responseData = rosConnection.getROS2Status();
            break;
        
        default:
            return sendError(clientId, `Unknown request type: ${requestType}`);
    }
    
    sendToClient(clientId, {
        type: 'data_response',
        requestType: requestType,
        deviceId: deviceId,
        data: responseData,
        timestamp: new Date().toISOString()
    });
}

// === HELPER FUNCTIONS ===

function sendInitialDataForTopic(clientId, topic, deviceId) {
    let initialData;
    
    switch (topic) {
        case 'real_time_data':
            if (deviceId) {
                initialData = rosConnection.getLiveData(deviceId);
            } else {
                initialData = rosConnection.getLiveData();
            }
            break;
        
        case 'device_events':
            initialData = {
                type: 'initial_devices',
                devices: global.connectedDevices || []
            };
            break;
        
        case 'map_events':
            if (deviceId) {
                initialData = {
                    type: 'initial_map',
                    deviceId: deviceId,
                    mapData: global.deviceMaps?.[deviceId]
                };
            }
            break;
    }
    
    if (initialData) {
        sendToClient(clientId, {
            type: 'initial_data',
            topic: topic,
            deviceId: deviceId,
            data: initialData,
            timestamp: new Date().toISOString()
        });
    }
}

function sendError(clientId, message) {
    sendToClient(clientId, {
        type: 'error',
        message: message,
        timestamp: new Date().toISOString()
    });
}

// These functions should be imported or defined in the WebSocket module
function sendToClient(clientId, message) {
    // This should reference the actual sendToClient function from clientConnection.js
    if (global.webSocketInstance && global.webSocketInstance.sendToClient) {
        global.webSocketInstance.sendToClient(clientId, message);
    }
}

function broadcastToSubscribers(topic, message, excludeClientId = null) {
    // This should reference the actual broadcastToSubscribers function from clientConnection.js
    if (global.webSocketInstance && global.webSocketInstance.broadcastToSubscribers) {
        global.webSocketInstance.broadcastToSubscribers(topic, message, excludeClientId);
    }
}

module.exports = {
    enhanceWebSocketHandlers,
    handleControlCommand,
    handleJoystickControl,
    handleMappingCommand,
    handleMapEdit,
    handleOrderCommand: () => {}, // TODO: Implement order management
    handleDeviceCommand: () => {}, // TODO: Implement device management
};