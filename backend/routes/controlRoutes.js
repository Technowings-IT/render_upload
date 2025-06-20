// routes/controlRoutes.js - Enhanced Control Routes for AGV Fleet Management
const express = require('express');
const router = express.Router();
const publishers = require('../ros/utils/publishers');
const { realAGVIntegration } = require('../ros/utils/subscribers');
const { broadcastToSubscribers } = require('../websocket/clientConnection');
const config = require('../config');
const fs = require('fs-extra');
const path = require('path');

// Middleware for device validation
function validateDevice(req, res, next) {
    const { deviceId } = req.params;
    
    if (!deviceId) {
        return res.status(400).json({
            success: false,
            message: 'Device ID is required'
        });
    }

    const device = global.connectedDevices?.find(d => d.id === deviceId);
    if (!device) {
        return res.status(404).json({
            success: false,
            message: 'Device not found or not connected'
        });
    }

    req.device = device;
    next();
}

// Movement control endpoints
router.post('/device/:deviceId/move', validateDevice, async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { linear, angular } = req.body;

        if (linear === undefined || angular === undefined) {
            return res.status(400).json({
                success: false,
                message: 'Linear and angular velocities are required'
            });
        }

        // Validate velocity limits
        if (Math.abs(linear) > config.AGV.MAX_LINEAR_SPEED) {
            return res.status(400).json({
                success: false,
                message: `Linear velocity exceeds maximum: ${config.AGV.MAX_LINEAR_SPEED} m/s`
            });
        }

        if (Math.abs(angular) > config.AGV.MAX_ANGULAR_SPEED) {
            return res.status(400).json({
                success: false,
                message: `Angular velocity exceeds maximum: ${config.AGV.MAX_ANGULAR_SPEED} rad/s`
            });
        }

        const result = publishers.publishVelocity(deviceId, linear, angular);
        
        // Update device last seen
        req.device.lastSeen = new Date().toISOString();
        req.device.lastCommand = 'move';

        // Broadcast movement command
        broadcastToSubscribers('control_events', {
            type: 'movement_command',
            deviceId,
            command: { linear, angular },
            timestamp: new Date().toISOString()
        });
        
        if (result.success) {
            res.json({
                success: true,
                message: 'Movement command sent successfully',
                linear: result.linear,
                angular: result.angular,
                timestamp: new Date().toISOString()
            });
        } else {
            res.status(500).json({
                success: false,
                message: 'Failed to send movement command',
                error: result.error
            });
        }
    } catch (error) {
        console.error('Error sending movement command:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to send movement command',
            error: error.message
        });
    }
});

router.post('/device/:deviceId/stop', validateDevice, async (req, res) => {
    try {
        const { deviceId } = req.params;
        
        const result = publishers.emergencyStop(deviceId);
        
        // Update device status
        req.device.lastSeen = new Date().toISOString();
        req.device.lastCommand = 'stop';

        // Broadcast stop command
        broadcastToSubscribers('control_events', {
            type: 'stop_command',
            deviceId,
            timestamp: new Date().toISOString()
        });
        
        if (result.success) {
            res.json({
                success: true,
                message: 'Stop command sent successfully',
                timestamp: new Date().toISOString()
            });
        } else {
            res.status(500).json({
                success: false,
                message: 'Failed to send stop command',
                error: result.error
            });
        }
    } catch (error) {
        console.error('Error sending stop command:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to send stop command',
            error: error.message
        });
    }
});

// Joystick control with deadman switch
router.post('/device/:deviceId/joystick', validateDevice, async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { x, y, deadman, sensitivity } = req.body;

        if (deadman === undefined) {
            return res.status(400).json({
                success: false,
                message: 'Deadman switch state is required for joystick control'
            });
        }

        let linear = 0;
        let angular = 0;

        if (deadman) {
            const sens = sensitivity || 1.0;
            linear = (y || 0) * config.AGV.MAX_LINEAR_SPEED * sens;
            angular = -(x || 0) * config.AGV.MAX_ANGULAR_SPEED * sens; // Invert for correct turning
        }

        const result = publishers.publishVelocity(deviceId, linear, angular);
        
        // Update device status
        req.device.lastSeen = new Date().toISOString();
        req.device.lastCommand = 'joystick';

        // Broadcast joystick command (less frequently to avoid spam)
        if (Math.random() < 0.1) { // 10% of joystick commands
            broadcastToSubscribers('control_events', {
                type: 'joystick_command',
                deviceId,
                command: { x, y, deadman, linear, angular },
                timestamp: new Date().toISOString()
            });
        }
        
        if (result.success) {
            res.json({
                success: true,
                message: 'Joystick command processed',
                input: { x, y, deadman },
                output: { linear: result.linear, angular: result.angular }
            });
        } else {
            res.status(500).json({
                success: false,
                message: 'Failed to process joystick command',
                error: result.error
            });
        }
    } catch (error) {
        console.error('Error processing joystick command:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to process joystick command',
            error: error.message
        });
    }
});

// Navigation control
router.post('/device/:deviceId/goal', validateDevice, async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { x, y, orientation, frame_id } = req.body;

        if (x === undefined || y === undefined) {
            return res.status(400).json({
                success: false,
                message: 'X and Y coordinates are required'
            });
        }

        const result = publishers.publishGoal(deviceId, x, y, orientation || 0);
        
        // Update device status
        req.device.lastSeen = new Date().toISOString();
        req.device.lastCommand = 'goal';

        // Broadcast goal command
        broadcastToSubscribers('control_events', {
            type: 'goal_command',
            deviceId,
            goal: { x, y, orientation: orientation || 0, frame_id: frame_id || 'map' },
            timestamp: new Date().toISOString()
        });
        
        if (result.success) {
            res.json({
                success: true,
                message: 'Goal sent successfully',
                goal: { x: result.x, y: result.y, orientation: result.orientation },
                timestamp: new Date().toISOString()
            });
        } else {
            res.status(500).json({
                success: false,
                message: 'Failed to send goal',
                error: result.error
            });
        }
    } catch (error) {
        console.error('Error sending goal:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to send goal',
            error: error.message
        });
    }
});

router.post('/device/:deviceId/initial-pose', validateDevice, async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { x, y, orientation } = req.body;

        if (x === undefined || y === undefined) {
            return res.status(400).json({
                success: false,
                message: 'X and Y coordinates are required'
            });
        }

        const result = publishers.setInitialPose(deviceId, x, y, orientation || 0);
        
        // Update device status
        req.device.lastSeen = new Date().toISOString();
        req.device.lastCommand = 'initial_pose';

        // Broadcast initial pose command
        broadcastToSubscribers('control_events', {
            type: 'initial_pose_set',
            deviceId,
            pose: { x, y, orientation: orientation || 0 },
            timestamp: new Date().toISOString()
        });
        
        if (result.success) {
            res.json({
                success: true,
                message: 'Initial pose set successfully',
                pose: { x: result.x, y: result.y, orientation: result.orientation },
                timestamp: new Date().toISOString()
            });
        } else {
            res.status(500).json({
                success: false,
                message: 'Failed to set initial pose',
                error: result.error
            });
        }
    } catch (error) {
        console.error('Error setting initial pose:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to set initial pose',
            error: error.message
        });
    }
});

// Mapping control
router.post('/device/:deviceId/mapping/start', validateDevice, async (req, res) => {
    try {
        const { deviceId } = req.params;
        
        const result = await publishers.startMapping(deviceId);
        
        // Update device status
        req.device.lastSeen = new Date().toISOString();
        req.device.lastCommand = 'start_mapping';
        req.device.mappingActive = true;

        // Broadcast mapping started
        broadcastToSubscribers('mapping_events', {
            type: 'mapping_started',
            deviceId,
            timestamp: new Date().toISOString()
        });
        
        if (result.success) {
            res.json({
                success: true,
                message: 'Mapping started successfully',
                timestamp: new Date().toISOString()
            });
        } else {
            res.status(500).json({
                success: false,
                message: 'Failed to start mapping',
                error: result.error
            });
        }
    } catch (error) {
        console.error('Error starting mapping:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to start mapping',
            error: error.message
        });
    }
});

router.post('/device/:deviceId/mapping/stop', validateDevice, async (req, res) => {
    try {
        const { deviceId } = req.params;
        
        const result = await publishers.stopMapping(deviceId);
        
        // Update device status
        req.device.lastSeen = new Date().toISOString();
        req.device.lastCommand = 'stop_mapping';
        req.device.mappingActive = false;

        // Broadcast mapping stopped
        broadcastToSubscribers('mapping_events', {
            type: 'mapping_stopped',
            deviceId,
            timestamp: new Date().toISOString()
        });
        
        if (result.success) {
            res.json({
                success: true,
                message: 'Mapping stopped successfully',
                timestamp: new Date().toISOString()
            });
        } else {
            res.status(500).json({
                success: false,
                message: 'Failed to stop mapping',
                error: result.error
            });
        }
    } catch (error) {
        console.error('Error stopping mapping:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to stop mapping',
            error: error.message
        });
    }
});

router.post('/device/:deviceId/mapping/save', validateDevice, async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, includeAnnotations } = req.body;
        
        if (!mapName) {
            return res.status(400).json({
                success: false,
                message: 'Map name is required'
            });
        }
        
        // Save via ROS2 service
        const result = await publishers.saveMap(deviceId, mapName);
        
        // Also save current map data with annotations if requested
        if (includeAnnotations && global.deviceMaps[deviceId]) {
            const mapWithAnnotations = {
                ...global.deviceMaps[deviceId],
                rosMapName: mapName,
                savedAt: new Date().toISOString()
            };
            
            const mapPath = path.join(config.STORAGE.MAPS_DIR, `${deviceId}_${mapName}.json`);
            await fs.writeJson(mapPath, mapWithAnnotations, { spaces: 2 });
        }
        
        // Update device status
        req.device.lastSeen = new Date().toISOString();
        req.device.lastCommand = 'save_map';
        req.device.lastMapSaved = mapName;

        // Broadcast map saved
        broadcastToSubscribers('mapping_events', {
            type: 'map_saved',
            deviceId,
            mapName,
            timestamp: new Date().toISOString()
        });
        
        if (result.success) {
            res.json({
                success: true,
                message: 'Map saved successfully',
                mapName: result.mapName || mapName,
                timestamp: new Date().toISOString()
            });
        } else {
            res.status(500).json({
                success: false,
                message: 'Failed to save map',
                error: result.error
            });
        }
    } catch (error) {
        console.error('Error saving map:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to save map',
            error: error.message
        });
    }
});

// Map editing endpoints
router.post('/device/:deviceId/map/shape/add', validateDevice, async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { shape } = req.body;

        if (!shape || !shape.type || !shape.coordinates) {
            return res.status(400).json({
                success: false,
                message: 'Shape with type and coordinates is required'
            });
        }

        // Initialize map if it doesn't exist
        if (!global.deviceMaps[deviceId]) {
            global.deviceMaps[deviceId] = {
                shapes: [],
                annotations: [],
                metadata: { deviceId }
            };
        }

        // Validate shape type
        if (!config.MAPS.SHAPES[shape.type.toUpperCase()]) {
            return res.status(400).json({
                success: false,
                message: 'Invalid shape type',
                validTypes: Object.keys(config.MAPS.SHAPES)
            });
        }

        // Create shape with ID and metadata
        const newShape = {
            id: `shape_${Date.now()}_${Math.random().toString(36).substr(2, 6)}`,
            type: shape.type,
            coordinates: shape.coordinates,
            properties: {
                ...shape.properties,
                color: shape.properties?.color || config.MAPS.SHAPES[shape.type.toUpperCase()].color,
                sides: shape.properties?.sides || config.MAPS.SHAPES[shape.type.toUpperCase()].sides,
                name: shape.properties?.name || `${shape.type}_${Date.now()}`,
                createdAt: new Date().toISOString(),
                createdBy: 'api'
            }
        };

        global.deviceMaps[deviceId].shapes.push(newShape);

        // Broadcast real-time update
        broadcastToSubscribers('map_events', {
            type: 'shape_added',
            deviceId,
            shape: newShape,
            timestamp: new Date().toISOString()
        });

        res.json({
            success: true,
            message: 'Shape added successfully',
            shape: newShape
        });

    } catch (error) {
        console.error('Error adding shape:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to add shape',
            error: error.message
        });
    }
});

router.put('/device/:deviceId/map/shape/:shapeId', validateDevice, async (req, res) => {
    try {
        const { deviceId, shapeId } = req.params;
        const { updates } = req.body;

        if (!global.deviceMaps[deviceId]) {
            return res.status(404).json({
                success: false,
                message: 'No map found for this device'
            });
        }

        const shapeIndex = global.deviceMaps[deviceId].shapes.findIndex(s => s.id === shapeId);
        if (shapeIndex === -1) {
            return res.status(404).json({
                success: false,
                message: 'Shape not found'
            });
        }

        // Update shape
        const shape = global.deviceMaps[deviceId].shapes[shapeIndex];
        Object.assign(shape, updates, {
            properties: {
                ...shape.properties,
                ...updates.properties,
                updatedAt: new Date().toISOString()
            }
        });

        // Broadcast real-time update
        broadcastToSubscribers('map_events', {
            type: 'shape_updated',
            deviceId,
            shape,
            timestamp: new Date().toISOString()
        });

        res.json({
            success: true,
            message: 'Shape updated successfully',
            shape
        });

    } catch (error) {
        console.error('Error updating shape:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to update shape',
            error: error.message
        });
    }
});

router.delete('/device/:deviceId/map/shape/:shapeId', validateDevice, async (req, res) => {
    try {
        const { deviceId, shapeId } = req.params;

        if (!global.deviceMaps[deviceId]) {
            return res.status(404).json({
                success: false,
                message: 'No map found for this device'
            });
        }

        const shapeIndex = global.deviceMaps[deviceId].shapes.findIndex(s => s.id === shapeId);
        if (shapeIndex === -1) {
            return res.status(404).json({
                success: false,
                message: 'Shape not found'
            });
        }

        // Remove shape
        const removedShape = global.deviceMaps[deviceId].shapes.splice(shapeIndex, 1)[0];

        // Broadcast real-time update
        broadcastToSubscribers('map_events', {
            type: 'shape_removed',
            deviceId,
            shapeId,
            timestamp: new Date().toISOString()
        });

        res.json({
            success: true,
            message: 'Shape removed successfully',
            removedShape
        });

    } catch (error) {
        console.error('Error removing shape:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to remove shape',
            error: error.message
        });
    }
});

// Status and data retrieval
router.get('/device/:deviceId/status', validateDevice, (req, res) => {
    try {
        const { deviceId } = req.params;
        
        const deviceData = realAGVIntegration.getDeviceData(deviceId);
        const liveData = global.liveData?.[deviceId];
        
        const status = {
            device: req.device,
            liveData,
            rosData: deviceData,
            hasMap: !!global.deviceMaps[deviceId],
            activeOrders: (global.deviceOrders[deviceId] || []).filter(order => 
                ['pending', 'active'].includes(order.status)
            ).length,
            connectionHealth: {
                lastSeen: req.device.lastSeen,
                timeSinceLastSeen: Date.now() - new Date(req.device.lastSeen).getTime(),
                isHealthy: Date.now() - new Date(req.device.lastSeen).getTime() < 30000 // 30 seconds
            }
        };
        
        res.json({
            success: true,
            status,
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error('Error getting device status:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to get device status',
            error: error.message
        });
    }
});

router.get('/devices', (req, res) => {
    try {
        const devices = global.connectedDevices || [];
        const devicesWithData = devices.map(device => ({
            ...device,
            liveData: global.liveData?.[device.id],
            rosData: realAGVIntegration.getDeviceData(device.id),
            hasMap: !!global.deviceMaps[device.id],
            activeOrders: (global.deviceOrders[device.id] || []).filter(order => 
                ['pending', 'active'].includes(order.status)
            ).length
        }));
        
        res.json({
            success: true,
            devices: devicesWithData,
            count: devicesWithData.length,
            summary: {
                total: devicesWithData.length,
                connected: devicesWithData.filter(d => d.status === 'connected').length,
                withMaps: devicesWithData.filter(d => d.hasMap).length,
                withActiveOrders: devicesWithData.filter(d => d.activeOrders > 0).length
            }
        });
    } catch (error) {
        console.error('Error getting devices:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to get devices',
            error: error.message
        });
    }
});

// Health check
router.get('/health', (req, res) => {
    res.json({
        success: true,
        message: 'AGV Control API is running',
        timestamp: new Date().toISOString(),
        connectedDevices: global.connectedDevices?.length || 0,
        totalMaps: Object.keys(global.deviceMaps || {}).length,
        totalOrders: Object.values(global.deviceOrders || {}).reduce((acc, orders) => acc + orders.length, 0),
        ros2Status: realAGVIntegration ? 'active' : 'inactive'
    });
});

module.exports = router;