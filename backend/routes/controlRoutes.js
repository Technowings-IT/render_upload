// routes/controlRoutes.js - Enhanced control routes for AGV fleet management
const express = require('express');
const router = express.Router();

const rosConnection = require('../ros/utils/ros_connection');
const { broadcastToSubscribers } = require('../websocket/clientConnection');
const storageManager = require('../ros/utils/storageManager');
const publishers = require('../ros/utils/publishers');
const pgmConverter = require('../ros/utils/pgmConverter');
const path = require('path');

// Initialize global storage if not exists
if (!global.deviceOrders) global.deviceOrders = {};
if (!global.connectedDevices) global.connectedDevices = [];
if (!global.deviceMaps) global.deviceMaps = {};

// Middleware for device validation
function validateDevice(req, res, next) {
    const { deviceId } = req.params;
    if (!deviceId) {
        return res.status(400).json({ error: 'Device ID is required' });
    }
    req.deviceId = deviceId;
    next();
}

// Middleware for order validation
function validateOrder(req, res, next) {
    const { orderId } = req.params;
    if (!orderId) {
        return res.status(400).json({ error: 'Order ID is required' });
    }
    req.orderId = orderId;
    next();
}

// === DEVICE MANAGEMENT ===

// Get all connected devices with enhanced info
router.get('/devices', (req, res) => {
    try {
        const devices = global.connectedDevices || [];
        const liveData = global.liveData || {};

        const devicesWithStatus = devices.map(device => {
            const deviceOrders = global.deviceOrders[device.id] || [];
            const deviceMap = global.deviceMaps[device.id];

            return {
                ...device,
                liveData: liveData[device.id] || {},
                isOnline: !!liveData[device.id]?.lastUpdate,
                orderCount: deviceOrders.length,
                activeOrders: deviceOrders.filter(o => o.status === 'active').length,
                hasMap: !!deviceMap,
                mapInfo: deviceMap ? {
                    width: deviceMap.info?.width,
                    height: deviceMap.info?.height,
                    shapes: deviceMap.shapes?.length || 0,
                    resolution: deviceMap.info?.resolution
                } : null
            };
        });

        res.json({
            success: true,
            devices: devicesWithStatus,
            total: devices.length,
            timestamp: new Date().toISOString()
        });

    } catch (error) {
        console.error('❌ Error getting devices:', error);
        res.status(500).json({ error: 'Failed to get devices' });
    }
});

// Connect a new device
router.post('/devices/connect', async (req, res) => {
    try {
        const { deviceId, name, type, ipAddress, capabilities } = req.body;

        if (!deviceId) {
            return res.status(400).json({ error: 'Device ID is required' });
        }

        // Check if device already exists
        const existingDeviceIndex = global.connectedDevices.findIndex(d => d.id === deviceId);

        const deviceInfo = {
            id: deviceId,
            name: name || `AGV ${deviceId}`,
            type: type || 'differential_drive',
            ipAddress: ipAddress,
            capabilities: capabilities || ['mapping', 'navigation', 'remote_control'],
            status: 'connected',
            connectedAt: new Date().toISOString(),
            lastSeen: new Date().toISOString()
        };

        if (existingDeviceIndex >= 0) {
            // Update existing device
            global.connectedDevices[existingDeviceIndex] = deviceInfo;
        } else {
            // Add new device
            global.connectedDevices.push(deviceInfo);
            if (!global.deviceOrders[deviceId]) {
                global.deviceOrders[deviceId] = [];
            }
        }

        // Broadcast device connection
        broadcastToSubscribers('device_events', {
            type: 'device_connected',
            deviceId: deviceId,
            device: deviceInfo,
            timestamp: new Date().toISOString()
        });

        res.json({
            success: true,
            message: 'Device connected successfully',
            deviceId: deviceId,
            device: deviceInfo
        });

        console.log(`✅ Device ${deviceId} connected via API`);

    } catch (error) {
        console.error('❌ Error connecting device:', error);
        res.status(500).json({
            error: 'Failed to connect device',
            details: error.message
        });
    }
});

// === ORDER MANAGEMENT APIS ===

// Get all orders for a device
router.get('/devices/:deviceId/orders', validateDevice, (req, res) => {
    try {
        const orders = global.deviceOrders[req.deviceId] || [];
        const sortedOrders = orders.sort((a, b) => new Date(b.createdAt || 0) - new Date(a.createdAt || 0));
        res.json({
            success: true,
            deviceId: req.deviceId,
            orders: sortedOrders,
            total: orders.length,
            active: orders.filter(o => o.status === 'active').length,
            pending: orders.filter(o => o.status === 'pending').length,
            completed: orders.filter(o => o.status === 'completed').length,
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error('❌ Error getting orders:', error);
        res.status(500).json({ error: 'Failed to get orders' });
    }
});

// Create new order with waypoint sequence
router.post('/devices/:deviceId/orders', validateDevice, async (req, res) => {
    try {
        const { name, waypoints, priority = 0, description } = req.body;
        if (!name || !waypoints || !Array.isArray(waypoints)) {
            return res.status(400).json({ error: 'Order name and waypoints array are required' });
        }
        if (waypoints.length === 0) {
            return res.status(400).json({ error: 'At least one waypoint is required' });
        }
        for (let i = 0; i < waypoints.length; i++) {
            const waypoint = waypoints[i];
            if (!waypoint.name || !waypoint.type || !waypoint.position) {
                return res.status(400).json({ error: `Invalid waypoint at index ${i}: missing name, type, or position` });
            }
        }
        const orderId = `order_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
        const newOrder = {
            id: orderId,
            name: name.trim(),
            description: description || '',
            deviceId: req.deviceId,
            waypoints: waypoints,
            priority: Math.max(0, Math.min(10, parseInt(priority) || 0)),
            status: 'pending',
            progress: {
                currentWaypoint: 0,
                totalWaypoints: waypoints.length,
                completedWaypoints: 0
            },
            createdAt: new Date().toISOString(),
            updatedAt: new Date().toISOString(),
            startedAt: null,
            completedAt: null
        };
        if (!global.deviceOrders[req.deviceId]) {
            global.deviceOrders[req.deviceId] = [];
        }
        global.deviceOrders[req.deviceId].push(newOrder);
        try { await storageManager.saveOrder(req.deviceId, orderId); } catch (storageError) { console.warn('⚠️ Failed to save order to storage:', storageError); }
        broadcastToSubscribers('order_events', {
            type: 'order_created',
            deviceId: req.deviceId,
            order: newOrder,
            timestamp: new Date().toISOString()
        });
        res.status(201).json({
            success: true,
            message: 'Order created successfully',
            order: newOrder,
            sequencePreview: waypoints.map((wp, index) => ({
                step: index + 1,
                name: wp.name,
                type: wp.type,
                position: wp.position
            }))
        });
        console.log(`📋 Order created: ${newOrder.name} for device ${req.deviceId} (${waypoints.length} waypoints)`);
    } catch (error) {
        console.error('❌ Error creating order:', error);
        res.status(500).json({ error: 'Failed to create order', details: error.message });
    }
});

// Get specific order details
router.get('/devices/:deviceId/orders/:orderId', validateDevice, validateOrder, (req, res) => {
    try {
        const orders = global.deviceOrders[req.deviceId] || [];
        const order = orders.find(o => o.id === req.orderId);
        if (!order) {
            return res.status(404).json({ error: 'Order not found' });
        }
        res.json({
            success: true,
            order: order,
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error('❌ Error getting order:', error);
        res.status(500).json({ error: 'Failed to get order' });
    }
});

// Update order status
router.put('/orders/:orderId/status', validateOrder, async (req, res) => {
    try {
        const { status, currentWaypoint } = req.body;
        if (!status) {
            return res.status(400).json({ error: 'Status is required' });
        }
        const validStatuses = ['pending', 'active', 'paused', 'completed', 'failed'];
        if (!validStatuses.includes(status)) {
            return res.status(400).json({ error: `Invalid status. Must be one of: ${validStatuses.join(', ')}` });
        }
        let orderFound = false;
        let deviceId = null;
        let updatedOrder = null;
        for (const [devId, orders] of Object.entries(global.deviceOrders)) {
            const orderIndex = orders.findIndex(o => o.id === req.orderId);
            if (orderIndex !== -1) {
                const order = orders[orderIndex];
                order.status = status;
                order.updatedAt = new Date().toISOString();
                if (currentWaypoint !== undefined) {
                    order.progress.currentWaypoint = Math.max(0, parseInt(currentWaypoint));
                    order.progress.completedWaypoints = Math.max(0, parseInt(currentWaypoint));
                }
                if (status === 'active' && !order.startedAt) {
                    order.startedAt = new Date().toISOString();
                } else if (status === 'completed' && !order.completedAt) {
                    order.completedAt = new Date().toISOString();
                    order.progress.completedWaypoints = order.progress.totalWaypoints;
                }
                orders[orderIndex] = order;
                deviceId = devId;
                updatedOrder = order;
                orderFound = true;
                break;
            }
        }
        if (!orderFound) {
            return res.status(404).json({ error: 'Order not found' });
        }
        try { await storageManager.saveOrder(deviceId, req.orderId); } catch (storageError) { console.warn('⚠️ Failed to save order status to storage:', storageError); }
        broadcastToSubscribers('order_events', {
            type: 'order_status_changed',
            deviceId: deviceId,
            orderId: req.orderId,
            status: status,
            order: updatedOrder,
            timestamp: new Date().toISOString()
        });
        res.json({
            success: true,
            message: 'Order status updated successfully',
            orderId: req.orderId,
            status: status,
            order: updatedOrder
        });
        console.log(`📋 Order ${req.orderId} status updated to: ${status}`);
    } catch (error) {
        console.error('❌ Error updating order status:', error);
        res.status(500).json({ error: 'Failed to update order status', details: error.message });
    }
});

// Execute order (send waypoints to AGV)
router.post('/devices/:deviceId/orders/:orderId/execute', validateDevice, validateOrder, async (req, res) => {
    try {
        const orders = global.deviceOrders[req.deviceId] || [];
        const order = orders.find(o => o.id === req.orderId);
        if (!order) {
            return res.status(404).json({ error: 'Order not found' });
        }
        if (order.status !== 'pending' && order.status !== 'paused') {
            return res.status(400).json({ error: `Cannot execute order with status: ${order.status}` });
        }
        const firstWaypoint = order.waypoints[0];
        if (!firstWaypoint) {
            return res.status(400).json({ error: 'Order has no waypoints' });
        }
        try {
            const goalResult = publishers.publishGoal(
                firstWaypoint.position.x,
                firstWaypoint.position.y,
                0
            );
            if (goalResult.success) {
                order.status = 'active';
                order.startedAt = new Date().toISOString();
                order.updatedAt = new Date().toISOString();
                order.progress.currentWaypoint = 0;
                broadcastToSubscribers('order_events', {
                    type: 'order_started',
                    deviceId: req.deviceId,
                    orderId: req.orderId,
                    order: order,
                    currentWaypoint: firstWaypoint,
                    timestamp: new Date().toISOString()
                });
                res.json({
                    success: true,
                    message: 'Order execution started',
                    order: order,
                    currentWaypoint: firstWaypoint,
                    goalResult: goalResult
                });
                console.log(`🚀 Order ${req.orderId} execution started - first waypoint: ${firstWaypoint.name}`);
            } else {
                res.status(500).json({
                    error: 'Failed to send waypoint to AGV',
                    details: goalResult.error
                });
            }
        } catch (rosError) {
            console.error('❌ ROS error during order execution:', rosError);
            res.status(500).json({
                error: 'Failed to communicate with AGV',
                details: rosError.message
            });
        }
    } catch (error) {
        console.error('❌ Error executing order:', error);
        res.status(500).json({ error: 'Failed to execute order', details: error.message });
    }
});

// Pause order execution
router.post('/devices/:deviceId/orders/:orderId/pause', validateDevice, validateOrder, async (req, res) => {
    try {
        const orders = global.deviceOrders[req.deviceId] || [];
        const order = orders.find(o => o.id === req.orderId);
        if (!order) {
            return res.status(404).json({ error: 'Order not found' });
        }
        if (order.status !== 'active') {
            return res.status(400).json({ error: `Cannot pause order with status: ${order.status}` });
        }
        const stopResult = publishers.publishVelocity(0, 0);
        order.status = 'paused';
        order.updatedAt = new Date().toISOString();
        broadcastToSubscribers('order_events', {
            type: 'order_paused',
            deviceId: req.deviceId,
            orderId: req.orderId,
            order: order,
            timestamp: new Date().toISOString()
        });
        res.json({
            success: true,
            message: 'Order paused successfully',
            order: order,
            stopResult: stopResult
        });
        console.log(`⏸️ Order ${req.orderId} paused`);
    } catch (error) {
        console.error('❌ Error pausing order:', error);
        res.status(500).json({ error: 'Failed to pause order', details: error.message });
    }
});

// Delete order
router.delete('/devices/:deviceId/orders/:orderId', validateDevice, validateOrder, async (req, res) => {
    try {
        const orders = global.deviceOrders[req.deviceId] || [];
        const orderIndex = orders.findIndex(o => o.id === req.orderId);
        if (orderIndex === -1) {
            return res.status(404).json({ error: 'Order not found' });
        }
        const order = orders[orderIndex];
        if (order.status === 'active') {
            return res.status(400).json({ error: 'Cannot delete active order. Pause it first.' });
        }
        orders.splice(orderIndex, 1);
        broadcastToSubscribers('order_events', {
            type: 'order_deleted',
            deviceId: req.deviceId,
            orderId: req.orderId,
            timestamp: new Date().toISOString()
        });
        res.json({
            success: true,
            message: 'Order deleted successfully',
            orderId: req.orderId
        });
        console.log(`🗑️ Order ${req.orderId} deleted`);
    } catch (error) {
        console.error('❌ Error deleting order:', error);
        res.status(500).json({ error: 'Failed to delete order', details: error.message });
    }
});

// Get all orders across all devices (for dashboard overview)
router.get('/orders', (req, res) => {
    try {
        const { status, deviceId, limit = 50 } = req.query;
        const allOrders = [];
        Object.entries(global.deviceOrders).forEach(([devId, orders]) => {
            orders.forEach(order => {
                allOrders.push({
                    ...order,
                    deviceId: devId,
                    deviceName: global.connectedDevices.find(d => d.id === devId)?.name || devId
                });
            });
        });
        let filteredOrders = allOrders;
        if (status) filteredOrders = allOrders.filter(o => o.status === status);
        if (deviceId) filteredOrders = filteredOrders.filter(o => o.deviceId === deviceId);
        filteredOrders.sort((a, b) => new Date(b.createdAt || 0) - new Date(a.createdAt || 0));
        const limitedOrders = filteredOrders.slice(0, parseInt(limit));
        const stats = {
            total: allOrders.length,
            pending: allOrders.filter(o => o.status === 'pending').length,
            active: allOrders.filter(o => o.status === 'active').length,
            paused: allOrders.filter(o => o.status === 'paused').length,
            completed: allOrders.filter(o => o.status === 'completed').length,
            failed: allOrders.filter(o => o.status === 'failed').length
        };
        res.json({
            success: true,
            orders: limitedOrders,
            stats: stats,
            filters: { status, deviceId, limit: parseInt(limit) },
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error('❌ Error getting all orders:', error);
        res.status(500).json({ error: 'Failed to get orders' });
    }
});

// === MAP MANAGEMENT FOR ORDER CREATION ===

// Get map data for order creation
router.get('/devices/:deviceId/map/stations', validateDevice, (req, res) => {
    try {
        const mapData = global.deviceMaps[req.deviceId];
        if (!mapData) {
            return res.status(404).json({
                error: 'No map found for this device',
                suggestion: 'Create a map first using the map editor'
            });
        }
        const stationsByType = {};
        const allStations = [];
        mapData.shapes.forEach(shape => {
            if (!stationsByType[shape.type]) stationsByType[shape.type] = [];
            const station = {
                id: shape.id,
                name: shape.name,
                type: shape.type,
                position: { x: shape.center.x, y: shape.center.y },
                createdAt: shape.createdAt
            };
            stationsByType[shape.type].push(station);
            allStations.push(station);
        });
        res.json({
            success: true,
            deviceId: req.deviceId,
            mapInfo: {
                width: mapData.info.width,
                height: mapData.info.height,
                resolution: mapData.info.resolution,
                totalStations: allStations.length
            },
            stationsByType: stationsByType,
            allStations: allStations,
            availableTypes: Object.keys(stationsByType),
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error('❌ Error getting map stations:', error);
        res.status(500).json({ error: 'Failed to get map stations' });
    }
});

// === (Your existing control, joystick, emergency stop, mapping, navigation, map, and utility endpoints remain unchanged) ===

// Connect/register a new device
router.post('/devices/:deviceId/connect', async (req, res) => {
    try {
        const { name, ipAddress, type, capabilities } = req.body;
        const deviceId = req.params.deviceId;

        if (!name || !ipAddress || !type) {
            return res.status(400).json({ error: 'Missing required fields' });
        }

        // Add or update device in global.connectedDevices
        let device = global.connectedDevices.find(d => d.id === deviceId);
        if (device) {
            // Update existing device
            Object.assign(device, { name, ipAddress, type, capabilities, status: 'connected', lastSeen: new Date().toISOString() });
        } else {
            // Add new device
            device = {
                id: deviceId,
                name,
                ipAddress,
                type,
                capabilities,
                status: 'connected',
                connectedAt: new Date().toISOString(),
                lastSeen: new Date().toISOString()
            };
            global.connectedDevices.push(device);
        }

        res.json({
            success: true,
            device: device,
            message: 'Device connected/registered successfully'
        });
        console.log(`✅ Device ${deviceId} connected/registered from ${ipAddress}`);
    } catch (error) {
        console.error('❌ Error connecting device:', error);
        res.status(500).json({ error: 'Failed to connect device' });
    }
});

// Get specific device status
router.get('/devices/:deviceId/status', validateDevice, (req, res) => {
    try {
        const device = global.connectedDevices?.find(d => d.id === req.deviceId);
        if (!device) {
            return res.status(404).json({ error: 'Device not found' });
        }

        const liveData = rosConnection.getLiveData(req.deviceId);
        const mappingStatus = rosConnection.getMappingStatus(req.deviceId);
        const rosStatus = rosConnection.getROS2Status();

        res.json({
            success: true,
            device: device,
            liveData: liveData,
            mappingStatus: mappingStatus,
            rosStatus: rosStatus,
            timestamp: new Date().toISOString()
        });

    } catch (error) {
        console.error('❌ Error getting device status:', error);
        res.status(500).json({ error: 'Failed to get device status' });
    }
});

// === JOYSTICK CONTROL ===

// Joystick control endpoint
router.post('/devices/:deviceId/joystick', validateDevice, (req, res) => {
    try {
        const { x, y, deadman } = req.body;

        // Validate input
        if (typeof x !== 'number' || typeof y !== 'number') {
            return res.status(400).json({ error: 'Invalid joystick input: x and y must be numbers' });
        }

        // Clamp values to [-1, 1] range
        const clampedX = Math.max(-1, Math.min(1, x));
        const clampedY = Math.max(-1, Math.min(1, y));

        // Publish joystick command
        const result = rosConnection.publishJoystick(clampedX, clampedY, deadman);

        // Broadcast control event
        broadcastToSubscribers('control_events', {
            type: 'joystick_command',
            deviceId: req.deviceId,
            data: { x: clampedX, y: clampedY, deadman },
            result: result,
            timestamp: new Date().toISOString()
        });

        res.json(result);

    } catch (error) {
        console.error('❌ Error processing joystick command:', error);
        res.status(500).json({
            error: 'Failed to process joystick command',
            details: error.message
        });
    }
});

// Direct velocity control
router.post('/devices/:deviceId/velocity', validateDevice, (req, res) => {
    try {
        const { linear, angular } = req.body;

        if (typeof linear !== 'number' || typeof angular !== 'number') {
            return res.status(400).json({ error: 'Invalid velocity input: linear and angular must be numbers' });
        }

        const result = publishers.publishVelocity(linear, angular);

        // Broadcast control event
        broadcastToSubscribers('control_events', {
            type: 'velocity_command',
            deviceId: req.deviceId,
            data: { linear, angular },
            result: result,
            timestamp: new Date().toISOString()
        });

        res.json(result);

    } catch (error) {
        console.error('❌ Error processing velocity command:', error);
        res.status(500).json({
            error: 'Failed to process velocity command',
            details: error.message
        });
    }
});

// Emergency stop
router.post('/devices/:deviceId/emergency-stop', validateDevice, (req, res) => {
    try {
        const result = rosConnection.emergencyStop();

        // Broadcast emergency stop
        broadcastToSubscribers('control_events', {
            type: 'emergency_stop',
            deviceId: req.deviceId,
            result: result,
            timestamp: new Date().toISOString()
        });

        console.log(`🛑 Emergency stop activated for device ${req.deviceId}`);

        res.json(result);

    } catch (error) {
        console.error('❌ Error processing emergency stop:', error);
        res.status(500).json({
            error: 'Failed to process emergency stop',
            details: error.message
        });
    }
});

// === MAPPING CONTROL ===

// Start mapping
router.post('/devices/:deviceId/mapping/start', validateDevice, (req, res) => {
    try {
        const result = rosConnection.startMapping();

        // Broadcast mapping event
        broadcastToSubscribers('mapping_events', {
            type: 'mapping_started',
            deviceId: req.deviceId,
            result: result,
            timestamp: new Date().toISOString()
        });

        console.log(`🗺️ Mapping started for device ${req.deviceId}`);

        res.json(result);

    } catch (error) {
        console.error('❌ Error starting mapping:', error);
        res.status(500).json({
            error: 'Failed to start mapping',
            details: error.message
        });
    }
});

// Stop mapping
router.post('/devices/:deviceId/mapping/stop', validateDevice, (req, res) => {
    try {
        const result = rosConnection.stopMapping();

        // Broadcast mapping event
        broadcastToSubscribers('mapping_events', {
            type: 'mapping_stopped',
            deviceId: req.deviceId,
            result: result,
            timestamp: new Date().toISOString()
        });

        console.log(`🛑 Mapping stopped for device ${req.deviceId}`);

        res.json(result);

    } catch (error) {
        console.error('❌ Error stopping mapping:', error);
        res.status(500).json({
            error: 'Failed to stop mapping',
            details: error.message
        });
    }
});

// Get mapping status
router.get('/devices/:deviceId/mapping/status', validateDevice, (req, res) => {
    try {
        const mappingStatus = rosConnection.getMappingStatus(req.deviceId);
        const liveData = rosConnection.getLiveData(req.deviceId);

        res.json({
            success: true,
            deviceId: req.deviceId,
            mappingStatus: mappingStatus,
            currentPosition: liveData.position || liveData.odometry?.position,
            trailLength: global.robotTrails?.[req.deviceId]?.length || 0,
            timestamp: new Date().toISOString()
        });

    } catch (error) {
        console.error('❌ Error getting mapping status:', error);
        res.status(500).json({ error: 'Failed to get mapping status' });
    }
});

// === NAVIGATION CONTROL ===

// Set navigation goal
router.post('/devices/:deviceId/goal', validateDevice, (req, res) => {
    try {
        const { x, y, orientation } = req.body;

        if (typeof x !== 'number' || typeof y !== 'number') {
            return res.status(400).json({ error: 'Invalid goal: x and y coordinates must be numbers' });
        }

        const goalOrientation = orientation || 0;
        const result = rosConnection.publishGoal(x, y, goalOrientation);

        // Broadcast navigation event
        broadcastToSubscribers('control_events', {
            type: 'goal_set',
            deviceId: req.deviceId,
            data: { x, y, orientation: goalOrientation },
            result: result,
            timestamp: new Date().toISOString()
        });

        console.log(`🎯 Goal set for device ${req.deviceId}: (${x}, ${y})`);

        res.json(result);

    } catch (error) {
        console.error('❌ Error setting goal:', error);
        res.status(500).json({
            error: 'Failed to set goal',
            details: error.message
        });
    }
});

// === MAP MANAGEMENT ===

// Get current map
router.get('/devices/:deviceId/map', validateDevice, (req, res) => {
    // Just call the /maps handler logic here
    const liveData = global.liveData[req.deviceId];
    const mapData = liveData ? liveData.map : undefined;

    res.json({
        success: true,
        deviceId: req.deviceId,
        mapData: mapData,
        liveMapData: liveData?.map,
        lastUpdate: liveData?.map?.timestamp,
        timestamp: new Date().toISOString()
    });
});

// Save current map
router.post('/devices/:deviceId/map/save', validateDevice, async (req, res) => {
    try {
        const { mapName } = req.body;
        const liveData = rosConnection.getLiveData(req.deviceId);

        if (!liveData.map) {
            return res.status(400).json({ error: 'No live map data available' });
        }

        // Save map data
        const mapData = {
            ...liveData.map,
            name: mapName || `Map_${req.deviceId}_${new Date().toISOString()}`,
            savedAt: new Date().toISOString(),
            shapes: global.deviceMaps?.[req.deviceId]?.shapes || []
        };

        global.deviceMaps[req.deviceId] = mapData;
        await storageManager.saveMap(req.deviceId);

        // Broadcast map event
        broadcastToSubscribers('map_events', {
            type: 'map_saved',
            deviceId: req.deviceId,
            mapName: mapData.name,
            timestamp: new Date().toISOString()
        });

        res.json({
            success: true,
            message: 'Map saved successfully',
            mapName: mapData.name,
            mapData: mapData
        });

        console.log(`💾 Map saved for device ${req.deviceId}: ${mapData.name}`);

    } catch (error) {
        console.error('❌ Error saving map:', error);
        res.status(500).json({
            error: 'Failed to save map',
            details: error.message
        });
    }
});

// === ADDITIONAL MAP ENDPOINTS ===

// Save map data
router.post('/devices/:deviceId/map/data', validateDevice, async (req, res) => {
    try {
        const { mapData } = req.body;
        if (!mapData) {
            return res.status(400).json({ error: 'Map data is required' });
        }

        // Add metadata
        const enhancedMapData = {
            ...mapData,
            deviceId: req.deviceId,
            savedAt: new Date().toISOString(),
            version: (mapData.version || 0) + 1
        };

        global.deviceMaps[req.deviceId] = enhancedMapData;
        await storageManager.saveMap(req.deviceId);

        // === Save PGM and YAML files ===
        const mapsDir = path.resolve(__dirname, '../../maps');
        const basePath = path.join(mapsDir, `map_${req.deviceId}_${Date.now()}`);
        await pgmConverter.saveMapPackage(enhancedMapData, basePath);

        // === Publish the edited map to ROS ===
        if (publishers.publishMap) {
            publishers.publishMap(req.deviceId, enhancedMapData);
        }

        // Broadcast map save
        broadcastToSubscribers('map_events', {
            type: 'map_saved',
            deviceId: req.deviceId,
            timestamp: new Date().toISOString()
        });

        res.json({
            success: true,
            message: 'Map data saved, published, and exported as PGM/YAML',
            mapData: enhancedMapData
        });

        console.log(`💾 Map data saved, published, and exported for device: ${req.deviceId}`);

    } catch (error) {
        console.error('❌ Error saving map data:', error);
        res.status(500).json({
            error: 'Failed to save map data',
            details: error.message
        });
    }
});

// === DEVICE MOVEMENT ENDPOINT ===

// Move device (simplified endpoint for API compatibility)
router.post('/devices/:deviceId/move', validateDevice, (req, res) => {
    try {
        const { linear, angular } = req.body;

        if (typeof linear !== 'number' || typeof angular !== 'number') {
            return res.status(400).json({ error: 'Linear and angular velocities must be numbers' });
        }

        const result = publishers.publishVelocity(linear, angular);

        // Broadcast movement command
        broadcastToSubscribers('control_events', {
            type: 'move_command',
            deviceId: req.deviceId,
            data: { linear, angular },
            result: result,
            timestamp: new Date().toISOString()
        });

        res.json(result);

    } catch (error) {
        console.error('❌ Error processing move command:', error);
        res.status(500).json({
            error: 'Failed to process move command',
            details: error.message
        });
    }
});

// Disconnect/remove a device
router.post('/devices/:deviceId/disconnect', (req, res) => {
    try {
        const deviceId = req.params.deviceId;
        if (!deviceId) {
            return res.status(400).json({ error: 'Device ID is required' });
        }

        // Remove device from global.connectedDevices
        if (global.connectedDevices) {
            const index = global.connectedDevices.findIndex(d => d.id === deviceId);
            if (index !== -1) {
                global.connectedDevices.splice(index, 1);
            }
        }

        // Optionally, clean up related data
        if (global.deviceOrders) delete global.deviceOrders[deviceId];
        if (global.deviceMaps) delete global.deviceMaps[deviceId];
        if (global.liveData) delete global.liveData[deviceId];

        res.json({
            success: true,
            message: `Device ${deviceId} disconnected and removed`
        });
        console.log(`🗑️ Device ${deviceId} disconnected and removed`);
    } catch (error) {
        console.error('❌ Error disconnecting device:', error);
        res.status(500).json({ error: 'Failed to disconnect device' });
    }
});

// === CONNECTION TEST ENDPOINT ===

// Test API connection
router.get('/test-connection', (req, res) => {
    try {
        const rosStatus = rosConnection.getROS2Status();

        res.json({
            success: true,
            message: 'API connection successful',
            server: {
                status: 'healthy',
                timestamp: new Date().toISOString(),
                ros2: rosStatus.initialized,
                connectedDevices: global.connectedDevices?.length || 0
            }
        });

    } catch (error) {
        console.error('❌ Error in connection test:', error);
        res.status(500).json({
            error: 'Connection test failed',
            details: error.message
        });
    }
});

// === THEME MANAGEMENT ===

// Update theme preference
router.post('/user/theme', (req, res) => {
    try {
        const { isDarkMode } = req.body;

        // In a real app, you'd save this to user preferences
        // For now, just acknowledge the request

        res.json({
            success: true,
            message: 'Theme preference updated',
            isDarkMode: isDarkMode,
            timestamp: new Date().toISOString()
        });

        console.log(`🎨 Theme updated: ${isDarkMode ? 'dark' : 'light'} mode`);

    } catch (error) {
        console.error('❌ Error updating theme:', error);
        res.status(500).json({
            error: 'Failed to update theme',
            details: error.message
        });
    }
});

// === EXPORT MAP AS PGM/YAML ===
const mapsDir = path.resolve(__dirname, '../../maps');

router.post('/devices/:deviceId/map/export-pgm', validateDevice, async (req, res) => {
    try {
        const liveData = rosConnection.getLiveData(req.deviceId);
        const mapData = global.deviceMaps?.[req.deviceId] || liveData.map;

        if (!mapData) {
            return res.status(400).json({ error: 'No map data available for export' });
        }

        // Generate unique filename
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
        const basePath = path.join(mapsDir, `${req.deviceId}_${timestamp}`);

        // Export PGM and YAML files
        const exportedFiles = await pgmConverter.saveMapPackage(mapData, basePath);

        res.json({
            success: true,
            message: 'Map exported successfully',
            files: {
                pgm: exportedFiles.pgm,
                yaml: exportedFiles.yaml
            },
            mapInfo: {
                width: mapData.info.width,
                height: mapData.info.height,
                resolution: mapData.info.resolution
            },
            timestamp: new Date().toISOString()
        });

        console.log(`📁 Map exported for ${req.deviceId}: ${basePath}.{pgm,yaml}`);

    } catch (error) {
        console.error('❌ Error exporting map:', error);
        res.status(500).json({
            error: 'Failed to export map',
            details: error.message
        });
    }
});

// === UPLOAD MAP TO AGV ===
router.post('/devices/:deviceId/map/upload-to-agv', validateDevice, async (req, res) => {
    try {
        const { mapName } = req.body;
        const mapData = global.deviceMaps?.[req.deviceId];

        if (!mapData) {
            return res.status(400).json({ error: 'No map data found for upload' });
        }

        // Publish map to AGV (ROS topic)
        const result = publishers.publishMap(req.deviceId, mapData);

        // Broadcast map upload event
        broadcastToSubscribers('map_events', {
            type: 'map_uploaded',
            deviceId: req.deviceId,
            mapName: mapName || mapData.name,
            timestamp: new Date().toISOString()
        });

        res.json({
            success: true,
            message: 'Map uploaded to AGV successfully',
            result: result
        });

        console.log(`⬆️ Map uploaded to AGV for device ${req.deviceId}: ${mapData.name}`);

    } catch (error) {
        console.error('❌ Error uploading map to AGV:', error);
        res.status(500).json({
            error: 'Failed to upload map to AGV',
            details: error.message
        });
    }
});
// ADD THESE NEW ROUTES after your existing routes

// ==========================================
// SCRIPT EXECUTION ROUTES
// ==========================================

// Start robot control script
router.post('/devices/:deviceId/scripts/start-robot', validateDevice, async (req, res) => {
    try {
        console.log(`🤖 Starting robot control for device: ${req.deviceId}`);
        
        // Get ROS2 script manager instance
        const scriptManager = global.ros2ScriptManager || require('../ros/utils/ros2ScriptManager');
        
        const result = await scriptManager.startRobotControl();
        
        // Broadcast script status
        broadcastToSubscribers('script_status', {
            type: 'script_status_update',
            deviceId: req.deviceId,
            script: 'robot_control',
            status: 'starting',
            message: 'Robot control script starting...',
            timestamp: new Date().toISOString()
        });
        
        res.json({
            success: true,
            message: 'Robot control script started',
            deviceId: req.deviceId,
            result: result
        });
        
    } catch (error) {
        console.error('❌ Error starting robot control:', error);
        
        // Broadcast error status
        broadcastToSubscribers('script_status', {
            type: 'script_status_update',
            deviceId: req.deviceId,
            script: 'robot_control',
            status: 'error',
            message: `Error starting robot control: ${error.message}`,
            timestamp: new Date().toISOString()
        });
        
        res.status(500).json({
            success: false,
            error: error.message,
            deviceId: req.deviceId
        });
    }
});

// Start SLAM mapping script
router.post('/devices/:deviceId/scripts/start-slam', validateDevice, async (req, res) => {
    try {
        const { mapName, useSimTime } = req.body;
        console.log(`🗺️ Starting SLAM for device: ${req.deviceId}`);
        
        const scriptManager = global.ros2ScriptManager || require('../ros/utils/ros2ScriptManager');
        
        const options = {
            mapName: mapName || `map_${req.deviceId}_${Date.now()}`,
            useSimTime: useSimTime || false
        };
        
        const result = await scriptManager.startSLAM(options);
        
        // Broadcast script status
        broadcastToSubscribers('script_status', {
            type: 'script_status_update',
            deviceId: req.deviceId,
            script: 'slam',
            status: 'starting',
            message: 'SLAM mapping script starting...',
            options: options,
            timestamp: new Date().toISOString()
        });
        
        res.json({
            success: true,
            message: 'SLAM mapping script started',
            deviceId: req.deviceId,
            options: options,
            result: result
        });
        
    } catch (error) {
        console.error('❌ Error starting SLAM:', error);
        
        broadcastToSubscribers('script_status', {
            type: 'script_status_update',
            deviceId: req.deviceId,
            script: 'slam',
            status: 'error',
            message: `Error starting SLAM: ${error.message}`,
            timestamp: new Date().toISOString()
        });
        
        res.status(500).json({
            success: false,
            error: error.message,
            deviceId: req.deviceId
        });
    }
});

// Stop specific script
router.post('/devices/:deviceId/scripts/stop/:scriptType', validateDevice, async (req, res) => {
    try {
        const { scriptType } = req.params;
        console.log(`🛑 Stopping ${scriptType} for device: ${req.deviceId}`);
        
        const scriptManager = global.ros2ScriptManager || require('../ros/utils/ros2ScriptManager');
        
        let result;
        if (scriptType === 'all') {
            result = await scriptManager.stopAll();
        } else {
            result = await scriptManager.stopProcess(scriptType);
        }
        
        // Broadcast script status
        broadcastToSubscribers('script_status', {
            type: 'script_status_update',
            deviceId: req.deviceId,
            script: scriptType,
            status: 'stopping',
            message: `Stopping ${scriptType}...`,
            timestamp: new Date().toISOString()
        });
        
        res.json({
            success: true,
            message: `${scriptType} script stop command sent`,
            deviceId: req.deviceId,
            result: result
        });
        
    } catch (error) {
        console.error(`❌ Error stopping ${req.params.scriptType}:`, error);
        res.status(500).json({
            success: false,
            error: error.message,
            deviceId: req.deviceId
        });
    }
});

// Get script status
router.get('/devices/:deviceId/scripts/status', validateDevice, async (req, res) => {
    try {
        const scriptManager = global.ros2ScriptManager || require('../ros/utils/ros2ScriptManager');
        const status = scriptManager.getDetailedStatus();
        
        res.json({
            success: true,
            deviceId: req.deviceId,
            status: status,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error getting script status:', error);
        res.status(500).json({
            success: false,
            error: error.message,
            deviceId: req.deviceId
        });
    }
});
module.exports = router;