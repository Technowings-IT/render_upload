// routes/controlRoutes.js - Enhanced control routes for AGV fleet management
const express = require('express');
const router = express.Router();

// Import ROS connection and WebSocket broadcasting
const rosConnection = require('../ros/utils/ros_connection');
const { broadcastToSubscribers } = require('../websocket/clientConnection');
const storageManager = require('../ros/utils/storageManager');
const publishers = require('../ros/utils/publishers');
const pgmConverter = require('../ros/utils/pgmConverter');
const path = require('path');

// Middleware for device validation
function validateDevice(req, res, next) {
    const { deviceId } = req.params;
    if (!deviceId) {
        return res.status(400).json({ error: 'Device ID is required' });
    }
    req.deviceId = deviceId;
    next();
}

// === DEVICE MANAGEMENT ===

// Connect a new AGV device
router.post('/devices/connect', async (req, res) => {
    try {
        const { deviceId, name, type, ipAddress, capabilities } = req.body;
        
        if (!deviceId) {
            return res.status(400).json({ error: 'Device ID is required' });
        }
        
        const deviceInfo = {
            id: deviceId,
            name: name || `AGV ${deviceId}`,
            type: type || 'differential_drive',
            ipAddress: ipAddress,
            capabilities: capabilities || ['mapping', 'navigation', 'remote_control'],
            connectedAt: new Date().toISOString()
        };
        
        // Add device to ROS connection manager
        const connectedDeviceId = rosConnection.addConnectedDevice(deviceInfo);
        
        // Save device permanently
        await storageManager.saveDevice(connectedDeviceId);
        
        // Broadcast device connection
        broadcastToSubscribers('device_events', {
            type: 'device_connected',
            deviceId: connectedDeviceId,
            device: deviceInfo,
            timestamp: new Date().toISOString()
        });
        
        res.json({
            success: true,
            message: 'Device connected successfully',
            deviceId: connectedDeviceId,
            device: deviceInfo
        });
        
        console.log(`✅ Device ${connectedDeviceId} connected via API`);
        
    } catch (error) {
        console.error('❌ Error connecting device:', error);
        res.status(500).json({ 
            error: 'Failed to connect device',
            details: error.message 
        });
    }
});

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

// Get all connected devices
router.get('/devices', (req, res) => {
    try {
        const devices = global.connectedDevices || [];
        const liveData = global.liveData || {};
        
        const devicesWithStatus = devices.map(device => ({
            ...device,
            liveData: liveData[device.id] || {},
            isOnline: !!liveData[device.id]?.lastUpdate
        }));
        
        res.json({
            success: true,
            devices: devicesWithStatus,
            total: devices.length
        });
        
    } catch (error) {
        console.error('❌ Error getting devices:', error);
        res.status(500).json({ error: 'Failed to get devices' });
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
    try {
        const mapData = global.deviceMaps?.[req.deviceId];
        const liveData = rosConnection.getLiveData(req.deviceId);
        
        res.json({
            success: true,
            deviceId: req.deviceId,
            mapData: mapData,
            liveMapData: liveData.map,
            lastUpdate: liveData.map?.timestamp,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error getting map:', error);
        res.status(500).json({ error: 'Failed to get map' });
    }
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

// === ORDER MANAGEMENT ENDPOINTS ===

// Get orders for a device
router.get('/devices/:deviceId/orders', validateDevice, (req, res) => {
    try {
        const orders = global.deviceOrders[req.deviceId] || [];
        
        res.json({
            success: true,
            deviceId: req.deviceId,
            orders: orders,
            total: orders.length,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error getting orders:', error);
        res.status(500).json({ error: 'Failed to get orders' });
    }
});

// Create new order
router.post('/devices/:deviceId/orders', validateDevice, async (req, res) => {
    try {
        const { name, waypoints, priority = 0 } = req.body;
        
        if (!name || !waypoints || !Array.isArray(waypoints)) {
            return res.status(400).json({ error: 'Order name and waypoints are required' });
        }
        
        const newOrder = {
            id: `order_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
            name: name,
            deviceId: req.deviceId,
            waypoints: waypoints,
            priority: priority,
            status: 'pending',
            createdAt: new Date().toISOString(),
            updatedAt: new Date().toISOString()
        };
        
        // Initialize orders array if needed
        if (!global.deviceOrders[req.deviceId]) {
            global.deviceOrders[req.deviceId] = [];
        }
        
        global.deviceOrders[req.deviceId].push(newOrder);
        
        // Save to storage
        await storageManager.saveOrder(req.deviceId, newOrder.id);
        
        // Broadcast order creation
        broadcastToSubscribers('order_events', {
            type: 'order_created',
            deviceId: req.deviceId,
            order: newOrder,
            timestamp: new Date().toISOString()
        });
        
        res.json({
            success: true,
            message: 'Order created successfully',
            order: newOrder
        });
        
        console.log(`📋 Order created: ${newOrder.name} for device ${req.deviceId}`);
        
    } catch (error) {
        console.error('❌ Error creating order:', error);
        res.status(500).json({ 
            error: 'Failed to create order',
            details: error.message 
        });
    }
});

// Update order status
router.put('/orders/:orderId/status', async (req, res) => {
    try {
        const { orderId } = req.params;
        const { status } = req.body;
        
        if (!status) {
            return res.status(400).json({ error: 'Status is required' });
        }
        
        let orderFound = false;
        let deviceId = null;
        
        // Find the order across all devices
        for (const [devId, orders] of Object.entries(global.deviceOrders)) {
            const orderIndex = orders.findIndex(o => o.id === orderId);
            if (orderIndex !== -1) {
                orders[orderIndex].status = status;
                orders[orderIndex].updatedAt = new Date().toISOString();
                deviceId = devId;
                orderFound = true;
                break;
            }
        }
        
        if (!orderFound) {
            return res.status(404).json({ error: 'Order not found' });
        }
        
        // Save to storage
        await storageManager.saveOrder(deviceId, orderId);
        
        // Broadcast status change
        broadcastToSubscribers('order_events', {
            type: 'order_status_changed',
            deviceId: deviceId,
            orderId: orderId,
            status: status,
            timestamp: new Date().toISOString()
        });
        
        res.json({
            success: true,
            message: 'Order status updated successfully',
            orderId: orderId,
            status: status
        });
        
        console.log(`📋 Order ${orderId} status updated to: ${status}`);
        
    } catch (error) {
        console.error('❌ Error updating order status:', error);
        res.status(500).json({ 
            error: 'Failed to update order status',
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

module.exports = router;