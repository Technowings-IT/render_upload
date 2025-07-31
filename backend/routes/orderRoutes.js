// routes/orderRoutes.js - Complete Order Management System
const express = require('express');
const path = require('path');
const fs = require('fs').promises;
const router = express.Router();

// Storage paths
const ORDERS_FILE = path.join(__dirname, '../storage/orders.json');
const DEVICES_FILE = path.join(__dirname, '../storage/devices.json');

// ==========================================
// ORDER CRUD OPERATIONS
// ==========================================

/**
 * Get all orders for a device
 * GET /api/orders/:deviceId
 */
router.get('/:deviceId', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { status, limit = 50, offset = 0 } = req.query;
        
        console.log(`📋 Getting orders for device: ${deviceId}`);
        
        const orders = await loadOrders();
        let deviceOrders = orders.filter(order => order.deviceId === deviceId);
        
        // Filter by status if provided
        if (status) {
            deviceOrders = deviceOrders.filter(order => order.status === status);
        }
        
        // Sort by creation time (newest first)
        deviceOrders.sort((a, b) => new Date(b.createdAt) - new Date(a.createdAt));
        
        // Apply pagination
        const total = deviceOrders.length;
        const paginatedOrders = deviceOrders.slice(
            parseInt(offset), 
            parseInt(offset) + parseInt(limit)
        );
        
        res.json({
            success: true,
            orders: paginatedOrders,
            pagination: {
                total,
                limit: parseInt(limit),
                offset: parseInt(offset),
                hasMore: parseInt(offset) + parseInt(limit) < total
            },
            deviceId
        });
        
    } catch (error) {
        console.error('❌ Error getting orders:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Get all orders across all devices (for dashboard)
 * GET /api/orders
 */
router.get('/', async (req, res) => {
    try {
        const { status, deviceId, limit = 100, offset = 0 } = req.query;
        
        console.log('📋 Getting all orders');
        
        let orders = await loadOrders();
        
        // Filter by device if provided
        if (deviceId) {
            orders = orders.filter(order => order.deviceId === deviceId);
        }
        
        // Filter by status if provided
        if (status) {
            orders = orders.filter(order => order.status === status);
        }
        
        // Sort by creation time (newest first)
        orders.sort((a, b) => new Date(b.createdAt) - new Date(a.createdAt));
        
        // Apply pagination
        const total = orders.length;
        const paginatedOrders = orders.slice(
            parseInt(offset), 
            parseInt(offset) + parseInt(limit)
        );
        
        // Add device names
        const devices = await loadDevices();
        const ordersWithDeviceNames = paginatedOrders.map(order => ({
            ...order,
            deviceName: devices.find(d => d.id === order.deviceId)?.name || order.deviceId
        }));
        
        res.json({
            success: true,
            orders: ordersWithDeviceNames,
            pagination: {
                total,
                limit: parseInt(limit),
                offset: parseInt(offset),
                hasMore: parseInt(offset) + parseInt(limit) < total
            }
        });
        
    } catch (error) {
        console.error('❌ Error getting all orders:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Create a new order
 * POST /api/orders/:deviceId
 */
router.post('/:deviceId', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { name, waypoints, priority = 0, description = '' } = req.body;
        
        console.log(`📝 Creating order for device: ${deviceId}`);
        
        // Validate required fields
        if (!name || !waypoints || !Array.isArray(waypoints) || waypoints.length === 0) {
            return res.status(400).json({
                success: false,
                error: 'Name and waypoints are required'
            });
        }
        
        // Validate waypoints
        for (let i = 0; i < waypoints.length; i++) {
            const waypoint = waypoints[i];
            if (!waypoint.name || !waypoint.type || !waypoint.position) {
                return res.status(400).json({
                    success: false,
                    error: `Invalid waypoint at index ${i}: name, type, and position are required`
                });
            }
        }
        
        // Check if device exists
        const devices = await loadDevices();
        const device = devices.find(d => d.id === deviceId);
        if (!device) {
            return res.status(404).json({
                success: false,
                error: `Device not found: ${deviceId}`
            });
        }
        
        // Create new order
        const order = {
            id: generateOrderId(deviceId),
            deviceId,
            name: name.trim(),
            description: description.trim(),
            waypoints: waypoints.map((wp, index) => ({
                id: `${deviceId}_wp_${Date.now()}_${index}`,
                stepNumber: index + 1,
                name: wp.name.trim(),
                type: wp.type, // pickup, drop, charging, waypoint
                position: {
                    x: parseFloat(wp.position.x),
                    y: parseFloat(wp.position.y),
                    z: parseFloat(wp.position.z || 0)
                },
                orientation: wp.orientation || 0,
                metadata: wp.metadata || {},
                completed: false,
                completedAt: null
            })),
            status: 'pending', // pending, active, paused, completed, failed, cancelled
            priority: parseInt(priority),
            currentWaypoint: 0,
            progress: {
                totalWaypoints: waypoints.length,
                completedWaypoints: 0,
                percentage: 0
            },
            createdAt: new Date().toISOString(),
            updatedAt: new Date().toISOString(),
            startedAt: null,
            completedAt: null,
            estimatedDuration: null,
            actualDuration: null,
            metadata: {
                createdBy: 'dashboard',
                version: '1.0'
            }
        };
        
        // Save order
        const orders = await loadOrders();
        orders.push(order);
        await saveOrders(orders);
        
        // Update global order queue
        updateGlobalOrderQueue(deviceId, order);
        
        res.json({
            success: true,
            message: 'Order created successfully',
            order,
            deviceId
        });
        
        console.log(`✅ Order created: ${order.id} for device: ${deviceId}`);
        
    } catch (error) {
        console.error('❌ Error creating order:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Get specific order
 * GET /api/orders/:deviceId/:orderId
 */
router.get('/:deviceId/:orderId', async (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        
        const orders = await loadOrders();
        const order = orders.find(o => o.id === orderId && o.deviceId === deviceId);
        
        if (!order) {
            return res.status(404).json({
                success: false,
                error: 'Order not found'
            });
        }
        
        // Add device name
        const devices = await loadDevices();
        const device = devices.find(d => d.id === deviceId);
        
        res.json({
            success: true,
            order: {
                ...order,
                deviceName: device?.name || deviceId
            }
        });
        
    } catch (error) {
        console.error('❌ Error getting order:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Update order status
 * PUT /api/orders/:deviceId/:orderId/status
 */
router.put('/:deviceId/:orderId/status', async (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        const { status, currentWaypoint, reason } = req.body;
        
        console.log(`🔄 Updating order status: ${orderId} to ${status}`);
        
        const validStatuses = ['pending', 'active', 'paused', 'completed', 'failed', 'cancelled'];
        if (!validStatuses.includes(status)) {
            return res.status(400).json({
                success: false,
                error: `Invalid status. Must be one of: ${validStatuses.join(', ')}`
            });
        }
        
        const orders = await loadOrders();
        const orderIndex = orders.findIndex(o => o.id === orderId && o.deviceId === deviceId);
        
        if (orderIndex === -1) {
            return res.status(404).json({
                success: false,
                error: 'Order not found'
            });
        }
        
        const order = orders[orderIndex];
        const previousStatus = order.status;
        
        // Update order
        order.status = status;
        order.updatedAt = new Date().toISOString();
        
        if (currentWaypoint !== undefined) {
            order.currentWaypoint = parseInt(currentWaypoint);
            order.progress.completedWaypoints = parseInt(currentWaypoint);
            order.progress.percentage = Math.round((parseInt(currentWaypoint) / order.waypoints.length) * 100);
            
            // Mark waypoints as completed
            order.waypoints.forEach((wp, index) => {
                if (index < parseInt(currentWaypoint)) {
                    wp.completed = true;
                    if (!wp.completedAt) {
                        wp.completedAt = new Date().toISOString();
                    }
                }
            });
        }
        
        // Handle status-specific logic
        switch (status) {
            case 'active':
                if (previousStatus === 'pending') {
                    order.startedAt = new Date().toISOString();
                }
                break;
                
            case 'completed':
                order.completedAt = new Date().toISOString();
                order.progress.completedWaypoints = order.waypoints.length;
                order.progress.percentage = 100;
                
                // Mark all waypoints as completed
                order.waypoints.forEach(wp => {
                    wp.completed = true;
                    if (!wp.completedAt) {
                        wp.completedAt = new Date().toISOString();
                    }
                });
                
                // Calculate actual duration
                if (order.startedAt) {
                    const startTime = new Date(order.startedAt);
                    const endTime = new Date(order.completedAt);
                    order.actualDuration = Math.round((endTime - startTime) / 1000); // seconds
                }
                break;
                
            case 'failed':
            case 'cancelled':
                if (reason) {
                    order.metadata.failureReason = reason;
                }
                break;
        }
        
        orders[orderIndex] = order;
        await saveOrders(orders);
        
        // Update global order queue
        updateGlobalOrderQueue(deviceId, order);
        
        res.json({
            success: true,
            message: `Order status updated from ${previousStatus} to ${status}`,
            order,
            previousStatus
        });
        
        console.log(`✅ Order ${orderId} status updated: ${previousStatus} → ${status}`);
        
    } catch (error) {
        console.error('❌ Error updating order status:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Execute order (start it)
 * POST /api/orders/:deviceId/:orderId/execute
 */
router.post('/:deviceId/:orderId/execute', async (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        
        console.log(`▶️ Executing order: ${orderId}`);
        
        const orders = await loadOrders();
        const orderIndex = orders.findIndex(o => o.id === orderId && o.deviceId === deviceId);
        
        if (orderIndex === -1) {
            return res.status(404).json({
                success: false,
                error: 'Order not found'
            });
        }
        
        const order = orders[orderIndex];
        
        if (order.status !== 'pending' && order.status !== 'paused') {
            return res.status(400).json({
                success: false,
                error: `Cannot execute order with status: ${order.status}`
            });
        }
        
        // Update order status
        order.status = 'active';
        order.updatedAt = new Date().toISOString();
        
        if (!order.startedAt) {
            order.startedAt = new Date().toISOString();
        }
        
        orders[orderIndex] = order;
        await saveOrders(orders);
        
        // Update global order queue
        updateGlobalOrderQueue(deviceId, order);
        
        // TODO: Send order to ROS navigation stack
        // This would typically involve:
        // 1. Publishing navigation goals to ROS
        // 2. Setting up goal monitoring
        // 3. Handling feedback and status updates
        
        res.json({
            success: true,
            message: 'Order execution started',
            order
        });
        
        console.log(`✅ Order execution started: ${orderId}`);
        
    } catch (error) {
        console.error('❌ Error executing order:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Pause order
 * POST /api/orders/:deviceId/:orderId/pause
 */
router.post('/:deviceId/:orderId/pause', async (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        
        console.log(`⏸️ Pausing order: ${orderId}`);
        
        const orders = await loadOrders();
        const orderIndex = orders.findIndex(o => o.id === orderId && o.deviceId === deviceId);
        
        if (orderIndex === -1) {
            return res.status(404).json({
                success: false,
                error: 'Order not found'
            });
        }
        
        const order = orders[orderIndex];
        
        if (order.status !== 'active') {
            return res.status(400).json({
                success: false,
                error: `Cannot pause order with status: ${order.status}`
            });
        }
        
        order.status = 'paused';
        order.updatedAt = new Date().toISOString();
        
        orders[orderIndex] = order;
        await saveOrders(orders);
        
        // Update global order queue
        updateGlobalOrderQueue(deviceId, order);
        
        // TODO: Send pause command to ROS navigation stack
        
        res.json({
            success: true,
            message: 'Order paused',
            order
        });
        
        console.log(`✅ Order paused: ${orderId}`);
        
    } catch (error) {
        console.error('❌ Error pausing order:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Delete order
 * DELETE /api/orders/:deviceId/:orderId
 */
router.delete('/:deviceId/:orderId', async (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        
        console.log(`🗑️ Deleting order: ${orderId}`);
        
        const orders = await loadOrders();
        const orderIndex = orders.findIndex(o => o.id === orderId && o.deviceId === deviceId);
        
        if (orderIndex === -1) {
            return res.status(404).json({
                success: false,
                error: 'Order not found'
            });
        }
        
        const order = orders[orderIndex];
        
        // Don't allow deletion of active orders
        if (order.status === 'active') {
            return res.status(400).json({
                success: false,
                error: 'Cannot delete active order. Pause or cancel it first.'
            });
        }
        
        orders.splice(orderIndex, 1);
        await saveOrders(orders);
        
        // Update global order queue
        removeFromGlobalOrderQueue(deviceId, orderId);
        
        res.json({
            success: true,
            message: 'Order deleted successfully',
            deletedOrder: order
        });
        
        console.log(`✅ Order deleted: ${orderId}`);
        
    } catch (error) {
        console.error('❌ Error deleting order:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Get order statistics
 * GET /api/orders/:deviceId/stats
 */
router.get('/:deviceId/stats', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { timeRange = '7d' } = req.query;
        
        const orders = await loadOrders();
        const deviceOrders = orders.filter(o => o.deviceId === deviceId);
        
        // Calculate time range
        const now = new Date();
        let startDate;
        switch (timeRange) {
            case '1d': startDate = new Date(now - 24 * 60 * 60 * 1000); break;
            case '7d': startDate = new Date(now - 7 * 24 * 60 * 60 * 1000); break;
            case '30d': startDate = new Date(now - 30 * 24 * 60 * 60 * 1000); break;
            default: startDate = new Date(now - 7 * 24 * 60 * 60 * 1000);
        }
        
        const recentOrders = deviceOrders.filter(o => new Date(o.createdAt) >= startDate);
        
        const stats = {
            total: deviceOrders.length,
            recent: recentOrders.length,
            byStatus: {
                pending: deviceOrders.filter(o => o.status === 'pending').length,
                active: deviceOrders.filter(o => o.status === 'active').length,
                paused: deviceOrders.filter(o => o.status === 'paused').length,
                completed: deviceOrders.filter(o => o.status === 'completed').length,
                failed: deviceOrders.filter(o => o.status === 'failed').length,
                cancelled: deviceOrders.filter(o => o.status === 'cancelled').length
            },
            averageDuration: calculateAverageDuration(deviceOrders.filter(o => o.actualDuration)),
            totalWaypoints: deviceOrders.reduce((sum, o) => sum + o.waypoints.length, 0),
            successRate: calculateSuccessRate(deviceOrders),
            timeRange
        };
        
        res.json({
            success: true,
            stats,
            deviceId
        });
        
    } catch (error) {
        console.error('❌ Error getting order stats:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Get available map stations for order creation
 * GET /api/orders/:deviceId/stations
 */
router.get('/:deviceId/stations', async (req, res) => {
    try {
        const { deviceId } = req.params;
        
        // Check if device has map data
        const deviceMaps = global.deviceMaps || {};
        const mapData = deviceMaps[deviceId];
        
        if (!mapData || !mapData.shapes) {
            return res.json({
                success: true,
                stations: [],
                message: 'No map data available for this device'
            });
        }
        
        // Extract stations from map shapes
        const stations = mapData.shapes.map(shape => ({
            id: shape.id,
            name: shape.name,
            type: shape.type,
            position: shape.center || (shape.points && shape.points[0]) || { x: 0, y: 0, z: 0 },
            metadata: {
                color: shape.color,
                createdAt: shape.createdAt,
                sides: shape.sides
            }
        }));
        
        // Group by type
        const stationsByType = {
            pickup: stations.filter(s => s.type === 'pickup'),
            drop: stations.filter(s => s.type === 'drop'),
            charging: stations.filter(s => s.type === 'charging'),
            waypoint: stations.filter(s => s.type === 'waypoint'),
            other: stations.filter(s => !['pickup', 'drop', 'charging', 'waypoint'].includes(s.type))
        };
        
        res.json({
            success: true,
            stations,
            stationsByType,
            deviceId,
            totalStations: stations.length
        });
        
    } catch (error) {
        console.error('❌ Error getting stations:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

// ==========================================
// HELPER FUNCTIONS
// ==========================================

async function loadOrders() {
    try {
        await ensureOrdersFileExists();
        const data = await fs.readFile(ORDERS_FILE, 'utf8');
        if (!data || data.trim() === '') return [];
        const parsed = JSON.parse(data);
        return Array.isArray(parsed) ? parsed : [];
    } catch (error) {
        console.error('❌ Error loading orders:', error);
        return [];
    }
}

async function saveOrders(orders) {
    try {
        await fs.writeFile(ORDERS_FILE, JSON.stringify(orders, null, 2));
        console.log(`💾 Saved ${orders.length} orders to storage`);
    } catch (error) {
        console.error('❌ Error saving orders:', error);
        throw error;
    }
}

async function loadDevices() {
    try {
        const data = await fs.readFile(DEVICES_FILE, 'utf8');
        return JSON.parse(data);
    } catch (error) {
        console.warn('⚠️ No devices file found, returning empty array');
        return [];
    }
}

async function ensureOrdersFileExists() {
    try {
        await fs.access(ORDERS_FILE);
    } catch (error) {
        console.log('📁 Creating orders.json file');
        await fs.writeFile(ORDERS_FILE, JSON.stringify([], null, 2));
    }
}

function generateOrderId(deviceId) {
    const timestamp = Date.now();
    const random = Math.random().toString(36).substring(2, 8);
    return `order_${deviceId}_${timestamp}_${random}`;
}

function updateGlobalOrderQueue(deviceId, order) {
    try {
        if (!global.deviceOrders) {
            global.deviceOrders = {};
        }
        
        if (!global.deviceOrders[deviceId]) {
            global.deviceOrders[deviceId] = [];
        }
        
        // Update or add order
        const existingIndex = global.deviceOrders[deviceId].findIndex(o => o.id === order.id);
        if (existingIndex !== -1) {
            global.deviceOrders[deviceId][existingIndex] = order;
        } else {
            global.deviceOrders[deviceId].push(order);
        }
        
        console.log(`🔄 Global order queue updated for device: ${deviceId}`);
    } catch (error) {
        console.error('❌ Error updating global order queue:', error);
    }
}

function removeFromGlobalOrderQueue(deviceId, orderId) {
    try {
        if (global.deviceOrders && global.deviceOrders[deviceId]) {
            global.deviceOrders[deviceId] = global.deviceOrders[deviceId].filter(o => o.id !== orderId);
            console.log(`🗑️ Order removed from global queue: ${orderId}`);
        }
    } catch (error) {
        console.error('❌ Error removing from global order queue:', error);
    }
}

function calculateAverageDuration(completedOrders) {
    if (completedOrders.length === 0) return 0;
    
    const totalDuration = completedOrders.reduce((sum, order) => sum + (order.actualDuration || 0), 0);
    return Math.round(totalDuration / completedOrders.length);
}

function calculateSuccessRate(orders) {
    if (orders.length === 0) return 0;
    
    const completedOrders = orders.filter(o => o.status === 'completed').length;
    return Math.round((completedOrders / orders.length) * 100);
}

module.exports = router;