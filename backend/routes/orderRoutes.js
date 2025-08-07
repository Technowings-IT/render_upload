// routes/enhanced_order_routes.js - Enhanced Order Management with ROS Integration
const express = require('express');
const path = require('path');
const fs = require('fs').promises;
const router = express.Router();

// Import ROS publishers with error handling
let rosPublishers = null;
try {
    rosPublishers = require('../ros/utils/publishers');
} catch (error) {
    console.warn('⚠️ ROS publishers not available:', error.message);
}

// Storage paths
const ORDERS_FILE = path.join(__dirname, '../storage/orders.json');
const DEVICES_FILE = path.join(__dirname, '../storage/devices.json');

// ==========================================
// DEVICE-SPECIFIC ORDER ENDPOINTS
// ==========================================

/**
 * Get order statistics for analytics
 * GET /api/orders/stats
 */
router.get('/stats', async (req, res) => {
    try {
        const { deviceId, timeRange = '7d' } = req.query;
        
        console.log(`� Getting order statistics for timeRange: ${timeRange}`);
        
        const orders = await loadOrders();
        
        // Filter by device if specified
        let filteredOrders = deviceId 
            ? orders.filter(order => order.deviceId === deviceId)
            : orders;
        
        // Filter by time range
        const now = new Date();
        const timeRangeMs = parseTimeRange(timeRange);
        const startDate = new Date(now.getTime() - timeRangeMs);
        
        filteredOrders = filteredOrders.filter(order => 
            new Date(order.createdAt) >= startDate
        );
        
        // Calculate statistics
        const stats = {
            total: filteredOrders.length,
            completed: filteredOrders.filter(o => o.status === 'completed').length,
            active: filteredOrders.filter(o => o.status === 'active').length,
            pending: filteredOrders.filter(o => o.status === 'pending').length,
            cancelled: filteredOrders.filter(o => o.status === 'cancelled').length,
            failed: filteredOrders.filter(o => o.status === 'failed').length,
        };
        
        stats.completionRate = stats.total > 0 
            ? Math.round((stats.completed / stats.total) * 100) 
            : 0;
        
        // Calculate daily breakdown for charts
        const dailyStats = calculateDailyStats(filteredOrders, timeRangeMs);
        
        res.json({
            success: true,
            stats,
            dailyStats,
            timeRange,
            deviceId: deviceId || 'all',
            calculatedAt: new Date().toISOString()
        });
        
        console.log(`✅ Order statistics calculated: ${stats.total} orders, ${stats.completionRate}% completion rate`);
        
    } catch (error) {
        console.error('❌ Error getting order statistics:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Get orders for a specific device
 * GET /api/orders/:deviceId
 */
router.get('/:deviceId', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { status, limit = 50, offset = 0 } = req.query;
        
        console.log(`� Getting orders for device: ${deviceId}`);
        
        const orders = await loadOrders();
        let deviceOrders = orders.filter(order => order.deviceId === deviceId);
        
        // Filter by status if provided
        if (status) {
            deviceOrders = deviceOrders.filter(order => order.status === status);
        }
        
        // Sort by creation date (newest first)
        deviceOrders.sort((a, b) => new Date(b.createdAt) - new Date(a.createdAt));
        
        // Apply pagination
        const startIndex = parseInt(offset);
        const endIndex = startIndex + parseInt(limit);
        const paginatedOrders = deviceOrders.slice(startIndex, endIndex);
        
        res.json({
            success: true,
            orders: paginatedOrders,
            deviceId,
            pagination: {
                total: deviceOrders.length,
                limit: parseInt(limit),
                offset: parseInt(offset),
                hasMore: endIndex < deviceOrders.length
            },
            filters: {
                status: status || 'all'
            }
        });
        
        console.log(`✅ Retrieved ${paginatedOrders.length}/${deviceOrders.length} orders for ${deviceId}`);
        
    } catch (error) {
        console.error('❌ Error getting orders for device:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

// Helper function to parse time range
function parseTimeRange(timeRange) {
    const ranges = {
        '1h': 60 * 60 * 1000,
        '6h': 6 * 60 * 60 * 1000,
        '24h': 24 * 60 * 60 * 1000,
        '7d': 7 * 24 * 60 * 60 * 1000,
        '30d': 30 * 24 * 60 * 60 * 1000,
        '90d': 90 * 24 * 60 * 60 * 1000
    };
    
    return ranges[timeRange] || ranges['7d'];
}

// Helper function to calculate daily statistics
function calculateDailyStats(orders, timeRangeMs) {
    const days = Math.ceil(timeRangeMs / (24 * 60 * 60 * 1000));
    const dailyStats = [];
    
    for (let i = days - 1; i >= 0; i--) {
        const date = new Date();
        date.setDate(date.getDate() - i);
        date.setHours(0, 0, 0, 0);
        
        const nextDate = new Date(date);
        nextDate.setDate(nextDate.getDate() + 1);
        
        const dayOrders = orders.filter(order => {
            const orderDate = new Date(order.createdAt);
            return orderDate >= date && orderDate < nextDate;
        });
        
        dailyStats.push({
            date: date.toISOString().split('T')[0],
            total: dayOrders.length,
            completed: dayOrders.filter(o => o.status === 'completed').length,
            failed: dayOrders.filter(o => o.status === 'failed').length,
            cancelled: dayOrders.filter(o => o.status === 'cancelled').length
        });
    }
    
    return dailyStats;
}

// ==========================================
// ENHANCED ORDER EXECUTION WITH ROS
// ==========================================

/**
 * Execute order with real-time ROS publishing
 * POST /api/orders/:deviceId/:orderId/execute
 */
router.post('/:deviceId/:orderId/execute', async (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        const { realTimeExecution = true, waypointDelay = 3000 } = req.body;
        
        console.log(`🚀 Executing order: ${orderId} for device: ${deviceId}`);
        
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
        
        // Update order status to active
        order.status = 'active';
        order.updatedAt = new Date().toISOString();
        if (!order.startedAt) {
            order.startedAt = new Date().toISOString();
        }
        
        orders[orderIndex] = order;
        await saveOrders(orders);
        
        // If real-time execution is requested, execute waypoints sequentially
        if (realTimeExecution && order.waypoints && order.waypoints.length > 0) {
            // Execute in background to return response immediately
            executeOrderWaypoints(deviceId, order, waypointDelay).catch(error => {
                console.error(`❌ Background order execution failed: ${error}`);
                updateOrderStatusInBackground(deviceId, orderId, 'failed', error.message);
            });
            
            res.json({
                success: true,
                message: 'Order execution started with real-time ROS publishing',
                order,
                execution: {
                    mode: 'real-time',
                    totalWaypoints: order.waypoints.length,
                    waypointDelay: waypointDelay,
                    estimatedDuration: `${(order.waypoints.length * waypointDelay) / 1000}s`,
                    rosAvailable: !!rosPublishers
                }
            });
        } else {
            // Basic execution without real-time waypoint publishing
            res.json({
                success: true,
                message: 'Order execution started',
                order,
                execution: {
                    mode: 'basic',
                    note: 'Use real-time execution for automatic waypoint publishing',
                    rosAvailable: !!rosPublishers
                }
            });
        }
        
        console.log(`✅ Order execution initiated: ${orderId}`);
        
    } catch (error) {
        console.error('❌ Error executing order:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Execute order waypoints sequentially with ROS publishing
 */
async function executeOrderWaypoints(deviceId, order, waypointDelay = 3000) {
    console.log(`🎯 Starting waypoint execution for order: ${order.id}`);
    console.log(`📍 Total waypoints: ${order.waypoints.length}`);
    
    try {
        for (let i = 0; i < order.waypoints.length; i++) {
            const waypoint = order.waypoints[i];
            const position = waypoint.position;
            
            console.log(`🚶 Executing waypoint ${i + 1}/${order.waypoints.length}: ${waypoint.name}`);
            console.log(`📍 Position: (${position.x}, ${position.y}) @ ${waypoint.orientation || 0}rad`);
            
            // Publish navigation goal to ROS target_pose topic (if ROS is available)
            if (rosPublishers && rosPublishers.publishGoalWithId) {
                const goalResult = await rosPublishers.publishGoalWithId(
                    position.x,
                    position.y,
                    waypoint.orientation || 0,
                    `${order.id}_wp_${i}_${Date.now()}`
                );
                
                if (!goalResult.success) {
                    throw new Error(`Failed to publish waypoint ${i + 1}: ${goalResult.error}`);
                }
                
                console.log(`✅ Published waypoint ${i + 1} goal ID: ${goalResult.goalId}`);
            } else {
                console.log(`⚠️ ROS not available, simulating waypoint ${i + 1} execution`);
                // Simulate execution when ROS is not available
            }
            
            // Update order progress
            await updateOrderProgress(deviceId, order.id, i + 1);
            
            // Wait before next waypoint (except for last one)
            if (i < order.waypoints.length - 1) {
                console.log(`⏳ Waiting ${waypointDelay / 1000}s before next waypoint...`);
                await delay(waypointDelay);
            }
        }
        
        // Mark order as completed
        await updateOrderStatusInBackground(deviceId, order.id, 'completed');
        console.log(`🎉 Order execution completed successfully: ${order.id}`);
        
        // Broadcast completion via WebSocket if available
        broadcastOrderUpdate(deviceId, order.id, 'completed');
        
    } catch (error) {
        console.error(`❌ Order execution failed: ${error}`);
        await updateOrderStatusInBackground(deviceId, order.id, 'failed', error.message);
        broadcastOrderUpdate(deviceId, order.id, 'failed', error.message);
        throw error;
    }
}

/**
 * Publish single waypoint goal
 * POST /api/orders/:deviceId/:orderId/waypoint/:waypointIndex
 */
router.post('/:deviceId/:orderId/waypoint/:waypointIndex', async (req, res) => {
    try {
        const { deviceId, orderId, waypointIndex } = req.params;
        
        console.log(`🎯 Publishing waypoint ${waypointIndex} for order: ${orderId}`);
        
        const orders = await loadOrders();
        const order = orders.find(o => o.id === orderId && o.deviceId === deviceId);
        
        if (!order) {
            return res.status(404).json({
                success: false,
                error: 'Order not found'
            });
        }
        
        const waypointIdx = parseInt(waypointIndex);
        if (waypointIdx < 0 || waypointIdx >= order.waypoints.length) {
            return res.status(400).json({
                success: false,
                error: 'Invalid waypoint index'
            });
        }
        
        const waypoint = order.waypoints[waypointIdx];
        const position = waypoint.position;
        
        // Publish navigation goal
        if (rosPublishers && rosPublishers.publishGoalWithId) {
            const goalResult = await rosPublishers.publishGoalWithId(
                position.x,
                position.y,
                waypoint.orientation || 0,
                `${orderId}_wp_${waypointIdx}_${Date.now()}`
            );
            
            if (goalResult.success) {
                // Update waypoint completion status
                waypoint.completed = false; // Will be completed when robot reaches
                waypoint.publishedAt = new Date().toISOString();
                
                await saveOrders(orders);
                
                res.json({
                    success: true,
                    message: `Waypoint ${waypointIdx + 1} published successfully`,
                    waypoint: {
                        index: waypointIdx,
                        name: waypoint.name,
                        position,
                        goalId: goalResult.goalId
                    },
                    goalResult
                });
            } else {
                res.status(500).json({
                    success: false,
                    error: `Failed to publish waypoint: ${goalResult.error}`
                });
            }
        } else {
            // ROS not available - simulate waypoint publishing
            waypoint.completed = false;
            waypoint.publishedAt = new Date().toISOString();
            waypoint.simulated = true;
            
            await saveOrders(orders);
            
            res.json({
                success: true,
                message: `Waypoint ${waypointIdx + 1} queued (ROS simulation mode)`,
                waypoint: {
                    index: waypointIdx,
                    name: waypoint.name,
                    position,
                    goalId: `sim_${Date.now()}`,
                    simulated: true
                },
                goalResult: {
                    success: true,
                    goalId: `sim_${Date.now()}`,
                    simulated: true
                }
            });
        }
        
    } catch (error) {
        console.error('❌ Error publishing waypoint:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Cancel order execution and navigation
 * POST /api/orders/:deviceId/:orderId/cancel
 */
router.post('/:deviceId/:orderId/cancel', async (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        const { reason } = req.body;
        
        console.log(`🛑 Cancelling order: ${orderId}`);
        
        // Cancel current navigation goal in ROS (if available)
        let cancelResult = { success: true, simulated: !rosPublishers };
        if (rosPublishers && rosPublishers.cancelCurrentGoal) {
            cancelResult = await rosPublishers.cancelCurrentGoal();
        } else {
            console.log('⚠️ ROS not available, simulating goal cancellation');
        }
        
        // Update order status
        const statusResult = await updateOrderStatusInBackground(
            deviceId, 
            orderId, 
            'cancelled',
            reason || 'Cancelled by user'
        );
        
        if (statusResult) {
            res.json({
                success: true,
                message: 'Order cancelled successfully',
                orderId,
                cancelResult,
                timestamp: new Date().toISOString()
            });
            
            broadcastOrderUpdate(deviceId, orderId, 'cancelled', reason);
        } else {
            res.status(404).json({
                success: false,
                error: 'Order not found'
            });
        }
        
    } catch (error) {
        console.error('❌ Error cancelling order:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Emergency stop for device (stops all orders and navigation)
 * POST /api/orders/:deviceId/emergency_stop
 */
router.post('/:deviceId/emergency_stop', async (req, res) => {
    try {
        const { deviceId } = req.params;
        
        console.log(`🚨 EMERGENCY STOP for device: ${deviceId}`);
        
        // Send emergency stop command to ROS (if available)
        let emergencyResult = { success: true, simulated: !rosPublishers };
        let cancelResult = { success: true, simulated: !rosPublishers };
        
        if (rosPublishers && rosPublishers.emergencyStop && rosPublishers.cancelCurrentGoal) {
            emergencyResult = await rosPublishers.emergencyStop();
            cancelResult = await rosPublishers.cancelCurrentGoal();
        } else {
            console.log('⚠️ ROS not available, simulating emergency stop');
        }
        
        // Set all active orders for this device to cancelled
        const orders = await loadOrders();
        let updatedCount = 0;
        
        for (let order of orders) {
            if (order.deviceId === deviceId && 
                (order.status === 'active' || order.status === 'pending')) {
                order.status = 'cancelled';
                order.updatedAt = new Date().toISOString();
                order.metadata.emergencyStop = true;
                order.metadata.emergencyReason = 'Emergency stop activated';
                updatedCount++;
            }
        }
        
        await saveOrders(orders);
        
        res.json({
            success: true,
            message: `Emergency stop activated for device ${deviceId}`,
            deviceId,
            ordersAffected: updatedCount,
            emergencyResult,
            cancelResult,
            timestamp: new Date().toISOString()
        });
        
        // Broadcast emergency stop
        broadcastEmergencyStop(deviceId, updatedCount);
        
        console.log(`🛑 Emergency stop completed for ${deviceId}: ${updatedCount} orders affected`);
        
    } catch (error) {
        console.error('❌ Error in emergency stop:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Get order execution status with ROS navigation info
 * GET /api/orders/:deviceId/:orderId/execution_status
 */
router.get('/:deviceId/:orderId/execution_status', async (req, res) => {
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
        
        // Get ROS navigation status (if available)
        let navStatus = { status: 'unknown', simulated: !rosPublishers };
        if (rosPublishers && rosPublishers.getNavigationStatus) {
            navStatus = await rosPublishers.getNavigationStatus();
        } else {
            navStatus = {
                status: 'simulated',
                simulated: true,
                message: 'ROS navigation not available'
            };
        }
        
        res.json({
            success: true,
            order: {
                id: order.id,
                name: order.name,
                status: order.status,
                progress: order.progress,
                currentWaypoint: order.currentWaypoint,
                totalWaypoints: order.waypoints.length,
                startedAt: order.startedAt,
                updatedAt: order.updatedAt
            },
            navigation: navStatus,
            execution: {
                isActive: order.status === 'active',
                nextWaypoint: order.currentWaypoint < order.waypoints.length 
                    ? order.waypoints[order.currentWaypoint] 
                    : null
            },
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error getting execution status:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Delete an order
 * DELETE /api/orders/:deviceId/:orderId
 */
router.delete('/:deviceId/:orderId', async (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        
        console.log(`🗑️ Deleting order: ${orderId} for device: ${deviceId}`);
        
        const orders = await loadOrders();
        const orderIndex = orders.findIndex(o => o.id === orderId && o.deviceId === deviceId);
        
        if (orderIndex === -1) {
            return res.status(404).json({
                success: false,
                error: 'Order not found'
            });
        }
        
        const order = orders[orderIndex];
        
        // Check if order can be deleted (not currently executing)
        if (order.status === 'active' || order.status === 'executing') {
            return res.status(400).json({
                success: false,
                error: 'Cannot delete an order that is currently executing. Stop the order first.'
            });
        }
        
        // Remove the order from the array
        orders.splice(orderIndex, 1);
        await saveOrders(orders);
        
        console.log(`✅ Order deleted successfully: ${orderId}`);
        
        res.json({
            success: true,
            message: 'Order deleted successfully',
            deletedOrder: {
                id: order.id,
                name: order.name,
                status: order.status,
                deviceId: order.deviceId
            },
            timestamp: new Date().toISOString()
        });
        
        // Broadcast order deletion
        broadcastOrderUpdate(deviceId, orderId, 'deleted');
        
    } catch (error) {
        console.error('❌ Error deleting order:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Batch execute multiple orders
 * POST /api/orders/:deviceId/batch_execute
 */
router.post('/:deviceId/batch_execute', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { orderIds, orderDelay = 60000, waypointDelay = 3000 } = req.body;
        
        if (!Array.isArray(orderIds) || orderIds.length === 0) {
            return res.status(400).json({
                success: false,
                error: 'orderIds must be a non-empty array'
            });
        }
        
        console.log(`📦 Starting batch execution of ${orderIds.length} orders for ${deviceId}`);
        
        // Execute batch in background
        executeBatchOrders(deviceId, orderIds, orderDelay, waypointDelay).catch(error => {
            console.error(`❌ Batch execution failed: ${error}`);
        });
        
        res.json({
            success: true,
            message: `Batch execution started for ${orderIds.length} orders`,
            deviceId,
            orderIds,
            execution: {
                totalOrders: orderIds.length,
                orderDelay: orderDelay,
                waypointDelay: waypointDelay,
                estimatedTotalTime: `${(orderIds.length * orderDelay) / 1000 / 60}min`
            },
            startedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error starting batch execution:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

// ==========================================
// HELPER FUNCTIONS
// ==========================================

async function executeBatchOrders(deviceId, orderIds, orderDelay, waypointDelay) {
    const results = [];
    
    for (let i = 0; i < orderIds.length; i++) {
        const orderId = orderIds[i];
        console.log(`📋 Executing batch order ${i + 1}/${orderIds.length}: ${orderId}`);
        
        try {
            const orders = await loadOrders();
            const order = orders.find(o => o.id === orderId && o.deviceId === deviceId);
            
            if (order) {
                await executeOrderWaypoints(deviceId, order, waypointDelay);
                results.push({ orderId, success: true });
            } else {
                results.push({ orderId, success: false, error: 'Order not found' });
            }
            
        } catch (error) {
            console.error(`❌ Batch order ${orderId} failed: ${error}`);
            results.push({ orderId, success: false, error: error.message });
        }
        
        // Wait between orders (except for last one)
        if (i < orderIds.length - 1) {
            console.log(`⏳ Waiting ${orderDelay / 1000}s before next batch order...`);
            await delay(orderDelay);
        }
    }
    
    const successCount = results.filter(r => r.success).length;
    console.log(`🏁 Batch execution completed: ${successCount}/${orderIds.length} successful`);
    
    broadcastBatchExecutionComplete(deviceId, orderIds.length, successCount, results);
}

async function updateOrderProgress(deviceId, orderId, currentWaypoint) {
    try {
        const orders = await loadOrders();
        const orderIndex = orders.findIndex(o => o.id === orderId && o.deviceId === deviceId);
        
        if (orderIndex !== -1) {
            const order = orders[orderIndex];
            order.currentWaypoint = currentWaypoint;
            order.progress.completedWaypoints = currentWaypoint;
            order.progress.percentage = Math.round((currentWaypoint / order.waypoints.length) * 100);
            order.updatedAt = new Date().toISOString();
            
            // Mark completed waypoints
            for (let i = 0; i < currentWaypoint && i < order.waypoints.length; i++) {
                order.waypoints[i].completed = true;
                if (!order.waypoints[i].completedAt) {
                    order.waypoints[i].completedAt = new Date().toISOString();
                }
            }
            
            orders[orderIndex] = order;
            await saveOrders(orders);
            
            console.log(`📈 Order progress updated: ${orderId} - ${currentWaypoint}/${order.waypoints.length} (${order.progress.percentage}%)`);
        }
    } catch (error) {
        console.error(`❌ Error updating order progress: ${error}`);
    }
}

async function updateOrderStatusInBackground(deviceId, orderId, status, reason = null) {
    try {
        const orders = await loadOrders();
        const orderIndex = orders.findIndex(o => o.id === orderId && o.deviceId === deviceId);
        
        if (orderIndex !== -1) {
            const order = orders[orderIndex];
            order.status = status;
            order.updatedAt = new Date().toISOString();
            
            if (status === 'completed') {
                order.completedAt = new Date().toISOString();
                order.progress.percentage = 100;
                order.progress.completedWaypoints = order.waypoints.length;
            }
            
            if (reason) {
                order.metadata.statusReason = reason;
            }
            
            orders[orderIndex] = order;
            await saveOrders(orders);
            
            console.log(`🔄 Order status updated: ${orderId} → ${status}`);
            return true;
        }
        return false;
    } catch (error) {
        console.error(`❌ Error updating order status: ${error}`);
        return false;
    }
}

// WebSocket broadcasting functions (implement based on your WebSocket setup)
function broadcastOrderUpdate(deviceId, orderId, status, reason = null) {
    try {
        if (global.webSocketBroadcast) {
            global.webSocketBroadcast('order_update', {
                deviceId,
                orderId,
                status,
                reason,
                timestamp: new Date().toISOString()
            });
        }
    } catch (error) {
        console.error('❌ Error broadcasting order update:', error);
    }
}

function broadcastEmergencyStop(deviceId, affectedOrders) {
    try {
        if (global.webSocketBroadcast) {
            global.webSocketBroadcast('emergency_stop', {
                deviceId,
                affectedOrders,
                timestamp: new Date().toISOString()
            });
        }
    } catch (error) {
        console.error('❌ Error broadcasting emergency stop:', error);
    }
}

function broadcastBatchExecutionComplete(deviceId, totalOrders, successCount, results) {
    try {
        if (global.webSocketBroadcast) {
            global.webSocketBroadcast('batch_execution_complete', {
                deviceId,
                totalOrders,
                successCount,
                failedCount: totalOrders - successCount,
                results,
                timestamp: new Date().toISOString()
            });
        }
    } catch (error) {
        console.error('❌ Error broadcasting batch execution complete:', error);
    }
}

// Utility functions
function delay(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

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

async function ensureOrdersFileExists() {
    try {
        await fs.access(ORDERS_FILE);
    } catch (error) {
        console.log('📁 Creating orders.json file');
        await fs.writeFile(ORDERS_FILE, JSON.stringify([], null, 2));
    }
}

module.exports = router;