// routes/enhanced_order_execution.js - Enhanced Order Execution with Target Pose Integration
const express = require('express');
const router = express.Router();
const publishers = require('../ros/utils/publishers');
const { broadcastToSubscribers } = require('../websocket/messageHandler');

// Global order execution state
const activeOrderExecutions = new Map();
const executionHistory = [];

/**
 * ✅ ENHANCED: Execute order with target_pose integration
 * POST /api/enhanced-orders/:deviceId/:orderId/execute
 */
router.post('/devices/:deviceId/orders/:orderId/execute', async (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        const { immediateStart = true, executionMode = 'sequential' } = req.body;
        
        console.log(`🚀 Enhanced order execution starting: ${orderId} for device: ${deviceId}`);
        
        // Get order from global storage
        const orders = global.deviceOrders[deviceId] || [];
        const order = orders.find(o => o.id === orderId);
        
        if (!order) {
            return res.status(404).json({
                success: false,
                error: 'Order not found'
            });
        }
        
        if (order.status !== 'pending' && order.status !== 'paused') {
            return res.status(400).json({
                success: false,
                error: `Cannot execute order with status: ${order.status}`
            });
        }
        
        // Create execution context
        const executionContext = {
            orderId: orderId,
            deviceId: deviceId,
            order: order,
            currentWaypointIndex: order.progress?.currentWaypoint || 0,
            startTime: new Date().toISOString(),
            status: 'initializing',
            executionMode: executionMode,
            waypointResults: [],
            errors: []
        };
        
        activeOrderExecutions.set(orderId, executionContext);
        
        // Update order status
        order.status = 'active';
        order.startedAt = executionContext.startTime;
        order.updatedAt = new Date().toISOString();
        
        // Broadcast order start event
        broadcastToSubscribers('order_events', {
            type: 'order_execution_started',
            deviceId: deviceId,
            orderId: orderId,
            order: order,
            executionContext: {
                mode: executionMode,
                totalWaypoints: order.waypoints.length,
                startTime: executionContext.startTime
            },
            timestamp: new Date().toISOString()
        });
        
        res.json({
            success: true,
            message: 'Order execution started',
            executionId: orderId,
            order: order,
            executionContext: executionContext
        });
        
        // Start execution immediately if requested
        if (immediateStart) {
            setImmediate(() => executeNextWaypoint(orderId));
        }
        
    } catch (error) {
        console.error('❌ Error starting enhanced order execution:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * ✅ NEW: Execute next waypoint in sequence
 */
async function executeNextWaypoint(orderId) {
    try {
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) {
            console.error(`❌ Execution context not found for order: ${orderId}`);
            return;
        }
        
        const { order, deviceId, currentWaypointIndex } = executionContext;
        
        if (currentWaypointIndex >= order.waypoints.length) {
            await completeOrderExecution(orderId);
            return;
        }
        
        const waypoint = order.waypoints[currentWaypointIndex];
        console.log(`🎯 Executing waypoint ${currentWaypointIndex + 1}/${order.waypoints.length}: ${waypoint.name}`);
        
        // Update execution context
        executionContext.status = 'navigating';
        executionContext.currentWaypoint = waypoint;
        
        // Broadcast waypoint start
        broadcastToSubscribers('order_events', {
            type: 'waypoint_execution_started',
            deviceId: deviceId,
            orderId: orderId,
            waypoint: waypoint,
            waypointIndex: currentWaypointIndex,
            progress: {
                current: currentWaypointIndex + 1,
                total: order.waypoints.length,
                percentage: Math.round(((currentWaypointIndex + 1) / order.waypoints.length) * 100)
            },
            timestamp: new Date().toISOString()
        });
        
        // Publish target pose to ROS
        const targetPoseResult = await publishTargetPose(waypoint, deviceId);
        
        if (targetPoseResult.success) {
            console.log(`✅ Target pose published successfully for waypoint: ${waypoint.name}`);
            
            // Record waypoint result
            const waypointResult = {
                waypointIndex: currentWaypointIndex,
                waypoint: waypoint,
                startTime: new Date().toISOString(),
                targetPoseResult: targetPoseResult,
                status: 'navigating'
            };
            
            executionContext.waypointResults.push(waypointResult);
            
            // Update order progress
            order.progress.currentWaypoint = currentWaypointIndex;
            order.progress.completedWaypoints = currentWaypointIndex;
            order.progress.percentage = Math.round((currentWaypointIndex / order.waypoints.length) * 100);
            
            // For demo purposes, simulate navigation completion after a delay
            // In a real implementation, you'd wait for ROS navigation feedback
            setTimeout(() => {
                handleWaypointCompletion(orderId, currentWaypointIndex, true);
            }, 5000); // 5 second simulation
            
        } else {
            console.error(`❌ Failed to publish target pose for waypoint: ${waypoint.name}`);
            await handleWaypointCompletion(orderId, currentWaypointIndex, false, targetPoseResult.error);
        }
        
    } catch (error) {
        console.error(`❌ Error executing waypoint for order ${orderId}:`, error);
        await handleOrderExecutionError(orderId, error);
    }
}

/**
 * ✅ ENHANCED: Publish target pose using your existing publisher
 */
async function publishTargetPose(waypoint, deviceId) {
    try {
        const position = waypoint.position;
        const orientation = waypoint.orientation || 0;
        
        console.log(`📍 Publishing target pose: (${position.x}, ${position.y}, orientation: ${orientation})`);
        
        // Use your existing publishGoal function from publishers.js
        const result = publishers.publishGoal(
            position.x,
            position.y,
            orientation
        );
        
        if (result.success) {
            console.log(`🎯 Target pose published successfully to /target_pose`);
            return {
                success: true,
                targetPose: {
                    x: result.x,
                    y: result.y,
                    orientation: result.orientation
                },
                messageCount: result.messageCount,
                timestamp: result.timestamp
            };
        } else {
            throw new Error(result.error || 'Failed to publish target pose');
        }
        
    } catch (error) {
        console.error(`❌ Error publishing target pose:`, error);
        return {
            success: false,
            error: error.message
        };
    }
}

/**
 * ✅ NEW: Handle waypoint completion
 */
async function handleWaypointCompletion(orderId, waypointIndex, success, errorMessage = null) {
    try {
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) return;
        
        const { order, deviceId } = executionContext;
        const waypoint = order.waypoints[waypointIndex];
        
        // Update waypoint result
        const waypointResult = executionContext.waypointResults[waypointIndex];
        if (waypointResult) {
            waypointResult.completedTime = new Date().toISOString();
            waypointResult.status = success ? 'completed' : 'failed';
            waypointResult.error = errorMessage;
        }
        
        // Mark waypoint as completed in order
        waypoint.completed = success;
        waypoint.completedAt = new Date().toISOString();
        if (errorMessage) {
            waypoint.error = errorMessage;
        }
        
        console.log(`${success ? '✅' : '❌'} Waypoint ${waypointIndex + 1} ${success ? 'completed' : 'failed'}: ${waypoint.name}`);
        
        // Broadcast waypoint completion
        broadcastToSubscribers('order_events', {
            type: 'waypoint_execution_completed',
            deviceId: deviceId,
            orderId: orderId,
            waypoint: waypoint,
            waypointIndex: waypointIndex,
            success: success,
            error: errorMessage,
            progress: {
                current: waypointIndex + 1,
                total: order.waypoints.length,
                percentage: Math.round(((waypointIndex + 1) / order.waypoints.length) * 100)
            },
            timestamp: new Date().toISOString()
        });
        
        if (success) {
            // Update order progress
            order.progress.completedWaypoints = waypointIndex + 1;
            order.progress.percentage = Math.round(((waypointIndex + 1) / order.waypoints.length) * 100);
            
            // Move to next waypoint
            executionContext.currentWaypointIndex = waypointIndex + 1;
            
            // Continue to next waypoint after a short delay
            setTimeout(() => {
                executeNextWaypoint(orderId);
            }, 1000);
            
        } else {
            // Handle waypoint failure
            await handleOrderExecutionError(orderId, new Error(errorMessage || 'Waypoint execution failed'));
        }
        
    } catch (error) {
        console.error(`❌ Error handling waypoint completion:`, error);
    }
}

/**
 * ✅ NEW: Complete order execution
 */
async function completeOrderExecution(orderId) {
    try {
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) return;
        
        const { order, deviceId } = executionContext;
        
        // Update order status
        order.status = 'completed';
        order.completedAt = new Date().toISOString();
        order.progress.completedWaypoints = order.waypoints.length;
        order.progress.percentage = 100;
        
        // Calculate execution duration
        const startTime = new Date(executionContext.startTime);
        const endTime = new Date(order.completedAt);
        order.actualDuration = Math.round((endTime - startTime) / 1000); // seconds
        
        // Update execution context
        executionContext.status = 'completed';
        executionContext.endTime = order.completedAt;
        executionContext.duration = order.actualDuration;
        
        console.log(`🎉 Order execution completed: ${orderId} (${order.actualDuration}s)`);
        
        // Add to execution history
        executionHistory.push({
            ...executionContext,
            completedAt: new Date().toISOString()
        });
        
        // Remove from active executions
        activeOrderExecutions.delete(orderId);
        
        // Broadcast order completion
        broadcastToSubscribers('order_events', {
            type: 'order_execution_completed',
            deviceId: deviceId,
            orderId: orderId,
            order: order,
            executionSummary: {
                duration: order.actualDuration,
                totalWaypoints: order.waypoints.length,
                successfulWaypoints: order.waypoints.filter(wp => wp.completed).length,
                failedWaypoints: order.waypoints.filter(wp => !wp.completed).length
            },
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error(`❌ Error completing order execution:`, error);
    }
}

/**
 * ✅ NEW: Handle order execution errors
 */
async function handleOrderExecutionError(orderId, error) {
    try {
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) return;
        
        const { order, deviceId } = executionContext;
        
        // Update order status
        order.status = 'failed';
        order.updatedAt = new Date().toISOString();
        if (!order.metadata) order.metadata = {};
        order.metadata.failureReason = error.message;
        
        // Update execution context
        executionContext.status = 'failed';
        executionContext.error = error.message;
        executionContext.endTime = new Date().toISOString();
        
        console.error(`❌ Order execution failed: ${orderId} - ${error.message}`);
        
        // Emergency stop
        try {
            publishers.emergencyStop();
            console.log('🛑 Emergency stop activated due to order execution failure');
        } catch (stopError) {
            console.error('❌ Failed to activate emergency stop:', stopError);
        }
        
        // Add to execution history
        executionHistory.push({
            ...executionContext,
            failedAt: new Date().toISOString()
        });
        
        // Remove from active executions
        activeOrderExecutions.delete(orderId);
        
        // Broadcast order failure
        broadcastToSubscribers('order_events', {
            type: 'order_execution_failed',
            deviceId: deviceId,
            orderId: orderId,
            order: order,
            error: error.message,
            timestamp: new Date().toISOString()
        });
        
    } catch (handlingError) {
        console.error(`❌ Error handling order execution error:`, handlingError);
    }
}

/**
 * ✅ NEW: Pause order execution
 * POST /api/enhanced-orders/:deviceId/:orderId/pause
 */
router.post('/devices/:deviceId/orders/:orderId/pause', (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) {
            return res.status(404).json({
                success: false,
                error: 'Order execution not found'
            });
        }
        
        // Update order and execution status
        executionContext.order.status = 'paused';
        executionContext.order.updatedAt = new Date().toISOString();
        executionContext.status = 'paused';
        executionContext.pausedAt = new Date().toISOString();
        
        // Emergency stop to halt current movement
        publishers.emergencyStop();
        
        console.log(`⏸️ Order execution paused: ${orderId}`);
        
        // Broadcast pause event
        broadcastToSubscribers('order_events', {
            type: 'order_execution_paused',
            deviceId: deviceId,
            orderId: orderId,
            order: executionContext.order,
            timestamp: new Date().toISOString()
        });
        
        res.json({
            success: true,
            message: 'Order execution paused',
            order: executionContext.order
        });
        
    } catch (error) {
        console.error('❌ Error pausing order execution:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * ✅ NEW: Resume order execution
 * POST /api/enhanced-orders/:deviceId/:orderId/resume
 */
router.post('/devices/:deviceId/orders/:orderId/resume', (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) {
            return res.status(404).json({
                success: false,
                error: 'Order execution not found'
            });
        }
        
        if (executionContext.status !== 'paused') {
            return res.status(400).json({
                success: false,
                error: 'Order is not paused'
            });
        }
        
        // Update order and execution status
        executionContext.order.status = 'active';
        executionContext.order.updatedAt = new Date().toISOString();
        executionContext.status = 'resuming';
        executionContext.resumedAt = new Date().toISOString();
        
        console.log(`▶️ Order execution resumed: ${orderId}`);
        
        // Broadcast resume event
        broadcastToSubscribers('order_events', {
            type: 'order_execution_resumed',
            deviceId: deviceId,
            orderId: orderId,
            order: executionContext.order,
            timestamp: new Date().toISOString()
        });
        
        res.json({
            success: true,
            message: 'Order execution resumed',
            order: executionContext.order
        });
        
        // Continue execution
        setImmediate(() => executeNextWaypoint(orderId));
        
    } catch (error) {
        console.error('❌ Error resuming order execution:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * ✅ NEW: Cancel order execution
 * POST /api/enhanced-orders/:deviceId/:orderId/cancel
 */
router.post('/devices/:deviceId/orders/:orderId/cancel', (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        const { reason = 'User cancelled' } = req.body;
        
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) {
            return res.status(404).json({
                success: false,
                error: 'Order execution not found'
            });
        }
        
        // Update order status
        executionContext.order.status = 'cancelled';
        executionContext.order.updatedAt = new Date().toISOString();
        if (!executionContext.order.metadata) executionContext.order.metadata = {};
        executionContext.order.metadata.cancelReason = reason;
        
        // Update execution context
        executionContext.status = 'cancelled';
        executionContext.cancelledAt = new Date().toISOString();
        executionContext.cancelReason = reason;
        
        // Emergency stop
        publishers.emergencyStop();
        
        console.log(`❌ Order execution cancelled: ${orderId} - ${reason}`);
        
        // Add to execution history
        executionHistory.push({
            ...executionContext,
            cancelledAt: new Date().toISOString()
        });
        
        // Remove from active executions
        activeOrderExecutions.delete(orderId);
        
        // Broadcast cancellation event
        broadcastToSubscribers('order_events', {
            type: 'order_execution_cancelled',
            deviceId: deviceId,
            orderId: orderId,
            order: executionContext.order,
            reason: reason,
            timestamp: new Date().toISOString()
        });
        
        res.json({
            success: true,
            message: 'Order execution cancelled',
            order: executionContext.order
        });
        
    } catch (error) {
        console.error('❌ Error cancelling order execution:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * ✅ NEW: Get execution status
 * GET /api/enhanced-orders/:deviceId/:orderId/status
 */
router.get('/devices/:deviceId/orders/:orderId/status', (req, res) => {
    try {
        const { orderId } = req.params;
        
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) {
            return res.status(404).json({
                success: false,
                error: 'Order execution not found'
            });
        }
        
        res.json({
            success: true,
            executionContext: executionContext,
            isActive: activeOrderExecutions.has(orderId)
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
 * ✅ NEW: Get execution history
 * GET /api/enhanced-orders/execution-history
 */
router.get('/execution-history', (req, res) => {
    try {
        const { deviceId, limit = 50, offset = 0 } = req.query;
        
        let history = executionHistory;
        
        if (deviceId) {
            history = history.filter(exec => exec.deviceId === deviceId);
        }
        
        const total = history.length;
        const paginatedHistory = history
            .slice(parseInt(offset), parseInt(offset) + parseInt(limit));
        
        res.json({
            success: true,
            history: paginatedHistory,
            total: total,
            hasMore: parseInt(offset) + parseInt(limit) < total
        });
        
    } catch (error) {
        console.error('❌ Error getting execution history:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * ✅ NEW: Get active executions
 * GET /api/enhanced-orders/active-executions
 */
router.get('/active-executions', (req, res) => {
    try {
        const { deviceId } = req.query;
        
        let activeExecutions = Array.from(activeOrderExecutions.values());
        
        if (deviceId) {
            activeExecutions = activeExecutions.filter(exec => exec.deviceId === deviceId);
        }
        
        res.json({
            success: true,
            activeExecutions: activeExecutions,
            count: activeExecutions.length
        });
        
    } catch (error) {
        console.error('❌ Error getting active executions:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

module.exports = router;
