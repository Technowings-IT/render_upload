// routes/enhanced_order_execution.js - Enhanced Order Execution with Real Navigation Feedback
const express = require('express');
const router = express.Router();
const publishers = require('../ros/utils/publishers');
const navigationFeedback = require('../ros/utils/navigation_feedback');
const { broadcastToSubscribers } = require('../websocket/messageHandler');

// Global order execution state
const activeOrderExecutions = new Map();
const executionHistory = [];

/**
 *  ENHANCED: Execute order with real navigation feedback integration
 * POST /api/enhanced-orders/:deviceId/:orderId/execute
 */
router.post('/devices/:deviceId/orders/:orderId/execute', async (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        const { immediateStart = true, executionMode = 'sequential' } = req.body;

        console.log(` Enhanced order execution starting: ${orderId} for device: ${deviceId}`);

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
            errors: [],
            useRealNavigation: true //  NEW: Flag for real navigation
        };

        activeOrderExecutions.set(orderId, executionContext);

        //  NEW: Set up navigation feedback callbacks
        navigationFeedback.setNavigationCallback(orderId, {
            onSuccess: (goalData) => handleNavigationSuccess(orderId, goalData),
            onFailure: (goalData, reason) => handleNavigationFailure(orderId, goalData, reason)
        });

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
                startTime: executionContext.startTime,
                useRealNavigation: true
            },
            timestamp: new Date().toISOString()
        });

        res.json({
            success: true,
            message: 'Order execution started with real navigation feedback',
            executionId: orderId,
            order: order,
            executionContext: executionContext
        });

        // Start execution immediately if requested
        if (immediateStart) {
            setImmediate(() => executeNextWaypointWithRealNav(orderId));
        }

    } catch (error) {
        console.error(' Error starting enhanced order execution:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 *  NEW: Execute waypoint with real navigation feedback
 */
async function executeNextWaypointWithRealNav(orderId) {
    try {
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) {
            console.error(` Execution context not found for order: ${orderId}`);
            return;
        }
        
        const { order, deviceId, currentWaypointIndex } = executionContext;
        
        if (currentWaypointIndex >= order.waypoints.length) {
            await completeOrderExecution(orderId);
            return;
        }
        
        const waypoint = order.waypoints[currentWaypointIndex];
        console.log(` Executing waypoint ${currentWaypointIndex + 1}/${order.waypoints.length}: ${waypoint.name}`);
        console.log(` Target coordinates: (${waypoint.position.x}, ${waypoint.position.y})`);
        console.log(` Using REAL navigation feedback`);
        
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
            coordinates: {
                x: waypoint.position.x,
                y: waypoint.position.y,
                orientation: waypoint.orientation || 0
            },
            progress: {
                current: currentWaypointIndex + 1,
                total: order.waypoints.length,
                percentage: Math.round(((currentWaypointIndex + 1) / order.waypoints.length) * 100)
            },
            navigationMode: 'real_feedback',
            timestamp: new Date().toISOString()
        });
        
        //  NEW: Publish target pose and register for navigation feedback
        const targetPoseResult = await publishTargetPoseWithTracking(waypoint, deviceId, orderId, currentWaypointIndex);
        
        if (targetPoseResult.success) {
            console.log(` Target pose published with tracking: ${waypoint.name}`);
            console.log(` Goal ID: ${targetPoseResult.goalId}`);
            console.log(` Waiting for real navigation feedback...`);
            
            // Record waypoint result
            const waypointResult = {
                waypointIndex: currentWaypointIndex,
                waypoint: waypoint,
                startTime: new Date().toISOString(),
                targetPoseResult: targetPoseResult,
                status: 'navigating',
                goalId: targetPoseResult.goalId,
                navigationMode: 'real_feedback'
            };
            
            executionContext.waypointResults.push(waypointResult);
            
            // Update order progress
            order.progress.currentWaypoint = currentWaypointIndex;
            
            //  NO TIMEOUT: Wait for real navigation feedback
            console.log(`⏳ Navigation started, waiting for action feedback from /navigate_to_pose/_action/result...`);
            
        } else {
            console.error(` Failed to publish target pose for waypoint: ${waypoint.name}`);
            await handleNavigationFailure(orderId, { waypointIndex: currentWaypointIndex }, targetPoseResult.error);
        }
        
    } catch (error) {
        console.error(` Error executing waypoint for order ${orderId}:`, error);
        await handleOrderExecutionError(orderId, error);
    }
}

/**
 *  NEW: Publish target pose with goal tracking
 */
async function publishTargetPoseWithTracking(waypoint, deviceId, orderId, waypointIndex) {
    try {
        const position = waypoint.position;
        const orientation = waypoint.orientation || 0;
        
        // Validate coordinates
        if (!isValidCoordinate(position.x) || !isValidCoordinate(position.y)) {
            throw new Error(`Invalid coordinates: x=${position.x}, y=${position.y}`);
        }
        
        console.log(` Publishing tracked target pose: (${position.x}, ${position.y}, orientation: ${orientation})`);
        
        //  NEW: Generate unique goal ID for tracking
        const goalId = `goal_${orderId}_wp${waypointIndex}_${Date.now()}`;
        
        // Publish target pose
        const result = publishers.publishGoalWithId ? 
            publishers.publishGoalWithId(position.x, position.y, orientation, goalId) :
            publishers.publishGoal(position.x, position.y, orientation);
        
        if (result.success) {
            //  NEW: Register goal for navigation feedback tracking
            navigationFeedback.registerNavigationGoal(
                goalId,
                orderId,
                waypointIndex,
                { x: position.x, y: position.y }
            );
            
            console.log(` Target pose published with tracking to /target_pose`);
            console.log(` Goal ID: ${goalId} registered for feedback`);
            
            return {
                success: true,
                goalId: goalId,
                targetPose: {
                    x: result.x,
                    y: result.y,
                    orientation: result.orientation
                },
                messageCount: result.messageCount,
                timestamp: result.timestamp,
                topic: result.topic || '/target_pose'
            };
        } else {
            throw new Error(result.error || 'Failed to publish target pose');
        }
        
    } catch (error) {
        console.error(` Error publishing tracked target pose:`, error);
        return {
            success: false,
            error: error.message,
            coordinates: { x: waypoint.position.x, y: waypoint.position.y }
        };
    }
}

/**
 *  NEW: Handle successful navigation (called by navigation feedback)
 */
async function handleNavigationSuccess(orderId, goalData) {
    try {
        console.log(` Navigation SUCCESS callback for order: ${orderId}`);
        console.log(` Goal reached: ${goalData.goalId}, waypoint: ${goalData.waypointIndex + 1}`);
        
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) {
            console.error(` Execution context not found for completed navigation: ${orderId}`);
            return;
        }
        
        const { order, deviceId } = executionContext;
        const waypointIndex = goalData.waypointIndex;
        const waypoint = order.waypoints[waypointIndex];
        
        // Calculate navigation time
        const waypointResult = executionContext.waypointResults[waypointIndex];
        let navigationDuration = 0;
        
        if (waypointResult && waypointResult.startTime) {
            const startTime = new Date(waypointResult.startTime);
            const endTime = new Date();
            navigationDuration = Math.round((endTime - startTime) / 1000);
        }
        
        // Update waypoint result
        if (waypointResult) {
            waypointResult.completedTime = new Date().toISOString();
            waypointResult.status = 'completed';
            waypointResult.actualDuration = navigationDuration;
            waypointResult.completionMethod = 'ros_action_feedback';
        }
        
        // Mark waypoint as completed
        waypoint.completed = true;
        waypoint.completedAt = new Date().toISOString();
        
        console.log(` Waypoint ${waypointIndex + 1} completed via ROS action: ${waypoint.name}`);
        console.log(`⏱️ Navigation time: ${navigationDuration} seconds`);
        
        // Broadcast waypoint completion
        broadcastToSubscribers('order_events', {
            type: 'waypoint_execution_completed',
            deviceId: deviceId,
            orderId: orderId,
            waypoint: waypoint,
            waypointIndex: waypointIndex,
            success: true,
            navigationDuration: navigationDuration,
            completionMethod: 'ros_action_feedback',
            goalId: goalData.goalId,
            progress: {
                current: waypointIndex + 1,
                total: order.waypoints.length,
                percentage: Math.round(((waypointIndex + 1) / order.waypoints.length) * 100)
            },
            timestamp: new Date().toISOString()
        });
        
        // Update order progress
        order.progress.completedWaypoints = waypointIndex + 1;
        order.progress.percentage = Math.round(((waypointIndex + 1) / order.waypoints.length) * 100);
        
        // Move to next waypoint
        executionContext.currentWaypointIndex = waypointIndex + 1;
        
        //  NEW: Small delay before next waypoint (configurable)
        const nextWaypointDelay = 2000; // 2 seconds between waypoints
        console.log(`⏳ Waiting ${nextWaypointDelay/1000} seconds before next waypoint...`);
        
        setTimeout(() => {
            executeNextWaypointWithRealNav(orderId);
        }, nextWaypointDelay);
        
    } catch (error) {
        console.error(` Error handling navigation success:`, error);
        await handleOrderExecutionError(orderId, error);
    }
}

/**
 *  NEW: Handle navigation failure (called by navigation feedback)
 */
async function handleNavigationFailure(orderId, goalData, reason) {
    try {
        console.error(` Navigation FAILURE callback for order: ${orderId}`);
        console.error(` Goal failed: ${goalData.goalId || 'unknown'}, reason: ${reason}`);
        
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) {
            console.error(` Execution context not found for failed navigation: ${orderId}`);
            return;
        }
        
        const { order, deviceId } = executionContext;
        const waypointIndex = goalData.waypointIndex || executionContext.currentWaypointIndex;
        const waypoint = order.waypoints[waypointIndex];
        
        // Update waypoint result
        const waypointResult = executionContext.waypointResults[waypointIndex];
        if (waypointResult) {
            waypointResult.completedTime = new Date().toISOString();
            waypointResult.status = 'failed';
            waypointResult.error = reason;
            waypointResult.completionMethod = 'ros_action_feedback';
        }
        
        // Mark waypoint as failed
        waypoint.error = reason;
        
        // Broadcast waypoint failure
        broadcastToSubscribers('order_events', {
            type: 'waypoint_execution_failed',
            deviceId: deviceId,
            orderId: orderId,
            waypoint: waypoint,
            waypointIndex: waypointIndex,
            success: false,
            error: reason,
            completionMethod: 'ros_action_feedback',
            goalId: goalData.goalId,
            timestamp: new Date().toISOString()
        });
        
        // Handle the failure
        await handleOrderExecutionError(orderId, new Error(`Waypoint ${waypointIndex + 1} navigation failed: ${reason}`));
        
    } catch (error) {
        console.error(` Error handling navigation failure:`, error);
    }
}

function isValidCoordinate(value) {
    return typeof value === 'number' &&
        !isNaN(value) &&
        isFinite(value) &&
        Math.abs(value) < 1000; // Reasonable bounds for map coordinates
}

/**
 *  ENHANCED: Complete order execution with navigation cleanup
 */
async function completeOrderExecution(orderId) {
    try {
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) return;

        const { order, deviceId } = executionContext;

        //  NEW: Clear navigation callbacks
        navigationFeedback.clearNavigationCallback(orderId);

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

        console.log(` Order execution completed with real navigation: ${orderId} (${order.actualDuration}s)`);

        // Add to execution history
        executionHistory.push({
            ...executionContext,
            completedAt: new Date().toISOString(),
            navigationMode: 'real_feedback'
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
                failedWaypoints: order.waypoints.filter(wp => !wp.completed).length,
                navigationMode: 'real_feedback'
            },
            timestamp: new Date().toISOString()
        });

    } catch (error) {
        console.error(` Error completing order execution:`, error);
    }
}

/**
 *  ENHANCED: Handle order execution errors with navigation cleanup
 */
async function handleOrderExecutionError(orderId, error) {
    try {
        const executionContext = activeOrderExecutions.get(orderId);
        if (!executionContext) return;

        const { order, deviceId } = executionContext;

        //  NEW: Clear navigation callbacks
        navigationFeedback.clearNavigationCallback(orderId);

        // Update order status
        order.status = 'failed';
        order.updatedAt = new Date().toISOString();
        if (!order.metadata) order.metadata = {};
        order.metadata.failureReason = error.message;

        // Update execution context
        executionContext.status = 'failed';
        executionContext.error = error.message;
        executionContext.endTime = new Date().toISOString();

        console.error(` Order execution failed: ${orderId} - ${error.message}`);

        // Emergency stop
        try {
            publishers.emergencyStop();
            console.log(' Emergency stop activated due to order execution failure');
        } catch (stopError) {
            console.error(' Failed to activate emergency stop:', stopError);
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
        console.error(` Error handling order execution error:`, handlingError);
    }
}

/**
 *  NEW: Pause order execution
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
        console.error(' Error pausing order execution:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 *  NEW: Resume order execution
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

        // Continue execution with real navigation
        setImmediate(() => executeNextWaypointWithRealNav(orderId));

    } catch (error) {
        console.error(' Error resuming order execution:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 *  ENHANCED: Cancel order execution with navigation cleanup
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

        //  NEW: Cancel active navigation goals
        navigationFeedback.cancelAllNavigationGoals();
        navigationFeedback.clearNavigationCallback(orderId);

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

        console.log(` Order execution cancelled with navigation stop: ${orderId} - ${reason}`);

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
            navigationCancelled: true,
            timestamp: new Date().toISOString()
        });

        res.json({
            success: true,
            message: 'Order execution cancelled and navigation stopped',
            order: executionContext.order
        });

    } catch (error) {
        console.error(' Error cancelling order execution:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 *  NEW: Get execution status
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
        console.error(' Error getting execution status:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 *  NEW: Get navigation status for debugging
 * GET /api/enhanced-orders/:deviceId/:orderId/navigation-status
 */
router.get('/devices/:deviceId/orders/:orderId/navigation-status', (req, res) => {
    try {
        const { orderId } = req.params;
        
        const executionContext = activeOrderExecutions.get(orderId);
        const activeGoals = navigationFeedback.getActiveNavigationGoals ? 
            navigationFeedback.getActiveNavigationGoals() : [];
        
        res.json({
            success: true,
            orderId: orderId,
            executionActive: !!executionContext,
            currentStatus: executionContext?.status,
            currentWaypoint: executionContext?.currentWaypointIndex,
            activeNavigationGoals: activeGoals.filter(goal => goal.orderId === orderId),
            allActiveGoals: activeGoals.length,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error(' Error getting navigation status:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 *  NEW: Get execution history
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
        console.error(' Error getting execution history:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 *  NEW: Get active executions
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
        console.error(' Error getting active executions:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

module.exports = router;
