// NEW FILE: ros/utils/navigation_feedback.js - Handle navigation action feedback
const rclnodejs = require('rclnodejs');
const { broadcastToSubscribers } = require('../../websocket/messageHandler');

let rosNode = null;
let navigationSubscriber = null;
let actionSubscribers = {};
let isInitialized = false;

// Track active navigation goals
const activeNavigationGoals = new Map();
const navigationCallbacks = new Map();

/**
 * Initialize navigation feedback system
 */
function initializeNavigationFeedback(node) {
    rosNode = node;
    isInitialized = true;
    
    try {
        // Subscribe to navigation action result
        setupNavigationActionSubscriber();
        
        // Subscribe to navigation status topics (alternatives)
        setupNavigationStatusSubscribers();
        
        console.log('✅ Navigation feedback system initialized');
        
    } catch (error) {
        console.error('❌ Error initializing navigation feedback:', error);
    }
}

/**
 * Setup navigation action result subscriber
 */
function setupNavigationActionSubscriber() {
    try {
        // Subscribe to navigate_to_pose action result
        navigationSubscriber = rosNode.createSubscriber(
            'action_msgs/msg/GoalStatusArray', 
            '/navigate_to_pose/_action/status',
            {
                depth: 10,
                reliability: 'best_effort',
                durability: 'volatile'
            }
        );
        
        navigationSubscriber.on('message', handleNavigationActionStatus);
        console.log('✅ Subscribed to /navigate_to_pose/_action/status');
        
        // Also try to subscribe to result topic if available
        try {
            const resultSubscriber = rosNode.createSubscriber(
                'nav2_msgs/action/NavigateToPose_Result',
                '/navigate_to_pose/_action/result',
                {
                    depth: 10,
                    reliability: 'best_effort',
                    durability: 'volatile'
                }
            );
            
            resultSubscriber.on('message', handleNavigationActionResult);
            actionSubscribers['result'] = resultSubscriber;
            console.log('✅ Subscribed to /navigate_to_pose/_action/result');
            
        } catch (resultError) {
            console.warn('⚠️ Could not subscribe to action result topic:', resultError.message);
        }
        
    } catch (error) {
        console.error('❌ Error setting up navigation action subscriber:', error);
    }
}

/**
 * Setup alternative navigation status subscribers
 */
function setupNavigationStatusSubscribers() {
    try {
        // Subscribe to move_base status (if available)
        const moveBaseSubscriber = rosNode.createSubscriber(
            'actionlib_msgs/msg/GoalStatusArray',
            '/move_base/status',
            {
                depth: 10,
                reliability: 'best_effort',
                durability: 'volatile'
            }
        );
        
        moveBaseSubscriber.on('message', handleMoveBaseStatus);
        actionSubscribers['move_base'] = moveBaseSubscriber;
        console.log('✅ Subscribed to /move_base/status (fallback)');
        
    } catch (error) {
        console.warn('⚠️ Could not subscribe to move_base status:', error.message);
    }
    
    try {
        // Subscribe to amcl_pose for position feedback
        const amclSubscriber = rosNode.createSubscriber(
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            '/amcl_pose',
            {
                depth: 10,
                reliability: 'best_effort',
                durability: 'volatile'
            }
        );
        
        amclSubscriber.on('message', handleAMCLPose);
        actionSubscribers['amcl'] = amclSubscriber;
        console.log('✅ Subscribed to /amcl_pose for position feedback');
        
    } catch (error) {
        console.warn('⚠️ Could not subscribe to amcl_pose:', error.message);
    }
}

/**
 * Handle navigation action status messages
 */
function handleNavigationActionStatus(message) {
    try {
        if (message.status_list && message.status_list.length > 0) {
            for (const status of message.status_list) {
                const goalId = status.goal_info?.goal_id?.uuid?.join('') || 'unknown';
                const statusCode = status.status;
                
                console.log(`🎯 Navigation status for goal ${goalId}: ${getStatusText(statusCode)}`);
                
                // Handle different status codes
                switch (statusCode) {
                    case 3: // SUCCEEDED
                        handleNavigationSuccess(goalId, status);
                        break;
                    case 4: // ABORTED
                        handleNavigationFailure(goalId, status, 'Navigation aborted');
                        break;
                    case 5: // REJECTED
                        handleNavigationFailure(goalId, status, 'Navigation rejected');
                        break;
                    case 6: // PREEMPTING
                        handleNavigationPreempted(goalId, status);
                        break;
                    case 7: // PREEMPTED
                        handleNavigationFailure(goalId, status, 'Navigation preempted');
                        break;
                    case 1: // ACTIVE
                        handleNavigationActive(goalId, status);
                        break;
                }
            }
        }
    } catch (error) {
        console.error('❌ Error handling navigation action status:', error);
    }
}

/**
 * Handle navigation action result messages
 */
function handleNavigationActionResult(message) {
    try {
        const goalId = message.goal_id?.uuid?.join('') || 'unknown';
        const result = message.result;
        
        console.log(`🎯 Navigation result for goal ${goalId}:`, result);
        
        if (result && result.result_code !== undefined) {
            if (result.result_code === 0) { // Success
                handleNavigationSuccess(goalId, message);
            } else {
                handleNavigationFailure(goalId, message, `Navigation failed with code: ${result.result_code}`);
            }
        }
    } catch (error) {
        console.error('❌ Error handling navigation action result:', error);
    }
}

/**
 * Handle move_base status (fallback)
 */
function handleMoveBaseStatus(message) {
    try {
        if (message.status_list && message.status_list.length > 0) {
            const latestStatus = message.status_list[message.status_list.length - 1];
            const statusCode = latestStatus.status;
            const goalId = latestStatus.goal_id?.id || 'move_base_goal';
            
            console.log(`🎯 Move Base status: ${getStatusText(statusCode)}`);
            
            switch (statusCode) {
                case 3: // SUCCEEDED
                    handleNavigationSuccess(goalId, latestStatus);
                    break;
                case 4: // ABORTED
                case 5: // REJECTED
                    handleNavigationFailure(goalId, latestStatus, 'Move Base goal failed');
                    break;
            }
        }
    } catch (error) {
        console.error('❌ Error handling move_base status:', error);
    }
}

/**
 * Handle AMCL pose updates for position tracking
 */
function handleAMCLPose(message) {
    try {
        const position = message.pose.pose.position;
        const currentPos = { x: position.x, y: position.y };
        
        // Check if robot is close to any active goal
        activeNavigationGoals.forEach((goalData, goalId) => {
            const targetPos = goalData.targetPosition;
            const distance = calculateDistance(currentPos, targetPos);
            
            // If within 0.5 meters of target, consider goal reached
            if (distance < 0.5) {
                console.log(`🎯 Robot reached target position via AMCL feedback: ${goalId}`);
                handleNavigationSuccess(goalId, { via: 'amcl_proximity' });
            }
        });
        
        // Broadcast current position to WebSocket clients
        broadcastToSubscribers('robot_position', {
            type: 'position_update',
            position: currentPos,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error handling AMCL pose:', error);
    }
}

/**
 * Handle successful navigation
 */
function handleNavigationSuccess(goalId, statusMessage) {
    console.log(`✅ Navigation succeeded for goal: ${goalId}`);
    
    const goalData = activeNavigationGoals.get(goalId);
    if (goalData) {
        const callback = navigationCallbacks.get(goalData.orderId);
        if (callback) {
            callback.onSuccess(goalData);
        }
        
        // Broadcast success to WebSocket clients
        broadcastToSubscribers('navigation_events', {
            type: 'navigation_success',
            goalId: goalId,
            orderId: goalData.orderId,
            waypointIndex: goalData.waypointIndex,
            targetPosition: goalData.targetPosition,
            timestamp: new Date().toISOString()
        });
        
        // Clean up
        activeNavigationGoals.delete(goalId);
        if (goalData.orderId) {
            navigationCallbacks.delete(goalData.orderId);
        }
    }
}

/**
 * Handle navigation failure
 */
function handleNavigationFailure(goalId, statusMessage, reason) {
    console.error(`❌ Navigation failed for goal: ${goalId} - ${reason}`);
    
    const goalData = activeNavigationGoals.get(goalId);
    if (goalData) {
        const callback = navigationCallbacks.get(goalData.orderId);
        if (callback) {
            callback.onFailure(goalData, reason);
        }
        
        // Broadcast failure to WebSocket clients
        broadcastToSubscribers('navigation_events', {
            type: 'navigation_failure',
            goalId: goalId,
            orderId: goalData.orderId,
            waypointIndex: goalData.waypointIndex,
            targetPosition: goalData.targetPosition,
            reason: reason,
            timestamp: new Date().toISOString()
        });
        
        // Clean up
        activeNavigationGoals.delete(goalId);
        if (goalData.orderId) {
            navigationCallbacks.delete(goalData.orderId);
        }
    }
}

/**
 * Handle navigation preempted
 */
function handleNavigationPreempted(goalId, statusMessage) {
    console.log(`⏸️ Navigation preempted for goal: ${goalId}`);
    
    const goalData = activeNavigationGoals.get(goalId);
    if (goalData) {
        // Broadcast preemption to WebSocket clients
        broadcastToSubscribers('navigation_events', {
            type: 'navigation_preempted',
            goalId: goalId,
            orderId: goalData.orderId,
            waypointIndex: goalData.waypointIndex,
            timestamp: new Date().toISOString()
        });
    }
}

/**
 * Handle navigation active status
 */
function handleNavigationActive(goalId, statusMessage) {
    console.log(`🔄 Navigation active for goal: ${goalId}`);
    
    const goalData = activeNavigationGoals.get(goalId);
    if (goalData) {
        // Broadcast active status to WebSocket clients
        broadcastToSubscribers('navigation_events', {
            type: 'navigation_active',
            goalId: goalId,
            orderId: goalData.orderId,
            waypointIndex: goalData.waypointIndex,
            timestamp: new Date().toISOString()
        });
    }
}

/**
 * Register a navigation goal for tracking
 */
function registerNavigationGoal(goalId, orderId, waypointIndex, targetPosition) {
    const goalData = {
        goalId,
        orderId,
        waypointIndex,
        targetPosition,
        startTime: new Date().toISOString()
    };
    
    activeNavigationGoals.set(goalId, goalData);
    console.log(`📝 Registered navigation goal: ${goalId} for order: ${orderId}`);
}

/**
 * Set navigation callback for order execution
 */
function setNavigationCallback(orderId, callbacks) {
    navigationCallbacks.set(orderId, callbacks);
    console.log(`📞 Set navigation callbacks for order: ${orderId}`);
}

/**
 * Clear navigation callback
 */
function clearNavigationCallback(orderId) {
    navigationCallbacks.delete(orderId);
    console.log(`🗑️ Cleared navigation callbacks for order: ${orderId}`);
}

/**
 * Get status text from status code
 */
function getStatusText(statusCode) {
    const statusTexts = {
        0: 'PENDING',
        1: 'ACTIVE', 
        2: 'PREEMPTED',
        3: 'SUCCEEDED',
        4: 'ABORTED',
        5: 'REJECTED',
        6: 'PREEMPTING',
        7: 'RECALLING',
        8: 'RECALLED',
        9: 'LOST'
    };
    
    return statusTexts[statusCode] || `UNKNOWN(${statusCode})`;
}

/**
 * Calculate distance between two points
 */
function calculateDistance(pos1, pos2) {
    const dx = pos1.x - pos2.x;
    const dy = pos1.y - pos2.y;
    return Math.sqrt(dx * dx + dy * dy);
}

/**
 * Get active navigation goals
 */
function getActiveNavigationGoals() {
    return Array.from(activeNavigationGoals.values());
}

/**
 * Cancel all active navigation goals
 */
function cancelAllNavigationGoals() {
    activeNavigationGoals.clear();
    navigationCallbacks.clear();
    console.log('🛑 Cancelled all active navigation goals');
}

/**
 * Cleanup navigation feedback system
 */
function cleanup() {
    try {
        if (navigationSubscriber) {
            navigationSubscriber.destroy();
        }
        
        Object.values(actionSubscribers).forEach(subscriber => {
            try {
                subscriber.destroy();
            } catch (e) {
                console.warn('⚠️ Error destroying subscriber:', e.message);
            }
        });
        
        cancelAllNavigationGoals();
        actionSubscribers = {};
        isInitialized = false;
        
        console.log('🧹 Navigation feedback system cleaned up');
    } catch (error) {
        console.error('❌ Error cleaning up navigation feedback:', error);
    }
}

module.exports = {
    initializeNavigationFeedback,
    registerNavigationGoal,
    setNavigationCallback,
    clearNavigationCallback,
    getActiveNavigationGoals,
    cancelAllNavigationGoals,
    cleanup
};