// ros/utils/publishers.js - FIXED with proper speed limiting from UI
const rclnodejs = require('rclnodejs');
const config = require('../../config');

let rosNode = null;
let publishers = {};
let isInitialized = false;

// Store current max speeds from UI (updated via WebSocket)
let currentMaxSpeeds = {
    linear: 1.0,  // Default values
    angular: 2.0
};

//  ADD: Active goal tracking at the top of the file
let activeGoals = new Map(); // goalId -> goal info
let goalCounter = 0;

// Initialize with the node from ros_connection.js
function initializePublishers(node) {
    rosNode = node;
    isInitialized = true;
    console.log(' Publishers initialized with ROS node');
    
    // Pre-create essential publishers for better performance
    createEssentialPublishers();
}

//  NEW: Update max speeds from UI
function updateMaxSpeeds(maxLinearSpeed, maxAngularSpeed) {
    currentMaxSpeeds.linear = Math.max(0.1, Math.min(2.0, maxLinearSpeed));
    currentMaxSpeeds.angular = Math.max(0.1, Math.min(3.0, maxAngularSpeed));
    
    console.log(` Updated max speeds: Linear=${currentMaxSpeeds.linear} m/s, Angular=${currentMaxSpeeds.angular} rad/s`);
    
    return currentMaxSpeeds;
}


//  NEW: Get current max speeds
function getCurrentMaxSpeeds() {
    return { ...currentMaxSpeeds };
}

//  FIXED: Pre-create publishers for immediate use
function createEssentialPublishers() {
    try {
        // Create cmd_vel_joystick publisher immediately for joystick control
        getOrCreatePublisher('/cmd_vel_joystick', 'geometry_msgs/msg/Twist');
        
        //  CRITICAL: Create target_pose publisher for navigation orders
        getOrCreatePublisher('/target_pose', 'geometry_msgs/msg/Twist');
        
        //  NEW: Create additional navigation publishers
        getOrCreatePublisher('/move_base_simple/goal', 'geometry_msgs/msg/PoseStamped');
        getOrCreatePublisher('/navigate_to_pose/_action/send_goal', 'nav2_msgs/action/NavigateToPose');
        
        //  NEW: Create cancel goal publisher
        getOrCreatePublisher('/navigate_to_pose/_action/cancel_goal', 'action_msgs/msg/CancelGoal');
        
        console.log(' Essential publishers created including target_pose and navigation topics');
        
    } catch (error) {
        console.error(' Error creating essential publishers:', error);
    }
}

// Create publisher if not exists
function getOrCreatePublisher(topicName, messageType) {
    if (!publishers[topicName]) {
        if (!rosNode) {
            throw new Error('ROS node not initialized. Call initializePublishers first.');
        }
        
        try {
            const publisher = rosNode.createPublisher(messageType, topicName, {
                depth: config.ROS2.QOS.DEPTH,
                reliability: config.ROS2.QOS.RELIABILITY,
                durability: config.ROS2.QOS.DURABILITY
            });
            
            publishers[topicName] = {
                publisher: publisher,
                messageType: messageType,
                createdAt: new Date().toISOString(),
                messageCount: 0
            };
            
            console.log(` Created publisher for topic: ${topicName} (${messageType})`);
            
        } catch (error) {
            console.error(` Failed to create publisher for ${topicName}:`, error);
            throw error;
        }
    }
    return publishers[topicName].publisher;
}

/**
 *  FIXED: Enhanced velocity publishing with safety limits and validation
 */
function publishVelocity(linear = 0, angular = 0) {
    try {
        if (!isInitialized) {
            throw new Error('Publishers not initialized');
        }
        
        // Apply safety limits from config
        const clampedLinear = Math.max(
            config.SAFETY.VELOCITY_LIMITS.LINEAR.MIN,
            Math.min(config.SAFETY.VELOCITY_LIMITS.LINEAR.MAX, linear)
        );
        
        const clampedAngular = Math.max(
            config.SAFETY.VELOCITY_LIMITS.ANGULAR.MIN,
            Math.min(config.SAFETY.VELOCITY_LIMITS.ANGULAR.MAX, angular)
        );
        
        const publisher = getOrCreatePublisher('/cmd_vel_joystick', 'geometry_msgs/msg/Twist');
        
        const velocityMsg = {
            linear: { 
                x: clampedLinear, 
                y: 0.0, 
                z: 0.0 
            },
            angular: { 
                x: 0.0, 
                y: 0.0, 
                z: clampedAngular 
            }
        };
        
        publisher.publish(velocityMsg);
        
        // Update message count
        if (publishers['/cmd_vel_joystick']) {
            publishers['/cmd_vel_joystick'].messageCount++;
        }
        
        // Log every few messages to avoid spam
        if (publishers['/cmd_vel_joystick'].messageCount % 10 === 0 || Math.abs(clampedLinear) > 0.1 || Math.abs(clampedAngular) > 0.1) {
            console.log(` Published velocity [${publishers['/cmd_vel_joystick'].messageCount}]: linear=${clampedLinear.toFixed(3)}, angular=${clampedAngular.toFixed(3)}`);
        }
        
        return {
            success: true,
            message: 'Velocity command published successfully',
            data: { 
                linear: clampedLinear, 
                angular: clampedAngular,
                messageCount: publishers['/cmd_vel_joystick'].messageCount
            },
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error(' Failed to publish velocity:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

/**
 *  FIXED: Enhanced joystick publishing with proper normalized value handling
 */
function publishJoystick(normalizedX, normalizedY, deadman = false, maxLinearSpeed = null, maxAngularSpeed = null) {
    try {
        if (!isInitialized) {
            throw new Error('Publishers not initialized');
        }

        // Only send velocity if deadman switch is active
        if (!deadman) {
            return publishVelocity(0.0, 0.0);
        }

        // Use provided max speeds or current stored ones
        const maxLinear = maxLinearSpeed || currentMaxSpeeds.linear;
        const maxAngular = maxAngularSpeed || currentMaxSpeeds.angular;

        // Apply deadzone for stability on normalized values
        const deadzone = 0.1;
        const processedX = Math.abs(normalizedX) < deadzone ? 0 : normalizedX;
        const processedY = Math.abs(normalizedY) < deadzone ? 0 : normalizedY;

        //  CRITICAL FIX: Convert normalized values to actual velocities
        // normalizedY comes as linear (forward/backward)
        // normalizedX comes as angular (left/right) 
        const actualLinear = processedY * maxLinear;
        const actualAngular = -processedX * maxAngular; // Negative for correct rotation direction

        console.log(` Joystick: normalized(${normalizedX.toFixed(3)}, ${normalizedY.toFixed(3)}) → actual(${actualLinear.toFixed(3)}, ${actualAngular.toFixed(3)}) | max(${maxLinear}, ${maxAngular}) | deadman=${deadman}`);

        const result = publishVelocity(actualLinear, actualAngular);

        // Add joystick-specific info
        result.joystickData = {
            normalizedX: normalizedX,
            normalizedY: normalizedY,
            processedX: processedX,
            processedY: processedY,
            actualLinear: actualLinear,
            actualAngular: actualAngular,
            maxLinear: maxLinear,
            maxAngular: maxAngular,
            deadman: deadman,
            deadzone: deadzone
        };

        return result;

    } catch (error) {
        console.error(' Failed to publish joystick command:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

/**
 *  ENHANCED: Goal publishing with Twist message for /target_pose
 */
function publishGoalWithId(x, y, orientation = 0, goalId = null) {
    try {
        if (!isInitialized) {
            throw new Error('Publishers not initialized');
        }
        
        //  NEW: Validate coordinates
        if (!isFinite(x) || !isFinite(y) || !isFinite(orientation)) {
            throw new Error(`Invalid coordinates: x=${x}, y=${y}, orientation=${orientation}`);
        }
        
        const publisher = getOrCreatePublisher('/target_pose', 'geometry_msgs/msg/Twist');
        
        const now = new Date();
        
        //  ENHANCED: Generate goal ID with counter
        const finalGoalId = goalId || `goal_${Date.now()}_${++goalCounter}`;
        
        //  CUSTOM: Create Twist message with x, y, and angular velocity for target_pose
        // Since your ROS code expects Twist, we'll encode position in linear and orientation in angular
        const goalMsg = {
            linear: { 
                x: parseFloat(x),    // Target X position
                y: parseFloat(y),    // Target Y position
                z: 0.0               // Not used for 2D navigation
            },
            angular: { 
                x: 0.0,              // Not used
                y: 0.0,              // Not used
                z: parseFloat(orientation)  // Target orientation (yaw)
            }
        };
        
        publisher.publish(goalMsg);
        
        if (publishers['/target_pose']) {
            publishers['/target_pose'].messageCount++;
        }
        
        //  NEW: Store active goal for tracking
        activeGoals.set(finalGoalId, {
            goalId: finalGoalId,
            x: parseFloat(x),
            y: parseFloat(y),
            orientation: orientation,
            publishedAt: now.toISOString(),
            status: 'sent'
        });
        
        console.log(` Published goal to /target_pose [${publishers['/target_pose']?.messageCount || 1}]:`);
        console.log(`   Goal ID: ${finalGoalId}`);
        console.log(`   Position: x=${x}m, y=${y}m, orientation=${orientation}rad (${(orientation * 180 / Math.PI).toFixed(1)}°)`);
        console.log(`   Message Type: geometry_msgs/msg/Twist`);
        
        return { 
            success: true, 
            goalId: finalGoalId,
            x: parseFloat(x), 
            y: parseFloat(y), 
            orientation: orientation,
            messageType: 'geometry_msgs/msg/Twist',
            messageCount: publishers['/target_pose']?.messageCount || 1,
            timestamp: now.toISOString(),
            topic: '/target_pose'
        };
        
    } catch (error) {
        console.error(' Failed to publish goal to target_pose:', error);
        return { 
            success: false, 
            error: error.message,
            timestamp: new Date().toISOString(),
            topic: '/target_pose'
        };
    }
}

/**
 *  NEW: Publish to move_base_simple/goal as alternative
 */
function publishMoveBaseGoal(x, y, orientation = 0) {
    try {
        if (!isInitialized) {
            throw new Error('Publishers not initialized');
        }
        
        const publisher = getOrCreatePublisher('/move_base_simple/goal', 'geometry_msgs/msg/PoseStamped');
        
        const now = new Date();
        const yaw = orientation;
        const qz = Math.sin(yaw / 2);
        const qw = Math.cos(yaw / 2);
        
        const goalMsg = {
            header: {
                stamp: { 
                    sec: Math.floor(now.getTime() / 1000), 
                    nanosec: (now.getTime() % 1000) * 1000000 
                },
                frame_id: 'map'
            },
            pose: {
                position: { 
                    x: parseFloat(x), 
                    y: parseFloat(y), 
                    z: 0.0 
                },
                orientation: {
                    x: 0.0,
                    y: 0.0,
                    z: qz,
                    w: qw
                }
            }
        };
        
        publisher.publish(goalMsg);
        
        if (publishers['/move_base_simple/goal']) {
            publishers['/move_base_simple/goal'].messageCount++;
        }
        
        console.log(` Published goal to /move_base_simple/goal: (${x}, ${y}) @ ${orientation}rad`);
        
        return {
            success: true,
            x: parseFloat(x),
            y: parseFloat(y),
            orientation: orientation,
            topic: '/move_base_simple/goal',
            messageCount: publishers['/move_base_simple/goal']?.messageCount || 1,
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error(' Failed to publish move_base goal:', error);
        return {
            success: false,
            error: error.message,
            topic: '/move_base_simple/goal',
            timestamp: new Date().toISOString()
        };
    }
}

/**
 *  ENHANCED: Updated cancelCurrentGoal with better error handling
 */
function cancelCurrentGoal() {
    try {
        if (!isInitialized) {
            throw new Error('Publishers not initialized');
        }
        
        let cancelResults = [];
        
        // Method 1: Try to publish cancel to action cancel topic
        try {
            const cancelPublisher = getOrCreatePublisher('/navigate_to_pose/_action/cancel_goal', 'action_msgs/msg/CancelGoal');
            
            const cancelMsg = {
                goal_info: {
                    goal_id: {
                        uuid: new Array(16).fill(0) // Cancel all goals
                    },
                    stamp: {
                        sec: Math.floor(Date.now() / 1000),
                        nanosec: (Date.now() % 1000) * 1000000
                    }
                }
            };
            
            cancelPublisher.publish(cancelMsg);
            cancelResults.push({ method: 'action_cancel', success: true });
            console.log(' Published navigation goal cancellation to action topic');
            
        } catch (cancelError) {
            console.warn('️ Could not publish to cancel topic:', cancelError.message);
            cancelResults.push({ method: 'action_cancel', success: false, error: cancelError.message });
        }
        
        // Method 2: Emergency stop as additional safety
        try {
            const emergencyResult = emergencyStop();
            cancelResults.push({ method: 'emergency_stop', success: emergencyResult.success });
        } catch (emergencyError) {
            cancelResults.push({ method: 'emergency_stop', success: false, error: emergencyError.message });
        }
        
        //  NEW: Clear active goals
        const clearedGoals = activeGoals.size;
        activeGoals.clear();
        
        return {
            success: true,
            message: 'Navigation goal cancellation attempted',
            methods: cancelResults, //  ENHANCED
            clearedGoals: clearedGoals, //  NEW
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error(' Failed to cancel navigation goal:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

/**
 *  ENHANCED: Updated getNavigationStatus with more details
 */
function getNavigationStatus() {
    try {
        const stats = getPublisherStats();
        const targetPoseStats = stats.publishers['/target_pose'];
        const moveBaseStats = stats.publishers['/move_base_simple/goal']; //  NEW
        
        return {
            success: true,
            isInitialized: isInitialized,
            publishers: { //  ENHANCED
                target_pose: {
                    available: !!targetPoseStats,
                    messageCount: targetPoseStats?.messageCount || 0,
                    createdAt: targetPoseStats?.createdAt
                },
                move_base: { //  NEW
                    available: !!moveBaseStats,
                    messageCount: moveBaseStats?.messageCount || 0,
                    createdAt: moveBaseStats?.createdAt
                }
            },
            activeGoals: { //  NEW
                count: activeGoals.size,
                goals: Array.from(activeGoals.values())
            },
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error(' Error getting navigation status:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

/**
 *  ENHANCED: Updated emergencyStop with additional safety measures
 */
function emergencyStop() {
    try {
        const result = publishVelocity(0.0, 0.0);
        console.log(' EMERGENCY STOP ACTIVATED - All motion halted');
        
        //  NEW: Also try to publish to cmd_vel (standard topic)
        try {
            const cmdVelPublisher = getOrCreatePublisher('/cmd_vel', 'geometry_msgs/msg/Twist');
            const stopMsg = {
                linear: { x: 0.0, y: 0.0, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: 0.0 }
            };
            cmdVelPublisher.publish(stopMsg);
        } catch (cmdVelError) {
            console.warn('️ Could not publish to /cmd_vel:', cmdVelError.message);
        }
        
        return {
            ...result,
            type: 'emergency_stop',
            message: 'Emergency stop activated - all motion halted',
            allTopics: ['/cmd_vel_joystick', '/cmd_vel'] //  NEW
        };
        
    } catch (error) {
        console.error(' Emergency stop failed:', error);
        return {
            success: false,
            error: error.message,
            type: 'emergency_stop'
        };
    }
}

/**
 *  ENHANCED: Updated getPublisherStats with active goals info
 */
function getPublisherStats() {
    const stats = {
        totalPublishers: Object.keys(publishers).length,
        isInitialized: isInitialized,
        currentMaxSpeeds: getCurrentMaxSpeeds(),
        activeGoals: activeGoals.size, //  NEW
        publishers: {}
    };
    
    Object.entries(publishers).forEach(([topic, info]) => {
        stats.publishers[topic] = {
            messageType: info.messageType,
            messageCount: info.messageCount,
            createdAt: info.createdAt
        };
    });
    
    return stats;
}

/**
 *  ENHANCED: Updated testPublishing with target_pose test
 */
function testPublishing() {
    try {
        if (!isInitialized) {
            return { success: false, error: 'Publishers not initialized' };
        }
        
        // Test cmd_vel_joystick publishing
        const velResult = publishVelocity(0.0, 0.0);
        
        //  NEW: Test target_pose publishing (origin)
        const goalResult = publishGoalWithId(0.0, 0.0, 0.0, `test_${Date.now()}`);
        
        return {
            success: true,
            message: 'Publishing test completed',
            tests: {
                velocity: velResult.success,
                targetPose: goalResult.success, //  NEW
                publishersCreated: Object.keys(publishers).length,
                availableTopics: Object.keys(publishers),
                currentMaxSpeeds: getCurrentMaxSpeeds(),
                activeGoals: activeGoals.size //  NEW
            },
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

/**
 *  ENHANCED: Updated cleanup with active goals cleanup
 */
function cleanup() {
    Object.entries(publishers).forEach(([topicName, info]) => {
        try {
            info.publisher.destroy();
        } catch (e) {
            console.warn(`️ Error destroying publisher ${topicName}:`, e.message);
        }
    });
    publishers = {};
    activeGoals.clear(); //  NEW
    isInitialized = false;
    goalCounter = 0; //  NEW
    console.log(' Publishers cleaned up');
}

/**
 *  NEW: Goal management functions
 */
function getActiveGoals() {
    return Array.from(activeGoals.values());
}

function clearActiveGoals() {
    const count = activeGoals.size;
    activeGoals.clear();
    return { cleared: count };
}

/**
 *  COMPATIBILITY: Legacy publishGoal function wrapper
 */
function publishGoal(x, y, orientation = 0) {
    return publishGoalWithId(x, y, orientation);
}

//  ENHANCED: Updated module exports
module.exports = {
    initializePublishers,
    updateMaxSpeeds,      
    getCurrentMaxSpeeds,  
    publishVelocity,
    publishJoystick,
    publishGoal,                    //  FIXED: Legacy compatibility
    publishGoalWithId,              //  Main goal publishing function
    publishMoveBaseGoal,            //  NEW: Alternative goal publishing
    cancelCurrentGoal,      
    getNavigationStatus,     
    // startMapping,
    // stopMapping,
    emergencyStop,
    getPublisherStats,
    testPublishing,
    cleanup,
    
    //  NEW: Goal management functions
    getActiveGoals,
    clearActiveGoals
};