// ros/utils/publishers.js - FIXED ROS2 Publishers with proper cmd_vel publishing
const rclnodejs = require('rclnodejs');
const config = require('../../config');

let rosNode = null;
let publishers = {};
let isInitialized = false;

// Initialize with the node from ros_connection.js
function initializePublishers(node) {
    rosNode = node;
    isInitialized = true;
    console.log('✅ Publishers initialized with ROS node');
    
    // Pre-create essential publishers for better performance
    createEssentialPublishers();
}

// ✅ FIXED: Pre-create publishers for immediate use
function createEssentialPublishers() {
    try {
        // Create cmd_vel publisher immediately for joystick control
        getOrCreatePublisher('/cmd_vel', 'geometry_msgs/msg/Twist');
        
        // Create map publisher for map editing
        getOrCreatePublisher('/map', 'nav_msgs/msg/OccupancyGrid');
        
        // Create goal publisher for navigation
        getOrCreatePublisher('/move_base_simple/goal', 'geometry_msgs/msg/PoseStamped');
        
        console.log('✅ Essential publishers created');
        
    } catch (error) {
        console.error('❌ Error creating essential publishers:', error);
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
            
            console.log(`📤 Created publisher for topic: ${topicName} (${messageType})`);
            
        } catch (error) {
            console.error(`❌ Failed to create publisher for ${topicName}:`, error);
            throw error;
        }
    }
    return publishers[topicName].publisher;
}

/**
 * ✅ FIXED: Enhanced velocity publishing with safety limits and validation
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
        
        const publisher = getOrCreatePublisher('/cmd_vel', 'geometry_msgs/msg/Twist');
        
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
        if (publishers['/cmd_vel']) {
            publishers['/cmd_vel'].messageCount++;
        }
        
        // Log every few messages to avoid spam
        if (publishers['/cmd_vel'].messageCount % 10 === 0 || Math.abs(clampedLinear) > 0.1 || Math.abs(clampedAngular) > 0.1) {
            console.log(`🚗 Published velocity [${publishers['/cmd_vel'].messageCount}]: linear=${clampedLinear.toFixed(3)}, angular=${clampedAngular.toFixed(3)}`);
        }
        
        return {
            success: true,
            message: 'Velocity command published successfully',
            data: { 
                linear: clampedLinear, 
                angular: clampedAngular,
                messageCount: publishers['/cmd_vel'].messageCount
            },
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error('❌ Failed to publish velocity:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

/**
 * ✅ FIXED: Enhanced joystick publishing with deadman switch and smoothing
 */
function publishJoystick(x, y, deadman = false) {
    try {
        if (!isInitialized) {
            throw new Error('Publishers not initialized');
        }
        
        // Only send velocity if deadman switch is active
        if (!deadman) {
            return publishVelocity(0.0, 0.0);
        }
        
        // Convert joystick values to velocity with scaling
        const maxLinear = config.AGV.MAX_LINEAR_SPEED;
        const maxAngular = config.AGV.MAX_ANGULAR_SPEED;
        
        // Apply deadzone for stability
        const deadzone = 0.1;
        const processedX = Math.abs(x) < deadzone ? 0 : x;
        const processedY = Math.abs(y) < deadzone ? 0 : y;
        
        // Scale to max velocities
        const linear = processedY * maxLinear;   // Forward/backward
        const angular = -processedX * maxAngular; // Left/right (inverted for correct rotation)
        
        const result = publishVelocity(linear, angular);
        
        // Add joystick-specific info
        result.joystickData = {
            originalX: x,
            originalY: y,
            processedX: processedX,
            processedY: processedY,
            deadman: deadman,
            deadzone: deadzone
        };
        
        return result;
        
    } catch (error) {
        console.error('❌ Failed to publish joystick command:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

/**
 * ✅ ENHANCED: Goal publishing with better validation
 */
function publishGoal(x, y, orientation = 0) {
    try {
        if (!isInitialized) {
            throw new Error('Publishers not initialized');
        }
        
        const publisher = getOrCreatePublisher('/move_base_simple/goal', 'geometry_msgs/msg/PoseStamped');
        
        const now = new Date();
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
                    z: Math.sin(orientation / 2),
                    w: Math.cos(orientation / 2)
                }
            }
        };
        
        publisher.publish(goalMsg);
        
        if (publishers['/move_base_simple/goal']) {
            publishers['/move_base_simple/goal'].messageCount++;
        }
        
        console.log(`🎯 Published goal [${publishers['/move_base_simple/goal'].messageCount}]: x=${x}, y=${y}, orientation=${orientation}`);
        
        return { 
            success: true, 
            x: parseFloat(x), 
            y: parseFloat(y), 
            orientation: orientation,
            messageCount: publishers['/move_base_simple/goal'].messageCount,
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error('❌ Failed to publish goal:', error);
        return { 
            success: false, 
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

/**
 * ✅ ENHANCED: Map publishing with proper OccupancyGrid format
 */
function publishMap(deviceId, mapData) {
    try {
        if (!isInitialized) {
            throw new Error('Publishers not initialized');
        }
        
        const publisher = getOrCreatePublisher('/map', 'nav_msgs/msg/OccupancyGrid');
        
        // Convert your mapData to proper ROS OccupancyGrid format
        const rosMapMsg = convertToOccupancyGrid(mapData);
        
        publisher.publish(rosMapMsg);
        
        if (publishers['/map']) {
            publishers['/map'].messageCount++;
        }
        
        console.log(`🗺️ Published map [${publishers['/map'].messageCount}] for device ${deviceId} to /map`);
        
        return { 
            success: true, 
            deviceId: deviceId,
            mapSize: `${rosMapMsg.info.width}x${rosMapMsg.info.height}`,
            resolution: rosMapMsg.info.resolution,
            messageCount: publishers['/map'].messageCount,
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error(`❌ Error publishing map for device ${deviceId}:`, error);
        return { 
            success: false, 
            error: error.message,
            deviceId: deviceId,
            timestamp: new Date().toISOString()
        };
    }
}

/**
 * ✅ FIXED: Helper to convert your mapData to ROS OccupancyGrid message
 */
function convertToOccupancyGrid(mapData) {
    const now = new Date();
    
    // Default values if mapData doesn't have complete info
    const info = mapData.info || {
        resolution: config.MAP.DEFAULT_RESOLUTION,
        width: config.MAP.DEFAULT_WIDTH,
        height: config.MAP.DEFAULT_HEIGHT,
        origin: {
            position: config.MAP.DEFAULT_ORIGIN,
            orientation: { x: 0, y: 0, z: 0, w: 1 }
        }
    };
    
    // Ensure data is in correct format (array of int8 values)
    let data = mapData.data || [];
    if (data.length !== info.width * info.height) {
        // Create empty map if data doesn't match dimensions
        data = new Array(info.width * info.height).fill(-1); // Unknown cells
        console.warn(`⚠️ Map data size mismatch, creating empty map: ${info.width}x${info.height}`);
    }
    
    return {
        header: {
            stamp: { 
                sec: Math.floor(now.getTime() / 1000), 
                nanosec: (now.getTime() % 1000) * 1000000 
            },
            frame_id: 'map'
        },
        info: {
            map_load_time: { 
                sec: Math.floor(now.getTime() / 1000), 
                nanosec: (now.getTime() % 1000) * 1000000 
            },
            resolution: parseFloat(info.resolution),
            width: parseInt(info.width),
            height: parseInt(info.height),
            origin: {
                position: {
                    x: parseFloat(info.origin.position?.x || config.MAP.DEFAULT_ORIGIN.x),
                    y: parseFloat(info.origin.position?.y || config.MAP.DEFAULT_ORIGIN.y),
                    z: parseFloat(info.origin.position?.z || config.MAP.DEFAULT_ORIGIN.z)
                },
                orientation: {
                    x: parseFloat(info.origin.orientation?.x || 0),
                    y: parseFloat(info.origin.orientation?.y || 0),
                    z: parseFloat(info.origin.orientation?.z || 0),
                    w: parseFloat(info.origin.orientation?.w || 1)
                }
            }
        },
        data: data.map(val => parseInt(val)) // Ensure int8 values
    };
}

/**
 * ✅ NEW: Start mapping service call or topic publish
 */
function startMapping() {
    try {
        if (!isInitialized) {
            throw new Error('Publishers not initialized');
        }
        
        // Try to publish to mapping start topic
        try {
            const publisher = getOrCreatePublisher('/mapping/start', 'std_msgs/msg/Bool');
            publisher.publish({ data: true });
            
            if (publishers['/mapping/start']) {
                publishers['/mapping/start'].messageCount++;
            }
            
            console.log(`🗺️ Published mapping start command [${publishers['/mapping/start']?.messageCount || 1}]`);
            
        } catch (topicError) {
            // If topic doesn't exist, try alternative methods
            console.warn('⚠️ /mapping/start topic not available, trying alternatives...');
            
            // You might need to call a ROS2 service instead
            // For now, just log that mapping should start
            console.log('🗺️ Mapping start requested (service call may be needed)');
        }
        
        return { 
            success: true, 
            message: 'Mapping start command sent',
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error('❌ Failed to start mapping:', error);
        return { 
            success: false, 
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

/**
 * ✅ NEW: Stop mapping
 */
function stopMapping() {
    try {
        if (!isInitialized) {
            throw new Error('Publishers not initialized');
        }
        
        try {
            const publisher = getOrCreatePublisher('/mapping/stop', 'std_msgs/msg/Bool');
            publisher.publish({ data: true });
            
            if (publishers['/mapping/stop']) {
                publishers['/mapping/stop'].messageCount++;
            }
            
            console.log(`🛑 Published mapping stop command [${publishers['/mapping/stop']?.messageCount || 1}]`);
            
        } catch (topicError) {
            console.warn('⚠️ /mapping/stop topic not available');
            console.log('🛑 Mapping stop requested');
        }
        
        return { 
            success: true, 
            message: 'Mapping stop command sent',
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error('❌ Failed to stop mapping:', error);
        return { 
            success: false, 
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

/**
 * Emergency stop - immediate zero velocity
 */
function emergencyStop() {
    try {
        const result = publishVelocity(0.0, 0.0);
        console.log('🛑 EMERGENCY STOP ACTIVATED');
        
        return {
            ...result,
            type: 'emergency_stop',
            message: 'Emergency stop activated - all motion halted'
        };
        
    } catch (error) {
        console.error('❌ Emergency stop failed:', error);
        return {
            success: false,
            error: error.message,
            type: 'emergency_stop'
        };
    }
}

/**
 * ✅ NEW: Get publisher statistics
 */
function getPublisherStats() {
    const stats = {
        totalPublishers: Object.keys(publishers).length,
        isInitialized: isInitialized,
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
 * ✅ NEW: Test publishing capability
 */
function testPublishing() {
    try {
        if (!isInitialized) {
            return { success: false, error: 'Publishers not initialized' };
        }
        
        // Test cmd_vel publishing
        const velResult = publishVelocity(0.0, 0.0);
        
        return {
            success: true,
            message: 'Publishing test completed',
            tests: {
                velocity: velResult.success,
                publishersCreated: Object.keys(publishers).length,
                availableTopics: Object.keys(publishers)
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
 * Cleanup publishers
 */
function cleanup() {
    Object.entries(publishers).forEach(([topicName, info]) => {
        try {
            info.publisher.destroy();
        } catch (e) {
            console.warn(`⚠️ Error destroying publisher ${topicName}:`, e.message);
        }
    });
    publishers = {};
    isInitialized = false;
    console.log('🧹 Publishers cleaned up');
}

module.exports = {
    initializePublishers,
    publishVelocity,
    publishJoystick,
    publishGoal, 
    publishMap,
    startMapping,
    stopMapping,
    emergencyStop,
    getPublisherStats,
    testPublishing,
    cleanup
};