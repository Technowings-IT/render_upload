// ros/utils/subscribers.js - Fixed with /pos topic and integrated with working ros_connection.js
const rclnodejs = require('rclnodejs');

let rosNode = null;
let subscribers = {};
let broadcastToSubscribers;

// Initialize with the node from ros_connection.js
function initializeSubscribers(node) {
    rosNode = node;
    
    // Import WebSocket broadcasting
    try {
        const wsModule = require('../../websocket/clientConnection');
        broadcastToSubscribers = wsModule.broadcastToSubscribers;
    } catch (error) {
        console.warn('⚠️ WebSocket module not available, using fallback');
        broadcastToSubscribers = () => {}; // Fallback function
    }
    
    console.log('✅ Subscribers initialized with ROS node');
}

// Create subscriber if not exists
function getOrCreateSubscriber(topicName, messageType, callback) {
    if (!subscribers[topicName]) {
        if (!rosNode) {
            throw new Error('ROS node not initialized. Call initializeSubscribers first.');
        }
        
        const subscriber = rosNode.createSubscription(
            messageType, 
            topicName,
            { depth: 10, reliability: 'reliable', durability: 'volatile' },
            callback
        );
        
        subscribers[topicName] = subscriber;
        console.log(`📥 Created subscriber for topic: ${topicName}`);
    }
    return subscribers[topicName];
}

/**
 * Subscribe to /pos topic for live position tracking (as requested by user)
 */
function subscribeToPosition() {
    const callback = (posMsg) => {
        try {
            const positionData = {
                position: {
                    x: posMsg.x || 0,
                    y: posMsg.y || 0,
                    z: posMsg.z || 0
                },
                timestamp: new Date().toISOString(),
                frame_id: posMsg.header?.frame_id || 'base_link'
            };
            
            // Update global live data
            updateGlobalLiveData('position', positionData);
            
            // Broadcast position update
            broadcastToSubscribers('real_time_data', {
                type: 'position_update',
                data: positionData,
                timestamp: new Date().toISOString()
            });
            
            // Console log every 2 seconds to avoid spam
            if (Date.now() % 2000 < 100) {
                console.log(`📍 Position: (${positionData.position.x.toFixed(2)}, ${positionData.position.y.toFixed(2)})`);
            }
            
        } catch (error) {
            console.error('❌ Error processing position message:', error);
        }
    };
    
    // Try to subscribe to /pos topic first (user's preference)
    try {
        getOrCreateSubscriber('/pos', 'geometry_msgs/msg/Point', callback);
        console.log('✅ Subscribed to /pos topic for position tracking');
    } catch (error) {
        console.warn('⚠️ /pos topic not available, falling back to /diff_drive_controller/odom');
        subscribeToOdometry(); // Fallback to odometry
    }
}

/**
 * Subscribe to odometry (fallback for position)
 */
function subscribeToOdometry() {
    const callback = (odomMsg) => {
        try {
            const processedOdom = {
                position: {
                    x: odomMsg.pose.pose.position.x,
                    y: odomMsg.pose.pose.position.y,
                    z: odomMsg.pose.pose.position.z
                },
                orientation: {
                    x: odomMsg.pose.pose.orientation.x,
                    y: odomMsg.pose.pose.orientation.y,
                    z: odomMsg.pose.pose.orientation.z,
                    w: odomMsg.pose.pose.orientation.w,
                    yaw: quaternionToYaw(odomMsg.pose.pose.orientation)
                },
                velocity: {
                    linear: {
                        x: odomMsg.twist.twist.linear.x,
                        y: odomMsg.twist.twist.linear.y,
                        z: odomMsg.twist.twist.linear.z
                    },
                    angular: {
                        x: odomMsg.twist.twist.angular.x,
                        y: odomMsg.twist.twist.angular.y,
                        z: odomMsg.twist.twist.angular.z
                    }
                },
                timestamp: new Date().toISOString()
            };
            
            // Update global live data
            updateGlobalLiveData('odometry', processedOdom);
            
            // Broadcast odometry update 
            broadcastToSubscribers('real_time_data', {
                type: 'odometry_update',
                data: processedOdom,
                timestamp: new Date().toISOString()
            });
            
        } catch (error) {
            console.error('❌ Error processing odometry:', error);
        }
    };
    
    getOrCreateSubscriber('/diff_drive_controller/odom', 'nav_msgs/msg/Odometry', callback);
}

/**
 * Subscribe to /map topic for live mapping
 */
function subscribeToMap() {
    const callback = (mapMsg) => {
        try {
            const mapData = {
                info: {
                    resolution: mapMsg.info.resolution,
                    width: mapMsg.info.width,
                    height: mapMsg.info.height,
                    origin: {
                        position: {
                            x: mapMsg.info.origin.position.x,
                            y: mapMsg.info.origin.position.y,
                            z: mapMsg.info.origin.position.z
                        },
                        orientation: {
                            x: mapMsg.info.origin.orientation.x,
                            y: mapMsg.info.origin.orientation.y,
                            z: mapMsg.info.origin.orientation.z,
                            w: mapMsg.info.origin.orientation.w
                        }
                    }
                },
                data: Array.from(mapMsg.data),
                timestamp: new Date().toISOString(),
                frame_id: mapMsg.header.frame_id
            };
            
            // Update global live data
            updateGlobalLiveData('map', mapData);
            
            // Broadcast map update (throttled to avoid overwhelming)
            if (!global.lastMapBroadcast || Date.now() - global.lastMapBroadcast > 2000) {
                broadcastToSubscribers('real_time_data', {
                    type: 'map_update',
                    data: mapData,
                    timestamp: new Date().toISOString()
                });
                global.lastMapBroadcast = Date.now();
                
                console.log(`🗺️ Map update: ${mapData.info.width}x${mapData.info.height} (${(mapData.data.length/1024).toFixed(1)}KB)`);
            }
            
        } catch (error) {
            console.error('❌ Error processing map:', error);
        }
    };
    
    getOrCreateSubscriber('/map', 'nav_msgs/msg/OccupancyGrid', callback);
}

/**
 * Subscribe to cmd_vel feedback
 */
function subscribeToVelocityFeedback() {
    const callback = (twistMsg) => {
        try {
            const velocityData = {
                linear: {
                    x: twistMsg.linear.x,
                    y: twistMsg.linear.y,
                    z: twistMsg.linear.z
                },
                angular: {
                    x: twistMsg.angular.x,
                    y: twistMsg.angular.y,
                    z: twistMsg.angular.z
                },
                timestamp: new Date().toISOString()
            };
            
            updateGlobalLiveData('velocity_feedback', velocityData);
            
            broadcastToSubscribers('real_time_data', {
                type: 'velocity_feedback',
                data: velocityData,
                timestamp: new Date().toISOString()
            });
            
        } catch (error) {
            console.error('❌ Error processing velocity feedback:', error);
        }
    };
    
    getOrCreateSubscriber('/cmd_vel', 'geometry_msgs/msg/Twist', callback);
}

/**
 * Subscribe to joint states for wheel encoders
 */
function subscribeToJointStates() {
    const callback = (jointMsg) => {
        try {
            const jointData = {
                joint_names: jointMsg.name,
                positions: jointMsg.position,
                velocities: jointMsg.velocity,
                efforts: jointMsg.effort,
                timestamp: new Date().toISOString()
            };
            
            updateGlobalLiveData('joint_states', jointData);
            
            // Broadcast less frequently to reduce noise
            if (Date.now() % 1000 < 100) {
                broadcastToSubscribers('real_time_data', {
                    type: 'joint_states_update',
                    data: jointData,
                    timestamp: new Date().toISOString()
                });
            }
            
        } catch (error) {
            console.error('❌ Error processing joint states:', error);
        }
    };
    
    getOrCreateSubscriber('/joint_states', 'sensor_msgs/msg/JointState', callback);
}

/**
 * Subscribe to battery state
 */
function subscribeToBattery() {
    const callback = (batteryMsg) => {
        try {
            const batteryData = {
                voltage: batteryMsg.voltage,
                current: batteryMsg.current,
                charge: batteryMsg.charge,
                capacity: batteryMsg.capacity,
                design_capacity: batteryMsg.design_capacity,
                percentage: batteryMsg.percentage,
                power_supply_status: batteryMsg.power_supply_status,
                power_supply_health: batteryMsg.power_supply_health,
                power_supply_technology: batteryMsg.power_supply_technology,
                present: batteryMsg.present,
                timestamp: new Date().toISOString()
            };
            
            updateGlobalLiveData('battery', batteryData);
            
            broadcastToSubscribers('real_time_data', {
                type: 'battery_update',
                data: batteryData,
                timestamp: new Date().toISOString()
            });
            
            console.log(`🔋 Battery: ${batteryData.percentage?.toFixed(1) || 'N/A'}%`);
            
        } catch (error) {
            console.error('❌ Error processing battery data:', error);
        }
    };
    
    try {
        getOrCreateSubscriber('/battery_state', 'sensor_msgs/msg/BatteryState', callback);
    } catch (error) {
        console.warn('⚠️ Battery topic not available');
    }
}

/**
 * Subscribe to all essential topics for AGV operation
 */
function subscribeToAllTopics() {
    console.log('📥 Subscribing to all AGV topics...');
    
    // Core position tracking (user's preference for /pos)
    subscribeToPosition();
    
    // Live mapping
    subscribeToMap();
    
    // Control feedback
    subscribeToVelocityFeedback();
    
    // Joint states (wheel encoders)
    subscribeToJointStates();
    
    // Battery monitoring
    subscribeToBattery();
    
    console.log('✅ All topic subscriptions initiated');
}

// Utility functions
function quaternionToYaw(q) {
    return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

function updateGlobalLiveData(dataType, data) {
    if (!global.liveData) {
        global.liveData = {};
    }
    
    // For single AGV setup, use 'agv_01' as default device ID
    const deviceId = 'agv_01';
    
    if (!global.liveData[deviceId]) {
        global.liveData[deviceId] = {};
    }
    
    global.liveData[deviceId][dataType] = data;
    global.liveData[deviceId].lastUpdate = new Date().toISOString();
}

/**
 * Cleanup all subscribers
 */
function cleanup() {
    Object.keys(subscribers).forEach(topicName => {
        try {
            subscribers[topicName].destroy();
        } catch (e) {
            // Ignore cleanup errors
        }
    });
    subscribers = {};
    console.log('🧹 Subscribers cleaned up');
}

module.exports = {
    initializeSubscribers,
    subscribeToAllTopics,
    subscribeToPosition,
    subscribeToOdometry,
    subscribeToMap,
    subscribeToVelocityFeedback,
    subscribeToJointStates,
    subscribeToBattery,
    cleanup
};