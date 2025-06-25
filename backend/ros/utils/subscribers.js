// ros/utils/subscribers.js - Enhanced with Device ID Support and Better Integration
const rclnodejs = require('rclnodejs');

let rosNode = null;
let subscribers = {};
let broadcastToSubscribers;
let currentDeviceId = 'piros'; // Default device ID

// Initialize with the node from ros_connection.js
function initializeSubscribers(node, deviceId = 'piros') {
    rosNode = node;
    currentDeviceId = deviceId;
    
    // Import WebSocket broadcasting
    try {
        const wsModule = require('../../websocket/clientConnection');
        broadcastToSubscribers = wsModule.broadcastToSubscribers;
    } catch (error) {
        console.warn('⚠️ WebSocket module not available, using fallback');
        broadcastToSubscribers = () => {}; // Fallback function
    }
    
    console.log(`✅ Subscribers initialized with ROS node for device: ${currentDeviceId}`);
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
 * Subscribe to /amcl_pose topic for live position tracking
 */
function subscribeToPosition() {
    const callback = (poseMsg) => {
        try {
            const positionData = {
                position: {
                    x: poseMsg.pose.pose.position.x || 0,
                    y: poseMsg.pose.pose.position.y || 0,
                    z: poseMsg.pose.pose.position.z || 0
                },
                orientation: {
                    x: poseMsg.pose.pose.orientation.x || 0,
                    y: poseMsg.pose.pose.orientation.y || 0,
                    z: poseMsg.pose.pose.orientation.z || 0,
                    w: poseMsg.pose.pose.orientation.w || 1,
                    yaw: quaternionToYaw(poseMsg.pose.pose.orientation)
                },
                timestamp: new Date().toISOString(),
                frame_id: poseMsg.header?.frame_id || 'map'
            };
            
            // Update global live data
            updateGlobalLiveData('position', positionData);
            
            // Broadcast position update with device ID
            broadcastToSubscribers('real_time_data', {
                type: 'position_update',
                data: positionData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });
            
            // Console log every 2 seconds to avoid spam
            if (Date.now() % 2000 < 100) {
                console.log(`📍 [${currentDeviceId}] Position: (${positionData.position.x.toFixed(2)}, ${positionData.position.y.toFixed(2)})`);
            }
            
        } catch (error) {
            console.error('❌ Error processing position message:', error);
        }
    };
    
    // Subscribe to /amcl_pose topic for position tracking
    try {
        getOrCreateSubscriber('/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', callback);
        console.log('✅ Subscribed to /amcl_pose topic for position tracking');
    } catch (error) {
        console.warn('⚠️ /amcl_pose topic not available, falling back to /diff_drive_controller/odom');
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
            
            // Broadcast odometry update with device ID
            broadcastToSubscribers('real_time_data', {
                type: 'odometry_update',
                data: processedOdom,
                deviceId: currentDeviceId,
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
            
            // Broadcast map update (throttled to avoid overwhelming) with device ID
            if (!global.lastMapBroadcast || Date.now() - global.lastMapBroadcast > 2000) {
                broadcastToSubscribers('real_time_data', {
                    type: 'map_update',
                    data: mapData,
                    deviceId: currentDeviceId,
                    timestamp: new Date().toISOString()
                });
                global.lastMapBroadcast = Date.now();
                
                console.log(`🗺️ [${currentDeviceId}] Map update: ${mapData.info.width}x${mapData.info.height} (${(mapData.data.length/1024).toFixed(1)}KB)`);
            }
            
        } catch (error) {
            console.error('❌ Error processing map:', error);
        }
    };
    
    getOrCreateSubscriber('/map', 'nav_msgs/msg/OccupancyGrid', callback);
}

/**
 * Subscribe to wheel_velocity feedback
 */
function subscribeToVelocityFeedback() {
    const callback = (msg) => {
        try {
            // msg.data is an array, e.g. [vx, vy, vz, wx, wy, wz]
            const arr = msg.data || [];
            const velocityData = {
                linear: {
                    x: arr[0] || 0,
                    y: arr[1] || 0,
                    z: arr[2] || 0
                },
                angular: {
                    x: arr[3] || 0,
                    y: arr[4] || 0,
                    z: arr[5] || 0
                },
                timestamp: new Date().toISOString()
            };

            updateGlobalLiveData('velocity_feedback', velocityData);

            broadcastToSubscribers('real_time_data', {
                type: 'velocity_feedback',
                data: velocityData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

        } catch (error) {
            console.error('❌ Error processing velocity feedback:', error);
        }
    };
    
    getOrCreateSubscriber('/wheel_velocity', 'std_msgs/msg/Float32MultiArray', callback);
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
                    deviceId: currentDeviceId,
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
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });
            
            console.log(`🔋 [${currentDeviceId}] Battery: ${batteryData.percentage?.toFixed(1) || 'N/A'}%`);
            
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
 * Subscribe to global costmap - ENHANCED for Flutter Integration
 */
function subscribeToGlobalCostmap() {
    const callback = (costmapMsg) => {
        try {
            const costmapData = {
                info: {
                    resolution: costmapMsg.info.resolution,
                    width: costmapMsg.info.width,
                    height: costmapMsg.info.height,
                    origin: {
                        position: {
                            x: costmapMsg.info.origin.position.x,
                            y: costmapMsg.info.origin.position.y,
                            z: costmapMsg.info.origin.position.z
                        },
                        orientation: {
                            x: costmapMsg.info.origin.orientation.x,
                            y: costmapMsg.info.origin.orientation.y,
                            z: costmapMsg.info.origin.orientation.z,
                            w: costmapMsg.info.origin.orientation.w
                        }
                    }
                },
                data: Array.from(costmapMsg.data),
                timestamp: new Date().toISOString(),
                frame_id: costmapMsg.header.frame_id
            };

            updateGlobalLiveData('global_costmap', costmapData);

            // Broadcast with device ID for Flutter app consumption
            broadcastToSubscribers('real_time_data', {
                type: 'global_costmap_update',
                data: costmapData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Optional: log occasionally
            if (!global.lastGlobalCostmapBroadcast || Date.now() - global.lastGlobalCostmapBroadcast > 3000) {
                console.log(`🌍 [${currentDeviceId}] Global costmap update: ${costmapData.info.width}x${costmapData.info.height}`);
                global.lastGlobalCostmapBroadcast = Date.now();
            }
        } catch (error) {
            console.error('❌ Error processing global costmap:', error);
        }
    };

    getOrCreateSubscriber('/global_costmap/costmap', 'nav_msgs/msg/OccupancyGrid', callback);
}

/**
 * Subscribe to local costmap - ENHANCED for Flutter Integration
 */
function subscribeToLocalCostmap() {
    const callback = (costmapMsg) => {
        try {
            const costmapData = {
                info: {
                    resolution: costmapMsg.info.resolution,
                    width: costmapMsg.info.width,
                    height: costmapMsg.info.height,
                    origin: {
                        position: {
                            x: costmapMsg.info.origin.position.x,
                            y: costmapMsg.info.origin.position.y,
                            z: costmapMsg.info.origin.position.z
                        },
                        orientation: {
                            x: costmapMsg.info.origin.orientation.x,
                            y: costmapMsg.info.origin.orientation.y,
                            z: costmapMsg.info.origin.orientation.z,
                            w: costmapMsg.info.origin.orientation.w
                        }
                    }
                },
                data: Array.from(costmapMsg.data),
                timestamp: new Date().toISOString(),
                frame_id: costmapMsg.header.frame_id
            };

            updateGlobalLiveData('local_costmap', costmapData);

            // Broadcast with device ID for Flutter app consumption
            broadcastToSubscribers('real_time_data', {
                type: 'local_costmap_update',
                data: costmapData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Optional: log occasionally (more frequent for local costmap)
            if (!global.lastLocalCostmapBroadcast || Date.now() - global.lastLocalCostmapBroadcast > 1500) {
                console.log(`🏠 [${currentDeviceId}] Local costmap update: ${costmapData.info.width}x${costmapData.info.height}`);
                global.lastLocalCostmapBroadcast = Date.now();
            }
        } catch (error) {
            console.error('❌ Error processing local costmap:', error);
        }
    };

    getOrCreateSubscriber('/local_costmap/costmap', 'nav_msgs/msg/OccupancyGrid', callback);
}

/**
 * Subscribe to all essential topics for AGV operation
 */
function subscribeToAllTopics() {
    console.log(`📥 Subscribing to all AGV topics for device: ${currentDeviceId}...`);
    
    // Core position tracking
    subscribeToPosition();
    
    // Live mapping
    subscribeToMap();
    
    // Control feedback
    subscribeToVelocityFeedback();
    
    // Joint states (wheel encoders)
    subscribeToJointStates();
    
    // Battery monitoring
    subscribeToBattery();
    
    // Costmaps for navigation visualization
    subscribeToGlobalCostmap();
    subscribeToLocalCostmap();
    
    console.log(`✅ All topic subscriptions initiated for device: ${currentDeviceId}`);
}

/**
 * Set device ID for multi-robot support
 */
function setDeviceId(deviceId) {
    currentDeviceId = deviceId;
    console.log(`🏷️ Device ID updated to: ${currentDeviceId}`);
}

// Utility functions
function quaternionToYaw(q) {
    return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

function updateGlobalLiveData(dataType, data) {
    if (!global.liveData) {
        global.liveData = {};
    }
    
    if (!global.liveData[currentDeviceId]) {
        global.liveData[currentDeviceId] = {};
    }
    
    global.liveData[currentDeviceId][dataType] = data;
    global.liveData[currentDeviceId].lastUpdate = new Date().toISOString();
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
    subscribeToGlobalCostmap,
    subscribeToLocalCostmap,
    setDeviceId,
    cleanup
};