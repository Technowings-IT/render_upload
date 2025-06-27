//ros/utils/subscribers.js - FIXED ROS2 Subscribers with Proper Error Handling
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
        broadcastToSubscribers = () => { }; // Fallback function
    }

    console.log(`✅ Subscribers initialized with ROS node for device: ${currentDeviceId}`);

    // List available topics for debugging
    setTimeout(() => {
        listAvailableTopics();
    }, 2000);
}

// List available topics for debugging
async function listAvailableTopics() {
    try {
        if (rosNode) {
            const topics = rosNode.getTopicNamesAndTypes();
            console.log('🔍 Available ROS topics:');
            topics.forEach(topic => {
                console.log(`   ${topic.name} [${topic.types.join(', ')}]`);
            });

            // Check for specific topics we need
            const requiredTopics = [
                '/amcl_pose',
                '/global_costmap/costmap',
                '/local_costmap/costmap',
                '/odom',
                // '/joint_states'
            ];

            console.log('🎯 Checking for required topics:');
            requiredTopics.forEach(topicName => {
                const found = topics.find(t => t.name === topicName);
                if (found) {
                    console.log(`   ✅ ${topicName} [${found.types.join(', ')}]`);
                } else {
                    console.log(`   ❌ ${topicName} - NOT FOUND`);
                }
            });
        }
    } catch (error) {
        console.error('❌ Error listing topics:', error);
    }
}

// ✅ FIXED: Create subscriber with proper error handling
function getOrCreateSubscriber(topicName, messageType, callback) {
    if (!subscribers[topicName]) {
        if (!rosNode) {
            throw new Error('ROS node not initialized. Call initializeSubscribers first.');
        }

        try {
            const subscriber = rosNode.createSubscription(
                messageType,
                topicName,
                {
                    depth: 10,
                    reliability: 'reliable',
                    durability: 'volatile',
                    history: 'keep_last'
                },
                callback
            );

            subscribers[topicName] = subscriber;
            console.log(`📥 ✅ Created subscriber for topic: ${topicName} [${messageType}]`);

            // ✅ FIXED: Add proper safety checks before calling isClosed()
            setTimeout(() => {
                try {
                    if (subscriber && typeof subscriber.isClosed === 'function' && !subscriber.isClosed()) {
                        console.log(`🔗 Confirmed subscription active: ${topicName}`);
                    } else if (subscriber && typeof subscriber.isClosed !== 'function') {
                        console.log(`🔗 Subscription created (isClosed method not available): ${topicName}`);
                    } else {
                        console.log(`❌ Subscription failed: ${topicName}`);
                    }
                } catch (verifyError) {
                    console.log(`⚠️ Could not verify subscription for ${topicName}: ${verifyError.message}`);
                }
            }, 1000);

        } catch (error) {
            console.error(`❌ Failed to create subscriber for ${topicName}:`, error);
            throw error;
        }
    }
    return subscribers[topicName];
}

/**
 * Subscribe to /amcl_pose topic for live position tracking (PRIORITY)
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
                covariance: {
                    pose: poseMsg.pose.covariance || [],
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

            // Console log every 3 seconds to avoid spam
            if (Date.now() % 3000 < 100) {
                console.log(`📍 [${currentDeviceId}] AMCL Position: (${positionData.position.x.toFixed(2)}, ${positionData.position.y.toFixed(2)}) θ=${positionData.orientation.yaw.toFixed(2)}`);
            }

        } catch (error) {
            console.error('❌ Error processing AMCL pose message:', error);
        }
    };

    // Try different possible AMCL topic names
    const amclTopics = [
        '/amcl_pose',
        '/amcl/pose',
        '/localization/amcl_pose',
        '/robot_pose'
    ];

    for (const topicName of amclTopics) {
        try {
            getOrCreateSubscriber(topicName, 'geometry_msgs/msg/PoseWithCovarianceStamped', callback);
            console.log(`✅ Successfully subscribed to AMCL topic: ${topicName}`);
            return; // Success, exit function
        } catch (error) {
            console.warn(`⚠️ Failed to subscribe to ${topicName}, trying next...`);
        }
    }

    // If all AMCL topics fail, fallback to odometry
    console.warn('⚠️ All AMCL topics failed, falling back to odometry');
    subscribeToOdometry();
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
                timestamp: new Date().toISOString(),
                frame_id: odomMsg.header?.frame_id || 'odom'
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

            // Console log every 5 seconds for odometry
            if (Date.now() % 5000 < 100) {
                console.log(`🔄 [${currentDeviceId}] Odom Position: (${processedOdom.position.x.toFixed(2)}, ${processedOdom.position.y.toFixed(2)})`);
            }

        } catch (error) {
            console.error('❌ Error processing odometry:', error);
        }
    };

    // Try different possible odometry topic names
    const odomTopics = [
        '/odom',
        '/odometry/local',
        '/diff_drive_controller/odom',
        '/robot/odom'
    ];

    for (const topicName of odomTopics) {
        try {
            getOrCreateSubscriber(topicName, 'nav_msgs/msg/Odometry', callback);
            console.log(`✅ Successfully subscribed to odometry topic: ${topicName}`);
            return;
        } catch (error) {
            console.warn(`⚠️ Failed to subscribe to ${topicName}, trying next...`);
        }
    }

    console.error('❌ Failed to subscribe to any odometry topic');
}

/**
 * Subscribe to global costmap - ENHANCED with multiple topic attempts
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
                frame_id: costmapMsg.header?.frame_id || 'map'
            };

            updateGlobalLiveData('global_costmap', costmapData);

            // Broadcast with device ID for Flutter app consumption
            broadcastToSubscribers('real_time_data', {
                type: 'global_costmap_update',
                data: costmapData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Log occasionally with more details
            if (!global.lastGlobalCostmapBroadcast || Date.now() - global.lastGlobalCostmapBroadcast > 3000) {
                console.log(`🌍 [${currentDeviceId}] Global costmap: ${costmapData.info.width}x${costmapData.info.height} @ ${costmapData.info.resolution.toFixed(3)}m/px`);
                global.lastGlobalCostmapBroadcast = Date.now();
            }
        } catch (error) {
            console.error('❌ Error processing global costmap:', error);
        }
    };

    // Try different possible global costmap topic names
    const globalCostmapTopics = [
        '/global_costmap/costmap',
        '/move_base/global_costmap/costmap',
        '/planner/global_costmap/costmap',
        '/navigation/global_costmap/costmap'
    ];

    for (const topicName of globalCostmapTopics) {
        try {
            getOrCreateSubscriber(topicName, 'nav_msgs/msg/OccupancyGrid', callback);
            console.log(`✅ Successfully subscribed to global costmap: ${topicName}`);
            return;
        } catch (error) {
            console.warn(`⚠️ Failed to subscribe to ${topicName}, trying next...`);
        }
    }

    console.warn('⚠️ No global costmap topics found');
}
/**
 * Subscribe to /map topic (for occupancy grid)
 */
function subscribeToMap() {
    const mapTopics = [
        '/map',
        '/navigation/map',
        '/slam_toolbox/map'
    ];
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
                frame_id: mapMsg.header?.frame_id || 'map'
            };

            updateGlobalLiveData('map', mapData);

            // Broadcast with device ID for Flutter app consumption
            broadcastToSubscribers('real_time_data', {
                type: 'map_update',
                data: mapData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Log occasionally
            if (!global.lastMapBroadcast || Date.now() - global.lastMapBroadcast > 3000) {
                console.log(`🗺️ [${currentDeviceId}] Map: ${mapData.info.width}x${mapData.info.height} @ ${mapData.info.resolution.toFixed(3)}m/px`);
                global.lastMapBroadcast = Date.now();
            }
        } catch (error) {
            console.error('❌ Error processing map:', error);
        }
    };
    for (const topicName of mapTopics) {
        try {
            getOrCreateSubscriber('/map', 'nav_msgs/msg/OccupancyGrid', callback);
            console.log(`✅ Successfully subscribed to map: ${topicName}`);
            return;
        } catch (error) {
            console.warn(`⚠️ Failed to subscribe to ${topicName}, trying next...`);
        }
    }

    console.warn('⚠️ No map topics found');
}
/**
 * Subscribe to local costmap - ENHANCED with multiple topic attempts
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
                frame_id: costmapMsg.header?.frame_id || 'base_link'
            };

            updateGlobalLiveData('local_costmap', costmapData);

            // Broadcast with device ID for Flutter app consumption
            broadcastToSubscribers('real_time_data', {
                type: 'local_costmap_update',
                data: costmapData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Log occasionally (more frequent for local costmap)
            if (!global.lastLocalCostmapBroadcast || Date.now() - global.lastLocalCostmapBroadcast > 2000) {
                console.log(`🏠 [${currentDeviceId}] Local costmap: ${costmapData.info.width}x${costmapData.info.height} @ ${costmapData.info.resolution.toFixed(3)}m/px`);
                global.lastLocalCostmapBroadcast = Date.now();
            }
        } catch (error) {
            console.error('❌ Error processing local costmap:', error);
        }
    };

    // Try different possible local costmap topic names
    const localCostmapTopics = [
        '/local_costmap/costmap',
        '/move_base/local_costmap/costmap',
        '/controller/local_costmap/costmap',
        '/navigation/local_costmap/costmap'
    ];

    for (const topicName of localCostmapTopics) {
        try {
            getOrCreateSubscriber(topicName, 'nav_msgs/msg/OccupancyGrid', callback);
            console.log(`✅ Successfully subscribed to local costmap: ${topicName}`);
            return;
        } catch (error) {
            console.warn(`⚠️ Failed to subscribe to ${topicName}, trying next...`);
        }
    }

    console.warn('⚠️ No local costmap topics found');
}

/**
 * Subscribe to velocity feedback (cmd_vel_out or similar)
 */
function subscribeToVelocityFeedback() {
    const callback = (twistMsg) => {
        try {
            const velocityData = {
                linear: {
                    x: twistMsg.linear.x || 0,
                    y: twistMsg.linear.y || 0,
                    z: twistMsg.linear.z || 0
                },
                angular: {
                    x: twistMsg.angular.x || 0,
                    y: twistMsg.angular.y || 0,
                    z: twistMsg.angular.z || 0
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

    // Try different velocity feedback topics
    const velocityTopics = [
        '/wheel_velocity',
        '/cmd_vel',
        '/velocity_feedback',
        '/diff_drive_controller/wheel_velocity'
    ];

    for (const topicName of velocityTopics) {
        try {
            getOrCreateSubscriber(topicName, 'geometry_msgs/msg/Twist', callback);
            console.log(`✅ Successfully subscribed to velocity feedback: ${topicName}`);
            return;
        } catch (error) {
            console.warn(`⚠️ Failed to subscribe to ${topicName}, trying next...`);
        }
    }

    console.warn('⚠️ No velocity feedback topics found');
}

/**
 * Subscribe to joint states for wheel encoders
 */
// function subscribeToJointStates() {
//     const callback = (jointMsg) => {
//         try {
//             const jointData = {
//                 joint_names: jointMsg.name || [],
//                 positions: jointMsg.position || [],
//                 velocities: jointMsg.velocity || [],
//                 efforts: jointMsg.effort || [],
//                 timestamp: new Date().toISOString()
//             };

//             updateGlobalLiveData('joint_states', jointData);

//             // Broadcast less frequently to reduce noise
//             if (Date.now() % 2000 < 100) {
//                 broadcastToSubscribers('real_time_data', {
//                     type: 'joint_states_update',
//                     data: jointData,
//                     deviceId: currentDeviceId,
//                     timestamp: new Date().toISOString()
//                 });
//             }

//         } catch (error) {
//             console.error('❌ Error processing joint states:', error);
//         }
//     };

//     try {
//         getOrCreateSubscriber('/joint_states', 'sensor_msgs/msg/JointState', callback);
//         console.log('✅ Successfully subscribed to joint states');
//     } catch (error) {
//         console.warn('⚠️ Joint states topic not available');
//     }
// }

/**
 * Subscribe to battery state
 */
function subscribeToBattery() {
    const callback = (batteryMsg) => {
        try {
            const batteryData = {
                voltage: batteryMsg.voltage || 0,
                current: batteryMsg.current || 0,
                charge: batteryMsg.charge || 0,
                capacity: batteryMsg.capacity || 0,
                design_capacity: batteryMsg.design_capacity || 0,
                percentage: batteryMsg.percentage || 0,
                power_supply_status: batteryMsg.power_supply_status || 0,
                power_supply_health: batteryMsg.power_supply_health || 0,
                power_supply_technology: batteryMsg.power_supply_technology || 0,
                present: batteryMsg.present || false,
                timestamp: new Date().toISOString()
            };

            updateGlobalLiveData('battery', batteryData);

            broadcastToSubscribers('real_time_data', {
                type: 'battery_update',
                data: batteryData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Log every 10 seconds
            if (Date.now() % 10000 < 100) {
                console.log(`🔋 [${currentDeviceId}] Battery: ${batteryData.percentage?.toFixed(1) || 'N/A'}% (${batteryData.voltage?.toFixed(1) || 'N/A'}V)`);
            }

        } catch (error) {
            console.error('❌ Error processing battery data:', error);
        }
    };

    const batteryTopics = [
        '/battery_state',
        '/power/battery_state',
        '/robot/battery'
    ];

    for (const topicName of batteryTopics) {
        try {
            getOrCreateSubscriber(topicName, 'sensor_msgs/msg/BatteryState', callback);
            console.log(`✅ Successfully subscribed to battery: ${topicName}`);
            return;
        } catch (error) {
            console.warn(`⚠️ Failed to subscribe to ${topicName}, trying next...`);
        }
    }

    console.warn('⚠️ No battery topics found');
}

/**
 * Subscribe to essential topics for AGV operation (REMOVED /map as requested)
 */
function subscribeToAllTopics() {
    console.log(`📥 Subscribing to essential AGV topics for device: ${currentDeviceId}...`);

    // Core position tracking (PRIORITY)
    console.log('🎯 Priority 1: Position tracking...');
    subscribeToPosition();

    // Costmaps for navigation visualization (PRIORITY)
    console.log('🎯 Priority 2: Costmaps...');
    subscribeToGlobalCostmap();
    subscribeToLocalCostmap();

    // Control feedback
    console.log('🎯 Priority 3: Control feedback...');
    subscribeToVelocityFeedback();

    // Joint states (wheel encoders) 
    console.log('🎯 Priority 4: Map...');
    subscribeToMap();

    // Battery monitoring
    console.log('🎯 Priority 5: Battery...');
    subscribeToBattery();

    console.log(`✅ All topic subscriptions attempted for device: ${currentDeviceId}`);

    // List active subscriptions after 3 seconds
    setTimeout(() => {
        console.log('📊 Active subscriptions:');
        Object.keys(subscribers).forEach(topic => {
            const sub = subscribers[topic];
            // ✅ FIXED: Added safety check here too
            try {
                if (sub && typeof sub.isClosed === 'function' && !sub.isClosed()) {
                    console.log(`   ✅ ${topic}`);
                } else if (sub && typeof sub.isClosed !== 'function') {
                    console.log(`   ✅ ${topic} (isClosed method not available)`);
                } else {
                    console.log(`   ❌ ${topic} (failed/closed)`);
                }
            } catch (statusError) {
                console.log(`   ⚠️ ${topic} (status unknown: ${statusError.message})`);
            }
        });
    }, 3000);
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
 * Get subscription status for debugging
 */
function getSubscriptionStatus() {
    const status = {};
    Object.keys(subscribers).forEach(topic => {
        const sub = subscribers[topic];
        try {
            // ✅ FIXED: Added safety check for isClosed method
            if (sub && typeof sub.isClosed === 'function') {
                status[topic] = sub.isClosed() ? 'inactive' : 'active';
            } else if (sub) {
                status[topic] = 'active (method unavailable)';
            } else {
                status[topic] = 'inactive';
            }
        } catch (error) {
            status[topic] = `error: ${error.message}`;
        }
    });
    return status;
}

/**
 * Cleanup all subscribers
 */
function cleanup() {
    console.log('🧹 Cleaning up subscribers...');
    Object.keys(subscribers).forEach(topicName => {
        try {
            const sub = subscribers[topicName];
            // ✅ FIXED: Added safety check for destroy method
            if (sub && typeof sub.destroy === 'function') {
                if (typeof sub.isClosed === 'function' && !sub.isClosed()) {
                    sub.destroy();
                    console.log(`   ✅ Cleaned up: ${topicName}`);
                } else if (typeof sub.isClosed !== 'function') {
                    sub.destroy();
                    console.log(`   ✅ Cleaned up: ${topicName}`);
                }
            }
        } catch (e) {
            console.log(`   ⚠️ Error cleaning ${topicName}:`, e.message);
        }
    });
    subscribers = {};
    console.log('✅ Subscribers cleanup complete');
}

/**
 * ✅ FIXED: Test subscribing status with proper error handling
 */
function testSubscribing() {
    try {
        const topics = Object.keys(subscribers);
        if (topics.length === 0) {
            return { success: false, error: 'No subscribers found' };
        }

        for (const topic of topics) {
            const sub = subscribers[topic];
            // ✅ FIXED: Only check isClosed if it exists and is a function
            if (sub && typeof sub.isClosed === 'function') {
                if (sub.isClosed()) {
                    return { success: false, error: `Subscriber for ${topic} is closed` };
                }
            } else if (!sub) {
                return { success: false, error: `Subscriber for ${topic} is null/undefined` };
            }
            // If isClosed method doesn't exist, assume subscriber is valid
        }

        return { success: true, topics };
    } catch (e) {
        return { success: false, error: e.message };
    }
}

module.exports = {
    initializeSubscribers,
    subscribeToAllTopics,
    subscribeToPosition,
    subscribeToOdometry,
    subscribeToGlobalCostmap,
    subscribeToLocalCostmap,
    subscribeToVelocityFeedback,
    // subscribeToJointStates,
    subscribeToMap,
    subscribeToBattery,
    setDeviceId,
    getSubscriptionStatus,
    listAvailableTopics,
    cleanup,
    testSubscribing,
};