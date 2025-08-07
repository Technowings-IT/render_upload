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
function getOrCreateSubscriber(topicName, messageType, callback, qosOptions = {}) {
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
                    durability: 'transient_local',
                    history: 'keep_last',
                    ...qosOptions // <-- Merge/override with caller's options
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
 * Subscribe to odometry as fallback for position tracking
 */
function subscribeToOdometry() {
    const callback = (odomMsg) => {
        try {
            const positionData = {
                position: {
                    x: odomMsg.pose.pose.position.x || 0,
                    y: odomMsg.pose.pose.position.y || 0,
                    z: odomMsg.pose.pose.position.z || 0
                },
                orientation: {
                    x: odomMsg.pose.pose.orientation.x || 0,
                    y: odomMsg.pose.pose.orientation.y || 0,
                    z: odomMsg.pose.pose.orientation.z || 0,
                    w: odomMsg.pose.pose.orientation.w || 1,
                    yaw: quaternionToYaw(odomMsg.pose.pose.orientation)
                },
                velocity: {
                    linear: {
                        x: odomMsg.twist.twist.linear.x || 0,
                        y: odomMsg.twist.twist.linear.y || 0,
                        z: odomMsg.twist.twist.linear.z || 0
                    },
                    angular: {
                        x: odomMsg.twist.twist.angular.x || 0,
                        y: odomMsg.twist.twist.angular.y || 0,
                        z: odomMsg.twist.twist.angular.z || 0
                    }
                },
                covariance: {
                    pose: odomMsg.pose.covariance || [],
                    twist: odomMsg.twist.covariance || []
                },
                timestamp: new Date().toISOString(),
                frame_id: odomMsg.header?.frame_id || 'odom',
                child_frame_id: odomMsg.child_frame_id || 'base_link'
            };

            // Update global live data
            updateGlobalLiveData('position', positionData);
            updateGlobalLiveData('odometry', positionData);

            // Broadcast position update with device ID
            broadcastToSubscribers('real_time_data', {
                type: 'position_update',
                data: positionData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Also broadcast as odometry data
            broadcastToSubscribers('real_time_data', {
                type: 'odometry_update',
                data: positionData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Console log every 3 seconds to avoid spam
            if (Date.now() % 3000 < 100) {
                console.log(`🧭 [${currentDeviceId}] Odometry: (${positionData.position.x.toFixed(2)}, ${positionData.position.y.toFixed(2)}) θ=${positionData.orientation.yaw.toFixed(2)} v=${positionData.velocity.linear.x.toFixed(2)}m/s`);
            }

        } catch (error) {
            console.error('❌ Error processing odometry message:', error);
        }
    };

    // Try different possible odometry topic names
    const odomTopics = [
        '/odom',
        '/odometry',
        '/robot/odom',
        '/base_controller/odom'
    ];

    for (const topicName of odomTopics) {
        try {
            getOrCreateSubscriber(topicName, 'nav_msgs/msg/Odometry', callback);
            console.log(`✅ Successfully subscribed to odometry topic: ${topicName}`);
            return; // Success, exit function
        } catch (error) {
            console.warn(`⚠️ Failed to subscribe to ${topicName}, trying next...`);
        }
    }

    console.error('❌ Failed to subscribe to any odometry topics');
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
            getOrCreateSubscriber(topicName, 'nav_msgs/msg/OccupancyGrid', callback, {
                depth: 10,
                reliability: 'reliable',
                durability: 'transient_local', // <-- This is the key!
                history: 'keep_last'
            });
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
// Enhance the subscribeToMap function:
function subscribeToMap() {
    const mapTopics = [
        '/map',
        '/map_server/map'
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

            // ✅ FIXED: Broadcast with proper type name
            broadcastToSubscribers('real_time_data', {
                type: 'map_update',
                data: mapData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            if (!global.lastMapBroadcast || Date.now() - global.lastMapBroadcast > 5000) {
                console.log(`🗺️ [${currentDeviceId}] Map: ${mapData.info.width}x${mapData.info.height} @ ${mapData.info.resolution.toFixed(3)}m/px`);
                global.lastMapBroadcast = Date.now();
            }
        } catch (error) {
            console.error('❌ Error processing map:', error);
        }
    };

    // Try each topic with better error handling
    for (const topicName of mapTopics) {
        try {
            getOrCreateSubscriber(topicName, 'nav_msgs/msg/OccupancyGrid', callback, {
                depth: 10,
                reliability: 'reliable',
                durability: 'transient_local', // <-- This is the fix!
                history: 'keep_last'
            });
            console.log(`✅ Successfully subscribed to map: ${topicName}`);
            return topicName;
        } catch (error) {
            console.warn(`⚠️ Failed to subscribe to ${topicName}: ${error.message}`);
        }
    }

    console.error('❌ Failed to subscribe to any /map topic');
    return null;
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
    // Callback for Twist messages (geometry_msgs/msg/Twist)
    const twistCallback = (twistMsg) => {
        try {
            const velocityData = {
                linear: {
                    x: twistMsg.linear?.x || 0,
                    y: twistMsg.linear?.y || 0,
                    z: twistMsg.linear?.z || 0
                },
                angular: {
                    x: twistMsg.angular?.x || 0,
                    y: twistMsg.angular?.y || 0,
                    z: twistMsg.angular?.z || 0
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

            // Log occasionally to avoid spam
            if (Date.now() % 2000 < 100) {
                console.log(`🚗 [${currentDeviceId}] Velocity: linear=(${velocityData.linear.x.toFixed(2)}, ${velocityData.linear.y.toFixed(2)}) angular=${velocityData.angular.z.toFixed(2)}`);
            }

        } catch (error) {
            console.error('❌ Error processing twist velocity feedback:', error);
        }
    };

    // Callback for Float32MultiArray messages (wheel velocities)
    const arrayCallback = (arrayMsg) => {
        try {
            const velocityData = {
                wheel_velocities: arrayMsg.data || [],
                timestamp: new Date().toISOString()
            };

            // If we have wheel velocities, try to interpret them
            if (velocityData.wheel_velocities.length >= 2) {
                // Assume differential drive: [left_wheel, right_wheel]
                const leftVel = velocityData.wheel_velocities[0];
                const rightVel = velocityData.wheel_velocities[1];
                
                // Calculate linear and angular velocity from wheel velocities
                // Assuming wheel separation (you may need to adjust this)
                const wheelSeparation = 0.5; // meters (adjust based on your robot)
                const wheelRadius = 0.1; // meters (adjust based on your robot)
                
                const linearVel = (leftVel + rightVel) * wheelRadius / 2.0;
                const angularVel = (rightVel - leftVel) * wheelRadius / wheelSeparation;
                
                velocityData.linear = {
                    x: linearVel,
                    y: 0,
                    z: 0
                };
                velocityData.angular = {
                    x: 0,
                    y: 0,
                    z: angularVel
                };
            }

            updateGlobalLiveData('velocity_feedback', velocityData);

            broadcastToSubscribers('real_time_data', {
                type: 'velocity_feedback',
                data: velocityData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Log occasionally to avoid spam
            if (Date.now() % 2000 < 100) {
                console.log(`🛞 [${currentDeviceId}] Wheel velocities: [${velocityData.wheel_velocities.map(v => v.toFixed(2)).join(', ')}]`);
            }

        } catch (error) {
            console.error('❌ Error processing array velocity feedback:', error);
        }
    };

    // Try different velocity feedback topics with appropriate message types
    const velocityTopicsWithTypes = [
        { topic: '/cmd_vel', type: 'geometry_msgs/msg/Twist', callback: twistCallback },
        { topic: '/cmd_vel_out', type: 'geometry_msgs/msg/Twist', callback: twistCallback },
        { topic: '/velocity_feedback', type: 'geometry_msgs/msg/Twist', callback: twistCallback },
        { topic: '/wheel_velocity', type: 'std_msgs/msg/Float32MultiArray', callback: arrayCallback },
        { topic: '/diff_drive_controller/wheel_velocity', type: 'std_msgs/msg/Float32MultiArray', callback: arrayCallback },
        { topic: '/wheel_speeds', type: 'std_msgs/msg/Float32MultiArray', callback: arrayCallback }
    ];

    for (const { topic, type, callback } of velocityTopicsWithTypes) {
        try {
            getOrCreateSubscriber(topic, type, callback);
            console.log(`✅ Successfully subscribed to velocity feedback: ${topic} [${type}]`);
            return;
        } catch (error) {
            console.warn(`⚠️ Failed to subscribe to ${topic} with ${type}, trying next...`);
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
 * Subscribe to battery state - ENHANCED to handle both BatteryState and Float32MultiArray
 */
function subscribeToBattery() {
    // Callback for BatteryState messages (sensor_msgs/msg/BatteryState)
    const batteryStateCallback = (batteryMsg) => {
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
                charging: batteryMsg.power_supply_status === 1, // Charging if status is 1
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
                console.log(`🔋 [${currentDeviceId}] Battery: ${batteryData.percentage?.toFixed(1) || 'N/A'}% (${batteryData.voltage?.toFixed(1) || 'N/A'}V) ${batteryData.charging ? '⚡' : ''}`);
            }

        } catch (error) {
            console.error('❌ Error processing battery state data:', error);
        }
    };

    // Callback for Float32MultiArray messages (std_msgs/msg/Float32MultiArray)
    const arrayCallback = (arrayMsg) => {
        try {
            const dataArray = arrayMsg.data || [];
            
            // Interpret Float32MultiArray based on common battery data formats
            let batteryData = {
                timestamp: new Date().toISOString()
            };

            if (dataArray.length >= 1) {
                // Assume first element is percentage
                batteryData.percentage = dataArray[0];
            }
            
            if (dataArray.length >= 2) {
                // Assume second element is voltage
                batteryData.voltage = dataArray[1];
            }
            
            if (dataArray.length >= 3) {
                // Assume third element is current
                batteryData.current = dataArray[2];
            }
            
            if (dataArray.length >= 4) {
                // Assume fourth element is charging status (0 = not charging, 1 = charging)
                batteryData.charging = dataArray[3] > 0;
                batteryData.power_supply_status = dataArray[3] > 0 ? 1 : 2; // 1 = charging, 2 = discharging
            }
            
            if (dataArray.length >= 5) {
                // Assume fifth element is temperature (if available)
                batteryData.temperature = dataArray[4];
            }

            // Set defaults for missing values
            batteryData.voltage = batteryData.voltage || 0;
            batteryData.current = batteryData.current || 0;
            batteryData.percentage = batteryData.percentage || 0;
            batteryData.charging = batteryData.charging || false;
            batteryData.present = true;

            // Store raw array data for debugging
            batteryData.raw_data = dataArray;

            updateGlobalLiveData('battery', batteryData);

            broadcastToSubscribers('real_time_data', {
                type: 'battery_update',
                data: batteryData,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Log every 10 seconds
            if (Date.now() % 10000 < 100) {
                console.log(`🔋 [${currentDeviceId}] Battery Array: ${batteryData.percentage?.toFixed(1) || 'N/A'}% (${batteryData.voltage?.toFixed(1) || 'N/A'}V) ${batteryData.charging ? '⚡' : ''} [${dataArray.map(v => v.toFixed(2)).join(', ')}]`);
            }

        } catch (error) {
            console.error('❌ Error processing battery array data:', error);
        }
    };

    // Try different battery topics with appropriate message types
    const batteryTopicsWithTypes = [
        // Standard BatteryState topics
        { topic: '/battery_status', type: 'std_msgs/msg/Float32MultiArray', callback: batteryStateCallback },
        { topic: '/power/battery_status', type: 'std_msgs/msg/Float32MultiArray', callback: batteryStateCallback },
        { topic: '/robot/battery', type: 'std_msgs/msg/Float32MultiArray', callback: batteryStateCallback },

        // Float32MultiArray topics (common for custom battery implementations)
        { topic: '/battery_status', type: 'std_msgs/msg/Float32MultiArray', callback: arrayCallback },
        { topic: '/battery_data', type: 'std_msgs/msg/Float32MultiArray', callback: arrayCallback },
        { topic: '/battery', type: 'std_msgs/msg/Float32MultiArray', callback: arrayCallback },
        { topic: '/power/battery_array', type: 'std_msgs/msg/Float32MultiArray', callback: arrayCallback },
        { topic: '/robot/battery_array', type: 'std_msgs/msg/Float32MultiArray', callback: arrayCallback }
    ];

    for (const { topic, type, callback } of batteryTopicsWithTypes) {
        try {
            getOrCreateSubscriber(topic, type, callback);
            console.log(`✅ Successfully subscribed to battery: ${topic} [${type}]`);
            return;
        } catch (error) {
            console.warn(`⚠️ Failed to subscribe to ${topic} with ${type}, trying next...`);
        }
    }

    console.warn('⚠️ No battery topics found');
}

/**
 * Subscribe to essential topics for AMR operation (REMOVED /map as requested)
 * FIXED: Enhanced error handling for missing Nav2 packages
 */
function subscribeToAllTopics() {
    console.log(`📥 Subscribing to essential AMR topics for device: ${currentDeviceId}...`);

    // Core position tracking (PRIORITY)
    console.log('🎯 Priority 1: Position tracking...');
    try {
        subscribeToPosition();
    } catch (error) {
        console.error('❌ Failed to subscribe to position topics:', error.message);
    }

    // Costmaps for navigation visualization (PRIORITY)
    console.log('🎯 Priority 2: Costmaps...');
    try {
        subscribeToGlobalCostmap();
        subscribeToLocalCostmap();
    } catch (error) {
        console.error('❌ Failed to subscribe to costmap topics:', error.message);
    }

    // Control feedback
    console.log('🎯 Priority 3: Control feedback...');
    try {
        subscribeToVelocityFeedback();
    } catch (error) {
        console.error('❌ Failed to subscribe to velocity feedback topics:', error.message);
    }

    // Map data
    console.log('🎯 Priority 4: Map...');
    try {
        subscribeToMap();
    } catch (error) {
        console.error('❌ Failed to subscribe to map topics:', error.message);
    }

    // Navigation feedback and status (OPTIONAL - may not be available)
    console.log('🎯 Priority 5: Navigation progress (optional)...');
    try {
        const navFeedbackResult = subscribeToNavigationFeedback();
        const navStatusResult = subscribeToNavigationStatus();
        
        if (!navFeedbackResult && !navStatusResult) {
            console.log('💡 Navigation feedback disabled - this is normal if Nav2 is not installed');
        }
    } catch (error) {
        console.warn('⚠️ Navigation feedback topics not available:', error.message);
        console.log('💡 This is normal if Nav2 packages are not installed on your system');
    }

    // Battery monitoring
    console.log('🎯 Priority 6: Battery...');
    try {
        subscribeToBattery();
    } catch (error) {
        console.error('❌ Failed to subscribe to battery topics:', error.message);
    }

    console.log(`✅ All topic subscriptions attempted for device: ${currentDeviceId}`);
    console.log('💡 Missing topics are normal - they depend on what ROS2 packages you have installed');

    // List active subscriptions after 3 seconds
    setTimeout(() => {
        console.log('📊 Active subscriptions summary:');
        const activeCount = Object.keys(subscribers).length;
        console.log(`   Total attempted: ${activeCount} topics`);
        
        let successCount = 0;
        Object.keys(subscribers).forEach(topic => {
            const sub = subscribers[topic];
            try {
                if (sub && typeof sub.isClosed === 'function' && !sub.isClosed()) {
                    console.log(`   ✅ ${topic}`);
                    successCount++;
                } else if (sub && typeof sub.isClosed !== 'function') {
                    console.log(`   ✅ ${topic} (status unknown)`);
                    successCount++;
                } else {
                    console.log(`   ❌ ${topic} (failed/closed)`);
                }
            } catch (statusError) {
                console.log(`   ⚠️ ${topic} (status check failed: ${statusError.message})`);
            }
        });
        
        console.log(`📈 Subscription success rate: ${successCount}/${activeCount} topics`);
        
        if (successCount === 0) {
            console.log('🚨 No active subscriptions! Check if ROS2 topics are being published');
            console.log('💡 Try: ros2 topic list');
        } else if (successCount < 3) {
            console.log('⚠️ Limited subscriptions active. Some AMR features may not work');
        } else {
            console.log('✅ Good subscription coverage - AMR monitoring should work well');
        }
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

// Add this function after the existing subscriber functions:

/**
 * Subscribe to navigate_to_pose action feedback for navigation progress tracking
 * FIXED: More robust error handling for missing nav2 messages
 */
function subscribeToNavigationFeedback() {
    console.log('🧭 Attempting to subscribe to navigation feedback...');
    
    // Simplified callback for basic navigation feedback
    const basicCallback = (feedbackMsg) => {
        try {
            // Handle different message formats gracefully
            const navigationFeedback = {
                current_pose: null,
                navigation_time: 0,
                distance_remaining: 0,
                status_text: 'ACTIVE',
                timestamp: new Date().toISOString(),
                frame_id: 'map'
            };

            // Try to extract data based on message structure
            if (feedbackMsg.pose) {
                // PoseStamped message
                navigationFeedback.current_pose = {
                    position: {
                        x: feedbackMsg.pose.position?.x || 0,
                        y: feedbackMsg.pose.position?.y || 0,
                        z: feedbackMsg.pose.position?.z || 0
                    },
                    orientation: {
                        x: feedbackMsg.pose.orientation?.x || 0,
                        y: feedbackMsg.pose.orientation?.y || 0,
                        z: feedbackMsg.pose.orientation?.z || 0,
                        w: feedbackMsg.pose.orientation?.w || 1,
                        yaw: feedbackMsg.pose.orientation ? 
                            quaternionToYaw(feedbackMsg.pose.orientation) : 0
                    }
                };
                navigationFeedback.frame_id = feedbackMsg.header?.frame_id || 'map';
            } else if (feedbackMsg.feedback) {
                // Action feedback message
                navigationFeedback.navigation_time = feedbackMsg.feedback.navigation_time?.sec || 0;
                navigationFeedback.distance_remaining = feedbackMsg.feedback.distance_remaining || 0;
                
                if (feedbackMsg.feedback.current_pose) {
                    navigationFeedback.current_pose = {
                        position: {
                            x: feedbackMsg.feedback.current_pose.pose?.position?.x || 0,
                            y: feedbackMsg.feedback.current_pose.pose?.position?.y || 0,
                            z: feedbackMsg.feedback.current_pose.pose?.position?.z || 0
                        },
                        orientation: {
                            x: feedbackMsg.feedback.current_pose.pose?.orientation?.x || 0,
                            y: feedbackMsg.feedback.current_pose.pose?.orientation?.y || 0,
                            z: feedbackMsg.feedback.current_pose.pose?.orientation?.z || 0,
                            w: feedbackMsg.feedback.current_pose.pose?.orientation?.w || 1,
                            yaw: feedbackMsg.feedback.current_pose.pose?.orientation ? 
                                quaternionToYaw(feedbackMsg.feedback.current_pose.pose.orientation) : 0
                        }
                    };
                }
            }

            // Update global live data
            updateGlobalLiveData('navigation_feedback', navigationFeedback);

            // Broadcast navigation feedback
            broadcastToSubscribers('real_time_data', {
                type: 'navigation_feedback_update',
                data: navigationFeedback,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Log occasionally
            if (!global.lastNavigationFeedbackLog || Date.now() - global.lastNavigationFeedbackLog > 5000) {
                console.log(`🧭 [${currentDeviceId}] Navigation feedback received`);
                if (navigationFeedback.current_pose) {
                    console.log(`   📍 Position: (${navigationFeedback.current_pose.position.x.toFixed(2)}, ${navigationFeedback.current_pose.position.y.toFixed(2)})`);
                }
                global.lastNavigationFeedbackLog = Date.now();
            }

        } catch (error) {
            console.error('❌ Error processing navigation feedback:', error);
        }
    };

    // Try simpler, more commonly available topics first
    const navigationTopics = [
        // Basic navigation status topics (more likely to exist)
        { topic: '/goal_pose', type: 'geometry_msgs/msg/PoseStamped' },
        { topic: '/current_goal', type: 'geometry_msgs/msg/PoseStamped' },
        { topic: '/navigation_goal', type: 'geometry_msgs/msg/PoseStamped' },
        
        // Try action topics with error handling
        { topic: '/navigate_to_pose/_action/feedback', type: 'nav2_msgs/action/NavigateToPose_Feedback' },
        { topic: '/move_base/_action/feedback', type: 'move_base_msgs/action/MoveBase_Feedback' },
        
        // Fallback to status arrays
        { topic: '/navigate_to_pose/_action/status', type: 'action_msgs/msg/GoalStatusArray' },
        { topic: '/move_base/status', type: 'actionlib_msgs/msg/GoalStatusArray' }
    ];

    for (const { topic, type } of navigationTopics) {
        try {
            getOrCreateSubscriber(topic, type, basicCallback, {
                depth: 10,
                reliability: 'reliable',
                durability: 'volatile',
                history: 'keep_last'
            });
            console.log(`✅ Successfully subscribed to navigation feedback: ${topic} [${type}]`);
            return topic;
        } catch (error) {
            console.warn(`⚠️ Navigation topic ${topic} with ${type} not available: ${error.message}`);
            continue;
        }
    }

    console.warn('⚠️ No navigation feedback topics available - navigation feedback disabled');
    console.log('💡 This is normal if Nav2 is not installed or navigation is not running');
    return null;
}

/**
 * Subscribe to navigation status/result for completion tracking
 * FIXED: More robust error handling for missing nav2 messages
 */
function subscribeToNavigationStatus() {
    console.log('📊 Attempting to subscribe to navigation status...');
    
    const statusCallback = (statusMsg) => {
        try {
            // Handle different status message formats
            let navigationStatus = {
                goal_id: null,
                status: 0,
                status_text: 'UNKNOWN',
                timestamp: new Date().toISOString()
            };

            // Handle GoalStatusArray messages
            if (statusMsg.status_list && Array.isArray(statusMsg.status_list)) {
                // actionlib_msgs/GoalStatusArray or action_msgs/GoalStatusArray
                const latestStatus = statusMsg.status_list[statusMsg.status_list.length - 1];
                if (latestStatus) {
                    navigationStatus.goal_id = latestStatus.goal_id?.id || latestStatus.goal_info?.goal_id?.uuid || null;
                    navigationStatus.status = latestStatus.status || 0;
                    navigationStatus.status_text = getNavigationStatusText(latestStatus.status || 0);
                }
            } else if (statusMsg.status !== undefined) {
                // Single status message
                navigationStatus.goal_id = statusMsg.goal_id || null;
                navigationStatus.status = statusMsg.status;
                navigationStatus.status_text = getNavigationStatusText(statusMsg.status);
            } else if (statusMsg.result) {
                // Action result message
                navigationStatus.result = {
                    success: statusMsg.result.success || false,
                    error_code: statusMsg.result.error_code || 0,
                    error_msg: statusMsg.result.error_msg || ''
                };
                navigationStatus.status_text = statusMsg.result.success ? 'SUCCEEDED' : 'FAILED';
            }

            // Update global live data
            updateGlobalLiveData('navigation_status', navigationStatus);

            // Broadcast navigation status
            broadcastToSubscribers('real_time_data', {
                type: 'navigation_status_update',
                data: navigationStatus,
                deviceId: currentDeviceId,
                timestamp: new Date().toISOString()
            });

            // Log status changes
            if (!global.lastNavigationStatus || global.lastNavigationStatus !== navigationStatus.status_text) {
                console.log(`📊 [${currentDeviceId}] Navigation Status: ${navigationStatus.status_text}`);
                if (navigationStatus.result) {
                    console.log(`   Result: ${navigationStatus.result.success ? 'SUCCESS' : 'FAILED'} - ${navigationStatus.result.error_msg}`);
                }
                global.lastNavigationStatus = navigationStatus.status_text;
            }

        } catch (error) {
            console.error('❌ Error processing navigation status:', error);
        }
    };

    // Try different navigation status topics with graceful fallbacks
    const statusTopics = [
        // More commonly available status topics
        { topic: '/move_base/status', type: 'actionlib_msgs/msg/GoalStatusArray' },
        { topic: '/navigation/status', type: 'std_msgs/msg/String' },
        
        // Nav2 specific topics (may not be available)
        { topic: '/navigate_to_pose/_action/status', type: 'action_msgs/msg/GoalStatusArray' },
        { topic: '/navigate_to_pose/_action/result', type: 'nav2_msgs/action/NavigateToPose_Result' },
        { topic: '/navigate_to_pose/status', type: 'actionlib_msgs/msg/GoalStatusArray' },
        
        // Fallback topics
        { topic: '/move_base/result', type: 'move_base_msgs/action/MoveBase_Result' }
    ];

    for (const { topic, type } of statusTopics) {
        try {
            getOrCreateSubscriber(topic, type, statusCallback, {
                depth: 10,
                reliability: 'reliable',
                durability: 'volatile',
                history: 'keep_last'
            });
            console.log(`✅ Successfully subscribed to navigation status: ${topic} [${type}]`);
            return topic;
        } catch (error) {
            console.warn(`⚠️ Navigation status topic ${topic} with ${type} not available: ${error.message}`);
            continue;
        }
    }

    console.warn('⚠️ No navigation status topics available - navigation status disabled');
    console.log('💡 This is normal if Nav2 is not installed or navigation is not running');
    return null;
}

// Helper function to convert status codes to readable text
function getNavigationStatusText(status) {
    const statusMap = {
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
    return statusMap[status] || `UNKNOWN(${status})`;
}

module.exports = {
    initializeSubscribers,
    subscribeToAllTopics,
    subscribeToPosition,
    subscribeToOdometry,                  // Added missing function
    subscribeToGlobalCostmap,
    subscribeToLocalCostmap,
    subscribeToVelocityFeedback,
    // subscribeToJointStates,
    subscribeToNavigationFeedback,    // NEW
    subscribeToNavigationStatus,      // NEW
    subscribeToMap,
    subscribeToBattery,
    setDeviceId,
    getSubscriptionStatus,
    listAvailableTopics,
    cleanup,
    testSubscribing,
};