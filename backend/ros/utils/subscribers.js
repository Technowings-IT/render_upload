// ros/utils/subscribers.js - Fixed ROS2 Subscribers for Real AGV Integration
const rclnodejs = require('rclnodejs');
const { getNode } = require('./ros_connection');
const config = require('../../config');

// Import WebSocket broadcasting
let broadcastToSubscribers;
try {
    const wsModule = require('../../websocket/clientConnection');
    broadcastToSubscribers = wsModule.broadcastToSubscribers;
} catch (error) {
    console.warn('⚠️ WebSocket module not available, using fallback');
    broadcastToSubscribers = () => {}; // Fallback function
}

// Message types
const Odometry = rclnodejs.require('nav_msgs/msg/Odometry');
const OccupancyGrid = rclnodejs.require('nav_msgs/msg/OccupancyGrid');
const LaserScan = rclnodejs.require('sensor_msgs/msg/LaserScan');
const Twist = rclnodejs.require('geometry_msgs/msg/Twist');
const JointState = rclnodejs.require('sensor_msgs/msg/JointState');
const BatteryState = rclnodejs.require('sensor_msgs/msg/BatteryState');

class RealAGVIntegration {
    constructor() {
        this.subscribers = new Map();
        this.robotTrails = new Map();
        this.deviceData = new Map();
        this.lastMapUpdate = new Map();
        this.mapUpdateThrottle = 2000; // 2 seconds between map updates
    }

    /**
     * Subscribe to your specific real AGV topics
     */
    subscribeToRealAGV(deviceId, options = {}) {
        console.log(`🤖 Setting up real AGV subscriptions for: ${deviceId}`);
        
        try {
            // Use actual topic names from your ROS2 list (without device prefix since you're connecting to the real AGV)
            this.subscribeToOdometry(deviceId, '/diff_drive_controller/odom');
            this.subscribeToMap(deviceId, '/map');
            this.subscribeToLaserScan(deviceId, '/scan');
            this.subscribeToJointStates(deviceId, '/joint_states');
            this.subscribeToVelocityFeedback(deviceId, '/cmd_vel_smoothed');
            this.subscribeToNavigationStatus(deviceId);
            
            // Initialize robot data structures
            this.initializeRobotData(deviceId);
            
            console.log(`✅ Successfully subscribed to real AGV: ${deviceId}`);
            
            // Update global live data reference
            if (global.liveData) {
                global.liveData[deviceId] = {
                    connected: true,
                    subscribedAt: new Date().toISOString(),
                    topics: ['/diff_drive_controller/odom', '/map', '/scan', '/joint_states', '/cmd_vel_smoothed']
                };
            }
            
        } catch (error) {
            console.error(`❌ Failed to subscribe to AGV ${deviceId}:`, error);
            throw error;
        }
    }

    /**
     * Subscribe to odometry from your real AGV
     */
    subscribeToOdometry(deviceId, topicName) {
        const callback = (msg) => {
            try {
                const position = {
                    x: msg.pose.pose.position.x,
                    y: msg.pose.pose.position.y,
                    z: msg.pose.pose.position.z,
                    orientation: {
                        x: msg.pose.pose.orientation.x,
                        y: msg.pose.pose.orientation.y,
                        z: msg.pose.pose.orientation.z,
                        w: msg.pose.pose.orientation.w
                    }
                };

                const velocity = {
                    linear: {
                        x: msg.twist.twist.linear.x,
                        y: msg.twist.twist.linear.y,
                        z: msg.twist.twist.linear.z
                    },
                    angular: {
                        x: msg.twist.twist.angular.x,
                        y: msg.twist.twist.angular.y,
                        z: msg.twist.twist.angular.z
                    }
                };

                // Calculate yaw angle from quaternion
                const yaw = Math.atan2(
                    2.0 * (position.orientation.w * position.orientation.z + position.orientation.x * position.orientation.y),
                    1.0 - 2.0 * (position.orientation.y * position.orientation.y + position.orientation.z * position.orientation.z)
                );

                // Update robot trail for visualization
                this.updateRobotTrail(deviceId, position);
                
                // Store current data
                const odometryData = {
                    deviceId,
                    timestamp: new Date().toISOString(),
                    frame_id: msg.header.frame_id,
                    position,
                    velocity,
                    yaw,
                    covariance: {
                        pose: Array.from(msg.pose.covariance),
                        twist: Array.from(msg.twist.covariance)
                    }
                };

                this.deviceData.set(`${deviceId}_odometry`, odometryData);
                
                // Update global live data
                if (global.liveData) {
                    if (!global.liveData[deviceId]) global.liveData[deviceId] = {};
                    global.liveData[deviceId].position = position;
                    global.liveData[deviceId].velocity = velocity;
                    global.liveData[deviceId].yaw = yaw;
                    global.liveData[deviceId].lastOdometryUpdate = odometryData.timestamp;
                }
                
                // Broadcast real-time position with trail
                broadcastToSubscribers('real_time_data', {
                    type: 'odometry_update',
                    deviceId,
                    data: {
                        ...odometryData,
                        trail: this.getRobotTrail(deviceId, 50)
                    }
                });

                // Less frequent console logging to avoid spam
                if (Date.now() % 10000 < 1000) { // Log every ~10 seconds
                    console.log(`📍 ${deviceId} position: (${position.x.toFixed(2)}, ${position.y.toFixed(2)}) yaw: ${yaw.toFixed(2)}rad`);
                }
                
            } catch (error) {
                console.error(`❌ Error processing odometry for ${deviceId}:`, error);
            }
        };

        this.createSubscriber(topicName, Odometry, callback, deviceId);
    }

    /**
     * Subscribe to SLAM map updates with throttling
     */
    subscribeToMap(deviceId, topicName) {
        const callback = (msg) => {
            try {
                // Throttle map updates to prevent overwhelming the system
                const now = Date.now();
                const lastUpdate = this.lastMapUpdate.get(deviceId) || 0;
                if (now - lastUpdate < this.mapUpdateThrottle) {
                    return; // Skip this update
                }
                this.lastMapUpdate.set(deviceId, now);

                const mapData = {
                    deviceId,
                    timestamp: new Date().toISOString(),
                    frame_id: msg.header.frame_id,
                    info: {
                        resolution: msg.info.resolution,
                        width: msg.info.width,
                        height: msg.info.height,
                        origin: {
                            position: {
                                x: msg.info.origin.position.x,
                                y: msg.info.origin.position.y,
                                z: msg.info.origin.position.z
                            },
                            orientation: {
                                x: msg.info.origin.orientation.x,
                                y: msg.info.origin.orientation.y,
                                z: msg.info.origin.orientation.z,
                                w: msg.info.origin.orientation.w
                            }
                        }
                    },
                    data: Array.from(msg.data),
                    metadata: {
                        isRealTime: true,
                        mapMode: 'slam_update',
                        dataSize: msg.data.length,
                        mapArea: msg.info.width * msg.info.height * Math.pow(msg.info.resolution, 2)
                    }
                };

                // Store latest map
                this.deviceData.set(`${deviceId}_map`, mapData);
                
                // Update global live data
                if (global.liveData) {
                    if (!global.liveData[deviceId]) global.liveData[deviceId] = {};
                    global.liveData[deviceId].map = mapData;
                    global.liveData[deviceId].lastMapUpdate = mapData.timestamp;
                }
                
                // Broadcast real-time map update (throttled)
                broadcastToSubscribers('real_time_data', {
                    type: 'map_update',
                    deviceId,
                    data: mapData
                });
                
                console.log(`🗺️ Map update for ${deviceId}: ${mapData.info.width}x${mapData.info.height} (${(mapData.metadata.dataSize/1024).toFixed(1)}KB)`);
                
            } catch (error) {
                console.error(`❌ Error processing map for ${deviceId}:`, error);
            }
        };

        this.createSubscriber(topicName, OccupancyGrid, callback, deviceId);
    }

    /**
     * Subscribe to LIDAR scan data
     */
    subscribeToLaserScan(deviceId, topicName) {
        const callback = (msg) => {
            try {
                const scanData = {
                    deviceId,
                    timestamp: new Date().toISOString(),
                    frame_id: msg.header.frame_id,
                    angle_min: msg.angle_min,
                    angle_max: msg.angle_max,
                    angle_increment: msg.angle_increment,
                    time_increment: msg.time_increment,
                    scan_time: msg.scan_time,
                    range_min: msg.range_min,
                    range_max: msg.range_max,
                    ranges: Array.from(msg.ranges),
                    intensities: Array.from(msg.intensities || [])
                };

                // Process scan for obstacles and features
                const processed = this.processLaserScan(scanData);
                
                // Store scan data
                this.deviceData.set(`${deviceId}_scan`, { ...scanData, ...processed });
                
                // Update global live data
                if (global.liveData) {
                    if (!global.liveData[deviceId]) global.liveData[deviceId] = {};
                    global.liveData[deviceId].scan = { ...scanData, ...processed };
                    global.liveData[deviceId].lastScanUpdate = scanData.timestamp;
                }
                
                // Broadcast live sensor data (with reduced frequency)
                if (Date.now() % 500 < 100) { // Broadcast ~every 500ms
                    broadcastToSubscribers('real_time_data', {
                        type: 'laser_scan',
                        deviceId,
                        data: { ...scanData, ...processed }
                    });
                }

                // Periodic logging
                if (Date.now() % 5000 < 100) {
                    console.log(`🔍 ${deviceId} scan: ${scanData.ranges.length} points, ${processed.obstacles.length} obstacles, ${processed.features.length} features`);
                }
                
            } catch (error) {
                console.error(`❌ Error processing laser scan for ${deviceId}:`, error);
            }
        };

        this.createSubscriber(topicName, LaserScan, callback, deviceId);
    }

    /**
     * Subscribe to joint states for wheel encoders
     */
    subscribeToJointStates(deviceId, topicName) {
        const callback = (msg) => {
            try {
                const jointData = {
                    deviceId,
                    timestamp: new Date().toISOString(),
                    frame_id: msg.header.frame_id,
                    joints: msg.name.map((name, index) => ({
                        name: name,
                        position: msg.position[index] || 0,
                        velocity: msg.velocity[index] || 0,
                        effort: msg.effort[index] || 0
                    }))
                };

                // Store joint data
                this.deviceData.set(`${deviceId}_joints`, jointData);
                
                // Update global live data
                if (global.liveData) {
                    if (!global.liveData[deviceId]) global.liveData[deviceId] = {};
                    global.liveData[deviceId].joints = jointData;
                    global.liveData[deviceId].lastJointUpdate = jointData.timestamp;
                }
                
                // Broadcast joint states (reduced frequency)
                if (Date.now() % 1000 < 100) {
                    broadcastToSubscribers('real_time_data', {
                        type: 'joint_states',
                        deviceId,
                        data: jointData
                    });
                }
                
            } catch (error) {
                console.error(`❌ Error processing joint states for ${deviceId}:`, error);
            }
        };

        this.createSubscriber(topicName, JointState, callback, deviceId);
    }

    /**
     * Subscribe to velocity feedback
     */
    subscribeToVelocityFeedback(deviceId, topicName) {
        const callback = (msg) => {
            try {
                const velocityFeedback = {
                    deviceId,
                    timestamp: new Date().toISOString(),
                    linear: {
                        x: msg.linear.x,
                        y: msg.linear.y,
                        z: msg.linear.z
                    },
                    angular: {
                        x: msg.angular.x,
                        y: msg.angular.y,
                        z: msg.angular.z
                    },
                    speed: Math.sqrt(Math.pow(msg.linear.x, 2) + Math.pow(msg.linear.y, 2)),
                    isMoving: Math.abs(msg.linear.x) > 0.01 || Math.abs(msg.angular.z) > 0.01
                };

                // Store velocity feedback
                this.deviceData.set(`${deviceId}_velocity_feedback`, velocityFeedback);
                
                // Update global live data
                if (global.liveData) {
                    if (!global.liveData[deviceId]) global.liveData[deviceId] = {};
                    global.liveData[deviceId].velocityFeedback = velocityFeedback;
                    global.liveData[deviceId].lastVelocityUpdate = velocityFeedback.timestamp;
                }
                
                // Broadcast velocity feedback
                broadcastToSubscribers('real_time_data', {
                    type: 'velocity_feedback',
                    deviceId,
                    data: velocityFeedback
                });
                
            } catch (error) {
                console.error(`❌ Error processing velocity feedback for ${deviceId}:`, error);
            }
        };

        this.createSubscriber(topicName, Twist, callback, deviceId);
    }

    /**
     * Subscribe to navigation status topics
     */
    subscribeToNavigationStatus(deviceId) {
        try {
            // Subscribe to goal status if available
            const goalStatusTopics = ['/goal_pose', '/move_base/status', '/nav2_goals'];
            
            goalStatusTopics.forEach(topic => {
                // These might not always be available, so we handle gracefully
                try {
                    // This is a simplified example - you'd need to adjust based on actual message types
                    console.log(`🎯 Attempting to subscribe to navigation topic: ${topic}`);
                } catch (error) {
                    console.log(`⚠️ Navigation topic ${topic} not available`);
                }
            });
            
        } catch (error) {
            console.log(`⚠️ Navigation status subscription failed for ${deviceId}:`, error.message);
        }
    }

    /**
     * Process laser scan for obstacle detection and feature extraction
     */
    processLaserScan(scanData) {
        const obstacles = [];
        const features = [];
        const robotPosition = this.deviceData.get(`${scanData.deviceId}_odometry`);
        
        if (!robotPosition) {
            return { obstacles: [], features: [], processed: false };
        }

        const pos = robotPosition.position;
        const yaw = robotPosition.yaw || 0;

        // Sample every 5th point for performance
        for (let i = 0; i < scanData.ranges.length; i += 5) {
            const range = scanData.ranges[i];
            const angle = scanData.angle_min + (i * scanData.angle_increment);
            const globalAngle = yaw + angle;
            
            // Check for valid range reading
            if (range >= scanData.range_min && range <= scanData.range_max && range < 3.0) {
                // Convert to global coordinates
                const obstacleX = pos.x + range * Math.cos(globalAngle);
                const obstacleY = pos.y + range * Math.sin(globalAngle);
                
                obstacles.push({
                    x: obstacleX,
                    y: obstacleY,
                    range: range,
                    angle: angle,
                    globalAngle: globalAngle,
                    intensity: scanData.intensities[i] || 0,
                    localX: range * Math.cos(angle),
                    localY: range * Math.sin(angle)
                });

                // Simple feature detection (corners, walls)
                if (i > 10 && i < scanData.ranges.length - 10) {
                    const prevRange = scanData.ranges[i - 5];
                    const nextRange = scanData.ranges[i + 5];
                    
                    // Detect corners (sudden range changes)
                    if (Math.abs(range - prevRange) > 0.3 || Math.abs(range - nextRange) > 0.3) {
                        features.push({
                            type: 'corner',
                            x: obstacleX,
                            y: obstacleY,
                            confidence: Math.min(Math.abs(range - prevRange), Math.abs(range - nextRange))
                        });
                    }
                }
            }
        }

        return {
            obstacles,
            features,
            processed: true,
            stats: {
                totalPoints: scanData.ranges.length,
                validPoints: obstacles.length,
                featuresDetected: features.length,
                minRange: Math.min(...scanData.ranges.filter(r => r >= scanData.range_min && r <= scanData.range_max)),
                maxRange: Math.max(...scanData.ranges.filter(r => r >= scanData.range_min && r <= scanData.range_max))
            }
        };
    }

    /**
     * Update robot trail for visualization
     */
    updateRobotTrail(deviceId, position) {
        if (!this.robotTrails.has(deviceId)) {
            this.robotTrails.set(deviceId, []);
        }
        
        const trail = this.robotTrails.get(deviceId);
        
        // Only add if robot has moved significantly
        if (trail.length === 0 || 
            Math.sqrt(Math.pow(position.x - trail[trail.length - 1].x, 2) + 
                     Math.pow(position.y - trail[trail.length - 1].y, 2)) > 0.05) {
            
            trail.push({
                x: position.x,
                y: position.y,
                timestamp: new Date().toISOString(),
                yaw: Math.atan2(
                    2.0 * (position.orientation.w * position.orientation.z + position.orientation.x * position.orientation.y),
                    1.0 - 2.0 * (position.orientation.y * position.orientation.y + position.orientation.z * position.orientation.z)
                )
            });
            
            // Keep only last 200 points
            if (trail.length > 200) {
                trail.shift();
            }
        }
    }

    /**
     * Get robot trail for visualization
     */
    getRobotTrail(deviceId, maxPoints = 100) {
        const trail = this.robotTrails.get(deviceId) || [];
        return trail.slice(-maxPoints);
    }

    /**
     * Initialize robot data structures
     */
    initializeRobotData(deviceId) {
        this.robotTrails.set(deviceId, []);
        console.log(`📊 Initialized data structures for ${deviceId}`);
    }

    /**
     * Create ROS2 subscriber with error handling
     */
    createSubscriber(topicName, messageType, callback, deviceId) {
        const subscriberKey = `${deviceId}_${topicName}`;
        
        if (this.subscribers.has(subscriberKey)) {
            console.log(`⚠️ Subscriber already exists for: ${topicName} (${deviceId})`);
            return;
        }

        try {
            const node = getNode();
            const subscriber = node.createSubscription(messageType, topicName, callback);
            this.subscribers.set(subscriberKey, subscriber);
            console.log(`✅ Created subscriber for: ${topicName} (${deviceId})`);
        } catch (error) {
            console.error(`❌ Failed to create subscriber for ${topicName} (${deviceId}):`, error);
            throw error;
        }
    }

    /**
     * Get all real-time data for a device
     */
    getRealTimeData(deviceId) {
        return {
            odometry: this.deviceData.get(`${deviceId}_odometry`),
            map: this.deviceData.get(`${deviceId}_map`),
            scan: this.deviceData.get(`${deviceId}_scan`),
            joints: this.deviceData.get(`${deviceId}_joints`),
            velocityFeedback: this.deviceData.get(`${deviceId}_velocity_feedback`),
            trail: this.getRobotTrail(deviceId),
            lastUpdate: new Date().toISOString()
        };
    }

    /**
     * Get device data for specific type
     */
    getDeviceData(deviceId, dataType = null) {
        if (dataType) {
            return this.deviceData.get(`${deviceId}_${dataType}`);
        }
        
        // Return all data for device
        const allData = {};
        for (const [key, value] of this.deviceData.entries()) {
            if (key.startsWith(`${deviceId}_`)) {
                const type = key.replace(`${deviceId}_`, '');
                allData[type] = value;
            }
        }
        return allData;
    }

    /**
     * Clear all data for a device
     */
    clearDeviceData(deviceId) {
        // Remove subscribers
        const toRemove = [];
        for (const [key, subscriber] of this.subscribers.entries()) {
            if (key.startsWith(`${deviceId}_`)) {
                toRemove.push(key);
            }
        }
        
        toRemove.forEach(key => {
            this.subscribers.delete(key);
        });

        // Clear data
        const dataToRemove = [];
        for (const key of this.deviceData.keys()) {
            if (key.startsWith(`${deviceId}_`)) {
                dataToRemove.push(key);
            }
        }
        
        dataToRemove.forEach(key => {
            this.deviceData.delete(key);
        });

        // Clear trail
        this.robotTrails.delete(deviceId);
        
        // Clear from global live data
        if (global.liveData && global.liveData[deviceId]) {
            delete global.liveData[deviceId];
        }
        
        console.log(`🧹 Cleared all data for device: ${deviceId}`);
    }

    /**
     * Get connection status for all devices
     */
    getConnectionStatus() {
        const status = {};
        for (const deviceId of this.robotTrails.keys()) {
            const lastOdom = this.deviceData.get(`${deviceId}_odometry`);
            status[deviceId] = {
                connected: !!lastOdom,
                lastUpdate: lastOdom?.timestamp,
                subscribers: Array.from(this.subscribers.keys()).filter(key => key.startsWith(`${deviceId}_`)).length
            };
        }
        return status;
    }

    /**
     * Cleanup all subscriptions and data
     */
    cleanup() {
        console.log('🧹 Cleaning up AGV integration...');
        
        // Clear all subscribers
        this.subscribers.clear();
        
        // Clear all data
        this.robotTrails.clear();
        this.deviceData.clear();
        this.lastMapUpdate.clear();
        
        console.log('✅ AGV integration cleanup complete');
    }
}

// Create and export singleton instance
const realAGVIntegration = new RealAGVIntegration();

module.exports = {
    RealAGVIntegration,
    realAGVIntegration,
    
    // Legacy compatibility functions
    subscribeToDevice: (deviceId) => realAGVIntegration.subscribeToRealAGV(deviceId),
    getDeviceData: (deviceId) => realAGVIntegration.getDeviceData(deviceId),
    clearDeviceData: (deviceId) => realAGVIntegration.clearDeviceData(deviceId),
    getRealTimeData: (deviceId) => realAGVIntegration.getRealTimeData(deviceId)
};