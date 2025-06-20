// ros/utils/ros_connection_working.js - Working version based on your successful test
const rclnodejs = require('rclnodejs');

let rosNode = null;
let isInitialized = false;
let publishers = {};
let subscribers = {};

async function initializeROS() {
    try {
        console.log('🤖 Initializing ROS2 connection...');
        
        // Use the same initialization that worked in your test
        await rclnodejs.init();
        console.log('✅ ROS2 initialized successfully');
        
        // Create the node
        rosNode = rclnodejs.createNode('agv_fleet_backend');
        console.log('✅ ROS2 node created: agv_fleet_backend');
        
        // Start spinning to process callbacks
        rclnodejs.spin(rosNode);
        console.log('✅ ROS2 node spinning');
        
        // Create publishers for AGV control
        await createPublishers();
        
        // Create subscribers for AGV data
        await createSubscribers();
        
        isInitialized = true;
        console.log('✅ Complete ROS2 integration initialized');
        return true;
        
    } catch (error) {
        console.error('❌ Failed to initialize ROS2:', error);
        isInitialized = false;
        throw error;
    }
}

async function createPublishers() {
    try {
        console.log('📤 Creating ROS2 publishers...');
        
        // Velocity command publisher
        publishers.cmdVel = rosNode.createPublisher(
            'geometry_msgs/msg/Twist', 
            '/cmd_vel',
            { depth: 10, reliability: 'reliable', durability: 'volatile' }
        );
        console.log('   ✅ cmd_vel publisher created');
        
        // Joystick publisher (if needed)
        publishers.joy = rosNode.createPublisher(
            'sensor_msgs/msg/Joy',
            '/joy',
            { depth: 10, reliability: 'reliable', durability: 'volatile' }
        );
        console.log('   ✅ joy publisher created');
        
        console.log(`📤 Created ${Object.keys(publishers).length} publishers`);
        
    } catch (error) {
        console.error('❌ Failed to create publishers:', error);
        throw error;
    }
}

async function createSubscribers() {
    try {
        console.log('📥 Creating ROS2 subscribers...');
        
        // Odometry subscriber
        subscribers.odom = rosNode.createSubscription(
            'nav_msgs/msg/Odometry',
            '/diff_drive_controller/odom',
            { depth: 10, reliability: 'reliable', durability: 'volatile' },
            (odomMsg) => handleOdometryMessage(odomMsg)
        );
        console.log('   ✅ odometry subscriber created');
        
        // Command velocity feedback subscriber
        subscribers.cmdVelFeedback = rosNode.createSubscription(
            'geometry_msgs/msg/Twist',
            '/diff_drive_controller/cmd_vel',
            { depth: 10, reliability: 'reliable', durability: 'volatile' },
            (twistMsg) => handleVelocityFeedback(twistMsg)
        );
        console.log('   ✅ cmd_vel feedback subscriber created');
        
        // Joint states subscriber
        subscribers.jointStates = rosNode.createSubscription(
            'sensor_msgs/msg/JointState',
            '/joint_states',
            { depth: 10, reliability: 'reliable', durability: 'volatile' },
            (jointMsg) => handleJointStates(jointMsg)
        );
        console.log('   ✅ joint_states subscriber created');
        
        console.log(`📥 Created ${Object.keys(subscribers).length} subscribers`);
        
    } catch (error) {
        console.error('❌ Failed to create subscribers:', error);
        throw error;
    }
}

// Handle incoming odometry data
function handleOdometryMessage(odomMsg) {
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
        
        // Update global live data for all connected devices
        updateLiveData('odometry', processedOdom);
        
        // Broadcast via WebSocket
        broadcastRealTimeData('odometry_update', processedOdom);
        
    } catch (error) {
        console.error('❌ Error processing odometry:', error);
    }
}

// Handle velocity feedback
function handleVelocityFeedback(twistMsg) {
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
        
        updateLiveData('velocity_feedback', velocityData);
        broadcastRealTimeData('velocity_feedback', velocityData);
        
    } catch (error) {
        console.error('❌ Error processing velocity feedback:', error);
    }
}

// Handle joint states
function handleJointStates(jointMsg) {
    try {
        const jointData = {
            joint_names: jointMsg.name,
            positions: jointMsg.position,
            velocities: jointMsg.velocity,
            efforts: jointMsg.effort,
            timestamp: new Date().toISOString()
        };
        
        updateLiveData('joint_states', jointData);
        broadcastRealTimeData('joint_states_update', jointData);
        
    } catch (error) {
        console.error('❌ Error processing joint states:', error);
    }
}

// Utility functions
function quaternionToYaw(q) {
    return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

function updateLiveData(dataType, data) {
    // Update live data for all connected devices
    global.connectedDevices?.forEach(device => {
        if (!global.liveData[device.id]) {
            global.liveData[device.id] = {};
        }
        global.liveData[device.id][dataType] = data;
        
        // Update device last seen
        device.lastSeen = new Date().toISOString();
    });
}

function broadcastRealTimeData(type, data) {
    try {
        // Import WebSocket broadcast function
        const { broadcastToSubscribers } = require('../../websocket/clientConnection');
        
        broadcastToSubscribers('real_time_data', {
            type: type,
            data: data,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        // WebSocket might not be available yet, ignore
    }
}

// Public API functions
function publishVelocity(linear = 0.0, angular = 0.0) {
    try {
        if (!isInitialized || !publishers.cmdVel) {
            throw new Error('ROS2 not initialized or cmd_vel publisher not available');
        }
        
        const velocityMsg = {
            linear: { x: linear, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: angular }
        };
        
        publishers.cmdVel.publish(velocityMsg);
        console.log(`🚗 Published velocity: linear=${linear}, angular=${angular}`);
        
        return {
            success: true,
            message: 'Velocity command sent successfully',
            data: { linear, angular }
        };
        
    } catch (error) {
        console.error('❌ Failed to publish velocity:', error);
        return {
            success: false,
            error: error.message
        };
    }
}

function emergencyStop() {
    return publishVelocity(0.0, 0.0);
}

function publishJoystick(x, y, deadman = false) {
    try {
        if (!isInitialized || !publishers.joy) {
            throw new Error('ROS2 not initialized or joy publisher not available');
        }
        
        const joyMsg = {
            header: {
                stamp: { sec: Math.floor(Date.now() / 1000), nanosec: (Date.now() % 1000) * 1000000 },
                frame_id: 'base_link'
            },
            axes: [x, y],
            buttons: [deadman ? 1 : 0]
        };
        
        publishers.joy.publish(joyMsg);
        console.log(`🕹️ Published joystick: x=${x}, y=${y}, deadman=${deadman}`);
        
        return {
            success: true,
            message: 'Joystick command sent successfully',
            data: { x, y, deadman }
        };
        
    } catch (error) {
        console.error('❌ Failed to publish joystick:', error);
        return {
            success: false,
            error: error.message
        };
    }
}

// Get ROS2 status
function getROS2Status() {
    const topics = rosNode ? rosNode.getTopicNamesAndTypes() : [];
    
    return {
        initialized: isInitialized,
        nodeActive: rosNode !== null,
        publishersCount: Object.keys(publishers).length,
        subscribersCount: Object.keys(subscribers).length,
        topicsDiscovered: topics.length,
        availableTopics: topics.map(t => t.name)
    };
}

// Test connectivity
async function testConnectivity() {
    try {
        if (!isInitialized) {
            throw new Error('ROS2 not initialized');
        }
        
        const topics = rosNode.getTopicNamesAndTypes();
        const agvTopics = ['/cmd_vel', '/diff_drive_controller/odom'];
        
        const results = {
            totalTopics: topics.length,
            agvTopicsFound: agvTopics.filter(topic => 
                topics.some(t => t.name === topic)
            ),
            publishersActive: Object.keys(publishers).length,
            subscribersActive: Object.keys(subscribers).length,
            canPublish: !!publishers.cmdVel,
            timestamp: new Date().toISOString()
        };
        
        console.log('🔍 ROS2 Connectivity Test:', results);
        return results;
        
    } catch (error) {
        console.error('❌ ROS2 connectivity test failed:', error);
        return { success: false, error: error.message };
    }
}

async function shutdown() {
    try {
        if (rosNode) {
            console.log('🛑 Shutting down ROS2 integration...');
            
            // Clean up publishers
            Object.keys(publishers).forEach(key => {
                try {
                    publishers[key].destroy();
                } catch (e) {
                    // Ignore cleanup errors
                }
            });
            publishers = {};
            
            // Clean up subscribers
            Object.keys(subscribers).forEach(key => {
                try {
                    subscribers[key].destroy();
                } catch (e) {
                    // Ignore cleanup errors
                }
            });
            subscribers = {};
            
            // Destroy node and shutdown
            rosNode.destroy();
            await rclnodejs.shutdown();
            rosNode = null;
            isInitialized = false;
            
            console.log('✅ ROS2 shutdown complete');
        }
    } catch (error) {
        console.error('❌ Error during ROS2 shutdown:', error);
    }
}

module.exports = {
    initializeROS,
    shutdown,
    publishVelocity,
    emergencyStop,
    publishJoystick,
    getROS2Status,
    testConnectivity,
    isRosInitialized: () => isInitialized
};