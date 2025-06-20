//publishers.js
const rclnodejs = require('rclnodejs');
const { getNode } = require('./ros_connection');
const config = require('../../config');

// Publishers cache
const publishers = new Map();
const serviceClients = new Map();

// Message types
const Twist = rclnodejs.require('geometry_msgs/msg/Twist');
const PoseStamped = rclnodejs.require('geometry_msgs/msg/PoseStamped');
const PoseWithCovarianceStamped = rclnodejs.require('geometry_msgs/msg/PoseWithCovarianceStamped');
const Empty = rclnodejs.require('std_msgs/msg/Empty');

function getPublisher(topicName, messageType) {
    if (!publishers.has(topicName)) {
        const node = getNode();
        const publisher = node.createPublisher(messageType, topicName);
        publishers.set(topicName, publisher);
        console.log(`Created publisher for topic: ${topicName}`);
    }
    return publishers.get(topicName);
}

function getServiceClient(serviceName, serviceType) {
    if (!serviceClients.has(serviceName)) {
        const node = getNode();
        const client = node.createClient(serviceType, serviceName);
        serviceClients.set(serviceName, client);
        console.log(`Created service client for: ${serviceName}`);
    }
    return serviceClients.get(serviceName);
}

/**
 * Publish velocity commands to AGV
 * @param {string} deviceId - AGV device identifier
 * @param {number} linear - Linear velocity (m/s)
 * @param {number} angular - Angular velocity (rad/s)
 */
function publishVelocity(deviceId, linear = 0, angular = 0) {
    try {
        // Clamp velocities to safe limits
        const clampedLinear = Math.max(-config.AGV.MAX_LINEAR_SPEED, 
                                     Math.min(config.AGV.MAX_LINEAR_SPEED, linear));
        const clampedAngular = Math.max(-config.AGV.MAX_ANGULAR_SPEED, 
                                      Math.min(config.AGV.MAX_ANGULAR_SPEED, angular));

        const topicName = `/${deviceId}${config.ROS2.TOPICS.CMD_VEL}`;
        const publisher = getPublisher(topicName, Twist);
        
        const twist = new Twist();
        twist.linear.x = clampedLinear;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = clampedAngular;
        
        publisher.publish(twist);
        
        console.log(`Published velocity to ${deviceId}: linear=${clampedLinear}, angular=${clampedAngular}`);
        
        return { success: true, linear: clampedLinear, angular: clampedAngular };
    } catch (error) {
        console.error(`Error publishing velocity to ${deviceId}:`, error);
        return { success: false, error: error.message };
    }
}

/**
 * Publish goal pose to AGV for navigation
 * @param {string} deviceId - AGV device identifier
 * @param {number} x - X coordinate
 * @param {number} y - Y coordinate
 * @param {number} orientation - Orientation in radians
 */
function publishGoal(deviceId, x, y, orientation = 0) {
    try {
        const topicName = `/${deviceId}${config.ROS2.TOPICS.GOAL_POSE}`;
        const publisher = getPublisher(topicName, PoseStamped);
        
        const goalPose = new PoseStamped();
        
        // Set header
        goalPose.header.stamp = rclnodejs.createMessageStamp();
        goalPose.header.frame_id = 'map';
        
        // Set position
        goalPose.pose.position.x = x;
        goalPose.pose.position.y = y;
        goalPose.pose.position.z = 0;
        
        // Set orientation (convert from yaw to quaternion)
        goalPose.pose.orientation.x = 0;
        goalPose.pose.orientation.y = 0;
        goalPose.pose.orientation.z = Math.sin(orientation / 2);
        goalPose.pose.orientation.w = Math.cos(orientation / 2);
        
        publisher.publish(goalPose);
        
        console.log(`Published goal to ${deviceId}: x=${x}, y=${y}, orientation=${orientation}`);
        
        return { success: true, x, y, orientation };
    } catch (error) {
        console.error(`Error publishing goal to ${deviceId}:`, error);
        return { success: false, error: error.message };
    }
}

/**
 * Set initial pose for localization
 * @param {string} deviceId - AGV device identifier
 * @param {number} x - X coordinate
 * @param {number} y - Y coordinate
 * @param {number} orientation - Orientation in radians
 */
function setInitialPose(deviceId, x, y, orientation = 0) {
    try {
        const topicName = `/${deviceId}/initialpose`;
        const publisher = getPublisher(topicName, PoseWithCovarianceStamped);
        
        const initialPose = new PoseWithCovarianceStamped();
        
        // Set header
        initialPose.header.stamp = rclnodejs.createMessageStamp();
        initialPose.header.frame_id = 'map';
        
        // Set pose
        initialPose.pose.pose.position.x = x;
        initialPose.pose.pose.position.y = y;
        initialPose.pose.pose.position.z = 0;
        
        initialPose.pose.pose.orientation.x = 0;
        initialPose.pose.pose.orientation.y = 0;
        initialPose.pose.pose.orientation.z = Math.sin(orientation / 2);
        initialPose.pose.pose.orientation.w = Math.cos(orientation / 2);
        
        // Set covariance (simplified - you may want to adjust based on your needs)
        const covariance = new Array(36).fill(0);
        covariance[0] = 0.25;   // x variance
        covariance[7] = 0.25;   // y variance
        covariance[35] = 0.07;  // yaw variance
        initialPose.pose.covariance = covariance;
        
        publisher.publish(initialPose);
        
        console.log(`Set initial pose for ${deviceId}: x=${x}, y=${y}, orientation=${orientation}`);
        
        return { success: true, x, y, orientation };
    } catch (error) {
        console.error(`Error setting initial pose for ${deviceId}:`, error);
        return { success: false, error: error.message };
    }
}

/**
 * Start mapping service call
 * @param {string} deviceId - AGV device identifier
 */
async function startMapping(deviceId) {
    try {
        const serviceName = `/${deviceId}${config.ROS2.SERVICES.START_MAPPING}`;
        const ServiceType = rclnodejs.require('std_srvs/srv/Empty');
        const client = getServiceClient(serviceName, ServiceType);
        
        // Wait for service to be available
        await client.waitForService(5000); // 5 seconds timeout
        
        const request = new ServiceType.Request();
        const response = await client.sendRequest(request);
        
        console.log(`Started mapping for ${deviceId}`);
        
        return { success: true, message: 'Mapping started' };
    } catch (error) {
        console.error(`Error starting mapping for ${deviceId}:`, error);
        return { success: false, error: error.message };
    }
}

/**
 * Stop mapping service call
 * @param {string} deviceId - AGV device identifier
 */
async function stopMapping(deviceId) {
    try {
        const serviceName = `/${deviceId}${config.ROS2.SERVICES.STOP_MAPPING}`;
        const ServiceType = rclnodejs.require('std_srvs/srv/Empty');
        const client = getServiceClient(serviceName, ServiceType);
        
        await client.waitForService(5000);
        
        const request = new ServiceType.Request();
        const response = await client.sendRequest(request);
        
        console.log(`Stopped mapping for ${deviceId}`);
        
        return { success: true, message: 'Mapping stopped' };
    } catch (error) {
        console.error(`Error stopping mapping for ${deviceId}:`, error);
        return { success: false, error: error.message };
    }
}

/**
 * Save current map
 * @param {string} deviceId - AGV device identifier
 * @param {string} mapName - Name for the saved map
 */
async function saveMap(deviceId, mapName) {
    try {
        const serviceName = `/${deviceId}${config.ROS2.SERVICES.SAVE_MAP}`;
        const ServiceType = rclnodejs.require('nav2_msgs/srv/SaveMap');
        const client = getServiceClient(serviceName, ServiceType);
        
        await client.waitForService(5000);
        
        const request = new ServiceType.Request();
        request.map_topic = `/${deviceId}/map`;
        request.map_url = mapName;
        request.image_format = 'pgm';
        request.map_mode = 'trinary';
        request.free_thresh = 0.25;
        request.occupied_thresh = 0.65;
        
        const response = await client.sendRequest(request);
        
        console.log(`Saved map for ${deviceId} as ${mapName}`);
        
        return { success: true, mapName, message: 'Map saved successfully' };
    } catch (error) {
        console.error(`Error saving map for ${deviceId}:`, error);
        return { success: false, error: error.message };
    }
}

/**
 * Emergency stop - immediately stop all movement
 * @param {string} deviceId - AGV device identifier
 */
function emergencyStop(deviceId) {
    return publishVelocity(deviceId, 0, 0);
}

/**
 * Cleanup publishers and service clients
 */
function cleanup() {
    publishers.clear();
    serviceClients.clear();
    console.log('Publishers and service clients cleaned up');
}

module.exports = {
    publishVelocity,
    publishGoal,
    setInitialPose,
    startMapping,
    stopMapping,
    saveMap,
    emergencyStop,
    cleanup
};