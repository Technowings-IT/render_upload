// ros/utils/ros_connection.js - Integrated with publishers and subscribers
const rclnodejs = require('rclnodejs');

let rosNode = null;
let isInitialized = false;

// Import our modular publishers and subscribers
const publishers = require('./publishers');
const subscribers = require('./subscribers');

async function initializeROS() {
    try {
        console.log('🤖 Initializing ROS2 connection...');
        
        // Initialize ROS2
        await rclnodejs.init();
        console.log('✅ ROS2 initialized successfully');
        
        // Create the node
        rosNode = rclnodejs.createNode('agv_fleet_backend');
        console.log('✅ ROS2 node created: agv_fleet_backend');
        
        // Initialize publishers and subscribers with the node
        publishers.initializePublishers(rosNode);
        subscribers.initializeSubscribers(rosNode);
        
        // Start spinning to process callbacks
        rclnodejs.spin(rosNode);
        console.log('✅ ROS2 node spinning');
        
        // Subscribe to all essential topics
        subscribers.subscribeToAllTopics();
        
        // Initialize global data structures
        initializeGlobalData();
        
        isInitialized = true;
        console.log('✅ Complete ROS2 integration initialized');
        return true;
        
    } catch (error) {
        console.error('❌ Failed to initialize ROS2:', error);
        isInitialized = false;
        throw error;
    }
}

function initializeGlobalData() {
    // Initialize global data structures for AGV management
    if (!global.connectedDevices) {
        global.connectedDevices = [];
    }
    
    if (!global.liveData) {
        global.liveData = {};
    }
    
    if (!global.deviceOrders) {
        global.deviceOrders = {};
    }
    
    if (!global.orderQueue) {
        global.orderQueue = {};
    }
    
    if (!global.deviceMaps) {
        global.deviceMaps = {};
    }
    
    if (!global.deviceMappingStates) {
        global.deviceMappingStates = {};
    }
    
    if (!global.robotTrails) {
        global.robotTrails = {};
    }
    
    console.log('📊 Global data structures initialized');
}

// Public API functions that delegate to publishers
function publishVelocity(linear = 0.0, angular = 0.0) {
    if (!isInitialized) {
        throw new Error('ROS2 not initialized');
    }
    return publishers.publishVelocity(linear, angular);
}

function publishGoal(x, y, orientation = 0) {
    if (!isInitialized) {
        throw new Error('ROS2 not initialized');
    }
    return publishers.publishGoal(x, y, orientation);
}

function publishMap(mapData) {
    if (!isInitialized) {
        throw new Error('ROS2 not initialized');
    }
    return publishers.publishMap(mapData);
}

function startMapping() {
    if (!isInitialized) {
        throw new Error('ROS2 not initialized');
    }
    
    // Set mapping state
    global.deviceMappingStates['agv_01'] = {
        active: true,
        startedAt: new Date().toISOString()
    };
    
    return publishers.startMapping();
}

function stopMapping() {
    if (!isInitialized) {
        throw new Error('ROS2 not initialized');
    }
    
    // Update mapping state
    if (global.deviceMappingStates['agv_01']) {
        global.deviceMappingStates['agv_01'].active = false;
        global.deviceMappingStates['agv_01'].stoppedAt = new Date().toISOString();
    }
    
    return publishers.stopMapping();
}

function emergencyStop() {
    return publishVelocity(0.0, 0.0);
}

// Enhanced joystick control with deadman switch
function publishJoystick(x, y, deadman = false) {
    try {
        if (!isInitialized) {
            throw new Error('ROS2 not initialized');
        }
        
        // Only send velocity if deadman switch is active
        if (deadman) {
            // Convert joystick values to velocity
            const linear = y; // Forward/backward
            const angular = -x; // Left/right (inverted for correct rotation)
            
            return publishVelocity(linear, angular);
        } else {
            // Stop the robot if deadman not active
            return publishVelocity(0.0, 0.0);
        }
        
    } catch (error) {
        console.error('❌ Failed to publish joystick command:', error);
        return {
            success: false,
            error: error.message
        };
    }
}

// Get ROS2 status
function getROS2Status() {
    if (!rosNode) {
        return {
            initialized: false,
            nodeActive: false,
            topicsDiscovered: 0,
            availableTopics: [],
            error: 'Node not created'
        };
    }
    
    try {
        const topics = rosNode.getTopicNamesAndTypes();
        
        return {
            initialized: isInitialized,
            nodeActive: rosNode !== null,
            topicsDiscovered: topics.length,
            availableTopics: topics.map(t => t.name),
            mappingActive: global.deviceMappingStates?.['agv_01']?.active || false,
            connectedDevices: global.connectedDevices?.length || 0,
            liveDataKeys: Object.keys(global.liveData || {}),
            lastUpdate: new Date().toISOString()
        };
    } catch (error) {
        return {
            initialized: isInitialized,
            nodeActive: false,
            error: error.message,
            lastUpdate: new Date().toISOString()
        };
    }
}

// Test connectivity with your specific topics
async function testConnectivity() {
    try {
        if (!isInitialized) {
            throw new Error('ROS2 not initialized');
        }
        
        const topics = rosNode.getTopicNamesAndTypes();
        const requiredTopics = ['/cmd_vel', '/pos', '/map', '/joint_states'];
        const optionalTopics = ['/diff_drive_controller/odom', '/battery_state'];
        
        const foundRequired = requiredTopics.filter(topic => 
            topics.some(t => t.name === topic)
        );
        
        const foundOptional = optionalTopics.filter(topic => 
            topics.some(t => t.name === topic)
        );
        
        const results = {
            totalTopics: topics.length,
            requiredTopicsFound: foundRequired,
            optionalTopicsFound: foundOptional,
            missingRequired: requiredTopics.filter(topic => !foundRequired.includes(topic)),
            canPublishVelocity: true, // We can always try to publish
            mappingActive: global.deviceMappingStates?.['agv_01']?.active || false,
            rosStatus: 'connected',
            timestamp: new Date().toISOString()
        };
        
        console.log('🔍 ROS2 Connectivity Test Results:');
        console.log(`   📊 Total topics: ${results.totalTopics}`);
        console.log(`   ✅ Required topics found: ${foundRequired.join(', ')}`);
        console.log(`   📍 Optional topics found: ${foundOptional.join(', ')}`);
        
        if (results.missingRequired.length > 0) {
            console.log(`   ⚠️ Missing required topics: ${results.missingRequired.join(', ')}`);
        }
        
        return results;
        
    } catch (error) {
        console.error('❌ ROS2 connectivity test failed:', error);
        return { 
            success: false, 
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

// Device management functions
function addConnectedDevice(deviceInfo) {
    const deviceId = deviceInfo.id || 'agv_01';
    
    // Check if device already exists
    const existingIndex = global.connectedDevices.findIndex(d => d.id === deviceId);
    
    if (existingIndex >= 0) {
        // Update existing device
        global.connectedDevices[existingIndex] = {
            ...global.connectedDevices[existingIndex],
            ...deviceInfo,
            status: 'connected',
            lastSeen: new Date().toISOString(),
            reconnectedAt: new Date().toISOString()
        };
    } else {
        // Add new device
        global.connectedDevices.push({
            id: deviceId,
            ...deviceInfo,
            status: 'connected',
            connectedAt: new Date().toISOString(),
            lastSeen: new Date().toISOString()
        });
    }
    
    // Initialize device data structures
    if (!global.liveData[deviceId]) {
        global.liveData[deviceId] = {};
    }
    if (!global.deviceOrders[deviceId]) {
        global.deviceOrders[deviceId] = [];
    }
    if (!global.orderQueue[deviceId]) {
        global.orderQueue[deviceId] = { current: null, pending: [] };
    }
    if (!global.deviceMappingStates[deviceId]) {
        global.deviceMappingStates[deviceId] = { active: false };
    }
    if (!global.robotTrails[deviceId]) {
        global.robotTrails[deviceId] = [];
    }
    
    console.log(`✅ Device ${deviceId} connected and initialized`);
    return deviceId;
}

// Get live data for all devices or specific device
function getLiveData(deviceId = null) {
    if (deviceId) {
        return global.liveData[deviceId] || {};
    }
    return global.liveData || {};
}

// Get current mapping status
function getMappingStatus(deviceId = 'agv_01') {
    return {
        active: global.deviceMappingStates?.[deviceId]?.active || false,
        startedAt: global.deviceMappingStates?.[deviceId]?.startedAt,
        stoppedAt: global.deviceMappingStates?.[deviceId]?.stoppedAt,
        trailLength: global.robotTrails?.[deviceId]?.length || 0
    };
}

async function shutdown() {
    try {
        if (rosNode) {
            console.log('🛑 Shutting down ROS2 integration...');
            
            // Cleanup publishers and subscribers
            publishers.cleanup();
            subscribers.cleanup();
            
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

// Export the node for use by other modules
function getNode() {
    return rosNode;
}

module.exports = {
    initializeROS,
    shutdown,
    getNode,
    
    // Control functions
    publishVelocity,
    publishJoystick,
    publishGoal,
    publishMap,
    emergencyStop,
    
    // Mapping functions
    startMapping,
    stopMapping,
    getMappingStatus,
    
    // Device management
    addConnectedDevice,
    getLiveData,
    
    // Status functions
    getROS2Status,
    testConnectivity,
    isRosInitialized: () => isInitialized
};