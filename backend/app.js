// app.js - Complete Application with Working ROS2 Integration
const express = require('express');
const http = require('http');
const cors = require('cors');
const bodyParser = require('body-parser');
const fs = require('fs-extra');
const path = require('path');

// Import configuration and utilities
const config = require('./config');
const { initializeWebSocketServer, broadcastToSubscribers } = require('./websocket/clientConnection');

// Import existing utilities
const controlRoutes = require('./routes/controlRoutes');
const storageManager = require('./ros/utils/storageManager');

// Import our WORKING ROS2 integration
const rclnodejs = require('rclnodejs');

// Create Express app and HTTP server
const app = express();
const server = http.createServer(app);

// Global data storage
global.connectedDevices = [];
global.deviceMaps = {};
global.deviceOrders = {};
global.orderQueue = {};
global.liveData = {};

// ROS2 integration variables
let rosNode = null;
let rosInitialized = false;
let rosPublishers = {};
let rosSubscribers = {};

// Middleware setup
app.use(cors({
    origin: [
        'http://localhost:37853',
        'http://localhost:3000',
        'http://localhost:8080', 
        'http://localhost:8000',
        'http://127.0.0.1:3000',
        'http://127.0.0.1:8080',
        '*' // Allow all for development
    ],
    methods: ['GET', 'POST', 'PUT', 'DELETE', 'PATCH', 'OPTIONS'],
    allowedHeaders: ['Content-Type', 'Authorization', 'Accept', 'Origin'],
    credentials: true
}));

app.options('*', cors()); // Handle preflight requests


app.use(bodyParser.json({ limit: config.PERFORMANCE.MAX_PAYLOAD_SIZE }));
app.use(bodyParser.urlencoded({ 
    extended: true, 
    limit: config.PERFORMANCE.MAX_PAYLOAD_SIZE 
}));

// Request logging
app.use((req, res, next) => {
    console.log(`${new Date().toISOString()} - ${req.method} ${req.path}`);
    next();
});

// Helper function for API responses
function formatApiResponse(success, data = null, message = '', error = null) {
    return {
        success,
        message,
        data,
        error,
        timestamp: new Date().toISOString()
    };
}

// ==========================================
// WORKING ROS2 INTEGRATION FUNCTIONS
// ==========================================

async function initializeROS2() {
    try {
        console.log('🤖 Initializing ROS2 integration...');
        
        // Use the working initialization method
        await rclnodejs.init();
        console.log('✅ ROS2 initialized successfully');
        
        // Create the node
        rosNode = rclnodejs.createNode('agv_fleet_backend');
        console.log('✅ ROS2 node created: agv_fleet_backend');
        
        // Start spinning to process callbacks
        rclnodejs.spin(rosNode);
        console.log('✅ ROS2 node spinning');
        
        // Create publishers and subscribers
        await createROS2Publishers();
        await createROS2Subscribers();
        
        rosInitialized = true;
        console.log('✅ Complete ROS2 integration initialized');
        return true;
        
    } catch (error) {
        console.error('⚠️ ROS2 initialization failed:', error.message);
        console.log('🔄 Backend will run without ROS2 (simulation mode)');
        rosInitialized = false;
        return false;
    }
}

async function createROS2Publishers() {
    try {
        console.log('📤 Creating ROS2 publishers...');
        
        // Velocity command publisher
        rosPublishers.cmdVel = rosNode.createPublisher(
            'geometry_msgs/msg/Twist', 
            '/cmd_vel',
            { depth: 10, reliability: 'reliable', durability: 'volatile' }
        );
        console.log('   ✅ cmd_vel publisher created');
        
        // Joystick publisher
        rosPublishers.joy = rosNode.createPublisher(
            'sensor_msgs/msg/Joy',
            '/joy',
            { depth: 10, reliability: 'reliable', durability: 'volatile' }
        );
        console.log('   ✅ joy publisher created');
        
        console.log(`📤 Created ${Object.keys(rosPublishers).length} publishers`);
        
    } catch (error) {
        console.error('❌ Failed to create publishers:', error);
        throw error;
    }
}

async function createROS2Subscribers() {
    try {
        console.log('📥 Creating ROS2 subscribers...');
        
        // Odometry subscriber
        rosSubscribers.odom = rosNode.createSubscription(
            'nav_msgs/msg/Odometry',
            '/diff_drive_controller/odom',
            { depth: 10, reliability: 'reliable', durability: 'volatile' },
            (odomMsg) => handleOdometryMessage(odomMsg)
        );
        console.log('   ✅ odometry subscriber created');
        
        // Command velocity feedback subscriber
        rosSubscribers.cmdVelFeedback = rosNode.createSubscription(
            'geometry_msgs/msg/Twist',
            '/diff_drive_controller/cmd_vel',
            { depth: 10, reliability: 'reliable', durability: 'volatile' },
            (twistMsg) => handleVelocityFeedback(twistMsg)
        );
        console.log('   ✅ cmd_vel feedback subscriber created');
        
        // Joint states subscriber
        rosSubscribers.jointStates = rosNode.createSubscription(
            'sensor_msgs/msg/JointState',
            '/joint_states',
            { depth: 10, reliability: 'reliable', durability: 'volatile' },
            (jointMsg) => handleJointStates(jointMsg)
        );
        console.log('   ✅ joint_states subscriber created');
        
        console.log(`📥 Created ${Object.keys(rosSubscribers).length} subscribers`);
        
    } catch (error) {
        console.error('❌ Failed to create subscribers:', error);
        throw error;
    }
}

// ROS2 Message Handlers
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
        updateLiveDataForAllDevices('odometry', processedOdom);
        
        // Broadcast via WebSocket
        broadcastToSubscribers('real_time_data', {
            type: 'odometry_update',
            data: processedOdom,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error processing odometry:', error);
    }
}

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
        
        updateLiveDataForAllDevices('velocity_feedback', velocityData);
        
        broadcastToSubscribers('real_time_data', {
            type: 'velocity_feedback',
            data: velocityData,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error processing velocity feedback:', error);
    }
}

function handleJointStates(jointMsg) {
    try {
        const jointData = {
            joint_names: jointMsg.name,
            positions: jointMsg.position,
            velocities: jointMsg.velocity,
            efforts: jointMsg.effort,
            timestamp: new Date().toISOString()
        };
        
        updateLiveDataForAllDevices('joint_states', jointData);
        
        broadcastToSubscribers('real_time_data', {
            type: 'joint_states_update',
            data: jointData,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error processing joint states:', error);
    }
}

// Utility functions
function quaternionToYaw(q) {
    return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

function updateLiveDataForAllDevices(dataType, data) {
    global.connectedDevices?.forEach(device => {
        if (!global.liveData[device.id]) {
            global.liveData[device.id] = {};
        }
        global.liveData[device.id][dataType] = data;
        
        // Update device last seen
        device.lastSeen = new Date().toISOString();
    });
}

// ROS2 Control Functions
function publishVelocityCommand(linear = 0.0, angular = 0.0) {
    try {
        if (!rosInitialized || !rosPublishers.cmdVel) {
            throw new Error('ROS2 not initialized or cmd_vel publisher not available');
        }
        
        const velocityMsg = {
            linear: { x: linear, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: angular }
        };
        
        rosPublishers.cmdVel.publish(velocityMsg);
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

function publishJoystickCommand(x, y, deadman = false) {
    try {
        if (!rosInitialized || !rosPublishers.joy) {
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
        
        rosPublishers.joy.publish(joyMsg);
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

// ==========================================
// API ENDPOINTS (EXISTING + NEW ROS2 CONTROL)
// ==========================================

// Health check endpoint (enhanced with ROS2 status)
app.get('/api/health', (req, res) => {
    const healthData = {
        status: 'running',
        timestamp: new Date().toISOString(),
        connectedDevices: global.connectedDevices?.length || 0,
        totalMaps: Object.keys(global.deviceMaps || {}).length,
        totalOrders: Object.values(global.deviceOrders || {}).reduce((acc, orders) => acc + orders.length, 0),
        ros2Status: rosInitialized ? 'active' : 'simulation',
        ros2Details: rosInitialized ? {
            nodeActive: !!rosNode,
            publishersCount: Object.keys(rosPublishers).length,
            subscribersCount: Object.keys(rosSubscribers).length,
            topicsDiscovered: rosNode ? rosNode.getTopicNamesAndTypes().length : 0
        } : null,
        version: '1.0.0',
        uptime: process.uptime()
    };
    
    res.json(formatApiResponse(true, healthData, 'AGV Fleet Management Backend is running'));
});

// Device Management (existing code preserved)
app.get('/api/devices', (req, res) => {
    const devicesWithLiveData = global.connectedDevices.map(device => ({
        ...device,
        liveData: global.liveData[device.id] || null,
        hasMap: !!global.deviceMaps[device.id],
        activeOrders: (global.deviceOrders[device.id] || []).filter(order => 
            ['pending', 'active'].includes(order.status)
        ).length,
        ros2Connected: rosInitialized
    }));

    res.json({
        success: true,
        devices: devicesWithLiveData,
        count: devicesWithLiveData.length
    });
});

app.post('/api/devices/connect', async (req, res) => {
    try {
        const { deviceId, deviceName, ipAddress } = req.body;

        if (!deviceId) {
            return res.status(400).json({
                success: false,
                message: 'Device ID is required'
            });
        }

        // Create device object
        const device = {
            id: deviceId,
            name: deviceName || `AGV_${deviceId}`,
            ip: ipAddress || config.AGV.DEFAULT_IP,
            status: 'connected',
            connectedAt: new Date().toISOString(),
            lastSeen: new Date().toISOString(),
            capabilities: ['navigation', 'mapping', 'joystick_control'],
            version: '1.0.0',
            ros2Connected: rosInitialized
        };

        // Remove existing device with same ID
        global.connectedDevices = global.connectedDevices.filter(d => d.id !== deviceId);
        global.connectedDevices.push(device);

        // Initialize device data structures
        if (!global.deviceOrders[deviceId]) {
            global.deviceOrders[deviceId] = [];
        }
        if (!global.orderQueue[deviceId]) {
            global.orderQueue[deviceId] = { current: null, pending: [] };
        }
        if (!global.liveData[deviceId]) {
            global.liveData[deviceId] = {};
        }

        // Save to persistent storage
        await savePersistedData();

        console.log(`✅ Device connected: ${deviceId} (${device.name})`);

        res.json({
            success: true,
            message: 'Device connected successfully',
            device,
            ros2_integration: rosInitialized
        });

    } catch (error) {
        console.error('❌ Error connecting device:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to connect device',
            error: error.message
        });
    }
});

app.delete('/api/devices/:deviceId', async (req, res) => {
    try {
        const { deviceId } = req.params;

        // Remove device
        global.connectedDevices = global.connectedDevices.filter(d => d.id !== deviceId);
        
        // Clean up data
        delete global.liveData[deviceId];
        delete global.orderQueue[deviceId];

        // Save to persistent storage
        await savePersistedData();

        res.json({
            success: true,
            message: 'Device disconnected successfully'
        });

    } catch (error) {
        console.error('❌ Error disconnecting device:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to disconnect device',
            error: error.message
        });
    }
});

// NEW ROS2 Control Endpoints
app.post('/api/control/move', async (req, res) => {
    try {
        const { deviceId, linear, angular } = req.body;

        if (!deviceId) {
            return res.status(400).json(formatApiResponse(false, null, 'Device ID is required'));
        }

        if (!rosInitialized) {
            return res.status(503).json(formatApiResponse(false, null, 'ROS2 not connected'));
        }

        const result = publishVelocityCommand(linear || 0, angular || 0);
        
        if (result.success) {
            res.json(formatApiResponse(true, result.data, 'Movement command sent successfully'));
        } else {
            res.status(500).json(formatApiResponse(false, null, 'Failed to send movement command', result.error));
        }

    } catch (error) {
        console.error('❌ Error sending movement command:', error);
        res.status(500).json(formatApiResponse(false, null, 'Internal server error', error.message));
    }
});

app.post('/api/control/stop', async (req, res) => {
    try {
        const { deviceId } = req.body;

        if (!deviceId) {
            return res.status(400).json(formatApiResponse(false, null, 'Device ID is required'));
        }

        if (!rosInitialized) {
            return res.status(503).json(formatApiResponse(false, null, 'ROS2 not connected'));
        }

        const result = publishVelocityCommand(0.0, 0.0); // Emergency stop
        
        if (result.success) {
            res.json(formatApiResponse(true, result.data, 'Stop command sent successfully'));
        } else {
            res.status(500).json(formatApiResponse(false, null, 'Failed to send stop command', result.error));
        }

    } catch (error) {
        console.error('❌ Error sending stop command:', error);
        res.status(500).json(formatApiResponse(false, null, 'Internal server error', error.message));
    }
});

app.post('/api/control/joystick', async (req, res) => {
    try {
        const { deviceId, x, y, deadman } = req.body;

        if (!deviceId) {
            return res.status(400).json(formatApiResponse(false, null, 'Device ID is required'));
        }

        if (!rosInitialized) {
            return res.status(503).json(formatApiResponse(false, null, 'ROS2 not connected'));
        }

        const result = publishJoystickCommand(x || 0, y || 0, deadman || false);
        
        if (result.success) {
            res.json(formatApiResponse(true, result.data, 'Joystick command sent successfully'));
        } else {
            res.status(500).json(formatApiResponse(false, null, 'Failed to send joystick command', result.error));
        }

    } catch (error) {
        console.error('❌ Error sending joystick command:', error);
        res.status(500).json(formatApiResponse(false, null, 'Internal server error', error.message));
    }
});

// ROS2 Status Endpoints
app.get('/api/ros2/status', (req, res) => {
    if (rosInitialized) {
        const topics = rosNode ? rosNode.getTopicNamesAndTypes() : [];
        const status = {
            initialized: rosInitialized,
            nodeActive: !!rosNode,
            publishersCount: Object.keys(rosPublishers).length,
            subscribersCount: Object.keys(rosSubscribers).length,
            topicsDiscovered: topics.length,
            availableTopics: topics.map(t => t.name)
        };
        res.json(formatApiResponse(true, status, 'ROS2 status retrieved successfully'));
    } else {
        res.json(formatApiResponse(false, null, 'ROS2 not initialized'));
    }
});

app.get('/api/ros2/test', async (req, res) => {
    try {
        if (!rosInitialized) {
            return res.status(503).json(formatApiResponse(false, null, 'ROS2 not initialized'));
        }

        const topics = rosNode.getTopicNamesAndTypes();
        const agvTopics = ['/cmd_vel', '/diff_drive_controller/odom'];
        
        const testResult = {
            totalTopics: topics.length,
            agvTopicsFound: agvTopics.filter(topic => 
                topics.some(t => t.name === topic)
            ),
            publishersActive: Object.keys(rosPublishers).length,
            subscribersActive: Object.keys(rosSubscribers).length,
            canPublish: !!rosPublishers.cmdVel,
            timestamp: new Date().toISOString()
        };
        
        res.json(formatApiResponse(true, testResult, 'ROS2 connectivity test completed'));

    } catch (error) {
        res.status(500).json(formatApiResponse(false, null, 'ROS2 test failed', error.message));
    }
});

// Map Management (existing code preserved)
app.get('/api/maps/:deviceId', (req, res) => {
    const { deviceId } = req.params;
    const mapData = global.deviceMaps[deviceId];

    if (mapData) {
        res.json({ 
            success: true, 
            mapData,
            hasLiveMap: !!(global.liveData[deviceId] && global.liveData[deviceId].map)
        });
    } else {
        res.status(404).json({ 
            success: false, 
            message: 'Map not found' 
        });
    }
});

app.post('/api/maps/save', async (req, res) => {
    try {
        const { deviceId, mapData, mapName } = req.body;

        if (!deviceId || !mapData) {
            return res.status(400).json({
                success: false,
                message: 'Device ID and map data are required'
            });
        }

        // Enhanced map data with metadata
        const enhancedMapData = {
            ...mapData,
            metadata: {
                deviceId,
                name: mapName || `Map_${deviceId}_${Date.now()}`,
                savedAt: new Date().toISOString(),
                version: (global.deviceMaps[deviceId]?.metadata?.version || 0) + 1,
                resolution: mapData.info?.resolution || config.MAPS.DEFAULT_RESOLUTION,
                dimensions: {
                    width: mapData.info?.width || 0,
                    height: mapData.info?.height || 0
                }
            },
            shapes: mapData.shapes || [],
            annotations: mapData.annotations || []
        };

        global.deviceMaps[deviceId] = enhancedMapData;

        // Save to persistent storage
        await savePersistedData();

        console.log(`🗺️ Map saved for device: ${deviceId}`);

        res.json({
            success: true,
            message: 'Map saved successfully',
            mapData: enhancedMapData
        });

    } catch (error) {
        console.error('❌ Error saving map:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to save map',
            error: error.message
        });
    }
});

app.post('/api/maps/update', async (req, res) => {
    try {
        const { deviceId, mapUpdate } = req.body;

        if (!deviceId || !mapUpdate) {
            return res.status(400).json({
                success: false,
                message: 'Device ID and map update are required'
            });
        }

        // Initialize map if it doesn't exist
        if (!global.deviceMaps[deviceId]) {
            global.deviceMaps[deviceId] = {
                shapes: [],
                annotations: [],
                metadata: { deviceId }
            };
        }

        // Apply incremental update
        if (mapUpdate.shapes) {
            global.deviceMaps[deviceId].shapes = mapUpdate.shapes;
        }
        if (mapUpdate.annotations) {
            global.deviceMaps[deviceId].annotations = mapUpdate.annotations;
        }
        if (mapUpdate.metadata) {
            global.deviceMaps[deviceId].metadata = {
                ...global.deviceMaps[deviceId].metadata,
                ...mapUpdate.metadata,
                lastUpdated: new Date().toISOString()
            };
        }

        // Save to persistent storage
        await savePersistedData();

        // Broadcast real-time update via WebSocket
        broadcastToSubscribers('map_events', {
            type: 'map_updated',
            deviceId,
            update: mapUpdate,
            timestamp: new Date().toISOString()
        });

        res.json({
            success: true,
            message: 'Map updated successfully',
            mapData: global.deviceMaps[deviceId]
        });

    } catch (error) {
        console.error('❌ Error updating map:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to update map',
            error: error.message
        });
    }
});

// Order Management (existing code preserved)
app.get('/api/orders/:deviceId', (req, res) => {
    const { deviceId } = req.params;
    const orders = global.deviceOrders[deviceId] || [];
    const queue = global.orderQueue[deviceId] || { current: null, pending: [] };

    res.json({
        success: true,
        orders,
        queue,
        count: orders.length
    });
});

app.post('/api/orders/create', async (req, res) => {
    try {
        const { deviceId, waypoints, orderName, priority } = req.body;

        if (!deviceId || !waypoints || !Array.isArray(waypoints) || waypoints.length === 0) {
            return res.status(400).json({
                success: false,
                message: 'Device ID and waypoints array are required'
            });
        }

        if (waypoints.length > config.ORDERS.MAX_WAYPOINTS) {
            return res.status(400).json({
                success: false,
                message: `Maximum ${config.ORDERS.MAX_WAYPOINTS} waypoints allowed`
            });
        }

        // Create order
        const order = {
            id: `order_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
            deviceId,
            name: orderName || `Order_${Date.now()}`,
            waypoints: waypoints.map((wp, index) => ({
                id: `wp_${index}`,
                ...wp,
                visited: false,
                visitedAt: null
            })),
            priority: priority || 'normal',
            status: 'pending',
            createdAt: new Date().toISOString(),
            estimatedDuration: waypoints.length * 30000, // 30s per waypoint estimate
            progress: {
                currentWaypoint: 0,
                completedWaypoints: 0,
                totalWaypoints: waypoints.length
            }
        };

        // Add to device orders
        if (!global.deviceOrders[deviceId]) {
            global.deviceOrders[deviceId] = [];
        }
        global.deviceOrders[deviceId].push(order);

        // Add to queue management
        if (!global.orderQueue[deviceId]) {
            global.orderQueue[deviceId] = { current: null, pending: [] };
        }

        // If no current order, start this one immediately
        if (!global.orderQueue[deviceId].current) {
            global.orderQueue[deviceId].current = order.id;
            order.status = 'active';
            order.startedAt = new Date().toISOString();
        } else {
            global.orderQueue[deviceId].pending.push(order.id);
        }

        // Save to persistent storage
        await savePersistedData();

        console.log(`📋 Order created for device: ${deviceId} (${order.id})`);

        res.json({
            success: true,
            message: 'Order created successfully',
            order
        });

    } catch (error) {
        console.error('❌ Error creating order:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to create order',
            error: error.message
        });
    }
});

app.patch('/api/orders/:orderId/status', async (req, res) => {
    try {
        const { orderId } = req.params;
        const { status } = req.body;

        if (!config.ORDERS.STATUSES.includes(status)) {
            return res.status(400).json({
                success: false,
                message: `Invalid status. Must be one of: ${config.ORDERS.STATUSES.join(', ')}`
            });
        }

        // Find and update order
        let orderFound = false;
        let deviceId = null;

        for (const [devId, orders] of Object.entries(global.deviceOrders)) {
            const orderIndex = orders.findIndex(order => order.id === orderId);
            if (orderIndex !== -1) {
                const order = orders[orderIndex];
                const oldStatus = order.status;
                
                order.status = status;
                order.updatedAt = new Date().toISOString();

                // Handle status-specific logic
                if (status === 'completed') {
                    order.completedAt = new Date().toISOString();
                    order.progress.completedWaypoints = order.progress.totalWaypoints;
                    
                    // Start next order in queue
                    const queue = global.orderQueue[devId];
                    if (queue && queue.current === orderId && queue.pending.length > 0) {
                        const nextOrderId = queue.pending.shift();
                        queue.current = nextOrderId;
                        
                        // Update next order status
                        const nextOrder = orders.find(o => o.id === nextOrderId);
                        if (nextOrder) {
                            nextOrder.status = 'active';
                            nextOrder.startedAt = new Date().toISOString();
                        }
                    } else if (queue && queue.current === orderId) {
                        queue.current = null;
                    }
                } else if (status === 'failed' || status === 'cancelled') {
                    order.failedAt = new Date().toISOString();
                    
                    // Handle queue management for failed/cancelled orders
                    const queue = global.orderQueue[devId];
                    if (queue && queue.current === orderId) {
                        if (queue.pending.length > 0) {
                            const nextOrderId = queue.pending.shift();
                            queue.current = nextOrderId;
                            
                            const nextOrder = orders.find(o => o.id === nextOrderId);
                            if (nextOrder) {
                                nextOrder.status = 'active';
                                nextOrder.startedAt = new Date().toISOString();
                            }
                        } else {
                            queue.current = null;
                        }
                    }
                }

                orderFound = true;
                deviceId = devId;
                
                console.log(`📋 Order ${orderId} status changed: ${oldStatus} → ${status}`);
                break;
            }
        }

        if (!orderFound) {
            return res.status(404).json({
                success: false,
                message: 'Order not found'
            });
        }

        // Save to persistent storage
        await savePersistedData();

        // Broadcast status change
        broadcastToSubscribers('order_events', {
            type: 'order_status_changed',
            orderId,
            deviceId,
            status,
            timestamp: new Date().toISOString()
        });

        res.json({
            success: true,
            message: 'Order status updated successfully'
        });

    } catch (error) {
        console.error('❌ Error updating order status:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to update order status',
            error: error.message
        });
    }
});

// Live data endpoints
app.get('/api/live/:deviceId', (req, res) => {
    const { deviceId } = req.params;
    const liveData = global.liveData[deviceId];

    if (liveData) {
        res.json({ 
            success: true, 
            data: liveData,
            timestamp: new Date().toISOString()
        });
    } else {
        res.status(404).json({ 
            success: false, 
            message: 'No live data available for this device'
        });
    }
});

// Use existing control routes
app.use('/api/control', controlRoutes);

// Initialize storage directories
async function initializeStorage() {
    try {
        await storageManager.initialize();
        console.log('✅ Storage initialized via StorageManager');
    } catch (error) {
        console.error('❌ Storage initialization failed:', error);
    }
}

// Save data to persistent storage
async function savePersistedData() {
    try {
        await storageManager.saveAllData();
    } catch (error) {
        console.error('❌ Error saving persisted data:', error);
    }
}

// Error handling middleware
app.use((err, req, res, next) => {
    console.error('❌ Server error:', err);
    
    // Determine error type and status code
    let statusCode = 500;
    let message = 'Internal server error';
    
    if (err.statusCode) {
        statusCode = err.statusCode;
        message = err.message;
    } else if (err.name === 'ValidationError') {
        statusCode = 400;
        message = 'Validation error';
    } else if (err.name === 'CastError') {
        statusCode = 400;
        message = 'Invalid data format';
    } else if (err.code === 'ENOTFOUND' || err.code === 'ECONNREFUSED') {
        statusCode = 503;
        message = 'Service unavailable';
    }
    
    const errorResponse = {
        success: false,
        message: message,
        error: config.NODE_ENV === 'development' ? {
            stack: err.stack,
            details: err.message,
            type: err.name || 'UnknownError'
        } : {
            type: err.name || 'ServerError'
        },
        timestamp: new Date().toISOString(),
        path: req.path,
        method: req.method
    };
    
    res.status(statusCode).json(errorResponse);
});

// 404 handler
app.use('*', (req, res) => {
    res.status(404).json({
        success: false,
        message: 'Endpoint not found'
    });
});

// Server startup
async function startServer() {
    try {
        console.log('🚀 Starting AGV Fleet Management Server...');
        
        // Initialize storage
        await initializeStorage();
        
        // Initialize ROS2 (working version)
        await initializeROS2();
        
        // Initialize WebSocket server
        initializeWebSocketServer(server);
        
        // Start HTTP server
        server.listen(config.PORT, config.HOST, () => {
            console.log(`✅ AGV Fleet Management Server running on ${config.HOST}:${config.PORT}`);
            console.log(`🌐 Health check: http://${config.HOST}:${config.PORT}/api/health`);
            console.log(`🔗 WebSocket ready for real-time communication`);
            console.log(`🤖 ROS2 Integration: ${rosInitialized ? 'Active' : 'Simulation Mode'}`);
            
            if (rosInitialized) {
                console.log('📡 Available ROS2 endpoints:');
                console.log('   POST /api/control/move - Send movement commands');
                console.log('   POST /api/control/stop - Emergency stop');
                console.log('   POST /api/control/joystick - Joystick control');
                console.log('   GET  /api/ros2/status - ROS2 status');
                console.log('   GET  /api/ros2/test - Test ROS2 connectivity');
            }
        });

    } catch (error) {
        console.error('❌ Failed to start server:', error);
        process.exit(1);
    }
}

// Graceful shutdown
process.on('SIGINT', async () => {
    console.log('\n🛑 Shutting down server...');
    
    try {
        // Cleanup storage manager (includes final save)
        await storageManager.cleanup();
        
        // Cleanup ROS2
        if (rosInitialized) {
            console.log('🤖 Shutting down ROS2...');
            
            // Clean up publishers
            Object.keys(rosPublishers).forEach(key => {
                try {
                    rosPublishers[key].destroy();
                } catch (e) {
                    // Ignore cleanup errors
                }
            });
            
            // Clean up subscribers
            Object.keys(rosSubscribers).forEach(key => {
                try {
                    rosSubscribers[key].destroy();
                } catch (e) {
                    // Ignore cleanup errors
                }
            });
            
            // Destroy node and shutdown
            if (rosNode) {
                rosNode.destroy();
            }
            await rclnodejs.shutdown();
            
            console.log('✅ ROS2 shutdown complete');
        }
        
        // Close server
        server.close(() => {
            console.log('✅ Server shut down gracefully');
            process.exit(0);
        });
        
    } catch (error) {
        console.error('❌ Error during shutdown:', error);
        process.exit(1);
    }
});

// Start the server
startServer();