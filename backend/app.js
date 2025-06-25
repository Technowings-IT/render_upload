// app.js - UPDATED with device discovery integration
const express = require('express');
const http = require('http');
const cors = require('cors');
const config = require('./config');

// Import core modules
const rosConnection = require('./ros/utils/ros_connection');
const storageManager = require('./ros/utils/storageManager');
const { initializeWebSocketServer } = require('./websocket/clientConnection');

// Import routes
const controlRoutes = require('./routes/controlRoutes');
const { router: discoveryRoutes, initializeUDPDiscovery } = require('./routes/discoveryRoutes');

const app = express();
const server = http.createServer(app);

// Global variables initialization
global.connectedDevices = [];
global.liveData = {};
global.deviceOrders = {};
global.orderQueue = {};
global.deviceMaps = {};
global.deviceMappingStates = {};
global.robotTrails = {};

// ✅ FIXED: Enhanced CORS middleware
app.use(cors({
    origin: function (origin, callback) {
        // Allow requests with no origin (mobile apps, Postman, etc.)
        if (!origin) return callback(null, true);
        
        // Allow all origins in development
        if (config.SERVER.ENV === 'development') {
            return callback(null, true);
        }
        
        // Check allowed origins
        if (config.CORS.ALLOWED_ORIGINS.includes(origin) || 
            config.CORS.ALLOWED_ORIGINS.includes('*')) {
            return callback(null, true);
        }
        
        // Allow 192.168.253.x subnet
        if (origin.includes('192.168.253.')) {
            return callback(null, true);
        }
        
        return callback(new Error('Not allowed by CORS'));
    },
    credentials: config.CORS.CREDENTIALS,
    methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
    allowedHeaders: ['Content-Type', 'Authorization', 'X-Requested-With']
}));

app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true, limit: '10mb' }));

// Request logging middleware
app.use((req, res, next) => {
    console.log(`📝 ${req.method} ${req.path} - ${req.ip || 'unknown'} - ${new Date().toISOString()}`);
    next();
});

// ✅ NEW: Add discovery routes
app.use('/api/discovery', discoveryRoutes);

// Existing routes
app.use('/api/control', controlRoutes);

// ✅ ENHANCED: Health check endpoint
app.get('/health', (req, res) => {
    try {
        const rosStatus = rosConnection.getROS2Status();
        const publisherStats = require('./ros/utils/publishers').getPublisherStats();
        
        res.json({
            success: true,
            status: 'healthy',
            timestamp: new Date().toISOString(),
            server: {
                host: config.SERVER.HOST,
                port: config.SERVER.PORT,
                uptime: process.uptime(),
                nodeVersion: process.version,
                env: config.SERVER.ENV
            },
            services: {
                ros2: {
                    initialized: rosStatus.initialized,
                    nodeActive: rosStatus.nodeActive,
                    topicsDiscovered: rosStatus.topicsDiscovered,
                    publishersActive: publisherStats.totalPublishers
                },
                webSocket: {
                    active: true,
                    url: `ws://${req.get('host')}`
                },
                storage: {
                    initialized: storageManager.initialized || false,
                    devicesCount: global.connectedDevices?.length || 0,
                    mapsCount: Object.keys(global.deviceMaps || {}).length
                },
                discovery: {
                    http: true,
                    udp: true,
                    networkScan: true
                }
            },
            capabilities: config.DEVICE.CAPABILITIES,
            version: '1.0.0'
        });
    } catch (error) {
        console.error('❌ Health check error:', error);
        res.status(500).json({ 
            success: false,
            error: 'Health check failed',
            details: error.message 
        });
    }
});

// ✅ ENHANCED: Get all connected devices
app.get('/api/devices', (req, res) => {
    try {
        const devices = global.connectedDevices.map(device => ({
            ...device,
            liveData: global.liveData[device.id] || {},
            mappingStatus: global.deviceMappingStates[device.id] || { active: false },
            orderCount: global.deviceOrders[device.id]?.length || 0,
            isOnline: !!global.liveData[device.id]?.lastUpdate
        }));
        
        res.json({
            success: true,
            devices: devices,
            total: devices.length,
            connected: devices.filter(d => d.status === 'connected').length,
            mapping: devices.filter(d => d.mappingStatus?.active).length,
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error('❌ Error getting devices:', error);
        res.status(500).json({ 
            success: false,
            error: 'Failed to get devices',
            details: error.message 
        });
    }
});

// ✅ ENHANCED: System status endpoint
app.get('/api/system/status', async (req, res) => {
    try {
        const rosStatus = rosConnection.getROS2Status();
        const connectivityTest = await rosConnection.testConnection();
        const publisherStats = require('./ros/utils/publishers').getPublisherStats();
        
        res.json({
            success: true,
            system: {
                server: {
                    uptime: process.uptime(),
                    memory: process.memoryUsage(),
                    cpu: process.cpuUsage(),
                    nodeVersion: process.version
                },
                ros2: {
                    ...rosStatus,
                    connectivity: connectivityTest,
                    publishers: publisherStats
                },
                devices: {
                    total: global.connectedDevices?.length || 0,
                    connected: global.connectedDevices?.filter(d => d.status === 'connected').length || 0,
                    liveDataActive: Object.keys(global.liveData || {}).length
                },
                mapping: {
                    activeDevices: Object.values(global.deviceMappingStates || {}).filter(s => s.active).length,
                    totalMaps: Object.keys(global.deviceMaps || {}).length
                },
                websocket: {
                    url: `ws://${req.get('host')}`,
                    active: true
                }
            },
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error('❌ Error getting system status:', error);
        res.status(500).json({ 
            success: false,
            error: 'Failed to get system status',
            details: error.message 
        });
    }
});

// ✅ NEW: Quick connect endpoint for AGVs
app.post('/api/connect', async (req, res) => {
    try {
        const clientIP = req.ip || req.connection.remoteAddress || req.socket.remoteAddress;
        const { deviceId, name } = req.body;
        
        const autoDeviceId = deviceId || 'agv_01';
        const deviceName = name || 'Primary AGV';
        
        const deviceInfo = {
            id: autoDeviceId,
            name: deviceName,
            type: 'differential_drive',
            ipAddress: clientIP,
            capabilities: config.DEVICE.CAPABILITIES,
            quickConnected: true,
            connectedAt: new Date().toISOString()
        };
        
        const connectedDeviceId = rosConnection.addConnectedDevice(deviceInfo);
        
        try {
            await storageManager.saveDevice(connectedDeviceId);
        } catch (storageError) {
            console.warn('⚠️ Could not save device to storage:', storageError.message);
        }
        
        res.json({
            success: true,
            message: 'Device connected successfully',
            deviceId: connectedDeviceId,
            device: deviceInfo,
            websocket: {
                url: `ws://${req.get('host')}`,
                connectMessage: {
                    type: 'device_connect',
                    deviceId: connectedDeviceId,
                    deviceInfo: deviceInfo
                }
            },
            timestamp: new Date().toISOString()
        });
        
        console.log(`✅ Quick-connected device: ${connectedDeviceId} from ${clientIP}`);
        
    } catch (error) {
        console.error('❌ Error quick-connecting device:', error);
        res.status(500).json({ 
            success: false,
            error: 'Failed to connect device',
            details: error.message 
        });
    }
});

// Error handling middleware
app.use((err, req, res, next) => {
    console.error('❌ Application error:', err);
    res.status(500).json({
        success: false,
        error: 'Internal server error',
        message: config.SERVER.ENV === 'development' ? err.message : 'Something went wrong',
        timestamp: new Date().toISOString()
    });
});

// 404 handler
app.use((req, res) => {
    res.status(404).json({ 
        success: false,
        error: 'Endpoint not found',
        path: req.path,
        method: req.method,
        timestamp: new Date().toISOString()
    });
});

async function initializeApplication() {
    console.log('🚀 Initializing AGV Fleet Management System...');
    
    try {
        // 1. Initialize storage manager
        console.log('📂 Initializing storage manager...');
        try {
            await storageManager.initialize();
            console.log('✅ Storage manager initialized');
        } catch (storageError) {
            console.warn('⚠️ Storage manager initialization failed:', storageError.message);
        }
        
        // 2. Initialize ROS2 connection
        console.log('🤖 Initializing ROS2 connection...');
        await rosConnection.initializeROS();
        console.log('✅ ROS2 connection established');
        
        // 3. Test ROS2 connectivity
        console.log('🔍 Testing ROS2 connectivity...');
        const connectivityTest = await rosConnection.testConnection();
        console.log(`📊 ROS2 connectivity: ${connectivityTest.requiredTopicsFound?.length || 0} required topics found`);
        
        // 4. Initialize WebSocket server
        console.log('🔌 Initializing WebSocket server...');
        initializeWebSocketServer(server);
        console.log('✅ WebSocket server initialized');
        
        // 5. Initialize UDP discovery server
        console.log('📡 Initializing UDP discovery...');
        const udpServer = initializeUDPDiscovery();
        if (udpServer) {
            console.log('✅ UDP discovery server started');
        } else {
            console.warn('⚠️ UDP discovery server failed to start');
        }
        
        // Store WebSocket instance globally for message handler
        global.webSocketInstance = {
            sendToClient: require('./websocket/clientConnection').sendToClient,
            broadcastToSubscribers: require('./websocket/clientConnection').broadcastToSubscribers
        };
        
        // 6. Auto-connect default AGV if not already connected
        setTimeout(async () => {
            if (global.connectedDevices.length === 0) {
                console.log('🔍 No devices found, attempting auto-connection...');
                try {
                    const deviceInfo = {
                        id: 'agv_01',
                        name: 'Primary AGV',
                        type: 'differential_drive',
                        ipAddress: '192.168.253.136', // Your AGV's IP
                        capabilities: config.DEVICE.CAPABILITIES,
                        autoConnected: true
                    };
                    
                    rosConnection.addConnectedDevice(deviceInfo);
                    
                    try {
                        await storageManager.saveDevice('agv_01');
                    } catch (saveError) {
                        console.warn('⚠️ Could not save auto-connected device:', saveError.message);
                    }
                    
                    console.log('✅ Default AGV auto-connected');
                } catch (error) {
                    console.warn('⚠️ Could not auto-connect default AGV:', error.message);
                }
            }
        }, 3000);
        
        console.log('✅ AGV Fleet Management System initialized successfully');
        return true;
        
    } catch (error) {
        console.error('❌ Failed to initialize application:', error);
        throw error;
    }
}

async function startServer() {
    try {
        await initializeApplication();
        
        server.listen(config.SERVER.PORT, config.SERVER.HOST, () => {
            const serverInfo = require('./routes/discoveryRoutes').getServerInfo ? 
                require('./routes/discoveryRoutes').getServerInfo() : 
                { ip: config.SERVER.HOST, port: config.SERVER.PORT };
                
            console.log(`\n🎉 AGV Fleet Backend Server Started`);
            console.log(`📍 Server: http://${serverInfo.ip}:${serverInfo.port}`);
            console.log(`🔌 WebSocket: ws://${serverInfo.ip}:${serverInfo.port}`);
            console.log(`🗂️ Health Check: http://${serverInfo.ip}:${serverInfo.port}/health`);
            console.log(`🎮 Control API: http://${serverInfo.ip}:${serverInfo.port}/api/control`);
            console.log(`🔍 Discovery API: http://${serverInfo.ip}:${serverInfo.port}/api/discovery`);
            console.log(`📡 UDP Discovery: Port ${config.NETWORK.DISCOVERY.PORT}`);
            console.log(`\n📊 System Status:`);
            // console.log(`   - ROS2: ${rosConnection.isRosInitialized() ? '✅ Connected' : '❌ Disconnected'}`);
            console.log(`   - Storage: ✅ Ready`);
            console.log(`   - WebSocket: ✅ Ready`);
            console.log(`   - Device Discovery: ✅ Ready`);
            console.log(`   - Connected Devices: ${global.connectedDevices.length}`);
            console.log(`\n🤖 Ready for AGV connections!\n`);
            
            // Test ROS2 publishing capability
            setTimeout(async () => {
                try {
                    const publisherTest = require('./ros/utils/publishers').testPublishing();
                    if (publisherTest.success) {
                        console.log('✅ ROS2 publishing test successful');
                    } else {
                        console.warn('⚠️ ROS2 publishing test failed:', publisherTest.error);
                    }
                } catch (testError) {
                    console.warn('⚠️ Could not test ROS2 publishing:', testError.message);
                }
            }, 2000);
        });
        
    } catch (error) {
        console.error('💥 Failed to start server:', error);
        process.exit(1);
    }
}

// Graceful shutdown handling
process.on('SIGINT', gracefulShutdown);
process.on('SIGTERM', gracefulShutdown);

async function gracefulShutdown(signal) {
    console.log(`\n📴 Received ${signal}, starting graceful shutdown...`);
    
    try {
        // Stop accepting new connections
        server.close(() => {
            console.log('🔌 HTTP server closed');
        });
        
        // Cleanup ROS2 connection
        await rosConnection.shutdown();
        console.log('🤖 ROS2 connection closed');
        
        // Cleanup storage
        try {
            await storageManager.cleanup();
            console.log('💾 Storage manager cleaned up');
        } catch (e) {
            console.warn('⚠️ Storage cleanup warning:', e.message);
        }
        
        console.log('✅ Graceful shutdown completed');
        process.exit(0);
        
    } catch (error) {
        console.error('❌ Error during shutdown:', error);
        process.exit(1);
    }
}

// Handle uncaught exceptions
process.on('uncaughtException', (error) => {
    console.error('💥 Uncaught Exception:', error);
    gracefulShutdown('UNCAUGHT_EXCEPTION');
});

process.on('unhandledRejection', (reason, promise) => {
    console.error('💥 Unhandled Rejection at:', promise, 'reason:', reason);
    gracefulShutdown('UNHANDLED_REJECTION');
});

// Start the server
if (require.main === module) {
    startServer();
}

module.exports = app;