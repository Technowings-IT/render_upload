// config.js - Configuration for AGV Fleet Management System
const path = require('path');

const config = {
    // Server Configuration
    PORT: process.env.PORT || 3000,
    HOST: process.env.HOST || '0.0.0.0',
    NODE_ENV: process.env.NODE_ENV || 'development',

    // ROS2 Configuration
    ROS2: {
        NODE_NAME: 'agv_fleet_backend',
        DOMAIN_ID: parseInt(process.env.ROS_DOMAIN_ID) || 0,
        TOPICS: {
            CMD_VEL: '/cmd_vel',
            ODOM: '/odom',
            MAP: '/map',
            SCAN: '/scan',
            GOAL_POSE: '/goal_pose',
            INITIAL_POSE: '/initialpose',
            JOINT_STATES: '/joint_states',
            BATTERY_STATE: '/battery_state'
        },
        SERVICES: {
            START_MAPPING: '/slam_toolbox/start_mapping',
            STOP_MAPPING: '/slam_toolbox/stop_mapping',
            SAVE_MAP: '/slam_toolbox/save_map',
            CLEAR_COSTMAP: '/local_costmap/clear_entirely_local_costmap'
        },
        QOS: {
            RELIABILITY: 'reliable',
            DURABILITY: 'volatile',
            DEPTH: 10
        }
    },

    // AGV Configuration
    AGV: {
        MAX_LINEAR_SPEED: 2.0,  // m/s
        MAX_ANGULAR_SPEED: 1.0, // rad/s
        DEFAULT_IP: '192.168.253.79',
        CONNECTION_TIMEOUT: 10000, // ms
        HEARTBEAT_INTERVAL: 5000,  // ms
        POSITION_TOLERANCE: 0.1,   // meters
        ANGLE_TOLERANCE: 0.1       // radians
    },

    // WebSocket Configuration
    WEBSOCKET: {
        HEARTBEAT_INTERVAL: 30000,    // 30 seconds
        CONNECTION_TIMEOUT: 120000,   // 2 minutes
        MAX_CONNECTIONS: 100,
        COMPRESSION: true,
        MESSAGE_TYPES: {
            HEARTBEAT: 'heartbeat',
            SUBSCRIBE: 'subscribe',
            UNSUBSCRIBE: 'unsubscribe',
            CONTROL_COMMAND: 'control_command',
            MAP_EDIT: 'map_edit',
            ORDER_COMMAND: 'order_command'
        }
    },

    // Database/Storage Configuration
    STORAGE: {
        MAPS_DIR: path.join(__dirname, 'storage', 'maps'),
        LOGS_DIR: path.join(__dirname, 'storage', 'logs'),
        UPLOADS_DIR: path.join(__dirname, 'storage', 'uploads'),
        DEVICES_FILE: path.join(__dirname, 'storage', 'devices.json'),
        ORDERS_FILE: path.join(__dirname, 'storage', 'orders.json')
    },

    // Map Configuration
    MAPS: {
        DEFAULT_RESOLUTION: 0.05,    // meters/pixel
        MAX_SIZE: 2048,              // pixels
        SUPPORTED_FORMATS: ['pgm', 'png', 'jpg'],
        COMPRESSION_QUALITY: 0.8,
        SHAPES: {
            PICKUP_LOCATION: { color: '#00FF00', sides: ['front', 'back', 'left', 'right'] },
            DROP_LOCATION: { color: '#FF0000', sides: ['front', 'back', 'left', 'right'] },
            CHARGING_LOCATION: { color: '#0000FF', sides: ['front', 'back'] },
            WAYPOINT: { color: '#FFFF00', sides: ['front'] },
            NO_GO_ZONE: { color: '#FF00FF', sides: [] }
        }
    },

    // Order Management Configuration
    ORDERS: {
        MAX_WAYPOINTS: 20,
        MAX_CONCURRENT_ORDERS: 5,
        RETRY_ATTEMPTS: 3,
        RETRY_DELAY: 5000,           // ms
        ORDER_TIMEOUT: 300000,       // 5 minutes
        STATUSES: ['pending', 'active', 'paused', 'completed', 'failed', 'cancelled']
    },

    // Security Configuration
    SECURITY: {
        ENABLE_AUTH: process.env.ENABLE_AUTH === 'true',
        JWT_SECRET: process.env.JWT_SECRET || 'your-secret-key',
        JWT_EXPIRES_IN: '24h',
        RATE_LIMIT: {
            WINDOW_MS: 15 * 60 * 1000, // 15 minutes
            MAX_REQUESTS: 100
        }
    },

    // Logging Configuration
    LOGGING: {
        LEVEL: process.env.LOG_LEVEL || 'info',
        FORMAT: 'combined',
        MAX_FILES: 10,
        MAX_SIZE: '10m',
        CONSOLE: process.env.NODE_ENV !== 'production'
    },

    // Network Configuration
    NETWORK: {
        DISCOVERY_TIMEOUT: 5000,     // ms
        PING_TIMEOUT: 1000,          // ms
        COMMON_PORTS: [11311, 7400, 7401, 9090, 22],
        SUBNET_SCAN_RANGE: [100, 200] // IP range to scan
    },

    // Performance Configuration
    PERFORMANCE: {
        MAX_PAYLOAD_SIZE: '50mb',
        COMPRESSION_THRESHOLD: 1024,
        CACHE_TTL: 300000,           // 5 minutes
        MAX_CONCURRENT_CONNECTIONS: 1000
    },

    // Development Configuration
    DEVELOPMENT: {
        ENABLE_CORS: true,
        CORS_ORIGINS: ['ws://192.168.253.79:3000', 'ws://192.168.253.79:8080'],
        ENABLE_SWAGGER: true,
        HOT_RELOAD: true,
        DEBUG_ROS: process.env.DEBUG_ROS === 'true'
    },

    // Simulation Configuration (when ROS2 not available)
    SIMULATION: {
        ENABLE: process.env.ENABLE_SIMULATION === 'true',
        UPDATE_INTERVAL: 1000,       // ms
        RANDOM_MOVEMENT: true,
        SIMULATED_DEVICES: 2
    }
};

// Environment-specific overrides
if (config.NODE_ENV === 'production') {
    config.LOGGING.CONSOLE = false;
    config.DEVELOPMENT.ENABLE_CORS = false;
    config.DEVELOPMENT.ENABLE_SWAGGER = false;
    config.SECURITY.ENABLE_AUTH = true;
}

// Validation
function validateConfig() {
    const required = ['PORT', 'ROS2.NODE_NAME'];
    
    for (const key of required) {
        const value = key.split('.').reduce((obj, k) => obj && obj[k], config);
        if (!value) {
            throw new Error(`Missing required configuration: ${key}`);
        }
    }
    
    // Validate directories
    const fs = require('fs');
    Object.values(config.STORAGE).forEach(dir => {
        if (dir.endsWith('.json')) return; // Skip files
        if (!fs.existsSync(dir)) {
            fs.mkdirSync(dir, { recursive: true });
            console.log(`Created directory: ${dir}`);
        }
    });
}

// Initialize configuration
try {
    validateConfig();
    console.log('✅ Configuration validated successfully');
} catch (error) {
    console.error('❌ Configuration validation failed:', error.message);
    process.exit(1);
}

module.exports = config;