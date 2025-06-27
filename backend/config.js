// config.js - FIXED Configuration with stable WebSocket settings
const path = require('path');

const config = {
    // Server Configuration
    SERVER: {
        PORT: process.env.PORT || 3000,
        HOST: process.env.HOST || '0.0.0.0', // ✅ FIXED: Listen on all interfaces
        ENV: process.env.NODE_ENV || 'development'
    },

    // ROS2 Configuration
    ROS2: {
        NODE_NAME: 'agv_fleet_backend',
        TOPICS: {
            CMD_VEL: '/cmd_vel',
            ODOM: '/diff_drive_controller/odom',
            POS: '/pos',
            MAP: '/map',
            GOAL_POSE: '/move_base_simple/goal',
            INITIAL_POSE: '/initialpose',
            JOINT_STATES: '/joint_states',
            BATTERY_STATE: '/battery_state',
            LASER_SCAN: '/scan'
        },
        SERVICES: {
            START_MAPPING: '/start_mapping',
            STOP_MAPPING: '/stop_mapping',
            SAVE_MAP: '/save_map',
            LOAD_MAP: '/load_map'
        },
        QOS: {
            RELIABILITY: 'reliable', // ✅ FIXED: Ensure reliable delivery
            DURABILITY: 'volatile',
            DEPTH: 10
        }
    },

    // AGV Configuration
    AGV: {
        MAX_LINEAR_SPEED: 1.0,   // ✅ FIXED: Reduced for safety
        MAX_ANGULAR_SPEED: 1.5,  // ✅ FIXED: Reduced for safety
        DEFAULT_LINEAR_SPEED: 0.3,
        DEFAULT_ANGULAR_SPEED: 0.8,
        SAFETY_TIMEOUT: 1000,
        DEADMAN_TIMEOUT: 500,
        POSITION_TOLERANCE: 0.1,
        ORIENTATION_TOLERANCE: 0.1,
        EMERGENCY_STOP_TIMEOUT: 100
    },

    // ✅ FIXED: WebSocket Configuration with stable connection settings
    WEBSOCKET: {
        COMPRESSION: true,
        MAX_PAYLOAD: 16 * 1024,
        PING_INTERVAL: 25000,     // ✅ FIXED: 25 seconds (less aggressive)
        HEARTBEAT_INTERVAL: 10000, // ✅ FIXED: 10 seconds
        CONNECTION_TIMEOUT: 180000, // ✅ FIXED: 3 minutes (much more lenient)
        MESSAGE_TYPES: {
            PING: 'ping',
            PONG: 'pong',
            HEARTBEAT: 'heartbeat',
            HEARTBEAT_ACK: 'heartbeat_ack',
            SUBSCRIBE: 'subscribe',
            UNSUBSCRIBE: 'unsubscribe',
            CONTROL_COMMAND: 'control_command',
            JOYSTICK_CONTROL: 'joystick_control',
            MAPPING_COMMAND: 'mapping_command',
            MAP_EDIT: 'map_edit',
            ORDER_COMMAND: 'order_command',
            DEVICE_COMMAND: 'device_command',
            REQUEST_DATA: 'request_data',
            REAL_TIME_DATA: 'real_time_data',
            CONTROL_EVENTS: 'control_events',
            MAPPING_EVENTS: 'mapping_events',
            MAP_EVENTS: 'map_events',
            ORDER_EVENTS: 'order_events',
            DEVICE_EVENTS: 'device_events'
        }
    },

    // Storage Configuration
    STORAGE: {
        BASE_DIR: path.join(__dirname, 'storage'),
        DEVICES_FILE: path.join(__dirname, 'storage', 'devices.json'),
        ORDERS_FILE: path.join(__dirname, 'storage', 'orders.json'),
        MAPS_DIR: path.join(__dirname, 'storage', 'maps'),
        LOGS_DIR: path.join(__dirname, 'storage', 'logs'),
        UPLOADS_DIR: path.join(__dirname, 'storage', 'uploads'),
        BACKUPS_DIR: path.join(__dirname, 'storage', 'backups'),
        BACKUP_INTERVAL: 3600000,
        MAX_BACKUP_FILES: 24,
        AUTO_SAVE_INTERVAL: 120000
    },

    // Logging Configuration
    LOGGING: {
        LEVEL: process.env.LOG_LEVEL || 'info',
        MAX_FILE_SIZE: '10m',
        MAX_FILES: '14d',
        FILES: {
            APP: path.join(__dirname, 'storage', 'logs', 'app.log'),
            ERROR: path.join(__dirname, 'storage', 'logs', 'error.log'),
            ROS: path.join(__dirname, 'storage', 'logs', 'ros.log'),
            ROS_TOPICS: path.join(__dirname, 'storage', 'logs', 'ros_topics.log')
        }
    },

    // ✅ FIXED: CORS Configuration for your network
    CORS: {
        ALLOWED_ORIGINS: [
            'http://localhost:3000',
            'http://localhost:8080',
            'http://127.0.0.1:3000',
            'http://127.0.0.1:8080',
            'http://192.168.0.156:3000', // ✅ FIXED: Your AGV IP
            'http://192.168.0.113:3000',  // ✅ FIXED: Your backend IP
            'http://192.168.0.*:*',      // ✅ FIXED: Allow entire subnet
            '*' // ✅ FIXED: Allow all origins for development
        ],
        CREDENTIALS: true
    },

    // Map Configuration
    MAP: {
        DEFAULT_RESOLUTION: 0.05,
        DEFAULT_WIDTH: 1000,
        DEFAULT_HEIGHT: 1000,
        DEFAULT_ORIGIN: {
            x: -25.0,
            y: -25.0,
            z: 0.0
        },
        MAX_MAP_SIZE: 4000,
        UPDATE_THROTTLE: 1000, // ✅ FIXED: Faster updates (1 second)
        OCCUPANCY_THRESHOLD: {
            FREE: 25,
            OCCUPIED: 65
        }
    },

    // Shape Configuration for Map Editor
    SHAPES: {
        TYPES: {
            PICKUP: 'pickup',
            DROP: 'drop',
            CHARGING: 'charging',
            OBSTACLE: 'obstacle',
            WAYPOINT: 'waypoint',
            ZONE: 'zone'
        },
        COLORS: {
            PICKUP: '#00FF00FF',
            DROP: '#FF0000FF',
            CHARGING: '#0000FFFF',
            OBSTACLE: '#FF8800FF',
            WAYPOINT: '#FFFF00FF',
            ZONE: '#FF00FFFF'
        },
        DEFAULT_SIDES: {
            left: '',
            right: '',
            front: '',
            back: ''
        }
    },

    // Order Management Configuration
    ORDERS: {
        MAX_QUEUE_SIZE: 10,
        DEFAULT_PRIORITY: 0,
        STATUS: {
            PENDING: 'pending',
            ACTIVE: 'active',
            PAUSED: 'paused',
            COMPLETED: 'completed',
            CANCELLED: 'cancelled',
            FAILED: 'failed'
        },
        TIMEOUT: 300000,
        RETRY_ATTEMPTS: 3
    },

    // Device Configuration
    DEVICE: {
        STATUS: {
            CONNECTED: 'connected',
            DISCONNECTED: 'disconnected',
            ERROR: 'error',
            UNKNOWN: 'unknown'
        },
        CAPABILITIES: [
            'mapping',
            'navigation',
            'remote_control',
            'autonomous',
            'obstacle_avoidance',
            'battery_monitoring'
        ],
        HEARTBEAT_INTERVAL: 5000,
        TIMEOUT: 30000,
        RECONNECT_ATTEMPTS: 5,
        RECONNECT_DELAY: 2000
    },

    // ✅ FIXED: Safety Configuration with proper limits
    SAFETY: {
        EMERGENCY_STOP_TIMEOUT: 100,
        DEADMAN_SWITCH_TIMEOUT: 1000, // ✅ FIXED: More lenient
        VELOCITY_LIMITS: {
            LINEAR: {
                MIN: -1.0,  // ✅ FIXED: Safer limits
                MAX: 1.0
            },
            ANGULAR: {
                MIN: -1.5,  // ✅ FIXED: Safer limits
                MAX: 1.5
            }
        },
        COLLISION_AVOIDANCE: {
            ENABLED: true,
            MIN_DISTANCE: 0.5,
            STOP_DISTANCE: 0.3
        }
    },

    // ✅ FIXED: Network Configuration for device discovery
    NETWORK: {
        DISCOVERY: {
            ENABLED: true,
            PORT: 8888,
            BROADCAST_INTERVAL: 5000,
            TIMEOUT: 10000,
            // ✅ NEW: AGV subnet configuration
            AGV_SUBNET: '192.168.0',
            AGV_IP_RANGE: {
                START: 100,
                END: 200
            },
            KNOWN_AGVS: [
                {
                    id: 'piros',
                    ip: '192.168.0.103',
                    name: 'Primary AGV'
                }
            ]
        },
        RETRY: {
            ATTEMPTS: 3,
            DELAY: 1000,
            BACKOFF: 2
        }
    },

    // ✅ FIXED: Performance Configuration with optimized throttling
    PERFORMANCE: {
        MESSAGE_THROTTLE: {
            JOYSTICK: 100,      // ✅ FIXED: Faster joystick response
            POSITION: 200,      // ✅ FIXED: Faster position updates
            MAP_UPDATE: 1000,   // ✅ FIXED: Faster map updates
            SENSOR_DATA: 500    // ✅ FIXED: Faster sensor updates
        },
        BUFFER_SIZES: {
            ROBOT_TRAIL: 1000,      // ✅ FIXED: Larger trail buffer
            MESSAGE_HISTORY: 200,   // ✅ FIXED: More message history
            ERROR_LOG: 1000
        }
    },

    // Development Configuration
    DEVELOPMENT: {
        MOCK_ROS: process.env.MOCK_ROS === 'true',
        DEBUG_WEBSOCKET: process.env.DEBUG_WS === 'true',
        VERBOSE_LOGGING: process.env.VERBOSE === 'true',
        SIMULATE_AGV: process.env.SIMULATE_AGV === 'true'
    }
};
// ✅ ADD: Network discovery helper
function getLocalIPAddress() {
    const os = require('os');
    const networkInterfaces = os.networkInterfaces();
    
    for (const [name, interfaces] of Object.entries(networkInterfaces)) {
        for (const iface of interfaces) {
            if (iface.family === 'IPv4' && !iface.internal) {
                console.log(`📡 Found network interface ${name}: ${iface.address}`);
                return iface.address;
            }
        }
    }
    return 'localhost';
}

module.exports = { ...config, getLocalIPAddress };
// Environment-specific overrides
if (config.SERVER.ENV === 'production') {
    config.LOGGING.LEVEL = 'warn';
    config.DEVELOPMENT.VERBOSE_LOGGING = false;
    config.DEVELOPMENT.DEBUG_WEBSOCKET = false;
    config.CORS.ALLOWED_ORIGINS = [
        'http://192.168.0.103:3000',
        'http://192.168.0.101:3000'
    ]; // Restrict CORS in production
}

if (config.SERVER.ENV === 'test') {
    config.STORAGE.AUTO_SAVE_INTERVAL = 5000;
    config.ROS2.QOS.DEPTH = 1;
    config.WEBSOCKET.PING_INTERVAL = 5000;
}

// ✅ FIXED: Enhanced validation function
function validateConfig() {
    const errors = [];

    // Check required configurations
    if (!config.SERVER.PORT || config.SERVER.PORT < 1 || config.SERVER.PORT > 65535) {
        errors.push('Invalid server port');
    }

    if (config.AGV.MAX_LINEAR_SPEED <= 0 || config.AGV.MAX_ANGULAR_SPEED <= 0) {
        errors.push('Invalid AGV speed limits');
    }

    if (config.WEBSOCKET.PING_INTERVAL < 1000) {
        errors.push('WebSocket ping interval too short');
    }

    // ✅ NEW: Validate network configuration
    if (config.NETWORK.DISCOVERY.AGV_IP_RANGE.START >= config.NETWORK.DISCOVERY.AGV_IP_RANGE.END) {
        errors.push('Invalid AGV IP range');
    }

    if (errors.length > 0) {
        throw new Error(`Configuration validation failed: ${errors.join(', ')}`);
    }

    console.log('✅ Configuration validated successfully');
    console.log(`📡 Server will listen on: ${config.SERVER.HOST}:${config.SERVER.PORT}`);
    console.log(`🔌 WebSocket ping interval: ${config.WEBSOCKET.PING_INTERVAL}ms`);
    console.log(`⏱️ Connection timeout: ${config.WEBSOCKET.CONNECTION_TIMEOUT}ms`);
    console.log(`🚗 Max speeds: ${config.AGV.MAX_LINEAR_SPEED}m/s linear, ${config.AGV.MAX_ANGULAR_SPEED}rad/s angular`);
    
    return true;
}

// Export configuration
module.exports = {
    ...config,
    validateConfig
};

// Auto-validate on import
try {
    validateConfig();
} catch (error) {
    console.error('❌ Configuration validation failed:', error.message);
    process.exit(1);
}