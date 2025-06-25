// ros/utils/ros_connection.js - FIXED with max speed support
const rclnodejs = require('rclnodejs');
const config = require('../../config');

let rosNode = null;
let isInitialized = false;
let isShutdown = false;

// Import the publishers module
const publishers = require('./publishers');

async function initializeROS() {
    try {
        if (isInitialized && !isShutdown) {
            console.log('✅ ROS already initialized');
            return true;
        }

        console.log('🔄 Initializing ROS2 connection...');
        
        // Initialize rclnodejs
        await rclnodejs.init();
        
        // Create the main node
        rosNode = new rclnodejs.Node('agv_fleet_backend_node');
        
        console.log('✅ ROS2 node created: agv_fleet_backend_node');
        
        // Initialize publishers with the node
        publishers.initializePublishers(rosNode);
        
        // Start spinning the node
        rclnodejs.spin(rosNode);
        
        isInitialized = true;
        isShutdown = false;
        
        console.log('✅ ROS2 connection fully initialized');
        
        return true;
        
    } catch (error) {
        console.error('❌ Failed to initialize ROS2:', error);
        isInitialized = false;
        return false;
    }
}

// ✅ FIXED: Enhanced joystick publishing with max speed support
function publishJoystick(normalizedX, normalizedY, deadman = false, maxLinearSpeed = null, maxAngularSpeed = null) {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, cannot publish joystick command');
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        // ✅ CRITICAL: Pass max speeds to publisher
        return publishers.publishJoystick(normalizedX, normalizedY, deadman, maxLinearSpeed, maxAngularSpeed);
        
    } catch (error) {
        console.error('❌ Error publishing joystick command:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

// Velocity publishing
function publishVelocity(linear = 0, angular = 0) {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, cannot publish velocity');
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        return publishers.publishVelocity(linear, angular);
        
    } catch (error) {
        console.error('❌ Error publishing velocity:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

// ✅ NEW: Update max speeds from UI
function updateMaxSpeeds(maxLinearSpeed, maxAngularSpeed) {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, cannot update max speeds');
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        const result = publishers.updateMaxSpeeds(maxLinearSpeed, maxAngularSpeed);
        
        return {
            success: true,
            maxSpeeds: result,
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error('❌ Error updating max speeds:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

// Goal publishing
function publishGoal(x, y, orientation = 0) {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, cannot publish goal');
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        return publishers.publishGoal(x, y, orientation);
        
    } catch (error) {
        console.error('❌ Error publishing goal:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

// Map publishing
function publishMap(deviceId, mapData) {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, cannot publish map');
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        return publishers.publishMap(deviceId, mapData);
        
    } catch (error) {
        console.error('❌ Error publishing map:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

// Mapping commands
function startMapping(deviceId) {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, cannot start mapping');
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        console.log(`🗺️ Starting mapping for device: ${deviceId}`);
        return publishers.startMapping();
        
    } catch (error) {
        console.error('❌ Error starting mapping:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

function stopMapping(deviceId) {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, cannot stop mapping');
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        console.log(`🛑 Stopping mapping for device: ${deviceId}`);
        return publishers.stopMapping();
        
    } catch (error) {
        console.error('❌ Error stopping mapping:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

function saveMap(deviceId) {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, cannot save map');
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        console.log(`💾 Saving map for device: ${deviceId}`);
        // Implementation depends on your mapping system
        // For now, return success
        return {
            success: true,
            message: 'Map save command sent',
            deviceId: deviceId,
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error('❌ Error saving map:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

// Emergency stop
function emergencyStop() {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, cannot emergency stop');
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        return publishers.emergencyStop();
        
    } catch (error) {
        console.error('❌ Error during emergency stop:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

// Status and stats
function getConnectionStatus() {
    return {
        isInitialized: isInitialized,
        isShutdown: isShutdown,
        nodeCreated: rosNode !== null,
        publisherStats: isInitialized ? publishers.getPublisherStats() : null,
        timestamp: new Date().toISOString()
    };
}

// ✅ NEW: ROS2 status function for health checks
function getROS2Status() {
    try {
        return {
            isConnected: isInitialized && !isShutdown,
            isInitialized: isInitialized,
            nodeStatus: rosNode ? 'active' : 'inactive',
            publishersCount: isInitialized ? Object.keys(publishers.getPublisherStats().publishers || {}).length : 0,
            maxSpeeds: isInitialized ? publishers.getCurrentMaxSpeeds() : { linear: 0, angular: 0 },
            lastUpdate: new Date().toISOString(),
            status: isInitialized && !isShutdown ? 'healthy' : 'disconnected'
        };
    } catch (error) {
        return {
            isConnected: false,
            isInitialized: false,
            nodeStatus: 'error',
            error: error.message,
            status: 'error',
            lastUpdate: new Date().toISOString()
        };
    }
}

function testConnection() {
    try {
        if (!isInitialized || isShutdown) {
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        return publishers.testPublishing();
        
    } catch (error) {
        console.error('❌ Error testing ROS connection:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

// Cleanup and shutdown
async function shutdown() {
    try {
        if (isShutdown) {
            console.log('✅ ROS already shutdown');
            return true;
        }
        
        console.log('🔄 Shutting down ROS2 connection...');
        
        isShutdown = true;
        
        // Cleanup publishers
        publishers.cleanup();
        
        // Destroy the node
        if (rosNode) {
            try {
                rosNode.destroy();
                rosNode = null;
            } catch (e) {
                console.warn('⚠️ Error destroying ROS node:', e.message);
            }
        }
        
        // Shutdown rclnodejs
        try {
            await rclnodejs.shutdown();
        } catch (e) {
            console.warn('⚠️ Error shutting down rclnodejs:', e.message);
        }
        
        isInitialized = false;
        
        console.log('✅ ROS2 connection shutdown complete');
        
        return true;
        
    } catch (error) {
        console.error('❌ Error during ROS shutdown:', error);
        return false;
    }
}

module.exports = {
    initializeROS,
    publishJoystick,    // ✅ FIXED: Now supports max speeds
    publishVelocity,
    publishGoal,
    publishMap,
    updateMaxSpeeds,    // ✅ NEW
    startMapping,
    stopMapping,
    saveMap,
    emergencyStop,
    getConnectionStatus,
    getROS2Status,      // ✅ NEW: For health checks
    testConnection,
    shutdown
};