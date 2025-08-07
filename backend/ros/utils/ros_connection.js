// ros/utils/ros_connection.js - FIXED to prevent double initialization
const rclnodejs = require('rclnodejs');
const config = require('../../config');
const navigationFeedback = require('./navigation_feedback');

let rosNode = null;
let isInitialized = false;
let isShutdown = false;
let initializationAttempts = 0;
let rclnodeInitialized = false; // Track if rclnodejs.init() was called
let nodeSpinning = false; // Track if node is already spinning
const MAX_INIT_ATTEMPTS = 3;

// Import the publishers module
const publishers = require('./publishers');
// Import the subscribers module
const subscribers = require('./subscribers');    

async function initializeROS() {
    try {
        if (isInitialized && !isShutdown) {
            console.log('✅ ROS already initialized');
            return true;
        }

        initializationAttempts++;
        console.log(`🔄 Initializing ROS2 connection (attempt ${initializationAttempts}/${MAX_INIT_ATTEMPTS})...`);
        
        // Reset shutdown flag for retry attempts
        isShutdown = false;
        
        // ✅ CRITICAL FIX: Only initialize rclnodejs once per process
        if (!rclnodeInitialized) {
            await rclnodejs.init();
            rclnodeInitialized = true;
            console.log('✅ ROS2 context initialized');
        } else {
            console.log('✅ ROS2 context already initialized, reusing...');
        }
        
        // Always create a new node (if previous one was destroyed)
        if (!rosNode) {
            rosNode = new rclnodejs.Node('AMR_fleet_backend_node');
            console.log('✅ ROS2 node created: AMR_fleet_backend_node');
        } else {
            console.log('✅ ROS2 node already exists, reusing...');
        }
        
        // Initialize publishers with the node
        publishers.initializePublishers(rosNode);

        // Initialize subscribers with the node
        subscribers.initializeSubscribers(rosNode);

        // ✅ NEW: Initialize navigation feedback system
        try {
            navigationFeedback.initializeNavigationFeedback(rosNode);
            console.log('✅ Navigation feedback system initialized');
        } catch (navError) {
            console.warn('⚠️ Failed to initialize navigation feedback:', navError.message);
            // Don't fail the entire initialization for navigation feedback
        }

        // ✅ CRITICAL FIX: Only start spinning once per process
        if (!nodeSpinning) {
            rclnodejs.spin(rosNode);
            nodeSpinning = true;
            console.log('✅ ROS2 node spinning started');
        } else {
            console.log('✅ ROS2 node already spinning, continuing...');
        }

        // Subscribe to all topics
        subscribers.subscribeToAllTopics();

        // Test subscribing after a short delay
        setTimeout(() => {
            const result = subscribers.testSubscribing();
            if (result.success) {
                console.log('✅ ROS2 subscribing test successful');
            } else {
                console.warn('⚠️ ROS2 subscribing test failed:', result.error);
            }
        }, 2000);

        // ✅ NEW: Test navigation feedback after initialization
        setTimeout(() => {
            testNavigationFeedback();
        }, 3000);

        isInitialized = true;
        isShutdown = false;
        initializationAttempts = 0; // Reset on success
        
        // Set global references
        global.rosNode = rosNode;
        global.rosInitialized = true;
        
        console.log('✅ ROS2 connection fully initialized');
        
        return true;
        
    } catch (error) {
        console.error(`❌ Failed to initialize ROS2 (attempt ${initializationAttempts}):`, error);
        isInitialized = false;
        
        // ✅ FIXED: Don't retry if it's a "already initialized" or "already spinning" error
        if (error.message.includes('already been initialized')) {
            console.log('⚠️ ROS2 context already initialized - this is normal, marking as successful');
            rclnodeInitialized = true;
            
            // Try to continue with node creation
            try {
                if (!rosNode) {
                    rosNode = new rclnodejs.Node('AMR_fleet_backend_node');
                    console.log('✅ ROS2 node created after context reuse');
                }
                
                // Initialize publishers and subscribers
                publishers.initializePublishers(rosNode);
                subscribers.initializeSubscribers(rosNode);
                
                // ✅ NEW: Initialize navigation feedback system after recovery
                try {
                    navigationFeedback.initializeNavigationFeedback(rosNode);
                    console.log('✅ Navigation feedback system initialized after recovery');
                } catch (navError) {
                    console.warn('⚠️ Failed to initialize navigation feedback after recovery:', navError.message);
                }
                
                if (!nodeSpinning) {
                    rclnodejs.spin(rosNode);
                    nodeSpinning = true;
                    console.log('✅ ROS2 node spinning started after recovery');
                }
                
                subscribers.subscribeToAllTopics();
                
                isInitialized = true;
                isShutdown = false;
                initializationAttempts = 0;
                
                // Set global references
                global.rosNode = rosNode;
                global.rosInitialized = true;
                
                console.log('✅ ROS2 connection recovered successfully');
                return true;
                
            } catch (nodeError) {
                console.error('❌ Failed to create node after context reuse:', nodeError);
            }
        }
        
        if (error.message.includes('already spinning')) {
            console.log('⚠️ ROS2 node already spinning - this is normal, marking as successful');
            nodeSpinning = true;
            
            // Try to continue with initialization
            try {
                // Just initialize publishers and subscribers
                publishers.initializePublishers(rosNode);
                subscribers.initializeSubscribers(rosNode);
                
                // ✅ NEW: Initialize navigation feedback system
                try {
                    navigationFeedback.initializeNavigationFeedback(rosNode);
                    console.log('✅ Navigation feedback system initialized (node was already spinning)');
                } catch (navError) {
                    console.warn('⚠️ Failed to initialize navigation feedback (node was already spinning):', navError.message);
                }
                
                subscribers.subscribeToAllTopics();
                
                isInitialized = true;
                isShutdown = false;
                initializationAttempts = 0;
                
                // Set global references
                global.rosNode = rosNode;
                global.rosInitialized = true;
                
                console.log('✅ ROS2 connection recovered successfully (node was already spinning)');
                return true;
                
            } catch (recoveryError) {
                console.error('❌ Failed to recover after spinning error:', recoveryError);
            }
        }
        
        // Auto-retry if we haven't exceeded max attempts and it's not a recoverable error
        if (initializationAttempts < MAX_INIT_ATTEMPTS && 
            !error.message.includes('already been initialized') && 
            !error.message.includes('already spinning')) {
            console.log(`🔄 Retrying ROS2 initialization in 5 seconds...`);
            setTimeout(() => {
                initializeROS();
            }, 5000);
        } else {
            console.error(`❌ Max ROS2 initialization attempts (${MAX_INIT_ATTEMPTS}) exceeded`);
        }
        
        return false;
    }
}

// ✅ ENHANCED: Auto-recovery with better context handling
async function ensureROSConnection() {
    if (!isInitialized || isShutdown) {
        console.log('🔄 ROS connection lost, attempting to recover...');
        return await initializeROS();
    }
    return true;
}

// ✅ ENHANCED: Recovery that handles context and spinning properly
async function recoverConnection() {
    console.log('🔄 Manual ROS recovery requested...');
    
    // Reset state but keep context and spinning awareness
    isShutdown = false;
    isInitialized = false;
    
    // Don't reset rclnodeInitialized or nodeSpinning - these should stay
    
    // Destroy old node if it exists
    if (rosNode) {
        try {
            rosNode.destroy();
            rosNode = null;
            console.log('🗑️ Old ROS node destroyed');
        } catch (e) {
            console.warn('⚠️ Error destroying old node:', e.message);
            rosNode = null; // Force null anyway
        }
    }
    
    // Attempt recovery
    return await initializeROS();
}

// ✅ NEW: Test navigation feedback system
async function testNavigationFeedback() {
    try {
        if (!isInitialized || isShutdown) {
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        console.log('🧪 Testing navigation feedback system...');
        
        // Get active navigation goals
        const activeGoals = navigationFeedback.getActiveNavigationGoals ? 
            navigationFeedback.getActiveNavigationGoals() : [];
        console.log(`📊 Active navigation goals: ${activeGoals.length}`);
        
        // Test publisher status (if available)
        let navStatus = null;
        try {
            navStatus = publishers.getNavigationStatus ? publishers.getNavigationStatus() : null;
            console.log('📊 Navigation publisher status:', navStatus);
        } catch (e) {
            console.warn('⚠️ Could not get navigation publisher status:', e.message);
        }
        
        console.log('✅ Navigation feedback test completed');
        
        return {
            success: true,
            rosInitialized: isInitialized,
            activeGoals: activeGoals.length,
            publisherStatus: navStatus,
            timestamp: new Date().toISOString()
        };
        
    } catch (error) {
        console.error('❌ Navigation feedback test failed:', error);
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

// ✅ ENHANCED: Joystick publishing with auto-recovery
function publishJoystick(normalizedX, normalizedY, deadman = false, maxLinearSpeed = null, maxAngularSpeed = null) {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, attempting recovery...');
            
            // Try to recover connection
            ensureROSConnection().then(recovered => {
                if (recovered) {
                    console.log('✅ ROS connection recovered, you can retry the joystick command');
                } else {
                    console.error('❌ Failed to recover ROS connection');
                }
            });
            
            return {
                success: false,
                error: 'ROS not initialized - recovery attempted',
                recoveryInProgress: true,
                timestamp: new Date().toISOString()
            };
        }
        
        // ✅ CRITICAL: Pass max speeds to publisher
        const result = publishers.publishJoystick(normalizedX, normalizedY, deadman, maxLinearSpeed, maxAngularSpeed);
        
        // If publish fails due to ROS issues, mark for recovery
        if (!result.success && result.error.includes('ROS')) {
            isInitialized = false;
            console.warn('⚠️ ROS publish failed, marking for recovery');
        }
        
        return result;
        
    } catch (error) {
        console.error('❌ Error publishing joystick command:', error);
        
        // If it's a ROS-related error, mark for recovery
        if (error.message.includes('ROS') || error.message.includes('node')) {
            isInitialized = false;
            console.warn('⚠️ ROS error detected, marking for recovery');
        }
        
        return {
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        };
    }
}

// ✅ ENHANCED: Velocity publishing with recovery
function publishVelocity(linear = 0, angular = 0) {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized for velocity command');
            
            // For safety commands like stop (0,0), try emergency fallback
            if (linear === 0 && angular === 0) {
                console.log('🛑 Emergency stop attempted - ROS unavailable');
            }
            
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        const result = publishers.publishVelocity(linear, angular);
        
        // If publish fails due to ROS issues, mark for recovery
        if (!result.success && result.error.includes('ROS')) {
            isInitialized = false;
        }
        
        return result;
        
    } catch (error) {
        console.error('❌ Error publishing velocity:', error);
        
        if (error.message.includes('ROS') || error.message.includes('node')) {
            isInitialized = false;
        }
        
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

// Goal publishing with recovery
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
        
        const result = publishers.publishGoal(x, y, orientation);
        
        if (!result.success && result.error.includes('ROS')) {
            isInitialized = false;
        }
        
        return result;
        
    } catch (error) {
        console.error('❌ Error publishing goal:', error);
        
        if (error.message.includes('ROS') || error.message.includes('node')) {
            isInitialized = false;
        }
        
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

async function saveMap(deviceId, options = {}) {
    try {
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, cannot save map');
            return {
                success: false,
                error: 'ROS not initialized',
                timestamp: new Date().toISOString()
            };
        }
        
        console.log(`💾 Saving map via ROS2 script for device: ${deviceId}`);
        
        // Use the enhanced ROS2 script manager for map saving
        // Try to use global instance first, otherwise create new instance
        let ros2ScriptManager;
        if (global.ros2ScriptManager) {
            ros2ScriptManager = global.ros2ScriptManager;
        } else {
            const ROS2ScriptManager = require('./ros2ScriptManager');
            ros2ScriptManager = new ROS2ScriptManager();
        }
        
        // Extract options with defaults
        const {
            mapName = `map_${Date.now()}`,
            directory = '/home/piros/my_map',
            format = 'pgm'
        } = options;
        
        console.log(`🗺️ Map save parameters: name="${mapName}", directory="${directory}"`);
        
        // Execute the enhanced save_map.sh script on Pi
        const result = await ros2ScriptManager.executeROS2MapSaver({
            deviceId,
            mapName,
            directory,
            format
        });
        
        if (result.success) {
            console.log(`✅ Map saved successfully via script: ${mapName}`);
            return {
                success: true,
                message: 'Map saved successfully via ROS2 script',
                deviceId: deviceId,
                mapName: mapName,
                directory: directory,
                files: result.files,
                scriptOutput: result.scriptOutput,
                timestamp: new Date().toISOString()
            };
        } else {
            console.error(`❌ Map save failed: ${result.error}`);
            return {
                success: false,
                error: result.error,
                deviceId: deviceId,
                mapName: mapName,
                timestamp: new Date().toISOString()
            };
        }
        
    } catch (error) {
        console.error('❌ Error saving map:', error);
        return {
            success: false,
            error: error.message,
            deviceId: deviceId,
            timestamp: new Date().toISOString()
        };
    }
}

// Emergency stop - always try, even if ROS seems down
function emergencyStop() {
    try {
        console.log('🛑 EMERGENCY STOP REQUESTED');
        
        if (!isInitialized || isShutdown) {
            console.warn('⚠️ ROS not initialized, but attempting emergency stop anyway');
            
            // Try to recover and stop
            ensureROSConnection().then(recovered => {
                if (recovered) {
                    publishers.emergencyStop();
                    console.log('🛑 Emergency stop executed after recovery');
                }
            });
            
            return {
                success: false,
                error: 'ROS not initialized - emergency recovery attempted',
                type: 'emergency_stop',
                timestamp: new Date().toISOString()
            };
        }
        
        return publishers.emergencyStop();
        
    } catch (error) {
        console.error('❌ Emergency stop failed:', error);
        return {
            success: false,
            error: error.message,
            type: 'emergency_stop'
        };
    }
}

// ✅ ENHANCED: Status with recovery info
function getConnectionStatus() {
    return {
        isInitialized: isInitialized,
        isShutdown: isShutdown,
        nodeCreated: rosNode !== null,
        rclnodeInitialized: rclnodeInitialized,
        nodeSpinning: nodeSpinning,
        initializationAttempts: initializationAttempts,
        maxInitAttempts: MAX_INIT_ATTEMPTS,
        publisherStats: isInitialized ? publishers.getPublisherStats() : null,
        canRecover: initializationAttempts < MAX_INIT_ATTEMPTS,
        navigationFeedbackActive: isInitialized ? (navigationFeedback.getActiveNavigationGoals ? navigationFeedback.getActiveNavigationGoals().length : 0) : 0,
        timestamp: new Date().toISOString()
    };
}

// ✅ ENHANCED: ROS2 status function for health checks
function getROS2Status() {
    try {
        return {
            isConnected: isInitialized && !isShutdown,
            isInitialized: isInitialized,
            isShutdown: isShutdown,
            rclnodeInitialized: rclnodeInitialized,
            nodeSpinning: nodeSpinning,
            nodeStatus: rosNode ? 'active' : 'inactive',
            publishersCount: isInitialized ? Object.keys(publishers.getPublisherStats().publishers || {}).length : 0,
            maxSpeeds: isInitialized ? publishers.getCurrentMaxSpeeds() : { linear: 0, angular: 0 },
            initializationAttempts: initializationAttempts,
            canRecover: initializationAttempts < MAX_INIT_ATTEMPTS,
            navigationFeedback: {
                initialized: isInitialized,
                activeGoals: isInitialized ? (navigationFeedback.getActiveNavigationGoals ? navigationFeedback.getActiveNavigationGoals().length : 0) : 0
            },
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
                canRecover: initializationAttempts < MAX_INIT_ATTEMPTS,
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

// ✅ ENHANCED: Shutdown with proper context cleanup
async function shutdown() {
    try {
        if (isShutdown) {
            console.log('✅ ROS already shutdown');
            return true;
        }
        
        console.log('🔄 Shutting down ROS2 connection...');
        
        isShutdown = true;
        
        // ✅ NEW: Cleanup navigation feedback first
        try {
            navigationFeedback.cleanup();
            console.log('🧹 Navigation feedback cleanup completed');
        } catch (e) {
            console.warn('⚠️ Error cleaning up navigation feedback:', e.message);
        }
        
        // Cleanup subscribers
        try {
            subscribers.cleanup();
        } catch (e) {
            console.warn('⚠️ Error cleaning up subscribers:', e.message);
        }
        
        // Cleanup publishers
        try {
            publishers.cleanup();
        } catch (e) {
            console.warn('⚠️ Error cleaning up publishers:', e.message);
        }
        
        // Destroy the node
        if (rosNode) {
            try {
                rosNode.destroy();
                rosNode = null;
                nodeSpinning = false; // Reset spinning state when node is destroyed
                console.log('🗑️ ROS node destroyed and spinning state reset');
            } catch (e) {
                console.warn('⚠️ Error destroying ROS node:', e.message);
                rosNode = null;
                nodeSpinning = false; // Reset anyway
            }
        }
        
        // Reset global references
        global.rosNode = null;
        global.rosInitialized = false;
        
        // ✅ FIXED: Only shutdown rclnodejs if we're doing a full shutdown
        // In most cases, we want to keep the context alive for recovery
        try {
            if (process.env.NODE_ENV !== 'development') {
                await rclnodejs.shutdown();
                rclnodeInitialized = false;
                nodeSpinning = false; // Reset spinning state on full shutdown
                console.log('✅ ROS2 context shutdown');
            } else {
                console.log('✅ ROS2 context kept alive for development');
            }
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

// ✅ NEW: Add a device management function that was missing
function addConnectedDevice(deviceInfo) {
    try {
        // Check if device already exists
        const existingIndex = global.connectedDevices.findIndex(d => d.id === deviceInfo.id);
        
        if (existingIndex >= 0) {
            // Update existing device
            global.connectedDevices[existingIndex] = {
                ...global.connectedDevices[existingIndex],
                ...deviceInfo,
                lastConnected: new Date().toISOString(),
                status: 'connected'
            };
            console.log(`🔄 Updated existing device: ${deviceInfo.id}`);
        } else {
            // Add new device
            const newDevice = {
                ...deviceInfo,
                connectedAt: new Date().toISOString(),
                lastConnected: new Date().toISOString(),
                status: 'connected'
            };
            global.connectedDevices.push(newDevice);
            console.log(`➕ Added new device: ${deviceInfo.id}`);
        }
        
        return deviceInfo.id;
        
    } catch (error) {
        console.error('❌ Error adding connected device:', error);
        return null;
    }
}

module.exports = {
    initializeROS,
    ensureROSConnection,    // ✅ NEW
    recoverConnection,      // ✅ ENHANCED: Better context handling
    testNavigationFeedback, // ✅ NEW: Test navigation feedback system
    publishJoystick,        // ✅ ENHANCED: Now with auto-recovery
    publishVelocity,
    publishGoal,
    publishMap,
    updateMaxSpeeds,        // ✅ NEW
    startMapping,
    stopMapping,
    saveMap,
    emergencyStop,          // ✅ ENHANCED: Works even when ROS is down
    getConnectionStatus,    // ✅ ENHANCED: More detailed info
    getROS2Status,          // ✅ ENHANCED: Recovery status
    testConnection,
    addConnectedDevice,     // ✅ NEW: Missing function
    shutdown                // ✅ ENHANCED: Now includes navigation feedback cleanup
};