// ros/utils/rosHealthMonitor.js - ROS Health Monitor with Auto-Recovery
const config = require('../../config');

let rosConnection = null;
let monitorInterval = null;
let isMonitoring = false;
let consecutiveFailures = 0;
const MAX_CONSECUTIVE_FAILURES = 3;
const MONITOR_INTERVAL_MS = 30000; // 30 seconds
const RECOVERY_COOLDOWN_MS = 60000; // 1 minute
let lastRecoveryAttempt = 0;

function initializeMonitor(rosConnectionModule) {
    rosConnection = rosConnectionModule;
    startMonitoring();
    console.log(' ROS health monitor initialized');
}

function startMonitoring() {
    if (isMonitoring) {
        console.log('️ ROS monitor already running');
        return;
    }
    
    isMonitoring = true;
    
    monitorInterval = setInterval(async () => {
        await checkROSHealth();
    }, MONITOR_INTERVAL_MS);
    
    console.log(` ROS health monitoring started (interval: ${MONITOR_INTERVAL_MS}ms)`);
}

function stopMonitoring() {
    if (monitorInterval) {
        clearInterval(monitorInterval);
        monitorInterval = null;
    }
    isMonitoring = false;
    console.log(' ROS health monitoring stopped');
}

async function checkROSHealth() {
    try {
        if (!rosConnection) {
            console.warn('️ ROS connection module not available for health check');
            return;
        }
        
        // Get current ROS status
        const status = rosConnection.getROS2Status();
        const isHealthy = status.isConnected && status.isInitialized && !status.isShutdown;
        
        if (isHealthy) {
            // Reset failure count on success
            if (consecutiveFailures > 0) {
                console.log(` ROS health restored after ${consecutiveFailures} failures`);
                consecutiveFailures = 0;
            }
            
            // Occasionally log health status
            if (Date.now() % (MONITOR_INTERVAL_MS * 4) < MONITOR_INTERVAL_MS) {
                console.log(` ROS health check: HEALTHY (${status.publishersCount} publishers active)`);
            }
            
        } else {
            consecutiveFailures++;
            console.warn(` ROS health check: UNHEALTHY (failure ${consecutiveFailures}/${MAX_CONSECUTIVE_FAILURES})`);
            console.warn(`   - Connected: ${status.isConnected}`);
            console.warn(`   - Initialized: ${status.isInitialized}`);
            console.warn(`   - Shutdown: ${status.isShutdown}`);
            console.warn(`   - Node Status: ${status.nodeStatus}`);
            
            // Attempt recovery if we've hit the threshold
            if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
                await attemptRecovery();
            }
        }
        
        // Broadcast health status via WebSocket if available
        broadcastHealthStatus(status, isHealthy);
        
    } catch (error) {
        consecutiveFailures++;
        console.error(` ROS health check error (failure ${consecutiveFailures}):`, error.message);
        
        if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
            await attemptRecovery();
        }
    }
}

async function attemptRecovery() {
    try {
        const now = Date.now();
        
        // Check recovery cooldown
        if (now - lastRecoveryAttempt < RECOVERY_COOLDOWN_MS) {
            const remainingCooldown = Math.ceil((RECOVERY_COOLDOWN_MS - (now - lastRecoveryAttempt)) / 1000);
            console.log(`⏳ Recovery cooldown active (${remainingCooldown}s remaining)`);
            return;
        }
        
        lastRecoveryAttempt = now;
        console.log(' Attempting automatic ROS recovery...');
        
        // Broadcast recovery attempt
        broadcastRecoveryAttempt();
        
        // Attempt recovery
        const recovered = await rosConnection.recoverConnection();
        
        if (recovered) {
            console.log(' Automatic ROS recovery successful!');
            consecutiveFailures = 0;
            
            // Test the connection
            setTimeout(() => {
                const testResult = rosConnection.testConnection();
                if (testResult.success) {
                    console.log(' ROS connection test passed after recovery');
                    broadcastRecoverySuccess();
                } else {
                    console.warn('️ ROS connection test failed after recovery');
                    broadcastRecoveryPartial();
                }
            }, 2000);
            
        } else {
            console.error(' Automatic ROS recovery failed');
            broadcastRecoveryFailed();
        }
        
    } catch (error) {
        console.error(' Error during automatic recovery:', error);
        broadcastRecoveryFailed();
    }
}

function broadcastHealthStatus(status, isHealthy) {
    try {
        if (global.webSocketInstance && global.webSocketInstance.broadcastToSubscribers) {
            global.webSocketInstance.broadcastToSubscribers('system_health', {
                type: 'ros_health_update',
                healthy: isHealthy,
                status: status,
                consecutiveFailures: consecutiveFailures,
                monitoringActive: isMonitoring,
                timestamp: new Date().toISOString()
            });
        }
    } catch (error) {
        // Silently ignore WebSocket errors
    }
}

function broadcastRecoveryAttempt() {
    try {
        if (global.webSocketInstance && global.webSocketInstance.broadcastToSubscribers) {
            global.webSocketInstance.broadcastToSubscribers('system_events', {
                type: 'ros_recovery_attempt',
                message: 'Attempting automatic ROS recovery',
                consecutiveFailures: consecutiveFailures,
                timestamp: new Date().toISOString()
            });
        }
    } catch (error) {
        // Silently ignore WebSocket errors
    }
}

function broadcastRecoverySuccess() {
    try {
        if (global.webSocketInstance && global.webSocketInstance.broadcastToSubscribers) {
            global.webSocketInstance.broadcastToSubscribers('system_events', {
                type: 'ros_recovery_success',
                message: 'ROS connection recovered successfully',
                timestamp: new Date().toISOString()
            });
        }
    } catch (error) {
        // Silently ignore WebSocket errors
    }
}

function broadcastRecoveryPartial() {
    try {
        if (global.webSocketInstance && global.webSocketInstance.broadcastToSubscribers) {
            global.webSocketInstance.broadcastToSubscribers('system_events', {
                type: 'ros_recovery_partial',
                message: 'ROS connection partially recovered - some issues remain',
                timestamp: new Date().toISOString()
            });
        }
    } catch (error) {
        // Silently ignore WebSocket errors
    }
}

function broadcastRecoveryFailed() {
    try {
        if (global.webSocketInstance && global.webSocketInstance.broadcastToSubscribers) {
            global.webSocketInstance.broadcastToSubscribers('system_events', {
                type: 'ros_recovery_failed',
                message: 'Automatic ROS recovery failed - manual intervention may be required',
                consecutiveFailures: consecutiveFailures,
                timestamp: new Date().toISOString()
            });
        }
    } catch (error) {
        // Silently ignore WebSocket errors
    }
}

function getMonitorStatus() {
    return {
        isMonitoring: isMonitoring,
        consecutiveFailures: consecutiveFailures,
        maxFailures: MAX_CONSECUTIVE_FAILURES,
        monitorInterval: MONITOR_INTERVAL_MS,
        lastRecoveryAttempt: lastRecoveryAttempt,
        recoveryCooldown: RECOVERY_COOLDOWN_MS,
        nextRecoveryAvailable: lastRecoveryAttempt + RECOVERY_COOLDOWN_MS,
        timeUntilNextRecovery: Math.max(0, (lastRecoveryAttempt + RECOVERY_COOLDOWN_MS) - Date.now())
    };
}

function resetFailureCount() {
    consecutiveFailures = 0;
    console.log(' ROS monitor failure count reset');
}

function cleanup() {
    stopMonitoring();
    rosConnection = null;
    consecutiveFailures = 0;
    lastRecoveryAttempt = 0;
    console.log(' ROS health monitor cleaned up');
}

module.exports = {
    initializeMonitor,
    startMonitoring,
    stopMonitoring,
    checkROSHealth,
    attemptRecovery,
    getMonitorStatus,
    resetFailureCount,
    cleanup
};