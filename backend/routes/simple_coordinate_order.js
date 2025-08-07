// routes/simple_coordinate_orders.js - Add this NEW file
const express = require('express');
const path = require('path');
const fs = require('fs').promises;
const router = express.Router();

// Import your existing ROS publishers
let rosPublishers = null;
try {
    rosPublishers = require('../ros/utils/publishers');
} catch (error) {
    console.warn('⚠️ ROS publishers not available:', error.message);
}

// Storage
const SIMPLE_ORDERS_FILE = path.join(__dirname, '../storage/simple_orders.json');

// Active order execution state
let activeOrderExecution = null;

/**
 * ✅ CREATE SIMPLE COORDINATE ORDER - EXACTLY what you need
 * POST /api/simple-orders/create
 */
router.post('/create', async (req, res) => {
    try {
        const { deviceId, name, coordinates } = req.body;
        
        console.log(`📝 Creating simple coordinate order: ${name}`);
        console.log(`📍 Coordinates: ${coordinates.length}`);
        
        // Validate coordinates
        if (!coordinates || coordinates.length === 0) {
            return res.status(400).json({
                success: false,
                error: 'At least one coordinate is required'
            });
        }
        
        // Create simple order - JUST coordinates in sequence
        const simpleOrder = {
            id: `order_${Date.now()}`,
            deviceId: deviceId,
            name: name,
            coordinates: coordinates, // Just your coordinates array!
            status: 'pending',
            currentCoordinate: 0,
            createdAt: new Date().toISOString(),
        };
        
        // Save to simple orders file
        const orders = await loadSimpleOrders();
        orders.push(simpleOrder);
        await saveSimpleOrders(orders);
        
        console.log(`✅ Simple coordinate order created: ${simpleOrder.id}`);
        
        res.json({
            success: true,
            order: simpleOrder,
            message: 'Simple coordinate order created successfully'
        });
        
    } catch (error) {
        console.error('❌ Error creating simple coordinate order:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * ✅ START ORDER EXECUTION - Send coordinates one by one
 * POST /api/simple-orders/:orderId/start
 */
router.post('/:orderId/start', async (req, res) => {
    try {
        const { orderId } = req.params;
        
        console.log(`🚀 Starting execution of order: ${orderId}`);
        
        // Load order
        const orders = await loadSimpleOrders();
        const order = orders.find(o => o.id === orderId);
        
        if (!order) {
            return res.status(404).json({
                success: false,
                error: 'Order not found'
            });
        }
        
        if (order.status !== 'pending') {
            return res.status(400).json({
                success: false,
                error: `Order is ${order.status}, cannot start execution`
            });
        }
        
        // Set as active execution
        activeOrderExecution = {
            orderId: orderId,
            deviceId: order.deviceId,
            currentCoordinate: 0,
            totalCoordinates: order.coordinates.length,
            startedAt: new Date().toISOString(),
        };
        
        // Update order status
        order.status = 'executing';
        order.startedAt = new Date().toISOString();
        await saveSimpleOrders(orders);
        
        // Start executing coordinates
        executeCoordinatesSequentially(order);
        
        res.json({
            success: true,
            message: 'Order execution started',
            execution: activeOrderExecution
        });
        
    } catch (error) {
        console.error('❌ Error starting order execution:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * ✅ EXECUTE COORDINATES SEQUENTIALLY - The heart of your system
 */
async function executeCoordinatesSequentially(order) {
    console.log(`📋 Starting sequential execution of ${order.coordinates.length} coordinates`);
    
    for (let i = 0; i < order.coordinates.length; i++) {
        const coordinate = order.coordinates[i];
        
        console.log(`📍 Executing coordinate ${i + 1}/${order.coordinates.length}: ${coordinate.name}`);
        console.log(`   Position: (${coordinate.x}, ${coordinate.y})`);
        
        try {
            // Update active execution state
            if (activeOrderExecution) {
                activeOrderExecution.currentCoordinate = i;
            }
            
            // Send coordinate to /target_pose
            if (rosPublishers && rosPublishers.publishGoalWithId) {
                const goalResult = await rosPublishers.publishGoalWithId(
                    coordinate.x,
                    coordinate.y,
                    0.0, // orientation
                    `${order.id}_coord_${i}_${Date.now()}`
                );
                
                if (!goalResult.success) {
                    throw new Error(`Failed to publish coordinate: ${goalResult.error}`);
                }
                
                console.log(`✅ Published coordinate ${i + 1} to /target_pose`);
                
                // Wait for navigation feedback
                const feedbackResult = await waitForNavigationFeedback(goalResult.goalId, 30000); // 30 second timeout
                
                if (!feedbackResult.success) {
                    throw new Error(`Navigation failed: ${feedbackResult.error}`);
                }
                
                console.log(`✅ Navigation to coordinate ${i + 1} completed successfully`);
                
            } else {
                console.log(`⚠️ ROS publishers not available, simulating coordinate ${i + 1}`);
                // Simulate execution
                await new Promise(resolve => setTimeout(resolve, 2000));
            }
            
        } catch (error) {
            console.error(`❌ Failed to execute coordinate ${i + 1}: ${error.message}`);
            
            // Mark order as failed
            await updateOrderStatus(order.id, 'failed', `Failed at coordinate ${i + 1}: ${error.message}`);
            
            // Clear active execution
            activeOrderExecution = null;
            
            // Broadcast failure (implement WebSocket broadcasting here)
            broadcastOrderFailure(order.id, i + 1, error.message);
            
            return;
        }
    }
    
    // All coordinates completed successfully
    console.log(`🎉 All coordinates completed successfully for order: ${order.id}`);
    
    await updateOrderStatus(order.id, 'completed', 'All coordinates executed successfully');
    
    // Clear active execution
    activeOrderExecution = null;
    
    // Broadcast completion
    broadcastOrderCompletion(order.id);
}

/**
 * ✅ WAIT FOR NAVIGATION FEEDBACK - Critical function
 */
function waitForNavigationFeedback(goalId, timeoutMs = 30000) {
    return new Promise((resolve) => {
        const startTime = Date.now();
        let feedbackReceived = false;
        
        // This should integrate with your existing navigation feedback system
        // For now, simulate the feedback mechanism
        
        const checkFeedback = async () => {
            const currentTime = Date.now();
            
            // Check for timeout
            if (currentTime - startTime > timeoutMs) {
                if (!feedbackReceived) {
                    feedbackReceived = true;
                    resolve({
                        success: false,
                        error: 'Navigation timeout'
                    });
                }
                return;
            }
            
            try {
                // TODO: Replace this with actual feedback check from your navigate_to_pose subscriber
                // Check if goal has been reached via your ROS feedback system
                const isGoalReached = await checkGoalStatus(goalId);
                
                if (isGoalReached.completed) {
                    if (!feedbackReceived) {
                        feedbackReceived = true;
                        resolve({
                            success: isGoalReached.success,
                            error: isGoalReached.error
                        });
                    }
                    return;
                }
                
                // Continue checking
                setTimeout(checkFeedback, 500); // Check every 500ms
                
            } catch (error) {
                if (!feedbackReceived) {
                    feedbackReceived = true;
                    resolve({
                        success: false,
                        error: `Feedback check failed: ${error.message}`
                    });
                }
            }
        };
        
        // Start checking
        checkFeedback();
    });
}

/**
 * ✅ CHECK GOAL STATUS - Integrate with your navigation feedback
 */
async function checkGoalStatus(goalId) {
    // TODO: Integrate with your existing navigate_to_pose subscriber
    // This should check the actual ROS navigation feedback
    
    // For now, simulate success after 3 seconds
    await new Promise(resolve => setTimeout(resolve, 100));
    
    // Simulate 90% success rate
    const success = Math.random() > 0.1;
    
    return {
        completed: true,
        success: success,
        error: success ? null : 'Simulated navigation failure'
    };
}

/**
 * ✅ GET EXECUTION STATUS
 * GET /api/simple-orders/execution-status
 */
router.get('/execution-status', (req, res) => {
    res.json({
        success: true,
        activeExecution: activeOrderExecution,
        isExecuting: activeOrderExecution !== null
    });
});

/**
 * ✅ STOP EXECUTION
 * POST /api/simple-orders/stop-execution
 */
router.post('/stop-execution', async (req, res) => {
    try {
        if (!activeOrderExecution) {
            return res.json({
                success: true,
                message: 'No active execution to stop'
            });
        }
        
        const orderId = activeOrderExecution.orderId;
        
        // Cancel current navigation goal
        if (rosPublishers && rosPublishers.cancelCurrentGoal) {
            await rosPublishers.cancelCurrentGoal();
        }
        
        // Update order status
        await updateOrderStatus(orderId, 'cancelled', 'Execution stopped by user');
        
        // Clear active execution
        activeOrderExecution = null;
        
        console.log(`🛑 Execution stopped for order: ${orderId}`);
        
        res.json({
            success: true,
            message: 'Execution stopped successfully'
        });
        
    } catch (error) {
        console.error('❌ Error stopping execution:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * ✅ GET ALL SIMPLE ORDERS
 * GET /api/simple-orders/:deviceId
 */
router.get('/:deviceId', async (req, res) => {
    try {
        const { deviceId } = req.params;
        
        const orders = await loadSimpleOrders();
        const deviceOrders = orders.filter(order => order.deviceId === deviceId);
        
        res.json({
            success: true,
            orders: deviceOrders
        });
        
    } catch (error) {
        console.error('❌ Error loading simple orders:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * ✅ RESTART FAILED ORDER - For your restart popup
 * POST /api/simple-orders/:orderId/restart
 */
router.post('/:orderId/restart', async (req, res) => {
    try {
        const { orderId } = req.params;
        
        console.log(`🔄 Restarting order: ${orderId}`);
        
        // Load order
        const orders = await loadSimpleOrders();
        const order = orders.find(o => o.id === orderId);
        
        if (!order) {
            return res.status(404).json({
                success: false,
                error: 'Order not found'
            });
        }
        
        // Reset order status
        order.status = 'pending';
        order.currentCoordinate = 0;
        order.restartedAt = new Date().toISOString();
        
        await saveSimpleOrders(orders);
        
        res.json({
            success: true,
            message: 'Order restarted successfully',
            order: order
        });
        
    } catch (error) {
        console.error('❌ Error restarting order:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * ✅ DELETE SIMPLE COORDINATE ORDER
 * DELETE /api/simple-orders/:deviceId/:orderId
 */
router.delete('/:deviceId/:orderId', async (req, res) => {
    try {
        const { deviceId, orderId } = req.params;
        
        console.log(`🗑️ Deleting simple coordinate order: ${orderId} for device: ${deviceId}`);
        
        const orders = await loadSimpleOrders();
        const orderIndex = orders.findIndex(o => o.id === orderId && o.deviceId === deviceId);
        
        if (orderIndex === -1) {
            return res.status(404).json({
                success: false,
                error: 'Simple coordinate order not found'
            });
        }
        
        const order = orders[orderIndex];
        
        // Check if order can be deleted (not currently executing)
        if (order.status === 'executing') {
            return res.status(400).json({
                success: false,
                error: 'Cannot delete an order that is currently executing. Stop the order first.'
            });
        }
        
        // If this is the currently active execution, clear it
        if (activeOrderExecution && activeOrderExecution.orderId === orderId) {
            activeOrderExecution = null;
        }
        
        // Remove the order from the array
        orders.splice(orderIndex, 1);
        await saveSimpleOrders(orders);
        
        console.log(`✅ Simple coordinate order deleted successfully: ${orderId}`);
        
        res.json({
            success: true,
            message: 'Simple coordinate order deleted successfully',
            deletedOrder: {
                id: order.id,
                name: order.name,
                status: order.status,
                deviceId: order.deviceId,
                coordinateCount: order.coordinates ? order.coordinates.length : 0
            },
            timestamp: new Date().toISOString()
        });
        
        // Broadcast order deletion
        broadcastOrderDeletion(deviceId, orderId);
        
    } catch (error) {
        console.error('❌ Error deleting simple coordinate order:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

// ==========================================
// HELPER FUNCTIONS
// ==========================================

async function loadSimpleOrders() {
    try {
        await ensureSimpleOrdersFileExists();
        const data = await fs.readFile(SIMPLE_ORDERS_FILE, 'utf8');
        if (!data || data.trim() === '') return [];
        return JSON.parse(data);
    } catch (error) {
        console.error('❌ Error loading simple orders:', error);
        return [];
    }
}

async function saveSimpleOrders(orders) {
    try {
        await fs.writeFile(SIMPLE_ORDERS_FILE, JSON.stringify(orders, null, 2));
        console.log(`💾 Saved ${orders.length} simple orders`);
    } catch (error) {
        console.error('❌ Error saving simple orders:', error);
        throw error;
    }
}

async function ensureSimpleOrdersFileExists() {
    try {
        await fs.access(SIMPLE_ORDERS_FILE);
    } catch (error) {
        console.log('📁 Creating simple_orders.json file');
        await fs.writeFile(SIMPLE_ORDERS_FILE, JSON.stringify([], null, 2));
    }
}

async function updateOrderStatus(orderId, status, reason = null) {
    try {
        const orders = await loadSimpleOrders();
        const orderIndex = orders.findIndex(o => o.id === orderId);
        
        if (orderIndex !== -1) {
            orders[orderIndex].status = status;
            orders[orderIndex].updatedAt = new Date().toISOString();
            
            if (status === 'completed') {
                orders[orderIndex].completedAt = new Date().toISOString();
            }
            
            if (reason) {
                orders[orderIndex].statusReason = reason;
            }
            
            await saveSimpleOrders(orders);
            console.log(`🔄 Order ${orderId} status updated: ${status}`);
        }
    } catch (error) {
        console.error('❌ Error updating order status:', error);
    }
}

// WebSocket broadcasting functions (implement based on your WebSocket setup)
function broadcastOrderFailure(orderId, coordinateIndex, error) {
    try {
        if (global.webSocketBroadcast) {
            global.webSocketBroadcast('coordinate_order_failure', {
                orderId: orderId,
                failedCoordinate: coordinateIndex,
                error: error,
                timestamp: new Date().toISOString(),
                showRestartPopup: true  // Signal frontend to show restart popup
            });
        }
    } catch (error) {
        console.error('❌ Error broadcasting order failure:', error);
    }
}

function broadcastOrderCompletion(orderId) {
    try {
        if (global.webSocketBroadcast) {
            global.webSocketBroadcast('coordinate_order_completed', {
                orderId: orderId,
                timestamp: new Date().toISOString()
            });
        }
    } catch (error) {
        console.error('❌ Error broadcasting order completion:', error);
    }
}

function broadcastOrderDeletion(deviceId, orderId) {
    try {
        if (global.webSocketBroadcast) {
            global.webSocketBroadcast('order_deleted', {
                deviceId: deviceId,
                orderId: orderId,
                type: 'coordinate_order',
                timestamp: new Date().toISOString()
            });
        }
    } catch (error) {
        console.error('❌ Error broadcasting order deletion:', error);
    }
}

module.exports = router;

// ==========================================
// ADD TO YOUR app.js:
// ==========================================
/*
// Add this line to your app.js:
const simpleCoordinateRoutes = require('./routes/simple_coordinate_orders');
app.use('/api/simple-orders', simpleCoordinateRoutes);
*/