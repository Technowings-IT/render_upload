// Example: Complete Order Creation & Execution Integration
// This shows how the enhanced system works end-to-end

// ==========================================
// 1. USER CREATES ORDER VIA INTERACTIVE MAP
// ==========================================

/*
User opens Dashboard → Device Card → "Interactive Map Order"
- Interactive Map Order Creator opens
- User clicks on map at coordinates: (5.2, 8.1)
- Popup asks: "Add PICKUP station?"
- User names it: "Station A"
- User clicks at (15.7, 12.3)
- Popup asks: "Add DROP station?" 
- User names it: "Station B"
- User clicks "Create Order"
*/

// Frontend API call:
const orderData = {
  deviceId: 'AMR-001',
  name: 'Pickup-Drop Route A→B',
  priority: 5,
  waypoints: [
    {
      name: 'Station A',
      type: 'pickup',
      position: { x: 5.2, y: 8.1, z: 0.0 },
      orientation: 0.0,
      metadata: {
        createdAt: '2025-01-31T10:30:00Z',
        color: '0xFF0000FF'
      }
    },
    {
      name: 'Station B', 
      type: 'drop',
      position: { x: 15.7, y: 12.3, z: 0.0 },
      orientation: 1.57, // 90 degrees
      metadata: {
        createdAt: '2025-01-31T10:31:00Z',
        color: '0xFF8800FF'
      }
    }
  ]
};

// ==========================================
// 2. BACKEND CREATES ORDER
// ==========================================

// POST /api/orders/AMR-001
// Backend creates order with ID: order_1738324200123
const createdOrder = {
  id: 'order_1738324200123',
  deviceId: 'AMR-001',
  name: 'Pickup-Drop Route A→B',
  status: 'pending',
  waypoints: [
    {
      id: 'AMR-001_wp_1738324200123_0',
      stepNumber: 1,
      name: 'Station A',
      type: 'pickup',
      position: { x: 5.2, y: 8.1, z: 0.0 },
      completed: false
    },
    {
      id: 'AMR-001_wp_1738324200123_1', 
      stepNumber: 2,
      name: 'Station B',
      type: 'drop',
      position: { x: 15.7, y: 12.3, z: 0.0 },
      completed: false
    }
  ],
  progress: {
    totalWaypoints: 2,
    completedWaypoints: 0,
    percentage: 0
  },
  createdAt: '2025-01-31T10:32:00Z'
};

// ==========================================
// 3. USER EXECUTES ORDER FROM DASHBOARD
// ==========================================

/*
Dashboard shows new order with "Execute" button
User clicks "Execute"
*/

// Frontend calls enhanced execution API:
await apiService.executeOrderEnhanced({
  deviceId: 'AMR-001',
  orderId: 'order_1738324200123',
  immediateStart: true,
  executionMode: 'sequential'
});

// ==========================================
// 4. BACKEND ENHANCED EXECUTION STARTS
// ==========================================

// enhanced_order_execution.js creates execution context:
const executionContext = {
  orderId: 'order_1738324200123',
  deviceId: 'AMR-001',
  currentWaypointIndex: 0,
  status: 'initializing',
  startTime: '2025-01-31T10:35:00Z'
};

// Broadcasts via WebSocket:
// Example broadcast payload:
const orderExecutionStartedPayload = {
  type: 'order_execution_started',
  deviceId: 'AMR-001',
  orderId: 'order_1738324200123',
  executionContext: { /* ... */ }
};

// ==========================================
// 5. WAYPOINT 1 EXECUTION (Station A)
// ==========================================

// Backend calls publishTargetPose():
const waypointA = order.waypoints[0]; // Station A

// publishers.js publishGoal() is called:
const result = publishers.publishGoal(
  5.2,  // x coordinate  
  8.1,  // y coordinate
  0.0   // orientation
);

// This publishes to ROS2 topic: /move_base_simple/goal
// Message format:
/*
{
  header: {
    stamp: { sec: 1738324500, nanosec: 0 },
    frame_id: 'map'
  },
  pose: {
    position: { x: 5.2, y: 8.1, z: 0.0 },
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
  }
}
*/

// Broadcasts waypoint start:
const waypointExecutionStartedPayload = {
  type: 'waypoint_execution_started',
  deviceId: 'AMR-001',
  waypointIndex: 0,
  waypoint: { name: 'Station A', type: 'pickup', position: { x: 5.2, y: 8.1, z: 0.0 }, orientation: 0.0, metadata: { createdAt: '2025-01-31T10:30:00Z', color: '0xFF0000FF' } },
  progress: { current: 1, total: 2, percentage: 50 }
};

// ==========================================
// 6. NAVIGATION SIMULATION (5 seconds)
// ==========================================

// For demo: setTimeout simulates navigation completion
// In real system: ROS2 navigation feedback would trigger this

setTimeout(() => {
  handleWaypointCompletion('order_1738324200123', 0, true);
}, 5000);

// ==========================================
// 7. WAYPOINT 1 COMPLETED
// ==========================================

// Updates order:
order.waypoints[0].completed = true;
order.waypoints[0].completedAt = '2025-01-31T10:35:05Z';
order.progress.completedWaypoints = 1;
order.progress.percentage = 50;

// Broadcasts completion:
// Example broadcast for waypoint completion:
const waypointExecutionCompletedPayload = {
  type: 'waypoint_execution_completed',
  deviceId: 'AMR-001',
  waypointIndex: 0,
  success: true,
  progress: { current: 1, total: 2, percentage: 50 }
};

// ==========================================
// 8. WAYPOINT 2 EXECUTION (Station B)
// ==========================================

// Moves to next waypoint automatically:
const waypointB = order.waypoints[1]; // Station B

// publishGoal() called again:
publishers.publishGoal(15.7, 12.3, 1.57);

// ROS2 message for Station B:
const stationBMessage = {
  pose: {
    position: { x: 15.7, y: 12.3, z: 0.0 },
    orientation: { x: 0.0, y: 0.0, z: 0.707, w: 0.707 } // 90 degrees
  }
};

// ==========================================
// 9. WAYPOINT 2 COMPLETED & ORDER FINISHED
// ==========================================

// After 5 seconds simulation:
order.waypoints[1].completed = true;
order.status = 'completed';
order.completedAt = '2025-01-31T10:35:10Z';
order.actualDuration = 10; // seconds
order.progress.percentage = 100;

// Final broadcast:
const orderExecutionCompletedPayload = {
  type: 'order_execution_completed',
  deviceId: 'AMR-001',
  orderId: 'order_1738324200123',
  executionSummary: {
    duration: 10,
    totalWaypoints: 2,
    successfulWaypoints: 2,
    failedWaypoints: 0
  }
};

// ==========================================
// 10. FRONTEND REAL-TIME UPDATES
// ==========================================

/*
Dashboard receives WebSocket updates and shows:

Order Status: COMPLETED ✅
Progress: 100% (2/2 waypoints)
Duration: 10 seconds
Route: Station A → Station B

Timeline:
✅ 10:35:00 - Order execution started
✅ 10:35:00 - Navigating to Station A (5.2, 8.1)
✅ 10:35:05 - Reached Station A
✅ 10:35:05 - Navigating to Station B (15.7, 12.3)  
✅ 10:35:10 - Reached Station B
✅ 10:35:10 - Order completed successfully
*/

// ==========================================
// KEY INTEGRATION POINTS
// ==========================================

/*
✅ FRONTEND:
- Interactive map for coordinate selection
- Real-time progress visualization
- WebSocket updates for live tracking

✅ BACKEND:
- Enhanced execution engine
- Sequential waypoint processing
- Real-time status broadcasting

✅ ROS2 INTEGRATION:
- Uses existing publishers.js
- publishGoal() → /move_base_simple/goal
- Emergency stop capability
- Target pose format compatibility

✅ USER EXPERIENCE:
- Click map → Add coordinates
- Visual waypoint sequence
- Real-time execution feedback
- Pause/Resume/Cancel controls
*/
