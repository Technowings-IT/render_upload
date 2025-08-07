//routes/analyticsRoutes.js - Analytics API for AMR Fleet Management
const express = require('express');
const router = express.Router();

// Health check endpoint for analytics
router.get('/health', (req, res) => {
    res.json({
        success: true,
        message: 'Analytics API is healthy',
        availableTypes: ['battery', 'orders', 'stats', 'events', 'performance'],
        timeRanges: ['1h', '6h', '24h', '7d', '30d'],
        timestamp: new Date().toISOString()
    });
});

// Get analytics data
router.get('/', async (req, res) => {
    try {
        const { type, timeRange = '24h', deviceId } = req.query;

        console.log(`📊 Analytics request: type=${type}, timeRange=${timeRange}, deviceId=${deviceId}`);

        // Validate required parameters
        if (!type) {
            return res.status(400).json({
                success: false,
                error: 'Missing required parameter: type',
                validTypes: ['battery', 'orders', 'stats', 'events', 'performance'],
                timestamp: new Date().toISOString()
            });
        }

        let analyticsData;

        switch (type) {
            case 'battery':
                analyticsData = await getBatteryAnalytics(deviceId, timeRange);
                break;
            case 'orders':
                analyticsData = await getOrdersAnalytics(deviceId, timeRange);
                break;
            case 'stats':
                analyticsData = await getStatsAnalytics(deviceId, timeRange);
                break;
            case 'events':
                analyticsData = await getEventsAnalytics(timeRange);
                break;
            case 'performance':
                analyticsData = await getPerformanceAnalytics(deviceId, timeRange);
                break;
            default:
                return res.status(400).json({
                    success: false,
                    error: `Invalid analytics type: ${type}`,
                    validTypes: ['battery', 'orders', 'stats', 'events', 'performance'],
                    timestamp: new Date().toISOString()
                });
        }

        res.json({
            success: true,
            type,
            timeRange,
            deviceId: deviceId || 'all',
            data: sanitizeAnalyticsData(analyticsData),
            timestamp: new Date().toISOString()
        });

    } catch (error) {
        console.error('❌ Analytics API error:', error);
        res.status(500).json({
            success: false,
            error: 'Failed to fetch analytics data',
            details: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

// Health check endpoint for analytics
router.get('/health', (req, res) => {
    res.json({
        success: true,
        message: 'Analytics API is healthy',
        availableTypes: ['battery', 'orders', 'stats', 'events', 'performance'],
        timeRanges: ['1h', '6h', '24h', '7d', '30d'],
        timestamp: new Date().toISOString()
    });
});

// Get battery analytics
async function getBatteryAnalytics(deviceId, timeRange) {
    const endTime = new Date();
    const startTime = getStartTime(endTime, timeRange);

    // Get current battery data from global live data
    const currentBatteryData = getBatteryFromLiveData(deviceId);
    
    // Generate historical battery data (mock for now - in production, you'd query from database)
    const historicalData = generateBatteryHistory(deviceId, startTime, endTime);

    return {
        current: currentBatteryData,
        history: historicalData,
        summary: {
            averageLevel: historicalData.length > 0 ? 
                parseFloat((historicalData.reduce((sum, d) => sum + d.percentage, 0) / historicalData.length).toFixed(2)) : 0.0,
            minLevel: historicalData.length > 0 ? 
                parseFloat(Math.min(...historicalData.map(d => d.percentage)).toFixed(2)) : 0.0,
            maxLevel: historicalData.length > 0 ? 
                parseFloat(Math.max(...historicalData.map(d => d.percentage)).toFixed(2)) : 100.0,
            chargeCycles: Math.floor(Math.random() * 5), // Mock data
            totalChargingTime: Math.floor(Math.random() * 120) // minutes
        }
    };
}

// Get orders analytics
async function getOrdersAnalytics(deviceId, timeRange) {
    const endTime = new Date();
    const startTime = getStartTime(endTime, timeRange);

    // Get orders from storage (if available)
    let orders = [];
    try {
        if (global.storageManager) {
            const deviceOrders = await global.storageManager.getOrdersForDevice(deviceId || 'piros');
            orders = deviceOrders.filter(order => {
                const orderTime = new Date(order.createdAt || order.timestamp);
                return orderTime >= startTime && orderTime <= endTime;
            });
        }
    } catch (error) {
        console.warn('⚠️ Could not load orders from storage:', error.message);
    }

    // Generate analytics from orders
    const completedOrders = orders.filter(o => o.status === 'completed');
    const pendingOrders = orders.filter(o => o.status === 'pending');
    const failedOrders = orders.filter(o => o.status === 'failed');

    return {
        total: orders.length,
        completed: completedOrders.length,
        pending: pendingOrders.length,
        failed: failedOrders.length,
        successRate: orders.length > 0 ? parseFloat((completedOrders.length / orders.length * 100).toFixed(2)) : 100.0,
        averageCompletionTime: parseFloat(calculateAverageCompletionTime(completedOrders).toFixed(2)),
        ordersByHour: generateOrdersTimeline(orders, startTime, endTime),
        recentOrders: orders.slice(-10) // Last 10 orders
    };
}

// Get system stats analytics
async function getStatsAnalytics(deviceId, timeRange) {
    const liveData = global.liveData || {};
    const deviceData = liveData[deviceId] || {};

    // Get current system stats
    const currentStats = {
        position: deviceData.position || {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        },
        battery: deviceData.battery || {
            percentage: 0.0,
            voltage: 0.0,
            current: 0.0,
            charging: false
        },
        velocity: deviceData.velocity_feedback || {
            linear: { x: 0.0, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: 0.0 }
        },
        navigationStatus: deviceData.navigation_status || {
            status: "unknown",
            goal_reached: false,
            distance_to_goal: 0.0
        },
        lastUpdate: deviceData.lastUpdate || new Date().toISOString()
    };

    // Calculate uptime and connectivity stats
    const uptimeStats = calculateUptimeStats(deviceId, timeRange);
    const connectivityStats = calculateConnectivityStats(deviceId, timeRange);

    return {
        current: currentStats,
        uptime: uptimeStats,
        connectivity: connectivityStats,
        systemHealth: {
            rosConnected: global.rosConnection ? await global.rosConnection.isHealthy() : false,
            webSocketConnected: global.webSocketConnections > 0,
            storageHealthy: global.storageManager ? true : false,
            subscribersActive: global.subscribersActive || 0
        },
        resourceUsage: {
            memoryUsage: process.memoryUsage(),
            cpuUsage: process.cpuUsage(),
            uptime: process.uptime()
        }
    };
}

// Get events analytics
async function getEventsAnalytics(timeRange) {
    const endTime = new Date();
    const startTime = getStartTime(endTime, timeRange);

    // Generate mock events data (in production, you'd query from logs/database)
    const events = generateSystemEvents(startTime, endTime);

    const errorEvents = events.filter(e => e.level === 'error');
    const warningEvents = events.filter(e => e.level === 'warning');
    const infoEvents = events.filter(e => e.level === 'info');

    // Get event groupings
    const eventsByType = groupEventsByType(events);

    return {
        total: events.length,
        errors: errorEvents.length,
        warnings: warningEvents.length,
        info: infoEvents.length,
        recentEvents: events.slice(0, 20), // First 20 events (most recent)
        eventsByType: eventsByType.byType,
        eventsGrouped: eventsByType.array, // Flutter-friendly format
        timeline: generateEventsTimeline(events, startTime, endTime),
        summary: {
            errorRate: events.length > 0 ? (errorEvents.length / events.length * 100) : 0,
            warningRate: events.length > 0 ? (warningEvents.length / events.length * 100) : 0,
            mostCommonType: eventsByType.array.length > 0 ? 
                eventsByType.array.reduce((a, b) => a.count > b.count ? a : b).type : 'none'
        }
    };
}

// Get performance analytics
async function getPerformanceAnalytics(deviceId, timeRange) {
    const endTime = new Date();
    const startTime = getStartTime(endTime, timeRange);

    // Get performance data from live data
    const liveData = global.liveData || {};
    const deviceData = liveData[deviceId] || {};

    // Calculate performance metrics
    const velocityData = deviceData.velocity_feedback || {};
    const positionData = deviceData.position || {};

    return {
        navigation: {
            averageSpeed: parseFloat((velocityData.linear?.x || 0).toFixed(2)),
            maxSpeed: parseFloat((Math.random() * 2).toFixed(2)), // Mock data
            totalDistance: parseFloat((Math.random() * 1000).toFixed(2)), // Mock data in meters
            pathEfficiency: parseFloat((85 + Math.random() * 10).toFixed(2)), // Mock efficiency percentage
            estimatedTimeRemaining: parseFloat((Math.random() * 300 + 60).toFixed(0)), // Mock estimated time in seconds
            navigationTime: parseFloat((Math.random() * 1800 + 300).toFixed(0)), // Mock navigation time in seconds
            recoveryCount: Math.floor(Math.random() * 5), // Mock recovery events count
            currentNavigationStatus: ['navigating', 'idle', 'paused'][Math.floor(Math.random() * 3)]
        },
        localization: {
            accuracy: parseFloat((95 + Math.random() * 5).toFixed(2)), // Mock accuracy percentage
            driftRate: parseFloat((Math.random() * 0.1).toFixed(4)), // Mock drift in meters/hour
            lastKnownPosition: positionData.position || {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        },
        communication: {
            latency: parseFloat((15 + Math.random() * 10).toFixed(2)), // Mock latency in ms
            packetLoss: parseFloat((Math.random() * 2).toFixed(2)), // Mock packet loss percentage
            throughput: parseFloat((50 + Math.random() * 50).toFixed(2)) // Mock throughput in KB/s
        },
        system: {
            responseTime: parseFloat((100 + Math.random() * 50).toFixed(2)), // Mock response time in ms
            errorRate: parseFloat((Math.random() * 5).toFixed(2)), // Mock error rate percentage
            availability: parseFloat((95 + Math.random() * 5).toFixed(2)) // Mock availability percentage
        }
    };
}

// Helper functions
function getStartTime(endTime, timeRange) {
    const ranges = {
        '1h': 1 * 60 * 60 * 1000,
        '6h': 6 * 60 * 60 * 1000,
        '24h': 24 * 60 * 60 * 1000,
        '7d': 7 * 24 * 60 * 60 * 1000,
        '30d': 30 * 24 * 60 * 60 * 1000
    };
    
    const offset = ranges[timeRange] || ranges['24h'];
    return new Date(endTime.getTime() - offset);
}

function getBatteryFromLiveData(deviceId) {
    const liveData = global.liveData || {};
    const deviceData = liveData[deviceId] || {};
    const batteryData = deviceData.battery;

    if (!batteryData) {
        return {
            percentage: parseFloat((85 + Math.random() * 15).toFixed(2)),
            voltage: parseFloat((12.0 + Math.random() * 2).toFixed(2)),
            current: parseFloat((Math.random() * 5).toFixed(2)),
            charging: Math.random() > 0.7,
            timestamp: new Date().toISOString()
        };
    }

    return {
        percentage: parseFloat((batteryData.percentage || 0).toFixed(2)),
        voltage: parseFloat((batteryData.voltage || 0).toFixed(2)),
        current: parseFloat((batteryData.current || 0).toFixed(2)),
        charging: batteryData.power_supply_status === 1, // 1 = charging
        timestamp: batteryData.timestamp || new Date().toISOString()
    };
}

function generateBatteryHistory(deviceId, startTime, endTime) {
    const history = [];
    const intervalMs = 30 * 60 * 1000; // 30 minutes
    let currentLevel = 85 + Math.random() * 15;
    
    for (let time = startTime.getTime(); time <= endTime.getTime(); time += intervalMs) {
        // Simulate battery drain and charging
        currentLevel += (Math.random() - 0.6) * 5; // Slight bias towards draining
        currentLevel = Math.max(10, Math.min(100, currentLevel));
        
        history.push({
            timestamp: new Date(time).toISOString(),
            percentage: parseFloat(currentLevel.toFixed(1)),
            voltage: parseFloat((11.5 + (currentLevel / 100) * 1.5).toFixed(2)),
            charging: currentLevel < 20 || Math.random() > 0.8
        });
    }
    
    return history;
}

function calculateAverageCompletionTime(completedOrders) {
    if (completedOrders.length === 0) return 0;
    
    const totalTime = completedOrders.reduce((sum, order) => {
        const start = new Date(order.createdAt || order.timestamp);
        const end = new Date(order.completedAt || order.updatedAt || order.timestamp);
        return sum + (end.getTime() - start.getTime());
    }, 0);
    
    return totalTime / completedOrders.length / 1000; // Return in seconds
}

function generateOrdersTimeline(orders, startTime, endTime) {
    const timeline = [];
    const intervalMs = 60 * 60 * 1000; // 1 hour intervals
    
    for (let time = startTime.getTime(); time <= endTime.getTime(); time += intervalMs) {
        const hourStart = new Date(time);
        const hourEnd = new Date(time + intervalMs);
        
        const ordersInHour = orders.filter(order => {
            const orderTime = new Date(order.createdAt || order.timestamp);
            return orderTime >= hourStart && orderTime < hourEnd;
        });
        
        timeline.push({
            timestamp: hourStart.toISOString(),
            count: ordersInHour.length,
            completed: ordersInHour.filter(o => o.status === 'completed').length,
            failed: ordersInHour.filter(o => o.status === 'failed').length
        });
    }
    
    return timeline;
}

function calculateUptimeStats(deviceId, timeRange) {
    // Mock uptime calculation (in production, track actual uptime)
    const totalTime = getStartTime(new Date(), timeRange);
    const uptime = 95 + Math.random() * 5; // Mock 95-100% uptime
    
    return {
        percentage: Math.round(uptime * 10) / 10,
        totalSeconds: Math.floor((Date.now() - totalTime.getTime()) / 1000),
        downtimeEvents: Math.floor(Math.random() * 3), // Mock downtime events
        lastDowntime: new Date(Date.now() - Math.random() * 24 * 60 * 60 * 1000).toISOString()
    };
}

function calculateConnectivityStats(deviceId, timeRange) {
    return {
        rosConnection: 98 + Math.random() * 2, // Mock 98-100% ROS connectivity
        webSocketConnection: 99 + Math.random() * 1, // Mock WebSocket connectivity
        networkLatency: 15 + Math.random() * 10, // Mock latency in ms
        packetLoss: Math.random() * 2, // Mock packet loss percentage
        reconnections: Math.floor(Math.random() * 5) // Mock reconnection count
    };
}

function generateSystemEvents(startTime, endTime) {
    const events = [];
    const eventTypes = [
        { type: 'navigation', level: 'info', message: 'Navigation goal reached' },
        { type: 'battery', level: 'warning', message: 'Battery level below 20%' },
        { type: 'connection', level: 'error', message: 'WebSocket connection lost' },
        { type: 'system', level: 'info', message: 'System health check completed' },
        { type: 'order', level: 'info', message: 'New order received' },
        { type: 'sensor', level: 'warning', message: 'Sensor calibration required' }
    ];
    
    const eventCount = Math.floor(Math.random() * 50) + 20; // 20-70 events
    
    for (let i = 0; i < eventCount; i++) {
        const eventTemplate = eventTypes[Math.floor(Math.random() * eventTypes.length)];
        const eventTime = new Date(startTime.getTime() + Math.random() * (endTime.getTime() - startTime.getTime()));
        
        events.push({
            id: `event_${eventTime.getTime()}_${i}`,
            timestamp: eventTime.toISOString(),
            type: eventTemplate.type,
            level: eventTemplate.level,
            message: eventTemplate.message,
            deviceId: 'piros'
        });
    }
    
    return events.sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp));
}

function groupEventsByType(events) {
    const groups = {};
    events.forEach(event => {
        const eventType = event.type || 'unknown';
        if (!groups[eventType]) {
            groups[eventType] = 0;
        }
        groups[eventType]++;
    });
    
    // Convert to array of objects for better Flutter compatibility
    const groupsArray = Object.keys(groups).map(type => ({
        type: type,
        count: groups[type]
    }));
    
    return {
        byType: groups,           // Original object format
        array: groupsArray,       // Array format for Flutter
        total: events.length
    };
}

function generateEventsTimeline(events, startTime, endTime) {
    const timeline = [];
    const intervalMs = 60 * 60 * 1000; // 1 hour intervals
    
    for (let time = startTime.getTime(); time <= endTime.getTime(); time += intervalMs) {
        const hourStart = new Date(time);
        const hourEnd = new Date(time + intervalMs);
        
        const eventsInHour = events.filter(event => {
            const eventTime = new Date(event.timestamp);
            return eventTime >= hourStart && eventTime < hourEnd;
        });
        
        timeline.push({
            timestamp: hourStart.toISOString(),
            total: eventsInHour.length,
            errors: eventsInHour.filter(e => e.level === 'error').length,
            warnings: eventsInHour.filter(e => e.level === 'warning').length,
            info: eventsInHour.filter(e => e.level === 'info').length
        });
    }
    
    return timeline;
}

// Utility function to ensure all numeric values are safe for Flutter
function sanitizeNumericValue(value, defaultValue = 0.0) {
    if (value === null || value === undefined || isNaN(value)) {
        return parseFloat(defaultValue.toFixed(2));
    }
    const numValue = Number(value);
    return parseFloat(numValue.toFixed(2)); // Ensure proper decimal formatting
}

// Utility function to sanitize analytics data for Flutter compatibility
function sanitizeAnalyticsData(data) {
    if (Array.isArray(data)) {
        return data.map(item => sanitizeAnalyticsData(item));
    } else if (data !== null && typeof data === 'object') {
        const sanitized = {};
        for (const [key, value] of Object.entries(data)) {
            if (typeof value === 'number') {
                // Ensure all numbers are properly formatted as doubles
                sanitized[key] = parseFloat(value.toFixed(2));
            } else if (typeof value === 'string' && !isNaN(Number(value))) {
                // Convert numeric strings to proper doubles
                sanitized[key] = parseFloat(Number(value).toFixed(2));
            } else if (typeof value === 'object' && value !== null) {
                sanitized[key] = sanitizeAnalyticsData(value);
            } else if (value === null && (key.includes('percentage') || key.includes('voltage') || key.includes('current') || key.includes('speed') || key.includes('distance') || key.includes('efficiency') || key.includes('accuracy') || key.includes('rate') || key.includes('time') || key.includes('latency') || key.includes('x') || key.includes('y') || key.includes('z') || key.includes('w'))) {
                // Replace null numeric fields with 0.0
                sanitized[key] = 0.0;
            } else {
                sanitized[key] = value;
            }
        }
        return sanitized;
    }
    return data;
}

module.exports = router;
