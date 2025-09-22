// routes/discoveryRoutes.js - Device Discovery API for AMR Fleet Management
const express = require('express');
const dgram = require('dgram');
const os = require('os');
const config = require('../config');
const rosConnection = require('../ros/utils/ros_connection');

const router = express.Router();

//  NEW: HTTP endpoint for device discovery
router.get('/discover', async (req, res) => {
    try {
        const networkInterfaces = os.networkInterfaces();
        const serverInfo = getServerInfo();
        const rosStatus = rosConnection.getROS2Status();
        
        // Get all available devices
        const devices = global.connectedDevices || [];
        
        res.json({
            success: true,
            server: {
                ...serverInfo,
                ros2Status: rosStatus.initialized,
                connectedDevices: devices.length,
                capabilities: config.DEVICE.CAPABILITIES,
                networkInterfaces: getNetworkInfo()
            },
            devices: devices.map(device => ({
                ...device,
                liveData: global.liveData[device.id] || {},
                mappingStatus: global.deviceMappingStates[device.id] || { active: false }
            })),
            timestamp: new Date().toISOString()
        });
        
        console.log(` Discovery request handled - ${devices.length} devices available`);
        
    } catch (error) {
        console.error(' Error handling discovery request:', error);
        res.status(500).json({
            success: false,
            error: 'Discovery failed',
            details: error.message
        });
    }
});

//  NEW: WebSocket endpoint info
router.get('/websocket-info', (req, res) => {
    try {
        const serverInfo = getServerInfo();
        
        res.json({
            success: true,
            websocket: {
                url: `ws://${serverInfo.ip}:${serverInfo.port}`,
                protocols: ['AMR-protocol'],
                capabilities: [
                    'real_time_control',
                    'live_mapping',
                    'device_monitoring',
                    'order_management'
                ],
                messageTypes: Object.values(config.WEBSOCKET.MESSAGE_TYPES)
            },
            server: serverInfo,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error(' Error getting WebSocket info:', error);
        res.status(500).json({
            success: false,
            error: 'Failed to get WebSocket info'
        });
    }
});

//  NEW: Auto-connect endpoint for AMRs
router.post('/auto-connect', async (req, res) => {
    try {
        const clientIP = req.ip || req.connection.remoteAddress || req.socket.remoteAddress;
        const { deviceId, name, capabilities } = req.body;
        
        const autoDeviceId = deviceId || `AMR_${clientIP.replace(/\./g, '_')}`;
        const deviceName = name || `AMR at ${clientIP}`;
        
        const deviceInfo = {
            id: autoDeviceId,
            name: deviceName,
            type: 'differential_drive',
            ipAddress: clientIP,
            capabilities: capabilities || config.DEVICE.CAPABILITIES,
            autoConnected: true,
            discoveryMethod: 'auto_connect',
            connectedAt: new Date().toISOString()
        };
        
        // Add device via ROS connection manager
        const connectedDeviceId = rosConnection.addConnectedDevice(deviceInfo);
        
        // Save device (if storage manager is available)
        try {
            const storageManager = require('../ros/utils/storageManager');
            await storageManager.saveDevice(connectedDeviceId);
        } catch (storageError) {
            console.warn('️ Could not save device to storage:', storageError.message);
        }
        
        res.json({
            success: true,
            message: 'Device auto-connected successfully',
            deviceId: connectedDeviceId,
            device: deviceInfo,
            server: getServerInfo(),
            websocket: {
                url: `ws://${getServerInfo().ip}:${config.SERVER.PORT}`,
                connectMessage: {
                    type: 'device_connect',
                    deviceId: connectedDeviceId,
                    deviceInfo: deviceInfo
                }
            }
        });
        
        console.log(` Auto-connected device: ${connectedDeviceId} from ${clientIP}`);
        
    } catch (error) {
        console.error(' Error auto-connecting device:', error);
        res.status(500).json({
            success: false,
            error: 'Auto-connection failed',
            details: error.message
        });
    }
});

//  NEW: Network scan endpoint
router.get('/network-scan', async (req, res) => {
    try {
        const { subnet, startIP, endIP } = req.query;
        
        const targetSubnet = subnet || config.NETWORK.DISCOVERY.AMR_SUBNET;
        const start = parseInt(startIP) || config.NETWORK.DISCOVERY.AMR_IP_RANGE.START;
        const end = parseInt(endIP) || config.NETWORK.DISCOVERY.AMR_IP_RANGE.END;
        
        console.log(` Starting network scan: ${targetSubnet}.${start}-${end}`);
        
        const discoveredDevices = await scanNetworkForAMRs(targetSubnet, start, end);
        
        res.json({
            success: true,
            subnet: targetSubnet,
            ipRange: `${start}-${end}`,
            devicesFound: discoveredDevices.length,
            devices: discoveredDevices,
            timestamp: new Date().toISOString()
        });
        
        console.log(` Network scan complete: ${discoveredDevices.length} devices found`);
        
    } catch (error) {
        console.error(' Network scan failed:', error);
        res.status(500).json({
            success: false,
            error: 'Network scan failed',
            details: error.message
        });
    }
});

//  NEW: Health check with discovery info
router.get('/health', (req, res) => {
    try {
        const rosStatus = rosConnection.getROS2Status();
        const serverInfo = getServerInfo();
        
        res.json({
            success: true,
            status: 'healthy',
            server: serverInfo,
            ros2: {
                initialized: rosStatus.initialized,
                nodeActive: rosStatus.nodeActive,
                topicsDiscovered: rosStatus.topicsDiscovered,
                availableTopics: rosStatus.availableTopics
            },
            devices: {
                total: global.connectedDevices?.length || 0,
                connected: global.connectedDevices?.filter(d => d.status === 'connected').length || 0,
                mapping: Object.values(global.deviceMappingStates || {}).filter(s => s.active).length
            },
            websocket: {
                url: `ws://${serverInfo.ip}:${serverInfo.port}`,
                available: true
            },
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        console.error(' Health check failed:', error);
        res.status(500).json({
            success: false,
            error: 'Health check failed'
        });
    }
});

// Helper Functions

function getServerInfo() {
    const networkInterfaces = os.networkInterfaces();
    let primaryIP = '127.0.0.1';
    
    // Try to find the primary network interface IP
    for (const [name, interfaces] of Object.entries(networkInterfaces)) {
        for (const iface of interfaces) {
            if (iface.family === 'IPv4' && !iface.internal) {
                // Prefer 192.168.0.x subnet if available
                if (iface.address.startsWith('192.168.0.')) {
                    primaryIP = iface.address;
                    break;
                } else if (primaryIP === '127.0.0.1') {
                    primaryIP = iface.address;
                }
            }
        }
        if (primaryIP.startsWith('192.168.0.')) break;
    }
    
    return {
        ip: primaryIP,
        port: config.SERVER.PORT,
        hostname: os.hostname(),
        platform: os.platform(),
        nodeVersion: process.version,
        uptime: process.uptime(),
        type: 'AMR_fleet_backend',
        version: '1.0.0'
    };
}

function getNetworkInfo() {
    const networkInterfaces = os.networkInterfaces();
    const interfaces = [];
    
    for (const [name, ifaces] of Object.entries(networkInterfaces)) {
        for (const iface of ifaces) {
            if (iface.family === 'IPv4' && !iface.internal) {
                interfaces.push({
                    name: name,
                    address: iface.address,
                    netmask: iface.netmask,
                    mac: iface.mac
                });
            }
        }
    }
    
    return interfaces;
}

async function scanNetworkForAMRs(subnet, startIP, endIP) {
    const discoveredDevices = [];
    const scanPromises = [];
    
    for (let i = startIP; i <= endIP; i++) {
        const ip = `${subnet}.${i}`;
        scanPromises.push(checkAMRAtIP(ip));
    }
    
    // Process in batches to avoid overwhelming the network
    const batchSize = 10;
    for (let i = 0; i < scanPromises.length; i += batchSize) {
        const batch = scanPromises.slice(i, i + batchSize);
        const results = await Promise.allSettled(batch);
        
        results.forEach(result => {
            if (result.status === 'fulfilled' && result.value) {
                discoveredDevices.push(result.value);
            }
        });
        
        // Small delay between batches
        if (i + batchSize < scanPromises.length) {
            await new Promise(resolve => setTimeout(resolve, 100));
        }
    }
    
    return discoveredDevices;
}

async function checkAMRAtIP(ip) {
    try {
        const http = require('http');
        
        return new Promise((resolve) => {
            const timeout = setTimeout(() => {
                resolve(null);
            }, 2000);
            
            const req = http.get(`http://${ip}:3000/api/health`, { timeout: 2000 }, (res) => {
                clearTimeout(timeout);
                
                if (res.statusCode === 200) {
                    let data = '';
                    res.on('data', chunk => data += chunk);
                    res.on('end', () => {
                        try {
                            const parsed = JSON.parse(data);
                            if (parsed.success || data.includes('AMR') || data.includes('fleet')) {
                                resolve({
                                    id: `AMR_${ip.replace(/\./g, '_')}`,
                                    name: `AMR at ${ip}`,
                                    ipAddress: ip,
                                    port: 3000,
                                    discoveryMethod: 'network_scan',
                                    status: 'discovered',
                                    healthData: parsed
                                });
                                return;
                            }
                        } catch (e) {
                            // Not JSON, but might still be an AMR
                        }
                        
                        resolve({
                            id: `device_${ip.replace(/\./g, '_')}`,
                            name: `Device at ${ip}`,
                            ipAddress: ip,
                            port: 3000,
                            discoveryMethod: 'network_scan',
                            status: 'discovered'
                        });
                    });
                } else {
                    resolve(null);
                }
            });
            
            req.on('error', () => {
                clearTimeout(timeout);
                resolve(null);
            });
            
            req.on('timeout', () => {
                clearTimeout(timeout);
                req.destroy();
                resolve(null);
            });
        });
        
    } catch (error) {
        return null;
    }
}

//  NEW: UDP broadcast listener for AMR discovery
function initializeUDPDiscovery() {
    try {
        const server = dgram.createSocket('udp4');
        
        server.on('message', (msg, rinfo) => {
            try {
                const message = JSON.parse(msg.toString());
                
                if (message.type === 'AMR_discovery') {
                    console.log(` Discovery request from ${rinfo.address}:${rinfo.port}`);
                    
                    // Send discovery response
                    const response = {
                        type: 'AMR_response',
                        id: 'AMR_fleet_backend',
                        name: 'AMR Fleet Backend',
                        ip: getServerInfo().ip,
                        port: config.SERVER.PORT,
                        websocket_port: config.SERVER.PORT,
                        services: ['http', 'websocket', 'ros2'],
                        capabilities: config.DEVICE.CAPABILITIES,
                        ros2_status: rosConnection.isRosInitialized() ? 'connected' : 'disconnected',
                        timestamp: new Date().toISOString()
                    };
                    
                    const responseBuffer = Buffer.from(JSON.stringify(response));
                    server.send(responseBuffer, rinfo.port, rinfo.address, (err) => {
                        if (err) {
                            console.error(' Error sending discovery response:', err);
                        } else {
                            console.log(` Sent discovery response to ${rinfo.address}:${rinfo.port}`);
                        }
                    });
                }
            } catch (error) {
                // Ignore invalid messages
            }
        });
        
        server.bind(config.NETWORK.DISCOVERY.PORT, () => {
            console.log(` UDP discovery server listening on port ${config.NETWORK.DISCOVERY.PORT}`);
        });
        
        return server;
        
    } catch (error) {
        console.error(' Failed to initialize UDP discovery:', error);
        return null;
    }
}

module.exports = {
    router,
    initializeUDPDiscovery
};