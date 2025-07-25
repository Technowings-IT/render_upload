// Create this file as backend/test_server.js
const http = require('http');
const WebSocket = require('ws');

const SERVER_HOST = '192.168.0.55';
const SERVER_PORT = 3000;

async function testBackendServer() {
    console.log('🚀 Testing AGV Fleet Management Backend...\n');
    
    // Test 1: HTTP Health Check
    console.log('1️⃣ Testing HTTP API...');
    try {
        const healthResponse = await makeRequest('GET', '/api/health');
        console.log('✅ Health endpoint working');
        console.log(`   Status: ${healthResponse.data?.status}`);
        console.log(`   Connected devices: ${healthResponse.data?.connectedDevices}`);
        console.log(`   ROS2 status: ${healthResponse.data?.ros2Status}`);
    } catch (error) {
        console.log('❌ Health endpoint failed:', error.message);
        return;
    }
    
    // Test 2: Device endpoints
    console.log('\n2️⃣ Testing device endpoints...');
    try {
        const devices = await makeRequest('GET', '/api/devices');
        console.log('✅ Devices endpoint working');
        console.log(`   Found ${devices.devices?.length || 0} devices`);
    } catch (error) {
        console.log('❌ Devices endpoint failed:', error.message);
    }
    
    // Test 3: WebSocket Connection
    console.log('\n3️⃣ Testing WebSocket connection...');
    testWebSocket();
}

function makeRequest(method, path) {
    return new Promise((resolve, reject) => {
        const options = {
            hostname: SERVER_HOST,
            port: SERVER_PORT,
            path: path,
            method: method,
            headers: {
                'Content-Type': 'application/json',
            },
            timeout: 5000
        };
        
        const req = http.request(options, (res) => {
            let data = '';
            
            res.on('data', (chunk) => {
                data += chunk;
            });
            
            res.on('end', () => {
                try {
                    const parsed = JSON.parse(data);
                    if (res.statusCode >= 200 && res.statusCode < 300) {
                        resolve(parsed);
                    } else {
                        reject(new Error(`HTTP ${res.statusCode}: ${parsed.message || 'Unknown error'}`));
                    }
                } catch (e) {
                    reject(new Error(`Failed to parse response: ${e.message}`));
                }
            });
        });
        
        req.on('error', (error) => {
            reject(new Error(`Request failed: ${error.message}`));
        });
        
        req.on('timeout', () => {
            req.destroy();
            reject(new Error('Request timeout'));
        });
        
        req.end();
    });
}

function testWebSocket() {
    const wsUrl = `ws://${SERVER_HOST}:${SERVER_PORT}`;
    console.log(`   Connecting to: ${wsUrl}`);
    
    const ws = new WebSocket(wsUrl);
    
    const timeout = setTimeout(() => {
        console.log('❌ WebSocket connection timeout');
        ws.close();
    }, 10000);
    
    ws.on('open', () => {
        clearTimeout(timeout);
        console.log('✅ WebSocket connected successfully');
        
        // Test sending a message
        ws.send(JSON.stringify({
            type: 'ping',
            timestamp: new Date().toISOString()
        }));
        console.log('   Sent ping message');
    });
    
    ws.on('message', (data) => {
        try {
            const message = JSON.parse(data);
            console.log(`   📨 Received: ${message.type}`);
            
            if (message.type === 'connection') {
                console.log(`   Client ID: ${message.clientId}`);
                console.log(`   Capabilities: ${Object.keys(message.capabilities || {}).join(', ')}`);
            }
        } catch (e) {
            console.log(`   📨 Received raw: ${data}`);
        }
    });
    
    ws.on('error', (error) => {
        clearTimeout(timeout);
        console.log('❌ WebSocket error:', error.message);
    });
    
    ws.on('close', () => {
        clearTimeout(timeout);
        console.log('🔌 WebSocket connection closed');
        console.log('\n🎉 Backend test completed!');
    });
    
    // Close connection after 5 seconds
    setTimeout(() => {
        ws.close();
    }, 5000);
}

// Run the test
if (require.main === module) {
    testBackendServer().catch(console.error);
}

module.exports = { testBackendServer };