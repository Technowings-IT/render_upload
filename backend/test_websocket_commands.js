#!/usr/bin/env node
// Test script to verify WebSocket script commands work

const WebSocket = require('ws');

const ws = new WebSocket('ws://localhost:3000');

ws.on('open', function open() {
    console.log('🔌 Connected to WebSocket server');
    
    // Test 1: Start robot control
    console.log('\n🤖 Test 1: Starting robot control...');
    const robotControlCommand = {
        type: 'script_command',
        deviceId: 'piros',
        command: 'start_robot_control',
        options: {}
    };
    
    ws.send(JSON.stringify(robotControlCommand));
});

ws.on('message', function message(data) {
    try {
        const parsedData = JSON.parse(data);
        console.log('📥 Received:', {
            type: parsedData.type,
            success: parsedData.success,
            command: parsedData.command,
            message: parsedData.data?.message,
            pid: parsedData.data?.pid
        });
        
        // After robot control starts, test SLAM
        if (parsedData.type === 'script_command_response' && 
            parsedData.command === 'start_robot_control' && 
            parsedData.success) {
            
            console.log('\n🗺️ Test 2: Starting SLAM...');
            setTimeout(() => {
                const slamCommand = {
                    type: 'script_command',
                    deviceId: 'piros',
                    command: 'start_slam',
                    options: {
                        mapName: 'test_slam_map_' + Date.now()
                    }
                };
                ws.send(JSON.stringify(slamCommand));
            }, 3000);
        }
        
        // After SLAM starts, test navigation
        if (parsedData.type === 'script_command_response' && 
            parsedData.command === 'start_slam' && 
            parsedData.success) {
            
            console.log('\n🚀 Test 3: Starting navigation...');
            setTimeout(() => {
                const navCommand = {
                    type: 'script_command',
                    deviceId: 'piros',
                    command: 'start_navigation',
                    options: {
                        mapPath: '/home/piros/fleet-management-system/ros_ws/src/AMR/maps/map_1750065869.yaml'
                    }
                };
                ws.send(JSON.stringify(navCommand));
            }, 3000);
        }
        
        // After navigation starts, test stop all
        if (parsedData.type === 'script_command_response' && 
            parsedData.command === 'start_navigation' && 
            parsedData.success) {
            
            console.log('\n🛑 Test 4: Stopping all scripts...');
            setTimeout(() => {
                const stopCommand = {
                    type: 'script_command',
                    deviceId: 'piros',
                    command: 'stop_all_scripts',
                    options: {}
                };
                ws.send(JSON.stringify(stopCommand));
            }, 5000);
        }
        
        // Close after stop all completes
        if (parsedData.type === 'script_command_response' && 
            parsedData.command === 'stop_all_scripts' && 
            parsedData.success) {
            
            console.log('\n✅ All tests completed successfully!');
            setTimeout(() => {
                ws.close();
                process.exit(0);
            }, 2000);
        }
        
    } catch (error) {
        console.error('❌ Error parsing message:', error);
        console.log('Raw message:', data.toString());
    }
});

ws.on('error', function error(err) {
    console.error('❌ WebSocket error:', err);
});

ws.on('close', function close() {
    console.log('🔌 WebSocket connection closed');
});

// Timeout after 60 seconds
setTimeout(() => {
    console.log('⏰ Test timeout - closing connection');
    ws.close();
    process.exit(1);
}, 60000);
