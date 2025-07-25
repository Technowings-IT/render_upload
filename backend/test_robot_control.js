#!/usr/bin/env node
// Test script for enhanced robot control functionality

const ROS2ScriptManager = require('./ros/utils/ros2ScriptManager');

async function testRobotControl() {
    console.log('🔬 Testing Enhanced Robot Control Functionality');
    console.log('=' .repeat(60));
    
    const manager = new ROS2ScriptManager();
    
    try {
        // Test 1: Pi Diagnostics
        console.log('\n📋 Test 1: Running Pi Diagnostics...');
        const diagnostics = await manager.checkPiStatus();
        console.log('📊 Diagnostics Result:', JSON.stringify(diagnostics, null, 2));
        
        // Test 2: SSH Connectivity
        console.log('\n🔗 Test 2: Testing SSH Connectivity...');
        const sshTest = await manager.testSSHConnectivity();
        console.log('✅ SSH Test Result:', sshTest);
        
        // Test 3: Start Robot Control
        console.log('\n🤖 Test 3: Starting Robot Control...');
        const startResult = await manager.startRobotControl();
        console.log('🚀 Start Result:', JSON.stringify(startResult, null, 2));
        
        // Wait a bit then check status
        console.log('\n⏳ Waiting 5 seconds to check status...');
        await new Promise(resolve => setTimeout(resolve, 5000));
        
        // Test 4: Get Status
        console.log('\n📊 Test 4: Getting Process Status...');
        const status = manager.getDetailedStatus();
        console.log('📈 Status:', JSON.stringify(status, null, 2));
        
        // Test 5: Stop Robot Control
        console.log('\n🛑 Test 5: Stopping Robot Control...');
        const stopResult = await manager.stopProcess('robot_control');
        console.log('⏹️ Stop Result:', JSON.stringify(stopResult, null, 2));
        
    } catch (error) {
        console.error('❌ Test Error:', error.message);
        console.error('📋 Stack:', error.stack);
    } finally {
        console.log('\n🧹 Cleaning up...');
        await manager.cleanup();
        console.log('✅ Test completed');
        process.exit(0);
    }
}

// Handle process termination
process.on('SIGINT', () => {
    console.log('\n🛑 Test interrupted by user');
    process.exit(0);
});

process.on('SIGTERM', () => {
    console.log('\n🛑 Test terminated');
    process.exit(0);
});

// Run the test
testRobotControl().catch(error => {
    console.error('💥 Fatal test error:', error);
    process.exit(1);
});
