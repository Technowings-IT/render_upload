#!/usr/bin/env node
// Test script for robot control specifically

const ROS2ScriptManager = require('./ros/utils/ros2ScriptManager');

async function testRobotControlSpecific() {
    console.log('🤖 Testing Robot Control Script Execution');
    console.log('=' .repeat(50));
    
    const manager = new ROS2ScriptManager();
    
    try {
        // Test: Start Robot Control with detailed logging
        console.log('\n🚀 Starting Robot Control with enhanced logging...');
        const startResult = await manager.startRobotControl();
        console.log('✅ Start Result:', JSON.stringify(startResult, null, 2));
        
        // Check the log file
        console.log('\n📋 Reading log file...');
        const fs = require('fs');
        const logPath = '/home/vidit-agrawal/ agv-fleet-management /backend/storage/logs/ros_script_manager.log';
        
        if (fs.existsSync(logPath)) {
            const logs = fs.readFileSync(logPath, 'utf8');
            console.log('📄 Recent logs:');
            console.log(logs.split('\n').slice(-20).join('\n'));
        } else {
            console.log('❌ Log file not found at:', logPath);
        }
        
    } catch (error) {
        console.error('❌ Robot Control Error:', error.message);
        console.error('📋 Stack:', error.stack);
        
        // Check logs even on error
        const fs = require('fs');
        const logPath = '/home/vidit-agrawal/ agv-fleet-management /backend/storage/logs/ros_script_manager.log';
        
        if (fs.existsSync(logPath)) {
            const logs = fs.readFileSync(logPath, 'utf8');
            console.log('\n📄 Error logs:');
            console.log(logs.split('\n').slice(-10).join('\n'));
        }
    } finally {
        await manager.cleanup();
        process.exit(0);
    }
}

testRobotControlSpecific().catch(error => {
    console.error('💥 Fatal error:', error);
    process.exit(1);
});
