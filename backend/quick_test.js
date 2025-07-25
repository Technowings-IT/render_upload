#!/usr/bin/env node
// Quick test for robot control

const ROS2ScriptManager = require('./ros/utils/ros2ScriptManager');

async function quickTest() {
    console.log('🔬 Quick Robot Control Test');
    const manager = new ROS2ScriptManager();
    
    try {
        console.log('🚀 Starting robot control...');
        const result = await manager.startRobotControl();
        console.log('📊 Result:', JSON.stringify(result, null, 2));
    } catch (error) {
        console.error('❌ Error:', error.message);
    } finally {
        console.log('🧹 Cleanup...');
        await manager.cleanup();
        console.log('✅ Done');
        process.exit(0);
    }
}

quickTest();
