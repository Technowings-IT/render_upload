#!/usr/bin/env node

console.log('Starting test...');

try {
    console.log('About to require ROS2ScriptManager...');
    const ROS2ScriptManager = require('./ros/utils/ros2ScriptManager');
    console.log('✅ Required successfully. Type:', typeof ROS2ScriptManager);
    console.log('✅ Is function:', typeof ROS2ScriptManager === 'function');
    
    if (typeof ROS2ScriptManager === 'function') {
        console.log('About to construct instance...');
        const manager = new ROS2ScriptManager();
        console.log('✅ Construction successful!');
        console.log('Manager type:', typeof manager);
        console.log('Manager constructor name:', manager.constructor.name);
    } else {
        console.error('❌ ROS2ScriptManager is not a constructor function!');
        console.log('ROS2ScriptManager:', ROS2ScriptManager);
    }
} catch (error) {
    console.error('❌ Error occurred:');
    console.error('Message:', error.message);
    console.error('Stack:', error.stack);
}

console.log('Test completed.');
