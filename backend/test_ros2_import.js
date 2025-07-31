const ROS2ScriptManager = require('./ros/utils/ros2ScriptManager');

console.log('Type of ROS2ScriptManager:', typeof ROS2ScriptManager);
console.log('ROS2ScriptManager constructor:', ROS2ScriptManager.constructor);
console.log('ROS2ScriptManager prototype:', ROS2ScriptManager.prototype);

if (typeof ROS2ScriptManager === 'function') {
    console.log('✅ ROS2ScriptManager is a function (constructor)');
    try {
        const instance = new ROS2ScriptManager();
        console.log('✅ Successfully created instance');
    } catch (error) {
        console.log('❌ Error creating instance:', error.message);
    }
} else {
    console.log('❌ ROS2ScriptManager is not a function');
}
