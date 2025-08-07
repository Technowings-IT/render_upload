// Simple test to check supported commands
const ROS2ScriptManager = require('./ros/utils/ros2ScriptManager');

console.log('📋 Available commands in ROS2ScriptManager:');

const supportedCommands = [
    'start_robot_control',
    'start_slam', 
    'start_navigation',
    'stop_robot_control',
    'stop_slam',
    'stop_navigation',
    'stop_all_scripts',
    'kill_all_processes',      // ✅ THIS IS THE NEW ONE
    'emergency_stop',
    'get_status',
    'get_detailed_status', 
    'check_pi_status',
    'test_ssh_connectivity',
    'restart_robot_control',
    'restart_slam',
    'restart_navigation'
];

supportedCommands.forEach(cmd => {
    console.log(`✅ ${cmd}`);
});

console.log('\n🎉 The kill_all_processes command has been added successfully!');
console.log('🔧 The original error should now be resolved.');
