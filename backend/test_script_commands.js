#!/usr/bin/env node

// Test script for ROS2ScriptManager commands
const ROS2ScriptManager = require('./ros/utils/ros2ScriptManager');

async function testScriptCommands() {
    console.log('🧪 Testing ROS2ScriptManager commands...');
    
    const scriptManager = new ROS2ScriptManager();
    
    try {
        // Test available commands
        const commandsToTest = [
            'get_status',
            'get_detailed_status',
            'check_pi_status',
            'kill_all_processes'
        ];
        
        for (const command of commandsToTest) {
            try {
                console.log(`\n🔧 Testing command: ${command}`);
                const result = await scriptManager.handleScriptCommand(command);
                console.log(`✅ Command '${command}' successful:`, result);
            } catch (error) {
                console.log(`❌ Command '${command}' failed:`, error.message);
            }
        }
        
        console.log('\n🎉 All tests completed!');
        
    } catch (error) {
        console.error('❌ Test failed:', error);
    }
    
    // Clean up
    await scriptManager.cleanup();
}

// Run tests if called directly
if (require.main === module) {
    testScriptCommands().catch(console.error);
}

module.exports = { testScriptCommands };
