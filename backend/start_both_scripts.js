#!/usr/bin/env node
// Script to start both SLAM and Navigation on Raspberry Pi

const ROS2ScriptManager = require('./ros/utils/ros2ScriptManager');

async function startBothScripts() {
    console.log('🚀 Starting SLAM and Navigation on Raspberry Pi');
    console.log('=' .repeat(60));
    
    const manager = new ROS2ScriptManager();
    
    try {
        // Step 1: Start Robot Control first
        console.log('\n🤖 Step 1: Starting Robot Control...');
        const robotControlResult = await manager.startRobotControl();
        console.log('✅ Robot Control Result:', JSON.stringify(robotControlResult, null, 2));
        
        // Wait for robot control to initialize
        console.log('\n⏳ Waiting 3 seconds for Robot Control to initialize...');
        await new Promise(resolve => setTimeout(resolve, 3000));
        
        // Step 2: Start SLAM
        console.log('\n🗺️ Step 2: Starting SLAM...');
        const slamResult = await manager.startSLAM({ 
            mapName: 'new_slam_map_' + Date.now() 
        });
        console.log('✅ SLAM Result:', JSON.stringify(slamResult, null, 2));
        
        // Wait a bit for SLAM to start
        console.log('\n⏳ Waiting 3 seconds for SLAM to initialize...');
        await new Promise(resolve => setTimeout(resolve, 3000));
        
        // Step 3: Start Navigation (if map exists)
        console.log('\n🚀 Step 3: Starting Navigation...');
        try {
            const navResult = await manager.startNavigation('your_map_name.yaml');
            console.log('✅ Navigation Result:', JSON.stringify(navResult, null, 2));
        } catch (navError) {
            console.log('⚠️ Navigation failed (this is expected if no map exists):', navError.message);
        }
        
        // Step 4: Check final status
        console.log('\n📊 Final Status Check...');
        const finalStatus = manager.getDetailedStatus();
        console.log('📈 System Status:', JSON.stringify(finalStatus, null, 2));
        
        console.log('\n✅ Script startup sequence completed!');
        
    } catch (error) {
        console.error('❌ Error during startup:', error.message);
        console.error('📋 Stack:', error.stack);
    } finally {
        console.log('\n🧹 Cleaning up...');
        await manager.cleanup();
        console.log('✅ Done');
        process.exit(0);
    }
}

// Handle process termination
process.on('SIGINT', () => {
    console.log('\n🛑 Script interrupted by user');
    process.exit(0);
});

// Run the script
startBothScripts().catch(error => {
    console.error('💥 Fatal error:', error);
    process.exit(1);
});
