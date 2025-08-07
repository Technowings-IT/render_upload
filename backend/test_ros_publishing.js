#!/usr/bin/env node
// test_ros_publishing.js - Test script to verify ROS2 publishing
const rosConnection = require('./ros/utils/ros_connection');
const publishers = require('./ros/utils/publishers');

async function testROS2Publishing() {
    console.log('🔍 Starting ROS2 Publishing Test...\n');
    
    try {
        // Initialize ROS2
        console.log('1. Initializing ROS2...');
        await rosConnection.initializeROS();
        console.log('✅ ROS2 initialized successfully\n');
        
        // Wait a moment for everything to stabilize
        await new Promise(resolve => setTimeout(resolve, 2000));
        
        // Test 1: Connectivity Test
        console.log('2. Testing ROS2 connectivity...');
        const connectivityTest = await rosConnection.testConnectivity();
        console.log('📊 Connectivity Results:');
        console.log(`   - Total topics: ${connectivityTest.totalTopics}`);
        console.log(`   - Required topics found: ${connectivityTest.requiredTopicsFound?.join(', ') || 'none'}`);
        console.log(`   - Missing required: ${connectivityTest.missingRequired?.join(', ') || 'none'}`);
        console.log(`   - Can publish velocity: ${connectivityTest.canPublishVelocity}\n`);
        
        // Test 2: Publisher Statistics
        console.log('3. Checking publisher statistics...');
        const publisherStats = publishers.getPublisherStats();
        console.log('📤 Publisher Stats:');
        console.log(`   - Total publishers: ${publisherStats.totalPublishers}`);
        console.log(`   - Initialized: ${publisherStats.isInitialized}`);
        if (publisherStats.totalPublishers > 0) {
            Object.entries(publisherStats.publishers).forEach(([topic, info]) => {
                console.log(`   - ${topic}: ${info.messageCount} messages sent`);
            });
        }
        console.log('');
        
        // Test 3: Publishing Test
        console.log('4. Testing publishing capability...');
        const publishTest = publishers.testPublishing();
        if (publishTest.success) {
            console.log('✅ Publishing test successful');
            console.log(`   - Available topics: ${publishTest.tests.availableTopics?.join(', ') || 'none'}`);
        } else {
            console.log('❌ Publishing test failed:', publishTest.error);
        }
        console.log('');
        
        // Test 4: Velocity Publishing Test
        console.log('5. Testing velocity publishing...');
        console.log('   Sending stop command (0, 0)...');
        const stopResult = publishers.publishVelocity(0.0, 0.0);
        if (stopResult.success) {
            console.log('✅ Stop command published successfully');
        } else {
            console.log('❌ Stop command failed:', stopResult.error);
        }
        
        await new Promise(resolve => setTimeout(resolve, 500));
        
        console.log('   Sending small forward command (0.1, 0)...');
        const forwardResult = publishers.publishVelocity(0.1, 0.0);
        if (forwardResult.success) {
            console.log('✅ Forward command published successfully');
        } else {
            console.log('❌ Forward command failed:', forwardResult.error);
        }
        
        await new Promise(resolve => setTimeout(resolve, 500));
        
        console.log('   Sending stop command again...');
        const stopResult2 = publishers.publishVelocity(0.0, 0.0);
        if (stopResult2.success) {
            console.log('✅ Final stop command published successfully');
        } else {
            console.log('❌ Final stop command failed:', stopResult2.error);
        }
        console.log('');
        
        // Test 5: Joystick Publishing Test
        console.log('6. Testing joystick publishing...');
        console.log('   Testing joystick with deadman=false (should stop)...');
        const joystickStop = publishers.publishJoystick(0.5, 0.5, false);
        if (joystickStop.success) {
            console.log('✅ Joystick deadman test successful (robot stopped)');
        } else {
            console.log('❌ Joystick deadman test failed:', joystickStop.error);
        }
        
        await new Promise(resolve => setTimeout(resolve, 500));
        
        console.log('   Testing joystick with deadman=true (should move)...');
        const joystickMove = publishers.publishJoystick(0.2, 0.1, true);
        if (joystickMove.success) {
            console.log('✅ Joystick movement test successful');
            console.log(`   - Linear: ${joystickMove.data.linear}, Angular: ${joystickMove.data.angular}`);
        } else {
            console.log('❌ Joystick movement test failed:', joystickMove.error);
        }
        
        await new Promise(resolve => setTimeout(resolve, 500));
        
        console.log('   Final stop via joystick...');
        const joystickFinalStop = publishers.publishJoystick(0.0, 0.0, true);
        if (joystickFinalStop.success) {
            console.log('✅ Joystick final stop successful');
        }
        console.log('');
        
        // Test 6: Check if topics are visible in ROS2
        console.log('7. Checking ROS2 topic visibility...');
        const rosStatus = rosConnection.getROS2Status();
        if (rosStatus.availableTopics && rosStatus.availableTopics.length > 0) {
            console.log('📋 Available ROS2 topics:');
            rosStatus.availableTopics.forEach(topic => {
                if (topic.includes('cmd_vel') || topic.includes('map') || topic.includes('pos') || topic.includes('odom')) {
                    console.log(`   ✅ ${topic} (essential topic)`);
                } else {
                    console.log(`   📍 ${topic}`);
                }
            });
        } else {
            console.log('⚠️ No topics discovered or ROS2 node not active');
        }
        console.log('');
        
        // Final summary
        console.log('📋 TEST SUMMARY:');
        console.log('================');
        console.log(`✅ ROS2 Initialized: ${rosStatus.initialized}`);
        console.log(`✅ Node Active: ${rosStatus.nodeActive}`);
        console.log(`✅ Topics Discovered: ${rosStatus.topicsDiscovered}`);
        console.log(`✅ Publishers Created: ${publisherStats.totalPublishers}`);
        console.log(`✅ Publishing Working: ${publishTest.success}`);
        console.log(`✅ Velocity Control: ${stopResult.success && forwardResult.success}`);
        console.log(`✅ Joystick Control: ${joystickMove.success}`);
        
        if (connectivityTest.missingRequired && connectivityTest.missingRequired.length > 0) {
            console.log(`⚠️ Missing Topics: ${connectivityTest.missingRequired.join(', ')}`);
        }
        
        console.log('\n🎉 ROS2 Publishing Test Complete!');
        console.log('\nYou should now be able to see the backend publishing to /cmd_vel in rqt_graph');
        console.log('Run: rqt_graph to verify the connection graph');
        
    } catch (error) {
        console.error('❌ Test failed:', error);
        console.error('\nTroubleshooting:');
        console.error('1. Ensure ROS2 Jazzy is properly sourced');
        console.error('2. Check if your AMR is running and topics are available');
        console.error('3. Verify network connectivity to AMR');
        console.error('4. Run "ros2 topic list" to see available topics');
    } finally {
        // Cleanup
        setTimeout(async () => {
            try {
                await rosConnection.shutdown();
                console.log('\n🛑 ROS2 connection closed');
                process.exit(0);
            } catch (shutdownError) {
                console.error('❌ Shutdown error:', shutdownError);
                process.exit(1);
            }
        }, 1000);
    }
}

// Command line argument handling
if (process.argv.includes('--help') || process.argv.includes('-h')) {
    console.log('ROS2 Publishing Test Script');
    console.log('Usage: node test_ros_publishing.js [options]');
    console.log('');
    console.log('This script tests ROS2 publishing functionality for the AMR Fleet Backend');
    console.log('It will:');
    console.log('1. Initialize ROS2 connection');
    console.log('2. Test topic discovery');
    console.log('3. Test velocity publishing');
    console.log('4. Test joystick control');
    console.log('5. Verify publishers are visible in rqt_graph');
    console.log('');
    console.log('Make sure your AMR is running and ROS2 is properly sourced before running this test.');
    process.exit(0);
}

// Run the test
testROS2Publishing();