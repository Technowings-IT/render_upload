#!/usr/bin/env node
// Direct script execution on Pi

const { Client } = require('ssh2');

const piConfig = {
    host: '192.168.0.64',
    username: 'piros',
    password: 'piros',
    port: 22
};

async function executeScriptDirect(scriptName, args = '') {
    return new Promise((resolve, reject) => {
        const conn = new Client();
        
        conn.on('ready', () => {
            console.log(`✅ SSH connected for ${scriptName}`);
            
            // Execute script in background with nohup
            const command = `cd /home/piros/scripts && nohup bash ${scriptName} ${args} > /tmp/${scriptName.replace('.sh', '')}.log 2>&1 & echo $!`;
            console.log(`📤 Executing: ${command}`);
            
            conn.exec(command, (err, stream) => {
                if (err) {
                    conn.end();
                    return reject(err);
                }
                
                let output = '';
                
                stream.on('data', (data) => {
                    output += data.toString();
                    console.log(`📥 ${scriptName} output:`, data.toString().trim());
                });
                
                stream.on('close', (code) => {
                    conn.end();
                    console.log(`🏁 ${scriptName} finished with code ${code}, PID: ${output.trim()}`);
                    resolve({ script: scriptName, code, pid: output.trim() });
                });
            });
        });
        
        conn.on('error', (err) => {
            console.error(`❌ SSH error for ${scriptName}:`, err.message);
            reject(err);
        });
        
        conn.connect(piConfig);
    });
}

async function startAllScripts() {
    console.log('🚀 Starting All Scripts on Raspberry Pi');
    console.log('=' .repeat(50));
    
    try {
        // Step 1: Start robot_control.sh
        console.log('\n🤖 Starting robot_control.sh...');
        const robotResult = await executeScriptDirect('robot_control.sh');
        console.log('✅ Robot Control started:', robotResult);
        
        // Wait a moment
        await new Promise(resolve => setTimeout(resolve, 2000));
        
        // Step 2: Start slam.sh
        console.log('\n🗺️ Starting slam.sh...');
        const slAMResult = await executeScriptDirect('slam.sh', 'new_slam_map_' + Date.now());
        console.log('✅ SLAM started:', slAMResult);
        
        // Wait a moment
        await new Promise(resolve => setTimeout(resolve, 2000));
        
        // Step 3: Start nav2.sh (if desired)
        console.log('\n🚀 Starting nav2.sh...');
        try {
            const navResult = await executeScriptDirect('nav2.sh', '/home/piros/fleet-management-system/ros_ws/src/AMR/maps/map_1750065869.yaml params_file:=/home/piros/fleet-management-system/ros_ws/src/AMR/config/nav2_params1.yaml use_sim_time:=False');
            console.log('✅ Navigation started:', navResult);
        } catch (navError) {
            console.log('⚠️ Navigation failed:', navError.message);
        }
        
        console.log('\n✅ All scripts started successfully!');
        
        // Check running processes
        console.log('\n📊 Checking running processes...');
        await checkRunningProcesses();
        
    } catch (error) {
        console.error('❌ Error:', error.message);
    }
}

async function checkRunningProcesses() {
    return new Promise((resolve, reject) => {
        const conn = new Client();
        
        conn.on('ready', () => {
            conn.exec('ps aux | grep -E "(ros2|slam|nav2)" | grep -v grep', (err, stream) => {
                if (err) {
                    conn.end();
                    return reject(err);
                }
                
                let output = '';
                
                stream.on('data', (data) => {
                    output += data.toString();
                });
                
                stream.on('close', (code) => {
                    conn.end();
                    console.log('📋 Running ROS2 processes:');
                    console.log(output || 'No ROS2 processes found');
                    resolve(output);
                });
            });
        });
        
        conn.on('error', reject);
        conn.connect(piConfig);
    });
}

startAllScripts().catch(console.error);
