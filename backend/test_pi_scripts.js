#!/usr/bin/env node
// Test Pi script existence and execution

const { Client } = require('ssh2');

const piConfig = {
    host: '192.168.0.84',
    username: 'piros',
    password: 'piros',
    port: 22
};

async function testPiScripts() {
    console.log('🔍 Testing Pi Script Existence and Execution');
    console.log('=' .repeat(50));
    
    return new Promise((resolve, reject) => {
        const conn = new Client();
        
        conn.on('ready', () => {
            console.log('✅ SSH connection established');
            
            // Test 1: Check if scripts directory exists
            conn.exec('ls -la /home/piros/scripts/', (err, stream) => {
                if (err) {
                    console.error('❌ Failed to list scripts directory:', err.message);
                    conn.end();
                    return reject(err);
                }
                
                let output = '';
                stream.on('data', (data) => {
                    output += data.toString();
                });
                
                stream.on('close', (code) => {
                    console.log('📂 Scripts directory listing:');
                    console.log(output);
                    
                    // Test 2: Check robot_control.sh specifically
                    conn.exec('ls -la /home/piros/scripts/robot_control.sh 2>&1', (err2, stream2) => {
                        if (err2) {
                            console.error('❌ Failed to check robot_control.sh:', err2.message);
                            conn.end();
                            return reject(err2);
                        }
                        
                        let output2 = '';
                        stream2.on('data', (data) => {
                            output2 += data.toString();
                        });
                        
                        stream2.on('close', (code2) => {
                            console.log('🤖 robot_control.sh check:');
                            console.log(output2);
                            
                            // Test 3: Try to read the script content
                            conn.exec('head -20 /home/piros/scripts/robot_control.sh 2>&1', (err3, stream3) => {
                                if (err3) {
                                    console.error('❌ Failed to read script:', err3.message);
                                    conn.end();
                                    return reject(err3);
                                }
                                
                                let output3 = '';
                                stream3.on('data', (data) => {
                                    output3 += data.toString();
                                });
                                
                                stream3.on('close', (code3) => {
                                    console.log('📄 Script content (first 20 lines):');
                                    console.log(output3);
                                    
                                    // Test 4: Test script execution (with timeout)
                                    console.log('🚀 Testing script execution...');
                                    conn.exec('cd /home/piros/scripts && timeout 5s bash robot_control.sh', (err4, stream4) => {
                                        if (err4) {
                                            console.error('❌ Failed to execute script:', err4.message);
                                            conn.end();
                                            return reject(err4);
                                        }
                                        
                                        let stdout = '';
                                        let stderr = '';
                                        
                                        stream4.on('data', (data) => {
                                            stdout += data.toString();
                                            console.log('📤 Script output:', data.toString().trim());
                                        });
                                        
                                        stream4.stderr.on('data', (data) => {
                                            stderr += data.toString();
                                            console.log('❌ Script error:', data.toString().trim());
                                        });
                                        
                                        stream4.on('close', (code4) => {
                                            console.log(`🏁 Script execution finished with code: ${code4}`);
                                            console.log('📋 Final stdout:', stdout);
                                            console.log('📋 Final stderr:', stderr);
                                            
                                            conn.end();
                                            resolve({
                                                scriptsDir: output,
                                                scriptFile: output2,
                                                scriptContent: output3,
                                                execution: { code: code4, stdout, stderr }
                                            });
                                        });
                                    });
                                });
                            });
                        });
                    });
                });
            });
        });
        
        conn.on('error', (err) => {
            console.error('❌ SSH connection failed:', err.message);
            reject(err);
        });
        
        console.log('🔌 Connecting to Pi...');
        conn.connect(piConfig);
    });
}

testPiScripts()
    .then((result) => {
        console.log('\n✅ Test completed successfully');
        process.exit(0);
    })
    .catch((error) => {
        console.error('\n❌ Test failed:', error.message);
        process.exit(1);
    });
