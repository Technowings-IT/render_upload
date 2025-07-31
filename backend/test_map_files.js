const express = require('express');
const path = require('path');
const fs = require('fs').promises;

// Test the map deployment logic
async function testMapDeployment() {
    console.log('🧪 Testing map deployment...');
    
    const mapName = 'piros_deploy_map'; // Known existing map
    const yamlPath = path.join(__dirname, 'maps', `${mapName}.yaml`);
    const pgmPath = path.join(__dirname, 'maps', `${mapName}.pgm`);
    
    console.log('Checking files:');
    console.log('  YAML:', yamlPath);
    console.log('  PGM:', pgmPath);
    
    try {
        await fs.access(yamlPath);
        console.log('✅ YAML file exists');
    } catch (error) {
        console.log('❌ YAML file not found:', error.message);
    }
    
    try {
        await fs.access(pgmPath);
        console.log('✅ PGM file exists');
    } catch (error) {
        console.log('❌ PGM file not found:', error.message);
    }
    
    // List all available maps
    console.log('\n📋 Available maps:');
    try {
        const files = await fs.readdir(path.join(__dirname, 'maps'));
        const mapNames = new Set();
        files.forEach(file => {
            if (file.endsWith('.yaml') || file.endsWith('.pgm')) {
                const mapName = file.replace(/\.(yaml|pgm)$/, '');
                mapNames.add(mapName);
            }
        });
        
        mapNames.forEach(name => {
            console.log(`  - ${name}`);
        });
    } catch (error) {
        console.log('❌ Error listing maps:', error.message);
    }
}

testMapDeployment();
