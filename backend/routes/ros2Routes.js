// routes/ros2Routes.js - Express routes for ROS2 control
const express = require('express');
const router = express.Router();
const ROS2ScriptManager = require('../ros/utils/ros2ScriptManager');
const MapConverter = require('../ros/utils/mapConverter');
const path = require('path');

// Initialize managers
const ros2Manager = new ROS2ScriptManager();
const mapConverter = new MapConverter();

// Start SLAM
router.post('/slam/start', async (req, res) => {
    try {
        const { mapName } = req.body;
        const result = await ros2Manager.startSLAM(mapName);
        
        res.json({
            success: true,
            data: result,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

// Start Navigation
router.post('/navigation/start', async (req, res) => {
    try {
        const { mapPath, mapName } = req.body;
        
        let actualMapPath = mapPath;
        
        // If mapName is provided instead of full path, construct path
        if (!mapPath && mapName) {
            actualMapPath = path.join(__dirname, '../maps', `${mapName}.yaml`);
        }
        
        if (!actualMapPath) {
            throw new Error('Either mapPath or mapName must be provided');
        }
        
        const result = await ros2Manager.startNavigation(actualMapPath);
        
        res.json({
            success: true,
            data: result,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

// Stop ROS2 process
router.post('/stop/:processName', async (req, res) => {
    try {
        const { processName } = req.params;
        const validProcesses = ['slam', 'navigation', 'robot_control'];
        
        if (!validProcesses.includes(processName)) {
            throw new Error(`Invalid process name. Valid options: ${validProcesses.join(', ')}`);
        }
        
        const result = await ros2Manager.stopProcess(processName);
        
        res.json({
            success: true,
            data: result,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

// Stop all ROS2 processes
router.post('/stop-all', async (req, res) => {
    try {
        const results = await ros2Manager.stopAll();
        
        res.json({
            success: true,
            data: results,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

// Restart ROS2 process
router.post('/restart/:processName', async (req, res) => {
    try {
        const { processName } = req.params;
        const { mapPath, mapName } = req.body;
        
        if (processName === 'navigation' && !mapPath && !mapName) {
            throw new Error('Map path or name required for navigation restart');
        }
        
        // For navigation restart, we need to stop and start with map
        if (processName === 'navigation') {
            await ros2Manager.stopProcess('navigation');
            await new Promise(resolve => setTimeout(resolve, 2000));
            
            let actualMapPath = mapPath;
            if (!mapPath && mapName) {
                actualMapPath = path.join(__dirname, '../maps', `${mapName}.yaml`);
            }
            
            const result = await ros2Manager.startNavigation(actualMapPath);
            res.json({
                success: true,
                data: result,
                timestamp: new Date().toISOString()
            });
        } else {
            const result = await ros2Manager.restart(processName);
            res.json({
                success: true,
                data: result,
                timestamp: new Date().toISOString()
            });
        }
        
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

// Get ROS2 status
router.get('/status', async (req, res) => {
    try {
        const status = ros2Manager.getStatus();
        
        res.json({
            success: true,
            data: status,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

// Convert map from JSON to PGM+YAML
router.post('/map/convert', async (req, res) => {
    try {
        const { mapData, mapName, outputDir } = req.body;
        
        if (!mapData || !mapName) {
            throw new Error('mapData and mapName are required');
        }
        
        const result = await mapConverter.convertJsonToPgmYaml(
            mapData, 
            mapName, 
            outputDir
        );
        
        res.json({
            success: true,
            data: result,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

// List available maps
router.get('/maps', async (req, res) => {
    try {
        const maps = await mapConverter.listAvailableMaps();
        
        res.json({
            success: true,
            data: maps,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

// Get map info
router.get('/maps/:mapName', async (req, res) => {
    try {
        const { mapName } = req.params;
        const yamlPath = path.join(__dirname, '../maps', `${mapName}.yaml`);
        const pgmPath = path.join(__dirname, '../maps', `${mapName}.pgm`);
        
        const mapInfo = await mapConverter.getMapInfo(yamlPath);
        const validation = await mapConverter.validateMapFiles(pgmPath, yamlPath);
        
        res.json({
            success: true,
            data: {
                name: mapName,
                info: mapInfo,
                validation: validation,
                yamlPath: yamlPath,
                pgmPath: pgmPath
            },
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

// Delete map
router.delete('/maps/:mapName', async (req, res) => {
    try {
        const { mapName } = req.params;
        const result = await mapConverter.deleteMap(mapName);
        
        res.json({
            success: true,
            data: result,
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

// Upload and convert map
router.post('/maps/upload', async (req, res) => {
    try {
        const { mapData, mapName } = req.body;
        
        if (!mapData || !mapName) {
            throw new Error('mapData and mapName are required');
        }
        
        // Convert to PGM+YAML
        const conversionResult = await mapConverter.convertJsonToPgmYaml(mapData, mapName);
        
        // Validate the converted files
        const validation = await mapConverter.validateMapFiles(
            conversionResult.pgmPath, 
            conversionResult.yamlPath
        );
        
        res.json({
            success: true,
            data: {
                conversion: conversionResult,
                validation: validation,
                mapName: mapName
            },
            timestamp: new Date().toISOString()
        });
        
    } catch (error) {
        res.status(500).json({
            success: false,
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

module.exports = router;