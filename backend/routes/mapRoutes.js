// routes/mapRoutes.js - New API routes for PGM conversion and map management
const express = require('express');
const path = require('path');
const fs = require('fs').promises;
const pgmConverter = require('../ros/utils/pgmConverter');
const rosConnection = require('../ros/utils/ros_connection');
const router = express.Router();

// ==========================================
// PGM EXPORT & CONVERSION ROUTES
// ==========================================

/**
 * Export current map data to PGM format
 * POST /api/maps/:deviceId/export/pgm
 */
router.post('/maps/:deviceId/export/pgm', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const {
            mapName,
            includeGlobalCostmap = true,
            includeLocalCostmap = true,
            includeStaticMap = true
        } = req.body;

        console.log(`🗺️ Exporting map to PGM for device: ${deviceId}`);

        // Get live data for the device
        const liveData = rosConnection.getLiveData(deviceId);
        if (!liveData) {
            return res.status(404).json({
                success: false,
                error: 'No live data available for device'
            });
        }

        const exportedFiles = [];
        const timestamp = Date.now();
        const baseMapName = mapName || `map_${deviceId}_${timestamp}`;

        // Export static map
        if (includeStaticMap && liveData.map) {
            try {
                const staticMapPath = path.join(__dirname, '../maps', `${baseMapName}_static`);
                const files = await pgmConverter.saveMapPackage(liveData.map, staticMapPath);
                exportedFiles.push({
                    type: 'static_map',
                    files: files
                });
                console.log(`✅ Static map exported: ${files.pgm}`);
            } catch (error) {
                console.error('❌ Error exporting static map:', error);
            }
        }

        // Export global costmap
        if (includeGlobalCostmap && liveData.global_costmap) {
            try {
                const globalCostmapPath = path.join(__dirname, '../maps', `${baseMapName}_global_costmap`);
                const files = await pgmConverter.saveMapPackage(liveData.global_costmap, globalCostmapPath);
                exportedFiles.push({
                    type: 'global_costmap',
                    files: files
                });
                console.log(`✅ Global costmap exported: ${files.pgm}`);
            } catch (error) {
                console.error('❌ Error exporting global costmap:', error);
            }
        }

        // Export local costmap
        if (includeLocalCostmap && liveData.local_costmap) {
            try {
                const localCostmapPath = path.join(__dirname, '../maps', `${baseMapName}_local_costmap`);
                const files = await pgmConverter.saveMapPackage(liveData.local_costmap, localCostmapPath);
                exportedFiles.push({
                    type: 'local_costmap',
                    files: files
                });
                console.log(`✅ Local costmap exported: ${files.pgm}`);
            } catch (error) {
                console.error('❌ Error exporting local costmap:', error);
            }
        }

        if (exportedFiles.length === 0) {
            return res.status(400).json({
                success: false,
                error: 'No valid map data available for export'
            });
        }

        res.json({
            success: true,
            message: 'Map exported to PGM successfully',
            deviceId: deviceId,
            mapName: baseMapName,
            files: exportedFiles,
            exportedAt: new Date().toISOString()
        });

    } catch (error) {
        console.error('❌ Error in PGM export:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Upload map to AGV (send PGM + YAML back to ROS)
 * POST /api/maps/:deviceId/upload
 */
router.post('/maps/:deviceId/upload', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, setAsActiveMap = true } = req.body;

        console.log(`🚀 Uploading map to AGV: ${mapName} for device: ${deviceId}`);

        // Find the map files
        const mapsDir = path.join(__dirname, '../maps');
        const yamlPath = path.join(mapsDir, `${mapName}.yaml`);
        const pgmPath = path.join(mapsDir, `${mapName}.pgm`);

        // Check if files exist
        try {
            await fs.access(yamlPath);
            await fs.access(pgmPath);
        } catch (error) {
            return res.status(404).json({
                success: false,
                error: `Map files not found: ${mapName}.yaml or ${mapName}.pgm`
            });
        }

        // Load map package
        const mapData = await pgmConverter.loadMapPackage(yamlPath);
        
        // Publish to ROS map_server or navigation stack
        let uploadResult;
        
        try {
            // Method 1: Try to use ROS map_server service
            uploadResult = await publishMapToROS(deviceId, mapData, mapName);
        } catch (rosError) {
            console.warn('⚠️ ROS map service unavailable, using file-based method');
            
            // Method 2: Copy files to ROS map directory
            uploadResult = await copyMapToROSDirectory(deviceId, yamlPath, pgmPath, mapName);
        }

        // If setAsActiveMap is true, make this the active map
        if (setAsActiveMap && uploadResult.success) {
            try {
                await setActiveMapOnAGV(deviceId, mapName);
                console.log(`✅ Map set as active: ${mapName}`);
            } catch (error) {
                console.warn(`⚠️ Could not set as active map: ${error.message}`);
            }
        }

        res.json({
            success: uploadResult.success,
            message: uploadResult.success ? 'Map uploaded to AGV successfully' : 'Upload failed',
            deviceId: deviceId,
            mapName: mapName,
            method: uploadResult.method,
            uploadedAt: new Date().toISOString(),
            details: uploadResult.details
        });

    } catch (error) {
        console.error('❌ Error uploading map to AGV:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Get available maps for a device
 * GET /api/maps/:deviceId/available
 */
router.get('/maps/:deviceId/available', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const mapsDir = path.join(__dirname, '../maps');

        // Read all files in maps directory
        const files = await fs.readdir(mapsDir);
        
        // Group files by map name
        const mapGroups = {};
        
        files.forEach(file => {
            const ext = path.extname(file);
            const baseName = path.basename(file, ext);
            
            if (ext === '.yaml' || ext === '.pgm') {
                if (!mapGroups[baseName]) {
                    mapGroups[baseName] = { yaml: false, pgm: false, files: [] };
                }
                mapGroups[baseName][ext.substring(1)] = true;
                mapGroups[baseName].files.push(file);
            }
        });

        // Convert to array with additional info
        const availableMaps = [];
        
        for (const [mapName, info] of Object.entries(mapGroups)) {
            if (info.yaml && info.pgm) {
                try {
                    // Get file stats
                    const yamlPath = path.join(mapsDir, `${mapName}.yaml`);
                    const pgmPath = path.join(mapsDir, `${mapName}.pgm`);
                    
                    const yamlStats = await fs.stat(yamlPath);
                    const pgmStats = await fs.stat(pgmPath);
                    
                    // Try to read map metadata
                    let metadata = null;
                    try {
                        const yamlContent = await fs.readFile(yamlPath, 'utf8');
                        metadata = pgmConverter.parseMapYAML(yamlContent);
                    } catch (error) {
                        console.warn(`⚠️ Could not parse metadata for ${mapName}`);
                    }

                    availableMaps.push({
                        name: mapName,
                        files: info.files,
                        size: {
                            yaml: yamlStats.size,
                            pgm: pgmStats.size,
                            total: yamlStats.size + pgmStats.size
                        },
                        createdAt: yamlStats.birthtime.toISOString(),
                        modifiedAt: yamlStats.mtime.toISOString(),
                        metadata: metadata
                    });
                } catch (error) {
                    console.warn(`⚠️ Error getting stats for ${mapName}:`, error.message);
                }
            }
        }

        // Sort by modification time (newest first)
        availableMaps.sort((a, b) => new Date(b.modifiedAt) - new Date(a.modifiedAt));

        res.json({
            success: true,
            deviceId: deviceId,
            maps: availableMaps,
            totalMaps: availableMaps.length
        });

    } catch (error) {
        console.error('❌ Error getting available maps:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Convert map between different formats
 * POST /api/maps/:deviceId/convert
 */
router.post('/maps/:deviceId/convert', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { sourceFormat, targetFormat, mapName } = req.body;

        console.log(`🔄 Converting map from ${sourceFormat} to ${targetFormat}`);

        const supportedFormats = ['occupancy_grid', 'pgm', 'yaml', 'json', 'png'];
        
        if (!supportedFormats.includes(sourceFormat) || !supportedFormats.includes(targetFormat)) {
            return res.status(400).json({
                success: false,
                error: `Unsupported format. Supported: ${supportedFormats.join(', ')}`
            });
        }

        let conversionResult;

        // Handle different conversion scenarios
        if (sourceFormat === 'occupancy_grid' && targetFormat === 'pgm') {
            // Convert live occupancy grid to PGM
            const liveData = rosConnection.getLiveData(deviceId);
            if (!liveData || !liveData.map) {
                return res.status(404).json({
                    success: false,
                    error: 'No live map data available'
                });
            }

            const outputPath = path.join(__dirname, '../maps', `${mapName || 'converted_map'}`);
            conversionResult = await pgmConverter.saveMapPackage(liveData.map, outputPath);

        } else if (sourceFormat === 'pgm' && targetFormat === 'json') {
            // Convert PGM to JSON
            const pgmPath = path.join(__dirname, '../maps', `${mapName}.pgm`);
            const pgmData = await pgmConverter.readPGM(pgmPath);
            
            const jsonPath = path.join(__dirname, '../maps', `${mapName}.json`);
            await fs.writeFile(jsonPath, JSON.stringify(pgmData, null, 2));
            
            conversionResult = { json: jsonPath };

        } else if (sourceFormat === 'yaml' && targetFormat === 'json') {
            // Convert YAML metadata to JSON
            const yamlPath = path.join(__dirname, '../maps', `${mapName}.yaml`);
            const yamlContent = await fs.readFile(yamlPath, 'utf8');
            const metadata = pgmConverter.parseMapYAML(yamlContent);
            
            const jsonPath = path.join(__dirname, '../maps', `${mapName}_metadata.json`);
            await fs.writeFile(jsonPath, JSON.stringify(metadata, null, 2));
            
            conversionResult = { json: jsonPath };

        } else {
            return res.status(400).json({
                success: false,
                error: `Conversion from ${sourceFormat} to ${targetFormat} not implemented`
            });
        }

        res.json({
            success: true,
            message: `Map converted from ${sourceFormat} to ${targetFormat}`,
            deviceId: deviceId,
            sourceFormat: sourceFormat,
            targetFormat: targetFormat,
            mapName: mapName,
            outputFiles: conversionResult,
            convertedAt: new Date().toISOString()
        });

    } catch (error) {
        console.error('❌ Error converting map format:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Process map with image operations
 * POST /api/maps/:deviceId/process
 */
router.post('/maps/:deviceId/process', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, operation, operationParams = {} } = req.body;

        console.log(`🔧 Processing map ${mapName} with operation: ${operation}`);

        // Load the map
        const yamlPath = path.join(__dirname, '../maps', `${mapName}.yaml`);
        const mapData = await pgmConverter.loadMapPackage(yamlPath);

        let processedMapData;

        switch (operation) {
            case 'erode':
            case 'dilate':
            case 'open':
            case 'close':
                const kernelSize = operationParams.kernelSize || 3;
                processedMapData = pgmConverter.morphologicalOperation(mapData, operation, kernelSize);
                break;

            case 'resize':
                const newWidth = operationParams.width || mapData.info.width;
                const newHeight = operationParams.height || mapData.info.height;
                processedMapData = pgmConverter.resizeMap(mapData, newWidth, newHeight);
                break;

            case 'blur':
                // Custom blur implementation would go here
                processedMapData = mapData; // Placeholder
                break;

            default:
                return res.status(400).json({
                    success: false,
                    error: `Unsupported operation: ${operation}`
                });
        }

        // Save processed map
        const outputName = `${mapName}_${operation}`;
        const outputPath = path.join(__dirname, '../maps', outputName);
        const savedFiles = await pgmConverter.saveMapPackage(processedMapData, outputPath);

        res.json({
            success: true,
            message: `Map processed with ${operation} operation`,
            deviceId: deviceId,
            operation: operation,
            operationParams: operationParams,
            inputMap: mapName,
            outputMap: outputName,
            outputFiles: savedFiles,
            processedAt: new Date().toISOString()
        });

    } catch (error) {
        console.error('❌ Error processing map:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Delete map files
 * DELETE /api/maps/:deviceId/:mapName
 */
router.delete('/maps/:deviceId/:mapName', async (req, res) => {
    try {
        const { deviceId, mapName } = req.params;
        const { deleteAllFormats = true } = req.query;

        console.log(`🗑️ Deleting map: ${mapName} for device: ${deviceId}`);

        const mapsDir = path.join(__dirname, '../maps');
        const deletedFiles = [];

        // Delete main files
        const extensions = deleteAllFormats === 'true' ? ['.yaml', '.pgm', '.json', '.png'] : ['.yaml', '.pgm'];

        for (const ext of extensions) {
            const filePath = path.join(mapsDir, `${mapName}${ext}`);
            try {
                await fs.unlink(filePath);
                deletedFiles.push(`${mapName}${ext}`);
                console.log(`✅ Deleted: ${mapName}${ext}`);
            } catch (error) {
                // File doesn't exist, ignore
            }
        }

        if (deletedFiles.length === 0) {
            return res.status(404).json({
                success: false,
                error: `No files found for map: ${mapName}`
            });
        }

        res.json({
            success: true,
            message: `Map ${mapName} deleted successfully`,
            deviceId: deviceId,
            mapName: mapName,
            deletedFiles: deletedFiles,
            deletedAt: new Date().toISOString()
        });

    } catch (error) {
        console.error('❌ Error deleting map:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

// ==========================================
// COSTMAP-SPECIFIC ROUTES
// ==========================================

/**
 * Get current costmap data
 * GET /api/maps/:deviceId/costmaps
 */
router.get('/maps/:deviceId/costmaps', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { type = 'both' } = req.query;

        const liveData = rosConnection.getLiveData(deviceId);
        if (!liveData) {
            return res.status(404).json({
                success: false,
                error: 'No live data available for device'
            });
        }

        const costmapData = {};

        if (type === 'global' || type === 'both') {
            costmapData.global = liveData.global_costmap || null;
        }

        if (type === 'local' || type === 'both') {
            costmapData.local = liveData.local_costmap || null;
        }

        res.json({
            success: true,
            deviceId: deviceId,
            costmaps: costmapData,
            retrievedAt: new Date().toISOString()
        });

    } catch (error) {
        console.error('❌ Error getting costmap data:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Clear costmap
 * POST /api/costmaps/:deviceId/clear
 */
router.post('/costmaps/:deviceId/clear', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { costmapType } = req.body;

        console.log(`🧹 Clearing ${costmapType} costmap for device: ${deviceId}`);

        // Try to call ROS service to clear costmap
        let clearResult = { success: false, method: 'none' };

        try {
            if (costmapType === 'global' || costmapType === 'both') {
                // Call global costmap clear service
                clearResult = await callROSService(deviceId, '/global_costmap/clear_costmap', {});
            }
            
            if (costmapType === 'local' || costmapType === 'both') {
                // Call local costmap clear service
                clearResult = await callROSService(deviceId, '/local_costmap/clear_costmap', {});
            }
        } catch (rosError) {
            console.warn('⚠️ ROS costmap clear service unavailable:', rosError.message);
            clearResult = { success: false, error: rosError.message };
        }

        res.json({
            success: clearResult.success,
            message: clearResult.success ? `${costmapType} costmap cleared` : 'Failed to clear costmap',
            deviceId: deviceId,
            costmapType: costmapType,
            method: clearResult.method,
            clearedAt: new Date().toISOString(),
            details: clearResult
        });

    } catch (error) {
        console.error('❌ Error clearing costmap:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

// ==========================================
// HELPER FUNCTIONS
// ==========================================

async function publishMapToROS(deviceId, mapData, mapName) {
    try {
        // Try to use ROS map_server to load the map
        const result = await callROSService(deviceId, '/map_server/load_map', {
            map_name: mapName,
            map_data: mapData
        });
        
        return {
            success: true,
            method: 'ros_service',
            details: result
        };
    } catch (error) {
        throw new Error(`ROS map service failed: ${error.message}`);
    }
}

async function copyMapToROSDirectory(deviceId, yamlPath, pgmPath, mapName) {
    try {
        // Default ROS map directory (adjust based on your setup)
        const rosMapDir = process.env.ROS_MAP_DIR || '/opt/ros/maps';
        
        // Create directory if it doesn't exist
        await fs.mkdir(rosMapDir, { recursive: true });
        
        // Copy files
        const targetYamlPath = path.join(rosMapDir, `${mapName}.yaml`);
        const targetPgmPath = path.join(rosMapDir, `${mapName}.pgm`);
        
        await fs.copyFile(yamlPath, targetYamlPath);
        await fs.copyFile(pgmPath, targetPgmPath);
        
        return {
            success: true,
            method: 'file_copy',
            details: {
                copied_files: [targetYamlPath, targetPgmPath]
            }
        };
    } catch (error) {
        throw new Error(`File copy failed: ${error.message}`);
    }
}

async function setActiveMapOnAGV(deviceId, mapName) {
    try {
        // Call ROS service to set active map
        return await callROSService(deviceId, '/map_server/set_active_map', {
            map_name: mapName
        });
    } catch (error) {
        throw new Error(`Failed to set active map: ${error.message}`);
    }
}

async function callROSService(deviceId, serviceName, params) {
    // This would integrate with your ROS service calling mechanism
    // For now, return a placeholder
    console.log(`📞 Calling ROS service: ${serviceName} for device: ${deviceId}`);
    console.log(`📄 Parameters:`, params);
    
    // Placeholder - implement actual ROS service calls
    return {
        success: true,
        service: serviceName,
        response: 'Service call completed'
    };
}

module.exports = router;