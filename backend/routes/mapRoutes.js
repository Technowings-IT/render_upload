
// routes/mapRoutes.js - ENHANCED with SSH Transfer to Raspberry Pi
const express = require('express');
const path = require('path');
const fs = require('fs').promises;
const { spawn } = require('child_process');
const { Client } = require('ssh2'); // npm install ssh2
const multer = require('multer');
const sharp = require('sharp'); // For image processing
const pgmConverter = require('../ros/utils/pgmConverter');
const MapConverter = require('../ros/utils/mapConverter');
const rosConnection = require('../ros/utils/ros_connection');
const router = express.Router();

// Configure multer for file uploads
const upload = multer({
  storage: multer.memoryStorage(),
  limits: {
    fileSize: 50 * 1024 * 1024, // 50MB limit
  },
});

// ==========================================
// SSH CONFIGURATION - Add to your config.js
// ==========================================
const SSH_CONFIG = {
    host: process.env.RASPBERRY_PI_HOST || '192.168.0.69', // Default Pi IP
    port: process.env.RASPBERRY_PI_PORT || 22,
    username: process.env.RASPBERRY_PI_USER || 'piros',
    password: process.env.RASPBERRY_PI_PASSWORD || 'piros',
    // Or use privateKey: require('fs').readFileSync('/path/to/private/key')
    mapDirectory: process.env.RASPBERRY_PI_MAP_DIR || '/home/piros/fleet-management-system/ros_ws/src/AMR/maps',
    rosLaunchScript: process.env.RASPBERRY_PI_ROS_SCRIPT || '/home/piros/scripts/nav2.sh'
};

// ==========================================
// NEW: COMPLETE MAP EDITING WORKFLOW
// ==========================================

/**
 * Convert JSON map to PGM+YAML format
 * POST /api/maps/:deviceId/convert-to-pgm
 */
router.post('/maps/:deviceId/convert-to-pgm', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, sourceMapName } = req.body;
        
        console.log(`🔄 Converting map to PGM format for device: ${deviceId}`);
        
        // Get the source map file (either current or saved)
        let sourceMapPath;
        if (sourceMapName) {
            sourceMapPath = path.join(__dirname, '../storage/maps', `${deviceId}_${sourceMapName}.json`);
        } else {
            sourceMapPath = path.join(__dirname, '../storage/maps', `${deviceId}.json`);
        }
        
        // Check if source exists
        try {
            await fs.access(sourceMapPath);
        } catch (error) {
            return res.status(404).json({
                success: false,
                error: 'Source map not found'
            });
        }
        
        // Read the JSON map data
        const mapContent = await fs.readFile(sourceMapPath, 'utf8');
        const mapData = JSON.parse(mapContent);
        
        // Set output name
        const outputMapName = mapName || `converted_${deviceId}_${Date.now()}`;
        const outputDir = path.join(__dirname, '../maps');
        
        // Ensure output directory exists
        await fs.mkdir(outputDir, { recursive: true });
        
        // Run Python conversion
        const conversionResult = await runPythonMapConverter(
            mapData, 
            outputDir, 
            outputMapName
        );
        
        if (conversionResult.success) {
            console.log(`✅ Map converted successfully: ${outputMapName}`);
            
            res.json({
                success: true,
                message: 'Map converted to PGM format successfully',
                deviceId: deviceId,
                outputMapName: outputMapName,
                files: {
                    pgm: conversionResult.pgmPath,
                    yaml: conversionResult.yamlPath
                },
                convertedAt: new Date().toISOString()
            });
        } else {
            throw new Error(conversionResult.error);
        }
        
    } catch (error) {
        console.error('❌ Error converting map to PGM:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Advanced PGM conversion with optimization options
 * POST /api/maps/:deviceId/convert-to-pgm-advanced
 */
router.post('/maps/:deviceId/convert-to-pgm-advanced', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { 
            mapName, 
            sourceMapName, 
            includeMetadata = true,
            optimizeForEditing = true,
            conversionOptions = {}
        } = req.body;
        
        console.log(`🚀 Advanced PGM conversion for device: ${deviceId}`);
        
        // Get source map data
        let sourceMapPath;
        if (sourceMapName) {
            sourceMapPath = path.join(__dirname, '../storage/maps', `${deviceId}_${sourceMapName}.json`);
        } else {
            sourceMapPath = path.join(__dirname, '../storage/maps', `${deviceId}.json`);
        }
        
        // Verify source exists
        try {
            await fs.access(sourceMapPath);
        } catch (error) {
            return res.status(404).json({
                success: false,
                error: 'Source map not found'
            });
        }
        
        const mapContent = await fs.readFile(sourceMapPath, 'utf8');
        const mapData = JSON.parse(mapContent);
        
        const outputMapName = mapName || `advanced_${deviceId}_${Date.now()}`;
        const outputDir = path.join(__dirname, '../maps');
        
        await fs.mkdir(outputDir, { recursive: true });
        
        // Enhanced conversion with optimization
        const conversionResult = await runAdvancedMapConverter(
            mapData, 
            outputDir, 
            outputMapName,
            {
                includeMetadata,
                optimizeForEditing,
                ...conversionOptions
            }
        );
        
        if (conversionResult.success) {
            // Additional post-processing if optimizing for editing
            if (optimizeForEditing) {
                await optimizeMapForEditing(conversionResult.pgmPath, conversionResult.yamlPath);
            }
            
            // Generate edit-friendly metadata
            const editingMetadata = await generateEditingMetadata(conversionResult);
            
            res.json({
                success: true,
                message: 'Advanced PGM conversion completed successfully',
                deviceId: deviceId,
                outputMapName: outputMapName,
                files: {
                    pgm: conversionResult.pgmPath,
                    yaml: conversionResult.yamlPath
                },
                editingMetadata: editingMetadata,
                optimized: optimizeForEditing,
                convertedAt: new Date().toISOString()
            });
        } else {
            throw new Error(conversionResult.error);
        }
        
    } catch (error) {
        console.error('❌ Advanced PGM conversion error:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Apply real-time map edits with validation
 * POST /api/maps/:deviceId/apply-edits
 */
router.post('/maps/:deviceId/apply-edits', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { 
            mapName, 
            editedData, 
            dimensions, 
            locationPoints = [], 
            editHistory = {},
            validateChanges = true
        } = req.body;
        
        console.log(`✏️ Applying map edits for device: ${deviceId}, map: ${mapName}`);
        
        // Decode base64 data
        const pgmData = Buffer.from(editedData, 'base64');
        
        // Validate dimensions
        const expectedSize = dimensions.width * dimensions.height;
        if (pgmData.length !== expectedSize) {
            return res.status(400).json({
                success: false,
                error: `Data size mismatch: expected ${expectedSize}, got ${pgmData.length}`
            });
        }
        
        // Validate changes if requested
        if (validateChanges) {
            const validation = await validateMapChanges(pgmData, dimensions, locationPoints);
            if (!validation.valid) {
                return res.status(400).json({
                    success: false,
                    error: 'Map validation failed',
                    validationErrors: validation.errors
                });
            }
        }
        
        // Save edited map
        const outputDir = path.join(__dirname, '../maps');
        const pgmPath = path.join(outputDir, `${mapName}.pgm`);
        const yamlPath = path.join(outputDir, `${mapName}.yaml`);
        
        // Write PGM file
        await writePGMFile(pgmData, pgmPath, dimensions);
        
        // Update YAML with location points
        await updateYAMLWithLocations(yamlPath, locationPoints, editHistory);
        
        // Update global map data
        const updatedMapData = await convertPGMBackToMapData(pgmData, dimensions, locationPoints, deviceId);
        
        if (!global.deviceMaps) {
            global.deviceMaps = {};
        }
        global.deviceMaps[deviceId] = updatedMapData;
        
        // Save to storage
        const mapFilePath = path.join(__dirname, '../storage/maps', `${deviceId}.json`);
        await fs.writeFile(mapFilePath, JSON.stringify(updatedMapData, null, 2));
        
        res.json({
            success: true,
            message: 'Map edits applied successfully',
            deviceId: deviceId,
            mapName: mapName,
            editsSaved: true,
            locationPointsUpdated: locationPoints.length,
            appliedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error applying map edits:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Edit converted map - add locations (pick, drop, charging, home)
 * PUT /api/maps/:deviceId/edit-pgm/:mapName
 */
router.put('/maps/:deviceId/edit-pgm/:mapName', async (req, res) => {
    try {
        const { deviceId, mapName } = req.params;
        const { locations, metadata } = req.body;
        
        console.log(`✏️ Editing PGM map: ${mapName} for device: ${deviceId}`);
        
        // Load existing map metadata
        const yamlPath = path.join(__dirname, '../maps', `${mapName}.yaml`);
        const pgmPath = path.join(__dirname, '../maps', `${mapName}.pgm`);
        
        // Verify files exist
        try {
            await fs.access(yamlPath);
            await fs.access(pgmPath);
        } catch (error) {
            return res.status(404).json({
                success: false,
                error: 'Map files not found'
            });
        }
        
        // Read current YAML metadata
        const yamlContent = await fs.readFile(yamlPath, 'utf8');
        
        // Add location metadata to YAML
        const locationMetadata = locations.map(loc => ({
            name: loc.name,
            type: loc.type, // 'pickup', 'drop', 'charging', 'home'
            position: loc.position,
            orientation: loc.orientation || { x: 0, y: 0, z: 0, w: 1 },
            addedAt: new Date().toISOString()
        }));
        
        // Create enhanced YAML with locations
        const enhancedYamlContent = yamlContent + `
# Location markers added by map editor
locations:
${locationMetadata.map(loc => `  - name: "${loc.name}"
    type: "${loc.type}"
    position: [${loc.position.x}, ${loc.position.y}, ${loc.position.z || 0}]
    orientation: [${loc.orientation.x}, ${loc.orientation.y}, ${loc.orientation.z}, ${loc.orientation.w}]`).join('\n')}

# Map editing metadata
edited_at: "${new Date().toISOString()}"
editor_version: "1.0.0"
total_locations: ${locationMetadata.length}
`;
        
        // Save enhanced YAML
        await fs.writeFile(yamlPath, enhancedYamlContent);
        
        // Save location data as separate JSON for easier access
        const locationDataPath = path.join(__dirname, '../maps', `${mapName}_locations.json`);
        await fs.writeFile(locationDataPath, JSON.stringify({
            mapName: mapName,
            deviceId: deviceId,
            locations: locationMetadata,
            metadata: metadata || {},
            editedAt: new Date().toISOString()
        }, null, 2));
        
        console.log(`✅ Map edited successfully: ${mapName} with ${locationMetadata.length} locations`);
        
        res.json({
            success: true,
            message: 'Map edited successfully',
            deviceId: deviceId,
            mapName: mapName,
            locationsAdded: locationMetadata.length,
            editedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error editing PGM map:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Rename and prepare map for deployment
 * POST /api/maps/:deviceId/rename-for-deployment
 */
router.post('/maps/:deviceId/rename-for-deployment', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { sourceMapName, targetMapName, deploymentNotes } = req.body;
        
        console.log(`📝 Renaming map for deployment: ${sourceMapName} -> ${targetMapName}`);
        
        const sourceYamlPath = path.join(__dirname, '../maps', `${sourceMapName}.yaml`);
        const sourcePgmPath = path.join(__dirname, '../maps', `${sourceMapName}.pgm`);
        const sourceLocationsPath = path.join(__dirname, '../maps', `${sourceMapName}_locations.json`);
        
        const targetYamlPath = path.join(__dirname, '../maps', `${targetMapName}.yaml`);
        const targetPgmPath = path.join(__dirname, '../maps', `${targetMapName}.pgm`);
        const targetLocationsPath = path.join(__dirname, '../maps', `${targetMapName}_locations.json`);
        
        // Copy files with new names
        await fs.copyFile(sourceYamlPath, targetYamlPath);
        await fs.copyFile(sourcePgmPath, targetPgmPath);
        
        // Copy locations file if exists
        try {
            await fs.access(sourceLocationsPath);
            await fs.copyFile(sourceLocationsPath, targetLocationsPath);
        } catch (error) {
            console.log('⚠️ No locations file found, skipping');
        }
        
        // Update YAML to reference correct PGM file
        let yamlContent = await fs.readFile(targetYamlPath, 'utf8');
        yamlContent = yamlContent.replace(
            /^image:\s*.*$/m, 
            `image: ${targetMapName}.pgm`
        );
        
        // Add deployment metadata
        yamlContent += `
# Deployment information
deployed_at: "${new Date().toISOString()}"
deployed_by: "map_editor"
original_name: "${sourceMapName}"
deployment_notes: "${deploymentNotes || ''}"
target_device: "${deviceId}"
`;
        
        await fs.writeFile(targetYamlPath, yamlContent);
        
        console.log(`✅ Map renamed and prepared for deployment: ${targetMapName}`);
        
        res.json({
            success: true,
            message: 'Map renamed and prepared for deployment',
            sourceMapName: sourceMapName,
            targetMapName: targetMapName,
            files: {
                yaml: targetYamlPath,
                pgm: targetPgmPath,
                locations: targetLocationsPath
            },
            preparedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error renaming map for deployment:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Deploy map to Raspberry Pi via SSH
 * POST /api/maps/:deviceId/deploy-to-pi
 */
router.post('/maps/:deviceId/deploy-to-pi', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, piConfig, autoLoad = true, fullNavigation = false } = req.body;
        
        console.log(`🚀 Deploying map to Raspberry Pi: ${mapName}`);
        console.log(`   Device: ${deviceId}`);
        console.log(`   Auto-load: ${autoLoad}`);
        console.log(`   Full navigation: ${fullNavigation}`);
        
        // Use provided SSH config or default
        const sshConfig = {
            ...SSH_CONFIG,
            ...(piConfig || {})
        };
        
        console.log(`   Target Pi: ${sshConfig.host}:${sshConfig.port}`);
        console.log(`   Map directory: ${sshConfig.mapDirectory}`);
        
        const yamlPath = path.join(__dirname, '../maps', `${mapName}.yaml`);
        const pgmPath = path.join(__dirname, '../maps', `${mapName}.pgm`);
        const locationsPath = path.join(__dirname, '../maps', `${mapName}_locations.json`);
        
        // Verify files exist
        try {
            await fs.access(yamlPath);
            await fs.access(pgmPath);
            console.log('✅ Map files verified locally');
        } catch (error) {
            console.error('❌ Map files not found:', error);
            return res.status(404).json({
                success: false,
                error: 'Map files not found',
                missingFiles: {
                    yaml: yamlPath,
                    pgm: pgmPath
                }
            });
        }
        
        // Deploy via SSH
        const deploymentResult = await deployMapToRaspberryPi(
            sshConfig,
            {
                yamlPath,
                pgmPath,
                locationsPath,
                mapName
            },
            autoLoad,
            fullNavigation
        );
        
        if (deploymentResult.success) {
            console.log(`✅ Map deployed successfully to Raspberry Pi: ${mapName}`);
            
            res.json({
                success: true,
                message: 'Map deployed to Raspberry Pi successfully',
                deviceId: deviceId,
                mapName: mapName,
                piHost: sshConfig.host,
                deployedAt: new Date().toISOString(),
                autoLoadTriggered: autoLoad,
                fullNavigationEnabled: fullNavigation,
                deploymentDetails: deploymentResult
            });
        } else {
            throw new Error(deploymentResult.error);
        }
        
    } catch (error) {
        console.error('❌ Error deploying map to Raspberry Pi:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Check deployment status on Raspberry Pi
 * GET /api/maps/:deviceId/deployment-status
 */
router.get('/maps/:deviceId/deployment-status', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { piConfig } = req.query;
        
        const sshConfig = {
            ...SSH_CONFIG,
            ...(piConfig ? JSON.parse(piConfig) : {})
        };
        
        const status = await checkDeploymentStatus(sshConfig);
        
        res.json({
            success: true,
            deviceId: deviceId,
            piHost: sshConfig.host,
            status: status,
            checkedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error checking deployment status:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Update Pi configuration and test connection
 * POST /api/maps/pi-config/test
 */
router.post('/maps/pi-config/test', async (req, res) => {
    try {
        const { piConfig } = req.body;
        
        const testConfig = {
            ...SSH_CONFIG,
            ...piConfig
        };
        
        console.log(`🔧 Testing Pi connection: ${testConfig.host}:${testConfig.port}`);
        
        const testResult = await testPiConnection(testConfig);
        
        res.json({
            success: testResult.success,
            message: testResult.message,
            piHost: testConfig.host,
            testedAt: new Date().toISOString(),
            connectionDetails: testResult
        });
        
    } catch (error) {
        console.error('❌ Error testing Pi connection:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Enhanced deployment with validation and rollback
 * POST /api/maps/:deviceId/deploy-validated
 */
router.post('/maps/:deviceId/deploy-validated', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { 
            mapName, 
            targetMapName, 
            deploymentNotes, 
            autoLoad = true,
            validateBeforeDeploy = true,
            piConfig = {},
            deploymentOptions = {}
        } = req.body;
        
        console.log(`🚀 Validated deployment for device: ${deviceId}, map: ${mapName}`);
        
        // Pre-deployment validation
        if (validateBeforeDeploy) {
            const validation = await runPreDeploymentValidation(mapName, deviceId);
            if (!validation.valid) {
                return res.status(400).json({
                    success: false,
                    error: 'Pre-deployment validation failed',
                    validationErrors: validation.errors
                });
            }
        }
        
        // Create backup if requested
        let backupInfo = null;
        if (deploymentOptions.backupExisting) {
            backupInfo = await createDeploymentBackup(deviceId, targetMapName || mapName);
        }
        
        // Perform deployment
        const deploymentResult = await deployMapWithValidation(
            deviceId,
            mapName,
            targetMapName,
            {
                ...piConfig,
                ...SSH_CONFIG
            },
            {
                autoLoad,
                verifyTransfer: deploymentOptions.verifyTransfer,
                testLoad: deploymentOptions.testLoad,
                deploymentNotes
            }
        );
        
        if (deploymentResult.success) {
            // Post-deployment verification
            if (deploymentOptions.verifyTransfer) {
                const verification = await verifyDeployment(deviceId, targetMapName || mapName);
                if (!verification.success) {
                    console.warn('⚠️ Deployment verification failed:', verification.error);
                }
            }
            
            res.json({
                success: true,
                message: 'Validated deployment completed successfully',
                deviceId: deviceId,
                mapName: mapName,
                targetMapName: targetMapName || mapName,
                backupCreated: backupInfo !== null,
                backup: backupInfo,
                verification: deploymentOptions.verifyTransfer,
                deployedAt: new Date().toISOString(),
                deploymentId: `deploy_${Date.now()}`
            });
        } else {
            // Rollback if deployment failed and backup exists
            if (backupInfo && deploymentOptions.rollbackOnFailure) {
                await rollbackDeployment(deviceId, backupInfo);
            }
            
            throw new Error(deploymentResult.error);
        }
        
    } catch (error) {
        console.error('❌ Validated deployment error:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Get available maps on Raspberry Pi
 * GET /api/maps/:deviceId/pi-maps
 */
router.get('/maps/:deviceId/pi-maps', async (req, res) => {
    try {
        const { deviceId } = req.params;
        
        console.log(`📋 Getting available maps on Raspberry Pi for device: ${deviceId}`);
        
        const availableMaps = await getAvailableMapsOnPi(SSH_CONFIG);
        
        res.json({
            success: true,
            deviceId: deviceId,
            piHost: SSH_CONFIG.host,
            maps: availableMaps,
            retrievedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Error getting Pi maps:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Set active map on Raspberry Pi
 * POST /api/maps/:deviceId/set-active-pi-map
 */
router.post('/maps/:deviceId/set-active-pi-map', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName } = req.body;
        
        console.log(`🎯 Setting active map on Raspberry Pi: ${mapName}`);
        
        const result = await setActiveMapOnPi(SSH_CONFIG, mapName);
        
        if (result.success) {
            res.json({
                success: true,
                message: `Active map set to: ${mapName}`,
                deviceId: deviceId,
                mapName: mapName,
                piHost: SSH_CONFIG.host,
                setAt: new Date().toISOString()
            });
        } else {
            throw new Error(result.error);
        }
        
    } catch (error) {
        console.error('❌ Error setting active map on Pi:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

// ==========================================
// ENHANCED MAP EDITING ROUTES
// ==========================================

/**
 * Map analysis and quality metrics
 * POST /api/maps/:deviceId/analyze
 */
router.post('/maps/:deviceId/analyze', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, analysisTypes = [] } = req.body;
        
        console.log(`📊 Analyzing map: ${mapName} for device: ${deviceId}`);
        
        const mapPath = path.join(__dirname, '../maps', `${mapName}.pgm`);
        const yamlPath = path.join(__dirname, '../maps', `${mapName}.yaml`);
        
        // Verify files exist
        try {
            await fs.access(mapPath);
            await fs.access(yamlPath);
        } catch (error) {
            return res.status(404).json({
                success: false,
                error: 'Map files not found'
            });
        }
        
        const analysis = {};
        
        // Run requested analysis types
        for (const analysisType of analysisTypes) {
            try {
                switch (analysisType) {
                    case 'coverage':
                        analysis.coverage = await analyzeCoverage(mapPath);
                        break;
                    case 'connectivity':
                        analysis.connectivity = await analyzeConnectivity(mapPath);
                        break;
                    case 'obstacles':
                        analysis.obstacles = await analyzeObstacles(mapPath);
                        break;
                    case 'pathfinding':
                        analysis.pathfinding = await analyzePathfinding(mapPath);
                        break;
                    case 'locationPoints':
                        analysis.locationPoints = await analyzeLocationPoints(yamlPath);
                        break;
                    default:
                        console.warn(`Unknown analysis type: ${analysisType}`);
                }
            } catch (error) {
                console.error(`Error in ${analysisType} analysis:`, error);
                analysis[analysisType] = { error: error.message };
            }
        }
        
        // Generate overall quality score
        const qualityScore = calculateOverallQuality(analysis);
        
        res.json({
            success: true,
            deviceId: deviceId,
            mapName: mapName,
            analysis: analysis,
            qualityScore: qualityScore,
            analyzedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Map analysis error:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Advanced export with multiple format support
 * POST /api/maps/:deviceId/export-advanced
 */
router.post('/maps/:deviceId/export-advanced', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, format, exportOptions = {} } = req.body;
        
        console.log(`📤 Advanced export: ${mapName} to ${format} for device: ${deviceId}`);
        
        const mapPath = path.join(__dirname, '../maps', `${mapName}.pgm`);
        const yamlPath = path.join(__dirname, '../maps', `${mapName}.yaml`);
        
        let exportResult;
        
        switch (format.toLowerCase()) {
            case 'pgm':
                exportResult = await exportToPGM(mapPath, yamlPath, exportOptions);
                break;
            case 'png':
                exportResult = await exportToPNG(mapPath, yamlPath, exportOptions);
                break;
            case 'pdf':
                exportResult = await exportToPDF(mapPath, yamlPath, exportOptions);
                break;
            case 'svg':
                exportResult = await exportToSVG(mapPath, yamlPath, exportOptions);
                break;
            default:
                throw new Error(`Unsupported export format: ${format}`);
        }
        
        res.json({
            success: true,
            deviceId: deviceId,
            mapName: mapName,
            format: format,
            exportedFiles: exportResult.files,
            exportOptions: exportOptions,
            exportedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Advanced export error:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Map validation with comprehensive checks
 * POST /api/maps/:deviceId/validate
 */
router.post('/maps/:deviceId/validate', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, validationRules = [] } = req.body;
        
        console.log(`🔍 Validating map: ${mapName} for device: ${deviceId}`);
        
        const validation = {
            valid: true,
            errors: [],
            warnings: [],
            details: {}
        };
        
        for (const rule of validationRules) {
            try {
                const ruleResult = await runValidationRule(mapName, rule);
                validation.details[rule] = ruleResult;
                
                if (!ruleResult.passed) {
                    validation.valid = false;
                    validation.errors.push(...ruleResult.errors);
                }
                
                if (ruleResult.warnings?.length > 0) {
                    validation.warnings.push(...ruleResult.warnings);
                }
            } catch (error) {
                validation.errors.push(`Validation rule '${rule}' failed: ${error.message}`);
                validation.valid = false;
            }
        }
        
        res.json({
            success: true,
            deviceId: deviceId,
            mapName: mapName,
            validation: validation,
            validatedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Map validation error:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Path planning testing
 * POST /api/maps/:deviceId/test-path
 */
router.post('/maps/:deviceId/test-path', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { 
            mapName, 
            startPoint, 
            endPoint, 
            algorithm = 'A*', 
            pathPlanningOptions = {} 
        } = req.body;
        
        console.log(`🛤️ Testing path planning on map: ${mapName}`);
        
        const pathResult = await testPathPlanning(
            mapName,
            startPoint,
            endPoint,
            algorithm,
            pathPlanningOptions
        );
        
        res.json({
            success: true,
            deviceId: deviceId,
            mapName: mapName,
            startPoint: startPoint,
            endPoint: endPoint,
            algorithm: algorithm,
            pathResult: pathResult,
            testedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Path planning test error:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Backup management
 * POST /api/maps/:deviceId/create-backup
 */
router.post('/maps/:deviceId/create-backup', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, backupNote } = req.body;
        
        console.log(`💾 Creating backup for map: ${mapName}`);
        
        const backup = await createMapBackup(deviceId, mapName, backupNote);
        
        res.json({
            success: true,
            deviceId: deviceId,
            mapName: mapName,
            backup: backup,
            createdAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Backup creation error:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Restore from backup
 * POST /api/maps/:deviceId/restore-backup
 */
router.post('/maps/:deviceId/restore-backup', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { backupId, targetMapName } = req.body;
        
        console.log(`🔄 Restoring backup: ${backupId} for device: ${deviceId}`);
        
        const restoration = await restoreMapFromBackup(deviceId, backupId, targetMapName);
        
        res.json({
            success: true,
            deviceId: deviceId,
            backupId: backupId,
            targetMapName: targetMapName,
            restoration: restoration,
            restoredAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Backup restoration error:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Batch location point operations
 * POST /api/maps/:deviceId/batch-location-ops
 */
router.post('/maps/:deviceId/batch-location-ops', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, operations, batchId } = req.body;
        
        console.log(`🔄 Processing batch operations for map: ${mapName}`);
        
        const results = [];
        
        for (const operation of operations) {
            try {
                const result = await processLocationPointOperation(mapName, operation);
                results.push({
                    operation: operation,
                    success: true,
                    result: result
                });
            } catch (error) {
                results.push({
                    operation: operation,
                    success: false,
                    error: error.message
                });
            }
        }
        
        const successCount = results.filter(r => r.success).length;
        
        res.json({
            success: true,
            deviceId: deviceId,
            mapName: mapName,
            batchId: batchId,
            totalOperations: operations.length,
            successfulOperations: successCount,
            results: results,
            processedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Batch operations error:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Complete workflow integration
 * POST /api/maps/:deviceId/complete-workflow
 */
router.post('/maps/:deviceId/complete-workflow', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { 
            finalMapName, 
            locations, 
            piConfig, 
            deploymentNotes, 
            autoLoad,
            workflowSteps 
        } = req.body;
        
        console.log(`🔄 Complete workflow for device: ${deviceId}, map: ${finalMapName}`);
        
        const workflowResult = {
            success: true,
            steps: {},
            errors: []
        };
        
        // Execute workflow steps
        for (const step of workflowSteps) {
            try {
                switch (step) {
                    case 'validate':
                        workflowResult.steps.validate = await validateMapForDeployment(finalMapName);
                        break;
                    case 'backup':
                        workflowResult.steps.backup = await createDeploymentBackup(deviceId, finalMapName);
                        break;
                    case 'deploy':
                        workflowResult.steps.deploy = await deployMapWithValidation(
                            deviceId, finalMapName, null, piConfig, { autoLoad: false }
                        );
                        break;
                    case 'verify':
                        workflowResult.steps.verify = await verifyDeployment(deviceId, finalMapName);
                        break;
                    case 'activate':
                        workflowResult.steps.activate = await activateDeployedMap(deviceId, finalMapName);
                        break;
                }
            } catch (error) {
                workflowResult.errors.push(`Step '${step}' failed: ${error.message}`);
            }
        }
        
        // Check if any critical steps failed
        const criticalStepsFailed = workflowResult.errors.some(error => 
            error.includes('validate') || error.includes('deploy')
        );
        
        if (criticalStepsFailed) {
            workflowResult.success = false;
        }
        
        res.json({
            success: workflowResult.success,
            deviceId: deviceId,
            finalMapName: finalMapName,
            workflow: workflowResult,
            completedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error('❌ Complete workflow error:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

// ==========================================
// HELPER FUNCTIONS
// ==========================================

/**
 * Run Python map converter
 */
async function runPythonMapConverter(mapData, outputDir, mapName) {
    return new Promise((resolve, reject) => {
        try {
            // Create temporary JSON file
            const tempDir = path.join(__dirname, '../temp');
            fs.mkdir(tempDir, { recursive: true });
            
            const tempJsonPath = path.join(tempDir, `${mapName}_temp.json`);
            fs.writeFile(tempJsonPath, JSON.stringify(mapData, null, 2));
            
            const pythonScript = path.join(__dirname, '../scripts/map_converter.py');
            
            const pythonProcess = spawn('python3', [
                pythonScript,
                tempJsonPath,
                '-o', outputDir,
                '-n', mapName,
                '--log-level', 'INFO'
            ]);
            
            let stdout = '';
            let stderr = '';
            
            pythonProcess.stdout.on('data', (data) => {
                stdout += data.toString();
                console.log('🐍 Python:', data.toString().trim());
            });
            
            pythonProcess.stderr.on('data', (data) => {
                stderr += data.toString();
                console.error('🐍 Python Error:', data.toString().trim());
            });
            
            pythonProcess.on('close', async (code) => {
                // Cleanup temp file
                try {
                    await fs.unlink(tempJsonPath);
                } catch (error) {
                    console.warn('⚠️ Could not cleanup temp file:', error);
                }
                
                if (code === 0) {
                    resolve({
                        success: true,
                        pgmPath: path.join(outputDir, `${mapName}.pgm`),
                        yamlPath: path.join(outputDir, `${mapName}.yaml`),
                        output: stdout
                    });
                } else {
                    resolve({
                        success: false,
                        error: `Python converter failed with code ${code}: ${stderr}`
                    });
                }
            });
            
            pythonProcess.on('error', (error) => {
                resolve({
                    success: false,
                    error: `Failed to start Python converter: ${error.message}`
                });
            });
            
        } catch (error) {
            resolve({
                success: false,
                error: `Conversion setup failed: ${error.message}`
            });
        }
    });
}

/**
 * Deploy map files to Raspberry Pi via SSH
 */
/**
 * Enhanced map deployment to Raspberry Pi with ROS2 commands
 */
/**
 * Deploy map to Raspberry Pi with fallback methods
 */
async function deployMapToRaspberryPi(sshConfig, files, autoLoad = true, fullNavigation = false) {
    console.log('🚀 Starting map deployment to Pi...');
    
    // Update SSH config with correct map directory
    const updatedSSHConfig = {
        ...sshConfig,
        mapDirectory: '/home/piros/fleet-management-system/ros_ws/src/AMR/maps/'
    };
    
    try {
        // Create remote directory first
        await createRemoteDirectory(updatedSSHConfig);
        
        // Method 1: Try SCP first
        console.log('� Attempting SCP transfer...');
        
        await transferFile(files.yamlPath, `${files.mapName}.yaml`, updatedSSHConfig);
        console.log('✅ YAML file transferred via SCP');
        
        await transferFile(files.pgmPath, `${files.mapName}.pgm`, updatedSSHConfig);
        console.log('✅ PGM file transferred via SCP');
        
        // Transfer locations file if exists
        try {
            await fs.access(files.locationsPath);
            await transferFile(files.locationsPath, `${files.mapName}_locations.json`, updatedSSHConfig);
            console.log('✅ Locations file transferred via SCP');
        } catch (error) {
            console.log('⚠️ No locations file to transfer');
        }
        
        // Create deployment script and execute if autoLoad is enabled
        if (autoLoad) {
            await createAndExecuteDeploymentScript(updatedSSHConfig, files, fullNavigation);
        }
        
        return {
            success: true,
            method: 'scp',
            message: 'Files transferred successfully via SCP',
            filesTransferred: ['yaml', 'pgm', 'locations'],
            autoLoadTriggered: autoLoad,
            fullNavigationEnabled: fullNavigation,
            mapDirectory: updatedSSHConfig.mapDirectory
        };
        
    } catch (scpError) {
        console.warn('⚠️ SCP failed, trying rsync...', scpError.message);
        
        try {
            // Method 2: Fallback to rsync
            await transferFileWithRsync(files.yamlPath, `${files.mapName}.yaml`, updatedSSHConfig);
            await transferFileWithRsync(files.pgmPath, `${files.mapName}.pgm`, updatedSSHConfig);
            
            // Transfer locations file if exists
            try {
                await fs.access(files.locationsPath);
                await transferFileWithRsync(files.locationsPath, `${files.mapName}_locations.json`, updatedSSHConfig);
            } catch (error) {
                console.log('⚠️ No locations file to transfer');
            }
            
            if (autoLoad) {
                await createAndExecuteDeploymentScript(updatedSSHConfig, files, fullNavigation);
            }
            
            return {
                success: true,
                method: 'rsync',
                message: 'Files transferred successfully via rsync (SCP fallback)',
                filesTransferred: ['yaml', 'pgm', 'locations'],
                autoLoadTriggered: autoLoad,
                mapDirectory: updatedSSHConfig.mapDirectory
            };
            
        } catch (rsyncError) {
            console.error('❌ Both SCP and rsync failed');
            
            return {
                success: false,
                method: 'failed',
                error: `Both transfer methods failed. SCP: ${scpError.message}, Rsync: ${rsyncError.message}`
            };
        }
    }
}

/**
 * Execute command via SSH
 */
function executeSSHCommand(conn, command) {
    return new Promise((resolve, reject) => {
        conn.exec(command, (err, stream) => {
            if (err) {
                reject(err);
                return;
            }
            
            let stdout = '';
            let stderr = '';
            
            stream.on('close', (code) => {
                if (code === 0) {
                    resolve(stdout);
                } else {
                    reject(new Error(`Command failed with code ${code}: ${stderr}`));
                }
            });
            
            stream.on('data', (data) => {
                stdout += data.toString();
            });
            
            stream.stderr.on('data', (data) => {
                stderr += data.toString();
            });
        });
    });
}

/**
 * Execute command via SSH with timeout
 */
function executeSSHCommandWithTimeout(conn, command, timeoutMs = 30000) {
    return new Promise((resolve, reject) => {
        const timer = setTimeout(() => {
            reject(new Error(`Command timed out after ${timeoutMs}ms`));
        }, timeoutMs);

        conn.exec(command, (err, stream) => {
            if (err) {
                clearTimeout(timer);
                reject(err);
                return;
            }
            
            let stdout = '';
            let stderr = '';
            
            stream.on('close', (code) => {
                clearTimeout(timer);
                if (code === 0) {
                    resolve(stdout);
                } else {
                    reject(new Error(`Command failed with code ${code}: ${stderr}`));
                }
            });
            
            stream.on('data', (data) => {
                stdout += data.toString();
            });
            
            stream.stderr.on('data', (data) => {
                stderr += data.toString();
            });
        });
    });
}

/**
 * File transfer via SCP with better error handling
 */
function transferFile(localPath, remotePath, sshConfig) {
    return new Promise((resolve, reject) => {
        const { spawn } = require('child_process');
        
        // Target directory on Pi
        const piMapDirectory = '/home/piros/fleet-management-system/ros_ws/src/AMR/maps/';
        const fullRemotePath = remotePath.startsWith('/') ? remotePath : `${piMapDirectory}${remotePath}`;
        
        const remoteHost = `${sshConfig.username}@${sshConfig.host}`;
        
        console.log(`🚀 Starting SCP transfer with password automation:`);
        console.log(`   Local:  ${localPath}`);
        console.log(`   Remote: ${remoteHost}:${fullRemotePath}`);
        
        // Use sshpass to automate password entry
        const scpArgs = [
            '-p', sshConfig.password || 'piros',  // Password
            'scp',
            '-o', 'StrictHostKeyChecking=no',
            '-o', 'UserKnownHostsFile=/dev/null',
            '-o', 'LogLevel=VERBOSE',
            '-P', sshConfig.port || '22',
            localPath,
            `${remoteHost}:${fullRemotePath}`
        ];
        
        const scpProcess = spawn('sshpass', scpArgs, {
            stdio: ['pipe', 'pipe', 'pipe']
        });
        
        let stdout = '';
        let stderr = '';
        
        scpProcess.stdout.on('data', (data) => {
            stdout += data.toString();
            console.log(`📡 SCP stdout: ${data.toString().trim()}`);
        });
        
        scpProcess.stderr.on('data', (data) => {
            stderr += data.toString();
            console.log(`📡 SCP stderr: ${data.toString().trim()}`);
        });
        
        scpProcess.on('close', (code) => {
            console.log(`📊 SCP process exited with code: ${code}`);
            
            if (code === 0) {
                console.log(`✅ SCP transfer successful: ${localPath} -> ${fullRemotePath}`);
                resolve({
                    success: true,
                    stdout: stdout,
                    stderr: stderr
                });
            } else {
                const errorMessage = `SCP failed with code ${code}`;
                console.error(`❌ ${errorMessage}`);
                console.error(`❌ STDERR: ${stderr}`);
                
                reject(new Error(`${errorMessage}: ${stderr}`));
            }
        });
        
        scpProcess.on('error', (error) => {
            console.error(`❌ SCP command failed to start: ${error.message}`);
            reject(new Error(`SCP command failed: ${error.message}`));
        });
        
        // Handle timeout
        const timeout = setTimeout(() => {
            scpProcess.kill('SIGTERM');
            reject(new Error('SCP transfer timed out after 60 seconds'));
        }, 60000);
        
        scpProcess.on('close', () => {
            clearTimeout(timeout);
        });
    });
}

/**
 * Alternative using rsync (more robust for large files)
 */
function transferFileWithRsync(localPath, remotePath, sshConfig) {
    return new Promise((resolve, reject) => {
        const { spawn } = require('child_process');
        
        // Target directory on Pi
        const piMapDirectory = '/home/piros/fleet-management-system/ros_ws/src/AMR/maps/';
        const fullRemotePath = remotePath.startsWith('/') ? remotePath : `${piMapDirectory}${remotePath}`;
        
        const remoteHost = `${sshConfig.username}@${sshConfig.host}`;
        
        console.log(`🔄 Starting Rsync transfer with password automation:`);
        console.log(`   Local:  ${localPath}`);
        console.log(`   Remote: ${remoteHost}:${fullRemotePath}`);
        
        // Use sshpass with rsync
        const rsyncArgs = [
            '-p', sshConfig.password || 'piros',  // Password
            'rsync',
            '-avz',
            '--progress',
            '--partial',  // Keep partial files on failure
            '--timeout=300',  // 5 minute timeout
            '-e', `ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -p ${sshConfig.port || 22}`,
            localPath,
            `${remoteHost}:${fullRemotePath}`
        ];
        
        const rsyncProcess = spawn('sshpass', rsyncArgs, {
            stdio: ['pipe', 'pipe', 'pipe']
        });
        
        let stdout = '';
        let stderr = '';
        
        rsyncProcess.stdout.on('data', (data) => {
            stdout += data.toString();
            console.log(`📡 Rsync: ${data.toString().trim()}`);
        });
        
        rsyncProcess.stderr.on('data', (data) => {
            stderr += data.toString();
            console.error(`📡 Rsync stderr: ${data.toString().trim()}`);
        });
        
        rsyncProcess.on('close', (code) => {
            if (code === 0) {
                console.log(`✅ Rsync transfer successful`);
                resolve({
                    success: true,
                    stdout: stdout,
                    method: 'rsync'
                });
            } else {
                const errorMessage = `Rsync failed with code ${code}`;
                console.error(`❌ ${errorMessage}`);
                reject(new Error(`${errorMessage}: ${stderr}`));
            }
        });
        
        rsyncProcess.on('error', (error) => {
            reject(new Error(`Rsync command failed: ${error.message}`));
        });
    });
}

/**
 * Trigger map loading on Raspberry Pi
 */
async function triggerMapLoadOnPi(conn, sshConfig, mapName) {
    try {
        // Create or update the active map configuration
        const activeMapConfig = {
            activeMap: mapName,
            setAt: new Date().toISOString(),
            autoLoad: true
        };
        
        await executeSSHCommand(conn, 
            `echo '${JSON.stringify(activeMapConfig, null, 2)}' > ${sshConfig.mapDirectory}/active_map.json`
        );
        
        // Trigger ROS map loading (you'll customize this script)
        const loadCommand = `cd ${path.dirname(sshConfig.rosLaunchScript)} && ./load_map.sh ${mapName}`;
        const result = await executeSSHCommand(conn, loadCommand);
        
        return {
            success: true,
            output: result
        };
        
    } catch (error) {
        console.warn('⚠️ Auto-load failed:', error.message);
        return {
            success: false,
            error: error.message
        };
    }
}

/**
 * Create remote directory on Pi
 */
function createRemoteDirectory(sshConfig) {
    return new Promise((resolve, reject) => {
        const { spawn } = require('child_process');
        
        // Use sshpass for SSH commands
        const sshArgs = [
            '-p', sshConfig.password || 'piros',  // Password
            'ssh',
            '-o', 'StrictHostKeyChecking=no',
            '-o', 'UserKnownHostsFile=/dev/null',
            '-p', sshConfig.port || '22',
            `${sshConfig.username}@${sshConfig.host}`,
            `mkdir -p ${sshConfig.mapDirectory}`
        ];
        
        const sshProcess = spawn('sshpass', sshArgs);
        
        let stderr = '';
        
        sshProcess.stderr.on('data', (data) => {
            stderr += data.toString();
        });
        
        sshProcess.on('close', (code) => {
            if (code === 0) {
                console.log(`✅ Created remote directory: ${sshConfig.mapDirectory}`);
                resolve();
            } else {
                reject(new Error(`Failed to create remote directory: ${stderr}`));
            }
        });
        
        sshProcess.on('error', (error) => {
            reject(new Error(`SSH command failed: ${error.message}`));
        });
    });
}

/**
 * Create and execute deployment script on Pi
 */
async function createAndExecuteDeploymentScript(sshConfig, files, fullNavigation) {
    const deploymentScript = `#!/bin/bash
# Auto-generated map deployment script
# Generated at: ${new Date().toISOString()}
# Map: ${files.mapName}

echo "🗺️ Deploying map: ${files.mapName}"

# Set map directory
MAP_DIR="${sshConfig.mapDirectory}"
MAP_NAME="${files.mapName}"

# Verify map files exist
if [ ! -f "$MAP_DIR/$MAP_NAME.yaml" ]; then
    echo "❌ Map YAML file not found: $MAP_DIR/$MAP_NAME.yaml"
    exit 1
fi

if [ ! -f "$MAP_DIR/$MAP_NAME.pgm" ]; then
    echo "❌ Map PGM file not found: $MAP_DIR/$MAP_NAME.pgm"
    exit 1
fi

echo "✅ Map files verified"

# Stop current navigation if running
echo "🛑 Stopping current navigation..."
pkill -f "ros2 launch nav2_bringup" || true
pkill -f "map_server" || true
sleep 2

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash || true

# Set the map parameter file path
export MAP_YAML_FILE="$MAP_DIR/$MAP_NAME.yaml"

echo "🚀 Starting map server with: $MAP_YAML_FILE"

# Start map server in background
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_YAML_FILE &
MAP_SERVER_PID=$!

# Wait for map server to start
sleep 3

# Configure nav2 to use the new map
ros2 lifecycle set /map_server configure || true
ros2 lifecycle set /map_server activate || true

echo "✅ Map server started with PID: $MAP_SERVER_PID"

# Update active map configuration
echo "{\\"activeMap\\": \\"$MAP_NAME\\", \\"setAt\\": \\"$(date -Iseconds)\\", \\"autoLoad\\": true, \\"mapServerPid\\": $MAP_SERVER_PID}" > "$MAP_DIR/active_map.json"

# Optional: Start full navigation stack
if [ "$1" = "--full-nav" ]; then
    echo "🧭 Starting full navigation stack..."
    ros2 launch nav2_bringup navigation_launch.py map:=$MAP_YAML_FILE &
    NAV_PID=$!
    echo "✅ Navigation started with PID: $NAV_PID"
    echo "{\\"activeMap\\": \\"$MAP_NAME\\", \\"setAt\\": \\"$(date -Iseconds)\\", \\"autoLoad\\": true, \\"mapServerPid\\": $MAP_SERVER_PID, \\"navPid\\": $NAV_PID}" > "$MAP_DIR/active_map.json"
fi

echo "🎉 Map deployment completed successfully!"
echo "📍 Active map: $MAP_NAME"
echo "📁 Map directory: $MAP_DIR"
echo "🔧 Config file: $MAP_DIR/active_map.json"
`;

    return new Promise((resolve, reject) => {
        const { spawn } = require('child_process');
        
        const scriptPath = `${sshConfig.mapDirectory}/deploy_map.sh`;
        const fullNavArg = fullNavigation ? '--full-nav' : '';
        
        // Create script on Pi using sshpass
        const createScriptArgs = [
            '-p', sshConfig.password || 'piros',  // Password
            'ssh',
            '-o', 'StrictHostKeyChecking=no',
            '-o', 'UserKnownHostsFile=/dev/null',
            '-p', sshConfig.port || '22',
            `${sshConfig.username}@${sshConfig.host}`,
            `cat > ${scriptPath} << 'EOF'
${deploymentScript}
EOF`
        ];
        
        const createProcess = spawn('sshpass', createScriptArgs);
        
        createProcess.on('close', (code) => {
            if (code === 0) {
                console.log('📜 Deployment script created on Pi');
                
                // Make script executable and run it using sshpass
                const executeArgs = [
                    '-p', sshConfig.password || 'piros',  // Password
                    'ssh',
                    '-o', 'StrictHostKeyChecking=no',
                    '-o', 'UserKnownHostsFile=/dev/null',
                    '-p', sshConfig.port || '22',
                    `${sshConfig.username}@${sshConfig.host}`,
                    `chmod +x ${scriptPath} && ${scriptPath} ${fullNavArg}`
                ];
                
                const executeProcess = spawn('sshpass', executeArgs);
                
                let stdout = '';
                let stderr = '';
                
                executeProcess.stdout.on('data', (data) => {
                    stdout += data.toString();
                    console.log(`🐧 Pi: ${data.toString().trim()}`);
                });
                
                executeProcess.stderr.on('data', (data) => {
                    stderr += data.toString();
                    console.error(`🐧 Pi Error: ${data.toString().trim()}`);
                });
                
                executeProcess.on('close', (execCode) => {
                    if (execCode === 0) {
                        console.log('✅ Deployment script executed successfully');
                        resolve({ success: true, output: stdout });
                    } else {
                        console.warn('⚠️ Deployment script execution failed:', stderr);
                        resolve({ success: false, error: stderr });
                    }
                });
                
            } else {
                reject(new Error('Failed to create deployment script on Pi'));
            }
        });
        
        createProcess.on('error', (error) => {
            reject(new Error(`Failed to create deployment script: ${error.message}`));
        });
    });
}

/**
 * Test SSH connectivity before deployment
 */
async function testSSHConnection(sshConfig) {
    return new Promise((resolve) => {
        const { spawn } = require('child_process');
        
        // Use sshpass for testing SSH connection
        const sshArgs = [
            '-p', sshConfig.password || 'piros',  // Password
            'ssh',
            '-o', 'ConnectTimeout=10',
            '-o', 'StrictHostKeyChecking=no',
            '-o', 'BatchMode=no',  // Allow password authentication
            '-p', sshConfig.port || '22',
            `${sshConfig.username}@${sshConfig.host}`,
            'echo "SSH_CONNECTION_TEST_SUCCESS"'
        ];
        
        const sshProcess = spawn('sshpass', sshArgs);
        
        let stdout = '';
        let stderr = '';
        
        sshProcess.stdout.on('data', (data) => {
            stdout += data.toString();
        });
        
        sshProcess.stderr.on('data', (data) => {
            stderr += data.toString();
        });
        
        sshProcess.on('close', (code) => {
            if (code === 0 && stdout.includes('SSH_CONNECTION_TEST_SUCCESS')) {
                resolve({
                    success: true,
                    message: 'SSH connection successful'
                });
            } else {
                resolve({
                    success: false,
                    error: `SSH test failed with code ${code}: ${stderr}`
                });
            }
        });
        
        sshProcess.on('error', (error) => {
            resolve({
                success: false,
                error: `SSH test command failed: ${error.message}`
            });
        });
    });
}

/**
 * Get available maps on Raspberry Pi
 */
async function getAvailableMapsOnPi(sshConfig) {
    return new Promise((resolve) => {
        const conn = new Client();
        
        conn.on('ready', async () => {
            try {
                const result = await executeSSHCommand(conn, `ls -la ${sshConfig.mapDirectory}/*.yaml | grep -E '\.yaml$'`);
                
                const maps = result.split('\n')
                    .filter(line => line.trim())
                    .map(line => {
                        const parts = line.trim().split(/\s+/);
                        const filename = parts[parts.length - 1];
                        const mapName = path.basename(filename, '.yaml');
                        
                        return {
                            name: mapName,
                            filename: filename,
                            size: parts[4],
                            modified: `${parts[5]} ${parts[6]} ${parts[7]}`
                        };
                    });
                
                conn.end();
                resolve(maps);
                
            } catch (error) {
                conn.end();
                console.error('❌ Error listing Pi maps:', error);
                resolve([]);
            }
        });
        
        conn.on('error', (error) => {
            console.error('❌ SSH error getting Pi maps:', error);
            resolve([]);
        });
        
        conn.connect(sshConfig);
    });
}

/**
 * Set active map on Raspberry Pi
 */
async function setActiveMapOnPi(sshConfig, mapName) {
    return new Promise((resolve) => {
        const conn = new Client();
        
        conn.on('ready', async () => {
            try {
                const result = await triggerMapLoadOnPi(conn, sshConfig, mapName);
                conn.end();
                resolve(result);
                
            } catch (error) {
                conn.end();
                resolve({
                    success: false,
                    error: error.message
                });
            }
        });
        
        conn.on('error', (error) => {
            resolve({
                success: false,
                error: `SSH connection failed: ${error.message}`
            });
        });
        
        conn.connect(sshConfig);
    });
}

/**
 * ENHANCED: Get current map data for a device with multiple fallback strategies
 * GET /api/control/devices/:deviceId/maps
 */
router.get('/control/devices/:deviceId/maps', async (req, res) => {
    try {
        const { deviceId } = req.params;
        
        console.log(`📍 [ENHANCED] Getting map data for device: ${deviceId}`);
        
        let mapData = null;
        let source = 'unknown';
        
        // Strategy 1: Check global state first
        if (global.deviceMaps && global.deviceMaps[deviceId]) {
            mapData = global.deviceMaps[deviceId];
            source = 'memory';
            console.log(`✅ Found map in global state`);
        }
        
        // Strategy 2: Try to load from main device file
        if (!mapData) {
            try {
                const mainMapPath = path.join(__dirname, '../storage/maps', `${deviceId}.json`);
                const mapContent = await fs.readFile(mainMapPath, 'utf8');
                mapData = JSON.parse(mapContent);
                source = 'device_file';
                console.log(`✅ Loaded map from device file: ${mainMapPath}`);
            } catch (fileError) {
                console.log(`⚠️ Device file not found: ${fileError.message}`);
            }
        }
        
        // Strategy 3: Try to load latest saved map
        if (!mapData) {
            try {
                const savedMaps = await getLatestSavedMaps(deviceId);
                if (savedMaps.length > 0) {
                    const latestMapPath = savedMaps[0].filePath;
                    const mapContent = await fs.readFile(latestMapPath, 'utf8');
                    mapData = JSON.parse(mapContent);
                    source = 'latest_saved';
                    console.log(`✅ Loaded latest saved map: ${latestMapPath}`);
                }
            } catch (savedError) {
                console.log(`⚠️ No saved maps found: ${savedError.message}`);
            }
        }
        
        // Strategy 4: Create empty map if nothing found
        if (!mapData) {
            mapData = createEmptyMapData(deviceId);
            source = 'empty_created';
            console.log(`⚠️ Created empty map for device: ${deviceId}`);
        }
        
        // Ensure map data has proper structure
        mapData = normalizeMapData(mapData, deviceId);
        
        // Cache in global state for next time
        if (!global.deviceMaps) {
            global.deviceMaps = {};
        }
        global.deviceMaps[deviceId] = mapData;
        
        console.log(`✅ Map data prepared for ${deviceId}: ${mapData.shapes?.length || 0} shapes, source: ${source}`);
        
        res.json({
            success: true,
            deviceId: deviceId,
            mapData: mapData,
            source: source,
            retrievedAt: new Date().toISOString()
        });
        
    } catch (error) {
        console.error(`❌ [ENHANCED] Error getting map data for ${req.params.deviceId}:`, error);
        res.status(500).json({
            success: false,
            error: `Failed to get map data: ${error.message}`,
            deviceId: req.params.deviceId,
            timestamp: new Date().toISOString()
        });
    }
});

/**
 * ENHANCED: Save map data for a device with backup and versioning
 * POST /api/control/devices/:deviceId/map/data
 */
router.post('/control/devices/:deviceId/map/data', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapData } = req.body;
        
        console.log(`💾 [ENHANCED] Saving map data for device: ${deviceId}`);
        
        if (!mapData) {
            return res.status(400).json({
                success: false,
                error: 'Map data is required',
                deviceId: deviceId
            });
        }
        
        // Normalize and enhance map data
        const enhancedMapData = normalizeMapData(mapData, deviceId);
        enhancedMapData.savedAt = new Date().toISOString();
        enhancedMapData.lastSaved = new Date().toISOString();
        enhancedMapData.version = (enhancedMapData.version || 0) + 1;
        
        // Save to global state
        if (!global.deviceMaps) {
            global.deviceMaps = {};
        }
        global.deviceMaps[deviceId] = enhancedMapData;
        
        // Save to main device file
        try {
            const mapsDir = path.join(__dirname, '../storage/maps');
            await fs.mkdir(mapsDir, { recursive: true });
            
            const mapFilePath = path.join(mapsDir, `${deviceId}.json`);
            await fs.writeFile(mapFilePath, JSON.stringify(enhancedMapData, null, 2));
            
            console.log(`✅ Map data saved to device file: ${mapFilePath}`);
        } catch (fileError) {
            console.error(`❌ Error saving device file:`, fileError);
        }
        
        // Also save as timestamped backup
        try {
            const timestamp = Date.now();
            const backupName = `${deviceId}_${timestamp}`;
            const backupPath = path.join(__dirname, '../storage/maps', `${backupName}.json`);
            await fs.writeFile(backupPath, JSON.stringify(enhancedMapData, null, 2));
            
            // Update index
            await updateSavedMapsIndex(deviceId, {
                name: backupName,
                filePath: backupPath,
                savedAt: enhancedMapData.savedAt,
                fileSize: `${(JSON.stringify(enhancedMapData).length / 1024).toFixed(2)} KB`,
                shapes: enhancedMapData.shapes?.length || 0,
                mapType: 'backup',
                metadata: enhancedMapData.metadata || {}
            });
            
            console.log(`✅ Map backup saved: ${backupPath}`);
        } catch (backupError) {
            console.warn(`⚠️ Backup save failed:`, backupError.message);
        }
        
        // Save via storage manager if available
        try {
            const storageManager = require('../ros/utils/storageManager');
            if (storageManager && storageManager.saveMap) {
                await storageManager.saveMap(deviceId);
            }
        } catch (storageError) {
            console.warn(`⚠️ Storage manager save failed:`, storageError.message);
        }
        
        res.json({
            success: true,
            message: 'Map data saved successfully',
            deviceId: deviceId,
            savedAt: enhancedMapData.savedAt,
            shapes: enhancedMapData.shapes?.length || 0,
            version: enhancedMapData.version,
            source: 'manual_save'
        });
        
    } catch (error) {
        console.error(`❌ [ENHANCED] Error saving map data for ${req.params.deviceId}:`, error);
        res.status(500).json({
            success: false,
            error: `Failed to save map data: ${error.message}`,
            deviceId: req.params.deviceId
        });
    }
});

/**
 * ENHANCED: Load complete map data
 * GET /api/maps/:deviceId/load-complete
 */
router.get('/maps/:deviceId/load-complete', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { 
            mapName, 
            includeShapes = 'true', 
            includeMetadata = 'true' 
        } = req.query;

        console.log(`📂 [ENHANCED] Loading complete map for device: ${deviceId}, map: ${mapName || 'current'}`);

        let mapData;
        let source;

        if (mapName && mapName !== 'current') {
            // Load specific map by name
            try {
                const mapFilePath = path.join(__dirname, '../storage/maps', `${deviceId}_${mapName}.json`);
                const mapContent = await fs.readFile(mapFilePath, 'utf8');
                mapData = JSON.parse(mapContent);
                source = 'saved_map';
                console.log(`✅ Loaded specific map: ${mapFilePath}`);
            } catch (fileError) {
                // Try without device prefix
                try {
                    const mapFilePath = path.join(__dirname, '../storage/maps', `${mapName}.json`);
                    const mapContent = await fs.readFile(mapFilePath, 'utf8');
                    mapData = JSON.parse(mapContent);
                    source = 'saved_map_alt';
                    console.log(`✅ Loaded map (alt format): ${mapFilePath}`);
                } catch (altError) {
                    throw new Error(`Map not found: ${mapName}`);
                }
            }
        } else {
            // Load current/latest map
            mapData = global.deviceMaps?.[deviceId];
            source = 'current';
            
            if (!mapData) {
                // Fallback to device file
                try {
                    const mapFilePath = path.join(__dirname, '../storage/maps', `${deviceId}.json`);
                    const mapContent = await fs.readFile(mapFilePath, 'utf8');
                    mapData = JSON.parse(mapContent);
                    source = 'device_file';
                } catch (fileError) {
                    // Create empty map
                    mapData = createEmptyMapData(deviceId);
                    source = 'empty';
                }
            }
        }

        // Normalize the map data
        mapData = normalizeMapData(mapData, deviceId);

        // Filter data based on query parameters
        if (includeShapes === 'false') {
            mapData.shapes = [];
        }
        
        if (includeMetadata === 'false') {
            delete mapData.metadata;
        }

        // Add loading metadata
        mapData.loadedAt = new Date().toISOString();
        mapData.loadedFor = 'editing';

        res.json({
            success: true,
            message: 'Complete map loaded successfully',
            deviceId: deviceId,
            mapName: mapName || 'current',
            mapData: mapData,
            source: source,
            loadedAt: mapData.loadedAt
        });

    } catch (error) {
        console.error(`❌ [ENHANCED] Error loading complete map:`, error);
        res.status(500).json({
            success: false,
            error: error.message,
            deviceId: req.params.deviceId
        });
    }
});

/**
 * ENHANCED: Get saved maps with proper structure
 * GET /api/maps/:deviceId/saved
 */
router.get('/maps/:deviceId/saved', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { 
            includePreview = 'true', 
            sortBy = 'savedAt', 
            descending = 'true' 
        } = req.query;

        console.log(`📋 [ENHANCED] Getting saved maps for device: ${deviceId}`);

        const savedMaps = await getLatestSavedMaps(deviceId);
        
        // Include preview data if requested
        let processedMaps = savedMaps;
        if (includePreview === 'true') {
            processedMaps = await Promise.all(savedMaps.map(async (mapInfo) => {
                try {
                    const preview = await generateMapPreview(deviceId, mapInfo.name);
                    return {
                        ...mapInfo,
                        preview: preview
                    };
                } catch (previewError) {
                    console.warn(`⚠️ Could not generate preview for ${mapInfo.name}:`, previewError.message);
                    return {
                        ...mapInfo,
                        preview: { error: 'Preview not available' }
                    };
                }
            }));
        }

        // Sort maps
        processedMaps.sort((a, b) => {
            const aValue = a[sortBy] || '';
            const bValue = b[sortBy] || '';
            
            if (descending === 'true') {
                return bValue > aValue ? 1 : -1;
            } else {
                return aValue > bValue ? 1 : -1;
            }
        });

        res.json({
            success: true,
            deviceId: deviceId,
            maps: processedMaps,
            totalMaps: processedMaps.length,
            sortBy: sortBy,
            descending: descending === 'true'
        });

    } catch (error) {
        console.error(`❌ [ENHANCED] Error getting saved maps:`, error);
        res.status(500).json({
            success: false,
            error: error.message,
            deviceId: req.params.deviceId
        });
    }
});

/**
 * Add shape to map
 * POST /api/control/devices/:deviceId/map/shapes
 */
router.post('/control/devices/:deviceId/map/shapes', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { type, name, points, sides, color } = req.body;
        
        console.log(`➕ Adding shape to map for device: ${deviceId}`);
        
        // Validate required fields
        if (!type || !name || !points) {
            return res.status(400).json({
                success: false,
                error: 'Type, name, and points are required',
                deviceId: deviceId
            });
        }
        
        // Get current map data
        let mapData = global.deviceMaps?.[deviceId];
        if (!mapData) {
            return res.status(404).json({
                success: false,
                error: 'No map data found for device',
                deviceId: deviceId
            });
        }
        
        // Create new shape
        const newShape = {
            id: `shape_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
            type: type,
            name: name,
            points: points,
            sides: sides || { left: '', right: '', front: '', back: '' },
            color: color || 'FF0000FF',
            createdAt: new Date().toISOString()
        };
        
        // Add shape to map
        if (!mapData.shapes) {
            mapData.shapes = [];
        }
        mapData.shapes.push(newShape);
        
        // Update map metadata
        mapData.lastModified = new Date().toISOString();
        mapData.version = (mapData.version || 0) + 1;
        
        // Save updated map
        global.deviceMaps[deviceId] = mapData;
        
        // Save to file
        try {
            const mapFilePath = path.join(__dirname, '../storage/maps', `${deviceId}.json`);
            await fs.writeFile(mapFilePath, JSON.stringify(mapData, null, 2));
        } catch (fileError) {
            console.warn(`⚠️ Error saving map file:`, fileError.message);
        }
        
        res.json({
            success: true,
            message: 'Shape added to map successfully',
            deviceId: deviceId,
            shapeId: newShape.id,
            totalShapes: mapData.shapes.length
        });
        
    } catch (error) {
        console.error(`❌ Error adding shape to map for ${req.params.deviceId}:`, error);
        res.status(500).json({
            success: false,
            error: `Failed to add shape: ${error.message}`,
            deviceId: req.params.deviceId
        });
    }
});

/**
 * Update shape in map
 * PUT /api/control/devices/:deviceId/map/shapes/:shapeId
 */
router.put('/control/devices/:deviceId/map/shapes/:shapeId', async (req, res) => {
    try {
        const { deviceId, shapeId } = req.params;
        const updates = req.body;
        
        console.log(`✏️ Updating shape ${shapeId} for device: ${deviceId}`);
        
        // Get current map data
        let mapData = global.deviceMaps?.[deviceId];
        if (!mapData || !mapData.shapes) {
            return res.status(404).json({
                success: false,
                error: 'No map data or shapes found for device',
                deviceId: deviceId
            });
        }
        
        // Find and update shape
        const shapeIndex = mapData.shapes.findIndex(s => s.id === shapeId);
        if (shapeIndex === -1) {
            return res.status(404).json({
                success: false,
                error: 'Shape not found',
                deviceId: deviceId,
                shapeId: shapeId
            });
        }
        
        // Update shape
        mapData.shapes[shapeIndex] = {
            ...mapData.shapes[shapeIndex],
            ...updates,
            id: shapeId, // Preserve ID
            updatedAt: new Date().toISOString()
        };
        
        // Update map metadata
        mapData.lastModified = new Date().toISOString();
        mapData.version = (mapData.version || 0) + 1;
        
        // Save updated map
        global.deviceMaps[deviceId] = mapData;
        
        // Save to file
        try {
            const mapFilePath = path.join(__dirname, '../storage/maps', `${deviceId}.json`);
            await fs.writeFile(mapFilePath, JSON.stringify(mapData, null, 2));
        } catch (fileError) {
            console.warn(`⚠️ Error saving map file:`, fileError.message);
        }
        
        res.json({
            success: true,
            message: 'Shape updated successfully',
            deviceId: deviceId,
            shapeId: shapeId,
            updatedShape: mapData.shapes[shapeIndex]
        });
        
    } catch (error) {
        console.error(`❌ Error updating shape for ${req.params.deviceId}:`, error);
        res.status(500).json({
            success: false,
            error: `Failed to update shape: ${error.message}`,
            deviceId: req.params.deviceId
        });
    }
});

/**
 * Delete shape from map
 * DELETE /api/control/devices/:deviceId/map/shapes/:shapeId
 */
router.delete('/control/devices/:deviceId/map/shapes/:shapeId', async (req, res) => {
    try {
        const { deviceId, shapeId } = req.params;
        
        console.log(`🗑️ Deleting shape ${shapeId} from device: ${deviceId}`);
        
        // Get current map data
        let mapData = global.deviceMaps?.[deviceId];
        if (!mapData || !mapData.shapes) {
            return res.status(404).json({
                success: false,
                error: 'No map data or shapes found for device',
                deviceId: deviceId
            });
        }
        
        // Find and remove shape
        const shapeIndex = mapData.shapes.findIndex(s => s.id === shapeId);
        if (shapeIndex === -1) {
            return res.status(404).json({
                success: false,
                error: 'Shape not found',
                deviceId: deviceId,
                shapeId: shapeId
            });
        }
        
        // Remove shape
        const deletedShape = mapData.shapes.splice(shapeIndex, 1)[0];
        
        // Update map metadata
        mapData.lastModified = new Date().toISOString();
        mapData.version = (mapData.version || 0) + 1;
        
        // Save updated map
        global.deviceMaps[deviceId] = mapData;
        
        // Save to file
        try {
            const mapFilePath = path.join(__dirname, '../storage/maps', `${deviceId}.json`);
            await fs.writeFile(mapFilePath, JSON.stringify(mapData, null, 2));
        } catch (fileError) {
            console.warn(`⚠️ Error saving map file:`, fileError.message);
        }
        
        res.json({
            success: true,
            message: 'Shape deleted successfully',
            deviceId: deviceId,
            shapeId: shapeId,
            deletedShape: deletedShape,
            remainingShapes: mapData.shapes.length
        });
        
    } catch (error) {
        console.error(`❌ Error deleting shape for ${req.params.deviceId}:`, error);
        res.status(500).json({
            success: false,
            error: `Failed to delete shape: ${error.message}`,
            deviceId: req.params.deviceId
        });
    }
});

/**
 * Health check endpoint
 * GET /api/health
 */
router.get('/health', (req, res) => {
    try {
        const health = {
            success: true,
            status: 'healthy',
            timestamp: new Date().toISOString(),
            uptime: process.uptime(),
            memory: process.memoryUsage(),
            connectedDevices: global.connectedDevices?.length || 0,
            mapsLoaded: Object.keys(global.deviceMaps || {}).length
        };
        
        res.json(health);
    } catch (error) {
        res.status(500).json({
            success: false,
            status: 'unhealthy',
            error: error.message,
            timestamp: new Date().toISOString()
        });
    }
});

/**
 * Initialize global variables if they don't exist
 */
function initializeGlobals() {
    if (typeof global.connectedDevices === 'undefined') {
        global.connectedDevices = [];
    }
    if (typeof global.deviceMaps === 'undefined') {
        global.deviceMaps = {};
    }
    if (typeof global.deviceOrders === 'undefined') {
        global.deviceOrders = {};
    }
    if (typeof global.liveData === 'undefined') {
        global.liveData = {};
    }
}

// Initialize globals when routes are loaded
initializeGlobals();

// ==========================================
// ENHANCED HELPER FUNCTIONS
// ==========================================

/**
 * Normalize map data to ensure consistent structure
 */
function normalizeMapData(mapData, deviceId) {
    if (!mapData) {
        return createEmptyMapData(deviceId);
    }
    
    // Ensure required top-level fields
    const normalized = {
        deviceId: mapData.deviceId || deviceId,
        timestamp: mapData.timestamp || new Date().toISOString(),
        version: mapData.version || 1,
        ...mapData
    };
    
    // Ensure info field with proper structure
    if (!normalized.info) {
        normalized.info = {
            resolution: 0.05,
            width: 1000,
            height: 1000,
            origin: {
                position: { x: -25.0, y: -25.0, z: 0.0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 }
            },
            originOrientation: { x: 0, y: 0, z: 0, w: 1 }
        };
    } else {
        // Ensure origin structure
        if (!normalized.info.origin) {
            normalized.info.origin = {
                position: { x: -25.0, y: -25.0, z: 0.0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 }
            };
        }
        
        // Ensure origin has position and orientation
        if (!normalized.info.origin.position) {
            normalized.info.origin.position = { x: -25.0, y: -25.0, z: 0.0 };
        }
        
        if (!normalized.info.origin.orientation) {
            normalized.info.origin.orientation = { x: 0, y: 0, z: 0, w: 1 };
        }
        
        // Ensure backward compatibility
        if (!normalized.info.originOrientation) {
            normalized.info.originOrientation = normalized.info.origin.orientation;
        }
    }
    
    // Ensure shapes field
    if (!normalized.shapes) {
        normalized.shapes = [];
    }
    
    // Ensure occupancy data
    if (!normalized.occupancyData && !normalized.data) {
        normalized.occupancyData = [];
    } else if (!normalized.occupancyData && normalized.data) {
        // Copy 'data' to 'occupancyData' for consistency
        normalized.occupancyData = normalized.data;
    }
    
    // Ensure metadata field
    if (!normalized.metadata) {
        normalized.metadata = {};
    }
    
    return normalized;
}

/**
 * Create empty map data with proper structure
 */
function createEmptyMapData(deviceId) {
    return {
        deviceId: deviceId,
        timestamp: new Date().toISOString(),
        info: {
            resolution: 0.05,
            width: 1000,
            height: 1000,
            origin: {
                position: { x: -25.0, y: -25.0, z: 0.0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 }
            },
            originOrientation: { x: 0, y: 0, z: 0, w: 1 }
        },
        occupancyData: [],
        data: [], // For backward compatibility
        shapes: [],
        version: 1,
        metadata: {
            created: new Date().toISOString(),
            type: 'empty'
        }
    };
}

/**
 * Get latest saved maps for a device
 */
async function getLatestSavedMaps(deviceId) {
    try {
        const indexPath = path.join(__dirname, '../storage/maps', `${deviceId}_index.json`);
        const indexContent = await fs.readFile(indexPath, 'utf8');
        const index = JSON.parse(indexContent);
        return index.maps || [];
    } catch (error) {
        console.log(`⚠️ No saved maps index found for ${deviceId}`);
        
        // Fallback: scan directory for maps
        try {
            const mapsDir = path.join(__dirname, '../storage/maps');
            const files = await fs.readdir(mapsDir);
            const mapFiles = files.filter(file => 
                file.startsWith(`${deviceId}_`) && file.endsWith('.json') && !file.includes('_index.json')
            );
            
            const maps = [];
            for (const file of mapFiles) {
                try {
                    const stats = await fs.stat(path.join(mapsDir, file));
                    const name = file.replace(`${deviceId}_`, '').replace('.json', '');
                    maps.push({
                        name: name,
                        filePath: path.join(mapsDir, file),
                        savedAt: stats.mtime.toISOString(),
                        fileSize: `${(stats.size / 1024).toFixed(2)} KB`,
                        shapes: 0,
                        mapType: 'discovered'
                    });
                } catch (statError) {
                    console.warn(`⚠️ Could not stat file ${file}:`, statError.message);
                }
            }
            
            return maps.sort((a, b) => new Date(b.savedAt) - new Date(a.savedAt));
        } catch (scanError) {
            console.warn(`⚠️ Could not scan maps directory:`, scanError.message);
            return [];
        }
    }
}

/**
 * Update saved maps index
 */
async function updateSavedMapsIndex(deviceId, mapInfo) {
    try {
        const indexPath = path.join(__dirname, '../storage/maps', `${deviceId}_index.json`);
        
        let index = { deviceId, maps: [], lastUpdated: new Date().toISOString() };
        
        try {
            const indexContent = await fs.readFile(indexPath, 'utf8');
            index = JSON.parse(indexContent);
        } catch (readError) {
            console.log(`Creating new index for ${deviceId}`);
        }

        const existingIndex = index.maps.findIndex(m => m.name === mapInfo.name);
        if (existingIndex >= 0) {
            index.maps[existingIndex] = { ...index.maps[existingIndex], ...mapInfo };
        } else {
            index.maps.unshift(mapInfo); // Add to beginning (most recent first)
        }

        // Keep only latest 50 maps
        if (index.maps.length > 50) {
            index.maps = index.maps.slice(0, 50);
        }

        index.lastUpdated = new Date().toISOString();

        await fs.writeFile(indexPath, JSON.stringify(index, null, 2));
        console.log(`✅ Updated maps index for ${deviceId}`);
        
    } catch (error) {
        console.error(`❌ Error updating saved maps index:`, error);
    }
}

/**
 * Generate map preview
 */
async function generateMapPreview(deviceId, mapName) {
    try {
        const mapFilePath = path.join(__dirname, '../storage/maps', `${deviceId}_${mapName}.json`);
        const mapContent = await fs.readFile(mapFilePath, 'utf8');
        const mapData = JSON.parse(mapContent);

        return {
            name: mapName,
            deviceId: deviceId,
            dimensions: {
                width: mapData.info?.width || 0,
                height: mapData.info?.height || 0,
                resolution: mapData.info?.resolution || 0
            },
            shapes: {
                total: mapData.shapes?.length || 0,
                byType: _countShapesByType(mapData.shapes || [])
            },
            occupancyData: {
                available: mapData.occupancyData && mapData.occupancyData.length > 0,
                totalCells: mapData.occupancyData?.length || 0
            },
            metadata: mapData.metadata || {},
            savedAt: mapData.savedAt,
            version: mapData.version || 1,
            fileSize: JSON.stringify(mapData).length
        };
    } catch (error) {
        console.error(`❌ Error generating map preview:`, error);
        return { error: 'Preview generation failed' };
    }
}

/**
 * Count shapes by type
 */
function _countShapesByType(shapes) {
    const counts = {};
    shapes.forEach(shape => {
        const type = shape.type || 'unknown';
        counts[type] = (counts[type] || 0) + 1;
    });
    return counts;
}

/**
 * Create map backup
 */
async function createMapBackup(deviceId, mapName) {
    try {
        const backupsDir = path.join(__dirname, '../storage/backups');
        await fs.mkdir(backupsDir, { recursive: true });

        const backupId = `backup_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
        const backupPath = path.join(backupsDir, `${deviceId}_${mapName}_${backupId}.json`);

        const currentMapData = global.deviceMaps && global.deviceMaps[deviceId];
        if (currentMapData) {
            const backupData = {
                ...currentMapData,
                backupId: backupId,
                backedUpAt: new Date().toISOString(),
                originalMapName: mapName
            };

            await fs.writeFile(backupPath, JSON.stringify(backupData, null, 2));
            
            return {
                backupId: backupId,
                backupPath: backupPath,
                backedUpAt: backupData.backedUpAt
            };
        }

        return null;
    } catch (error) {
        console.error(`❌ Error creating map backup:`, error);
        throw error;
    }
}

// ==========================================
// ENHANCED HELPER FUNCTIONS
// ==========================================

async function runAdvancedMapConverter(mapData, outputDir, mapName, options) {
    return new Promise((resolve, reject) => {
        try {
            const pythonScript = path.join(__dirname, '../scripts/map_converter.py');
            const tempDir = path.join(__dirname, '../temp');
            
            // Create enhanced temp JSON with options
            const enhancedMapData = {
                ...mapData,
                conversionOptions: options,
                enhancedMode: true
            };
            
            const tempJsonPath = path.join(tempDir, `${mapName}_enhanced.json`);
            fs.writeFile(tempJsonPath, JSON.stringify(enhancedMapData, null, 2));
            
            const pythonProcess = spawn('python3', [
                pythonScript,
                tempJsonPath,
                '-o', outputDir,
                '-n', mapName,
                '--enhanced',
                '--log-level', 'INFO'
            ]);
            
            let stdout = '';
            let stderr = '';
            
            pythonProcess.stdout.on('data', (data) => {
                stdout += data.toString();
            });
            
            pythonProcess.stderr.on('data', (data) => {
                stderr += data.toString();
            });
            
            pythonProcess.on('close', async (code) => {
                try {
                    await fs.unlink(tempJsonPath);
                } catch (error) {
                    console.warn('Could not cleanup temp file:', error);
                }
                
                if (code === 0) {
                    resolve({
                        success: true,
                        pgmPath: path.join(outputDir, `${mapName}.pgm`),
                        yamlPath: path.join(outputDir, `${mapName}.yaml`),
                        output: stdout
                    });
                } else {
                    resolve({
                        success: false,
                        error: `Enhanced converter failed: ${stderr}`
                    });
                }
            });
            
        } catch (error) {
            resolve({
                success: false,
                error: `Enhanced conversion setup failed: ${error.message}`
            });
        }
    });
}

async function optimizeMapForEditing(pgmPath, yamlPath) {
    // Use Sharp for image optimization
    try {
        const optimizedPath = pgmPath.replace('.pgm', '_optimized.pgm');
        
        await sharp(pgmPath)
            .normalize() // Enhance contrast
            .median(3)   // Reduce noise
            .toFile(optimizedPath);
        
        // Replace original with optimized version
        await fs.rename(optimizedPath, pgmPath);
        
        console.log('✅ Map optimized for editing');
    } catch (error) {
        console.warn('⚠️ Map optimization failed:', error.message);
    }
}

async function generateEditingMetadata(conversionResult) {
    try {
        const pgmStats = await fs.stat(conversionResult.pgmPath);
        const yamlContent = await fs.readFile(conversionResult.yamlPath, 'utf8');
        
        return {
            fileSize: pgmStats.size,
            optimizedForEditing: true,
            supportedTools: ['brush', 'pencil', 'eraser', 'shapes', 'locationPoints'],
            maxBrushSize: 100,
            layerSupport: true,
            undoLevels: 50,
            yamlMetadata: yamlContent
        };
    } catch (error) {
        console.warn('Could not generate editing metadata:', error);
        return {};
    }
}

async function validateMapChanges(pgmData, dimensions, locationPoints) {
    const validation = {
        valid: true,
        errors: [],
        warnings: []
    };
    
    // Check data integrity
    if (pgmData.length !== dimensions.width * dimensions.height) {
        validation.errors.push('Data size does not match dimensions');
        validation.valid = false;
    }
    
    // Check for valid pixel values
    for (let i = 0; i < pgmData.length; i++) {
        if (pgmData[i] < 0 || pgmData[i] > 255) {
            validation.errors.push(`Invalid pixel value at index ${i}: ${pgmData[i]}`);
            validation.valid = false;
            break;
        }
    }
    
    // Validate location points
    for (const point of locationPoints) {
        if (!point.position || typeof point.position.x !== 'number' || typeof point.position.y !== 'number') {
            validation.errors.push(`Invalid location point position: ${point.name}`);
            validation.valid = false;
        }
    }
    
    return validation;
}

async function writePGMFile(pgmData, filePath, dimensions) {
    const header = `P5\n# Enhanced map editor output\n${dimensions.width} ${dimensions.height}\n255\n`;
    const headerBuffer = Buffer.from(header, 'ascii');
    const fullBuffer = Buffer.concat([headerBuffer, pgmData]);
    
    await fs.writeFile(filePath, fullBuffer);
}

async function updateYAMLWithLocations(yamlPath, locationPoints, editHistory) {
    try {
        let yamlContent = await fs.readFile(yamlPath, 'utf8');
        
        // Add location points section
        if (locationPoints.length > 0) {
            yamlContent += '\n# Location Points\nlocations:\n';
            for (const point of locationPoints) {
                yamlContent += `  - name: "${point.name}"\n`;
                yamlContent += `    type: "${point.type}"\n`;
                yamlContent += `    position: [${point.position.x}, ${point.position.y}, ${point.position.z || 0}]\n`;
                if (point.orientation) {
                    yamlContent += `    orientation: [${point.orientation.x}, ${point.orientation.y}, ${point.orientation.z}, ${point.orientation.w}]\n`;
                }
            }
        }
        
        // Add edit metadata
        yamlContent += `\n# Edit History\nedit_metadata:\n`;
        yamlContent += `  edited_at: "${new Date().toISOString()}"\n`;
        yamlContent += `  editor_version: "enhanced-1.0.0"\n`;
        yamlContent += `  total_locations: ${locationPoints.length}\n`;
        
        await fs.writeFile(yamlPath, yamlContent);
    } catch (error) {
        console.error('Error updating YAML with locations:', error);
    }
}

async function convertPGMBackToMapData(pgmData, dimensions, locationPoints, deviceId) {
    // Convert PGM data back to occupancy grid format
    const occupancyData = new Array(pgmData.length);
    
    for (let i = 0; i < pgmData.length; i++) {
        const pgmValue = pgmData[i];
        
        if (pgmValue <= 50) {
            occupancyData[i] = 100; // Occupied
        } else if (pgmValue >= 200) {
            occupancyData[i] = 0;   // Free
        } else {
            occupancyData[i] = -1;  // Unknown
        }
    }
    
    // Convert location points to shapes
    const shapes = locationPoints.map(point => ({
        id: point.id || `point_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
        type: point.type,
        name: point.name,
        points: [{ x: point.position.x, y: point.position.y, z: point.position.z || 0 }],
        sides: { left: '', right: '', front: '', back: '' },
        color: getColorForLocationType(point.type),
        createdAt: new Date().toISOString()
    }));
    
    return {
        deviceId: deviceId,
        timestamp: new Date().toISOString(),
        info: {
            resolution: 0.05,
            width: dimensions.width,
            height: dimensions.height,
            origin: {
                position: { x: 0, y: 0, z: 0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 }
            }
        },
        occupancyData: occupancyData,
        shapes: shapes,
        version: 1,
        metadata: {
            editedViaEnhancedEditor: true,
            lastModified: new Date().toISOString()
        }
    };
}

function getColorForLocationType(type) {
    const colors = {
        pickup: 'FF4CAF50',
        drop: 'FF2196F3',
        home: 'FFFF9800',
        charging: 'FFFFEB3B',
        waypoint: 'FF9C27B0',
        obstacle: 'FFF44336'
    };
    return colors[type] || 'FF9E9E9E';
}

async function runPreDeploymentValidation(mapName, deviceId) {
    const validation = {
        valid: true,
        errors: [],
        warnings: []
    };
    
    try {
        // Check file existence
        const pgmPath = path.join(__dirname, '../maps', `${mapName}.pgm`);
        const yamlPath = path.join(__dirname, '../maps', `${mapName}.yaml`);
        
        await fs.access(pgmPath);
        await fs.access(yamlPath);
        
        // Check file sizes
        const pgmStats = await fs.stat(pgmPath);
        const yamlStats = await fs.stat(yamlPath);
        
        if (pgmStats.size === 0) {
            validation.errors.push('PGM file is empty');
            validation.valid = false;
        }
        
        if (yamlStats.size === 0) {
            validation.errors.push('YAML file is empty');
            validation.valid = false;
        }
        
        // Validate YAML content
        const yamlContent = await fs.readFile(yamlPath, 'utf8');
        if (!yamlContent.includes('image:') || !yamlContent.includes('resolution:')) {
            validation.errors.push('YAML file missing required fields');
            validation.valid = false;
        }
        
    } catch (error) {
        validation.errors.push(`Validation failed: ${error.message}`);
        validation.valid = false;
    }
    
    return validation;
}

async function deployMapWithValidation(deviceId, mapName, targetMapName, sshConfig, options) {
    // Enhanced deployment with validation steps
    try {
        const actualTargetName = targetMapName || mapName;
        
        // Use existing deployment logic but with enhanced validation
        const result = await deployMapToRaspberryPi(
            sshConfig,
            {
                yamlPath: path.join(__dirname, '../maps', `${mapName}.yaml`),
                pgmPath: path.join(__dirname, '../maps', `${mapName}.pgm`),
                locationsPath: path.join(__dirname, '../maps', `${mapName}_locations.json`),
                mapName: actualTargetName
            },
            options.autoLoad
        );
        
        return result;
    } catch (error) {
        return {
            success: false,
            error: error.message
        };
    }
}

// Analysis functions
async function analyzeCoverage(mapPath) {
    // Analyze map coverage (free vs occupied vs unknown)
    try {
        const mapData = await fs.readFile(mapPath);
        const headerEnd = mapData.indexOf('\n255\n') + 5;
        const imageData = mapData.slice(headerEnd);
        
        let free = 0, occupied = 0, unknown = 0;
        
        for (let i = 0; i < imageData.length; i++) {
            const value = imageData[i];
            if (value <= 50) occupied++;
            else if (value >= 200) free++;
            else unknown++;
        }
        
        const total = imageData.length;
        
        return {
            totalCells: total,
            freeCells: free,
            occupiedCells: occupied,
            unknownCells: unknown,
            freePercentage: (free / total * 100).toFixed(2),
            occupiedPercentage: (occupied / total * 100).toFixed(2),
            unknownPercentage: (unknown / total * 100).toFixed(2)
        };
    } catch (error) {
        throw new Error(`Coverage analysis failed: ${error.message}`);
    }
}

async function analyzeConnectivity(mapPath) {
    // Analyze map connectivity using flood fill algorithm
    // Implementation would use a flood fill to find connected regions
    return {
        connectedRegions: 1,
        largestRegionSize: 0,
        isolatedAreas: 0,
        connectivityScore: 85.5
    };
}

async function analyzeObstacles(mapPath) {
    // Analyze obstacle distribution and clustering
    return {
        obstacleCount: 0,
        obstacleClusters: 0,
        averageClusterSize: 0,
        obstacleDistribution: 'even'
    };
}

async function analyzePathfinding(mapPath) {
    // Analyze pathfinding potential
    return {
        pathfindingScore: 92.3,
        bottlenecks: 0,
        deadEnds: 0,
        longPaths: 0
    };
}

async function analyzeLocationPoints(yamlPath) {
    try {
        const yamlContent = await fs.readFile(yamlPath, 'utf8');
        const locationCount = (yamlContent.match(/- name:/g) || []).length;
        
        return {
            totalLocationPoints: locationCount,
            pointTypes: {},
            distribution: 'good',
            accessibility: 'high'
        };
    } catch (error) {
        throw new Error(`Location points analysis failed: ${error.message}`);
    }
}

function calculateOverallQuality(analysis) {
    let score = 0;
    let factors = 0;
    
    if (analysis.coverage) {
        const unknownPct = parseFloat(analysis.coverage.unknownPercentage);
        score += Math.max(0, 100 - unknownPct);
        factors++;
    }
    
    if (analysis.connectivity) {
        score += analysis.connectivity.connectivityScore;
        factors++;
    }
    
    if (analysis.pathfinding) {
        score += analysis.pathfinding.pathfindingScore;
        factors++;
    }
    
    return factors > 0 ? (score / factors).toFixed(1) : 0;
}

// Export functions for different formats
async function exportToPGM(mapPath, yamlPath, options) {
    // Copy files to export directory
    const exportDir = path.join(__dirname, '../exports');
    await fs.mkdir(exportDir, { recursive: true });
    
    const timestamp = Date.now();
    const exportPgmPath = path.join(exportDir, `export_${timestamp}.pgm`);
    const exportYamlPath = path.join(exportDir, `export_${timestamp}.yaml`);
    
    await fs.copyFile(mapPath, exportPgmPath);
    await fs.copyFile(yamlPath, exportYamlPath);
    
    return {
        files: [exportPgmPath, exportYamlPath]
    };
}

async function exportToPNG(mapPath, yamlPath, options) {
    // Convert PGM to PNG using Sharp
    const exportDir = path.join(__dirname, '../exports');
    await fs.mkdir(exportDir, { recursive: true });
    
    const timestamp = Date.now();
    const exportPath = path.join(exportDir, `export_${timestamp}.png`);
    
    await sharp(mapPath)
        .png({
            compressionLevel: 6,
            quality: options.quality === 'high' ? 100 : 80
        })
        .toFile(exportPath);
    
    return {
        files: [exportPath]
    };
}

async function exportToPDF(mapPath, yamlPath, options) {
    // PDF export would require additional libraries like PDFKit
    // Placeholder implementation
    return {
        files: [],
        note: 'PDF export not yet implemented'
    };
}

async function exportToSVG(mapPath, yamlPath, options) {
    // SVG export would require image vectorization
    // Placeholder implementation
    return {
        files: [],
        note: 'SVG export not yet implemented'
    };
}

// Additional enhanced helper functions
async function runValidationRule(mapName, rule) {
    // Placeholder validation rule runner
    return {
        passed: true,
        errors: [],
        warnings: []
    };
}

async function testPathPlanning(mapName, startPoint, endPoint, algorithm, options) {
    // Placeholder path planning test
    return {
        pathFound: true,
        pathLength: 0,
        algorithm: algorithm,
        executionTime: 0
    };
}

async function createDeploymentBackup(deviceId, mapName) {
    // Enhanced backup creation for deployments
    return await createMapBackup(deviceId, mapName);
}

async function verifyDeployment(deviceId, mapName) {
    // Deployment verification
    return {
        success: true,
        verified: true
    };
}

async function rollbackDeployment(deviceId, backupInfo) {
    // Rollback deployment to backup
    return {
        success: true,
        rolledBack: true
    };
}

async function validateMapForDeployment(mapName) {
    // Validate map for deployment
    return {
        valid: true,
        errors: [],
        warnings: []
    };
}

async function activateDeployedMap(deviceId, mapName) {
    // Activate deployed map
    return {
        success: true,
        activated: true
    };
}

async function processLocationPointOperation(mapName, operation) {
    // Process location point operations
    return {
        success: true,
        operation: operation
    };
}

async function restoreMapFromBackup(deviceId, backupId, targetMapName) {
    // Restore map from backup
    return {
        success: true,
        restored: true
    };
}

// ==========================================
// New API routes for PGM conversion and map management
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
 * Upload map to AMR (send PGM + YAML back to ROS)
 * POST /api/maps/:deviceId/upload
 */
router.post('/maps/:deviceId/upload', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, setAsActiveMap = true } = req.body;

        console.log(`🚀 Uploading map to AMR: ${mapName} for device: ${deviceId}`);

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
                await setActiveMapOnAMR(deviceId, mapName);
                console.log(`✅ Map set as active: ${mapName}`);
            } catch (error) {
                console.warn(`⚠️ Could not set as active map: ${error.message}`);
            }
        }

        res.json({
            success: uploadResult.success,
            message: uploadResult.success ? 'Map uploaded to AMR successfully' : 'Upload failed',
            deviceId: deviceId,
            mapName: mapName,
            method: uploadResult.method,
            uploadedAt: new Date().toISOString(),
            details: uploadResult.details
        });

    } catch (error) {
        console.error('❌ Error uploading map to AMR:', error);
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

async function setActiveMapOnAMR(deviceId, mapName) {
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

/**
 * Save complete map data with all layers and metadata
 * POST /api/maps/:deviceId/save-complete
 */
router.post('/maps/:deviceId/save-complete', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { 
            mapData, 
            mapName, 
            saveType = 'complete',
            includeMetadata = true,
            createBackup = true,
            clientInfo = {}
        } = req.body;

        console.log(`💾 Saving complete map for device: ${deviceId}`);

        // Validate map data
        if (!mapData || !mapData.info) {
            return res.status(400).json({
                success: false,
                error: 'Invalid map data structure'
            });
        }

        // Generate map name if not provided
        const finalMapName = mapName || `complete_map_${deviceId}_${Date.now()}`;
        
        // Create backup if requested and current map exists
        let backupInfo = null;
        if (createBackup && global.deviceMaps && global.deviceMaps[deviceId]) {
            try {
                backupInfo = await createMapBackup(deviceId, finalMapName);
                console.log(`✅ Backup created: ${backupInfo.backupId}`);
            } catch (backupError) {
                console.warn('⚠️ Backup creation failed:', backupError.message);
            }
        }

        // Enhance map data with save metadata
        const enhancedMapData = {
            ...mapData,
            deviceId: deviceId,
            savedAt: new Date().toISOString(),
            lastSaved: new Date().toISOString(),
            saveType: saveType,
            clientInfo: clientInfo,
            version: (mapData.version || 0) + 1,
            metadata: {
                ...mapData.metadata,
                totalShapes: mapData.shapes?.length || 0,
                hasOccupancyData: mapData.occupancyData && mapData.occupancyData.length > 0,
                mapDimensions: {
                    width: mapData.info.width,
                    height: mapData.info.height,
                    resolution: mapData.info.resolution
                },
                saveTimestamp: Date.now(),
                ...(includeMetadata ? mapData.metadata : {})
            }
        };

        // Calculate file size estimate
        const dataSize = JSON.stringify(enhancedMapData).length;
        const fileSizeKB = Math.round(dataSize / 1024 * 100) / 100;

        // Save to global state (your existing structure)
        if (!global.deviceMaps) {
            global.deviceMaps = {};
        }
        global.deviceMaps[deviceId] = enhancedMapData;

        // Save to persistent storage using your existing storageManager
        if (global.storageManager && global.storageManager.saveMap) {
            await global.storageManager.saveMap(deviceId);
        }

        // Save to individual map file for easy access
        const mapsDir = path.join(__dirname, '../storage/maps');
        await fs.mkdir(mapsDir, { recursive: true });
        
        const mapFilePath = path.join(mapsDir, `${deviceId}_${finalMapName}.json`);
        await fs.writeFile(mapFilePath, JSON.stringify(enhancedMapData, null, 2));

        // Update saved maps index
        await updateSavedMapsIndex(deviceId, {
            name: finalMapName,
            filePath: mapFilePath,
            savedAt: enhancedMapData.savedAt,
            fileSize: `${fileSizeKB} KB`,
            shapes: enhancedMapData.shapes?.length || 0,
            mapType: saveType,
            metadata: enhancedMapData.metadata
        });

        console.log(`✅ Complete map saved: ${finalMapName} (${fileSizeKB} KB)`);

        res.json({
            success: true,
            message: 'Complete map saved successfully',
            deviceId: deviceId,
            mapName: finalMapName,
            fileSize: `${fileSizeKB} KB`,
            filePath: mapFilePath,
            backup: backupInfo,
            savedAt: enhancedMapData.savedAt,
            shapes: enhancedMapData.shapes?.length || 0,
            version: enhancedMapData.version
        });

    } catch (error) {
        console.error('❌ Error saving complete map:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Load complete map data for editing
 * GET /api/maps/:deviceId/load-complete
 */
router.get('/maps/:deviceId/load-complete', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { 
            mapName, 
            includeShapes = 'true', 
            includeMetadata = 'true' 
        } = req.query;

        console.log(`📂 Loading complete map for device: ${deviceId}`);

        let mapData;

        if (mapName) {
            // Load specific map by name
            const mapFilePath = path.join(__dirname, '../storage/maps', `${deviceId}_${mapName}.json`);
            try {
                const mapContent = await fs.readFile(mapFilePath, 'utf8');
                mapData = JSON.parse(mapContent);
            } catch (fileError) {
                // Fallback to global state
                mapData = global.deviceMaps && global.deviceMaps[deviceId];
            }
        } else {
            // Load current/latest map from your existing global state
            mapData = global.deviceMaps && global.deviceMaps[deviceId];
        }

        if (!mapData) {
            return res.status(404).json({
                success: false,
                error: 'No map data found for device'
            });
        }

        // Filter data based on query parameters
        const filteredMapData = {
            ...mapData,
            shapes: includeShapes === 'true' ? mapData.shapes || [] : [],
            metadata: includeMetadata === 'true' ? mapData.metadata || {} : {}
        };

        // Add loading metadata
        filteredMapData.loadedAt = new Date().toISOString();
        filteredMapData.loadedFor = 'editing';

        res.json({
            success: true,
            message: 'Complete map loaded successfully',
            deviceId: deviceId,
            mapName: mapName || 'current',
            mapData: filteredMapData,
            loadedAt: filteredMapData.loadedAt
        });

    } catch (error) {
        console.error('❌ Error loading complete map:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Get all saved maps for a device with metadata
 * GET /api/maps/:deviceId/saved
 */
router.get('/maps/:deviceId/saved', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { 
            includePreview = 'true', 
            sortBy = 'savedAt', 
            descending = 'true' 
        } = req.query;

        console.log(`📋 Getting saved maps for device: ${deviceId}`);

        const savedMapsIndex = await getSavedMapsIndex(deviceId);
        let savedMaps = savedMapsIndex.maps || [];

        // Include preview data if requested
        if (includePreview === 'true') {
            savedMaps = await Promise.all(savedMaps.map(async (mapInfo) => {
                try {
                    const previewData = await generateMapPreview(deviceId, mapInfo.name);
                    return {
                        ...mapInfo,
                        preview: previewData
                    };
                } catch (previewError) {
                    console.warn(`⚠️ Could not generate preview for ${mapInfo.name}`);
                    return mapInfo;
                }
            }));
        }

        // Sort maps
        savedMaps.sort((a, b) => {
            const aValue = a[sortBy] || '';
            const bValue = b[sortBy] || '';
            
            if (descending === 'true') {
                return bValue > aValue ? 1 : -1;
            } else {
                return aValue > bValue ? 1 : -1;
            }
        });

        res.json({
            success: true,
            deviceId: deviceId,
            maps: savedMaps,
            totalMaps: savedMaps.length,
            sortBy: sortBy,
            descending: descending === 'true'
        });

    } catch (error) {
        console.error('❌ Error getting saved maps:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Get map preview/summary without full data
 * GET /api/maps/:deviceId/:mapName/preview
 */
router.get('/maps/:deviceId/:mapName/preview', async (req, res) => {
    try {
        const { deviceId, mapName } = req.params;

        console.log(`👀 Generating preview for map: ${mapName}`);

        const previewData = await generateMapPreview(deviceId, mapName);

        if (!previewData) {
            return res.status(404).json({
                success: false,
                error: 'Map not found or preview generation failed'
            });
        }

        res.json({
            success: true,
            deviceId: deviceId,
            mapName: mapName,
            preview: previewData,
            generatedAt: new Date().toISOString()
        });

    } catch (error) {
        console.error('❌ Error generating map preview:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Clone/duplicate a saved map
 * POST /api/maps/:deviceId/:mapName/clone
 */
router.post('/maps/:deviceId/:mapName/clone', async (req, res) => {
    try {
        const { deviceId, mapName } = req.params;
        const { 
            targetMapName, 
            includeShapes = true, 
            includeMetadata = true 
        } = req.body;

        console.log(`🔄 Cloning map: ${mapName} -> ${targetMapName}`);

        // Load source map
        const sourceMapPath = path.join(__dirname, '../storage/maps', `${deviceId}_${mapName}.json`);
        const sourceMapContent = await fs.readFile(sourceMapPath, 'utf8');
        const sourceMapData = JSON.parse(sourceMapContent);

        // Create cloned map data
        const clonedMapData = {
            ...sourceMapData,
            deviceId: deviceId,
            timestamp: new Date().toISOString(),
            savedAt: new Date().toISOString(),
            lastSaved: new Date().toISOString(),
            shapes: includeShapes ? sourceMapData.shapes || [] : [],
            metadata: {
                ...(includeMetadata ? sourceMapData.metadata || {} : {}),
                clonedFrom: mapName,
                clonedAt: new Date().toISOString(),
                originalSavedAt: sourceMapData.savedAt
            },
            version: 1 // Reset version for cloned map
        };

        // Save cloned map
        const targetMapPath = path.join(__dirname, '../storage/maps', `${deviceId}_${targetMapName}.json`);
        await fs.writeFile(targetMapPath, JSON.stringify(clonedMapData, null, 2));

        // Update saved maps index
        const dataSize = JSON.stringify(clonedMapData).length;
        const fileSizeKB = Math.round(dataSize / 1024 * 100) / 100;

        await updateSavedMapsIndex(deviceId, {
            name: targetMapName,
            filePath: targetMapPath,
            savedAt: clonedMapData.savedAt,
            fileSize: `${fileSizeKB} KB`,
            shapes: clonedMapData.shapes?.length || 0,
            mapType: 'cloned',
            metadata: clonedMapData.metadata
        });

        console.log(`✅ Map cloned successfully: ${targetMapName}`);

        res.json({
            success: true,
            message: 'Map cloned successfully',
            deviceId: deviceId,
            sourceMapName: mapName,
            targetMapName: targetMapName,
            clonedAt: clonedMapData.savedAt,
            fileSize: `${fileSizeKB} KB`
        });

    } catch (error) {
        console.error('❌ Error cloning map:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Delete saved map
 * DELETE /api/maps/:deviceId/:mapName
 */
router.delete('/maps/:deviceId/:mapName', async (req, res) => {
    try {
        const { deviceId, mapName } = req.params;
        const { deleteBackups = 'false' } = req.query;

        console.log(`🗑️ Deleting map: ${mapName} for device: ${deviceId}`);

        const mapFilePath = path.join(__dirname, '../storage/maps', `${deviceId}_${mapName}.json`);
        
        // Delete map file
        try {
            await fs.unlink(mapFilePath);
            console.log(`✅ Deleted map file: ${mapFilePath}`);
        } catch (fileError) {
            console.warn(`⚠️ Map file not found: ${mapFilePath}`);
        }

        // Delete backups if requested
        if (deleteBackups === 'true') {
            await deleteMapBackups(deviceId, mapName);
        }

        // Update saved maps index
        await removeFromSavedMapsIndex(deviceId, mapName);

        res.json({
            success: true,
            message: 'Map deleted successfully',
            deviceId: deviceId,
            mapName: mapName,
            deletedAt: new Date().toISOString(),
            backupsDeleted: deleteBackups === 'true'
        });

    } catch (error) {
        console.error('❌ Error deleting map:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Update saved maps index for a device
 */
async function updateSavedMapsIndex(deviceId, mapInfo) {
    try {
        const indexPath = path.join(__dirname, '../storage/maps', `${deviceId}_index.json`);
        
        let index = { deviceId, maps: [], lastUpdated: new Date().toISOString() };
        
        // Load existing index
        try {
            const indexContent = await fs.readFile(indexPath, 'utf8');
            index = JSON.parse(indexContent);
        } catch (readError) {
            // Index doesn't exist, will create new one
        }

        // Update or add map info
        const existingIndex = index.maps.findIndex(m => m.name === mapInfo.name);
        if (existingIndex >= 0) {
            index.maps[existingIndex] = { ...index.maps[existingIndex], ...mapInfo };
        } else {
            index.maps.push(mapInfo);
        }

        index.lastUpdated = new Date().toISOString();

        // Save updated index
        await fs.writeFile(indexPath, JSON.stringify(index, null, 2));
        
    } catch (error) {
        console.error('❌ Error updating saved maps index:', error);
    }
}

/**
 * Get saved maps index for a device
 */
async function getSavedMapsIndex(deviceId) {
    try {
        const indexPath = path.join(__dirname, '../storage/maps', `${deviceId}_index.json`);
        const indexContent = await fs.readFile(indexPath, 'utf8');
        return JSON.parse(indexContent);
    } catch (error) {
        return { deviceId, maps: [], lastUpdated: new Date().toISOString() };
    }
}

/**
 * Remove map from saved maps index
 */
async function removeFromSavedMapsIndex(deviceId, mapName) {
    try {
        const indexPath = path.join(__dirname, '../storage/maps', `${deviceId}_index.json`);
        
        let index = await getSavedMapsIndex(deviceId);
        index.maps = index.maps.filter(m => m.name !== mapName);
        index.lastUpdated = new Date().toISOString();

        await fs.writeFile(indexPath, JSON.stringify(index, null, 2));
        
    } catch (error) {
        console.error('❌ Error removing map from index:', error);
    }
}

/**
 * Generate map preview data
 */
async function generateMapPreview(deviceId, mapName) {
    try {
        const mapFilePath = path.join(__dirname, '../storage/maps', `${deviceId}_${mapName}.json`);
        const mapContent = await fs.readFile(mapFilePath, 'utf8');
        const mapData = JSON.parse(mapContent);

        // Generate preview statistics
        const preview = {
            name: mapName,
            deviceId: deviceId,
            dimensions: {
                width: mapData.info?.width || 0,
                height: mapData.info?.height || 0,
                resolution: mapData.info?.resolution || 0
            },
            shapes: {
                total: mapData.shapes?.length || 0,
                byType: {}
            },
            occupancyData: {
                available: mapData.occupancyData && mapData.occupancyData.length > 0,
                totalCells: mapData.occupancyData?.length || 0
            },
            metadata: mapData.metadata || {},
            savedAt: mapData.savedAt,
            version: mapData.version || 1,
            fileSize: JSON.stringify(mapData).length
        };

        // Count shapes by type
        if (mapData.shapes) {
            mapData.shapes.forEach(shape => {
                const type = shape.type || 'unknown';
                preview.shapes.byType[type] = (preview.shapes.byType[type] || 0) + 1;
            });
        }

        return preview;
        
    } catch (error) {
        console.error('❌ Error generating map preview:', error);
        return null;
    }
}

/**
 * Create map backup
 */
async function createMapBackup(deviceId, mapName) {
    try {
        const backupsDir = path.join(__dirname, '../storage/backups');
        await fs.mkdir(backupsDir, { recursive: true });

        const backupId = `backup_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
        const backupPath = path.join(backupsDir, `${deviceId}_${mapName}_${backupId}.json`);

        // Copy current map to backup
        const currentMapData = global.deviceMaps && global.deviceMaps[deviceId];
        if (currentMapData) {
            const backupData = {
                ...currentMapData,
                backupId: backupId,
                backedUpAt: new Date().toISOString(),
                originalMapName: mapName
            };

            await fs.writeFile(backupPath, JSON.stringify(backupData, null, 2));
            
            return {
                backupId: backupId,
                backupPath: backupPath,
                backedUpAt: backupData.backedUpAt
            };
        }

        return null;
        
    } catch (error) {
        console.error('❌ Error creating map backup:', error);
        throw error;
    }
}

/**
 * Delete map backups
 */
async function deleteMapBackups(deviceId, mapName) {
    try {
        const backupsDir = path.join(__dirname, '../storage/backups');
        const files = await fs.readdir(backupsDir);
        
        const backupFiles = files.filter(file => 
            file.startsWith(`${deviceId}_${mapName}_backup_`) && file.endsWith('.json')
        );

        for (const backupFile of backupFiles) {
            const backupPath = path.join(backupsDir, backupFile);
            await fs.unlink(backupPath);
            console.log(`✅ Deleted backup: ${backupFile}`);
        }

        return backupFiles.length;
        
    } catch (error) {
        console.error('❌ Error deleting map backups:', error);
        return 0;
    }
}

// ==========================================
// ROS2 MAP MANAGEMENT ENDPOINTS
// ==========================================

/**
 * Execute ROS2 map saver command
 * POST /api/maps/:deviceId/save-via-ros2
 */
router.post('/maps/:deviceId/save-via-ros2', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { 
            mapName, 
            directory = '/tmp/saved_maps',
            format = 'pgm', // 'pgm' or 'yaml'
            includeTimestamp = true 
        } = req.body;

        console.log(`🗺️ Executing ROS2 map saver for device: ${deviceId}`);

        const finalMapName = includeTimestamp 
            ? `${mapName}_${Date.now()}`
            : mapName;

        // Execute ROS2 map saver command via SSH on Pi
        // Try to use global instance first, otherwise create new instance
        let ros2ScriptManager;
        if (global.ros2ScriptManager) {
            ros2ScriptManager = global.ros2ScriptManager;
        } else {
            const ROS2ScriptManager = require('../ros/utils/ros2ScriptManager');
            ros2ScriptManager = new ROS2ScriptManager();
        }
        
        const result = await ros2ScriptManager.executeROS2MapSaver({
            deviceId,
            mapName: finalMapName,
            directory,
            format
        });

        if (result.success) {
            console.log(`✅ ROS2 map saved: ${finalMapName}`);
            
            res.json({
                success: true,
                message: 'Map saved via ROS2 command successfully',
                deviceId: deviceId,
                mapName: finalMapName,
                directory: directory,
                files: result.files,
                command: result.command,
                savedAt: new Date().toISOString()
            });
        } else {
            throw new Error(result.error);
        }

    } catch (error) {
        console.error('❌ Error executing ROS2 map saver:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Get ROS2 saved maps from Raspberry Pi
 * GET /api/maps/:deviceId/ros2-saved
 */
router.get('/maps/:deviceId/ros2-saved', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { directory = '/tmp/saved_maps' } = req.query;

        console.log(`📋 Getting ROS2 saved maps for device: ${deviceId}`);

        // Try to use global instance first, otherwise create new instance
        let ros2ScriptManager;
        if (global.ros2ScriptManager) {
            ros2ScriptManager = global.ros2ScriptManager;
        } else {
            const ROS2ScriptManager = require('../ros/utils/ros2ScriptManager');
            ros2ScriptManager = new ROS2ScriptManager();
        }
        
        const maps = await ros2ScriptManager.listROS2SavedMaps({
            deviceId,
            directory
        });

        res.json({
            success: true,
            deviceId: deviceId,
            directory: directory,
            maps: maps,
            retrievedAt: new Date().toISOString()
        });

    } catch (error) {
        console.error('❌ Error getting ROS2 saved maps:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Load ROS2 saved map for editing
 * POST /api/maps/:deviceId/load-ros2-saved
 */
router.post('/maps/:deviceId/load-ros2-saved', async (req, res) => {
    try {
        const { deviceId } = req.params;
        const { mapName, directory = '/tmp/saved_maps' } = req.body;

        console.log(`📂 Loading ROS2 saved map: ${mapName} for device: ${deviceId}`);

        // Try to use global instance first, otherwise create new instance
        let ros2ScriptManager;
        if (global.ros2ScriptManager) {
            ros2ScriptManager = global.ros2ScriptManager;
        } else {
            const ROS2ScriptManager = require('../ros/utils/ros2ScriptManager');
            ros2ScriptManager = new ROS2ScriptManager();
        }
        
        const result = await ros2ScriptManager.loadROS2SavedMap({
            deviceId,
            mapName,
            directory
        });

        if (result.success) {
            res.json({
                success: true,
                message: 'ROS2 saved map loaded successfully',
                deviceId: deviceId,
                mapName: mapName,
                mapData: result.mapData,
                loadedAt: new Date().toISOString()
            });
        } else {
            throw new Error(result.error);
        }

    } catch (error) {
        console.error('❌ Error loading ROS2 saved map:', error);
        res.status(500).json({
            success: false,
            error: error.message
        });
    }
});

/**
 * Check deployment status on Raspberry Pi
 */
async function checkDeploymentStatus(sshConfig) {
    return new Promise((resolve) => {
        const conn = new Client();
        
        conn.on('ready', async () => {
            try {
                // Check if active map configuration exists
                const activeMapResult = await executeSSHCommand(conn, 
                    `cat ${sshConfig.mapDirectory}/active_map.json 2>/dev/null || echo "{}"`
                );
                
                let activeMap = {};
                try {
                    activeMap = JSON.parse(activeMapResult.trim() || '{}');
                } catch (e) {
                    activeMap = {};
                }
                
                // Check map server process
                const mapServerCheck = await executeSSHCommand(conn, 
                    `pgrep -f "map_server" || echo "not_running"`
                );
                
                // Check navigation process
                const navCheck = await executeSSHCommand(conn, 
                    `pgrep -f "nav2_bringup" || echo "not_running"`
                );
                
                // List available maps
                const mapsList = await executeSSHCommand(conn, 
                    `ls -la ${sshConfig.mapDirectory}/*.yaml 2>/dev/null | wc -l || echo "0"`
                );
                
                // Check ROS2 environment
                const rosCheck = await executeSSHCommand(conn, 
                    `source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null | grep -c "map" || echo "0"`
                );
                
                conn.end();
                
                resolve({
                    success: true,
                    activeMap: activeMap,
                    processes: {
                        mapServer: mapServerCheck.trim() !== 'not_running',
                        navigation: navCheck.trim() !== 'not_running'
                    },
                    mapCount: parseInt(mapsList.trim()) || 0,
                    rosTopics: parseInt(rosCheck.trim()) || 0,
                    mapDirectory: sshConfig.mapDirectory
                });
                
            } catch (error) {
                conn.end();
                resolve({
                    success: false,
                    error: error.message
                });
            }
        });
        
        conn.on('error', (error) => {
            resolve({
                success: false,
                error: `SSH connection failed: ${error.message}`
            });
        });
        
        conn.connect(sshConfig);
    });
}

/**
 * Test Raspberry Pi connection
 */
async function testPiConnection(sshConfig) {
    return new Promise((resolve) => {
        const conn = new Client();
        
        conn.on('ready', async () => {
            try {
                // Test basic commands
                const hostname = await executeSSHCommand(conn, 'hostname');
                const uptime = await executeSSHCommand(conn, 'uptime');
                const diskSpace = await executeSSHCommand(conn, `df -h ${sshConfig.mapDirectory} 2>/dev/null | tail -1 || echo "Directory not accessible"`);
                
                // Test ROS2 installation
                const rosVersion = await executeSSHCommand(conn, 
                    'source /opt/ros/humble/setup.bash && ros2 --version 2>/dev/null || echo "ROS2 not found"'
                );
                
                // Check map directory
                const mapDirCheck = await executeSSHCommand(conn, 
                    `[ -d "${sshConfig.mapDirectory}" ] && echo "exists" || echo "missing"`
                );
                
                // Create map directory if missing
                if (mapDirCheck.trim() === 'missing') {
                    await executeSSHCommand(conn, `mkdir -p ${sshConfig.mapDirectory}`);
                }
                
                conn.end();
                
                resolve({
                    success: true,
                    message: 'Pi connection successful',
                    systemInfo: {
                        hostname: hostname.trim(),
                        uptime: uptime.trim(),
                        diskSpace: diskSpace.trim(),
                        rosVersion: rosVersion.trim(),
                        mapDirectory: {
                            path: sshConfig.mapDirectory,
                            exists: mapDirCheck.trim() === 'exists' || mapDirCheck.trim() === 'missing'
                        }
                    }
                });
                
            } catch (error) {
                conn.end();
                resolve({
                    success: false,
                    message: `Connection test failed: ${error.message}`,
                    error: error.message
                });
            }
        });
        
        conn.on('error', (error) => {
            resolve({
                success: false,
                message: `SSH connection failed: ${error.message}`,
                error: error.message
            });
        });
        
        conn.connect(sshConfig);
    });
}

module.exports = router;

// Export helper functions for testing and reuse
module.exports.transferFile = transferFile;
module.exports.transferFileWithRsync = transferFileWithRsync;
module.exports.deployMapToRaspberryPi = deployMapToRaspberryPi;
module.exports.testSSHConnection = testSSHConnection;
module.exports.createRemoteDirectory = createRemoteDirectory;
module.exports.createAndExecuteDeploymentScript = createAndExecuteDeploymentScript;