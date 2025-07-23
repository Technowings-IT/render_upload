// utils/mapConverter.js - Node.js wrapper for Python map converter
const { spawn } = require('child_process');
const fs = require('fs').promises;
const path = require('path');

class MapConverter {
    constructor() {
        this.pythonScript = path.join(__dirname, '../../scripts/map_converter.py');
        this.tempDir = path.join(__dirname, '../../temp');
    }

    async convertJsonToPgmYaml(mapData, mapName, outputDir = null) {
        try {
            // Ensure temp directory exists
            await fs.mkdir(this.tempDir, { recursive: true });
            
            // Set default output directory
            if (!outputDir) {
                outputDir = path.join(__dirname, '../../maps');
            }
            
            // Ensure output directory exists
            await fs.mkdir(outputDir, { recursive: true });
            
            // Create temporary JSON file
            const tempJsonPath = path.join(this.tempDir, `${mapName}_temp.json`);
            await fs.writeFile(tempJsonPath, JSON.stringify(mapData, null, 2));
            
            // Run Python converter
            const result = await this.runPythonConverter(tempJsonPath, outputDir, mapName);
            
            // Cleanup temp file
            await fs.unlink(tempJsonPath).catch(console.error);
            
            return result;
            
        } catch (error) {
            throw new Error(`Map conversion failed: ${error.message}`);
        }
    }

    async runPythonConverter(inputFile, outputDir, mapName) {
        return new Promise((resolve, reject) => {
            const pythonProcess = spawn('python3', [
                this.pythonScript,
                inputFile,
                '-o', outputDir,
                '-n', mapName
            ]);

            let stdout = '';
            let stderr = '';

            pythonProcess.stdout.on('data', (data) => {
                stdout += data.toString();
            });

            pythonProcess.stderr.on('data', (data) => {
                stderr += data.toString();
            });

            pythonProcess.on('close', (code) => {
                if (code === 0) {
                    resolve({
                        success: true,
                        pgmPath: path.join(outputDir, `${mapName}.pgm`),
                        yamlPath: path.join(outputDir, `${mapName}.yaml`),
                        output: stdout
                    });
                } else {
                    reject(new Error(`Python converter failed with code ${code}: ${stderr}`));
                }
            });

            pythonProcess.on('error', (error) => {
                reject(new Error(`Failed to start Python converter: ${error.message}`));
            });
        });
    }

    async validateMapFiles(pgmPath, yamlPath) {
        try {
            // Check if files exist
            await fs.access(pgmPath);
            await fs.access(yamlPath);
            
            // Validate YAML content
            const yamlContent = await fs.readFile(yamlPath, 'utf8');
            const yamlData = require('yaml').parse(yamlContent);
            
            // Check required YAML fields
            const requiredFields = ['image', 'resolution', 'origin', 'occupied_thresh', 'free_thresh'];
            for (const field of requiredFields) {
                if (!(field in yamlData)) {
                    throw new Error(`Missing required field in YAML: ${field}`);
                }
            }
            
            // Get file stats
            const pgmStats = await fs.stat(pgmPath);
            const yamlStats = await fs.stat(yamlPath);
            
            return {
                valid: true,
                pgmSize: pgmStats.size,
                yamlSize: yamlStats.size,
                yamlData: yamlData
            };
            
        } catch (error) {
            return {
                valid: false,
                error: error.message
            };
        }
    }

    async getMapInfo(yamlPath) {
        try {
            const yamlContent = await fs.readFile(yamlPath, 'utf8');
            const yamlData = require('yaml').parse(yamlContent);
            
            return {
                image: yamlData.image,
                resolution: yamlData.resolution,
                origin: yamlData.origin,
                occupiedThresh: yamlData.occupied_thresh,
                freeThresh: yamlData.free_thresh,
                negate: yamlData.negate || 0
            };
            
        } catch (error) {
            throw new Error(`Failed to read map info: ${error.message}`);
        }
    }

    async listAvailableMaps(mapsDir = null) {
        try {
            if (!mapsDir) {
                mapsDir = path.join(__dirname, '../maps');
            }
            
            const files = await fs.readdir(mapsDir);
            const yamlFiles = files.filter(file => file.endsWith('.yaml'));
            
            const maps = [];
            for (const yamlFile of yamlFiles) {
                const mapName = path.basename(yamlFile, '.yaml');
                const yamlPath = path.join(mapsDir, yamlFile);
                const pgmPath = path.join(mapsDir, `${mapName}.pgm`);
                
                try {
                    const mapInfo = await this.getMapInfo(yamlPath);
                    const validation = await this.validateMapFiles(pgmPath, yamlPath);
                    
                    maps.push({
                        name: mapName,
                        yamlPath: yamlPath,
                        pgmPath: pgmPath,
                        info: mapInfo,
                        valid: validation.valid,
                        ...validation
                    });
                } catch (error) {
                    maps.push({
                        name: mapName,
                        yamlPath: yamlPath,
                        pgmPath: pgmPath,
                        valid: false,
                        error: error.message
                    });
                }
            }
            
            return maps;
            
        } catch (error) {
            throw new Error(`Failed to list maps: ${error.message}`);
        }
    }

    async deleteMap(mapName, mapsDir = null) {
        try {
            if (!mapsDir) {
                mapsDir = path.join(__dirname, '../maps');
            }
            
            const yamlPath = path.join(mapsDir, `${mapName}.yaml`);
            const pgmPath = path.join(mapsDir, `${mapName}.pgm`);
            
            // Delete both files
            await Promise.all([
                fs.unlink(yamlPath).catch(() => {}), // Ignore errors if file doesn't exist
                fs.unlink(pgmPath).catch(() => {})
            ]);
            
            return { success: true, message: `Map ${mapName} deleted successfully` };
            
        } catch (error) {
            throw new Error(`Failed to delete map: ${error.message}`);
        }
    }
}

module.exports = MapConverter;