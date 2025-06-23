// ros/utils/pgmconverter.js - PGM (Portable Gray Map) Converter for ROS Maps
const fs = require('fs');
const path = require('path');

const mapsDir = path.resolve(__dirname, '../../maps');
if (!fs.existsSync(mapsDir)) {
    fs.mkdirSync(mapsDir, { recursive: true });
}

class PGMConverter {
  constructor() {
    this.magicNumber = 'P5'; // PGM binary format
    this.maxValue = 255;
  }

  /**
   * Convert ROS occupancy grid data to PGM format
   * @param {Object} mapData - ROS occupancy grid data
   * @param {string} outputPath - Output file path for PGM
   * @returns {Promise<boolean>} - Success status
   */
  async convertOccupancyGridToPGM(mapData, outputPath) {
    try {
      const { info, data } = mapData;
      const width = info.width;
      const height = info.height;
      const resolution = info.resolution;
      
      console.log(`Converting occupancy grid to PGM: ${width}x${height}, resolution: ${resolution}`);

      // Create PGM header
      const header = [
        this.magicNumber,
        `# Generated from ROS occupancy grid`,
        `# Resolution: ${resolution} meters/pixel`,
        `# Origin: ${info.origin.position.x}, ${info.origin.position.y}`,
        `${width} ${height}`,
        `${this.maxValue}`
      ].join('\n') + '\n';

      // Convert occupancy grid data to PGM pixel values
      const pixelData = this.convertOccupancyToPixels(data, width, height);
      
      // Combine header and pixel data
      const headerBuffer = Buffer.from(header, 'ascii');
      const imageBuffer = Buffer.concat([headerBuffer, pixelData]);

      // Write to file
      await fs.writeFile(outputPath, imageBuffer);
      
      console.log(`✅ PGM file saved: ${outputPath}`);
      return true;

    } catch (error) {
      console.error('❌ Error converting occupancy grid to PGM:', error);
      throw error;
    }
  }

  /**
   * Convert occupancy grid values to PGM pixel values
   * @param {Array} occupancyData - ROS occupancy grid data (-1 to 100)
   * @param {number} width - Image width
   * @param {number} height - Image height
   * @returns {Buffer} - Pixel data buffer
   */
  convertOccupancyToPixels(occupancyData, width, height) {
    const pixelBuffer = Buffer.alloc(width * height);
    
    for (let i = 0; i < occupancyData.length; i++) {
      const occupancyValue = occupancyData[i];
      let pixelValue;

      // Convert ROS occupancy values to grayscale:
      // -1 (unknown) -> 128 (gray)
      // 0 (free) -> 255 (white)
      // 100 (occupied) -> 0 (black)
      if (occupancyValue === -1) {
        pixelValue = 128; // Unknown areas as gray
      } else if (occupancyValue >= 0 && occupancyValue <= 100) {
        // Linear mapping: 0->255, 100->0
        pixelValue = Math.round(255 - (occupancyValue * 255 / 100));
      } else {
        pixelValue = 128; // Default to gray for invalid values
      }

      pixelBuffer[i] = pixelValue;
    }

    return pixelBuffer;
  }

  /**
   * Read and parse PGM file
   * @param {string} filePath - Path to PGM file
   * @returns {Promise<Object>} - Parsed PGM data
   */
  async readPGM(filePath) {
    try {
      const data = await fs.readFile(filePath);
      return this.parsePGM(data);
    } catch (error) {
      console.error(`❌ Error reading PGM file ${filePath}:`, error);
      throw error;
    }
  }

  /**
   * Parse PGM file data
   * @param {Buffer} data - PGM file buffer
   * @returns {Object} - Parsed PGM data
   */
  parsePGM(data) {
    let offset = 0;
    const lines = [];
    
    // Read header line by line
    while (offset < data.length) {
      const lineEnd = data.indexOf('\n', offset);
      if (lineEnd === -1) break;
      
      const line = data.slice(offset, lineEnd).toString('ascii').trim();
      offset = lineEnd + 1;
      
      // Skip comments
      if (line.startsWith('#')) continue;
      
      lines.push(line);
      
      // Stop after we have magic number, dimensions, and max value
      if (lines.length >= 3) break;
    }

    // Parse header
    const magicNumber = lines[0];
    const [width, height] = lines[1].split(' ').map(Number);
    const maxValue = parseInt(lines[2]);

    // Validate format
    if (magicNumber !== 'P5') {
      throw new Error(`Unsupported PGM format: ${magicNumber}`);
    }

    // Extract pixel data
    const pixelData = data.slice(offset);
    
    if (pixelData.length !== width * height) {
      throw new Error(`Pixel data size mismatch: expected ${width * height}, got ${pixelData.length}`);
    }

    return {
      width,
      height,
      maxValue,
      pixels: pixelData
    };
  }

  /**
   * Convert PGM to ROS occupancy grid format
   * @param {Object} pgmData - Parsed PGM data
   * @param {number} resolution - Map resolution in meters/pixel
   * @param {Object} origin - Map origin position
   * @returns {Object} - ROS occupancy grid data
   */
  convertPGMToOccupancyGrid(pgmData, resolution = 0.05, origin = { x: 0, y: 0, theta: 0 }) {
    const { width, height, pixels } = pgmData;
    const occupancyData = new Array(width * height);

    for (let i = 0; i < pixels.length; i++) {
      const pixelValue = pixels[i];
      let occupancyValue;

      // Convert grayscale to occupancy probability:
      // 255 (white) -> 0 (free)
      // 0 (black) -> 100 (occupied)
      // 128 (gray) -> -1 (unknown)
      if (pixelValue > 200) {
        occupancyValue = 0; // Free space
      } else if (pixelValue < 50) {
        occupancyValue = 100; // Occupied space
      } else {
        occupancyValue = -1; // Unknown space
      }

      occupancyData[i] = occupancyValue;
    }

    return {
      info: {
        resolution: resolution,
        width: width,
        height: height,
        origin: {
          position: { x: origin.x, y: origin.y, z: 0 },
          orientation: { x: 0, y: 0, z: Math.sin(origin.theta / 2), w: Math.cos(origin.theta / 2) }
        }
      },
      data: occupancyData
    };
  }

  /**
   * Create a blank map
   * @param {number} width - Map width in pixels
   * @param {number} height - Map height in pixels
   * @param {number} resolution - Map resolution in meters/pixel
   * @returns {Object} - Blank occupancy grid
   */
  createBlankMap(width, height, resolution = 0.05) {
    const data = new Array(width * height).fill(-1); // All unknown

    return {
      info: {
        resolution: resolution,
        width: width,
        height: height,
        origin: {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 }
        }
      },
      data: data
    };
  }

  /**
   * Resize occupancy grid
   * @param {Object} mapData - Original map data
   * @param {number} newWidth - New width
   * @param {number} newHeight - New height
   * @returns {Object} - Resized map data
   */
  resizeMap(mapData, newWidth, newHeight) {
    const { info, data } = mapData;
    const oldWidth = info.width;
    const oldHeight = info.height;
    
    const newData = new Array(newWidth * newHeight).fill(-1);

    // Simple nearest neighbor resizing
    for (let y = 0; y < newHeight; y++) {
      for (let x = 0; x < newWidth; x++) {
        const oldX = Math.floor((x / newWidth) * oldWidth);
        const oldY = Math.floor((y / newHeight) * oldHeight);
        
        if (oldX < oldWidth && oldY < oldHeight) {
          const oldIndex = oldY * oldWidth + oldX;
          const newIndex = y * newWidth + x;
          newData[newIndex] = data[oldIndex];
        }
      }
    }

    return {
      info: {
        ...info,
        width: newWidth,
        height: newHeight,
        resolution: info.resolution * (oldWidth / newWidth) // Adjust resolution
      },
      data: newData
    };
  }

  /**
   * Apply morphological operations to clean up the map
   * @param {Object} mapData - Map data to process
   * @param {string} operation - 'erode', 'dilate', 'open', 'close'
   * @param {number} kernelSize - Size of the morphological kernel
   * @returns {Object} - Processed map data
   */
  morphologicalOperation(mapData, operation = 'close', kernelSize = 3) {
    const { info, data } = mapData;
    const { width, height } = info;
    const newData = [...data];

    const radius = Math.floor(kernelSize / 2);

    for (let y = radius; y < height - radius; y++) {
      for (let x = radius; x < width - radius; x++) {
        const centerIndex = y * width + x;
        
        // Only process occupied or free cells
        if (data[centerIndex] === -1) continue;

        let neighbors = [];
        
        // Collect neighbor values
        for (let ky = -radius; ky <= radius; ky++) {
          for (let kx = -radius; kx <= radius; kx++) {
            const nx = x + kx;
            const ny = y + ky;
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
              const neighborIndex = ny * width + nx;
              if (data[neighborIndex] !== -1) {
                neighbors.push(data[neighborIndex]);
              }
            }
          }
        }

        if (neighbors.length === 0) continue;

        // Apply operation
        switch (operation) {
          case 'erode':
            newData[centerIndex] = Math.max(...neighbors);
            break;
          case 'dilate':
            newData[centerIndex] = Math.min(...neighbors);
            break;
          case 'open': // Erode then dilate
          case 'close': // Dilate then erode
            // These would require two passes
            break;
        }
      }
    }

    return {
      info: info,
      data: newData
    };
  }

  /**
   * Get map metadata as YAML string (compatible with ROS map_server)
   * @param {Object} mapData - Map data
   * @param {string} imageFile - Name of the image file
   * @returns {string} - YAML metadata
   */
  generateMapYAML(mapData, imageFile) {
    const { info } = mapData;
    
    return `image: ${imageFile}
resolution: ${info.resolution}
origin: [${info.origin.position.x}, ${info.origin.position.y}, ${Math.atan2(2 * info.origin.orientation.z * info.origin.orientation.w, 1 - 2 * info.origin.orientation.z * info.origin.orientation.z)}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
`;
  }

  /**
   * Save complete map package (PGM + YAML)
   * @param {Object} mapData - Map data to save
   * @param {string} basePath - Base path without extension
   * @returns {Promise<Object>} - Paths of saved files
   */
  async saveMapPackage(mapData, basePath) {
    try {
      const pgmPath = `${basePath}.pgm`;
      const yamlPath = `${basePath}.yaml`;
      const imageName = path.basename(pgmPath);

      // Save PGM file
      await this.convertOccupancyGridToPGM(mapData, pgmPath);
      
      // Save YAML metadata
      const yamlContent = this.generateMapYAML(mapData, imageName);
      await fs.writeFile(yamlPath, yamlContent, 'utf8');

      console.log(`✅ Map package saved: ${basePath}.{pgm,yaml}`);
      
      return {
        pgm: pgmPath,
        yaml: yamlPath
      };

    } catch (error) {
      console.error('❌ Error saving map package:', error);
      throw error;
    }
  }

  /**
   * Load complete map package (PGM + YAML)
   * @param {string} yamlPath - Path to YAML file
   * @returns {Promise<Object>} - Loaded map data
   */
  async loadMapPackage(yamlPath) {
    try {
      // Read YAML metadata
      const yamlContent = await fs.readFile(yamlPath, 'utf8');
      const metadata = this.parseMapYAML(yamlContent);
      
      // Read PGM file
      const pgmPath = path.resolve(path.dirname(yamlPath), metadata.image);
      const pgmData = await this.readPGM(pgmPath);
      
      // Convert to occupancy grid
      const origin = {
        x: metadata.origin[0],
        y: metadata.origin[1],
        theta: metadata.origin[2]
      };
      
      const mapData = this.convertPGMToOccupancyGrid(pgmData, metadata.resolution, origin);
      
      console.log(`✅ Map package loaded: ${yamlPath}`);
      return mapData;

    } catch (error) {
      console.error('❌ Error loading map package:', error);
      throw error;
    }
  }

  /**
   * Parse map YAML metadata
   * @param {string} yamlContent - YAML content
   * @returns {Object} - Parsed metadata
   */
  parseMapYAML(yamlContent) {
    const lines = yamlContent.split('\n');
    const metadata = {};

    lines.forEach(line => {
      const match = line.match(/^(\w+):\s*(.+)$/);
      if (match) {
        const [, key, value] = match;
        
        if (key === 'origin') {
          // Parse origin array [x, y, theta]
          const coords = value.match(/\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]/);
          if (coords) {
            metadata[key] = [
              parseFloat(coords[1]),
              parseFloat(coords[2]),
              parseFloat(coords[3])
            ];
          }
        } else if (key === 'resolution' || key === 'occupied_thresh' || key === 'free_thresh') {
          metadata[key] = parseFloat(value);
        } else if (key === 'negate') {
          metadata[key] = parseInt(value);
        } else {
          metadata[key] = value;
        }
      }
    });

    return metadata;
  }
}

// Create singleton instance
const pgmConverter = new PGMConverter();

module.exports = pgmConverter;

