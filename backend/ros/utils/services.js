// ros/utils/services.js - Utility Services and Validation
const { exec } = require('child_process');
const { promisify } = require('util');
const execAsync = promisify(exec);
const config = require('../../config');

class UtilityServices {
  constructor() {
    this.networkInterfaces = new Map();
  }

  // Network utilities
  async discoverAMRsOnNetwork() {
    try {
      console.log(' Discovering AMRs on network...');
      
      const interfaces = await this.getNetworkInterfaces();
      const discoveries = [];
      
      for (const [name, info] of interfaces) {
        if (info.family === 'IPv4' && !info.internal) {
          const subnet = this.getSubnet(info.address, info.netmask);
          const AMRs = await this.scanSubnetForAMRs(subnet);
          discoveries.push(...AMRs);
        }
      }
      
      console.log(` Discovery complete. Found ${discoveries.length} potential AMRs`);
      return discoveries;
      
    } catch (error) {
      console.error(' Error during AMR discovery:', error);
      return [];
    }
  }

  async getNetworkInterfaces() {
    try {
      const interfaces = require('os').networkInterfaces();
      const interfaceMap = new Map();
      
      Object.keys(interfaces).forEach(name => {
        interfaces[name].forEach(addr => {
          if (addr.family === 'IPv4' && !addr.internal) {
            interfaceMap.set(name, {
              name: name,
              address: addr.address,
              netmask: addr.netmask,
              family: addr.family,
              internal: addr.internal
            });
          }
        });
      });
      
      return interfaceMap;
    } catch (error) {
      console.error('Error getting network interfaces:', error);
      return new Map();
    }
  }

  async scanSubnetForAMRs(subnet) {
    try {
      console.log(` Scanning subnet ${subnet} for AMRs...`);
      const baseIp = subnet.split('/')[0].split('.').slice(0, 3).join('.');
      const AMRs = [];
      
      // Scan common AMR IP ranges (last octet 100-200)
      const pingPromises = [];
      for (let i = 100; i <= 200; i++) {
        const ip = `${baseIp}.${i}`;
        pingPromises.push(this.pingHost(ip));
      }
      
      const results = await Promise.allSettled(pingPromises);
      
      for (let i = 0; i < results.length; i++) {
        if (results[i].status === 'fulfilled' && results[i].value) {
          const ip = `${baseIp}.${i + 100}`;
          const AMRInfo = await this.checkIfAMR(ip);
          if (AMRInfo) {
            AMRs.push(AMRInfo);
          }
        }
      }
      
      return AMRs;
    } catch (error) {
      console.error(`Error scanning subnet ${subnet}:`, error);
      return [];
    }
  }

  async pingHost(ip) {
    try {
      await execAsync(`ping -c 1 -W 1 ${ip}`);
      return true;
    } catch (error) {
      return false;
    }
  }

  async checkIfAMR(ip) {
    try {
      // Try to connect to common ROS/AMR ports
      const commonPorts = [11311, 7400, 7401, 9090];
      
      for (const port of commonPorts) {
        const isOpen = await this.checkPort(ip, port);
        if (isOpen) {
          return {
            ipAddress: ip,
            port: port,
            type: 'AMR',
            status: 'discovered',
            name: `AMR_${ip.split('.')[3]}`,
            capabilities: ['navigation', 'mapping'],
            discoveredAt: new Date().toISOString()
          };
        }
      }
      
      return null;
    } catch (error) {
      return null;
    }
  }

  async checkPort(ip, port) {
    try {
      const { stdout } = await execAsync(`timeout 2 bash -c "echo >/dev/tcp/${ip}/${port}" 2>/dev/null && echo "open" || echo "closed"`);
      return stdout.trim() === 'open';
    } catch (error) {
      return false;
    }
  }

  // ROS utilities
  async checkROSEnvironment() {
    try {
      console.log(' Checking ROS2 environment...');
      
      const checks = {
        rosDistro: await this.checkROSDistro(),
        rosDomainId: await this.checkROSDomainId(),
        rosVersion: await this.checkROSVersion(),
        pythonROS: await this.checkPythonROSPackages(),
        nodeROS: await this.checkNodeROSPackages()
      };
      
      console.log(' ROS2 environment check complete');
      return checks;
    } catch (error) {
      console.error(' Error checking ROS environment:', error);
      return null;
    }
  }

  async checkROSDistro() {
    try {
      const distro = process.env.ROS_DISTRO;
      if (distro) {
        return { available: true, distro: distro };
      } else {
        const { stdout } = await execAsync('ls /opt/ros/ 2>/dev/null || echo "none"');
        const available = stdout.trim() !== 'none';
        return { available, distro: available ? stdout.trim().split('\n')[0] : null };
      }
    } catch (error) {
      return { available: false, distro: null };
    }
  }

  async checkROSDomainId() {
    const domainId = process.env.ROS_DOMAIN_ID || config.ros.domainId;
    return { domainId: parseInt(domainId) };
  }

  async checkROSVersion() {
    try {
      const { stdout } = await execAsync('ros2 --version 2>/dev/null || echo "not available"');
      const available = !stdout.includes('not available');
      return { available, version: available ? stdout.trim() : null };
    } catch (error) {
      return { available: false, version: null };
    }
  }

  async checkPythonROSPackages() {
    try {
      const { stdout } = await execAsync('python3 -c "import rclpy; print(rclpy.__version__)" 2>/dev/null || echo "not available"');
      const available = !stdout.includes('not available');
      return { available, version: available ? stdout.trim() : null };
    } catch (error) {
      return { available: false, version: null };
    }
  }

  async checkNodeROSPackages() {
    try {
      const packageJson = require('../../../package.json');
      const rclnodejsVersion = packageJson.dependencies.rclnodejs;
      return { available: true, version: rclnodejsVersion };
    } catch (error) {
      return { available: false, version: null };
    }
  }

  // Validation utilities
  validateAMRCommand(command, params) {
    const validCommands = {
      move: {
        required: [],
        optional: ['linear', 'angular'],
        types: { linear: 'number', angular: 'number' }
      },
      stop: {
        required: [],
        optional: [],
        types: {}
      },
      goto: {
        required: ['target'],
        optional: [],
        types: { target: 'object' }
      },
      mission: {
        required: ['waypoints'],
        optional: ['priority'],
        types: { waypoints: 'array', priority: 'string' }
      },
      emergency_stop: {
        required: [],
        optional: [],
        types: {}
      }
    };

    const commandDef = validCommands[command];
    if (!commandDef) {
      throw new Error(`Invalid command: ${command}`);
    }

    // Check required parameters
    for (const required of commandDef.required) {
      if (!(required in params)) {
        throw new Error(`Missing required parameter: ${required}`);
      }
    }

    // Check parameter types
    for (const [param, value] of Object.entries(params)) {
      const expectedType = commandDef.types[param];
      if (expectedType) {
        if (expectedType === 'array' && !Array.isArray(value)) {
          throw new Error(`Parameter ${param} must be an array`);
        } else if (expectedType !== 'array' && typeof value !== expectedType) {
          throw new Error(`Parameter ${param} must be of type ${expectedType}`);
        }
      }
    }

    return true;
  }

  validateWaypoints(waypoints) {
    if (!Array.isArray(waypoints)) {
      throw new Error('Waypoints must be an array');
    }

    if (waypoints.length === 0) {
      throw new Error('At least one waypoint is required');
    }

    if (waypoints.length > config.missions.maxWaypoints) {
      throw new Error(`Too many waypoints. Maximum: ${config.missions.maxWaypoints}`);
    }

    waypoints.forEach((waypoint, index) => {
      if (typeof waypoint !== 'object' || waypoint === null) {
        throw new Error(`Waypoint ${index} must be an object`);
      }

      if (typeof waypoint.x !== 'number' || typeof waypoint.y !== 'number') {
        throw new Error(`Waypoint ${index} must have numeric x and y coordinates`);
      }

      if (isNaN(waypoint.x) || isNaN(waypoint.y)) {
        throw new Error(`Waypoint ${index} coordinates cannot be NaN`);
      }
    });

    return true;
  }

  // Helper utilities
  getSubnet(ip, netmask) {
    const ipParts = ip.split('.').map(Number);
    const maskParts = netmask.split('.').map(Number);
    
    const networkParts = ipParts.map((part, index) => part & maskParts[index]);
    const network = networkParts.join('.');
    
    // Calculate CIDR
    const cidr = maskParts.reduce((count, part) => {
      return count + part.toString(2).split('1').length - 1;
    }, 0);
    
    return `${network}/${cidr}`;
  }

  calculateDistance(point1, point2) {
    const dx = point1.x - point2.x;
    const dy = point1.y - point2.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  calculateBearing(from, to) {
    const dx = to.x - from.x;
    const dy = to.y - from.y;
    return Math.atan2(dy, dx);
  }

  formatDuration(milliseconds) {
    const seconds = Math.floor(milliseconds / 1000);
    const minutes = Math.floor(seconds / 60);
    const hours = Math.floor(minutes / 60);
    
    if (hours > 0) {
      return `${hours}h ${minutes % 60}m ${seconds % 60}s`;
    } else if (minutes > 0) {
      return `${minutes}m ${seconds % 60}s`;
    } else {
      return `${seconds}s`;
    }
  }

  formatBytes(bytes) {
    const sizes = ['Bytes', 'KB', 'MB', 'GB', 'TB'];
    if (bytes === 0) return '0 Bytes';
    const i = Math.floor(Math.log(bytes) / Math.log(1024));
    return Math.round(bytes / Math.pow(1024, i) * 100) / 100 + ' ' + sizes[i];
  }

  // Logging utilities
  logAMREvent(AMRId, event, data = {}) {
    const logEntry = {
      timestamp: new Date().toISOString(),
      AMRId,
      event,
      data
    };
    
    console.log(` AMR Event: ${JSON.stringify(logEntry)}`);
    return logEntry;
  }

  logSystemEvent(event, data = {}) {
    const logEntry = {
      timestamp: new Date().toISOString(),
      system: true,
      event,
      data
    };
    
    console.log(` System Event: ${JSON.stringify(logEntry)}`);
    return logEntry;
  }
}

// Create singleton instance
const utilityServices = new UtilityServices();
module.exports = utilityServices;