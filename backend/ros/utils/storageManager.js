// utils/storageManager.js - Persistent Storage Manager for AMR Fleet Data
const fs = require('fs-extra');
const path = require('path');
const config = require('../../config');

class StorageManager {
    constructor() {
        this.initialized = false;
        this.autosaveInterval = null;
    }

    async initialize() {
        try {
            // Ensure all directories exist
            await this.ensureDirectories();
            
            // Load existing data
            await this.loadAllData();
            
            // Setup autosave
            this.setupAutosave();
            
            this.initialized = true;
            console.log('✅ Storage Manager initialized');
        } catch (error) {
            console.error('❌ Storage Manager initialization failed:', error);
            throw error;
        }
    }

    async ensureDirectories() {
        const directories = [
            config.STORAGE.MAPS_DIR,
            config.STORAGE.LOGS_DIR,
            config.STORAGE.UPLOADS_DIR,
            path.dirname(config.STORAGE.DEVICES_FILE),
            path.dirname(config.STORAGE.ORDERS_FILE)
        ];

        for (const dir of directories) {
            await fs.ensureDir(dir);
        }
    }

    async loadAllData() {
        try {
            // Load devices
            await this.loadDevices();
            
            // Load orders
            await this.loadOrders();
            
            // Load maps
            await this.loadMaps();
            
            console.log('📂 All persistent data loaded');
        } catch (error) {
            console.error('❌ Error loading persistent data:', error);
            // Initialize with empty data if loading fails
            this.initializeEmptyData();
        }
    }

    async loadDevices() {
        try {
            if (await fs.pathExists(config.STORAGE.DEVICES_FILE)) {
                const devices = await fs.readJson(config.STORAGE.DEVICES_FILE);
                global.connectedDevices = Array.isArray(devices) ? devices : [];
                
                // Update device status to disconnected on startup
                global.connectedDevices.forEach(device => {
                    device.status = 'disconnected';
                    device.lastStartup = new Date().toISOString();
                });
                
                console.log(`📱 Loaded ${global.connectedDevices.length} devices`);
            } else {
                global.connectedDevices = [];
            }
        } catch (error) {
            console.error('❌ Error loading devices:', error);
            global.connectedDevices = [];
        }
    }

    async loadOrders() {
        try {
            if (await fs.pathExists(config.STORAGE.ORDERS_FILE)) {
                const orders = await fs.readJson(config.STORAGE.ORDERS_FILE);
                global.deviceOrders = orders || {};
                global.orderQueue = {};
                
                // Initialize order queues and mark active orders as pending
                for (const [deviceId, deviceOrders] of Object.entries(global.deviceOrders)) {
                    global.orderQueue[deviceId] = { current: null, pending: [] };
                    
                    // Reset order statuses on startup
                    deviceOrders.forEach(order => {
                        if (order.status === 'active') {
                            order.status = 'pending';
                            order.resetAt = new Date().toISOString();
                        }
                    });
                }
                
                const totalOrders = Object.values(global.deviceOrders).reduce((acc, orders) => acc + orders.length, 0);
                console.log(`📋 Loaded ${totalOrders} orders for ${Object.keys(global.deviceOrders).length} devices`);
            } else {
                global.deviceOrders = {};
                global.orderQueue = {};
            }
        } catch (error) {
            console.error('❌ Error loading orders:', error);
            global.deviceOrders = {};
            global.orderQueue = {};
        }
    }

    async loadMaps() {
        try {
            global.deviceMaps = {};
            
            const mapFiles = await fs.readdir(config.STORAGE.MAPS_DIR).catch(() => []);
            
            for (const file of mapFiles) {
                if (file.endsWith('.json')) {
                    try {
                        const deviceId = file.replace('.json', '');
                        const mapPath = path.join(config.STORAGE.MAPS_DIR, file);
                        const mapData = await fs.readJson(mapPath);
                        
                        // Validate map data structure
                        if (this.validateMapData(mapData)) {
                            global.deviceMaps[deviceId] = mapData;
                        } else {
                            console.warn(`⚠️ Invalid map data for device ${deviceId}, skipping`);
                        }
                    } catch (error) {
                        console.error(`❌ Error loading map file ${file}:`, error);
                    }
                }
            }
            
            console.log(`🗺️ Loaded maps for ${Object.keys(global.deviceMaps).length} devices`);
        } catch (error) {
            console.error('❌ Error loading maps:', error);
            global.deviceMaps = {};
        }
    }

    initializeEmptyData() {
        global.connectedDevices = global.connectedDevices || [];
        global.deviceOrders = global.deviceOrders || {};
        global.orderQueue = global.orderQueue || {};
        global.deviceMaps = global.deviceMaps || {};
        global.liveData = global.liveData || {};
    }

    validateMapData(mapData) {
        return mapData && 
               typeof mapData === 'object' && 
               Array.isArray(mapData.shapes) && 
               Array.isArray(mapData.annotations);
    }

    async saveAllData() {
        try {
            await Promise.all([
                this.saveDevices(),
                this.saveOrders(),
                this.saveMaps()
            ]);
        } catch (error) {
            console.error('❌ Error saving all data:', error);
            throw error;
        }
    }

    async saveDevices() {
        try {
            await fs.writeJson(config.STORAGE.DEVICES_FILE, global.connectedDevices || [], { spaces: 2 });
        } catch (error) {
            console.error('❌ Error saving devices:', error);
            throw error;
        }
    }

    async saveOrders() {
        try {
            await fs.writeJson(config.STORAGE.ORDERS_FILE, global.deviceOrders || {}, { spaces: 2 });
        } catch (error) {
            console.error('❌ Error saving orders:', error);
            throw error;
        }
    }

    async saveMaps() {
        try {
            for (const [deviceId, mapData] of Object.entries(global.deviceMaps || {})) {
                const mapPath = path.join(config.STORAGE.MAPS_DIR, `${deviceId}.json`);
                await fs.writeJson(mapPath, mapData, { spaces: 2 });
            }
        } catch (error) {
            console.error('❌ Error saving maps:', error);
            throw error;
        }
    }

    async saveDevice(deviceId) {
        const device = global.connectedDevices?.find(d => d.id === deviceId);
        if (device) {
            device.lastSaved = new Date().toISOString();
            await this.saveDevices();
        }
    }

    async saveMap(deviceId) {
        const mapData = global.deviceMaps?.[deviceId];
        if (mapData) {
            mapData.lastSaved = new Date().toISOString();
            const mapPath = path.join(config.STORAGE.MAPS_DIR, `${deviceId}.json`);
            await fs.writeJson(mapPath, mapData, { spaces: 2 });
        }
    }

    async saveOrder(deviceId, orderId) {
        const orders = global.deviceOrders?.[deviceId];
        const order = orders?.find(o => o.id === orderId);
        if (order) {
            order.lastSaved = new Date().toISOString();
            await this.saveOrders();
        }
    }

    async exportDeviceData(deviceId, exportPath) {
        try {
            const device = global.connectedDevices?.find(d => d.id === deviceId);
            const orders = global.deviceOrders?.[deviceId] || [];
            const mapData = global.deviceMaps?.[deviceId];
            const liveData = global.liveData?.[deviceId];

            const exportData = {
                device,
                orders,
                mapData,
                liveData,
                exportedAt: new Date().toISOString(),
                version: '1.0'
            };

            await fs.writeJson(exportPath, exportData, { spaces: 2 });
            
            console.log(`📤 Exported data for device ${deviceId} to ${exportPath}`);
            return exportData;
        } catch (error) {
            console.error(`❌ Error exporting device data for ${deviceId}:`, error);
            throw error;
        }
    }

    async importDeviceData(importPath) {
        try {
            const importData = await fs.readJson(importPath);
            
            if (!importData.device || !importData.device.id) {
                throw new Error('Invalid import data: missing device information');
            }

            const deviceId = importData.device.id;

            // Import device
            if (importData.device) {
                const existingIndex = global.connectedDevices.findIndex(d => d.id === deviceId);
                if (existingIndex >= 0) {
                    global.connectedDevices[existingIndex] = {
                        ...importData.device,
                        importedAt: new Date().toISOString(),
                        status: 'disconnected'
                    };
                } else {
                    global.connectedDevices.push({
                        ...importData.device,
                        importedAt: new Date().toISOString(),
                        status: 'disconnected'
                    });
                }
            }

            // Import orders
            if (importData.orders) {
                global.deviceOrders[deviceId] = importData.orders.map(order => ({
                    ...order,
                    importedAt: new Date().toISOString(),
                    status: order.status === 'active' ? 'pending' : order.status
                }));
            }

            // Import map
            if (importData.mapData) {
                global.deviceMaps[deviceId] = {
                    ...importData.mapData,
                    importedAt: new Date().toISOString()
                };
            }

            // Save all imported data
            await this.saveAllData();

            console.log(`📥 Imported data for device ${deviceId} from ${importPath}`);
            return { deviceId, imported: true };
        } catch (error) {
            console.error(`❌ Error importing device data:`, error);
            throw error;
        }
    }

    async getStorageStats() {
        try {
            const stats = {
                devices: {
                    count: global.connectedDevices?.length || 0,
                    connected: global.connectedDevices?.filter(d => d.status === 'connected').length || 0,
                    withMaps: Object.keys(global.deviceMaps || {}).length,
                    withOrders: Object.keys(global.deviceOrders || {}).length
                },
                orders: {
                    total: Object.values(global.deviceOrders || {}).reduce((acc, orders) => acc + orders.length, 0),
                    pending: 0,
                    active: 0,
                    completed: 0
                },
                maps: {
                    total: Object.keys(global.deviceMaps || {}).length,
                    withShapes: 0,
                    withAnnotations: 0,
                    totalShapes: 0,
                    totalAnnotations: 0
                },
                storage: {
                    mapsDir: await this.getDirectorySize(config.STORAGE.MAPS_DIR),
                    logsDir: await this.getDirectorySize(config.STORAGE.LOGS_DIR),
                    uploadsDir: await this.getDirectorySize(config.STORAGE.UPLOADS_DIR)
                }
            };

            // Calculate order statistics
            Object.values(global.deviceOrders || {}).forEach(orders => {
                orders.forEach(order => {
                    if (order.status === 'pending') stats.orders.pending++;
                    else if (order.status === 'active') stats.orders.active++;
                    else if (order.status === 'completed') stats.orders.completed++;
                });
            });

            // Calculate map statistics
            Object.values(global.deviceMaps || {}).forEach(mapData => {
                if (mapData.shapes && mapData.shapes.length > 0) {
                    stats.maps.withShapes++;
                    stats.maps.totalShapes += mapData.shapes.length;
                }
                if (mapData.annotations && mapData.annotations.length > 0) {
                    stats.maps.withAnnotations++;
                    stats.maps.totalAnnotations += mapData.annotations.length;
                }
            });

            return stats;
        } catch (error) {
            console.error('❌ Error getting storage stats:', error);
            return null;
        }
    }

    async getDirectorySize(dirPath) {
        try {
            let totalSize = 0;
            const files = await fs.readdir(dirPath);
            
            for (const file of files) {
                const filePath = path.join(dirPath, file);
                const stats = await fs.stat(filePath);
                totalSize += stats.size;
            }
            
            return {
                bytes: totalSize,
                human: this.formatBytes(totalSize),
                fileCount: files.length
            };
        } catch (error) {
            return { bytes: 0, human: '0 B', fileCount: 0 };
        }
    }

    formatBytes(bytes) {
        const sizes = ['B', 'KB', 'MB', 'GB', 'TB'];
        if (bytes === 0) return '0 B';
        const i = Math.floor(Math.log(bytes) / Math.log(1024));
        return Math.round(bytes / Math.pow(1024, i) * 100) / 100 + ' ' + sizes[i];
    }

    setupAutosave() {
        // Save data every 2 minutes
        this.autosaveInterval = setInterval(async () => {
            try {
                await this.saveAllData();
                console.log('💾 Autosave completed');
            } catch (error) {
                console.error('❌ Autosave failed:', error);
            }
        }, 120000); // 2 minutes
    }

    async cleanup() {
        try {
            if (this.autosaveInterval) {
                clearInterval(this.autosaveInterval);
            }
            
            // Final save before cleanup
            await this.saveAllData();
            
            console.log('🧹 Storage Manager cleanup completed');
        } catch (error) {
            console.error('❌ Storage Manager cleanup error:', error);
        }
    }
}

// Create singleton instance
const storageManager = new StorageManager();

module.exports = storageManager;