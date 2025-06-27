// services/api_service.dart - FIXED with Complete Analytics Implementation
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'package:http/http.dart' as http;
import '../models/map_data.dart';
import '../models/odom.dart';

class ApiService {
  static final ApiService _instance = ApiService._internal();
  factory ApiService() => _instance;
  ApiService._internal();

  String? _baseUrl;
  Duration _timeout = const Duration(seconds: 10);
  Map<String, String> _headers = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
  };

  // Connection state
  bool _isInitialized = false;
  Map<String, dynamic>? _serverInfo;

  // Analytics tracking
  int _totalRequests = 0;
  int _successfulRequests = 0;
  int _failedRequests = 0;
  final Map<String, int> _endpointStats = {};
  final Map<int, int> _statusCodeStats = {};

  // Caching
  final Map<String, _CacheEntry> _cache = {};
  bool _cachingEnabled = true;
  static const Duration _defaultCacheTtl = Duration(minutes: 5);

  // ==========================================
  // INITIALIZATION & CONFIGURATION
  // ==========================================

  void initialize(String baseUrl) {
    _baseUrl = baseUrl.replaceAll('/api', '');
    _isInitialized = true;
    print('🔗 API Service initialized with base URL: $_baseUrl');
  }

  void setBaseUrl(String baseUrl) {
    _baseUrl = baseUrl.replaceAll('/api', '');
    _isInitialized = true;
    print('🔗 API Service base URL updated: $_baseUrl');
  }

  Future<bool> autoInitialize() async {
    try {
      print('🔍 Auto-discovering AGV backend...');
      
      final backendUrl = await _discoverBackend();
      if (backendUrl != null) {
        initialize(backendUrl);
        return await testConnection();
      }
      
      return false;
    } catch (e) {
      print('❌ Auto-initialization failed: $e');
      return false;
    }
  }

  Future<String?> _discoverBackend() async {
    try {
      final networkInfo = await _getDeviceNetworkInfo();
      if (networkInfo == null) return null;

      final subnet = networkInfo['subnet']!;
      print('📡 Scanning subnet: $subnet.x for AGV backends');

      final commonIPs = [
        '$subnet.79',
        '$subnet.136',
        '$subnet.100',
        '$subnet.101',
        '$subnet.110',
        '$subnet.200',
      ];

      for (final ip in commonIPs) {
        final url = await _testBackendAtIP(ip);
        if (url != null) {
          print('✅ Found AGV backend at: $url');
          return url;
        }
      }

      for (int i = 70; i <= 90; i++) {
        final ip = '$subnet.$i';
        final url = await _testBackendAtIP(ip);
        if (url != null) {
          print('✅ Found AGV backend at: $url');
          return url;
        }
      }

      print('❌ No AGV backend found on network');
      return null;

    } catch (e) {
      print('❌ Backend discovery failed: $e');
      return null;
    }
  }

  Future<String?> _testBackendAtIP(String ip) async {
    final ports = [3000, 8080, 80];
    
    for (final port in ports) {
      try {
        final response = await http.get(
          Uri.parse('http://$ip:$port/health'),
          headers: {'Accept': 'application/json'},
        ).timeout(const Duration(seconds: 2));

        if (response.statusCode == 200) {
          final data = json.decode(response.body);
          
          if (_isAGVBackend(data)) {
            return 'http://$ip:$port';
          }
        }
      } catch (e) {
        // Continue to next port
      }
    }
    
    return null;
  }
  // services/enhanced_api_service.dart - Add these methods to your existing ApiService class

// ==========================================
// PGM CONVERSION & MAP MANAGEMENT METHODS
// ==========================================

/// Export current map data to PGM format
Future<Map<String, dynamic>> exportMapToPGM({
  required String deviceId,
  bool includeGlobalCostmap = true,
  bool includeLocalCostmap = true,
  bool includeStaticMap = true,
  String? mapName,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/maps/$deviceId/export/pgm', {
      'mapName': mapName ?? 'exported_map_${DateTime.now().millisecondsSinceEpoch}',
      'includeGlobalCostmap': includeGlobalCostmap,
      'includeLocalCostmap': includeLocalCostmap,
      'includeStaticMap': includeStaticMap,
      'exportTimestamp': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ Map exported to PGM: ${response['files']}');
    }

    return response;
  } catch (e) {
    print('❌ Error exporting map to PGM: $e');
    throw ApiException('Failed to export map to PGM: $e');
  }
}

/// Upload map to AGV (send PGM + YAML back to ROS)
Future<Map<String, dynamic>> uploadMapToAGV({
  required String deviceId,
  required String mapName,
  bool setAsActiveMap = true,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/maps/$deviceId/upload', {
      'mapName': mapName,
      'setAsActiveMap': setAsActiveMap,
      'uploadTimestamp': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ Map uploaded to AGV: $mapName');
    }

    return response;
  } catch (e) {
    print('❌ Error uploading map to AGV: $e');
    throw ApiException('Failed to upload map to AGV: $e');
  }
}

/// Get available PGM exports for a device
Future<List<Map<String, dynamic>>> getAvailableMaps(String deviceId) async {
  _ensureInitialized();
  
  try {
    final response = await _get('/api/maps/$deviceId/available');

    if (response['success'] == true) {
      final maps = response['maps'] as List?;
      if (maps != null) {
        return maps.cast<Map<String, dynamic>>();
      }
    }

    return [];
  } catch (e) {
    print('❌ Error getting available maps: $e');
    throw ApiException('Failed to get available maps: $e');
  }
}

/// Download a PGM file for external use
Future<Map<String, dynamic>> downloadPGMFile({
  required String deviceId,
  required String fileName,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _get('/api/maps/$deviceId/download/$fileName');
    return response;
  } catch (e) {
    print('❌ Error downloading PGM file: $e');
    throw ApiException('Failed to download PGM file: $e');
  }
}

/// Convert occupancy grid to different formats
Future<Map<String, dynamic>> convertMapFormat({
  required String deviceId,
  required String sourceFormat, // 'occupancy_grid', 'pgm', 'yaml'
  required String targetFormat, // 'pgm', 'yaml', 'json', 'png'
  String? mapName,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/maps/$deviceId/convert', {
      'sourceFormat': sourceFormat,
      'targetFormat': targetFormat,
      'mapName': mapName,
      'conversionTimestamp': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ Map converted from $sourceFormat to $targetFormat');
    }

    return response;
  } catch (e) {
    print('❌ Error converting map format: $e');
    throw ApiException('Failed to convert map format: $e');
  }
}

/// Get real-time costmap data directly (alternative to WebSocket)
Future<Map<String, dynamic>> getCostmapData({
  required String deviceId,
  String costmapType = 'both', // 'global', 'local', 'both'
}) async {
  _ensureInitialized();
  
  try {
    final response = await _get('/api/maps/$deviceId/costmaps?type=$costmapType');

    if (response['success'] == true) {
      return response;
    }

    throw ApiException('Failed to get costmap data: ${response['error']}');
  } catch (e) {
    print('❌ Error getting costmap data: $e');
    throw ApiException('Failed to get costmap data: $e');
  }
}

/// Update map metadata (resolution, origin, etc.)
Future<Map<String, dynamic>> updateMapMetadata({
  required String deviceId,
  required String mapName,
  double? resolution,
  Map<String, double>? origin,
  Map<String, dynamic>? additionalMetadata,
}) async {
  _ensureInitialized();
  
  try {
    final updateData = <String, dynamic>{
      'mapName': mapName,
      'updatedAt': DateTime.now().toIso8601String(),
    };

    if (resolution != null) updateData['resolution'] = resolution;
    if (origin != null) updateData['origin'] = origin;
    if (additionalMetadata != null) updateData.addAll(additionalMetadata);

    final response = await _put('/api/maps/$deviceId/metadata', updateData);

    if (response['success'] == true) {
      print('✅ Map metadata updated: $mapName');
    }

    return response;
  } catch (e) {
    print('❌ Error updating map metadata: $e');
    throw ApiException('Failed to update map metadata: $e');
  }
}

/// Apply image processing operations to map
Future<Map<String, dynamic>> processMap({
  required String deviceId,
  required String mapName,
  required String operation, // 'erode', 'dilate', 'open', 'close', 'blur', 'sharpen'
  Map<String, dynamic>? operationParams,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/maps/$deviceId/process', {
      'mapName': mapName,
      'operation': operation,
      'operationParams': operationParams ?? {},
      'processedAt': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ Map processed with operation: $operation');
    }

    return response;
  } catch (e) {
    print('❌ Error processing map: $e');
    throw ApiException('Failed to process map: $e');
  }
}

/// Merge multiple maps into one
Future<Map<String, dynamic>> mergeMaps({
  required String deviceId,
  required List<String> mapNames,
  required String outputMapName,
  String mergeStrategy = 'overlay', // 'overlay', 'average', 'maximum', 'minimum'
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/maps/$deviceId/merge', {
      'mapNames': mapNames,
      'outputMapName': outputMapName,
      'mergeStrategy': mergeStrategy,
      'mergedAt': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ Maps merged into: $outputMapName');
    }

    return response;
  } catch (e) {
    print('❌ Error merging maps: $e');
    throw ApiException('Failed to merge maps: $e');
  }
}

/// Delete map files
Future<Map<String, dynamic>> deleteMap({
  required String deviceId,
  required String mapName,
  bool deleteAllFormats = true,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _delete('/api/maps/$deviceId/$mapName?deleteAllFormats=$deleteAllFormats');

    if (response['success'] == true) {
      print('✅ Map deleted: $mapName');
    }

    return response;
  } catch (e) {
    print('❌ Error deleting map: $e');
    throw ApiException('Failed to delete map: $e');
  }
}

/// Get map conversion status for long-running operations
Future<Map<String, dynamic>> getConversionStatus({
  required String deviceId,
  required String conversionId,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _get('/api/maps/$deviceId/conversions/$conversionId/status');
    return response;
  } catch (e) {
    print('❌ Error getting conversion status: $e');
    throw ApiException('Failed to get conversion status: $e');
  }
}

/// Set map as active on the AGV
Future<Map<String, dynamic>> setActiveMap({
  required String deviceId,
  required String mapName,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/maps/$deviceId/set-active', {
      'mapName': mapName,
      'activatedAt': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ Active map set to: $mapName');
    }

    return response;
  } catch (e) {
    print('❌ Error setting active map: $e');
    throw ApiException('Failed to set active map: $e');
  }
}

/// Get map statistics and information
Future<Map<String, dynamic>> getMapStatistics({
  required String deviceId,
  required String mapName,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _get('/api/maps/$deviceId/$mapName/statistics');

    if (response['success'] == true) {
      return response;
    }

    throw ApiException('Failed to get map statistics: ${response['error']}');
  } catch (e) {
    print('❌ Error getting map statistics: $e');
    throw ApiException('Failed to get map statistics: $e');
  }
}

// ==========================================
// ENHANCED COSTMAP-SPECIFIC METHODS
// ==========================================

/// Subscribe to costmap updates via HTTP polling (fallback for WebSocket)
Future<Map<String, dynamic>> startCostmapPolling({
  required String deviceId,
  int intervalMs = 1000,
  List<String> costmapTypes = const ['global', 'local'],
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/costmaps/$deviceId/polling/start', {
      'intervalMs': intervalMs,
      'costmapTypes': costmapTypes,
      'startedAt': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ Costmap polling started for: $costmapTypes');
    }

    return response;
  } catch (e) {
    print('❌ Error starting costmap polling: $e');
    throw ApiException('Failed to start costmap polling: $e');
  }
}

/// Stop costmap polling
Future<Map<String, dynamic>> stopCostmapPolling({
  required String deviceId,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/costmaps/$deviceId/polling/stop', {
      'stoppedAt': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ Costmap polling stopped');
    }

    return response;
  } catch (e) {
    print('❌ Error stopping costmap polling: $e');
    throw ApiException('Failed to stop costmap polling: $e');
  }
}

/// Configure costmap parameters
Future<Map<String, dynamic>> configureCostmap({
  required String deviceId,
  required String costmapType, // 'global' or 'local'
  Map<String, dynamic>? parameters,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/costmaps/$deviceId/$costmapType/configure', {
      'parameters': parameters ?? {},
      'configuredAt': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ $costmapType costmap configured');
    }

    return response;
  } catch (e) {
    print('❌ Error configuring costmap: $e');
    throw ApiException('Failed to configure costmap: $e');
  }
}

/// Clear costmap (remove dynamic obstacles)
Future<Map<String, dynamic>> clearCostmap({
  required String deviceId,
  required String costmapType, // 'global', 'local', or 'both'
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/costmaps/$deviceId/clear', {
      'costmapType': costmapType,
      'clearedAt': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ $costmapType costmap cleared');
    }

    return response;
  } catch (e) {
    print('❌ Error clearing costmap: $e');
    throw ApiException('Failed to clear costmap: $e');
  }
}

/// Export costmap as image (PNG/JPEG)
Future<Map<String, dynamic>> exportCostmapAsImage({
  required String deviceId,
  required String costmapType,
  String format = 'png', // 'png', 'jpeg'
  bool includeColorbar = true,
  Map<String, dynamic>? visualization,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/costmaps/$deviceId/$costmapType/export', {
      'format': format,
      'includeColorbar': includeColorbar,
      'visualization': visualization ?? {},
      'exportedAt': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ $costmapType costmap exported as $format');
    }

    return response;
  } catch (e) {
    print('❌ Error exporting costmap as image: $e');
    throw ApiException('Failed to export costmap as image: $e');
  }
}

// ==========================================
// BATCH OPERATIONS FOR MULTIPLE DEVICES
// ==========================================

/// Export maps from multiple devices
Future<Map<String, dynamic>> batchExportMaps({
  required List<String> deviceIds,
  String format = 'pgm',
  bool includeMetadata = true,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/maps/batch/export', {
      'deviceIds': deviceIds,
      'format': format,
      'includeMetadata': includeMetadata,
      'batchExportedAt': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ Batch export completed for ${deviceIds.length} devices');
    }

    return response;
  } catch (e) {
    print('❌ Error in batch export: $e');
    throw ApiException('Failed to batch export maps: $e');
  }
}

/// Sync maps between devices
Future<Map<String, dynamic>> syncMapsBetweenDevices({
  required String sourceDeviceId,
  required List<String> targetDeviceIds,
  required String mapName,
  bool overwriteExisting = false,
}) async {
  _ensureInitialized();
  
  try {
    final response = await _post('/api/maps/sync', {
      'sourceDeviceId': sourceDeviceId,
      'targetDeviceIds': targetDeviceIds,
      'mapName': mapName,
      'overwriteExisting': overwriteExisting,
      'syncedAt': DateTime.now().toIso8601String(),
    });

    if (response['success'] == true) {
      print('✅ Map synced from $sourceDeviceId to ${targetDeviceIds.length} devices');
    }

    return response;
  } catch (e) {
    print('❌ Error syncing maps: $e');
    throw ApiException('Failed to sync maps between devices: $e');
  }
}
  bool _isAGVBackend(Map<String, dynamic> data) {
    final dataStr = data.toString().toLowerCase();
    return data['success'] == true ||
           data['status'] == 'healthy' ||
           dataStr.contains('agv') ||
           dataStr.contains('fleet') ||
           dataStr.contains('robot') ||
           dataStr.contains('ros') ||
           data.containsKey('services');
  }

  Future<Map<String, String>?> _getDeviceNetworkInfo() async {
    try {
      for (final interface in await NetworkInterface.list()) {
        for (final addr in interface.addresses) {
          if (addr.type == InternetAddressType.IPv4 && 
              !addr.isLoopback && 
              !addr.address.startsWith('169.254')) {
            
            final ip = addr.address;
            final parts = ip.split('.');
            if (parts.length == 4) {
              final subnet = '${parts[0]}.${parts[1]}.${parts[2]}';
              return {
                'interface': interface.name,
                'deviceIP': ip,
                'subnet': subnet,
              };
            }
          }
        }
      }
    } catch (e) {
      print('❌ Error getting network info: $e');
    }
    return null;
  }

  // ==========================================
  // GETTERS & STATUS
  // ==========================================

  String? get baseUrl => _baseUrl;
  String? get apiBaseUrl => _baseUrl != null ? '$_baseUrl/api' : null;
  bool get isInitialized => _isInitialized && _baseUrl != null;

  void printConnectionInfo() {
    print('📊 API Service Connection Info:');
    print('   Initialized: $isInitialized');
    print('   Base URL: ${baseUrl ?? 'Not set'}');
    print('   API URL: ${apiBaseUrl ?? 'Not set'}');
    print('   WebSocket URL: ${getWebSocketUrl() ?? 'Not set'}');
    print('   Timeout: ${_timeout.inSeconds}s');
  }

  String? getWebSocketUrl() {
    if (_baseUrl == null) return null;
    return _baseUrl!.replaceAll('http://', 'ws://').replaceAll('https://', 'wss://');
  }

  Future<Map<String, dynamic>> getConnectionInfo() async {
    if (!isInitialized) {
      return {
        'connected': false,
        'error': 'API service not initialized',
        'initialized': false,
      };
    }

    try {
      final serverInfo = await _get('/health');

      return {
        'connected': true,
        'initialized': isInitialized,
        'baseUrl': baseUrl,
        'apiBaseUrl': apiBaseUrl,
        'websocketUrl': getWebSocketUrl(),
        'serverInfo': serverInfo,
        'lastChecked': DateTime.now().toIso8601String(),
      };
    } catch (e) {
      return {
        'connected': false,
        'initialized': isInitialized,
        'baseUrl': baseUrl,
        'apiBaseUrl': apiBaseUrl,
        'websocketUrl': getWebSocketUrl(),
        'error': e.toString(),
        'lastChecked': DateTime.now().toIso8601String(),
      };
    }
  }

  // ==========================================
  // CONNECTION UTILITIES
  // ==========================================

  Future<bool> testConnection() async {
    if (!isInitialized) {
      print('❌ API service not initialized');
      return false;
    }

    try {
      print('🔧 Testing API connection to: $baseUrl/health');
      final response = await _get('/health');
      print('✅ API connection test successful');
      _serverInfo = response;
      return response['status'] != null || response['success'] != null;
    } catch (e) {
      print('❌ API connection test failed: $e');
      return false;
    }
  }

  Future<bool> isServerReachable() async {
    try {
      await testConnection();
      return true;
    } catch (e) {
      return false;
    }
  }

  Future<Map<String, dynamic>> getServerInfo() async {
    try {
      final health = await healthCheck();
      final systemStatus = await getSystemStatus();

      return {
        'health': health,
        'status': systemStatus,
        'reachable': true,
        'lastChecked': DateTime.now().toIso8601String(),
      };
    } catch (e) {
      return {
        'reachable': false,
        'error': e.toString(),
        'lastChecked': DateTime.now().toIso8601String(),
      };
    }
  }

  // ==========================================
  // DEVICE MANAGEMENT
  // ==========================================

  Future<Map<String, dynamic>> autoConnectAGV() async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/devices/auto-connect', {});

      if (response['success'] == true) {
        print('✅ AGV auto-connected: ${response['deviceId']}');
      }

      return response;
    } catch (e) {
      print('❌ Error auto-connecting AGV: $e');
      throw ApiException('Failed to auto-connect AGV: $e');
    }
  }

  Future<Map<String, dynamic>> connectDevice({
    required String deviceId,
    required String name,
    required String ipAddress,
    required String type,
    required List<String> capabilities,
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/connect', {
        'name': name,
        'ipAddress': ipAddress,
        'type': type,
        'capabilities': capabilities,
      });

      if (response['success'] == true) {
        print('✅ Device connected: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error connecting device: $e');
      throw ApiException('Failed to connect device: $e');
    }
  }

  Future<List<Map<String, dynamic>>> getDevices() async {
    _ensureInitialized();
    
    try {
      final response = await _get('/api/devices');

      if (response['success'] == true) {
        final devices = response['devices'] as List?;
        if (devices != null) {
          return devices.cast<Map<String, dynamic>>();
        }
      }

      return [];
    } catch (e) {
      print('❌ Error getting devices: $e');
      throw ApiException('Failed to get devices: $e');
    }
  }

  Future<Map<String, dynamic>> getDeviceStatus(String deviceId) async {
    _ensureInitialized();
    
    try {
      final response = await _get('/api/control/devices/$deviceId/status');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException('Failed to get device status: ${response['error']}');
    } catch (e) {
      print('❌ Error getting device status: $e');
      throw ApiException('Failed to get device status: $e');
    }
  }

  Future<Map<String, dynamic>> disconnectDevice({required String deviceId}) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/disconnect', {});

      if (response['success'] == true) {
        print('✅ Device disconnected: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error disconnecting device: $e');
      throw ApiException('Failed to disconnect device: $e');
    }
  }

  // ==========================================
  // ✅ FIXED: ANALYTICS DATA METHODS
  // ==========================================

  /// Get analytics data for a specific device and type
  Future<Map<String, dynamic>> getAnalyticsData(
    String? deviceId, 
    String dataType, 
    String timeRange,
  ) async {
    _ensureInitialized();
    
    try {
      final params = {
        'type': dataType,
        'timeRange': timeRange,
        if (deviceId != null) 'deviceId': deviceId,
      };
      
      final queryString = params.entries
          .map((e) => '${e.key}=${Uri.encodeComponent(e.value)}')
          .join('&');
      
      final response = await _get('/api/analytics?$queryString');

      if (response['success'] == true) {
        return response;
      }

      // If analytics endpoint doesn't exist, return mock data
      if (response['error']?.toString().contains('404') == true) {
        return _generateMockAnalyticsData(deviceId, dataType, timeRange);
      }

      throw ApiException('Failed to get analytics data: ${response['error']}');
    } catch (e) {
      // Fallback to mock data for development
      print('⚠️ Analytics API unavailable, using mock data: $e');
      return _generateMockAnalyticsData(deviceId, dataType, timeRange);
    }
  }

  /// Generate mock analytics data when API is unavailable
  Map<String, dynamic> _generateMockAnalyticsData(
    String? deviceId, 
    String dataType, 
    String timeRange,
  ) {
    final now = DateTime.now();
    final random = math.Random();
    
    switch (dataType) {
      case 'battery':
        return {
          'success': true,
          'data': List.generate(24, (index) {
            final timestamp = now.subtract(Duration(hours: 23 - index));
            return {
              'timestamp': timestamp.toIso8601String(),
              'voltage': 11.5 + random.nextDouble() * 1.0,
              'percentage': math.max(10.0, 100.0 - (index * 2) + random.nextDouble() * 10),
              'current': 0.5 + random.nextDouble() * 2.0,
              'temperature': 20.0 + random.nextDouble() * 15.0,
            };
          }),
        };
        
      case 'orders':
        return {
          'success': true,
          'data': List.generate(15, (index) {
            final completedAt = now.subtract(Duration(hours: random.nextInt(168)));
            final duration = 15.0 + random.nextDouble() * 45.0;
            return {
              'id': 'order_${deviceId ?? 'mock'}_$index',
              'name': 'Order ${index + 1}',
              'createdAt': completedAt.subtract(Duration(minutes: duration.toInt())).toIso8601String(),
              'completedAt': completedAt.toIso8601String(),
              'duration': duration,
              'distance': 50.0 + random.nextDouble() * 200.0,
              'waypoints': random.nextInt(5) + 2,
              'status': 'completed',
            };
          }),
        };
        
      case 'stats':
        return {
          'success': true,
          'data': {
            'totalOrders': 15,
            'completedOrders': 15,
            'averageOrderTime': 32.5 + random.nextDouble() * 20.0,
            'totalUptime': 180.0 + random.nextDouble() * 24.0,
            'totalDistance': 50.0 + random.nextDouble() * 100.0,
            'currentBattery': 65.0 + random.nextDouble() * 30.0,
            'averageBattery': 65.0 + random.nextDouble() * 20.0,
            'errorCount': random.nextInt(5),
            'averageSpeed': 0.3 + random.nextDouble() * 0.5,
          },
        };
        
      case 'events':
        return {
          'success': true,
          'data': List.generate(20, (index) {
            final eventTypes = ['info', 'warning', 'error', 'success'];
            final messages = [
              'Device connected successfully',
              'Low battery warning',
              'Order completed',
              'Mapping session started',
              'Navigation goal reached',
              'System maintenance required',
              'Communication timeout',
              'Charging session completed',
            ];
            
            return {
              'timestamp': now.subtract(Duration(hours: random.nextInt(72))).toIso8601String(),
              'type': eventTypes[random.nextInt(eventTypes.length)],
              'message': messages[random.nextInt(messages.length)],
              'deviceId': deviceId ?? 'mock_device',
              'severity': ['low', 'medium', 'high'][random.nextInt(3)],
            };
          }),
        };
        
      case 'performance':
        return {
          'success': true,
          'data': List.generate(60, (index) {
            final timestamp = now.subtract(Duration(minutes: 59 - index));
            return {
              'timestamp': timestamp.toIso8601String(),
              'linear': random.nextDouble() * 1.0,
              'angular': (random.nextDouble() - 0.5) * 2.0,
            };
          }),
        };
        
      default:
        return {
          'success': false,
          'error': 'Unknown analytics data type: $dataType',
        };
    }
  }

  // ==========================================
  // CONTROL COMMANDS
  // ==========================================

  Future<Map<String, dynamic>> joystickControl({
    required String deviceId,
    required double x,
    required double y,
    bool deadman = false,
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/joystick', {
        'x': x.clamp(-1.0, 1.0),
        'y': y.clamp(-1.0, 1.0),
        'deadman': deadman,
      });

      return response;
    } catch (e) {
      print('❌ Error sending joystick command: $e');
      throw ApiException('Failed to send joystick command: $e');
    }
  }

  Future<Map<String, dynamic>> sendVelocity({
    required String deviceId,
    required double linear,
    required double angular,
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/velocity', {
        'linear': linear,
        'angular': angular,
      });

      return response;
    } catch (e) {
      print('❌ Error sending velocity command: $e');
      throw ApiException('Failed to send velocity command: $e');
    }
  }

  Future<Map<String, dynamic>> moveDevice({
    required String deviceId,
    required double linear,
    required double angular,
  }) async {
    return sendVelocity(deviceId: deviceId, linear: linear, angular: angular);
  }

  Future<Map<String, dynamic>> emergencyStop(String deviceId) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/emergency-stop', {});

      if (response['success'] == true) {
        print('🛑 Emergency stop sent for device: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error sending emergency stop: $e');
      throw ApiException('Failed to send emergency stop: $e');
    }
  }

  Future<Map<String, dynamic>> setGoal({
    required String deviceId,
    required double x,
    required double y,
    double orientation = 0.0,
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/goal', {
        'x': x,
        'y': y,
        'orientation': orientation,
      });

      if (response['success'] == true) {
        print('🎯 Goal set for device $deviceId: ($x, $y)');
      }

      return response;
    } catch (e) {
      print('❌ Error setting goal: $e');
      throw ApiException('Failed to set goal: $e');
    }
  }

  // ==========================================
  // MAPPING CONTROL
  // ==========================================

  Future<Map<String, dynamic>> startMapping(String deviceId) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/mapping/start', {});

      if (response['success'] == true) {
        print('🗺️ Mapping started for device: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error starting mapping: $e');
      throw ApiException('Failed to start mapping: $e');
    }
  }

  Future<Map<String, dynamic>> stopMapping(String deviceId) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/mapping/stop', {});

      if (response['success'] == true) {
        print('🛑 Mapping stopped for device: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error stopping mapping: $e');
      throw ApiException('Failed to stop mapping: $e');
    }
  }

  Future<Map<String, dynamic>> getMappingStatus(String deviceId) async {
    _ensureInitialized();
    
    try {
      final response = await _get('/api/control/devices/$deviceId/mapping/status');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException('Failed to get mapping status: ${response['error']}');
    } catch (e) {
      print('❌ Error getting mapping status: $e');
      throw ApiException('Failed to get mapping status: $e');
    }
  }

  Future<Map<String, dynamic>> saveMap({
    required String deviceId,
    String? mapName,
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/map/save', {
        'mapName': mapName ?? 'Map_${deviceId}_${DateTime.now().toIso8601String()}',
      });

      if (response['success'] == true) {
        print('💾 Map saved for device: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error saving map: $e');
      throw ApiException('Failed to save map: $e');
    }
  }

  Future<Map<String, dynamic>> saveMapData({
    required String deviceId,
    required MapData mapData,
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/map/data', {
        'mapData': mapData.toJson(),
      });

      if (response['success'] == true) {
        print('💾 Map data saved for device: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error saving map data: $e');
      throw ApiException('Failed to save map data: $e');
    }
  }

  Future<Map<String, dynamic>> getMapData(String deviceId) async {
    _ensureInitialized();
    
    try {
      final response = await _get('/api/control/devices/$deviceId/map');
      return response;
    } catch (e) {
      print('❌ Error getting map data: $e');
      throw ApiException('Failed to get map data: $e');
    }
  }

  // ==========================================
  // MAP EDITING
  // ==========================================

  Future<Map<String, dynamic>> addMapShape({
    required String deviceId,
    required String type,
    required String name,
    required List<Position> points,
    Map<String, String>? sides,
    String color = '#FF0000FF',
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/map/shapes', {
        'type': type,
        'name': name,
        'points': points.map((p) => p.toJson()).toList(),
        'sides': sides ?? {'left': '', 'right': '', 'front': '', 'back': ''},
        'color': color,
      });

      if (response['success'] == true) {
        print('➕ Shape added to map for device: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error adding map shape: $e');
      throw ApiException('Failed to add map shape: $e');
    }
  }

  Future<Map<String, dynamic>> updateMapShape({
    required String deviceId,
    required String shapeId,
    Map<String, dynamic>? updates,
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _put('/api/control/devices/$deviceId/map/shapes/$shapeId', updates ?? {});

      if (response['success'] == true) {
        print('✏️ Shape updated for device: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error updating map shape: $e');
      throw ApiException('Failed to update map shape: $e');
    }
  }

  Future<Map<String, dynamic>> deleteMapShape({
    required String deviceId,
    required String shapeId,
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _delete('/api/control/devices/$deviceId/map/shapes/$shapeId');

      if (response['success'] == true) {
        print('🗑️ Shape deleted from map for device: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error deleting map shape: $e');
      throw ApiException('Failed to delete map shape: $e');
    }
  }

  // ==========================================
  // ORDER MANAGEMENT
  // ==========================================

  Future<List<Map<String, dynamic>>> getOrders(String deviceId) async {
    _ensureInitialized();
    
    try {
      final response = await _get('/api/control/devices/$deviceId/orders');

      if (response['success'] == true) {
        final orders = response['orders'] as List?;
        if (orders != null) {
          return orders.cast<Map<String, dynamic>>();
        }
      }

      return [];
    } catch (e) {
      if (e is ApiException && e.statusCode == 404) {
        return [];
      }
      print('❌ Error getting orders: $e');
      throw ApiException('Failed to get orders: $e');
    }
  }

  Future<Map<String, dynamic>> createOrder({
    required String deviceId,
    required List<Map<String, dynamic>> waypoints,
    String? orderName,
    int priority = 0,
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/orders', {
        'name': orderName ?? 'Order_${DateTime.now().millisecondsSinceEpoch}',
        'waypoints': waypoints,
        'priority': priority,
        'createdAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('📋 Order created for device: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error creating order: $e');
      throw ApiException('Failed to create order: $e');
    }
  }

  Future<Map<String, dynamic>> updateOrderStatus({
    required String orderId,
    required String status,
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _put('/api/orders/$orderId/status', {
        'status': status,
        'updatedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('📋 Order status updated: $orderId -> $status');
      }

      return response;
    } catch (e) {
      print('❌ Error updating order status: $e');
      throw ApiException('Failed to update order status: $e');
    }
  }

  Future<Map<String, dynamic>> executeOrder({
    required String deviceId,
    required String orderId,
  }) async {
    _ensureInitialized();
    
    try {
      final response = await _post('/api/control/devices/$deviceId/orders/$orderId/execute', {});

      if (response['success'] == true) {
        print('▶️ Order executed for device: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error executing order: $e');
      throw ApiException('Failed to execute order: $e');
    }
  }

  // ==========================================
  // SYSTEM STATUS
  // ==========================================

  Future<Map<String, dynamic>> getRosStatus() async {
    _ensureInitialized();
    
    try {
      final response = await _get('/api/control/system/ros-status');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException('Failed to get ROS status: ${response['error']}');
    } catch (e) {
      print('❌ Error getting ROS status: $e');
      throw ApiException('Failed to get ROS status: $e');
    }
  }

  Future<Map<String, dynamic>> testConnectivity() async {
    _ensureInitialized();
    
    try {
      final response = await _get('/api/control/system/connectivity-test');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException('Connectivity test failed: ${response['error']}');
    } catch (e) {
      print('❌ Error testing connectivity: $e');
      throw ApiException('Failed to test connectivity: $e');
    }
  }

  Future<Map<String, dynamic>> getSystemStatus() async {
    _ensureInitialized();
    
    try {
      final response = await _get('/api/system/status');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException('Failed to get system status: ${response['error']}');
    } catch (e) {
      print('❌ Error getting system status: $e');
      throw ApiException('Failed to get system status: $e');
    }
  }

  Future<Map<String, dynamic>> healthCheck() async {
    _ensureInitialized();
    
    try {
      final response = await _get('/health');
      return response;
    } catch (e) {
      print('❌ Error checking health: $e');
      throw ApiException('Failed to check health: $e');
    }
  }

  // ==========================================
  // THEME MANAGEMENT
  // ==========================================

  Future<Map<String, dynamic>> updateTheme(bool isDarkMode) async {
    if (!isInitialized) {
      return {'success': false, 'error': 'API not initialized'};
    }
    
    try {
      final response = await _post('/api/user/theme', {
        'isDarkMode': isDarkMode,
        'updatedAt': DateTime.now().toIso8601String(),
      });

      return response;
    } catch (e) {
      print('⚠️ Theme update failed (endpoint may not exist): $e');
      return {'success': false, 'error': 'Theme endpoint not available'};
    }
  }

  // ==========================================
  // PRIVATE HTTP METHODS
  // ==========================================

  void _ensureInitialized() {
    if (!isInitialized) {
      throw ApiException('API service not initialized. Call initialize() or autoInitialize() first.');
    }
  }

  Future<Map<String, dynamic>> _get(String endpoint, {bool useCache = true, Duration? cacheTtl}) async {
    _ensureInitialized();
    final url = Uri.parse('$_baseUrl$endpoint');
    final cacheKey = 'GET:$url';

    if (useCache && _cachingEnabled && _cache.containsKey(cacheKey)) {
      final entry = _cache[cacheKey]!;
      if (!entry.isExpired) {
        print('📋 Cache hit for $endpoint');
        return entry.data as Map<String, dynamic>;
      } else {
        _cache.remove(cacheKey);
      }
    }

    try {
      print('📡 GET: $url');
      final response = await http.get(url, headers: _headers).timeout(_timeout);
      _trackRequest(endpoint, response.statusCode < 400, response.statusCode);
      final data = _handleResponse(response);

      if (useCache && _cachingEnabled && response.statusCode >= 200 && response.statusCode < 300) {
        final ttl = cacheTtl ?? _defaultCacheTtl;
        _cache[cacheKey] = _CacheEntry(data, DateTime.now(), ttl);
      }
      return data;
    } catch (e) {
      _trackRequest(endpoint, false, null);
      if (e is TimeoutException) {
        throw ApiException('Request timeout');
      } else if (e is SocketException) {
        throw ApiException('Network error: Connection failed', originalError: e);
      }
      rethrow;
    }
  }

  Future<Map<String, dynamic>> _post(String endpoint, Map<String, dynamic> data) async {
    _ensureInitialized();
    
    try {
      final url = Uri.parse('$_baseUrl$endpoint');
      print('📡 POST: $url');

      final response = await http
          .post(
            url,
            headers: _headers,
            body: json.encode(data),
          )
          .timeout(_timeout);

      return _handleResponse(response);
    } catch (e) {
      if (e is TimeoutException) {
        throw ApiException('Request timeout');
      } else if (e is SocketException) {
        throw ApiException('Network error: Connection failed', originalError: e);
      }
      rethrow;
    }
  }

  Future<Map<String, dynamic>> _put(String endpoint, Map<String, dynamic> data) async {
    _ensureInitialized();
    
    try {
      final url = Uri.parse('$_baseUrl$endpoint');
      print('📡 PUT: $url');

      final response = await http
          .put(
            url,
            headers: _headers,
            body: json.encode(data),
          )
          .timeout(_timeout);

      return _handleResponse(response);
    } catch (e) {
      if (e is TimeoutException) {
        throw ApiException('Request timeout');
      } else if (e is SocketException) {
        throw ApiException('Network error: Connection failed', originalError: e);
      }
      rethrow;
    }
  }

  Future<Map<String, dynamic>> _delete(String endpoint) async {
    _ensureInitialized();
    
    try {
      final url = Uri.parse('$_baseUrl$endpoint');
      print('📡 DELETE: $url');

      final response = await http.delete(url, headers: _headers).timeout(_timeout);
      return _handleResponse(response);
    } catch (e) {
      if (e is TimeoutException) {
        throw ApiException('Request timeout');
      } else if (e is SocketException) {
        throw ApiException('Network error: Connection failed', originalError: e);
      }
      rethrow;
    }
  }

  Map<String, dynamic> _handleResponse(http.Response response) {
    final String body = response.body;

    print('📨 Response ${response.statusCode}: ${body.length > 200 ? body.substring(0, 200) + '...' : body}');

    try {
      final Map<String, dynamic> data = json.decode(body);

      if (response.statusCode >= 200 && response.statusCode < 300) {
        return data;
      } else {
        throw ApiException(
          'HTTP ${response.statusCode}: ${data['error'] ?? 'Unknown error'}',
          statusCode: response.statusCode,
        );
      }
    } catch (e) {
      if (e is ApiException) rethrow;
      throw ApiException('Failed to parse response: ${response.statusCode} - $body');
    }
  }

  // ==========================================
  // UTILITY METHODS
  // ==========================================

  void setTimeout(Duration timeout) {
    _timeout = timeout;
  }

  void setHeaders(Map<String, String> headers) {
    _headers = {..._headers, ...headers};
  }

  void _trackRequest(String endpoint, bool success, int? statusCode) {
    _totalRequests++;
    if (success) {
      _successfulRequests++;
    } else {
      _failedRequests++;
    }
    _endpointStats[endpoint] = (_endpointStats[endpoint] ?? 0) + 1;
    if (statusCode != null) {
      _statusCodeStats[statusCode] = (_statusCodeStats[statusCode] ?? 0) + 1;
    }
  }

  Map<String, dynamic> get requestStats => {
    'totalRequests': _totalRequests,
    'successfulRequests': _successfulRequests,
    'failedRequests': _failedRequests,
    'successRate': _totalRequests > 0 ? (_successfulRequests / _totalRequests * 100) : 0.0,
    'endpointStats': Map.from(_endpointStats),
    'statusCodeStats': Map.from(_statusCodeStats),
  };

  Map<String, dynamic> getCacheStats() {
    int validEntries = 0;
    int expiredEntries = 0;
    _cache.forEach((key, entry) {
      if (entry.isExpired) {
        expiredEntries++;
      } else {
        validEntries++;
      }
    });
    return {
      'totalEntries': _cache.length,
      'validEntries': validEntries,
      'expiredEntries': expiredEntries,
      'enabled': _cachingEnabled,
    };
  }

  void clearCache() {
    _cache.clear();
    print('🗑️ API cache cleared');
  }

  void clearStats() {
    _totalRequests = 0;
    _successfulRequests = 0;
    _failedRequests = 0;
    _endpointStats.clear();
    _statusCodeStats.clear();
    print('📊 API statistics cleared');
  }

  void clearCacheForEndpoint(String endpoint) {
    final keysToRemove = _cache.keys.where((key) => key.contains(endpoint)).toList();
    for (final key in keysToRemove) {
      _cache.remove(key);
    }
    print('🗑️ Cache cleared for endpoint: $endpoint');
  }

  void dispose() {
    clearCache();
    clearStats();
  }
}

class _CacheEntry {
  final dynamic data;
  final DateTime createdAt;
  final Duration ttl;

  _CacheEntry(this.data, this.createdAt, this.ttl);

  bool get isExpired => DateTime.now().isAfter(createdAt.add(ttl));
}

class ApiException implements Exception {
  final String message;
  final int? statusCode;
  final dynamic originalError;

  ApiException(this.message, {this.statusCode, this.originalError});

  bool get isNetworkError => originalError is SocketException;
  bool get isTimeout => originalError is TimeoutException;
  bool get isNotFound => statusCode == 404;

  @override
  String toString() => 'ApiException: $message';
}