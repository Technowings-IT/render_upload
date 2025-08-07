// services/api_service.dart - Enhanced with Advanced Map Editing Features
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'dart:typed_data';
import 'package:http/http.dart' as http;
import '../models/map_data.dart';
import '../models/odom.dart';

/// Custom exception for API errors
class ApiException implements Exception {
  final String message;
  final int? statusCode;
  final dynamic originalError;
  bool get isNotFound => statusCode == 404;

  ApiException(this.message, {this.statusCode, this.originalError});

  @override
  String toString() =>
      'ApiException: $message${statusCode != null ? ' (HTTP $statusCode)' : ''}';
}

class _CacheEntry {
  final dynamic data;
  final DateTime createdAt;
  final Duration ttl;

  _CacheEntry(this.data, this.createdAt, this.ttl);

  bool get isExpired => DateTime.now().difference(createdAt) > ttl;
}

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
      print('🔍 Auto-discovering AMR backend...');

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
      print('📡 Scanning subnet: $subnet.x for AMR backends');

      final commonIPs = [
        '$subnet.63', // Current working backend IP
        '$subnet.136',
        '$subnet.100',
        '$subnet.101',
        '$subnet.110',
        '$subnet.200',
      ];

      for (final ip in commonIPs) {
        final url = await _testBackendAtIP(ip);
        if (url != null) {
          print('✅ Found AMR backend at: $url');
          return url;
        }
      }

      for (int i = 70; i <= 90; i++) {
        final ip = '$subnet.$i';
        final url = await _testBackendAtIP(ip);
        if (url != null) {
          print('✅ Found AMR backend at: $url');
          return url;
        }
      }

      print('❌ No AMR backend found on network');
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

          if (_isAMRBackend(data)) {
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
        'mapName':
            mapName ?? 'exported_map_${DateTime.now().millisecondsSinceEpoch}',
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

  /// Convert JSON map to PGM+YAML format
  Future<Map<String, dynamic>> convertMapToPGM({
    required String deviceId,
    String? mapName,
    String? sourceMapName,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/convert-to-pgm', {
        'mapName':
            mapName ?? 'converted_${DateTime.now().millisecondsSinceEpoch}',
        'sourceMapName': sourceMapName,
        'convertedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Map converted to PGM: ${response['outputMapName']}');
      }

      return response;
    } catch (e) {
      print('❌ Error converting map to PGM: $e');
      throw ApiException('Failed to convert map to PGM: $e');
    }
  }

  /// Edit PGM map by adding location markers
  Future<Map<String, dynamic>> editPGMMap({
    required String deviceId,
    required String mapName,
    required List<Map<String, dynamic>> locations,
    Map<String, dynamic>? metadata,
  }) async {
    _ensureInitialized();

    try {
      final response = await _put('/api/maps/$deviceId/edit-pgm/$mapName', {
        'locations': locations,
        'metadata': metadata ?? {},
        'editedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ PGM map edited: $mapName with ${locations.length} locations');
      }

      return response;
    } catch (e) {
      print('❌ Error editing PGM map: $e');
      throw ApiException('Failed to edit PGM map: $e');
    }
  }

  /// Rename map for deployment
  Future<Map<String, dynamic>> renameMapForDeployment({
    required String deviceId,
    required String sourceMapName,
    required String targetMapName,
    String? deploymentNotes,
  }) async {
    _ensureInitialized();

    try {
      final response =
          await _post('/api/maps/$deviceId/rename-for-deployment', {
        'sourceMapName': sourceMapName,
        'targetMapName': targetMapName,
        'deploymentNotes': deploymentNotes ?? '',
        'preparedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Map renamed for deployment: $sourceMapName -> $targetMapName');
      }

      return response;
    } catch (e) {
      print('❌ Error renaming map for deployment: $e');
      throw ApiException('Failed to rename map for deployment: $e');
    }
  }

  /// Upload map to AMR (send PGM + YAML back to ROS)
  Future<Map<String, dynamic>> uploadMapToAMR({
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
        print('✅ Map uploaded to AMR: $mapName');
      }

      return response;
    } catch (e) {
      print('❌ Error uploading map to AMR: $e');
      throw ApiException('Failed to upload map to AMR: $e');
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

// ==========================================
// RASPBERRY PI DEPLOYMENT METHODS
// ==========================================

  /// Deploy map to Raspberry Pi via SSH
  Future<Map<String, dynamic>> deployMapToRaspberryPi({
    required String deviceId,
    required String mapName,
    Map<String, dynamic>? piConfig,
    bool autoLoad = true,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/deploy-to-pi', {
        'mapName': mapName,
        'piConfig': piConfig,
        'autoLoad': autoLoad,
        'deployedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Map deployed to Raspberry Pi: $mapName');
      }

      return response;
    } catch (e) {
      print('❌ Error deploying map to Raspberry Pi: $e');
      throw ApiException('Failed to deploy map to Raspberry Pi: $e');
    }
  }

  /// Get available maps on Raspberry Pi
  Future<List<Map<String, dynamic>>> getAvailablePiMaps(String deviceId) async {
    _ensureInitialized();

    try {
      final response = await _get('/api/maps/$deviceId/pi-maps');

      if (response['success'] == true) {
        final maps = response['maps'] as List?;
        if (maps != null) {
          return maps.cast<Map<String, dynamic>>();
        }
      }

      return [];
    } catch (e) {
      print('❌ Error getting Pi maps: $e');
      throw ApiException('Failed to get Pi maps: $e');
    }
  }

  /// Set active map on Raspberry Pi
  Future<Map<String, dynamic>> setActivePiMap({
    required String deviceId,
    required String mapName,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/set-active-pi-map', {
        'mapName': mapName,
        'setAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Active Pi map set: $mapName');
      }

      return response;
    } catch (e) {
      print('❌ Error setting active Pi map: $e');
      throw ApiException('Failed to set active Pi map: $e');
    }
  }

  /// Get Raspberry Pi status
  Future<Map<String, dynamic>> getRaspberryPiStatus(String deviceId) async {
    _ensureInitialized();

    try {
      final response = await _get('/api/maps/$deviceId/pi-status');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException('Failed to get Pi status: ${response['error']}');
    } catch (e) {
      print('❌ Error getting Pi status: $e');
      throw ApiException('Failed to get Pi status: $e');
    }
  }

  /// Test SSH connection to Raspberry Pi
  Future<Map<String, dynamic>> testPiConnection({
    required String deviceId,
    Map<String, dynamic>? piConfig,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/test-pi-connection', {
        'piConfig': piConfig,
        'testedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Pi connection test successful');
      }

      return response;
    } catch (e) {
      print('❌ Error testing Pi connection: $e');
      throw ApiException('Failed to test Pi connection: $e');
    }
  }

  /// Get Pi map loading logs
  Future<Map<String, dynamic>> getPiMapLogs(String deviceId) async {
    _ensureInitialized();

    try {
      final response = await _get('/api/maps/$deviceId/pi-logs');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException('Failed to get Pi logs: ${response['error']}');
    } catch (e) {
      print('❌ Error getting Pi logs: $e');
      throw ApiException('Failed to get Pi logs: $e');
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
      final response =
          await _get('/api/maps/$deviceId/costmaps?type=$costmapType');

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
    required String
        operation, // 'erode', 'dilate', 'open', 'close', 'blur', 'sharpen'
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
    String mergeStrategy =
        'overlay', // 'overlay', 'average', 'maximum', 'minimum'
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
      final response = await _delete(
          '/api/maps/$deviceId/$mapName?deleteAllFormats=$deleteAllFormats');

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
      final response =
          await _get('/api/maps/$deviceId/conversions/$conversionId/status');
      return response;
    } catch (e) {
      print('❌ Error getting conversion status: $e');
      throw ApiException('Failed to get conversion status: $e');
    }
  }

  /// Set map as active on the AMR
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
// COMPLETE MAP EDITING WORKFLOW
// ==========================================

  /// Complete workflow: Convert -> Edit -> Rename -> Deploy
  Future<Map<String, dynamic>> completeMapDeploymentWorkflow({
    required String deviceId,
    String? sourceMapName,
    required String finalMapName,
    required List<Map<String, dynamic>> locations,
    Map<String, dynamic>? piConfig,
    String? deploymentNotes,
    bool autoLoad = true,
  }) async {
    _ensureInitialized();

    try {
      print('🚀 Starting complete map deployment workflow...');

      final workflowResults = <String, dynamic>{};

      // Step 1: Convert to PGM
      print('📋 Step 1: Converting to PGM format...');
      final convertResult = await convertMapToPGM(
        deviceId: deviceId,
        sourceMapName: sourceMapName,
        mapName: 'temp_${DateTime.now().millisecondsSinceEpoch}',
      );

      if (convertResult['success'] != true) {
        throw ApiException('Conversion failed: ${convertResult['error']}');
      }

      workflowResults['conversion'] = convertResult;
      final tempMapName = convertResult['outputMapName'];

      // Step 2: Edit map with locations
      print('📋 Step 2: Adding location markers...');
      final editResult = await editPGMMap(
        deviceId: deviceId,
        mapName: tempMapName,
        locations: locations,
      );

      if (editResult['success'] != true) {
        throw ApiException('Editing failed: ${editResult['error']}');
      }

      workflowResults['editing'] = editResult;

      // Step 3: Rename for deployment
      print('📋 Step 3: Renaming for deployment...');
      final renameResult = await renameMapForDeployment(
        deviceId: deviceId,
        sourceMapName: tempMapName,
        targetMapName: finalMapName,
        deploymentNotes: deploymentNotes,
      );

      if (renameResult['success'] != true) {
        throw ApiException('Renaming failed: ${renameResult['error']}');
      }

      workflowResults['renaming'] = renameResult;

      // Step 4: Deploy to Raspberry Pi
      print('📋 Step 4: Deploying to Raspberry Pi...');
      final deployResult = await deployMapToRaspberryPi(
        deviceId: deviceId,
        mapName: finalMapName,
        piConfig: piConfig,
        autoLoad: autoLoad,
      );

      if (deployResult['success'] != true) {
        throw ApiException('Deployment failed: ${deployResult['error']}');
      }

      workflowResults['deployment'] = deployResult;

      print('✅ Complete map deployment workflow finished successfully!');

      return {
        'success': true,
        'message': 'Complete workflow executed successfully',
        'deviceId': deviceId,
        'finalMapName': finalMapName,
        'locationsAdded': locations.length,
        'autoLoadEnabled': autoLoad,
        'completedAt': DateTime.now().toIso8601String(),
        'workflowResults': workflowResults,
      };
    } catch (e) {
      print('❌ Map deployment workflow failed: $e');
      throw ApiException('Map deployment workflow failed: $e');
    }
  }

// ==========================================
// ENHANCED MAP CONVERSION & EDITING METHODS
// ==========================================

  /// Advanced PGM conversion with optimization options
  Future<Map<String, dynamic>> convertMapToPGMAdvanced({
    required String deviceId,
    String? mapName,
    String? sourceMapName,
    bool includeMetadata = true,
    bool optimizeForEditing = true,
  }) async {
    _ensureInitialized();

    try {
      final response =
          await _post('/api/maps/$deviceId/convert-to-pgm-advanced', {
        'mapName': mapName,
        'sourceMapName': sourceMapName,
        'includeMetadata': includeMetadata,
        'optimizeForEditing': optimizeForEditing,
        'conversionOptions': {
          'preserveDetails': true,
          'enhanceContrast': false,
          'smoothing': false,
        },
      });

      if (response['success'] == true) {
        print('✅ Advanced PGM conversion completed');
      }

      return response;
    } catch (e) {
      print('❌ Error in advanced PGM conversion: $e');
      throw ApiException('Advanced PGM conversion failed: $e');
    }
  }

  /// Save location points to map
  Future<Map<String, dynamic>> saveLocationPoints({
    required String deviceId,
    required String mapName,
    required List<Map<String, dynamic>> locationPoints,
    Map<String, dynamic>? metadata,
  }) async {
    _ensureInitialized();

    try {
      final response = await _put('/api/maps/$deviceId/edit-pgm/$mapName', {
        'locations': locationPoints,
        'metadata': metadata ?? {},
        'editedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Location points saved: ${locationPoints.length} points');
      }

      return response;
    } catch (e) {
      print('❌ Error saving location points: $e');
      throw ApiException('Failed to save location points: $e');
    }
  }

  /// Apply comprehensive map edits
  Future<Map<String, dynamic>> applyMapEdits({
    required String deviceId,
    required String mapName,
    required Uint8List editedMapData,
    required int width,
    required int height,
    List<Map<String, dynamic>>? locationPoints,
    Map<String, dynamic>? editHistory,
  }) async {
    _ensureInitialized();

    try {
      // Convert Uint8List to base64 for JSON transmission
      final base64Data = base64Encode(editedMapData);

      final response = await _post('/api/maps/$deviceId/apply-edits', {
        'mapName': mapName,
        'editedData': base64Data,
        'dimensions': {
          'width': width,
          'height': height,
        },
        'locationPoints': locationPoints ?? [],
        'editHistory': editHistory ?? {},
        'editedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Map edits applied successfully');
      }

      return response;
    } catch (e) {
      print('❌ Error applying map edits: $e');
      throw ApiException('Failed to apply map edits: $e');
    }
  }

  /// Deploy map with validation and backup
  Future<Map<String, dynamic>> deployMapWithValidation({
    required String deviceId,
    required String mapName,
    String? targetMapName,
    String? deploymentNotes,
    bool autoLoad = true,
    bool validateBeforeDeploy = true,
    Map<String, dynamic>? piConfig,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/deploy-validated', {
        'mapName': mapName,
        'targetMapName': targetMapName,
        'deploymentNotes': deploymentNotes,
        'autoLoad': autoLoad,
        'validateBeforeDeploy': validateBeforeDeploy,
        'piConfig': piConfig,
        'deploymentOptions': {
          'backupExisting': true,
          'verifyTransfer': true,
          'testLoad': validateBeforeDeploy,
        },
      });

      if (response['success'] == true) {
        print('✅ Map deployed with validation');
      }

      return response;
    } catch (e) {
      print('❌ Error in validated deployment: $e');
      throw ApiException('Validated deployment failed: $e');
    }
  }

  /// Create map backup for versioning
  Future<Map<String, dynamic>> createMapBackup({
    required String deviceId,
    required String mapName,
    String? backupNote,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/create-backup', {
        'mapName': mapName,
        'backupNote': backupNote,
        'timestamp': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Map backup created');
      }

      return response;
    } catch (e) {
      print('❌ Error creating backup: $e');
      throw ApiException('Backup creation failed: $e');
    }
  }

  /// Restore map from backup
  Future<Map<String, dynamic>> restoreMapFromBackup({
    required String deviceId,
    required String backupId,
    String? targetMapName,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/restore-backup', {
        'backupId': backupId,
        'targetMapName': targetMapName,
        'restoreTimestamp': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Map restored from backup');
      }

      return response;
    } catch (e) {
      print('❌ Error restoring backup: $e');
      throw ApiException('Backup restoration failed: $e');
    }
  }

  /// Advanced export with multiple format support
  Future<Map<String, dynamic>> exportMapAdvanced({
    required String deviceId,
    required String mapName,
    required String format, // 'pgm', 'png', 'pdf', 'svg'
    Map<String, dynamic>? exportOptions,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/export-advanced', {
        'mapName': mapName,
        'format': format,
        'exportOptions': exportOptions ??
            {
              'includeLocationPoints': true,
              'includeMetadata': true,
              'includeGrid': false,
              'includeScale': true,
              'quality': 'high',
            },
      });

      if (response['success'] == true) {
        print('✅ Advanced export completed: $format');
      }

      return response;
    } catch (e) {
      print('❌ Error in advanced export: $e');
      throw ApiException('Advanced export failed: $e');
    }
  }

  /// Analyze map for quality and connectivity
  Future<Map<String, dynamic>> analyzeMap({
    required String deviceId,
    required String mapName,
    List<String>? analysisTypes,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/analyze', {
        'mapName': mapName,
        'analysisTypes': analysisTypes ??
            [
              'coverage',
              'connectivity',
              'obstacles',
              'pathfinding',
              'locationPoints',
            ],
      });

      if (response['success'] == true) {
        print('✅ Map analysis completed');
      }

      return response;
    } catch (e) {
      print('❌ Error analyzing map: $e');
      throw ApiException('Map analysis failed: $e');
    }
  }

  /// Validate map quality and navigation safety
  Future<Map<String, dynamic>> validateMapQuality({
    required String deviceId,
    required String mapName,
    List<String>? validationRules,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/validate', {
        'mapName': mapName,
        'validationRules': validationRules ??
            [
              'connectivity',
              'reachability',
              'obstacleConsistency',
              'locationPointAccessibility',
              'navigationSafety',
            ],
      });

      if (response['success'] == true) {
        print('✅ Map validation completed');
      }

      return response;
    } catch (e) {
      print('❌ Error validating map: $e');
      throw ApiException('Map validation failed: $e');
    }
  }

  /// Test path planning between points
  Future<Map<String, dynamic>> testPathPlanning({
    required String deviceId,
    required String mapName,
    required Map<String, double> startPoint,
    required Map<String, double> endPoint,
    String? algorithm,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/test-path', {
        'mapName': mapName,
        'startPoint': startPoint,
        'endPoint': endPoint,
        'algorithm': algorithm ?? 'A*',
        'pathPlanningOptions': {
          'smoothPath': true,
          'avoidObstacles': true,
          'optimizeForSpeed': false,
        },
      });

      if (response['success'] == true) {
        print('✅ Path planning test completed');
      }

      return response;
    } catch (e) {
      print('❌ Error testing path planning: $e');
      throw ApiException('Path planning test failed: $e');
    }
  }

  /// Batch location point operations for efficiency
  Future<Map<String, dynamic>> batchLocationPointOperations({
    required String deviceId,
    required String mapName,
    required List<Map<String, dynamic>> operations,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/batch-location-ops', {
        'mapName': mapName,
        'operations': operations,
        'batchId': DateTime.now().millisecondsSinceEpoch.toString(),
      });

      if (response['success'] == true) {
        print('✅ Batch operations completed: ${operations.length} operations');
      }

      return response;
    } catch (e) {
      print('❌ Error in batch operations: $e');
      throw ApiException('Batch operations failed: $e');
    }
  }

// ==========================================
// ENHANCED LOCATION POINT UTILITIES
// ==========================================

  /// Enhanced create a standardized location marker with advanced properties
  Map<String, dynamic> createEnhancedLocationMarker({
    required String name,
    required String type,
    required Map<String, double> position,
    Map<String, double>? orientation,
    Map<String, dynamic>? properties,
    String? description,
    String? category,
  }) {
    return {
      'name': name,
      'type': type,
      'position': position,
      'orientation': orientation ?? {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
      'properties': properties ?? {},
      'description': description ?? '',
      'category': category ?? 'general',
      'createdAt': DateTime.now().toIso8601String(),
      'id':
          'loc_${DateTime.now().millisecondsSinceEpoch}_${name.toLowerCase().replaceAll(' ', '_')}',
    };
  }

// ==========================================
// UTILITY METHODS FOR MAP CONVERSION
// ==========================================

  /// Convert base64 to Uint8List
  Uint8List convertBase64ToUint8List(String base64String) {
    try {
      return base64Decode(base64String);
    } catch (e) {
      throw ApiException('Failed to decode base64 data: $e');
    }
  }

  /// Convert Uint8List to base64
  String convertUint8ListToBase64(Uint8List data) {
    try {
      return base64Encode(data);
    } catch (e) {
      throw ApiException('Failed to encode data to base64: $e');
    }
  }

// ==========================================
// MAP SELECTION FOR NAVIGATION
// ==========================================

  /// Get available maps for navigation selection
  Future<Map<String, dynamic>> getNavigationMapOptions(String deviceId) async {
    _ensureInitialized();

    try {
      // Get local maps
      final localMaps = await getAvailableMaps(deviceId);

      // Get Pi maps
      final piMaps = await getAvailablePiMaps(deviceId);

      // Get saved maps
      final savedMaps = await getSavedMaps(deviceId);

      return {
        'success': true,
        'deviceId': deviceId,
        'localMaps': localMaps,
        'piMaps': piMaps,
        'savedMaps': savedMaps,
        'retrievedAt': DateTime.now().toIso8601String(),
      };
    } catch (e) {
      print('❌ Error getting navigation map options: $e');
      throw ApiException('Failed to get navigation map options: $e');
    }
  }

  /// Start navigation with selected map
  Future<Map<String, dynamic>> startNavigationWithMap({
    required String deviceId,
    required String mapName,
    required String mapSource, // 'local', 'pi', 'saved'
    bool setAsActive = true,
  }) async {
    _ensureInitialized();

    try {
      print('🧭 Starting navigation with map: $mapName from $mapSource');

      // If using Pi map, set it as active first
      if (mapSource == 'pi' && setAsActive) {
        await setActivePiMap(deviceId: deviceId, mapName: mapName);
      }

      // Start navigation
      final response =
          await _post('/api/control/devices/$deviceId/navigation/start', {
        'mapName': mapName,
        'mapSource': mapSource,
        'setAsActive': setAsActive,
        'startedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Navigation started with map: $mapName');
      }

      return response;
    } catch (e) {
      print('❌ Error starting navigation with map: $e');
      throw ApiException('Failed to start navigation with map: $e');
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
      final response =
          await _post('/api/costmaps/$deviceId/$costmapType/configure', {
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
      final response =
          await _post('/api/costmaps/$deviceId/$costmapType/export', {
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
        print(
            '✅ Map synced from $sourceDeviceId to ${targetDeviceIds.length} devices');
      }

      return response;
    } catch (e) {
      print('❌ Error syncing maps: $e');
      throw ApiException('Failed to sync maps between devices: $e');
    }
  }

  bool _isAMRBackend(Map<String, dynamic> data) {
    final dataStr = data.toString().toLowerCase();
    return data['success'] == true ||
        data['status'] == 'healthy' ||
        dataStr.contains('AMR') ||
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
    return _baseUrl!
        .replaceAll('http://', 'ws://')
        .replaceAll('https://', 'wss://');
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

  Future<Map<String, dynamic>> autoConnectAMR() async {
    _ensureInitialized();

    try {
      final response = await _post('/api/devices/auto-connect', {});

      if (response['success'] == true) {
        print('✅ AMR auto-connected: ${response['deviceId']}');
      }

      return response;
    } catch (e) {
      print('❌ Error auto-connecting AMR: $e');
      throw ApiException('Failed to auto-connect AMR: $e');
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

  Future<Map<String, dynamic>> disconnectDevice(
      {required String deviceId}) async {
    _ensureInitialized();

    try {
      final response =
          await _post('/api/control/devices/$deviceId/disconnect', {});

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
  // ORDER MANAGEMENT METHODS
  // ==========================================

  /// Get orders for a specific device
  Future<List<Map<String, dynamic>>> getOrdersForDevice({
    required String deviceId,
    String? status,
    int limit = 50,
    int offset = 0,
  }) async {
    _ensureInitialized();

    try {
      final params = <String, String>{
        'limit': limit.toString(),
        'offset': offset.toString(),
      };

      if (status != null) {
        params['status'] = status;
      }

      final queryString = params.entries
          .map((e) => '${e.key}=${Uri.encodeComponent(e.value)}')
          .join('&');

      final response = await _get('/api/orders/$deviceId?$queryString');

      if (response['success'] == true) {
        final orders = response['orders'] as List?;
        if (orders != null) {
          return orders.cast<Map<String, dynamic>>();
        }
      }

      return [];
    } catch (e) {
      print('❌ Error getting orders for $deviceId: $e');
      throw ApiException('Failed to get orders: $e');
    }
  }

  /// Get order statistics for analytics
  Future<Map<String, dynamic>> getOrderStatistics({
    String? deviceId,
    String timeRange = '7d',
  }) async {
    _ensureInitialized();

    try {
      final params = <String, String>{
        'timeRange': timeRange,
      };

      if (deviceId != null) {
        params['deviceId'] = deviceId;
      }

      final queryString = params.entries
          .map((e) => '${e.key}=${Uri.encodeComponent(e.value)}')
          .join('&');

      final response = await _get('/api/orders/stats?$queryString');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException(
          'Failed to get order statistics: ${response['error']}');
    } catch (e) {
      print('❌ Error getting order statistics: $e');
      throw ApiException('Failed to get order statistics: $e');
    }
  }

  /// Execute an order for a device
  Future<Map<String, dynamic>> executeOrder({
    required String deviceId,
    required String orderId,
    bool realTimeExecution = true,
    int waypointDelay = 3000,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/orders/$deviceId/$orderId/execute', {
        'realTimeExecution': realTimeExecution,
        'waypointDelay': waypointDelay,
      });

      if (response['success'] == true) {
        print('✅ Order execution started: $orderId');
      }

      return response;
    } catch (e) {
      print('❌ Error executing order: $e');
      throw ApiException('Failed to execute order: $e');
    }
  }

  /// Cancel an order
  Future<Map<String, dynamic>> cancelOrder({
    required String deviceId,
    required String orderId,
    String? reason,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/orders/$deviceId/$orderId/cancel', {
        if (reason != null) 'reason': reason,
      });

      if (response['success'] == true) {
        print('✅ Order cancelled: $orderId');
      }

      return response;
    } catch (e) {
      print('❌ Error cancelling order: $e');
      throw ApiException('Failed to cancel order: $e');
    }
  }

  /// Emergency stop all orders for a device
  Future<Map<String, dynamic>> emergencyStopDevice({
    required String deviceId,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/orders/$deviceId/emergency_stop', {});

      if (response['success'] == true) {
        print('🚨 Emergency stop activated for: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error in emergency stop: $e');
      throw ApiException('Failed to emergency stop: $e');
    }
  }

  /// Get order execution status
  Future<Map<String, dynamic>> getOrderExecutionStatus({
    required String deviceId,
    required String orderId,
  }) async {
    _ensureInitialized();

    try {
      final response =
          await _get('/api/orders/$deviceId/$orderId/execution_status');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException(
          'Failed to get execution status: ${response['error']}');
    } catch (e) {
      print('❌ Error getting execution status: $e');
      throw ApiException('Failed to get execution status: $e');
    }
  }

  /// Publish specific waypoint
  Future<Map<String, dynamic>> publishWaypoint({
    required String deviceId,
    required String orderId,
    required int waypointIndex,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post(
          '/api/orders/$deviceId/$orderId/waypoint/$waypointIndex', {});

      if (response['success'] == true) {
        print('✅ Waypoint published: $waypointIndex');
      }

      return response;
    } catch (e) {
      print('❌ Error publishing waypoint: $e');
      throw ApiException('Failed to publish waypoint: $e');
    }
  }

  /// Batch execute multiple orders
  Future<Map<String, dynamic>> batchExecuteOrders({
    required String deviceId,
    required List<String> orderIds,
    int orderDelay = 60000,
    int waypointDelay = 3000,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/orders/$deviceId/batch_execute', {
        'orderIds': orderIds,
        'orderDelay': orderDelay,
        'waypointDelay': waypointDelay,
      });

      if (response['success'] == true) {
        print('✅ Batch execution started for ${orderIds.length} orders');
      }

      return response;
    } catch (e) {
      print('❌ Error in batch execution: $e');
      throw ApiException('Failed to batch execute orders: $e');
    }
  }

  /// Get analytics data for a specific device and type
  Future<Map<String, dynamic>> getAnalyticsData(
    String? deviceId,
    String dataType,
    String timeRange,
  ) async {
    _ensureInitialized();

    try {
      final params = <String, String>{
        'timeRange': timeRange,
      };

      if (deviceId != null) {
        params['deviceId'] = deviceId;
      }

      final queryString = params.entries
          .map((e) => '${e.key}=${Uri.encodeComponent(e.value)}')
          .join('&');

      String endpoint;
      switch (dataType) {
        case 'orders':
          endpoint = '/api/orders/stats?$queryString';
          break;
        case 'battery':
        case 'performance':
        case 'events':
          endpoint = '/api/analytics?type=$dataType&$queryString';
          break;
        default:
          endpoint = '/api/analytics?type=$dataType&$queryString';
      }

      final response = await _get(endpoint);

      if (response['success'] == true) {
        return response;
      }

      // If analytics endpoint doesn't exist, return mock data
      if (response['error']?.toString().contains('404') == true ||
          response['error']?.toString().contains('Endpoint not found') ==
              true) {
        print('⚠️ Analytics endpoint not available, using mock data');
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
              'percentage': math.max(
                  10.0, 100.0 - (index * 2) + random.nextDouble() * 10),
              'current': 0.5 + random.nextDouble() * 2.0,
              'temperature': 20.0 + random.nextDouble() * 15.0,
            };
          }),
        };

      case 'orders':
        return {
          'success': true,
          'data': List.generate(15, (index) {
            final completedAt =
                now.subtract(Duration(hours: random.nextInt(168)));
            final duration = 15.0 + random.nextDouble() * 45.0;
            return {
              'id': 'order_${deviceId ?? 'mock'}_$index',
              'name': 'Order ${index + 1}',
              'createdAt': completedAt
                  .subtract(Duration(minutes: duration.toInt()))
                  .toIso8601String(),
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
              'timestamp': now
                  .subtract(Duration(hours: random.nextInt(72)))
                  .toIso8601String(),
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
      final response =
          await _post('/api/control/devices/$deviceId/emergency-stop', {});

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
      final response =
          await _post('/api/control/devices/$deviceId/mapping/start', {});

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
      final response =
          await _post('/api/control/devices/$deviceId/mapping/stop', {});

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
      final response =
          await _get('/api/control/devices/$deviceId/mapping/status');

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
        'mapName':
            mapName ?? 'Map_${deviceId}_${DateTime.now().toIso8601String()}',
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

  /// NEW: Save map via ROS2 map saver command
  Future<Map<String, dynamic>> saveMapViaROS2({
    required String deviceId,
    required String mapName,
    String? directory,
    String format = 'pgm',
    bool includeTimestamp = true,
  }) async {
    _ensureInitialized();

    try {
      print('🚀 Saving map via ROS2 for device: $deviceId');

      final response = await _post('/api/maps/$deviceId/save-via-ros2', {
        'mapName': mapName,
        'directory': directory ?? '/tmp/saved_maps',
        'format': format,
        'includeTimestamp': includeTimestamp,
        'executedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Map saved via ROS2: $mapName');
      }

      return response;
    } catch (e) {
      print('❌ Error saving map via ROS2: $e');
      throw ApiException('Failed to save map via ROS2: $e');
    }
  }

  /// NEW: Get ROS2 saved maps from Raspberry Pi
  Future<List<Map<String, dynamic>>> getROS2SavedMaps(
    String deviceId, {
    String? directory,
  }) async {
    _ensureInitialized();

    try {
      print('📋 Getting ROS2 saved maps for device: $deviceId');

      final queryParams = <String, String>{};
      if (directory != null) queryParams['directory'] = directory;

      final queryString = queryParams.entries
          .map((e) => '${e.key}=${Uri.encodeComponent(e.value)}')
          .join('&');

      final response = await _get(
        '/api/maps/$deviceId/ros2-saved${queryString.isNotEmpty ? '?$queryString' : ''}',
      );

      if (response['success'] == true) {
        final maps = response['maps'] as List?;
        if (maps != null) {
          print('✅ Found ${maps.length} ROS2 saved maps');
          return maps.cast<Map<String, dynamic>>();
        }
      }

      return [];
    } catch (e) {
      print('❌ Error getting ROS2 saved maps: $e');
      return [];
    }
  }

  /// NEW: Load ROS2 saved map for editing
  Future<Map<String, dynamic>> loadROS2SavedMap({
    required String deviceId,
    required String mapName,
    String? directory,
  }) async {
    _ensureInitialized();

    try {
      print('📂 Loading ROS2 saved map: $mapName for device: $deviceId');

      final response = await _post('/api/maps/$deviceId/load-ros2-saved', {
        'mapName': mapName,
        'directory': directory ?? '/tmp/saved_maps',
        'loadedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ ROS2 saved map loaded: $mapName');
      }

      return response;
    } catch (e) {
      print('❌ Error loading ROS2 saved map: $e');
      throw ApiException('Failed to load ROS2 saved map: $e');
    }
  }

  /// FIXED: Enhanced map data loading with proper error handling
  Future<Map<String, dynamic>> getMapData(String deviceId) async {
    _ensureInitialized();

    try {
      print('🗺️ Loading map data for device: $deviceId');
      print('🔗 Base URL: $_baseUrl');

      // Try multiple endpoints to get map data
      Map<String, dynamic> response;

      // Method 1: Try getting saved maps first (most reliable)
      try {
        final savedMapsUrl = '/api/maps/$deviceId/saved';
        print('🔗 Trying saved maps: $_baseUrl$savedMapsUrl');

        response = await _get(savedMapsUrl, useCache: false);
        print(
            '📥 Saved maps response: ${response.toString().substring(0, 200)}...');

        if (response['success'] == true &&
            response['maps'] != null &&
            response['maps'].isNotEmpty) {
          print(
              '✅ Found ${response['maps'].length} saved maps for device: $deviceId');

          // Use the most recent map
          final maps = response['maps'] as List;
          if (maps.isNotEmpty) {
            final latestMap = maps.first; // Maps should be sorted by date
            print('✅ Using latest map: ${latestMap['name'] ?? 'unnamed'}');

            // Load the actual map file content
            try {
              final mapName = latestMap['name'];
              final loadCompleteUrl =
                  '/api/maps/$deviceId/load-complete?mapName=$mapName';
              print('🔗 Loading complete map: $_baseUrl$loadCompleteUrl');

              final mapContentResponse =
                  await _get(loadCompleteUrl, useCache: false);
              if (mapContentResponse['success'] == true &&
                  mapContentResponse['mapData'] != null) {
                print('✅ Loaded complete map data for: ${latestMap['name']}');
                return _normalizeMapResponse(mapContentResponse);
              }
            } catch (loadError) {
              print('⚠️ Failed to load complete map data: $loadError');
            }

            // Fallback: Create minimal map data from saved map metadata
            print('🔄 Creating fallback map data from metadata');
            final mapData = {
              'success': true,
              'mapData': {
                'deviceId': deviceId,
                'timestamp':
                    latestMap['savedAt'] ?? DateTime.now().toIso8601String(),
                'info': {
                  'resolution': latestMap['metadata']?['mapDimensions']
                          ?['resolution'] ??
                      0.05,
                  'width':
                      latestMap['metadata']?['mapDimensions']?['width'] ?? 800,
                  'height':
                      latestMap['metadata']?['mapDimensions']?['height'] ?? 600,
                  'origin': {
                    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                  }
                },
                'occupancyData': [], // Empty for now, but map exists
                'shapes': [], // Will be loaded when needed
                'version': 1
              }
            };
            return _normalizeMapResponse(mapData);
          }
        } else {
          print('⚠️ No saved maps found or invalid response structure');
        }
      } catch (e) {
        print('❌ Saved maps endpoint failed: $e');
      }

      // Method 2: Try the control endpoint (your existing API)
      try {
        response =
            await _get('/api/control/devices/$deviceId/maps', useCache: false);
        if (response['success'] == true && response['mapData'] != null) {
          print('✅ Map data loaded from control endpoint');
          return _normalizeMapResponse(response);
        }
      } catch (e) {
        print('⚠️ Control endpoint failed: $e');
      }

      // Method 2: Try the enhanced map endpoint
      try {
        response =
            await _get('/api/maps/$deviceId/load-complete', useCache: false);
        if (response['success'] == true && response['mapData'] != null) {
          print('✅ Map data loaded from enhanced endpoint');
          return _normalizeMapResponse(response);
        }
      } catch (e) {
        print('⚠️ Enhanced endpoint failed: $e');
      }

      // Method 3: Try to get the latest saved map
      try {
        final savedMaps = await getSavedMapsEnhanced(deviceId);
        if (savedMaps.isNotEmpty) {
          final latestMap = savedMaps.first;
          response = await loadCompleteMapDataEnhanced(
            deviceId: deviceId,
            mapName: latestMap['name'],
          );
          if (response['success'] == true && response['mapData'] != null) {
            print('✅ Map data loaded from saved maps');
            return _normalizeMapResponse(response);
          }
        }
      } catch (e) {
        print('⚠️ Saved maps endpoint failed: $e');
      }

      // Method 4: Create empty map if nothing found
      print('⚠️ No map data found, creating empty map');
      return _createEmptyMapResponse(deviceId);
    } catch (e) {
      print('❌ Error getting map data: $e');
      throw ApiException('Failed to get map data: $e');
    }
  }

  /// FIXED: Normalize map response to consistent format
  Map<String, dynamic> _normalizeMapResponse(Map<String, dynamic> response) {
    if (response['mapData'] != null) {
      final mapData = response['mapData'];

      // Ensure required fields exist
      mapData['deviceId'] = mapData['deviceId'] ?? 'unknown';
      mapData['timestamp'] =
          mapData['timestamp'] ?? DateTime.now().toIso8601String();
      mapData['version'] = mapData['version'] ?? 1;

      // Ensure info field exists
      if (mapData['info'] == null) {
        mapData['info'] = {
          'resolution': 0.05,
          'width': 1000,
          'height': 1000,
          'origin': {
            'position': {'x': -25.0, 'y': -25.0, 'z': 0.0},
            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
          }
        };
      }

      // Ensure shapes field exists
      mapData['shapes'] = mapData['shapes'] ?? [];

      // Ensure occupancy data exists
      if (mapData['occupancyData'] == null && mapData['data'] == null) {
        mapData['occupancyData'] = [];
      }

      // Backward compatibility: copy 'data' to 'occupancyData' if needed
      if (mapData['occupancyData'] == null && mapData['data'] != null) {
        mapData['occupancyData'] = mapData['data'];
      }

      return {
        'success': true,
        'mapData': mapData,
        'source': 'normalized',
        'loadedAt': DateTime.now().toIso8601String(),
      };
    }

    throw ApiException('Invalid map response format');
  }

  /// Create empty map response when no data is available
  Map<String, dynamic> _createEmptyMapResponse(String deviceId) {
    return {
      'success': true,
      'mapData': {
        'deviceId': deviceId,
        'timestamp': DateTime.now().toIso8601String(),
        'info': {
          'resolution': 0.05,
          'width': 1000,
          'height': 1000,
          'origin': {
            'position': {'x': -25.0, 'y': -25.0, 'z': 0.0},
            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
          }
        },
        'occupancyData': [],
        'shapes': [],
        'version': 1,
      },
      'source': 'empty',
      'loadedAt': DateTime.now().toIso8601String(),
    };
  }

  /// FIXED: Get saved maps with proper error handling
  Future<List<Map<String, dynamic>>> getSavedMapsEnhanced(
    String deviceId, {
    bool includePreview = true,
    String? sortBy = 'savedAt',
    bool descending = true,
  }) async {
    _ensureInitialized();

    try {
      print('📋 Getting saved maps for device: $deviceId');

      final params = {
        'includePreview': includePreview.toString(),
        'sortBy': sortBy ?? 'savedAt',
        'descending': descending.toString(),
      };

      final queryString = params.entries
          .map((e) => '${e.key}=${Uri.encodeComponent(e.value)}')
          .join('&');

      final response = await _get('/api/maps/$deviceId/saved?$queryString');

      if (response['success'] == true) {
        final maps = response['maps'] as List?;
        if (maps != null) {
          print('✅ Found ${maps.length} saved maps');
          return maps.cast<Map<String, dynamic>>();
        }
      }

      print('⚠️ No saved maps found');
      return [];
    } catch (e) {
      print('❌ Error getting saved maps: $e');
      // Return empty list instead of throwing to handle gracefully
      return [];
    }
  }

  /// FIXED: Load specific map with proper validation
  Future<Map<String, dynamic>> loadCompleteMapDataEnhanced({
    required String deviceId,
    String? mapName,
    bool includeShapes = true,
    bool includeMetadata = true,
  }) async {
    _ensureInitialized();

    try {
      print(
          '📂 Loading map data: ${mapName ?? 'current'} for device: $deviceId');

      final params = <String, String>{
        'includeShapes': includeShapes.toString(),
        'includeMetadata': includeMetadata.toString(),
      };

      if (mapName != null) {
        params['mapName'] = mapName;
      }

      final queryString = params.entries
          .map((e) => '${e.key}=${Uri.encodeComponent(e.value)}')
          .join('&');

      final response =
          await _get('/api/maps/$deviceId/load-complete?$queryString');

      if (response['success'] == true) {
        print('✅ Map loaded successfully: ${mapName ?? 'current'}');
        return _normalizeMapResponse(response);
      }

      throw ApiException('Failed to load map: ${response['error']}');
    } catch (e) {
      print('❌ Error loading map data: $e');
      throw ApiException('Failed to load map data: $e');
    }
  }

  Future<Map<String, dynamic>> getLiveMapData(String deviceId) async {
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
      final response =
          await _post('/api/control/devices/$deviceId/map/shapes', {
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
      final response = await _put(
          '/api/control/devices/$deviceId/map/shapes/$shapeId', updates ?? {});

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
      final response =
          await _delete('/api/control/devices/$deviceId/map/shapes/$shapeId');

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
// COMPLETE ORDER MANAGEMENT METHODS
// ==========================================

  /// Get all orders for a specific device
  Future<List<Map<String, dynamic>>> getOrders(String deviceId) async {
    _ensureInitialized();
    try {
      final response = await _get('/api/orders/$deviceId', useCache: false);
      if (response['success'] == true && response['orders'] is List) {
        return List<Map<String, dynamic>>.from(response['orders']);
      }
      return [];
    } catch (e) {
      print('❌ Error getting orders for $deviceId: $e');
      throw ApiException('Failed to get orders: $e');
    }
  }

  /// Get all orders across all devices (for dashboard)
  Future<Map<String, dynamic>> getAllOrders({
    String? status,
    String? deviceId,
    int limit = 50,
    int offset = 0,
  }) async {
    _ensureInitialized();
    try {
      final queryParams = <String, String>{
        'limit': limit.toString(),
        'offset': offset.toString(),
      };
      if (status != null) queryParams['status'] = status;
      if (deviceId != null) queryParams['deviceId'] = deviceId;
      final queryString = queryParams.entries
          .map((e) => '${e.key}=${Uri.encodeComponent(e.value)}')
          .join('&');
      final response = await _get('/api/orders?$queryString', useCache: false);
      if (response['success'] == true) {
        return response;
      }
      throw ApiException('Failed to get all orders: ${response['error']}');
    } catch (e) {
      print('❌ Error getting all orders: $e');
      throw ApiException('Failed to get all orders: $e');
    }
  }

  /// Create a new order with waypoint sequence
  Future<Map<String, dynamic>> createOrder({
    required String deviceId,
    required String name,
    required List<Map<String, dynamic>> waypoints,
    int priority = 0,
    String? description,
  }) async {
    _ensureInitialized();
    try {
      final response = await _post('/api/orders/$deviceId', {
        'name': name,
        'waypoints': waypoints,
        'priority': priority,
        'description': description ?? '',
      });
      if (response['success'] == true) {
        print(
            '✅ Order created: ${response['order']['id']} for device: $deviceId');
      }
      return response;
    } catch (e) {
      print('❌ Error creating order: $e');
      throw ApiException('Failed to create order: $e');
    }
  }

  /// ✅ NEW: Execute order with enhanced target_pose integration
  Future<Map<String, dynamic>> executeOrderEnhanced({
    required String deviceId,
    required String orderId,
    bool immediateStart = true,
    String executionMode = 'sequential',
  }) async {
    _ensureInitialized();
    try {
      final response = await _post(
          '/api/enhanced-orders/devices/$deviceId/orders/$orderId/execute', {
        'immediateStart': immediateStart,
        'executionMode': executionMode,
      });
      if (response['success'] == true) {
        print(
            '✅ Enhanced order execution started: $orderId for device: $deviceId');
      }
      return response;
    } catch (e) {
      print('❌ Error executing enhanced order: $e');
      throw ApiException('Failed to execute enhanced order: $e');
    }
  }

  /// ✅ NEW: Pause order execution
  Future<Map<String, dynamic>> pauseOrderExecution({
    required String deviceId,
    required String orderId,
  }) async {
    _ensureInitialized();
    try {
      final response = await _post(
          '/api/enhanced-orders/devices/$deviceId/orders/$orderId/pause', {});
      if (response['success'] == true) {
        print('✅ Order execution paused: $orderId');
      }
      return response;
    } catch (e) {
      print('❌ Error pausing order execution: $e');
      throw ApiException('Failed to pause order execution: $e');
    }
  }

  /// ✅ NEW: Resume order execution
  Future<Map<String, dynamic>> resumeOrderExecution({
    required String deviceId,
    required String orderId,
  }) async {
    _ensureInitialized();
    try {
      final response = await _post(
          '/api/enhanced-orders/devices/$deviceId/orders/$orderId/resume', {});
      if (response['success'] == true) {
        print('✅ Order execution resumed: $orderId');
      }
      return response;
    } catch (e) {
      print('❌ Error resuming order execution: $e');
      throw ApiException('Failed to resume order execution: $e');
    }
  }

  /// ✅ NEW: Cancel order execution
  Future<Map<String, dynamic>> cancelOrderExecution({
    required String deviceId,
    required String orderId,
    String? reason,
  }) async {
    _ensureInitialized();
    try {
      final response = await _post(
          '/api/enhanced-orders/devices/$deviceId/orders/$orderId/cancel', {
        'reason': reason ?? 'User cancelled',
      });
      if (response['success'] == true) {
        print('✅ Order execution cancelled: $orderId');
      }
      return response;
    } catch (e) {
      print('❌ Error cancelling order execution: $e');
      throw ApiException('Failed to cancel order execution: $e');
    }
  }

  /// ✅ NEW: Get enhanced execution status
  Future<Map<String, dynamic>> getEnhancedOrderExecutionStatus({
    required String deviceId,
    required String orderId,
  }) async {
    _ensureInitialized();
    try {
      final response = await _get(
          '/api/enhanced-orders/devices/$deviceId/orders/$orderId/status');
      if (response['success'] == true) {
        return response;
      }
      throw ApiException(
          'Failed to get execution status: ${response['error']}');
    } catch (e) {
      print('❌ Error getting execution status: $e');
      throw ApiException('Failed to get execution status: $e');
    }
  }

  /// ✅ NEW: Get active executions
  Future<Map<String, dynamic>> getActiveOrderExecutions(
      {String? deviceId}) async {
    _ensureInitialized();
    try {
      String url = '/api/enhanced-orders/active-executions';
      if (deviceId != null) {
        url += '?deviceId=$deviceId';
      }
      final response = await _get(url);
      if (response['success'] == true) {
        return response;
      }
      throw ApiException(
          'Failed to get active executions: ${response['error']}');
    } catch (e) {
      print('❌ Error getting active executions: $e');
      throw ApiException('Failed to get active executions: $e');
    }
  }

  /// ✅ NEW: Get execution history
  Future<Map<String, dynamic>> getOrderExecutionHistory({
    String? deviceId,
    int limit = 50,
    int offset = 0,
  }) async {
    _ensureInitialized();
    try {
      final queryParams = <String, String>{
        'limit': limit.toString(),
        'offset': offset.toString(),
      };
      if (deviceId != null) {
        queryParams['deviceId'] = deviceId;
      }

      final queryString = queryParams.entries
          .map((e) => '${e.key}=${Uri.encodeComponent(e.value)}')
          .join('&');

      final response =
          await _get('/api/enhanced-orders/execution-history?$queryString');
      if (response['success'] == true) {
        return response;
      }
      throw ApiException(
          'Failed to get execution history: ${response['error']}');
    } catch (e) {
      print('❌ Error getting execution history: $e');
      throw ApiException('Failed to get execution history: $e');
    }
  }

  /// Get specific order details
  Future<Map<String, dynamic>> getOrder(String deviceId, String orderId) async {
    _ensureInitialized();
    try {
      final response = await _get('/api/orders/$deviceId/$orderId');
      if (response['success'] == true) {
        return response;
      }
      throw ApiException('Failed to get order: ${response['error']}');
    } catch (e) {
      print('❌ Error getting order: $e');
      throw ApiException('Failed to get order: $e');
    }
  }

  /// Update order status
  Future<Map<String, dynamic>> updateOrderStatus({
    required String orderId,
    required String status,
    int? currentWaypoint,
    String? reason,
  }) async {
    _ensureInitialized();
    try {
      // Extract deviceId from orderId if needed
      final deviceId = _extractDeviceIdFromOrderId(orderId);
      final body = <String, dynamic>{'status': status};
      if (currentWaypoint != null) body['currentWaypoint'] = currentWaypoint;
      if (reason != null) body['reason'] = reason;
      final response =
          await _put('/api/orders/$deviceId/$orderId/status', body);
      if (response['success'] == true) {
        print('✅ Order status updated: $orderId -> $status');
      }
      return response;
    } catch (e) {
      print('❌ Error updating order status: $e');
      throw ApiException('Failed to update order status: $e');
    }
  }

  /// Execute an order (start it) - legacy version, renamed to avoid conflict
  Future<Map<String, dynamic>> executeOrderLegacy(
      String deviceId, String orderId) async {
    _ensureInitialized();
    try {
      final response =
          await _post('/api/orders/$deviceId/$orderId/execute', {});
      if (response['success'] == true) {
        print('✅ Order execution started: $orderId');
      }
      return response;
    } catch (e) {
      print('❌ Error executing order: $e');
      throw ApiException('Failed to execute order: $e');
    }
  }

  /// Pause an active order
  Future<Map<String, dynamic>> pauseOrder(
      String deviceId, String orderId) async {
    _ensureInitialized();
    try {
      final response = await _post('/api/orders/$deviceId/$orderId/pause', {});
      if (response['success'] == true) {
        print('✅ Order paused: $orderId');
      }
      return response;
    } catch (e) {
      print('❌ Error pausing order: $e');
      throw ApiException('Failed to pause order: $e');
    }
  }

  /// Delete an order
  Future<Map<String, dynamic>> deleteOrder(
      String deviceId, String orderId) async {
    _ensureInitialized();
    try {
      final response = await _delete('/api/orders/$deviceId/$orderId');
      if (response['success'] == true) {
        print('✅ Order deleted: $orderId');
      }
      return response;
    } catch (e) {
      print('❌ Error deleting order: $e');
      throw ApiException('Failed to delete order: $e');
    }
  }

  /// Get order statistics for a device
  Future<Map<String, dynamic>> getOrderStats(String deviceId,
      {String timeRange = '7d'}) async {
    _ensureInitialized();
    try {
      final response =
          await _get('/api/orders/$deviceId/stats?timeRange=$timeRange');
      if (response['success'] == true) {
        return response;
      }
      throw ApiException('Failed to get order stats: ${response['error']}');
    } catch (e) {
      print('❌ Error getting order stats: $e');
      throw ApiException('Failed to get order stats: $e');
    }
  }

  /// Get system-wide order statistics
  Future<Map<String, dynamic>> getSystemOrderStats(
      {String timeRange = '7d'}) async {
    _ensureInitialized();
    try {
      final response =
          await _get('/api/orders/stats?timeRange=$timeRange', useCache: false);
      if (response['success'] == true) {
        return response;
      }
      throw ApiException(
          'Failed to get system order stats: ${response['error']}');
    } catch (e) {
      print('❌ Error getting system order stats: $e');
      throw ApiException('Failed to get system order stats: $e');
    }
  }

  /// Get available stations from device map for order creation
  Future<Map<String, dynamic>> getMapStations(String deviceId) async {
    _ensureInitialized();
    try {
      final response = await _get('/api/orders/$deviceId/stations');
      if (response['success'] == true) {
        return response;
      }
      throw ApiException('Failed to get map stations: ${response['error']}');
    } catch (e) {
      print('❌ Error getting map stations: $e');
      throw ApiException('Failed to get map stations: $e');
    }
  }

  /// Extract device ID from order ID (based on order ID format: order_{deviceId}_{timestamp}_{random})
  String _extractDeviceIdFromOrderId(String orderId) {
    try {
      final parts = orderId.split('_');
      if (parts.length >= 2 && parts[0] == 'order') {
        return parts[1];
      }
    } catch (e) {
      print('❌ Error extracting device ID from order ID: $e');
    }
    // Fallback - assume first connected device if extraction fails
    return 'piros'; // or throw an exception
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
      throw ApiException(
          'API service not initialized. Call initialize() or autoInitialize() first.');
    }
  }

  Future<Map<String, dynamic>> _get(String endpoint,
      {bool useCache = true, Duration? cacheTtl}) async {
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

      if (useCache &&
          _cachingEnabled &&
          response.statusCode >= 200 &&
          response.statusCode < 300) {
        final ttl = cacheTtl ?? _defaultCacheTtl;
        _cache[cacheKey] = _CacheEntry(data, DateTime.now(), ttl);
      }
      return data;
    } catch (e) {
      _trackRequest(endpoint, false, null);
      if (e is TimeoutException) {
        throw ApiException('Request timeout');
      } else if (e is SocketException) {
        throw ApiException('Network error: Connection failed',
            originalError: e);
      }
      rethrow;
    }
  }

  Future<Map<String, dynamic>> _post(
      String endpoint, Map<String, dynamic> data) async {
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
        throw ApiException('Network error: Connection failed',
            originalError: e);
      }
      rethrow;
    }
  }

  Future<Map<String, dynamic>> _put(
      String endpoint, Map<String, dynamic> data) async {
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
        throw ApiException('Network error: Connection failed',
            originalError: e);
      }
      rethrow;
    }
  }

  Future<Map<String, dynamic>> _delete(String endpoint) async {
    _ensureInitialized();

    try {
      final url = Uri.parse('$_baseUrl$endpoint');
      print('📡 DELETE: $url');

      final response =
          await http.delete(url, headers: _headers).timeout(_timeout);
      return _handleResponse(response);
    } catch (e) {
      if (e is TimeoutException) {
        throw ApiException('Request timeout');
      } else if (e is SocketException) {
        throw ApiException('Network error: Connection failed',
            originalError: e);
      }
      rethrow;
    }
  }

  Map<String, dynamic> _handleResponse(http.Response response) {
    final String body = response.body;

    print(
        '📨 Response ${response.statusCode}: ${body.length > 200 ? body.substring(0, 200) + '...' : body}');

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
      throw ApiException(
          'Failed to parse response: ${response.statusCode} - $body');
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
        'successRate': _totalRequests > 0
            ? (_successfulRequests / _totalRequests * 100)
            : 0.0,
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
    final keysToRemove =
        _cache.keys.where((key) => key.contains(endpoint)).toList();
    for (final key in keysToRemove) {
      _cache.remove(key);
    }
    print('🗑️ Cache cleared for endpoint: $endpoint');
  }

  void dispose() {
    clearCache();
    clearStats();
  }

  // ✅ ADD THESE NEW METHODS to your existing ApiService class:

  /// Save complete map data with all layers and metadata
  Future<Map<String, dynamic>> saveCompleteMapData({
    required String deviceId,
    required MapData mapData,
    String? mapName,
    bool includeMetadata = true,
    bool createBackup = true,
  }) async {
    _ensureInitialized();

    try {
      final completeMapName = mapName ??
          'complete_map_${deviceId}_${DateTime.now().millisecondsSinceEpoch}';

      final requestData = {
        'mapData': mapData.toJson(),
        'mapName': completeMapName,
        'saveType': 'complete',
        'includeMetadata': includeMetadata,
        'createBackup': createBackup,
        'savedAt': DateTime.now().toIso8601String(),
        'clientInfo': {
          'platform': 'flutter',
          'version': '1.0.0',
          'source': 'live_mapping',
        },
      };

      final response =
          await _post('/api/maps/$deviceId/save-complete', requestData);

      if (response['success'] == true) {
        print('✅ Complete map saved successfully: $completeMapName');

        // Clear cache for this device to force reload
        clearCacheForEndpoint('/api/maps/$deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error saving complete map data: $e');
      throw ApiException('Failed to save complete map data: $e');
    }
  }

  /// Load complete map data for editing
  Future<Map<String, dynamic>> loadCompleteMapData({
    required String deviceId,
    String? mapName,
    bool includeShapes = true,
    bool includeMetadata = true,
  }) async {
    _ensureInitialized();

    try {
      final params = <String, String>{
        'includeShapes': includeShapes.toString(),
        'includeMetadata': includeMetadata.toString(),
      };

      if (mapName != null) {
        params['mapName'] = mapName;
      }

      final queryString = params.entries
          .map((e) => '${e.key}=${Uri.encodeComponent(e.value)}')
          .join('&');

      final response =
          await _get('/api/maps/$deviceId/load-complete?$queryString');

      if (response['success'] == true) {
        print('✅ Complete map loaded for device: $deviceId');
        return response;
      }

      throw ApiException('Failed to load complete map: ${response['error']}');
    } catch (e) {
      print('❌ Error loading complete map data: $e');
      throw ApiException('Failed to load complete map data: $e');
    }
  }

  /// Get all saved maps for a device with metadata
  Future<List<Map<String, dynamic>>> getSavedMaps(
    String deviceId, {
    bool includePreview = true,
    String? sortBy = 'savedAt',
    bool descending = true,
  }) async {
    _ensureInitialized();

    try {
      final params = {
        'includePreview': includePreview.toString(),
        'sortBy': sortBy ?? 'savedAt',
        'descending': descending.toString(),
      };

      final queryString = params.entries
          .map((e) => '${e.key}=${Uri.encodeComponent(e.value)}')
          .join('&');

      final response = await _get('/api/maps/$deviceId/saved?$queryString');

      if (response['success'] == true) {
        final maps = response['maps'] as List?;
        if (maps != null) {
          return maps.cast<Map<String, dynamic>>();
        }
      }

      return [];
    } catch (e) {
      print('❌ Error getting saved maps: $e');
      // Return empty list instead of throwing to handle gracefully
      return [];
    }
  }

  /// Get map preview/summary without full data
  Future<Map<String, dynamic>> getMapPreview({
    required String deviceId,
    required String mapName,
  }) async {
    _ensureInitialized();

    try {
      final response = await _get('/api/maps/$deviceId/$mapName/preview');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException('Failed to get map preview: ${response['error']}');
    } catch (e) {
      print('❌ Error getting map preview: $e');
      throw ApiException('Failed to get map preview: $e');
    }
  }

  /// Clone/duplicate a saved map
  Future<Map<String, dynamic>> cloneMap({
    required String deviceId,
    required String sourceMapName,
    required String targetMapName,
    bool includeShapes = true,
    bool includeMetadata = true,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/$sourceMapName/clone', {
        'targetMapName': targetMapName,
        'includeShapes': includeShapes,
        'includeMetadata': includeMetadata,
        'clonedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Map cloned: $sourceMapName -> $targetMapName');

        // Clear cache to force reload
        clearCacheForEndpoint('/api/maps/$deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error cloning map: $e');
      throw ApiException('Failed to clone map: $e');
    }
  }

  /// Delete saved map
  Future<Map<String, dynamic>> deleteSavedMap({
    required String deviceId,
    required String mapName,
    bool deleteBackups = false,
  }) async {
    _ensureInitialized();

    try {
      final response = await _delete(
          '/api/maps/$deviceId/$mapName?deleteBackups=$deleteBackups');

      if (response['success'] == true) {
        print('✅ Map deleted: $mapName');

        // Clear cache for this device
        clearCacheForEndpoint('/api/maps/$deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error deleting saved map: $e');
      throw ApiException('Failed to delete saved map: $e');
    }
  }

  /// Export maps in various formats
  Future<Map<String, dynamic>> exportMaps({
    required String deviceId,
    required List<String> mapNames,
    required String format, // 'json', 'pgm', 'png', 'zip'
    bool includeMetadata = true,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/export', {
        'mapNames': mapNames,
        'format': format,
        'includeMetadata': includeMetadata,
        'exportedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Maps exported in $format format: ${mapNames.join(', ')}');
        return response;
      }

      throw ApiException('Failed to export maps: ${response['error']}');
    } catch (e) {
      print('❌ Error exporting maps: $e');
      throw ApiException('Failed to export maps: $e');
    }
  }

  /// Share map with other devices
  Future<Map<String, dynamic>> shareMapWithDevices({
    required String sourceDeviceId,
    required String mapName,
    required List<String> targetDeviceIds,
    bool overwriteExisting = false,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$sourceDeviceId/$mapName/share', {
        'targetDeviceIds': targetDeviceIds,
        'overwriteExisting': overwriteExisting,
        'sharedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Map shared: $mapName to ${targetDeviceIds.length} devices');

        // Clear cache for all affected devices
        for (final deviceId in targetDeviceIds) {
          clearCacheForEndpoint('/api/maps/$deviceId');
        }
      }

      return response;
    } catch (e) {
      print('❌ Error sharing map: $e');
      throw ApiException('Failed to share map: $e');
    }
  }

// ==========================================
// LOCATION MANAGEMENT FOR MAP EDITING
// ==========================================

  /// Create location marker for map
  Map<String, dynamic> createLocationMarker({
    required String name,
    required String type, // 'pickup', 'drop', 'charging', 'home', 'waypoint'
    required Map<String, double> position,
    Map<String, double>? orientation,
    Map<String, dynamic>? metadata,
  }) {
    return {
      'name': name,
      'type': type,
      'position': {
        'x': position['x'] ?? 0.0,
        'y': position['y'] ?? 0.0,
        'z': position['z'] ?? 0.0,
      },
      'orientation': {
        'x': orientation?['x'] ?? 0.0,
        'y': orientation?['y'] ?? 0.0,
        'z': orientation?['z'] ?? 0.0,
        'w': orientation?['w'] ?? 1.0,
      },
      'metadata': metadata ?? {},
      'createdAt': DateTime.now().toIso8601String(),
    };
  }

  /// Validate location data
  bool validateLocationData(Map<String, dynamic> location) {
    final requiredFields = ['name', 'type', 'position'];

    for (final field in requiredFields) {
      if (!location.containsKey(field) || location[field] == null) {
        return false;
      }
    }

    final validTypes = ['pickup', 'drop', 'charging', 'home', 'waypoint'];
    if (!validTypes.contains(location['type'])) {
      return false;
    }

    final position = location['position'];
    if (position is! Map ||
        !position.containsKey('x') ||
        !position.containsKey('y')) {
      return false;
    }

    return true;
  }

  /// Get location type color
  String getLocationTypeColor(String type) {
    switch (type) {
      case 'pickup':
        return '#4CAF50'; // Green
      case 'drop':
        return '#2196F3'; // Blue
      case 'charging':
        return '#FF9800'; // Orange
      case 'home':
        return '#9C27B0'; // Purple
      case 'waypoint':
        return '#607D8B'; // Blue Grey
      default:
        return '#757575'; // Grey
    }
  }

  /// Get location type icon
  String getLocationTypeIcon(String type) {
    switch (type) {
      case 'pickup':
        return '📦';
      case 'drop':
        return '🎯';
      case 'charging':
        return '🔋';
      case 'home':
        return '🏠';
      case 'waypoint':
        return '📍';
      default:
        return '⭕';
    }
  }

// ==========================================
// DEPLOYMENT CONFIGURATION MANAGEMENT
// ==========================================

  /// Save deployment configuration
  Future<Map<String, dynamic>> saveDeploymentConfig({
    required String deviceId,
    required Map<String, dynamic> config,
  }) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/maps/$deviceId/deployment-config', {
        'config': config,
        'savedAt': DateTime.now().toIso8601String(),
      });

      if (response['success'] == true) {
        print('✅ Deployment config saved');
      }

      return response;
    } catch (e) {
      print('❌ Error saving deployment config: $e');
      throw ApiException('Failed to save deployment config: $e');
    }
  }

  /// Get saved deployment configuration
  Future<Map<String, dynamic>> getDeploymentConfig(String deviceId) async {
    _ensureInitialized();

    try {
      final response = await _get('/api/maps/$deviceId/deployment-config');

      if (response['success'] == true) {
        return response;
      }

      // Return default config if none saved
      return {
        'success': true,
        'config': {
          'piHost': '192.168.1.100',
          'piPort': 22,
          'piUser': 'pi',
          'piMapDir': '/home/pi/ros2_ws/src/nav2_bringup/maps',
          'autoLoad': true,
          'backupMaps': true,
        },
      };
    } catch (e) {
      print('❌ Error getting deployment config: $e');
      throw ApiException('Failed to get deployment config: $e');
    }
  }

  /// Deploy multiple maps to Raspberry Pi
  Future<Map<String, dynamic>> batchDeployMaps({
    required String deviceId,
    required List<String> mapNames,
    Map<String, dynamic>? piConfig,
    bool autoLoad = false, // Only last map auto-loads
  }) async {
    _ensureInitialized();

    try {
      print('🚀 Starting batch deployment of ${mapNames.length} maps...');

      final results = <String, dynamic>{};
      String? lastDeployedMap;

      for (int i = 0; i < mapNames.length; i++) {
        final mapName = mapNames[i];
        final isLastMap = i == mapNames.length - 1;

        print('📋 Deploying ${i + 1}/${mapNames.length}: $mapName');

        try {
          final result = await deployMapToRaspberryPi(
            deviceId: deviceId,
            mapName: mapName,
            piConfig: piConfig,
            autoLoad: autoLoad && isLastMap, // Only auto-load the last map
          );

          results[mapName] = result;

          if (result['success'] == true) {
            lastDeployedMap = mapName;
            print('✅ Deployed: $mapName');
          } else {
            print('❌ Failed to deploy: $mapName');
          }

          // Small delay between deployments
          await Future.delayed(Duration(seconds: 2));
        } catch (e) {
          results[mapName] = {
            'success': false,
            'error': e.toString(),
          };
          print('❌ Error deploying $mapName: $e');
        }
      }

      final successCount =
          results.values.where((result) => result['success'] == true).length;

      return {
        'success': successCount > 0,
        'message': 'Batch deployment completed',
        'totalMaps': mapNames.length,
        'successfulDeployments': successCount,
        'failedDeployments': mapNames.length - successCount,
        'lastDeployedMap': lastDeployedMap,
        'results': results,
        'completedAt': DateTime.now().toIso8601String(),
      };
    } catch (e) {
      print('❌ Batch deployment failed: $e');
      throw ApiException('Batch deployment failed: $e');
    }
  }

  /// Get map analytics/statistics
  Future<Map<String, dynamic>> getMapAnalytics({
    required String deviceId,
    required String mapName,
  }) async {
    _ensureInitialized();

    try {
      final response = await _get('/api/maps/$deviceId/$mapName/analytics');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException('Failed to get map analytics: ${response['error']}');
    } catch (e) {
      print('❌ Error getting map analytics: $e');
      throw ApiException('Failed to get map analytics: $e');
    }
  }

// ==========================================
// ENHANCED ROS2 NAVIGATION METHODS
// ==========================================

  /// Publish navigation goal to target_pose topic (ROS2 specific)
  Future<Map<String, dynamic>> publishGoal(
    String deviceId,
    double x,
    double y, {
    double orientation = 0.0,
    String? goalId,
  }) async {
    _ensureInitialized();

    try {
      final requestBody = {
        'x': x,
        'y': y,
        'orientation': orientation,
        if (goalId != null) 'goalId': goalId,
      };

      final response =
          await _post('/api/ros2/$deviceId/publish_goal', requestBody);

      if (response['success'] == true) {
        print('🎯 Navigation goal published: ($x, $y) @ ${orientation}rad');
      }

      return response;
    } catch (e) {
      print('❌ Error publishing goal: $e');
      throw ApiException('Failed to publish goal: $e');
    }
  }

  /// Cancel current navigation goal (ROS2 specific)
  Future<Map<String, dynamic>> cancelGoal(String deviceId) async {
    _ensureInitialized();

    try {
      final response = await _post('/api/ros2/$deviceId/cancel_goal', {});

      if (response['success'] == true) {
        print('🛑 Navigation goal cancelled for device: $deviceId');
      }

      return response;
    } catch (e) {
      print('❌ Error cancelling goal: $e');
      throw ApiException('Failed to cancel goal: $e');
    }
  }

  /// Get navigation status (ROS2 specific)
  Future<Map<String, dynamic>> getNavigationStatus(String deviceId) async {
    _ensureInitialized();

    try {
      final response = await _get('/api/ros2/$deviceId/navigation_status');

      if (response['success'] == true) {
        return response;
      }

      throw ApiException(
          'Failed to get navigation status: ${response['error']}');
    } catch (e) {
      print('❌ Error getting navigation status: $e');
      throw ApiException('Failed to get navigation status: $e');
    }
  }

  /// Publish velocity command (ROS2 specific)
  Future<Map<String, dynamic>> publishVelocity(
    String deviceId,
    double linear,
    double angular,
  ) async {
    _ensureInitialized();

    try {
      final requestBody = {
        'linear': linear,
        'angular': angular,
      };

      final response =
          await _post('/api/ros2/$deviceId/publish_velocity', requestBody);

      if (response['success'] == true) {
        print(
            '🏃 Velocity command published: linear=$linear, angular=$angular');
      }

      return response;
    } catch (e) {
      print('❌ Error publishing velocity: $e');
      throw ApiException('Failed to publish velocity: $e');
    }
  }

// ==========================================
// ENHANCED ORDER EXECUTION WITH TRACKING
// ==========================================

  /// Execute order with real-time waypoint publishing and progress tracking
  Future<Map<String, dynamic>> executeOrderWithTracking(
    String deviceId,
    String orderId, {
    Duration waypointDelay = const Duration(seconds: 3),
    Function(int currentWaypoint, int totalWaypoints)? onProgress,
  }) async {
    _ensureInitialized();

    try {
      // Get order details
      final orderResponse = await getOrder(deviceId, orderId);
      if (orderResponse['success'] != true) {
        throw Exception(
            orderResponse['error'] ?? 'Failed to get order details');
      }

      final order = orderResponse['order'];
      final waypoints = order['waypoints'] as List<dynamic>? ?? [];

      if (waypoints.isEmpty) {
        throw Exception('Order has no waypoints');
      }

      print('🚀 Starting order execution: ${order['name']}');
      print('📍 Total waypoints: ${waypoints.length}');

      // Update order status to active
      await updateOrderStatus(
        orderId: orderId,
        status: 'active',
      );

      // Execute each waypoint sequentially
      for (int i = 0; i < waypoints.length; i++) {
        final waypoint = waypoints[i];
        final position = waypoint['position'] as Map<String, dynamic>? ?? {};

        final x = position['x']?.toDouble() ?? 0.0;
        final y = position['y']?.toDouble() ?? 0.0;
        final orientation = waypoint['orientation']?.toDouble() ?? 0.0;

        print(
            '🎯 Executing waypoint ${i + 1}/${waypoints.length}: ${waypoint['name']}');
        print('📍 Coordinates: ($x, $y) @ ${orientation}rad');

        // Publish navigation goal to target_pose topic
        final goalResponse = await publishGoal(
          deviceId,
          x,
          y,
          orientation: orientation,
          goalId: '${orderId}_wp_${i}',
        );

        if (goalResponse['success'] != true) {
          throw Exception(
              'Failed to publish waypoint ${i + 1}: ${goalResponse['error']}');
        }

        // Update order progress
        await updateOrderStatus(
          orderId: orderId,
          status: 'active',
          currentWaypoint: i + 1,
        );

        // Call progress callback if provided
        onProgress?.call(i + 1, waypoints.length);

        // Wait before next waypoint (except for last one)
        if (i < waypoints.length - 1) {
          print(
              '⏳ Waiting ${waypointDelay.inSeconds}s before next waypoint...');
          await Future.delayed(waypointDelay);
        }
      }

      // Mark order as completed
      await updateOrderStatus(
        orderId: orderId,
        status: 'completed',
      );

      print('✅ Order execution completed successfully!');

      return {
        'success': true,
        'message': 'Order executed successfully',
        'orderId': orderId,
        'waypointsExecuted': waypoints.length,
        'executionTime': DateTime.now().toIso8601String(),
      };
    } catch (e) {
      // Mark order as failed
      try {
        await updateOrderStatus(
          orderId: orderId,
          status: 'failed',
          reason: e.toString(),
        );
      } catch (updateError) {
        print('❌ Failed to update order status to failed: $updateError');
      }

      print('❌ Order execution failed: $e');
      throw ApiException('Order execution failed: $e');
    }
  }

  /// Batch execute multiple orders with delays
  Future<Map<String, dynamic>> executeBatchOrders(
    String deviceId,
    List<String> orderIds, {
    Duration orderDelay = const Duration(minutes: 1),
    Function(int currentOrder, int totalOrders)? onOrderProgress,
    Function(int currentWaypoint, int totalWaypoints)? onWaypointProgress,
  }) async {
    _ensureInitialized();

    try {
      final results = <Map<String, dynamic>>[];

      for (int i = 0; i < orderIds.length; i++) {
        final orderId = orderIds[i];
        print('📦 Executing batch order ${i + 1}/${orderIds.length}: $orderId');

        // Call order progress callback if provided
        onOrderProgress?.call(i + 1, orderIds.length);

        final result = await executeOrderWithTracking(
          deviceId,
          orderId,
          onProgress: onWaypointProgress,
        );

        results.add(result);

        // Wait between orders (except for last one)
        if (i < orderIds.length - 1 && result['success'] == true) {
          print('⏳ Waiting ${orderDelay.inMinutes}min before next order...');
          await Future.delayed(orderDelay);
        }
      }

      final successCount = results.where((r) => r['success'] == true).length;

      return {
        'success': true,
        'message': 'Batch execution completed',
        'totalOrders': orderIds.length,
        'successfulOrders': successCount,
        'failedOrders': orderIds.length - successCount,
        'results': results,
        'completedAt': DateTime.now().toIso8601String(),
      };
    } catch (e) {
      print('❌ Batch execution failed: $e');
      throw ApiException('Batch execution failed: $e');
    }
  }

// ==========================================
// CONVENIENCE METHODS FOR QUICK OPERATIONS
// ==========================================

  /// Quick order creation from station IDs
  Future<Map<String, dynamic>> createQuickOrder(
    String deviceId,
    String fromStationId,
    String toStationId, {
    String? AMRSerial,
    String? orderName,
    int priority = 0,
  }) async {
    _ensureInitialized();

    try {
      // Get available stations
      final stationsResponse = await getMapStations(deviceId);
      if (stationsResponse['success'] != true) {
        throw Exception('Failed to get stations: ${stationsResponse['error']}');
      }

      final stations = stationsResponse['stations'] as List<dynamic>? ?? [];

      final fromStation = stations.firstWhere(
        (s) => s['id'] == fromStationId,
        orElse: () => null,
      );

      final toStation = stations.firstWhere(
        (s) => s['id'] == toStationId,
        orElse: () => null,
      );

      if (fromStation == null || toStation == null) {
        throw Exception('Station not found');
      }

      final waypoints = [
        {
          'name': fromStation['name'],
          'type': fromStation['type'],
          'position': fromStation['position'],
          'metadata': {
            'stationId': fromStation['id'],
            'isStart': true,
          },
        },
        {
          'name': toStation['name'],
          'type': toStation['type'],
          'position': toStation['position'],
          'metadata': {
            'stationId': toStation['id'],
            'isEnd': true,
          },
        },
      ];

      return await createOrder(
        deviceId: deviceId,
        name: orderName ?? '${fromStation['name']} → ${toStation['name']}',
        waypoints: waypoints,
        priority: priority,
        description:
            'Quick order from ${fromStation['name']} to ${toStation['name']}${AMRSerial != null ? ' (AMR: $AMRSerial)' : ''}',
      );
    } catch (e) {
      print('❌ Error creating quick order: $e');
      throw ApiException('Failed to create quick order: $e');
    }
  }

  /// Get comprehensive fleet summary
  Future<Map<String, dynamic>> getFleetSummary() async {
    _ensureInitialized();

    try {
      final devicesResponse = await getDevices();
      final ordersResponse = await getAllOrders(limit: 1000);

      if (devicesResponse.isEmpty) {
        print('⚠️ No devices found');
      }

      if (ordersResponse['success'] != true) {
        throw Exception('Failed to get orders: ${ordersResponse['error']}');
      }

      final devices = devicesResponse;
      final orders = ordersResponse['orders'] as List<dynamic>? ?? [];

      // Calculate summary statistics
      final statusCounts = <String, int>{};
      final deviceOrderCounts = <String, int>{};

      for (final order in orders) {
        final status = order['status'] ?? 'unknown';
        final deviceId = order['deviceId'] ?? 'unknown';

        statusCounts[status] = (statusCounts[status] ?? 0) + 1;
        deviceOrderCounts[deviceId] = (deviceOrderCounts[deviceId] ?? 0) + 1;
      }

      return {
        'success': true,
        'fleet': {
          'totalDevices': devices.length,
          'totalOrders': orders.length,
          'activeOrders': statusCounts['active'] ?? 0,
          'pendingOrders': statusCounts['pending'] ?? 0,
          'completedOrders': statusCounts['completed'] ?? 0,
          'failedOrders': statusCounts['failed'] ?? 0,
        },
        'devices': devices,
        'ordersByStatus': statusCounts,
        'ordersByDevice': deviceOrderCounts,
        'recentOrders': orders.take(10).toList(),
        'timestamp': DateTime.now().toIso8601String(),
      };
    } catch (e) {
      print('❌ Error getting fleet summary: $e');
      throw ApiException('Failed to get fleet summary: $e');
    }
  }

  /// Get available devices (enhanced version)
  Future<Map<String, dynamic>> getAvailableDevices() async {
    _ensureInitialized();

    try {
      final response = await _get('/api/discovery/devices');

      if (response['success'] == true) {
        return response;
      }

      // Fallback to existing getDevices method
      final devices = await getDevices();
      return {
        'success': true,
        'devices': devices,
        'source': 'fallback',
      };
    } catch (e) {
      print('❌ Error getting available devices: $e');
      throw ApiException('Failed to get available devices: $e');
    }
  }

  /// Enhanced device status with more details
  Future<Map<String, dynamic>> getEnhancedDeviceStatus(String deviceId) async {
    _ensureInitialized();

    try {
      // Try ROS2-specific status endpoint first
      try {
        final response = await _get('/api/ros2/$deviceId/status');
        if (response['success'] == true) {
          return response;
        }
      } catch (e) {
        print('⚠️ ROS2 status endpoint not available: $e');
      }

      // Fallback to existing method
      return await getDeviceStatus(deviceId);
    } catch (e) {
      print('❌ Error getting enhanced device status: $e');
      throw ApiException('Failed to get enhanced device status: $e');
    }
  }
}
