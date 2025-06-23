// services/api_service.dart - Updated for dynamic backend connection
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:http/http.dart' as http;
import '../models/map_data.dart';
import '../models/odom.dart';

class ApiService {
  static final ApiService _instance = ApiService._internal();
  factory ApiService() => _instance;
  ApiService._internal();

  // ✅ REMOVED: Hard-coded IP addresses
  String? _baseUrl; // Now dynamic
  Duration _timeout = Duration(seconds: 10);
  Map<String, String> _headers = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
  };

  // Connection state
  bool _isInitialized = false;
  Map<String, dynamic>? _serverInfo;

  // ==========================================
  // INITIALIZATION & CONFIGURATION
  // ==========================================

  /// Initialize the API service with dynamic base URL
  void initialize(String baseUrl) {
    _baseUrl = baseUrl.replaceAll('/api', ''); // Remove /api suffix if present
    _isInitialized = true;
    print('🔗 API Service initialized with base URL: $_baseUrl');
  }

  /// Set base URL dynamically (can be called multiple times)
  void setBaseUrl(String baseUrl) {
    _baseUrl = baseUrl.replaceAll('/api', '');
    _isInitialized = true;
    print('🔗 API Service base URL updated: $_baseUrl');
  }

  /// Auto-initialize with discovered backend
  Future<bool> autoInitialize() async {
    try {
      print('🔍 Auto-discovering AGV backend...');
      
      // Try to find backend on current network
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

  /// Discover AGV backend on current network
  Future<String?> _discoverBackend() async {
    try {
      // Get device's network subnet
      final networkInfo = await _getDeviceNetworkInfo();
      if (networkInfo == null) return null;

      final subnet = networkInfo['subnet']!;
      print('📡 Scanning subnet: $subnet.x for AGV backends');

      // Common AGV backend IP patterns
      final commonIPs = [
        '$subnet.79',   // Your current setup
        '$subnet.136',  // AGV device might run backend
        '$subnet.100',  // Common backend IP
        '$subnet.101',  // Common backend IP
        '$subnet.110',  // Common backend IP
        '$subnet.200',  // High-range IP
      ];

      // Test common IPs first (fastest)
      for (final ip in commonIPs) {
        final url = await _testBackendAtIP(ip);
        if (url != null) {
          print('✅ Found AGV backend at: $url');
          return url;
        }
      }

      // If not found in common IPs, scan broader range
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

  /// Test if IP has AGV backend
  Future<String?> _testBackendAtIP(String ip) async {
    final ports = [3000, 8080, 80]; // Common AGV backend ports
    
    for (final port in ports) {
      try {
        final response = await http.get(
          Uri.parse('http://$ip:$port/health'),
          headers: {'Accept': 'application/json'},
        ).timeout(Duration(seconds: 2));

        if (response.statusCode == 200) {
          final data = json.decode(response.body);
          
          // Check if this looks like an AGV backend
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

  /// Check if response indicates AGV backend
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

  /// Get device network information
  Future<Map<String, String>?> _getDeviceNetworkInfo() async {
    try {
      for (final interface in await NetworkInterface.list()) {
        for (final addr in interface.addresses) {
          if (addr.type == InternetAddressType.IPv4 && 
              !addr.isLoopback && 
              !addr.address.startsWith('169.254')) { // Skip link-local
            
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

  /// Print connection information for debugging
  void printConnectionInfo() {
    print('📊 API Service Connection Info:');
    print('   Initialized: $isInitialized');
    print('   Base URL: ${baseUrl ?? 'Not set'}');
    print('   API URL: ${apiBaseUrl ?? 'Not set'}');
    print('   WebSocket URL: ${getWebSocketUrl() ?? 'Not set'}');
    print('   Timeout: ${_timeout.inSeconds}s');
  }

  /// Get WebSocket URL
  String? getWebSocketUrl() {
    if (_baseUrl == null) return null;
    return _baseUrl!.replaceAll('http://', 'ws://').replaceAll('https://', 'wss://');
  }

  /// Get connection info for debugging
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

  /// Test API connection
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

  /// Check if server is reachable
  Future<bool> isServerReachable() async {
    try {
      await testConnection();
      return true;
    } catch (e) {
      return false;
    }
  }

  /// Get server information
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

  /// Auto-connect to the primary AGV
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

  /// Connect a new AGV device
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

  /// Get all connected devices
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

  /// Get specific device status
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

  /// Disconnect a device
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
  // CONTROL COMMANDS
  // ==========================================

  /// Send joystick control command
  Future<Map<String, dynamic>> joystickControl({
    required String deviceId,
    required double x, // -1.0 to 1.0
    required double y, // -1.0 to 1.0
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

  /// Send direct velocity command
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

  /// Move device (alias for sendVelocity)
  Future<Map<String, dynamic>> moveDevice({
    required String deviceId,
    required double linear,
    required double angular,
  }) async {
    return sendVelocity(deviceId: deviceId, linear: linear, angular: angular);
  }

  /// Emergency stop
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

  /// Set navigation goal
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

  /// Start mapping
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

  /// Stop mapping
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

  /// Get mapping status
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

  /// Save current map
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

  /// Save map data with full map object
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

  /// Get current map data
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

  /// Add shape to map
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

  /// Update map shape
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

  /// Delete map shape
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

  /// Get orders for a device
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
      // Return empty list if endpoint doesn't exist yet
      if (e is ApiException && e.statusCode == 404) {
        return [];
      }
      print('❌ Error getting orders: $e');
      throw ApiException('Failed to get orders: $e');
    }
  }

  /// Create new order
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

  /// Update order status
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

  /// Execute order
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

  /// Get ROS2 system status
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

  /// Test ROS2 connectivity
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

  /// Get overall system status
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

  /// Check server health
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

  /// Update theme preference
  Future<Map<String, dynamic>> updateTheme(bool isDarkMode) async {
    if (!isInitialized) {
      // Theme is optional, don't fail if not initialized
      return {'success': false, 'error': 'API not initialized'};
    }
    
    try {
      final response = await _post('/api/user/theme', {
        'isDarkMode': isDarkMode,
        'updatedAt': DateTime.now().toIso8601String(),
      });

      return response;
    } catch (e) {
      // Theme endpoint might not exist, fail silently
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

  Future<Map<String, dynamic>> _get(String endpoint) async {
    _ensureInitialized();
    
    try {
      final url = Uri.parse('$_baseUrl$endpoint');
      print('📡 GET: $url');

      final response = await http.get(url, headers: _headers).timeout(_timeout);
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

  /// Set custom timeout
  void setTimeout(Duration timeout) {
    _timeout = timeout;
  }

  /// Set custom headers
  void setHeaders(Map<String, String> headers) {
    _headers = {..._headers, ...headers};
  }
}

/// Custom exception for API errors
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