//services/api_service.dart - CORRECTED to match your backend endpoints
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:http/http.dart' as http;

class ApiService {
  static final ApiService _instance = ApiService._internal();
  factory ApiService() => _instance;
  ApiService._internal();

  String baseUrl = 'http://192.168.253.79:3000'; // Update this IP if needed
  String get apiBaseUrl => '$baseUrl/api';
  Duration _timeout = Duration(seconds: 30);

  void setBaseUrl(String url) {
    baseUrl = url.endsWith('/') ? url.substring(0, url.length - 1) : url;
  }

  void setTimeout(Duration timeout) {
    _timeout = timeout;
  }

  // Debug helper
  void printConnectionInfo() {
    print('🔧 API Service Configuration:');
    print('   Base URL: $baseUrl');
    print('   API Base URL: $apiBaseUrl');
    print('   WebSocket URL: ${getWebSocketUrl()}');
    print('   Timeout: ${_timeout.inSeconds}s');
  }

  // Generic HTTP methods
  Future<Map<String, dynamic>> _makeRequest(
    String method,
    String endpoint, {
    Map<String, dynamic>? body,
    Map<String, String>? headers,
    bool useApiPrefix = true,
  }) async {
    try {
      final baseUrlToUse = useApiPrefix ? apiBaseUrl : baseUrl;
      final uri = Uri.parse('$baseUrlToUse$endpoint');
      final defaultHeaders = {
        'Content-Type': 'application/json',
        'Accept': 'application/json',
      };

      if (headers != null) {
        defaultHeaders.addAll(headers);
      }

      print('🌐 ${method.toUpperCase()} $uri');

      http.Response response;

      switch (method.toUpperCase()) {
        case 'GET':
          response = await http.get(uri, headers: defaultHeaders).timeout(_timeout);
          break;
        case 'POST':
          response = await http
              .post(
                uri,
                headers: defaultHeaders,
                body: body != null ? json.encode(body) : null,
              )
              .timeout(_timeout);
          break;
        case 'PUT':
          response = await http
              .put(
                uri,
                headers: defaultHeaders,
                body: body != null ? json.encode(body) : null,
              )
              .timeout(_timeout);
          break;
        case 'PATCH':
          response = await http
              .patch(
                uri,
                headers: defaultHeaders,
                body: body != null ? json.encode(body) : null,
              )
              .timeout(_timeout);
          break;
        case 'DELETE':
          response = await http.delete(uri, headers: defaultHeaders).timeout(_timeout);
          break;
        default:
          throw Exception('Unsupported HTTP method: $method');
      }

      print('📡 Response: ${response.statusCode} ${response.reasonPhrase}');

      if (response.statusCode >= 200 && response.statusCode < 300) {
        final responseData = json.decode(response.body);
        print('✅ Success: ${responseData['message'] ?? 'OK'}');
        return responseData;
      } else {
        print('❌ Error response body: ${response.body}');
        final errorBody = json.decode(response.body);
        throw ApiException(
          statusCode: response.statusCode,
          message: errorBody['message'] ?? 'Unknown error',
          details: errorBody,
        );
      }
    } on TimeoutException {
      print('⏰ Request timeout for $method $endpoint');
      throw ApiException(
        statusCode: 408,
        message: 'Request timeout',
        details: {'error': 'The request took too long to complete'},
      );
    } on SocketException catch (e) {
      print('🌐 Network error for $method $endpoint: $e');
      throw ApiException(
        statusCode: 0,
        message: 'Network error - cannot reach server',
        details: {'error': 'Could not connect to server: $e'},
      );
    } catch (e) {
      if (e is ApiException) {
        rethrow;
      }
      print('❌ Unexpected error for $method $endpoint: $e');
      throw ApiException(
        statusCode: 0,
        message: 'Unexpected error: $e',
        details: {'error': e.toString()},
      );
    }
  }

  // ==========================================
  // CORRECTED ENDPOINTS TO MATCH YOUR BACKEND
  // ==========================================

  // Health check - CORRECT
  Future<Map<String, dynamic>> healthCheck() async {
    return await _makeRequest('GET', '/health');
  }

  // Device Management - CORRECT (matches your backend)
  Future<List<Map<String, dynamic>>> getDevices() async {
    final response = await _makeRequest('GET', '/devices');
    return List<Map<String, dynamic>>.from(response['devices'] ?? []);
  }

  Future<Map<String, dynamic>> connectDevice({
    required String deviceId,
    required String deviceName,
    required String ipAddress,
  }) async {
    return await _makeRequest('POST', '/devices/connect', body: {
      'deviceId': deviceId,
      'deviceName': deviceName,
      'ipAddress': ipAddress,
    });
  }

  Future<Map<String, dynamic>> disconnectDevice(String deviceId) async {
    return await _makeRequest('DELETE', '/devices/$deviceId');
  }

  // NEW ROS2 Control Endpoints - CORRECTED to match your backend
  Future<Map<String, dynamic>> moveDevice({
    required String deviceId,
    required double linear,
    required double angular,
  }) async {
    // CORRECTED: Your backend expects /api/control/move (not /control/device/...)
    return await _makeRequest('POST', '/control/move', body: {
      'deviceId': deviceId,
      'linear': linear,
      'angular': angular,
    });
  }

  Future<Map<String, dynamic>> stopDevice(String deviceId) async {
    // CORRECTED: Your backend expects /api/control/stop
    return await _makeRequest('POST', '/control/stop', body: {
      'deviceId': deviceId,
    });
  }

  Future<Map<String, dynamic>> joystickControl({
    required String deviceId,
    required double x,
    required double y,
    required bool deadman,
    double sensitivity = 1.0,
  }) async {
    // CORRECTED: Your backend expects /api/control/joystick
    return await _makeRequest('POST', '/control/joystick', body: {
      'deviceId': deviceId,
      'x': x,
      'y': y,
      'deadman': deadman,
      'sensitivity': sensitivity,
    });
  }

  // ROS2 Status Endpoints - NEW (matching your backend)
  Future<Map<String, dynamic>> getROS2Status() async {
    return await _makeRequest('GET', '/ros2/status');
  }

  Future<Map<String, dynamic>> testROS2Connectivity() async {
    return await _makeRequest('GET', '/ros2/test');
  }

  // Map Management - CORRECT (matches your backend)
  Future<Map<String, dynamic>> getMapData(String deviceId) async {
    return await _makeRequest('GET', '/maps/$deviceId');
  }

  Future<Map<String, dynamic>> saveMapData({
    required String deviceId,
    required Map<String, dynamic> mapData,
    String? mapName,
  }) async {
    return await _makeRequest('POST', '/maps/save', body: {
      'deviceId': deviceId,
      'mapData': mapData,
      'mapName': mapName,
    });
  }

  Future<Map<String, dynamic>> updateMapData({
    required String deviceId,
    required Map<String, dynamic> mapUpdate,
  }) async {
    return await _makeRequest('POST', '/maps/update', body: {
      'deviceId': deviceId,
      'mapUpdate': mapUpdate,
    });
  }

  // Order Management - CORRECT (matches your backend)
  Future<List<Map<String, dynamic>>> getOrders(String deviceId) async {
    final response = await _makeRequest('GET', '/orders/$deviceId');
    return List<Map<String, dynamic>>.from(response['orders'] ?? []);
  }

  Future<Map<String, dynamic>> createOrder({
    required String deviceId,
    required List<Map<String, dynamic>> waypoints,
    required String orderName,
    String priority = 'normal',
  }) async {
    return await _makeRequest('POST', '/orders/create', body: {
      'deviceId': deviceId,
      'waypoints': waypoints,
      'orderName': orderName,
      'priority': priority,
    });
  }

  Future<Map<String, dynamic>> updateOrderStatus({
    required String orderId,
    required String status,
  }) async {
    return await _makeRequest('PATCH', '/orders/$orderId/status', body: {
      'status': status,
    });
  }

  // Live Data - CORRECT
  Future<Map<String, dynamic>> getLiveData(String deviceId) async {
    return await _makeRequest('GET', '/live/$deviceId');
  }

  // WebSocket URL - CORRECT
  String getWebSocketUrl() {
    final baseUri = Uri.parse(baseUrl);
    final wsScheme = baseUri.scheme == 'https' ? 'wss' : 'ws';
    final wsUrl = '$wsScheme://${baseUri.host}:${baseUri.port}';
    print('🔗 Generated WebSocket URL: $wsUrl');
    return wsUrl;
  }

  // Enhanced connectivity testing
  Future<bool> testConnection() async {
    try {
      print('🔍 Testing API connection...');
      printConnectionInfo();
      
      final health = await healthCheck();
      print('✅ API test successful: ${health['message']}');
      print('📊 Server status: ${health['data']?['status']}');
      print('🤖 ROS2 status: ${health['data']?['ros2Status']}');
      print('🔗 Connected devices: ${health['data']?['connectedDevices']}');
      return true;
    } catch (e) {
      print('❌ API test failed: $e');
      if (e is ApiException) {
        print('   Status Code: ${e.statusCode}');
        print('   Message: ${e.message}');
        if (e.isNetworkError) {
          print('   💡 Check if server is running at $baseUrl');
          print('   💡 Check if IP address is correct');
          print('   💡 Check if server is bound to 0.0.0.0 (not localhost)');
        }
      }
      return false;
    }
  }

  // Network utilities
  Future<bool> pingHost(String host, {int port = 3000}) async {
    try {
      print('🏓 Pinging $host:$port...');
      final socket = await Socket.connect(host, port, timeout: Duration(seconds: 5));
      socket.destroy();
      print('✅ Ping successful');
      return true;
    } catch (e) {
      print('❌ Ping failed: $e');
      return false;
    }
  }

  // Comprehensive connection info
  Future<Map<String, dynamic>> getConnectionInfo() async {
    try {
      final health = await healthCheck();
      return {
        'connected': true,
        'baseUrl': baseUrl,
        'apiBaseUrl': apiBaseUrl,
        'websocketUrl': getWebSocketUrl(),
        'serverInfo': health,
        'timestamp': DateTime.now().toIso8601String(),
      };
    } catch (e) {
      return {
        'connected': false,
        'error': e.toString(),
        'baseUrl': baseUrl,
        'apiBaseUrl': apiBaseUrl,
        'websocketUrl': getWebSocketUrl(),
        'timestamp': DateTime.now().toIso8601String(),
      };
    }
  }

  // Quick test methods for development
  Future<void> testAllEndpoints() async {
    print('🧪 Testing all API endpoints...');
    
    try {
      // Test health
      await healthCheck();
      print('✅ Health endpoint working');
    } catch (e) {
      print('❌ Health endpoint failed: $e');
    }
    
    try {
      // Test devices
      await getDevices();
      print('✅ Devices endpoint working');
    } catch (e) {
      print('❌ Devices endpoint failed: $e');
    }
    
    try {
      // Test ROS2 status
      await getROS2Status();
      print('✅ ROS2 status endpoint working');
    } catch (e) {
      print('❌ ROS2 status endpoint failed: $e');
    }
  }
}

class ApiException implements Exception {
  final int statusCode;
  final String message;
  final Map<String, dynamic> details;

  ApiException({
    required this.statusCode,
    required this.message,
    required this.details,
  });

  @override
  String toString() {
    return 'ApiException($statusCode): $message';
  }

  bool get isNetworkError => statusCode == 0;
  bool get isTimeout => statusCode == 408;
  bool get isServerError => statusCode >= 500;
  bool get isClientError => statusCode >= 400 && statusCode < 500;
  bool get isUnauthorized => statusCode == 401;
  bool get isForbidden => statusCode == 403;
  bool get isNotFound => statusCode == 404;
}