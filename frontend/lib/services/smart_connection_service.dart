// services/smart_connection_service.dart - Auto-discovery and flexible connection
import 'dart:async';
import 'dart:io';
import 'dart:convert';
import 'package:http/http.dart' as http;
import 'api_service.dart';
import 'web_socket_service.dart';

class SmartConnectionService {
  static final SmartConnectionService _instance =
      SmartConnectionService._internal();
  factory SmartConnectionService() => _instance;
  SmartConnectionService._internal();

  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();

  // Connection state
  bool _isConnected = false;
  String? _connectedBackendUrl;
  String? _connectedWebSocketUrl;
  Map<String, dynamic>? _backendInfo;

  // Discovery state
  bool _isDiscovering = false;
  List<Map<String, dynamic>> _discoveredBackends = [];

  // Stream controllers
  final StreamController<List<Map<String, dynamic>>> _backendsController =
      StreamController<List<Map<String, dynamic>>>.broadcast();
  final StreamController<String> _connectionStatusController =
      StreamController<String>.broadcast();
  final StreamController<bool> _isConnectedController =
      StreamController<bool>.broadcast();

  // Public streams
  Stream<List<Map<String, dynamic>>> get discoveredBackends =>
      _backendsController.stream;
  Stream<String> get connectionStatus => _connectionStatusController.stream;
  Stream<bool> get isConnectedStream => _isConnectedController.stream;

  // Getters
  bool get isConnected => _isConnected;
  bool get isDiscovering => _isDiscovering;
  String? get connectedBackendUrl => _connectedBackendUrl;
  String? get connectedWebSocketUrl => _connectedWebSocketUrl;
  List<Map<String, dynamic>> get availableBackends => _discoveredBackends;

  // ==========================================
  // AUTO-DISCOVERY METHODS
  // ==========================================

  /// Discover AMR backends on the current network
  Future<List<Map<String, dynamic>>> discoverBackends({
    Duration timeout = const Duration(seconds: 15),
    bool useKnownIPs = true,
    bool useNetworkScan = true,
  }) async {
    if (_isDiscovering) {
      print('🔄 Discovery already in progress...');
      return _discoveredBackends;
    }

    _isDiscovering = true;
    _discoveredBackends.clear();
    _updateConnectionStatus('Discovering AMR backends...');

    try {
      print('🔍 Starting AMR backend discovery...');

      // Get device's current network info
      final networkInfo = await _getDeviceNetworkInfo();
      if (networkInfo == null) {
        throw Exception('Could not determine network information');
      }

      final subnet = networkInfo['subnet']!;
      print('📡 Scanning subnet: $subnet.x');

      final List<Future<List<Map<String, dynamic>>>> discoveryTasks = [];

      // 1. Check known/common AMR backend IPs first
      if (useKnownIPs) {
        discoveryTasks.add(_discoverKnownBackends(subnet));
      }

      // 2. Scan network for AMR backends
      if (useNetworkScan) {
        discoveryTasks.add(_scanNetworkForBackends(subnet, timeout));
      }

      // Wait for all discovery methods to complete
      final results = await Future.wait(discoveryTasks, eagerError: false);

      // Combine and deduplicate results
      final allBackends = <Map<String, dynamic>>[];
      for (final backendList in results) {
        allBackends.addAll(backendList);
      }

      _discoveredBackends = _deduplicateBackends(allBackends);

      // Sort by connection quality/response time
      _discoveredBackends.sort((a, b) {
        final aTime = a['responseTime'] as int? ?? 999999;
        final bTime = b['responseTime'] as int? ?? 999999;
        return aTime.compareTo(bTime);
      });

      print(
          '✅ Discovery complete. Found ${_discoveredBackends.length} AMR backends');
      _backendsController.add(_discoveredBackends);

      return _discoveredBackends;
    } catch (e) {
      print('❌ Discovery error: $e');
      _updateConnectionStatus('Discovery failed: $e');
      return [];
    } finally {
      _isDiscovering = false;
    }
  }

  /// Discover known/common AMR backend IPs
  Future<List<Map<String, dynamic>>> _discoverKnownBackends(
      String subnet) async {
    print('🎯 Checking known AMR backend IPs...');

    final backends = <Map<String, dynamic>>[];

    // Common AMR backend IP patterns
    final knownIPs = [
      '${subnet}.79', // Your current laptop
      '${subnet}.136', // Your current AMR
      '${subnet}.100', // Common backend IP
      '${subnet}.101', // Common backend IP
      '${subnet}.110', // Common backend IP
      '${subnet}.200', // High-range IP
    ];

    for (final ip in knownIPs) {
      try {
        final backend = await _testBackendAtIP(ip);
        if (backend != null) {
          backends.add(backend);
          print('✅ Found known backend: ${backend['name']} at $ip');
        }
      } catch (e) {
        // Skip failed connections
      }
    }

    return backends;
  }

  /// Scan network for AMR backends
  Future<List<Map<String, dynamic>>> _scanNetworkForBackends(
      String subnet, Duration timeout) async {
    print('🔍 Scanning network for AMR backends...');

    final backends = <Map<String, dynamic>>[];

    // Define scan ranges (to avoid scanning entire subnet)
    final scanRanges = [
      {'start': 70, 'end': 90}, // Common device range
      {'start': 100, 'end': 120}, // Common server range
      {'start': 200, 'end': 220}, // High-end range
    ];

    final scanTasks = <Future<void>>[];

    for (final range in scanRanges) {
      for (int i = range['start']!; i <= range['end']!; i++) {
        final ip = '$subnet.$i';

        scanTasks.add(_scanIPForBackend(ip, backends));

        // Process in batches to avoid overwhelming network
        if (scanTasks.length >= 10) {
          await Future.wait(scanTasks, eagerError: false);
          scanTasks.clear();
          await Future.delayed(Duration(milliseconds: 50)); // Small delay
        }
      }
    }

    // Process remaining tasks
    if (scanTasks.isNotEmpty) {
      await Future.wait(scanTasks, eagerError: false);
    }

    print('✅ Network scan complete. Found ${backends.length} backends');
    return backends;
  }

  /// Test if an IP has an AMR backend
  Future<void> _scanIPForBackend(
      String ip, List<Map<String, dynamic>> backends) async {
    try {
      final backend = await _testBackendAtIP(ip);
      if (backend != null) {
        backends.add(backend);
        print('✅ Found backend: ${backend['name']} at $ip');
      }
    } catch (e) {
      // Skip failed connections
    }
  }

  /// Test a specific IP for AMR backend
  Future<Map<String, dynamic>?> _testBackendAtIP(String ip) async {
    final ports = [3000, 8080, 80]; // Common AMR backend ports

    for (final port in ports) {
      try {
        final stopwatch = Stopwatch()..start();

        final response = await http.get(
          Uri.parse('http://$ip:$port/health'),
          headers: {'Accept': 'application/json'},
        ).timeout(Duration(seconds: 3));

        stopwatch.stop();

        if (response.statusCode == 200) {
          try {
            final data = json.decode(response.body);

            // Check if this looks like an AMR fleet backend
            if (_isAMRBackend(data)) {
              return {
                'id': 'backend_${ip.replaceAll('.', '_')}',
                'name': _extractBackendName(data, ip),
                'ipAddress': ip,
                'port': port,
                'httpUrl': 'http://$ip:$port',
                'webSocketUrl': 'ws://$ip:$port',
                'responseTime': stopwatch.elapsedMilliseconds,
                'status': 'healthy',
                'version': data['version'] ?? '1.0.0',
                'capabilities': _extractCapabilities(data),
                'devices': _extractDeviceCount(data),
                'ros2Status': _extractROS2Status(data),
                'discoveryMethod': 'network_scan',
                'healthData': data,
              };
            }
          } catch (parseError) {
            // If JSON parsing fails but we got 200, might still be a backend
            return {
              'id': 'backend_${ip.replaceAll('.', '_')}',
              'name': 'Backend at $ip',
              'ipAddress': ip,
              'port': port,
              'httpUrl': 'http://$ip:$port',
              'webSocketUrl': 'ws://$ip:$port',
              'responseTime': stopwatch.elapsedMilliseconds,
              'status': 'unknown',
              'discoveryMethod': 'network_scan',
              'note': 'Responded but could not parse JSON',
            };
          }
        }
      } catch (e) {
        // Continue to next port
      }
    }

    return null;
  }

  /// Check if response data indicates AMR backend
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

  /// Extract backend name from health data
  String _extractBackendName(Map<String, dynamic> data, String ip) {
    if (data['serverInfo']?['name'] != null) {
      return data['serverInfo']['name'];
    }
    if (data['services']?['ros2'] != null) {
      return 'AMR Fleet Backend';
    }
    return 'Backend at $ip';
  }

  /// Extract capabilities from health data
  List<String> _extractCapabilities(Map<String, dynamic> data) {
    final capabilities = <String>[];

    if (data['services']?['ros2'] != null) capabilities.add('ROS2');
    if (data['services']?['webSocket'] != null) capabilities.add('WebSocket');
    if (data['services']?['storage'] != null) capabilities.add('Storage');
    if (data['services']?['discovery'] != null) capabilities.add('Discovery');
    if (data['capabilities'] != null) {
      capabilities.addAll(List<String>.from(data['capabilities']));
    }

    return capabilities.toSet().toList(); // Remove duplicates
  }

  /// Extract device count from health data
  int _extractDeviceCount(Map<String, dynamic> data) {
    return data['services']?['storage']?['devicesCount'] ??
        data['devices']?['total'] ??
        0;
  }

  /// Extract ROS2 status from health data
  String _extractROS2Status(Map<String, dynamic> data) {
    final ros2 = data['services']?['ros2'];
    if (ros2 != null) {
      return ros2['initialized'] == true ? 'connected' : 'disconnected';
    }
    return 'unknown';
  }

  // ==========================================
  // CONNECTION METHODS
  // ==========================================

  /// Connect to the best available backend
  Future<bool> connectToBestBackend() async {
    try {
      _updateConnectionStatus('Finding best backend...');

      // Discover backends if not already done
      if (_discoveredBackends.isEmpty) {
        await discoverBackends();
      }

      if (_discoveredBackends.isEmpty) {
        throw Exception('No AMR backends found on network');
      }

      // Try to connect to the best backend (first in sorted list)
      final bestBackend = _discoveredBackends.first;
      return await connectToBackend(bestBackend);
    } catch (e) {
      print('❌ Error connecting to best backend: $e');
      _updateConnectionStatus('Connection failed: $e');
      return false;
    }
  }

  /// Connect to a specific backend
  Future<bool> connectToBackend(Map<String, dynamic> backend) async {
    try {
      final httpUrl = backend['httpUrl'] as String;
      final wsUrl = backend['webSocketUrl'] as String;

      _updateConnectionStatus('Connecting to ${backend['name']}...');

      // Initialize API service with this backend
      _apiService.setBaseUrl(httpUrl);

      // Test API connection
      final apiConnected = await _apiService.testConnection();
      if (!apiConnected) {
        throw Exception('API connection failed');
      }

      // Connect WebSocket
      final wsConnected = await _webSocketService.connect(
        wsUrl,
        deviceId: 'flutter_app',
        deviceInfo: {
          'name': 'Flutter AMR Controller',
          'type': 'mobile_client',
          'platform': Platform.isAndroid ? 'android' : 'ios',
          'version': '1.0.0',
        },
      );

      if (!wsConnected) {
        throw Exception('WebSocket connection failed');
      }

      // Success!
      _isConnected = true;
      _connectedBackendUrl = httpUrl;
      _connectedWebSocketUrl = wsUrl;
      _backendInfo = backend;

      _updateConnectionStatus('Connected to ${backend['name']}');
      _isConnectedController.add(true);

      print('✅ Successfully connected to backend: ${backend['name']}');
      print('   HTTP: $httpUrl');
      print('   WebSocket: $wsUrl');

      return true;
    } catch (e) {
      print('❌ Error connecting to backend: $e');
      _updateConnectionStatus('Connection failed: $e');
      await disconnect();
      return false;
    }
  }

  /// Disconnect from current backend
  Future<void> disconnect() async {
    try {
      _updateConnectionStatus('Disconnecting...');

      _webSocketService.disconnect();

      _isConnected = false;
      _connectedBackendUrl = null;
      _connectedWebSocketUrl = null;
      _backendInfo = null;

      _updateConnectionStatus('Disconnected');
      _isConnectedController.add(false);

      print('🔌 Disconnected from backend');
    } catch (e) {
      print('❌ Error during disconnect: $e');
    }
  }

  // ==========================================
  // UTILITY METHODS
  // ==========================================

  /// Get device's current network information
  Future<Map<String, String>?> _getDeviceNetworkInfo() async {
    try {
      for (final interface in await NetworkInterface.list()) {
        for (final addr in interface.addresses) {
          if (addr.type == InternetAddressType.IPv4 &&
              !addr.isLoopback &&
              !addr.address.startsWith('169.254')) {
            // Skip link-local

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

  /// Remove duplicate backends
  List<Map<String, dynamic>> _deduplicateBackends(
      List<Map<String, dynamic>> backends) {
    final seen = <String>{};
    final unique = <Map<String, dynamic>>[];

    for (final backend in backends) {
      final key = '${backend['ipAddress']}:${backend['port']}';
      if (!seen.contains(key)) {
        seen.add(key);
        unique.add(backend);
      }
    }

    return unique;
  }

  /// Update connection status
  void _updateConnectionStatus(String status) {
    print('📡 Connection Status: $status');
    _connectionStatusController.add(status);
  }

  /// Get connection summary
  Map<String, dynamic> getConnectionSummary() {
    return {
      'isConnected': _isConnected,
      'connectedBackend': _backendInfo,
      'availableBackends': _discoveredBackends.length,
      'discoveredBackends': _discoveredBackends,
      'lastDiscovery': DateTime.now().toIso8601String(),
    };
  }

  /// Refresh backend list
  Future<void> refreshBackends() async {
    await discoverBackends();
  }

  void dispose() {
    _backendsController.close();
    _connectionStatusController.close();
    _isConnectedController.close();
  }
}
