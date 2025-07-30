// screens/connect_screen.dart - Enhanced UI with Modern Design
import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:io';
import 'package:http/http.dart' as http;
import 'dart:convert';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import '../services/network_discovery_service.dart';

class ConnectScreen extends StatefulWidget {
  @override
  _ConnectScreenState createState() => _ConnectScreenState();
}

class _ConnectScreenState extends State<ConnectScreen>
    with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();
  final NetworkDiscoveryService _discoveryService = NetworkDiscoveryService();

  // Auto-connection controllers
  final _deviceIdController = TextEditingController(text: 'agv_01');
  final _deviceNameController = TextEditingController(text: 'Primary AGV');

  // Manual connection controllers
  final _manualBackendIpController = TextEditingController();
  final _manualBackendPortController = TextEditingController(text: '3000');
  final _manualDeviceIdController = TextEditingController();
  final _manualDeviceNameController = TextEditingController();
  final _manualDeviceIpController = TextEditingController();
  final _manualDevicePortController = TextEditingController(text: '3000');

  List<Map<String, dynamic>> _connectedDevices = [];
  List<AGVDevice> _discoveredDevices = [];
  String? _detectedBackendIP;
  bool _isLoading = false;
  bool _isConnecting = false;
  bool _isDiscovering = false;
  bool _isWebSocketConnected = false;
  String _connectionStatus = 'Disconnected';
  late StreamSubscription _deviceEventsSubscription;
  late StreamSubscription _connectionStateSubscription;

  // Animation controllers for enhanced UI
  late AnimationController _pulseController;
  late AnimationController _slideController;
  late Animation<double> _pulseAnimation;
  late Animation<Offset> _slideAnimation;

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _initializeConnections();
    _startAutoDiscovery();
    _loadSavedConnections();
  }

  @override
  void dispose() {
    _deviceIdController.dispose();
    _deviceNameController.dispose();
    _manualBackendIpController.dispose();
    _manualBackendPortController.dispose();
    _manualDeviceIdController.dispose();
    _manualDeviceNameController.dispose();
    _manualDeviceIpController.dispose();
    _manualDevicePortController.dispose();
    _deviceEventsSubscription.cancel();
    _connectionStateSubscription.cancel();
    _pulseController.dispose();
    _slideController.dispose();
    super.dispose();
  }

  void _initializeAnimations() {
    _pulseController = AnimationController(
      duration: Duration(seconds: 2),
      vsync: this,
    )..repeat(reverse: true);

    _slideController = AnimationController(
      duration: Duration(milliseconds: 800),
      vsync: this,
    );

    _pulseAnimation = Tween<double>(begin: 0.8, end: 1.2).animate(
      CurvedAnimation(parent: _pulseController, curve: Curves.easeInOut),
    );

    _slideAnimation = Tween<Offset>(
      begin: Offset(0, 0.1),
      end: Offset.zero,
    ).animate(CurvedAnimation(parent: _slideController, curve: Curves.easeOut));

    _slideController.forward();
  }

  void _initializeConnections() {
    // Subscribe to WebSocket connection state
    _connectionStateSubscription =
        _webSocketService.connectionState.listen((connected) {
      setState(() {
        _isWebSocketConnected = connected;
        _connectionStatus =
            connected ? 'Connected to AGV Fleet Backend' : 'Disconnected';
      });

      if (connected) {
        _loadConnectedDevices();
      }
    });

    // Subscribe to device events
    _deviceEventsSubscription = _webSocketService.deviceEvents.listen((event) {
      switch (event['type']) {
        case 'device_connected':
        case 'device_disconnected':
        case 'initial_data':
          _loadConnectedDevices();
          break;
      }
    });
  }

  void _loadSavedConnections() {
    setState(() {
      _manualBackendIpController.text = '192.168.0.55';
      _manualDeviceIpController.text = '192.168.0.93';
    });
  }

  void _startAutoDiscovery() async {
    setState(() {
      _connectionStatus = 'Searching for AGV Fleet Backend...';
    });

    try {
      final backendIP = await _detectBackendIP();

      if (backendIP != null) {
        setState(() {
          _detectedBackendIP = backendIP;
          _connectionStatus = 'Found backend at $backendIP';
        });

        await _connectToDetectedBackend(backendIP);
      } else {
        setState(() {
          _connectionStatus =
              'No AGV backend found - manual connection available';
        });
      }

      await _discoverAGVDevices();
    } catch (e) {
      setState(() {
        _connectionStatus = 'Auto-discovery failed: $e';
      });
    }
  }

  Future<String?> _detectBackendIP() async {
    final networkInfo = await _getNetworkInfo();
    if (networkInfo == null) return null;

    final subnet = networkInfo['subnet']!;
    print('🔍 Scanning subnet: $subnet for AGV backend...');

    final deviceIP = networkInfo['ip']!;
    final deviceIPLast = int.parse(deviceIP.split('.').last);

    final scanIPs = <String>[];
    scanIPs.add(deviceIP);

    for (int i = 70; i <= 90; i++) {
      if (i != deviceIPLast) {
        scanIPs.add('$subnet.$i');
      }
    }

    for (int offset = 1; offset <= 10; offset++) {
      final ip1 = deviceIPLast + offset;
      final ip2 = deviceIPLast - offset;

      if (ip1 <= 254) scanIPs.add('$subnet.$ip1');
      if (ip2 >= 1) scanIPs.add('$subnet.$ip2');
    }

    for (int i = 0; i < scanIPs.length; i += 5) {
      final batch = scanIPs.skip(i).take(5);
      final futures = batch.map((ip) => _testBackendAtIP(ip));
      final results = await Future.wait(futures);

      for (int j = 0; j < results.length; j++) {
        if (results[j] != null) {
          final foundIP = batch.elementAt(j);
          print('✅ Found AGV backend at: $foundIP');
          return foundIP;
        }
      }
    }

    return null;
  }

  Future<String?> _testBackendAtIP(String ip) async {
    try {
      final response = await http.get(
        Uri.parse('http://$ip:3000/health'),
        headers: {'Accept': 'application/json'},
      ).timeout(Duration(seconds: 2));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        if (data['success'] == true && data['status'] == 'healthy') {
          return ip;
        }
      }
    } catch (e) {
      // Ignore connection failures
    }
    return null;
  }

  Future<Map<String, String>?> _getNetworkInfo() async {
    try {
      for (final interface in await NetworkInterface.list()) {
        for (final addr in interface.addresses) {
          if (addr.type == InternetAddressType.IPv4 &&
              !addr.isLoopback &&
              (addr.address.startsWith('192.168.') ||
                  addr.address.startsWith('10.') ||
                  addr.address.startsWith('172.'))) {
            final ip = addr.address;
            final subnet = ip.split('.').take(3).join('.');
            return {
              'interface': interface.name,
              'ip': ip,
              'subnet': subnet,
            };
          }
        }
      }
    } catch (e) {
      print('❌ Error getting network info: $e');
    }
    return null;
  }

  Future<void> _connectToDetectedBackend(String ip) async {
    try {
      _apiService.setBaseUrl('http://$ip:3000');

      final apiWorking = await _apiService.testConnection();
      if (!apiWorking) {
        throw Exception('API connection failed');
      }

      final wsUrl = 'ws://$ip:3000';
      final wsConnected = await _webSocketService
          .connect(wsUrl, deviceId: 'flutter_app', deviceInfo: {
        'name': 'Flutter AGV Controller',
        'type': 'mobile_client',
        'platform': Platform.isAndroid ? 'android' : 'ios',
      });

      if (wsConnected) {
        setState(() {
          _connectionStatus = 'Connected to AGV Fleet Backend';
          _isWebSocketConnected = true;
        });

        _showSuccessSnackBar('Auto-connected to AGV backend at $ip');
      } else {
        throw Exception('WebSocket connection failed');
      }
    } catch (e) {
      print('❌ Error connecting to detected backend: $e');
      setState(() {
        _connectionStatus = 'Connection failed: $e';
      });
    }
  }

  Future<void> _discoverAGVDevices() async {
    setState(() {
      _isDiscovering = true;
    });

    try {
      final devices = await _discoveryService.discoverDevices(
        timeout: Duration(seconds: 15),
        useNetworkScan: true,
        useMDNS: false,
        useBroadcast: false,
      );

      setState(() {
        _discoveredDevices = devices;
      });

      print('🔍 Discovered ${devices.length} AGV devices');
    } catch (e) {
      print('❌ Device discovery failed: $e');
    } finally {
      setState(() {
        _isDiscovering = false;
      });
    }
  }

  void _connectManualWebSocket() async {
    final ip = _manualBackendIpController.text.trim();
    final port = _manualBackendPortController.text.trim();

    if (ip.isEmpty) {
      _showErrorSnackBar('Backend IP address is required');
      return;
    }

    if (!_isValidIP(ip)) {
      _showErrorSnackBar('Invalid IP address format');
      return;
    }

    setState(() {
      _isConnecting = true;
      _connectionStatus = 'Connecting to $ip:$port...';
    });

    try {
      _apiService.setBaseUrl('http://$ip:$port');
      final apiWorking = await _apiService.testConnection();

      if (!apiWorking) {
        throw Exception('Backend API not responding at $ip:$port');
      }

      final wsUrl = 'ws://$ip:$port';
      final wsConnected = await _webSocketService
          .connect(wsUrl, deviceId: 'flutter_app', deviceInfo: {
        'name': 'Flutter AGV Controller (Manual)',
        'type': 'mobile_client',
        'platform': Platform.isAndroid ? 'android' : 'ios',
      });

      if (wsConnected) {
        setState(() {
          _connectionStatus = 'Connected to AGV Fleet Backend';
          _isWebSocketConnected = true;
          _detectedBackendIP = ip;
        });

        _showSuccessSnackBar('Manually connected to AGV backend at $ip:$port');
        _saveConnectionSettings();
      } else {
        throw Exception('WebSocket connection failed');
      }
    } catch (e) {
      _showErrorSnackBar('Manual connection failed: $e');
      setState(() {
        _connectionStatus = 'Manual connection failed: $e';
      });
    } finally {
      setState(() {
        _isConnecting = false;
      });
    }
  }

  void _connectManualAGV() async {
    final deviceId = _manualDeviceIdController.text.trim();
    final deviceName = _manualDeviceNameController.text.trim();
    final deviceIp = _manualDeviceIpController.text.trim();
    final devicePort = _manualDevicePortController.text.trim();

    if (deviceId.isEmpty || deviceIp.isEmpty) {
      _showErrorSnackBar('Device ID and IP Address are required');
      return;
    }

    if (!_isValidIP(deviceIp)) {
      _showErrorSnackBar('Invalid device IP address format');
      return;
    }

    setState(() {
      _isConnecting = true;
    });

    try {
      // Check if API service is properly initialized
      if (!_apiService.isInitialized) {
        throw Exception(
            'Backend not connected. Please connect to backend first.');
      }

      // Test backend connection before attempting to add device
      final connectionWorking = await _apiService.testConnection();
      if (!connectionWorking) {
        throw Exception(
            'Backend connection lost. Please reconnect to backend.');
      }

      final deviceReachable = await _testDeviceConnection(deviceIp, devicePort);
      if (!deviceReachable) {
        _showWarningSnackBar(
            'Device at $deviceIp:$devicePort not responding, but adding anyway...');
      }

      final result = await _apiService.connectDevice(
        deviceId: deviceId,
        name: deviceName.isNotEmpty ? deviceName : 'AGV $deviceId',
        ipAddress: deviceIp,
        type: 'differential_drive',
        capabilities: ['mapping', 'navigation', 'remote_control'],
      );

      if (result['success'] == true) {
        _showSuccessSnackBar('AGV device connected successfully');
        _loadConnectedDevices();

        _manualDeviceIdController.clear();
        _manualDeviceNameController.clear();
        _manualDeviceIpController.clear();

        _showDashboardDialog();
      } else {
        _showErrorSnackBar(
            'Failed to connect device: ${result['error'] ?? 'Unknown error'}');
      }
    } catch (e) {
      String errorMessage = e.toString();

      // Provide more user-friendly error messages
      if (errorMessage.contains('Failed to parse response') ||
          errorMessage.contains('Network error') ||
          errorMessage.contains('Connection failed')) {
        errorMessage =
            'Connection to backend lost. Please check network and reconnect.';
      } else if (errorMessage.contains('API service not initialized')) {
        errorMessage =
            'Backend not connected. Please connect to backend first.';
      } else if (errorMessage.contains('Request timeout')) {
        errorMessage = 'Request timed out. Please check backend connection.';
      }

      _showErrorSnackBar('Failed to connect AGV device: $errorMessage');

      // If there's a connection issue, suggest reconnecting
      if (errorMessage.contains('connection') ||
          errorMessage.contains('network')) {
        _showWarningSnackBar(
            'Try reconnecting to the backend if the problem persists.');
      }
    } finally {
      setState(() {
        _isConnecting = false;
      });
    }
  }

  Future<bool> _testDeviceConnection(String ip, String port) async {
    try {
      final response = await http.get(
        Uri.parse('http://$ip:$port/health'),
        headers: {'Accept': 'application/json'},
      ).timeout(Duration(seconds: 3));

      return response.statusCode == 200;
    } catch (e) {
      print('❌ Device connection test failed: $e');
      return false;
    }
  }

  bool _isValidIP(String ip) {
    final parts = ip.split('.');
    if (parts.length != 4) return false;

    for (final part in parts) {
      final num = int.tryParse(part);
      if (num == null || num < 0 || num > 255) return false;
    }

    return true;
  }

  /// Check if backend connection is healthy
  Future<bool> _checkBackendConnection() async {
    try {
      if (!_apiService.isInitialized) {
        return false;
      }

      return await _apiService.testConnection();
    } catch (e) {
      print('❌ Backend connection check failed: $e');
      return false;
    }
  }

  /// Show connection status to user
  void _updateConnectionStatus() async {
    final isConnected = await _checkBackendConnection();
    setState(() {
      _isWebSocketConnected = isConnected;
      _connectionStatus = isConnected
          ? 'Connected to AGV Fleet Backend'
          : 'Backend connection lost - please reconnect';
    });
  }

  void _saveConnectionSettings() {
    print('💾 Saving connection settings for future use');
  }

  void _loadConnectedDevices() async {
    setState(() {
      _isLoading = true;
    });

    try {
      // Check if API service is initialized before loading devices
      if (!_apiService.isInitialized) {
        print('⚠️ API service not initialized, skipping device load');
        setState(() {
          _connectedDevices = [];
        });
        return;
      }

      final devices = await _apiService.getDevices();
      setState(() {
        _connectedDevices = devices;
      });
    } catch (e) {
      print('❌ Error loading devices: $e');

      String errorMessage = e.toString();

      // Only show error if it's not just an initialization issue
      if (!errorMessage.contains('API service not initialized')) {
        if (errorMessage.contains('Failed to parse response') ||
            errorMessage.contains('Network error') ||
            errorMessage.contains('Connection failed')) {
          _showWarningSnackBar(
              'Lost connection to backend. Some features may not work.');
        } else {
          _showErrorSnackBar('Failed to load devices: $errorMessage');
        }
      }

      setState(() {
        _connectedDevices = [];
      });
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  void _autoConnectAGV() async {
    setState(() {
      _isConnecting = true;
    });

    try {
      final result = await _apiService.autoConnectAGV();

      if (result['success'] == true) {
        _showSuccessSnackBar('AGV auto-connected successfully');
        _loadConnectedDevices();
        _showDashboardDialog();
      } else {
        _showErrorSnackBar('Auto-connect failed: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Auto-connect failed: $e');
    } finally {
      setState(() {
        _isConnecting = false;
      });
    }
  }

  void _showDashboardDialog() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
        title: Row(
          children: [
            Icon(Icons.check_circle, color: Colors.green, size: 28),
            SizedBox(width: 12),
            Text(
              'Device Connected',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
          ],
        ),
        content: Text(
          'Your AGV device has been successfully connected. Would you like to go to the Dashboard to manage your fleet?',
          style: TextStyle(fontSize: 16),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Stay Here'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              Navigator.pushNamed(context, '/dashboard');
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.blue,
              foregroundColor: Colors.white,
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(12),
              ),
            ),
            child: Text('Go to Dashboard'),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(
          'AGV Fleet Connection',
          style: TextStyle(fontWeight: FontWeight.bold),
        ),
        backgroundColor: _isWebSocketConnected ? Colors.green : Colors.blue,
        foregroundColor: Colors.white,
        elevation: 0,
        flexibleSpace: Container(
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: _isWebSocketConnected
                  ? [Colors.green.shade600, Colors.green.shade400]
                  : [Colors.blue.shade600, Colors.blue.shade400],
              begin: Alignment.topLeft,
              end: Alignment.bottomRight,
            ),
          ),
        ),
        actions: [
          IconButton(
            icon: Icon(Icons.refresh),
            onPressed: _isLoading
                ? null
                : () {
                    _updateConnectionStatus();
                    _loadConnectedDevices();
                    _startAutoDiscovery();
                  },
            tooltip: 'Refresh',
          ),
          IconButton(
            icon: Icon(Icons.search),
            onPressed: _isDiscovering ? null : _discoverAGVDevices,
            tooltip: 'Discover Devices',
          ),
        ],
      ),
      body: _isLoading
          ? Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  CircularProgressIndicator(
                    valueColor: AlwaysStoppedAnimation<Color>(Colors.blue),
                  ),
                  SizedBox(height: 16),
                  Text(
                    'Loading AGV Fleet...',
                    style: TextStyle(
                      fontSize: 16,
                      fontWeight: FontWeight.w500,
                      color: Colors.grey[600],
                    ),
                  ),
                ],
              ),
            )
          : Container(
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  begin: Alignment.topCenter,
                  end: Alignment.bottomCenter,
                  colors: [
                    Colors.grey.shade50,
                    Colors.white,
                  ],
                ),
              ),
              child: SlideTransition(
                position: _slideAnimation,
                child: SingleChildScrollView(
                  padding: EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      _buildEnhancedConnectionStatusCard(),
                      SizedBox(height: 20),
                      _buildEnhancedAutoDiscoveryCard(),
                      SizedBox(height: 20),
                      _buildEnhancedManualWebSocketCard(),
                      SizedBox(height: 20),
                      _buildEnhancedManualAGVCard(),
                      SizedBox(height: 20),
                      _buildEnhancedAutoConnectCard(),
                      SizedBox(height: 20),
                      if (_discoveredDevices.isNotEmpty) ...[
                        _buildEnhancedDiscoveredDevicesCard(),
                        SizedBox(height: 20),
                      ],
                      _buildEnhancedConnectedDevicesCard(),
                    ],
                  ),
                ),
              ),
            ),
    );
  }

  Widget _buildEnhancedConnectionStatusCard() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: _isWebSocketConnected
              ? [Colors.green.shade50, Colors.green.shade100]
              : [Colors.orange.shade50, Colors.orange.shade100],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(
          color: _isWebSocketConnected
              ? Colors.green.withOpacity(0.3)
              : Colors.orange.withOpacity(0.3),
          width: 1,
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Row(
          children: [
            Container(
              padding: EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: _isWebSocketConnected ? Colors.green : Colors.orange,
                borderRadius: BorderRadius.circular(16),
                boxShadow: [
                  BoxShadow(
                    color:
                        (_isWebSocketConnected ? Colors.green : Colors.orange)
                            .withOpacity(0.3),
                    blurRadius: 8,
                    offset: Offset(0, 2),
                  ),
                ],
              ),
              child: AnimatedBuilder(
                animation: _pulseAnimation,
                builder: (context, child) {
                  return Transform.scale(
                    scale: _isWebSocketConnected ? _pulseAnimation.value : 1.0,
                    child: Icon(
                      _isWebSocketConnected ? Icons.wifi : Icons.wifi_off,
                      color: Colors.white,
                      size: 28,
                    ),
                  );
                },
              ),
            ),
            SizedBox(width: 16),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    _isWebSocketConnected ? 'Connected' : 'Disconnected',
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                      fontSize: 18,
                      color: _isWebSocketConnected
                          ? Colors.green.shade800
                          : Colors.orange.shade800,
                    ),
                  ),
                  SizedBox(height: 4),
                  Text(
                    _connectionStatus,
                    style: TextStyle(
                      color: Colors.grey[700],
                      fontSize: 14,
                    ),
                  ),
                  if (_detectedBackendIP != null) ...[
                    SizedBox(height: 8),
                    Container(
                      padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                      decoration: BoxDecoration(
                        color: Colors.white.withOpacity(0.8),
                        borderRadius: BorderRadius.circular(8),
                      ),
                      child: Text(
                        'Backend: $_detectedBackendIP:3000',
                        style: TextStyle(
                          color: Colors.grey[600],
                          fontSize: 12,
                          fontWeight: FontWeight.w500,
                        ),
                      ),
                    ),
                  ],
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedAutoDiscoveryCard() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.blue.shade50, Colors.blue.shade100],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: Colors.blue.withOpacity(0.3)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.blue,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.blue.withOpacity(0.3),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child:
                      Icon(Icons.network_check, color: Colors.white, size: 24),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Auto Network Discovery',
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: Colors.blue.shade800,
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        'Find AGV backends and devices automatically',
                        style: TextStyle(
                          color: Colors.grey[600],
                          fontSize: 14,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            SizedBox(height: 20),
            Container(
              width: double.infinity,
              height: 50,
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [Colors.blue.shade600, Colors.blue.shade400],
                ),
                borderRadius: BorderRadius.circular(25),
                boxShadow: [
                  BoxShadow(
                    color: Colors.blue.withOpacity(0.3),
                    blurRadius: 8,
                    offset: Offset(0, 4),
                  ),
                ],
              ),
              child: ElevatedButton.icon(
                onPressed: _isDiscovering ? null : _startAutoDiscovery,
                icon: _isDiscovering
                    ? SizedBox(
                        width: 20,
                        height: 20,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          valueColor:
                              AlwaysStoppedAnimation<Color>(Colors.white),
                        ),
                      )
                    : Icon(Icons.search, color: Colors.white),
                label: Text(
                  _isDiscovering ? 'Discovering...' : 'Auto-Discover Network',
                  style: TextStyle(
                    color: Colors.white,
                    fontWeight: FontWeight.bold,
                    fontSize: 16,
                  ),
                ),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.transparent,
                  shadowColor: Colors.transparent,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(25),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedManualWebSocketCard() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.purple.shade50, Colors.purple.shade100],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: Colors.purple.withOpacity(0.3)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.purple,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.purple.withOpacity(0.3),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child: Icon(Icons.settings_ethernet,
                      color: Colors.white, size: 24),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Manual Backend Connection',
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: Colors.purple.shade800,
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        'Connect to a specific AGV Fleet Backend',
                        style: TextStyle(
                          color: Colors.grey[600],
                          fontSize: 14,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            SizedBox(height: 20),
            Row(
              children: [
                Expanded(
                  flex: 3,
                  child: Container(
                    decoration: BoxDecoration(
                      color: Colors.white,
                      borderRadius: BorderRadius.circular(12),
                      boxShadow: [
                        BoxShadow(
                          color: Colors.black.withOpacity(0.1),
                          blurRadius: 4,
                          offset: Offset(0, 2),
                        ),
                      ],
                    ),
                    child: TextField(
                      controller: _manualBackendIpController,
                      decoration: InputDecoration(
                        labelText: 'Backend IP Address',
                        prefixIcon: Icon(Icons.computer, color: Colors.purple),
                        hintText: '192.168.0.55',
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(12),
                          borderSide: BorderSide.none,
                        ),
                        filled: true,
                        fillColor: Colors.white,
                        labelStyle: TextStyle(color: Colors.purple.shade700),
                      ),
                      keyboardType:
                          TextInputType.numberWithOptions(decimal: true),
                    ),
                  ),
                ),
                SizedBox(width: 12),
                Expanded(
                  flex: 1,
                  child: Container(
                    decoration: BoxDecoration(
                      color: Colors.white,
                      borderRadius: BorderRadius.circular(12),
                      boxShadow: [
                        BoxShadow(
                          color: Colors.black.withOpacity(0.1),
                          blurRadius: 4,
                          offset: Offset(0, 2),
                        ),
                      ],
                    ),
                    child: TextField(
                      controller: _manualBackendPortController,
                      decoration: InputDecoration(
                        labelText: 'Port',
                        hintText: '3000',
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(12),
                          borderSide: BorderSide.none,
                        ),
                        filled: true,
                        fillColor: Colors.white,
                        labelStyle: TextStyle(color: Colors.purple.shade700),
                      ),
                      keyboardType: TextInputType.number,
                    ),
                  ),
                ),
              ],
            ),
            SizedBox(height: 20),
            Container(
              width: double.infinity,
              height: 50,
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [Colors.purple.shade600, Colors.purple.shade400],
                ),
                borderRadius: BorderRadius.circular(25),
                boxShadow: [
                  BoxShadow(
                    color: Colors.purple.withOpacity(0.3),
                    blurRadius: 8,
                    offset: Offset(0, 4),
                  ),
                ],
              ),
              child: ElevatedButton.icon(
                onPressed: _isConnecting ? null : _connectManualWebSocket,
                icon: _isConnecting
                    ? SizedBox(
                        width: 20,
                        height: 20,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          valueColor:
                              AlwaysStoppedAnimation<Color>(Colors.white),
                        ),
                      )
                    : Icon(Icons.wifi, color: Colors.white),
                label: Text(
                  _isConnecting ? 'Connecting...' : 'Connect to Backend',
                  style: TextStyle(
                    color: Colors.white,
                    fontWeight: FontWeight.bold,
                    fontSize: 16,
                  ),
                ),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.transparent,
                  shadowColor: Colors.transparent,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(25),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedManualAGVCard() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.orange.shade50, Colors.orange.shade100],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: Colors.orange.withOpacity(0.3)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.orange,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.orange.withOpacity(0.3),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child: Icon(Icons.precision_manufacturing,
                      color: Colors.white, size: 24),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Manual AGV Device Connection',
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: Colors.orange.shade800,
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        'Add a specific AGV device using its IP address',
                        style: TextStyle(
                          color: Colors.grey[600],
                          fontSize: 14,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            SizedBox(height: 20),
            _buildEnhancedTextField(
              controller: _manualDeviceIdController,
              label: 'Device ID',
              hint: 'agv_01',
              icon: Icons.badge,
              color: Colors.orange,
            ),
            SizedBox(height: 16),
            _buildEnhancedTextField(
              controller: _manualDeviceNameController,
              label: 'Device Name (Optional)',
              hint: 'Primary AGV',
              icon: Icons.label,
              color: Colors.orange,
            ),
            SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  flex: 3,
                  child: _buildEnhancedTextField(
                    controller: _manualDeviceIpController,
                    label: 'AGV IP Address',
                    hint: '192.168.0.89',
                    icon: Icons.smart_toy,
                    color: Colors.orange,
                    keyboardType:
                        TextInputType.numberWithOptions(decimal: true),
                  ),
                ),
                SizedBox(width: 12),
                Expanded(
                  flex: 1,
                  child: _buildEnhancedTextField(
                    controller: _manualDevicePortController,
                    label: 'Port',
                    hint: '3000',
                    icon: null,
                    color: Colors.orange,
                    keyboardType: TextInputType.number,
                  ),
                ),
              ],
            ),
            SizedBox(height: 20),
            Container(
              width: double.infinity,
              height: 50,
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [Colors.orange.shade600, Colors.orange.shade400],
                ),
                borderRadius: BorderRadius.circular(25),
                boxShadow: [
                  BoxShadow(
                    color: Colors.orange.withOpacity(0.3),
                    blurRadius: 8,
                    offset: Offset(0, 4),
                  ),
                ],
              ),
              child: ElevatedButton.icon(
                onPressed: _isConnecting ? null : _connectManualAGV,
                icon: _isConnecting
                    ? SizedBox(
                        width: 20,
                        height: 20,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          valueColor:
                              AlwaysStoppedAnimation<Color>(Colors.white),
                        ),
                      )
                    : Icon(Icons.add_circle, color: Colors.white),
                label: Text(
                  _isConnecting ? 'Adding...' : 'Add AGV Device',
                  style: TextStyle(
                    color: Colors.white,
                    fontWeight: FontWeight.bold,
                    fontSize: 16,
                  ),
                ),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.transparent,
                  shadowColor: Colors.transparent,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(25),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedTextField({
    required TextEditingController controller,
    required String label,
    required String hint,
    IconData? icon,
    required Color color,
    TextInputType? keyboardType,
  }) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(12),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 4,
            offset: Offset(0, 2),
          ),
        ],
      ),
      child: TextField(
        controller: controller,
        keyboardType: keyboardType,
        decoration: InputDecoration(
          labelText: label,
          prefixIcon: icon != null ? Icon(icon, color: color) : null,
          hintText: hint,
          border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(12),
            borderSide: BorderSide.none,
          ),
          filled: true,
          fillColor: Colors.white,
          labelStyle: TextStyle(color: color),
          hintStyle: TextStyle(color: Colors.grey[400]),
        ),
      ),
    );
  }

  Widget _buildEnhancedAutoConnectCard() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.green.shade50, Colors.green.shade100],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: Colors.green.withOpacity(0.3)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.green,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.green.withOpacity(0.3),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child:
                      Icon(Icons.auto_fix_high, color: Colors.white, size: 24),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Quick Auto-Connect',
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: Colors.green.shade800,
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        'Connect to your AGV with default settings',
                        style: TextStyle(
                          color: Colors.grey[600],
                          fontSize: 14,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            SizedBox(height: 20),
            Container(
              width: double.infinity,
              height: 50,
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [Colors.green.shade600, Colors.green.shade400],
                ),
                borderRadius: BorderRadius.circular(25),
                boxShadow: [
                  BoxShadow(
                    color: Colors.green.withOpacity(0.3),
                    blurRadius: 8,
                    offset: Offset(0, 4),
                  ),
                ],
              ),
              child: ElevatedButton.icon(
                onPressed: _isConnecting ? null : _autoConnectAGV,
                icon: _isConnecting
                    ? SizedBox(
                        width: 20,
                        height: 20,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          valueColor:
                              AlwaysStoppedAnimation<Color>(Colors.white),
                        ),
                      )
                    : Icon(Icons.smart_toy, color: Colors.white),
                label: Text(
                  _isConnecting ? 'Connecting...' : 'Auto-Connect AGV',
                  style: TextStyle(
                    color: Colors.white,
                    fontWeight: FontWeight.bold,
                    fontSize: 16,
                  ),
                ),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.transparent,
                  shadowColor: Colors.transparent,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(25),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedDiscoveredDevicesCard() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.indigo.shade50, Colors.indigo.shade100],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: Colors.indigo.withOpacity(0.3)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.indigo,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.indigo.withOpacity(0.3),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child: Icon(Icons.devices, color: Colors.white, size: 24),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: Text(
                    'Discovered AGV Devices (${_discoveredDevices.length})',
                    style: TextStyle(
                      fontSize: 18,
                      fontWeight: FontWeight.bold,
                      color: Colors.indigo.shade800,
                    ),
                  ),
                ),
              ],
            ),
            SizedBox(height: 16),
            ListView.separated(
              shrinkWrap: true,
              physics: NeverScrollableScrollPhysics(),
              itemCount: _discoveredDevices.length,
              separatorBuilder: (context, index) =>
                  Divider(color: Colors.indigo.shade200),
              itemBuilder: (context, index) {
                final device = _discoveredDevices[index];
                return Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.white.withOpacity(0.8),
                    borderRadius: BorderRadius.circular(12),
                  ),
                  child: ListTile(
                    leading: Container(
                      padding: EdgeInsets.all(8),
                      decoration: BoxDecoration(
                        color: Colors.indigo.shade100,
                        borderRadius: BorderRadius.circular(8),
                      ),
                      child: Icon(Icons.router, color: Colors.indigo),
                    ),
                    title: Text(
                      device.name,
                      style: TextStyle(fontWeight: FontWeight.bold),
                    ),
                    subtitle: Text(
                      '${device.ipAddress}:${device.port} (${device.discoveryMethod})',
                      style: TextStyle(color: Colors.grey[600]),
                    ),
                    trailing: ElevatedButton(
                      onPressed: () async {
                        await _connectToDetectedBackend(device.ipAddress);
                      },
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.indigo,
                        foregroundColor: Colors.white,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(12),
                        ),
                      ),
                      child: Text('Connect'),
                    ),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedConnectedDevicesCard() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.white, Colors.grey.shade50],
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: Colors.grey.withOpacity(0.3)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.blue,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.blue.withOpacity(0.3),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child:
                      Icon(Icons.devices_other, color: Colors.white, size: 24),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: Text(
                    'Connected Devices',
                    style: TextStyle(
                      fontSize: 20,
                      fontWeight: FontWeight.bold,
                      color: Colors.grey[800],
                    ),
                  ),
                ),
                Container(
                  padding: EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                  decoration: BoxDecoration(
                    color: Colors.blue.shade100,
                    borderRadius: BorderRadius.circular(20),
                  ),
                  child: Text(
                    'Total: ${_connectedDevices.length}',
                    style: TextStyle(
                      color: Colors.blue.shade700,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                ),
              ],
            ),
            SizedBox(height: 20),
            if (_connectedDevices.isEmpty)
              Container(
                padding: EdgeInsets.all(40),
                width: double.infinity,
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [Colors.grey.shade50, Colors.grey.shade100],
                    begin: Alignment.topCenter,
                    end: Alignment.bottomCenter,
                  ),
                  borderRadius: BorderRadius.circular(16),
                  border: Border.all(
                      color: Colors.grey.shade300, style: BorderStyle.none),
                ),
                child: Column(
                  children: [
                    Container(
                      padding: EdgeInsets.all(20),
                      decoration: BoxDecoration(
                        color: Colors.grey.shade200,
                        shape: BoxShape.circle,
                      ),
                      child: Icon(
                        Icons.devices_other,
                        size: 48,
                        color: Colors.grey.shade500,
                      ),
                    ),
                    SizedBox(height: 20),
                    Text(
                      'No Devices Connected',
                      style: TextStyle(
                        fontSize: 20,
                        fontWeight: FontWeight.bold,
                        color: Colors.grey[700],
                      ),
                    ),
                    SizedBox(height: 8),
                    Text(
                      _isWebSocketConnected
                          ? 'Backend connected. Use auto-discovery or manual connection\nto add AGV devices to your fleet'
                          : 'Connect to the backend first, then add AGV devices\nusing auto-discovery or manual connection',
                      textAlign: TextAlign.center,
                      style: TextStyle(
                        color: Colors.grey[600],
                        fontSize: 16,
                      ),
                    ),
                    if (!_isWebSocketConnected) ...[
                      SizedBox(height: 16),
                      Container(
                        padding:
                            EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                        decoration: BoxDecoration(
                          color: Colors.orange.shade50,
                          borderRadius: BorderRadius.circular(12),
                          border: Border.all(color: Colors.orange.shade200),
                        ),
                        child: Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(Icons.warning,
                                color: Colors.orange.shade700, size: 20),
                            SizedBox(width: 8),
                            Text(
                              'Backend Not Connected',
                              style: TextStyle(
                                color: Colors.orange.shade700,
                                fontWeight: FontWeight.w600,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ]
                  ],
                ),
              )
            else
              ListView.separated(
                shrinkWrap: true,
                physics: NeverScrollableScrollPhysics(),
                itemCount: _connectedDevices.length,
                separatorBuilder: (context, index) =>
                    Divider(color: Colors.grey.shade300),
                itemBuilder: (context, index) {
                  final device = _connectedDevices[index];
                  return _buildEnhancedDeviceListItem(device);
                },
              ),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedDeviceListItem(Map<String, dynamic> device) {
    final deviceId = device['id']?.toString() ?? '';
    final deviceName = device['name']?.toString() ?? deviceId;
    final deviceStatus = device['status']?.toString() ?? 'unknown';
    final deviceType = device['type']?.toString() ?? 'differential_drive';
    final isOnline = deviceStatus == 'connected';

    return Container(
      margin: EdgeInsets.symmetric(vertical: 4),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: isOnline
              ? [Colors.green.shade50, Colors.green.shade100]
              : [Colors.grey.shade50, Colors.grey.shade100],
          begin: Alignment.centerLeft,
          end: Alignment.centerRight,
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(
          color: isOnline
              ? Colors.green.withOpacity(0.3)
              : Colors.grey.withOpacity(0.3),
        ),
      ),
      child: ListTile(
        contentPadding: EdgeInsets.all(16),
        leading: Stack(
          children: [
            Container(
              padding: EdgeInsets.all(12),
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: isOnline
                      ? [Colors.green.shade300, Colors.green.shade500]
                      : [Colors.grey.shade300, Colors.grey.shade500],
                ),
                shape: BoxShape.circle,
                boxShadow: [
                  BoxShadow(
                    color: (isOnline ? Colors.green : Colors.grey)
                        .withOpacity(0.3),
                    blurRadius: 6,
                    offset: Offset(0, 2),
                  ),
                ],
              ),
              child: Icon(
                Icons.smart_toy,
                color: Colors.white,
                size: 24,
              ),
            ),
            Positioned(
              right: 0,
              bottom: 0,
              child: Container(
                width: 16,
                height: 16,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: isOnline ? Colors.green : Colors.red,
                  border: Border.all(color: Colors.white, width: 2),
                  boxShadow: [
                    BoxShadow(
                      color: Colors.black.withOpacity(0.2),
                      blurRadius: 2,
                      offset: Offset(0, 1),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
        title: Text(
          deviceName,
          style: TextStyle(
            fontWeight: FontWeight.bold,
            fontSize: 16,
            color: Colors.grey[800],
          ),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            SizedBox(height: 4),
            Text(
              'ID: $deviceId',
              style: TextStyle(color: Colors.grey[600], fontSize: 12),
            ),
            Text(
              'Type: $deviceType',
              style: TextStyle(color: Colors.grey[600], fontSize: 12),
            ),
            Container(
              margin: EdgeInsets.only(top: 4),
              padding: EdgeInsets.symmetric(horizontal: 8, vertical: 2),
              decoration: BoxDecoration(
                color: isOnline ? Colors.green : Colors.red,
                borderRadius: BorderRadius.circular(8),
              ),
              child: Text(
                'Status: ${deviceStatus.toUpperCase()}',
                style: TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  fontSize: 10,
                ),
              ),
            ),
            if (device['ipAddress'] != null)
              Padding(
                padding: EdgeInsets.only(top: 4),
                child: Text(
                  'IP: ${device['ipAddress']}',
                  style: TextStyle(
                    fontSize: 12,
                    color: Colors.grey[600],
                    fontFamily: 'monospace',
                  ),
                ),
              ),
          ],
        ),
        trailing: PopupMenuButton<String>(
          shape:
              RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
          itemBuilder: (context) => [
            PopupMenuItem(
              value: 'control',
              child: ListTile(
                leading: Icon(Icons.gamepad, color: Colors.blue),
                title: Text('Control'),
                dense: true,
              ),
            ),
            PopupMenuItem(
              value: 'map',
              child: ListTile(
                leading: Icon(Icons.map, color: Colors.green),
                title: Text('Map Editor'),
                dense: true,
              ),
            ),
            PopupMenuItem(
              value: 'status',
              child: ListTile(
                leading: Icon(Icons.info, color: Colors.orange),
                title: Text('Status'),
                dense: true,
              ),
            ),
            PopupMenuItem(
              value: 'remove',
              child: ListTile(
                leading: Icon(Icons.delete, color: Colors.red),
                title: Text('Remove', style: TextStyle(color: Colors.red)),
                dense: true,
              ),
            ),
          ],
          onSelected: (value) => _handleDeviceAction(device, value),
        ),
        onTap: () => _navigateToControl(device),
      ),
    );
  }

  void _handleDeviceAction(Map<String, dynamic> device, String action) async {
    // Check backend connectivity for actions that require API calls
    if (action == 'remove') {
      final isConnected = await _checkBackendConnection();
      if (!isConnected) {
        _showErrorSnackBar(
            'Backend not connected. Please reconnect to remove devices.');
        return;
      }
    }

    switch (action) {
      case 'control':
        _navigateToControl(device);
        break;
      case 'map':
        _navigateToMap(device);
        break;
      case 'status':
        _showDeviceStatus(device);
        break;
      case 'remove':
        _confirmRemoveDevice(device);
        break;
    }
  }

  void _navigateToControl(Map<String, dynamic> device) async {
    await Navigator.pushNamed(
      context,
      '/control',
      arguments: {
        'deviceId': device['id'],
        'deviceName': device['name'],
      },
    );
    Navigator.pushReplacementNamed(context, '/dashboard');
  }

  void _navigateToMap(Map<String, dynamic> device) {
    Navigator.pushNamed(
      context,
      '/map',
      arguments: {
        'deviceId': device['id'],
      },
    );
  }

  void _showDeviceStatus(Map<String, dynamic> device) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
        title: Row(
          children: [
            Icon(Icons.info, color: Colors.blue, size: 28),
            SizedBox(width: 12),
            Text(
              'Device Status',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
          ],
        ),
        content: Container(
          decoration: BoxDecoration(
            color: Colors.grey.shade50,
            borderRadius: BorderRadius.circular(12),
          ),
          padding: EdgeInsets.all(16),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              _buildStatusRow('Device ID', device['id']?.toString() ?? ''),
              _buildStatusRow('Name', device['name']?.toString() ?? ''),
              _buildStatusRow('Type', device['type']?.toString() ?? ''),
              _buildStatusRow('Status', device['status']?.toString() ?? ''),
              if (device['ipAddress'] != null)
                _buildStatusRow('IP Address', device['ipAddress'].toString()),
              if (device['connectedAt'] != null)
                _buildStatusRow(
                    'Connected At', _formatDateTime(device['connectedAt'])),
            ],
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Close'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _navigateToControl(device);
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.blue,
              foregroundColor: Colors.white,
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(12),
              ),
            ),
            child: Text('Control'),
          ),
        ],
      ),
    );
  }

  Widget _buildStatusRow(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 6),
      child: Row(
        children: [
          SizedBox(
            width: 100,
            child: Text(
              '$label:',
              style: TextStyle(
                fontWeight: FontWeight.w600,
                color: Colors.grey[700],
              ),
            ),
          ),
          Expanded(
            child: Text(
              value,
              style: TextStyle(
                color: Colors.grey[800],
                fontWeight: FontWeight.w500,
              ),
            ),
          ),
        ],
      ),
    );
  }

  void _confirmRemoveDevice(Map<String, dynamic> device) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
        title: Row(
          children: [
            Icon(Icons.warning, color: Colors.red, size: 28),
            SizedBox(width: 12),
            Text(
              'Remove Device',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
          ],
        ),
        content: Text(
          'Are you sure you want to remove "${device['name'] ?? device['id']}" from your fleet?',
          style: TextStyle(fontSize: 16),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.red,
              foregroundColor: Colors.white,
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(12),
              ),
            ),
            onPressed: () {
              Navigator.of(context).pop();
              _removeDevice(device['id']);
            },
            child: Text('Remove'),
          ),
        ],
      ),
    );
  }

  void _removeDevice(String deviceId) async {
    setState(() {
      _isLoading = true;
    });

    try {
      // Check if API service is properly initialized
      if (!_apiService.isInitialized) {
        throw Exception(
            'Backend not connected. Please connect to backend first.');
      }

      // Test connection before attempting to remove device
      final connectionWorking = await _apiService.testConnection();
      if (!connectionWorking) {
        throw Exception(
            'Backend connection lost. Please reconnect to backend.');
      }

      final result = await _apiService.disconnectDevice(deviceId: deviceId);
      if (result['success'] == true) {
        _showSuccessSnackBar('Device removed successfully');
        _loadConnectedDevices(); // Reload the device list
      } else {
        _showErrorSnackBar(
            'Failed to remove device: ${result['error'] ?? 'Unknown error'}');
      }
    } catch (e) {
      String errorMessage = e.toString();

      // Provide more user-friendly error messages
      if (errorMessage.contains('Failed to parse response') ||
          errorMessage.contains('Network error') ||
          errorMessage.contains('Connection failed')) {
        errorMessage =
            'Connection to backend lost. Please check network and reconnect.';
      } else if (errorMessage.contains('API service not initialized')) {
        errorMessage =
            'Backend not connected. Please connect to backend first.';
      } else if (errorMessage.contains('Request timeout')) {
        errorMessage = 'Request timed out. Please check backend connection.';
      }

      _showErrorSnackBar('Failed to remove device: $errorMessage');

      // If there's a connection issue, suggest reconnecting
      if (errorMessage.contains('connection') ||
          errorMessage.contains('network')) {
        _showWarningSnackBar(
            'Try reconnecting to the backend if the problem persists.');
      }
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  String _formatDateTime(dynamic dateTime) {
    try {
      if (dateTime is String) {
        final dt = DateTime.parse(dateTime);
        return '${dt.day}/${dt.month}/${dt.year} ${dt.hour}:${dt.minute.toString().padLeft(2, '0')}';
      }
      return dateTime.toString();
    } catch (e) {
      return 'Unknown';
    }
  }

  void _showSuccessSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Row(
          children: [
            Icon(Icons.check_circle, color: Colors.white),
            SizedBox(width: 12),
            Text(message),
          ],
        ),
        backgroundColor: Colors.green,
        duration: Duration(seconds: 2),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      ),
    );
  }

  void _showErrorSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Row(
          children: [
            Icon(Icons.error, color: Colors.white),
            SizedBox(width: 12),
            Expanded(child: Text(message)),
          ],
        ),
        backgroundColor: Colors.red,
        duration: Duration(seconds: 4),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      ),
    );
  }

  void _showWarningSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Row(
          children: [
            Icon(Icons.warning, color: Colors.white),
            SizedBox(width: 12),
            Expanded(child: Text(message)),
          ],
        ),
        backgroundColor: Colors.orange,
        duration: Duration(seconds: 3),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      ),
    );
  }
}
