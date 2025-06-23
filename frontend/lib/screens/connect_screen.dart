// screens/connect_screen.dart - Enhanced with Manual WebSocket & AGV Connection
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

class _ConnectScreenState extends State<ConnectScreen> {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();
  final NetworkDiscoveryService _discoveryService = NetworkDiscoveryService();

  // Auto-connection controllers
  final _deviceIdController = TextEditingController(text: 'agv_01');
  final _deviceNameController = TextEditingController(text: 'Primary AGV');

  // ✅ NEW: Manual connection controllers
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

  @override
  void initState() {
    super.initState();
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
    super.dispose();
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

  // ✅ NEW: Load saved connection settings
  void _loadSavedConnections() {
    // You can implement SharedPreferences here to save/load last used IPs
    // For now, we'll populate with common defaults
    setState(() {
      _manualBackendIpController.text = '192.168.253.79'; // Your current setup
      _manualDeviceIpController.text = '192.168.253.136'; // Your AGV IP
    });
  }

  // Auto-discovery methods (keeping existing ones)
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

  // ✅ NEW: Manual WebSocket connection
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
      // Test API connection first
      _apiService.setBaseUrl('http://$ip:$port');
      final apiWorking = await _apiService.testConnection();

      if (!apiWorking) {
        throw Exception('Backend API not responding at $ip:$port');
      }

      // Connect WebSocket
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
          _detectedBackendIP = ip; // Update detected IP
        });

        _showSuccessSnackBar('Manually connected to AGV backend at $ip:$port');
        _saveConnectionSettings(); // Save for next time
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

  // ✅ NEW: Manual AGV device connection
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
      // Test if device is reachable
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

        // Clear form
        _manualDeviceIdController.clear();
        _manualDeviceNameController.clear();
        _manualDeviceIpController.clear();

        _showDashboardDialog();
      } else {
        _showErrorSnackBar('Failed to connect device: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Failed to connect AGV device: $e');
    } finally {
      setState(() {
        _isConnecting = false;
      });
    }
  }

  // ✅ NEW: Test device connection
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

  // ✅ NEW: IP validation
  bool _isValidIP(String ip) {
    final parts = ip.split('.');
    if (parts.length != 4) return false;

    for (final part in parts) {
      final num = int.tryParse(part);
      if (num == null || num < 0 || num > 255) return false;
    }

    return true;
  }

  // ✅ NEW: Save connection settings
  void _saveConnectionSettings() {
    // Implement SharedPreferences to save last used IPs
    print('💾 Saving connection settings for future use');
  }

  void _loadConnectedDevices() async {
    setState(() {
      _isLoading = true;
    });

    try {
      final devices = await _apiService.getDevices();
      setState(() {
        _connectedDevices = devices;
      });
    } catch (e) {
      print('❌ Error loading devices: $e');
      _showErrorSnackBar('Failed to load devices: $e');
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
        title: Text('Device Connected'),
        content: Text('Do you want to go to the Dashboard?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Stay'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              Navigator.pushNamed(context, '/dashboard');
            },
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
        title: Text('AGV Fleet Connection'),
        backgroundColor: _isWebSocketConnected ? Colors.green : Colors.orange,
        actions: [
          IconButton(
            icon: Icon(Icons.refresh),
            onPressed: _isLoading
                ? null
                : () {
                    _loadConnectedDevices();
                    _startAutoDiscovery();
                  },
          ),
          IconButton(
            icon: Icon(Icons.search),
            onPressed: _isDiscovering ? null : _discoverAGVDevices,
          ),
        ],
      ),
      body: _isLoading
          ? Center(child: CircularProgressIndicator())
          : SingleChildScrollView(
              padding: EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  _buildConnectionStatusCard(),
                  SizedBox(height: 16),
                  _buildAutoDiscoveryCard(),
                  SizedBox(height: 16),
                  _buildManualWebSocketCard(), // ✅ NEW
                  SizedBox(height: 16),
                  _buildManualAGVCard(), // ✅ NEW
                  SizedBox(height: 16),
                  _buildAutoConnectCard(),
                  SizedBox(height: 16),
                  if (_discoveredDevices.isNotEmpty) ...[
                    _buildDiscoveredDevicesCard(),
                    SizedBox(height: 16),
                  ],
                  _buildConnectedDevicesCard(),
                ],
              ),
            ),
    );
  }

  // Existing cards (keeping your current implementation)
  Widget _buildConnectionStatusCard() {
    return Card(
      color:
          _isWebSocketConnected ? Colors.green.shade50 : Colors.orange.shade50,
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Row(
          children: [
            Icon(
              _isWebSocketConnected ? Icons.wifi : Icons.wifi_off,
              color: _isWebSocketConnected ? Colors.green : Colors.orange,
              size: 32,
            ),
            SizedBox(width: 16),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    _connectionStatus,
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                      color: _isWebSocketConnected
                          ? Colors.green.shade800
                          : Colors.orange.shade800,
                    ),
                  ),
                  if (_detectedBackendIP != null)
                    Text(
                      'Backend: $_detectedBackendIP:3000',
                      style: TextStyle(color: Colors.grey[600], fontSize: 12),
                    ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildAutoDiscoveryCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.network_check, color: Colors.blue),
                SizedBox(width: 8),
                Text(
                  'Auto Network Discovery',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            SizedBox(height: 12),
            Text('Automatically find AGV backends and devices on your network'),
            SizedBox(height: 16),
            SizedBox(
              width: double.infinity,
              child: ElevatedButton.icon(
                onPressed: _isDiscovering ? null : _startAutoDiscovery,
                icon: _isDiscovering
                    ? SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(strokeWidth: 2))
                    : Icon(Icons.search),
                label: Text(_isDiscovering
                    ? 'Discovering...'
                    : 'Auto-Discover Network'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.blue,
                  padding: EdgeInsets.symmetric(vertical: 12),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  // ✅ NEW: Manual WebSocket connection card
  Widget _buildManualWebSocketCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.settings_ethernet, color: Colors.purple),
                SizedBox(width: 8),
                Text(
                  'Manual Backend Connection',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            SizedBox(height: 12),
            Text('Connect to a specific AGV Fleet Backend using IP address'),
            SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  flex: 3,
                  child: TextField(
                    controller: _manualBackendIpController,
                    decoration: InputDecoration(
                      labelText: 'Backend IP Address',
                      prefixIcon: Icon(Icons.computer),
                      hintText: '192.168.253.79',
                      border: OutlineInputBorder(),
                    ),
                    keyboardType:
                        TextInputType.numberWithOptions(decimal: true),
                  ),
                ),
                SizedBox(width: 8),
                Expanded(
                  flex: 1,
                  child: TextField(
                    controller: _manualBackendPortController,
                    decoration: InputDecoration(
                      labelText: 'Port',
                      hintText: '3000',
                      border: OutlineInputBorder(),
                    ),
                    keyboardType: TextInputType.number,
                  ),
                ),
              ],
            ),
            SizedBox(height: 16),
            SizedBox(
              width: double.infinity,
              child: ElevatedButton.icon(
                onPressed: _isConnecting ? null : _connectManualWebSocket,
                icon: _isConnecting
                    ? SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(strokeWidth: 2))
                    : Icon(Icons.wifi),
                label: Text(
                    _isConnecting ? 'Connecting...' : 'Connect to Backend'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.purple,
                  padding: EdgeInsets.symmetric(vertical: 12),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  // ✅ NEW: Manual AGV device connection card
  Widget _buildManualAGVCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.precision_manufacturing, color: Colors.orange),
                SizedBox(width: 8),
                Text(
                  'Manual AGV Device Connection',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            SizedBox(height: 12),
            Text('Add a specific AGV device using its IP address'),
            SizedBox(height: 16),
            TextField(
              controller: _manualDeviceIdController,
              decoration: InputDecoration(
                labelText: 'Device ID',
                prefixIcon: Icon(Icons.badge),
                hintText: 'agv_01',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 12),
            TextField(
              controller: _manualDeviceNameController,
              decoration: InputDecoration(
                labelText: 'Device Name (Optional)',
                prefixIcon: Icon(Icons.label),
                hintText: 'Primary AGV',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 12),
            Row(
              children: [
                Expanded(
                  flex: 3,
                  child: TextField(
                    controller: _manualDeviceIpController,
                    decoration: InputDecoration(
                      labelText: 'AGV IP Address',
                      prefixIcon: Icon(Icons.smart_toy),
                      hintText: '192.168.253.136',
                      border: OutlineInputBorder(),
                    ),
                    keyboardType:
                        TextInputType.numberWithOptions(decimal: true),
                  ),
                ),
                SizedBox(width: 8),
                Expanded(
                  flex: 1,
                  child: TextField(
                    controller: _manualDevicePortController,
                    decoration: InputDecoration(
                      labelText: 'Port',
                      hintText: '3000',
                      border: OutlineInputBorder(),
                    ),
                    keyboardType: TextInputType.number,
                  ),
                ),
              ],
            ),
            SizedBox(height: 16),
            SizedBox(
              width: double.infinity,
              child: ElevatedButton.icon(
                onPressed: _isConnecting ? null : _connectManualAGV,
                icon: _isConnecting
                    ? SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(strokeWidth: 2))
                    : Icon(Icons.add_circle),
                label: Text(_isConnecting ? 'Adding...' : 'Add AGV Device'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.orange,
                  padding: EdgeInsets.symmetric(vertical: 12),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildAutoConnectCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.auto_fix_high, color: Colors.green),
                SizedBox(width: 8),
                Text(
                  'Quick Auto-Connect',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            SizedBox(height: 12),
            Text('Automatically connect to your AGV with default settings'),
            SizedBox(height: 16),
            SizedBox(
              width: double.infinity,
              child: ElevatedButton.icon(
                onPressed: _isConnecting ? null : _autoConnectAGV,
                icon: _isConnecting
                    ? SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(strokeWidth: 2))
                    : Icon(Icons.smart_toy),
                label:
                    Text(_isConnecting ? 'Connecting...' : 'Auto-Connect AGV'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.green,
                  padding: EdgeInsets.symmetric(vertical: 12),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildDiscoveredDevicesCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Discovered AGV Devices (${_discoveredDevices.length})',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 12),
            ListView.separated(
              shrinkWrap: true,
              physics: NeverScrollableScrollPhysics(),
              itemCount: _discoveredDevices.length,
              separatorBuilder: (context, index) => Divider(),
              itemBuilder: (context, index) {
                final device = _discoveredDevices[index];
                return ListTile(
                  leading: Icon(Icons.router, color: Colors.blue),
                  title: Text(device.name),
                  subtitle: Text(
                      '${device.ipAddress}:${device.port} (${device.discoveryMethod})'),
                  trailing: ElevatedButton(
                    onPressed: () async {
                      await _connectToDetectedBackend(device.ipAddress);
                    },
                    child: Text('Connect'),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildConnectedDevicesCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Text(
                  'Connected Devices',
                  style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
                ),
                Spacer(),
                Text('Total: ${_connectedDevices.length}'),
              ],
            ),
            SizedBox(height: 16),
            if (_connectedDevices.isEmpty)
              Container(
                padding: EdgeInsets.all(32),
                width: double.infinity,
                decoration: BoxDecoration(
                  color: Colors.grey[100],
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Column(
                  children: [
                    Icon(Icons.devices_other, size: 64, color: Colors.grey),
                    SizedBox(height: 16),
                    Text(
                      'No Devices Connected',
                      style:
                          TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                    ),
                    SizedBox(height: 8),
                    Text(
                        'Use auto-discovery or manual connection to add AGV devices'),
                  ],
                ),
              )
            else
              ListView.separated(
                shrinkWrap: true,
                physics: NeverScrollableScrollPhysics(),
                itemCount: _connectedDevices.length,
                separatorBuilder: (context, index) => Divider(),
                itemBuilder: (context, index) {
                  final device = _connectedDevices[index];
                  return _buildDeviceListItem(device);
                },
              ),
          ],
        ),
      ),
    );
  }

  Widget _buildDeviceListItem(Map<String, dynamic> device) {
    final deviceId = device['id']?.toString() ?? '';
    final deviceName = device['name']?.toString() ?? deviceId;
    final deviceStatus = device['status']?.toString() ?? 'unknown';
    final deviceType = device['type']?.toString() ?? 'differential_drive';
    final isOnline = deviceStatus == 'connected';

    return ListTile(
      leading: Stack(
        children: [
          CircleAvatar(
            child: Icon(Icons.smart_toy),
            backgroundColor: isOnline ? Colors.green[100] : Colors.grey[300],
            foregroundColor: isOnline ? Colors.green[700] : Colors.grey[600],
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
              ),
            ),
          ),
        ],
      ),
      title: Text(
        deviceName,
        style: TextStyle(fontWeight: FontWeight.bold),
      ),
      subtitle: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('ID: $deviceId'),
          Text('Type: $deviceType'),
          Text(
            'Status: ${deviceStatus.toUpperCase()}',
            style: TextStyle(
              color: isOnline ? Colors.green : Colors.red,
              fontWeight: FontWeight.w500,
            ),
          ),
          if (device['ipAddress'] != null)
            Text(
              'IP: ${device['ipAddress']}',
              style: TextStyle(fontSize: 12, color: Colors.grey[600]),
            ),
        ],
      ),
      trailing: PopupMenuButton<String>(
        itemBuilder: (context) => [
          PopupMenuItem(
            value: 'control',
            child: ListTile(
              leading: Icon(Icons.gamepad),
              title: Text('Control'),
              dense: true,
            ),
          ),
          PopupMenuItem(
            value: 'map',
            child: ListTile(
              leading: Icon(Icons.map),
              title: Text('Map Editor'),
              dense: true,
            ),
          ),
          PopupMenuItem(
            value: 'status',
            child: ListTile(
              leading: Icon(Icons.info),
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
    );
  }

  void _handleDeviceAction(Map<String, dynamic> device, String action) {
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

  void _navigateToControl(Map<String, dynamic> device) {
    Navigator.pushNamed(
      context,
      '/control',
      arguments: {
        'deviceId': device['id'],
        'deviceName': device['name'],
      },
    );
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
        title: Text('Device Status'),
        content: Column(
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
            child: Text('Control'),
          ),
        ],
      ),
    );
  }

  Widget _buildStatusRow(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          SizedBox(
            width: 100,
            child: Text(
              '$label:',
              style: TextStyle(fontWeight: FontWeight.w500),
            ),
          ),
          Expanded(child: Text(value)),
        ],
      ),
    );
  }

  void _confirmRemoveDevice(Map<String, dynamic> device) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Remove Device'),
        content: Text(
            'Are you sure you want to remove "${device['name'] ?? device['id']}"?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
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
      final result = await _apiService.disconnectDevice(deviceId: deviceId);
      if (result['success'] == true) {
        _showSuccessSnackBar('Device removed');
        _loadConnectedDevices();
      } else {
        _showErrorSnackBar('Failed to remove device: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Failed to remove device: $e');
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
        content: Text(message),
        backgroundColor: Colors.green,
        duration: Duration(seconds: 2),
      ),
    );
  }

  void _showErrorSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.red,
        duration: Duration(seconds: 4),
      ),
    );
  }

  void _showWarningSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.orange,
        duration: Duration(seconds: 3),
      ),
    );
  }
}
