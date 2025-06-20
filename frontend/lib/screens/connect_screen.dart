//connect_screen.dart - Updated with Success Popup
import 'package:flutter/material.dart';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import 'package:shared_preferences/shared_preferences.dart';

class ConnectScreen extends StatefulWidget {
  @override
  _ConnectScreenState createState() => _ConnectScreenState();
  // Add this static method to provide SharedPreferences instance
  static Future<SharedPreferences> getInstance() async {
    return await SharedPreferences.getInstance();
  }
}

class _ConnectScreenState extends State<ConnectScreen> {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();

  final _formKey = GlobalKey<FormState>();
  final _deviceIdController = TextEditingController();
  final _deviceNameController = TextEditingController();
  final _ipAddressController = TextEditingController();
  final _serverUrlController =
      TextEditingController(text: 'ws://192.168.253.79:3000');

  bool _isConnecting = false;
  bool _isServerConnected = false;
  bool _isDeviceConnected = false;
  List<Map<String, dynamic>> _connectedDevices = [];
  List<Map<String, dynamic>> _availableDevices = [];
  bool _isScanning = false;

  @override
  void initState() {
    super.initState();
    _checkServerConnection();
    _loadConnectedDevices();
    _subscribeToDeviceEvents();
  }

  @override
  void dispose() {
    _deviceIdController.dispose();
    _deviceNameController.dispose();
    _ipAddressController.dispose();
    _serverUrlController.dispose();
    super.dispose();
  }

  void _checkServerConnection() async {
    setState(() {
      _isServerConnected = _webSocketService.isConnected;
    });

    if (!_isServerConnected) {
      _connectToServer();
    }
  }

  void _subscribeToDeviceEvents() {
    _webSocketService.deviceEvents.listen((event) {
      if (event['type'] == 'device_connected' ||
          event['type'] == 'device_disconnected') {
        _loadConnectedDevices();
      }
    });

    _webSocketService.connectionState.listen((connected) {
      setState(() {
        _isServerConnected = connected;
      });
    });
  }

  void _loadConnectedDevices() async {
    try {
      final devices = await _apiService.getDevices();
      setState(() {
        _connectedDevices = devices;
        _isDeviceConnected = devices.isNotEmpty;
      });
    } catch (e) {
      print('Error loading devices: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Connect Devices'),
        actions: [
          IconButton(
            icon: Icon(
              _isServerConnected ? Icons.cloud_done : Icons.cloud_off,
              color: _isServerConnected ? Colors.green : Colors.red,
            ),
            onPressed: _showServerConnectionDialog,
          ),
          // Add dashboard navigation button if ready
          if (_isServerConnected && _isDeviceConnected)
            IconButton(
              icon: Icon(Icons.dashboard, color: Colors.green),
              onPressed: _navigateToDashboard,
              tooltip: 'Go to Dashboard',
            ),
        ],
      ),
      body: SingleChildScrollView(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            _buildServerStatusCard(),
            SizedBox(height: 16),
            _buildConnectedDevicesCard(),
            SizedBox(height: 16),
            _buildDeviceDiscoveryCard(),
            SizedBox(height: 16),
            _buildManualConnectCard(),
            SizedBox(height: 16),
            // Show success banner if both are connected
            if (_isServerConnected && _isDeviceConnected)
              _buildSuccessBanner(),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: _showAddDeviceDialog,
        child: Icon(Icons.add),
        tooltip: 'Add Device',
      ),
    );
  }

  Widget _buildSuccessBanner() {
    return Card(
      color: Colors.green[50],
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          children: [
            Row(
              children: [
                Icon(Icons.check_circle, color: Colors.green, size: 32),
                SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Connection Successful! 🎉',
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: Colors.green[700],
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        'Server and AGV devices are connected and ready',
                        style: TextStyle(color: Colors.green[600]),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _navigateToDashboard,
                    icon: Icon(Icons.dashboard),
                    label: Text('Go to Dashboard'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.green,
                      foregroundColor: Colors.white,
                      padding: EdgeInsets.symmetric(vertical: 12),
                    ),
                  ),
                ),
                SizedBox(width: 12),
                ElevatedButton.icon(
                  onPressed: _showConnectionSummary,
                  icon: Icon(Icons.info),
                  label: Text('Details'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.blue,
                    foregroundColor: Colors.white,
                    padding: EdgeInsets.symmetric(vertical: 12),
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildServerStatusCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(
                  _isServerConnected ? Icons.check_circle : Icons.error,
                  color: _isServerConnected ? Colors.green : Colors.red,
                ),
                SizedBox(width: 8),
                Text(
                  'Server Connection',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                Spacer(),
                TextButton(
                  onPressed: _isServerConnected
                      ? _disconnectFromServer
                      : _connectToServer,
                  child: Text(_isServerConnected ? 'Disconnect' : 'Connect'),
                ),
              ],
            ),
            SizedBox(height: 8),
            Text(
              _isServerConnected
                  ? 'Connected to fleet management server'
                  : 'Not connected to server',
              style: TextStyle(
                color: _isServerConnected ? Colors.green : Colors.red,
              ),
            ),
            if (!_isServerConnected) ...[
              SizedBox(height: 8),
              Text(
                'Server URL: ${_serverUrlController.text}',
                style: TextStyle(color: Colors.grey[600]),
              ),
            ],
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
                Icon(Icons.devices),
                SizedBox(width: 8),
                Text(
                  'Connected Devices (${_connectedDevices.length})',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                Spacer(),
                TextButton(
                  onPressed: _loadConnectedDevices,
                  child: Text('Refresh'),
                ),
              ],
            ),
            SizedBox(height: 16),
            if (_connectedDevices.isEmpty)
              Container(
                padding: EdgeInsets.all(24),
                width: double.infinity,
                decoration: BoxDecoration(
                  color: Colors.grey[100],
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Column(
                  children: [
                    Icon(Icons.devices_other, size: 48, color: Colors.grey),
                    SizedBox(height: 8),
                    Text(
                      'No devices connected',
                      style: TextStyle(fontSize: 16, color: Colors.grey[600]),
                    ),
                    SizedBox(height: 4),
                    Text(
                      'Add a device to get started',
                      style: TextStyle(color: Colors.grey[500]),
                    ),
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
    final isOnline = device['status'] == 'connected';

    return ListTile(
      leading: CircleAvatar(
        backgroundColor: isOnline ? Colors.green : Colors.red,
        child: Icon(
          Icons.smart_toy,
          color: Colors.white,
        ),
      ),
      title: Text(device['name'] ?? 'Unknown Device'),
      subtitle: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('ID: ${device['id']}'),
          Text('IP: ${device['ip'] ?? 'Unknown'}'),
          Text(
            'Status: ${device['status']?.toUpperCase() ?? 'UNKNOWN'}',
            style: TextStyle(
              color: isOnline ? Colors.green : Colors.red,
              fontWeight: FontWeight.w500,
            ),
          ),
        ],
      ),
      trailing: PopupMenuButton(
        itemBuilder: (context) => [
          PopupMenuItem(
            value: 'control',
            child: Row(
              children: [
                Icon(Icons.gamepad),
                SizedBox(width: 8),
                Text('Control'),
              ],
            ),
          ),
          PopupMenuItem(
            value: 'disconnect',
            child: Row(
              children: [
                Icon(Icons.link_off, color: Colors.red),
                SizedBox(width: 8),
                Text('Disconnect'),
              ],
            ),
          ),
        ],
        onSelected: (value) => _handleDeviceAction(device, value),
      ),
      onTap: () => _navigateToControl(device),
    );
  }

  Widget _buildDeviceDiscoveryCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.search),
                SizedBox(width: 8),
                Text(
                  'Device Discovery',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                Spacer(),
                if (_isScanning)
                  SizedBox(
                    width: 20,
                    height: 20,
                    child: CircularProgressIndicator(strokeWidth: 2),
                  )
                else
                  TextButton(
                    onPressed: _scanForDevices,
                    child: Text('Scan'),
                  ),
              ],
            ),
            SizedBox(height: 8),
            Text(
              'Automatically discover AGV devices on the network',
              style: TextStyle(color: Colors.grey[600]),
            ),
            SizedBox(height: 16),
            if (_availableDevices.isNotEmpty) ...[
              Text(
                'Found Devices:',
                style: TextStyle(fontWeight: FontWeight.w500),
              ),
              SizedBox(height: 8),
              ListView.builder(
                shrinkWrap: true,
                physics: NeverScrollableScrollPhysics(),
                itemCount: _availableDevices.length,
                itemBuilder: (context, index) {
                  final device = _availableDevices[index];
                  return ListTile(
                    dense: true,
                    leading: Icon(Icons.wifi),
                    title: Text(device['name'] ?? 'AGV Device'),
                    subtitle: Text('IP: ${device['ip']}'),
                    trailing: ElevatedButton(
                      onPressed: () => _connectToDiscoveredDevice(device),
                      child: Text('Connect'),
                    ),
                  );
                },
              ),
            ] else if (_isScanning)
              Center(
                child: Padding(
                  padding: EdgeInsets.all(16),
                  child: Text('Scanning for devices...'),
                ),
              )
            else
              Center(
                child: Padding(
                  padding: EdgeInsets.all(16),
                  child: Text(
                    'No devices found. Tap "Scan" to search.',
                    style: TextStyle(color: Colors.grey[600]),
                  ),
                ),
              ),
          ],
        ),
      ),
    );
  }

  Widget _buildManualConnectCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Form(
          key: _formKey,
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                children: [
                  Icon(Icons.settings_ethernet),
                  SizedBox(width: 8),
                  Text(
                    'Manual Connection',
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                ],
              ),
              SizedBox(height: 16),
              TextFormField(
                controller: _deviceIdController,
                decoration: InputDecoration(
                  labelText: 'Device ID',
                  border: OutlineInputBorder(),
                  hintText: 'e.g., agv_001',
                ),
                validator: (value) {
                  if (value == null || value.isEmpty) {
                    return 'Please enter a device ID';
                  }
                  return null;
                },
              ),
              SizedBox(height: 16),
              TextFormField(
                controller: _deviceNameController,
                decoration: InputDecoration(
                  labelText: 'Device Name',
                  border: OutlineInputBorder(),
                  hintText: 'e.g., Warehouse AGV 1',
                ),
                validator: (value) {
                  if (value == null || value.isEmpty) {
                    return 'Please enter a device name';
                  }
                  return null;
                },
              ),
              SizedBox(height: 16),
              TextFormField(
                controller: _ipAddressController,
                decoration: InputDecoration(
                  labelText: 'IP Address',
                  border: OutlineInputBorder(),
                  hintText: 'e.g., 192.168.253.79',
                ),
                validator: (value) {
                  if (value == null || value.isEmpty) {
                    return 'Please enter an IP address';
                  }
                  // Basic IP validation
                  final ipRegex = RegExp(r'^(\d{1,3}\.){3}\d{1,3}$');
                  if (!ipRegex.hasMatch(value)) {
                    return 'Please enter a valid IP address';
                  }
                  return null;
                },
              ),
              SizedBox(height: 24),
              SizedBox(
                width: double.infinity,
                child: ElevatedButton.icon(
                  onPressed: _isConnecting ? null : _connectDevice,
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
                      : Icon(Icons.link),
                  label:
                      Text(_isConnecting ? 'Connecting...' : 'Connect Device'),
                  style: ElevatedButton.styleFrom(
                    padding: EdgeInsets.symmetric(vertical: 16),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  void _showServerConnectionDialog() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Server Connection'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextFormField(
              controller: _serverUrlController,
              decoration: InputDecoration(
                labelText: 'Server URL',
                border: OutlineInputBorder(),
                hintText: 'ws://192.168.253.79:3000',
              ),
            ),
            SizedBox(height: 16),
            Text(
              'Current Status: ${_isServerConnected ? "Connected" : "Disconnected"}',
              style: TextStyle(
                color: _isServerConnected ? Colors.green : Colors.red,
              ),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              if (_isServerConnected) {
                _disconnectFromServer();
              } else {
                _connectToServer();
              }
            },
            child: Text(_isServerConnected ? 'Disconnect' : 'Connect'),
          ),
        ],
      ),
    );
  }

  void _showAddDeviceDialog() {
    final deviceIdController = TextEditingController();
    final deviceNameController = TextEditingController();
    final ipController = TextEditingController();

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Add Device'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextFormField(
              controller: deviceIdController,
              decoration: InputDecoration(
                labelText: 'Device ID',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 16),
            TextFormField(
              controller: deviceNameController,
              decoration: InputDecoration(
                labelText: 'Device Name',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 16),
            TextFormField(
              controller: ipController,
              decoration: InputDecoration(
                labelText: 'IP Address',
                border: OutlineInputBorder(),
              ),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _connectSpecificDevice(
                deviceIdController.text,
                deviceNameController.text,
                ipController.text,
              );
            },
            child: Text('Connect'),
          ),
        ],
      ),
    );
  }

  // Enhanced success popup
  void _showConnectionSuccessPopup() {
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(16),
        ),
        title: Column(
          children: [
            Container(
              padding: EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: Colors.green[100],
                shape: BoxShape.circle,
              ),
              child: Icon(
                Icons.check_circle,
                color: Colors.green,
                size: 48,
              ),
            ),
            SizedBox(height: 16),
            Text(
              'Connection Successful! 🎉',
              style: TextStyle(
                fontSize: 20,
                fontWeight: FontWeight.bold,
                color: Colors.green[700],
              ),
              textAlign: TextAlign.center,
            ),
          ],
        ),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(
              'Great! Your AGV fleet management system is ready to use.',
              textAlign: TextAlign.center,
              style: TextStyle(fontSize: 16),
            ),
            SizedBox(height: 16),
            Container(
              padding: EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: Colors.green[50],
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.green[200]!),
              ),
              child: Column(
                children: [
                  Row(
                    children: [
                      Icon(Icons.cloud_done, color: Colors.green, size: 20),
                      SizedBox(width: 8),
                      Text('Server Connected'),
                      Spacer(),
                      Icon(Icons.check, color: Colors.green),
                    ],
                  ),
                  SizedBox(height: 8),
                  Row(
                    children: [
                      Icon(Icons.smart_toy, color: Colors.green, size: 20),
                      SizedBox(width: 8),
                      Text('${_connectedDevices.length} Device(s) Connected'),
                      Spacer(),
                      Icon(Icons.check, color: Colors.green),
                    ],
                  ),
                ],
              ),
            ),
          ],
        ),
        actions: [
          Row(
            children: [
              Expanded(
                child: OutlinedButton(
                  onPressed: () => Navigator.of(context).pop(),
                  child: Text('Stay Here'),
                ),
              ),
              SizedBox(width: 12),
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: () {
                    Navigator.of(context).pop();
                    _navigateToDashboard();
                  },
                  icon: Icon(Icons.dashboard),
                  label: Text('Dashboard'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.green,
                    foregroundColor: Colors.white,
                  ),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  void _showConnectionSummary() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Connection Summary'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            _buildStatusRow('Server Connection', _isServerConnected),
            _buildStatusRow('Device Connection', _isDeviceConnected),
            SizedBox(height: 16),
            Text(
              'Connected Devices:',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            ..._connectedDevices.map((device) => Padding(
              padding: EdgeInsets.only(left: 16, bottom: 4),
              child: Row(
                children: [
                  Icon(Icons.smart_toy, size: 16, color: Colors.green),
                  SizedBox(width: 8),
                  Text(device['name'] ?? device['id']),
                ],
              ),
            )),
            SizedBox(height: 16),
            Text(
              'Server URL: ${_serverUrlController.text}',
              style: TextStyle(fontSize: 12, color: Colors.grey[600]),
            ),
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
              _navigateToDashboard();
            },
            child: Text('Go to Dashboard'),
          ),
        ],
      ),
    );
  }

  Widget _buildStatusRow(String label, bool status) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          Icon(
            status ? Icons.check_circle : Icons.error,
            color: status ? Colors.green : Colors.red,
            size: 20,
          ),
          SizedBox(width: 8),
          Text(label),
          Spacer(),
          Text(
            status ? 'Connected' : 'Disconnected',
            style: TextStyle(
              color: status ? Colors.green : Colors.red,
              fontWeight: FontWeight.w500,
            ),
          ),
        ],
      ),
    );
  }

  void _connectToServer() async {
    try {
      final success =
          await _webSocketService.connect(_serverUrlController.text);
      if (success) {
        setState(() {
          _isServerConnected = true;
        });
        _showSuccessSnackBar('Connected to server');
        _checkAndShowSuccessPopup();
      } else {
        _showErrorSnackBar('Failed to connect to server');
      }
    } catch (e) {
      _showErrorSnackBar('Connection error: $e');
    }
  }

  void _disconnectFromServer() async {
    await _webSocketService.disconnect();
    setState(() {
      _isServerConnected = false;
    });
    _showInfoSnackBar('Disconnected from server');
  }

  void _scanForDevices() async {
    setState(() {
      _isScanning = true;
      _availableDevices.clear();
    });

    // Simulate device discovery
    await Future.delayed(Duration(seconds: 2));

    // Mock discovered devices
    setState(() {
      _availableDevices = [
        {'name': 'piros', 'ip': '192.168.253.136', 'id': 'piros'},
        {'name': 'AGV-001', 'ip': '192.168.1.101', 'id': 'agv_001'},
        {'name': 'AGV-002', 'ip': '192.168.1.102', 'id': 'agv_002'},
      ];
      _isScanning = false;
    });
  }

  void _connectDevice() async {
    if (!_formKey.currentState!.validate()) return;

    setState(() {
      _isConnecting = true;
    });

    try {
      await _apiService.connectDevice(
        deviceId: _deviceIdController.text,
        deviceName: _deviceNameController.text,
        ipAddress: _ipAddressController.text,
      );

      setState(() {
        _isDeviceConnected = true;
      });
      _showSuccessSnackBar('Device connected successfully');
      _loadConnectedDevices();

      // Clear form
      _deviceIdController.clear();
      _deviceNameController.clear();
      _ipAddressController.clear();

      _checkAndShowSuccessPopup();
    } catch (e) {
      _showErrorSnackBar('Failed to connect device: $e');
    } finally {
      setState(() {
        _isConnecting = false;
      });
    }
  }

  void _connectToDiscoveredDevice(Map<String, dynamic> device) async {
    try {
      await _apiService.connectDevice(
        deviceId: device['id'],
        deviceName: device['name'],
        ipAddress: device['ip'],
      );

      _showSuccessSnackBar('Device ${device['name']} connected');
      _loadConnectedDevices();

      setState(() {
        _availableDevices.remove(device);
        _isDeviceConnected = true;
      });

      _checkAndShowSuccessPopup();
    } catch (e) {
      _showErrorSnackBar('Failed to connect to ${device['name']}: $e');
    }
  }

  void _connectSpecificDevice(
      String deviceId, String deviceName, String ip) async {
    if (deviceId.isEmpty || deviceName.isEmpty || ip.isEmpty) return;

    try {
      await _apiService.connectDevice(
        deviceId: deviceId,
        deviceName: deviceName,
        ipAddress: ip,
      );

      _showSuccessSnackBar('Device $deviceName connected');
      _loadConnectedDevices();
      setState(() {
        _isDeviceConnected = true;
      });

      _checkAndShowSuccessPopup();
    } catch (e) {
      _showErrorSnackBar('Failed to connect device: $e');
    }
  }

  void _handleDeviceAction(Map<String, dynamic> device, String action) async {
    switch (action) {
      case 'control':
        _navigateToControl(device);
        break;
      case 'disconnect':
        _disconnectDevice(device);
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

  void _navigateToDashboard() {
    Navigator.pushReplacementNamed(context, '/dashboard');
  }

  void _disconnectDevice(Map<String, dynamic> device) async {
    try {
      await _apiService.disconnectDevice(device['id']);
      _showInfoSnackBar('Device ${device['name']} disconnected');
      _loadConnectedDevices();
    } catch (e) {
      _showErrorSnackBar('Failed to disconnect device: $e');
    }
  }

  // Check if both server and device are connected, then show success popup
  void _checkAndShowSuccessPopup() {
    if (_isServerConnected && _isDeviceConnected) {
      // Delay slightly to ensure UI updates
      Future.delayed(Duration(milliseconds: 500), () {
        _showConnectionSuccessPopup();
      });
    }
  }

  void _showSuccessSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.green,
      ),
    );
  }

  void _showErrorSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.red,
      ),
    );
  }

  void _showInfoSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.blue,
      ),
    );
  }
}