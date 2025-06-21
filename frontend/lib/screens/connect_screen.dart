// screens/connect_screen.dart - Simple device connection management screen
import 'package:flutter/material.dart';
import 'dart:async';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';

class ConnectScreen extends StatefulWidget {
  @override
  _ConnectScreenState createState() => _ConnectScreenState();
}

class _ConnectScreenState extends State<ConnectScreen> {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();
  final _deviceIdController = TextEditingController(text: 'agv_01');
  final _deviceNameController = TextEditingController(text: 'Primary AGV');
  final TextEditingController _deviceIpController = TextEditingController();

  List<Map<String, dynamic>> _connectedDevices = [];
  bool _isLoading = false;
  bool _isConnecting = false;
  late StreamSubscription _deviceEventsSubscription;

  @override
  void initState() {
    super.initState();
    _loadConnectedDevices();
    _subscribeToDeviceEvents();
  }

  @override
  void dispose() {
    _deviceIdController.dispose();
    _deviceNameController.dispose();
    _deviceIpController.dispose();
    _deviceEventsSubscription.cancel();
    super.dispose();
  }

  void _subscribeToDeviceEvents() {
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

  void _connectDevice() async {
    final deviceId = _deviceIdController.text.trim();
    final deviceName = _deviceNameController.text.trim();
    final deviceIp = _deviceIpController.text.trim();

    if (deviceId.isEmpty || deviceIp.isEmpty) {
      _showErrorSnackBar('Device ID and IP Address are required');
      return;
    }

    setState(() {
      _isConnecting = true;
    });

    try {
      final result = await _apiService.connectDevice(
        deviceId: deviceId,
        name: deviceName.isNotEmpty ? deviceName : 'AGV $deviceId',
        ipAddress: deviceIp,
        type: 'differential_drive',
        capabilities: ['mapping', 'navigation', 'remote_control'],
      );

      if (result['success'] == true) {
        _showSuccessSnackBar('Device connected successfully');
        _loadConnectedDevices();

        // Clear form
        _deviceIdController.clear();
        _deviceNameController.clear();
        _deviceIpController.clear();

        // Optionally show dashboard dialog
        _showDashboardDialog();
      } else {
        _showErrorSnackBar('Failed to connect device: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Failed to connect device: $e');
    } finally {
      setState(() {
        _isConnecting = false;
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

        // Show dashboard popup
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

  void _connectWebSocket() async {
    try {
      await _webSocketService.connect('ws://192.168.253.79:3000');
      _showSuccessSnackBar('WebSocket connected to ws://192.168.253.79:3000');
    } catch (e) {
      _showErrorSnackBar('WebSocket connection failed: $e');
    }
  }

  // Add this method to show the dashboard dialog
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
        title: Text('Device Management'),
        actions: [
          IconButton(
            icon: Icon(Icons.refresh),
            onPressed: _isLoading ? null : _loadConnectedDevices,
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
                  _buildConnectionCard(),
                  SizedBox(height: 20),
                  _buildConnectedDevicesCard(),
                ],
              ),
            ),
    );
  }

  Widget _buildConnectionCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Connect AGV Device',
              style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),

            // Auto-connect section
            Container(
              padding: EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: Colors.blue[50],
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.blue[200]!),
              ),
              child: Column(
                children: [
                  Row(
                    children: [
                      Icon(Icons.auto_fix_high, color: Colors.blue),
                      SizedBox(width: 8),
                      Text(
                        'Quick Setup',
                        style: TextStyle(
                          fontWeight: FontWeight.bold,
                          color: Colors.blue[700],
                        ),
                      ),
                    ],
                  ),
                  SizedBox(height: 8),
                  Text(
                      'Automatically connect to your AGV with default settings'),
                  SizedBox(height: 12),
                  SizedBox(
                    width: double.infinity,
                    child: ElevatedButton.icon(
                      onPressed: _isConnecting ? null : _autoConnectAGV,
                      icon: _isConnecting
                          ? SizedBox(
                              width: 16,
                              height: 16,
                              child: CircularProgressIndicator(strokeWidth: 2),
                            )
                          : Icon(Icons.smart_toy),
                      label: Text(
                          _isConnecting ? 'Connecting...' : 'Auto-Connect AGV'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.blue,
                        padding: EdgeInsets.symmetric(vertical: 12),
                      ),
                    ),
                  ),
                ],
              ),
            ),

            SizedBox(height: 20),
            Divider(),
            SizedBox(height: 20),

            // Manual connection section
            Text(
              'Manual Connection',
              style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 12),
            TextFormField(
              controller: _deviceIdController,
              decoration: InputDecoration(
                labelText: 'Device ID',
                border: OutlineInputBorder(),
                hintText: 'e.g., agv_01',
              ),
            ),
            SizedBox(height: 12),
            TextFormField(
              controller: _deviceNameController,
              decoration: InputDecoration(
                labelText: 'Device Name (Optional)',
                border: OutlineInputBorder(),
                hintText: 'e.g., Primary AGV',
              ),
            ),
            SizedBox(height: 12),
            TextField(
              controller: _deviceIpController,
              decoration: InputDecoration(
                labelText: 'Device IP Address',
                prefixIcon: Icon(Icons.language),
                hintText: 'e.g. 192.168.253.79',
              ),
              keyboardType: TextInputType.numberWithOptions(decimal: true),
            ),
            SizedBox(height: 16),
            SizedBox(
              width: double.infinity,
              child: ElevatedButton.icon(
                onPressed: _isConnecting ? null : _connectDevice,
                icon: _isConnecting
                    ? SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(strokeWidth: 2),
                      )
                    : Icon(Icons.add),
                label: Text(_isConnecting ? 'Connecting...' : 'Connect Device'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.green,
                  padding: EdgeInsets.symmetric(vertical: 12),
                ),
              ),
            ),
            SizedBox(height: 16),
            SizedBox(
              width: double.infinity,
              child: ElevatedButton.icon(
                onPressed: _connectWebSocket,
                icon: Icon(Icons.wifi),
                label: Text('Connect WebSocket (ws://192.168.253.79:3000)'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.teal,
                  padding: EdgeInsets.symmetric(vertical: 12),
                ),
              ),
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
                    Text('Connect your AGV devices to get started'),
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
          if (device['connectedAt'] != null)
            Text(
              'Connected: ${_formatDateTime(device['connectedAt'])}',
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

  void _showDeviceStatus(Map<String, dynamic> device) async {
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
            if (device['capabilities'] != null) ...[
              SizedBox(height: 8),
              Text('Capabilities:',
                  style: TextStyle(fontWeight: FontWeight.bold)),
              ...((device['capabilities'] as List<dynamic>)
                  .map((cap) => Padding(
                        padding: EdgeInsets.only(left: 16),
                        child: Text('• $cap'),
                      ))),
            ],
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
}
