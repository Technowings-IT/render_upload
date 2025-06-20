//screens/control_page.dart - Updated for Backend Integration
import 'package:flutter/material.dart';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import '../widgets/joystick.dart';
import '../models/odom.dart';
import 'dart:async';

class ControlPage extends StatefulWidget {
  final String deviceId;
  final String deviceName;

  const ControlPage({
    Key? key,
    required this.deviceId,
    required this.deviceName,
  }) : super(key: key);

  @override
  _ControlPageState createState() => _ControlPageState();
}

class _ControlPageState extends State<ControlPage> with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();
  
  late TabController _tabController;
  late StreamSubscription _realTimeSubscription;
  late StreamSubscription _controlEventsSubscription;
  late StreamSubscription _mappingEventsSubscription;
  late StreamSubscription _connectionSubscription;
  
  // Device status
  bool _isConnected = false;
  bool _isWebSocketConnected = false;
  bool _isMappingActive = false;
  bool _isROS2Connected = false;
  OdometryData? _latestOdometry;
  Map<String, dynamic>? _batteryState;
  Map<String, dynamic>? _robotState;
  Map<String, dynamic>? _latestScan;
  List<Offset> _robotTrail = [];
  
  // Control state
  bool _autoMode = false;
  double _currentLinear = 0.0;
  double _currentAngular = 0.0;
  bool _deadmanActive = false;
  
  // Goal setting
  final _goalXController = TextEditingController();
  final _goalYController = TextEditingController();
  final _goalOrientationController = TextEditingController();

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 4, vsync: this);
    _initializeConnections();
    _subscribeToUpdates();
  }

  @override
  void dispose() {
    _tabController.dispose();
    _realTimeSubscription.cancel();
    _controlEventsSubscription.cancel();
    _mappingEventsSubscription.cancel();
    _connectionSubscription.cancel();
    _goalXController.dispose();
    _goalYController.dispose();
    _goalOrientationController.dispose();
    super.dispose();
  }

  void _initializeConnections() async {
    // Print API configuration for debugging
    _apiService.printConnectionInfo();
    
    // Test API connectivity first
    final apiConnected = await _apiService.testConnection();
    if (!apiConnected) {
      _showErrorSnackBar('Failed to connect to API server. Check network settings.');
      return;
    }

    // Check ROS2 status
    try {
      final ros2Status = await _apiService.getROS2Status();
      setState(() {
        _isROS2Connected = ros2Status['connected'] == true;
      });
      
      if (!_isROS2Connected) {
        _showWarningSnackBar('ROS2 not connected. Some features may be limited.');
      }
    } catch (e) {
      print('❌ Error checking ROS2 status: $e');
      _showWarningSnackBar('Could not check ROS2 status.');
    }

    // Check WebSocket connection
    setState(() {
      _isWebSocketConnected = _webSocketService.isConnected;
    });

    // If not connected, try to connect
    if (!_isWebSocketConnected) {
      final wsUrl = _apiService.getWebSocketUrl();
      final connected = await _webSocketService.connect(wsUrl);
      setState(() {
        _isWebSocketConnected = connected;
      });
    }

    // Check if device is connected via API
    try {
      final devices = await _apiService.getDevices();
      final device = devices.firstWhere(
        (d) => d['id'] == widget.deviceId,
        orElse: () => <String, dynamic>{},
      );
      
      setState(() {
        _isConnected = device.isNotEmpty && device['status'] == 'connected';
      });
    } catch (e) {
      print('❌ Error checking device status: $e');
      if (e is ApiException) {
        if (e.isNetworkError) {
          _showErrorSnackBar('Network error: Cannot reach server. Check IP address and port.');
        } else if (e.isServerError) {
          _showErrorSnackBar('Server error: ${e.message}');
        } else {
          _showErrorSnackBar('API error: ${e.message}');
        }
      } else {
        _showErrorSnackBar('Unexpected error: $e');
      }
      setState(() {
        _isConnected = false;
      });
    }
  }

  void _subscribeToUpdates() {
    // Subscribe to real-time data
    _realTimeSubscription = _webSocketService.realTimeData.listen((data) {
      if (data['deviceId'] == widget.deviceId) {
        _handleRealTimeData(data);
      }
    });

    // Subscribe to control events
    _controlEventsSubscription = _webSocketService.controlEvents.listen((data) {
      if (data['deviceId'] == widget.deviceId) {
        _handleControlEvent(data);
      }
    });

    // Subscribe to mapping events
    _mappingEventsSubscription = _webSocketService.mappingEvents.listen((data) {
      if (data['deviceId'] == widget.deviceId) {
        _handleMappingEvent(data);
      }
    });

    // Subscribe to connection state changes
    _connectionSubscription = _webSocketService.connectionState.listen((connected) {
      setState(() {
        _isWebSocketConnected = connected;
      });
    });

    // Subscribe to device-specific real-time data
    if (_isWebSocketConnected) {
      _webSocketService.subscribe('real_time_data', deviceId: widget.deviceId);
      _webSocketService.subscribe('control_events', deviceId: widget.deviceId);
      _webSocketService.subscribe('mapping_events', deviceId: widget.deviceId);
    }
  }

  void _handleRealTimeData(Map<String, dynamic> data) {
    switch (data['type']) {
      case 'odometry_update':
        setState(() {
          _latestOdometry = OdometryData.fromJson(data['data']);
          
          // Update robot trail
          if (_latestOdometry != null) {
            _robotTrail.add(Offset(_latestOdometry!.position.x, _latestOdometry!.position.y));
            if (_robotTrail.length > 100) {
              _robotTrail.removeAt(0);
            }
          }
        });
        break;
        
      case 'battery_update':
        setState(() {
          _batteryState = data['data'];
        });
        break;
        
      case 'laser_scan':
        setState(() {
          _latestScan = data['data'];
        });
        break;
        
      case 'joint_states':
        // Handle joint states if needed
        break;
        
      case 'velocity_feedback':
        // Handle velocity feedback
        break;
    }
  }

  void _handleControlEvent(Map<String, dynamic> data) {
    switch (data['type']) {
      case 'movement_command':
        // Visual feedback for movement commands
        break;
        
      case 'stop_command':
        setState(() {
          _currentLinear = 0.0;
          _currentAngular = 0.0;
          _deadmanActive = false;
        });
        break;
        
      case 'goal_command':
        _showInfoSnackBar('Navigation goal set: (${data['goal']['x']}, ${data['goal']['y']})');
        break;
    }
  }

  void _handleMappingEvent(Map<String, dynamic> data) {
    switch (data['type']) {
      case 'mapping_started':
        setState(() {
          _isMappingActive = true;
        });
        _showInfoSnackBar('Mapping started');
        break;
        
      case 'mapping_stopped':
        setState(() {
          _isMappingActive = false;
        });
        _showInfoSnackBar('Mapping stopped');
        break;
        
      case 'map_saved':
        _showInfoSnackBar('Map saved: ${data['mapName']}');
        break;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Control: ${widget.deviceName}'),
        backgroundColor: _getConnectionColor(),
        actions: [
          // ROS2 status indicator
          Container(
            margin: EdgeInsets.only(right: 8),
            child: Center(
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Container(
                    width: 8,
                    height: 8,
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: _isROS2Connected ? Colors.green : Colors.orange,
                    ),
                  ),
                  SizedBox(width: 4),
                  Text('ROS2', style: TextStyle(fontSize: 10)),
                ],
              ),
            ),
          ),
          // WebSocket connection indicator
          Container(
            margin: EdgeInsets.only(right: 8),
            child: Center(
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Container(
                    width: 8,
                    height: 8,
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: _isWebSocketConnected ? Colors.green : Colors.red,
                    ),
                  ),
                  SizedBox(width: 4),
                  Text('WS', style: TextStyle(fontSize: 10)),
                ],
              ),
            ),
          ),
          IconButton(
            icon: Icon(_isConnected ? Icons.wifi : Icons.wifi_off),
            onPressed: _toggleConnection,
            tooltip: _isConnected ? 'Disconnect' : 'Connect',
          ),
          IconButton(
            icon: Icon(Icons.emergency),
            onPressed: _emergencyStop,
            color: Colors.white,
            tooltip: 'Emergency Stop',
          ),
          PopupMenuButton(
            icon: Icon(Icons.more_vert),
            itemBuilder: (context) => [
              PopupMenuItem(
                child: ListTile(
                  leading: Icon(Icons.build),
                  title: Text('Test ROS2'),
                  dense: true,
                ),
                onTap: _testROS2Connection,
              ),
              PopupMenuItem(
                child: ListTile(
                  leading: Icon(Icons.refresh),
                  title: Text('Refresh Status'),
                  dense: true,
                ),
                onTap: _refreshConnectionStatus,
              ),
              PopupMenuItem(
                child: ListTile(
                  leading: Icon(Icons.settings),
                  title: Text('Connection Info'),
                  dense: true,
                ),
                onTap: _showConnectionInfo,
              ),
            ],
          ),
        ],
        bottom: TabBar(
          controller: _tabController,
          tabs: [
            Tab(icon: Icon(Icons.gamepad), text: 'Manual'),
            Tab(icon: Icon(Icons.map), text: 'Mapping'),
            Tab(icon: Icon(Icons.navigation), text: 'Auto'),
            Tab(icon: Icon(Icons.info), text: 'Status'),
          ],
        ),
      ),
      body: _isConnected 
          ? TabBarView(
              controller: _tabController,
              children: [
                _buildManualControlTab(),
                _buildMappingTab(),
                _buildAutoNavigationTab(),
                _buildStatusTab(),
              ],
            )
          : _buildDisconnectedView(),
    );
  }

  Color _getConnectionColor() {
    if (!_isConnected) return Colors.red;
    if (!_isROS2Connected) return Colors.orange;
    return Colors.green;
  }

  Widget _buildDisconnectedView() {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(
            Icons.wifi_off,
            size: 80,
            color: Colors.red,
          ),
          SizedBox(height: 16),
          Text(
            'Device Disconnected',
            style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
          ),
          SizedBox(height: 8),
          Text('Connect to the device to start controlling'),
          SizedBox(height: 16),
          if (!_isWebSocketConnected) ...[
            Container(
              padding: EdgeInsets.all(16),
              margin: EdgeInsets.symmetric(horizontal: 32),
              decoration: BoxDecoration(
                color: Colors.red[50],
                border: Border.all(color: Colors.red),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Column(
                children: [
                  Row(
                    children: [
                      Icon(Icons.error, color: Colors.red),
                      SizedBox(width: 8),
                      Text(
                        'WebSocket Disconnected',
                        style: TextStyle(color: Colors.red, fontWeight: FontWeight.bold),
                      ),
                    ],
                  ),
                  SizedBox(height: 8),
                  Text('Real-time features are unavailable'),
                  SizedBox(height: 8),
                  ElevatedButton(
                    onPressed: _reconnectWebSocket,
                    child: Text('Reconnect WebSocket'),
                  ),
                ],
              ),
            ),
            SizedBox(height: 16),
          ],
          if (!_isROS2Connected) ...[
            Container(
              padding: EdgeInsets.all(16),
              margin: EdgeInsets.symmetric(horizontal: 32),
              decoration: BoxDecoration(
                color: Colors.orange[50],
                border: Border.all(color: Colors.orange),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Column(
                children: [
                  Row(
                    children: [
                      Icon(Icons.warning, color: Colors.orange),
                      SizedBox(width: 8),
                      Text(
                        'ROS2 Not Connected',
                        style: TextStyle(color: Colors.orange, fontWeight: FontWeight.bold),
                      ),
                    ],
                  ),
                  SizedBox(height: 8),
                  Text('Robot control functions may be limited'),
                  SizedBox(height: 8),
                  ElevatedButton(
                    onPressed: _testROS2Connection,
                    child: Text('Test ROS2'),
                  ),
                ],
              ),
            ),
            SizedBox(height: 16),
          ],
          ElevatedButton.icon(
            onPressed: _connectDevice,
            icon: Icon(Icons.wifi),
            label: Text('Connect Device'),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.green,
              padding: EdgeInsets.symmetric(horizontal: 32, vertical: 16),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildManualControlTab() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(16),
      child: Column(
        children: [
          // Current velocity display
          _buildVelocityCard(),
          SizedBox(height: 16),
          
          // Real-time position display
          if (_latestOdometry != null) _buildPositionCard(),
          if (_latestOdometry != null) SizedBox(height: 16),
          
          // Joystick control
          Card(
            child: Padding(
              padding: EdgeInsets.all(16),
              child: Column(
                children: [
                  Text(
                    'Manual Control',
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                  SizedBox(height: 16),
                  JoystickControlPage(
                    deviceId: widget.deviceId,
                    onVelocityChanged: _onVelocityChanged,
                  ),
                ],
              ),
            ),
          ),
          
          SizedBox(height: 16),
          
          // Quick action buttons
          _buildQuickActions(),
        ],
      ),
    );
  }

  Widget _buildMappingTab() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(16),
      child: Column(
        children: [
          // Mapping status
          Card(
            child: Padding(
              padding: EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(
                    children: [
                      Icon(
                        _isMappingActive ? Icons.radio_button_checked : Icons.radio_button_unchecked,
                        color: _isMappingActive ? Colors.green : Colors.grey,
                      ),
                      SizedBox(width: 8),
                      Text(
                        'Mapping Status: ${_isMappingActive ? "Active" : "Inactive"}',
                        style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                      ),
                    ],
                  ),
                  SizedBox(height: 16),
                  Row(
                    children: [
                      Expanded(
                        child: ElevatedButton.icon(
                          onPressed: _isMappingActive ? null : _startMapping,
                          icon: Icon(Icons.play_arrow),
                          label: Text('Start Mapping'),
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.green,
                          ),
                        ),
                      ),
                      SizedBox(width: 8),
                      Expanded(
                        child: ElevatedButton.icon(
                          onPressed: _isMappingActive ? _stopMapping : null,
                          icon: Icon(Icons.stop),
                          label: Text('Stop Mapping'),
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.red,
                          ),
                        ),
                      ),
                    ],
                  ),
                  SizedBox(height: 8),
                  SizedBox(
                    width: double.infinity,
                    child: ElevatedButton.icon(
                      onPressed: _saveCurrentMap,
                      icon: Icon(Icons.save),
                      label: Text('Save Map'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.blue,
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
          
          SizedBox(height: 16),
          
          // Mapping control with joystick
          Card(
            child: Padding(
              padding: EdgeInsets.all(16),
              child: Column(
                children: [
                  Text(
                    'Drive for Mapping',
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                  SizedBox(height: 8),
                  Text(
                    'Use the joystick to drive the AGV around the area to create a map',
                    textAlign: TextAlign.center,
                    style: TextStyle(color: Colors.grey[600]),
                  ),
                  SizedBox(height: 16),
                  JoystickControlPage(
                    deviceId: widget.deviceId,
                    onVelocityChanged: _onVelocityChanged,
                  ),
                ],
              ),
            ),
          ),
          
          SizedBox(height: 16),
          
          // Mapping tips
          _buildMappingTips(),
          
          SizedBox(height: 16),
          
          // Live scan data visualization (if available)
          if (_latestScan != null) _buildScanVisualization(),
        ],
      ),
    );
  }

  Widget _buildAutoNavigationTab() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(16),
      child: Column(
        children: [
          // Auto mode toggle
          Card(
            child: Padding(
              padding: EdgeInsets.all(16),
              child: Row(
                children: [
                  Icon(
                    _autoMode ? Icons.smart_toy : Icons.person,
                    color: _autoMode ? Colors.blue : Colors.grey,
                  ),
                  SizedBox(width: 8),
                  Text(
                    'Mode: ${_autoMode ? "Autonomous" : "Manual"}',
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                  Spacer(),
                  Switch(
                    value: _autoMode,
                    onChanged: _toggleAutoMode,
                  ),
                ],
              ),
            ),
          ),
          
          SizedBox(height: 16),
          
          if (_autoMode) ...[
            // Goal setting
            Card(
              child: Padding(
                padding: EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Set Navigation Goal',
                      style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                    ),
                    SizedBox(height: 16),
                    Row(
                      children: [
                        Expanded(
                          child: TextFormField(
                            controller: _goalXController,
                            decoration: InputDecoration(
                              labelText: 'X Position (m)',
                              border: OutlineInputBorder(),
                            ),
                            keyboardType: TextInputType.numberWithOptions(decimal: true),
                          ),
                        ),
                        SizedBox(width: 8),
                        Expanded(
                          child: TextFormField(
                            controller: _goalYController,
                            decoration: InputDecoration(
                              labelText: 'Y Position (m)',
                              border: OutlineInputBorder(),
                            ),
                            keyboardType: TextInputType.numberWithOptions(decimal: true),
                          ),
                        ),
                        SizedBox(width: 8),
                        Expanded(
                          child: TextFormField(
                            controller: _goalOrientationController,
                            decoration: InputDecoration(
                              labelText: 'Orientation (°)',
                              border: OutlineInputBorder(),
                            ),
                            keyboardType: TextInputType.numberWithOptions(decimal: true),
                          ),
                        ),
                      ],
                    ),
                    SizedBox(height: 16),
                    Row(
                      children: [
                        Expanded(
                          child: ElevatedButton.icon(
                            onPressed: _sendGoal,
                            icon: Icon(Icons.navigation),
                            label: Text('Send Goal'),
                            style: ElevatedButton.styleFrom(
                              backgroundColor: Colors.blue,
                            ),
                          ),
                        ),
                        SizedBox(width: 8),
                        ElevatedButton.icon(
                          onPressed: _setCurrentAsInitialPose,
                          icon: Icon(Icons.my_location),
                          label: Text('Set Current Pose'),
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.orange,
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
              ),
            ),
            
            SizedBox(height: 16),
            
            // Quick goal buttons
            _buildQuickGoals(),
          ] else
            Card(
              child: Padding(
                padding: EdgeInsets.all(16),
                child: Column(
                  children: [
                    Icon(Icons.info, size: 48, color: Colors.blue),
                    SizedBox(height: 16),
                    Text(
                      'Auto Navigation Disabled',
                      style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                    ),
                    SizedBox(height: 8),
                    Text(
                      'Enable auto mode to use navigation features',
                      textAlign: TextAlign.center,
                    ),
                  ],
                ),
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildStatusTab() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(16),
      child: Column(
        children: [
          // Device info
          _buildDeviceInfoCard(),
          SizedBox(height: 16),
          
          // Connection status
          _buildConnectionCard(),
          SizedBox(height: 16),
          
          // Odometry data
          _buildOdometryCard(),
          SizedBox(height: 16),
          
          // Battery status
          if (_batteryState != null) _buildBatteryCard(),
          if (_batteryState != null) SizedBox(height: 16),
          
          // Robot state
          if (_robotState != null) _buildRobotStateCard(),
        ],
      ),
    );
  }

  Widget _buildVelocityCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          children: [
            Row(
              children: [
                Text(
                  'Current Velocity',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                Spacer(),
                Container(
                  padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                  decoration: BoxDecoration(
                    color: _deadmanActive ? Colors.green : Colors.grey,
                    borderRadius: BorderRadius.circular(12),
                  ),
                  child: Text(
                    _deadmanActive ? 'ACTIVE' : 'SAFE',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 12,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                ),
              ],
            ),
            SizedBox(height: 16),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                Column(
                  children: [
                    Icon(Icons.arrow_upward, 
                         color: _currentLinear > 0 ? Colors.green : 
                               _currentLinear < 0 ? Colors.red : Colors.grey),
                    Text('Linear'),
                    Text('${_currentLinear.toStringAsFixed(2)} m/s',
                         style: TextStyle(fontWeight: FontWeight.bold)),
                  ],
                ),
                Column(
                  children: [
                    Icon(Icons.rotate_right,
                         color: _currentAngular > 0 ? Colors.green :
                               _currentAngular < 0 ? Colors.red : Colors.grey),
                    Text('Angular'),
                    Text('${_currentAngular.toStringAsFixed(2)} rad/s',
                         style: TextStyle(fontWeight: FontWeight.bold)),
                  ],
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildPositionCard() {
    if (_latestOdometry == null) return SizedBox.shrink();
    
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Current Position',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 12),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                Column(
                  children: [
                    Text('X'),
                    Text('${_latestOdometry!.position.x.toStringAsFixed(2)} m',
                         style: TextStyle(fontWeight: FontWeight.bold)),
                  ],
                ),
                Column(
                  children: [
                    Text('Y'),
                    Text('${_latestOdometry!.position.y.toStringAsFixed(2)} m',
                         style: TextStyle(fontWeight: FontWeight.bold)),
                  ],
                ),
                Column(
                  children: [
                    Text('Angle'),
                    Text('${_latestOdometry!.orientation.yawDegrees.toStringAsFixed(1)}°',
                         style: TextStyle(fontWeight: FontWeight.bold)),
                  ],
                ),
              ],
            ),
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
            Row(
              children: [
                Text(
                  'Connection Status',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                Spacer(),
                TextButton(
                  onPressed: _refreshConnectionStatus,
                  child: Text('Refresh'),
                ),
              ],
            ),
            SizedBox(height: 16),
            _buildInfoRow('Device Connected', _isConnected ? 'Yes' : 'No', 
                         color: _isConnected ? Colors.green : Colors.red),
            _buildInfoRow('WebSocket Connected', _isWebSocketConnected ? 'Yes' : 'No',
                         color: _isWebSocketConnected ? Colors.green : Colors.red),
            _buildInfoRow('ROS2 Connected', _isROS2Connected ? 'Yes' : 'No',
                         color: _isROS2Connected ? Colors.green : Colors.orange),
            _buildInfoRow('Real-time Data', _latestOdometry != null ? 'Receiving' : 'No Data',
                         color: _latestOdometry != null ? Colors.green : Colors.orange),
            SizedBox(height: 8),
            Text('Server: ${_apiService.baseUrl}', style: TextStyle(fontSize: 12, color: Colors.grey)),
          ],
        ),
      ),
    );
  }

  Widget _buildQuickActions() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Quick Actions',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            GridView.count(
              crossAxisCount: 2,
              shrinkWrap: true,
              physics: NeverScrollableScrollPhysics(),
              crossAxisSpacing: 8,
              mainAxisSpacing: 8,
              childAspectRatio: 2.5,
              children: [
                ElevatedButton.icon(
                  onPressed: () => _moveForward(),
                  icon: Icon(Icons.keyboard_arrow_up),
                  label: Text('Forward'),
                ),
                ElevatedButton.icon(
                  onPressed: () => _moveBackward(),
                  icon: Icon(Icons.keyboard_arrow_down),
                  label: Text('Backward'),
                ),
                ElevatedButton.icon(
                  onPressed: () => _turnLeft(),
                  icon: Icon(Icons.keyboard_arrow_left),
                  label: Text('Turn Left'),
                ),
                ElevatedButton.icon(
                  onPressed: () => _turnRight(),
                  icon: Icon(Icons.keyboard_arrow_right),
                  label: Text('Turn Right'),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildQuickGoals() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Quick Goals',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            GridView.count(
              crossAxisCount: 2,
              shrinkWrap: true,
              physics: NeverScrollableScrollPhysics(),
              crossAxisSpacing: 8,
              mainAxisSpacing: 8,
              childAspectRatio: 2.5,
              children: [
                ElevatedButton.icon(
                  onPressed: () => _sendQuickGoal(0, 0, 0),
                  icon: Icon(Icons.home),
                  label: Text('Origin'),
                ),
                ElevatedButton.icon(
                  onPressed: () => _sendQuickGoal(1, 0, 0),
                  icon: Icon(Icons.arrow_forward),
                  label: Text('1m Forward'),
                ),
                ElevatedButton.icon(
                  onPressed: () => _sendQuickGoal(-1, 0, 180),
                  icon: Icon(Icons.arrow_back),
                  label: Text('1m Back'),
                ),
                ElevatedButton.icon(
                  onPressed: () => _sendQuickGoal(0, 0, 90),
                  icon: Icon(Icons.rotate_90_degrees_ccw),
                  label: Text('Turn 90°'),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildScanVisualization() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Live LIDAR Scan',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            Text('Obstacles detected: ${_latestScan!['obstacles']?.length ?? 0}'),
            Text('Features detected: ${_latestScan!['features']?.length ?? 0}'),
            // Could add a mini visualization here
          ],
        ),
      ),
    );
  }

  // Helper widgets
  Widget _buildMappingTips() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Mapping Tips',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            ...[
              '• Drive slowly and steadily for best results',
              '• Cover all areas you want to map',
              '• Avoid rapid turns or sudden movements',
              '• Ensure good lighting in the environment',
              '• Keep obstacles clear from the path',
              '• Use the deadman switch for safety',
            ].map((tip) => Padding(
              padding: EdgeInsets.symmetric(vertical: 2),
              child: Text(tip),
            )),
          ],
        ),
      ),
    );
  }

  Widget _buildDeviceInfoCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Device Information',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            _buildInfoRow('Device ID', widget.deviceId),
            _buildInfoRow('Device Name', widget.deviceName),
            _buildInfoRow('Mapping Status', _isMappingActive ? 'Active' : 'Inactive'),
            _buildInfoRow('Auto Mode', _autoMode ? 'Enabled' : 'Disabled'),
          ],
        ),
      ),
    );
  }

  Widget _buildOdometryCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Odometry Data',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            if (_latestOdometry != null) ...[
              _buildInfoRow('X Position', '${_latestOdometry!.position.x.toStringAsFixed(3)} m'),
              _buildInfoRow('Y Position', '${_latestOdometry!.position.y.toStringAsFixed(3)} m'),
              _buildInfoRow('Orientation', '${_latestOdometry!.orientation.yawDegrees.toStringAsFixed(1)}°'),
              _buildInfoRow('Linear Velocity', '${_latestOdometry!.linearVelocity.x.toStringAsFixed(3)} m/s'),
              _buildInfoRow('Angular Velocity', '${_latestOdometry!.angularVelocity.z.toStringAsFixed(3)} rad/s'),
              _buildInfoRow('Trail Points', '${_robotTrail.length}'),
            ] else
              Text('No odometry data available'),
          ],
        ),
      ),
    );
  }

  Widget _buildBatteryCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Battery Status',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            if (_batteryState != null) ...[
              Row(
                children: [
                  Expanded(
                    child: LinearProgressIndicator(
                      value: (_batteryState!['percentage'] ?? 0) / 100.0,
                      backgroundColor: Colors.grey[300],
                      valueColor: AlwaysStoppedAnimation<Color>(
                        (_batteryState!['percentage'] ?? 0) > 30 ? Colors.green : Colors.red,
                      ),
                    ),
                  ),
                  SizedBox(width: 8),
                  Text('${_batteryState!['percentage']?.toStringAsFixed(1) ?? '0'}%'),
                ],
              ),
              SizedBox(height: 8),
              _buildInfoRow('Voltage', '${_batteryState!['voltage']?.toStringAsFixed(2) ?? '0'} V'),
              _buildInfoRow('Current', '${_batteryState!['current']?.toStringAsFixed(2) ?? '0'} A'),
            ] else
              Text('No battery data available'),
          ],
        ),
      ),
    );
  }

  Widget _buildRobotStateCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Robot State',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            if (_robotState != null) ...[
              _buildInfoRow('Mode', _robotState!['mode']?.toString().toUpperCase() ?? 'Unknown'),
              _buildInfoRow('Status', _robotState!['status']?.toString().toUpperCase() ?? 'Unknown'),
              if (_robotState!['errors']?.isNotEmpty == true)
                _buildInfoRow('Errors', _robotState!['errors'].join(', ')),
            ] else
              Text('No robot state data available'),
          ],
        ),
      ),
    );
  }

  Widget _buildInfoRow(String label, String value, {Color? color}) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 4),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          SizedBox(
            width: 120,
            child: Text(
              '$label:',
              style: TextStyle(fontWeight: FontWeight.w500),
            ),
          ),
          Expanded(
            child: Text(
              value,
              style: TextStyle(color: color),
            ),
          ),
        ],
      ),
    );
  }

  // Control methods
  void _onVelocityChanged(double linear, double angular, bool deadmanActive) {
    setState(() {
      _currentLinear = linear;
      _currentAngular = angular;
      _deadmanActive = deadmanActive;
    });
  }

  void _emergencyStop() async {
    try {
      await _apiService.stopDevice(widget.deviceId);
      _webSocketService.stopRobot(widget.deviceId);
      
      setState(() {
        _currentLinear = 0.0;
        _currentAngular = 0.0;
        _deadmanActive = false;
      });
      
      _showInfoSnackBar('Emergency stop activated');
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to stop device: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to stop device: $e');
      }
    }
  }

  void _toggleConnection() async {
    if (_isConnected) {
      _disconnectDevice();
    } else {
      _connectDevice();
    }
  }

  void _connectDevice() async {
    try {
      await _apiService.connectDevice(
        deviceId: widget.deviceId,
        deviceName: widget.deviceName,
        ipAddress: '192.168.253.79', // Your AGV IP
      );
      setState(() {
        _isConnected = true;
      });
      _showInfoSnackBar('Device connected successfully');
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to connect: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to connect: $e');
      }
    }
  }

  void _disconnectDevice() async {
    try {
      await _apiService.disconnectDevice(widget.deviceId);
      setState(() {
        _isConnected = false;
        _latestOdometry = null;
        _batteryState = null;
        _robotState = null;
      });
      _showInfoSnackBar('Device disconnected');
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to disconnect: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to disconnect: $e');
      }
    }
  }

  void _reconnectWebSocket() async {
    final wsUrl = _apiService.getWebSocketUrl();
    final connected = await _webSocketService.connect(wsUrl);
    setState(() {
      _isWebSocketConnected = connected;
    });
    
    if (connected) {
      _showInfoSnackBar('WebSocket reconnected');
      // Re-subscribe to channels
      _webSocketService.subscribe('real_time_data', deviceId: widget.deviceId);
      _webSocketService.subscribe('control_events', deviceId: widget.deviceId);
      _webSocketService.subscribe('mapping_events', deviceId: widget.deviceId);
    } else {
      _showErrorSnackBar('Failed to reconnect WebSocket');
    }
  }

  void _testROS2Connection() async {
    try {
      _showInfoSnackBar('Testing ROS2 connection...');
      final result = await _apiService.testROS2Connectivity();
      
      setState(() {
        _isROS2Connected = result['connected'] == true;
      });
      
      if (_isROS2Connected) {
        _showInfoSnackBar('ROS2 connection test successful');
      } else {
        _showWarningSnackBar('ROS2 connection test failed: ${result['message']}');
      }
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('ROS2 test failed: ${e.message}');
      } else {
        _showErrorSnackBar('ROS2 test failed: $e');
      }
    }
  }

  void _refreshConnectionStatus() async {
    try {
      _showInfoSnackBar('Refreshing connection status...');
      
      // Check ROS2
      final ros2Status = await _apiService.getROS2Status();
      setState(() {
        _isROS2Connected = ros2Status['connected'] == true;
      });
      
      // Check device connection
      final devices = await _apiService.getDevices();
      final device = devices.firstWhere(
        (d) => d['id'] == widget.deviceId,
        orElse: () => <String, dynamic>{},
      );
      
      setState(() {
        _isConnected = device.isNotEmpty && device['status'] == 'connected';
      });
      
      _showInfoSnackBar('Status refreshed');
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to refresh status: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to refresh status: $e');
      }
    }
  }

  void _showConnectionInfo() async {
    try {
      final info = await _apiService.getConnectionInfo();
      
      showDialog(
        context: context,
        builder: (context) => AlertDialog(
          title: Text('Connection Information'),
          content: SingleChildScrollView(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisSize: MainAxisSize.min,
              children: [
                _buildInfoRow('Status', info['connected'] ? 'Connected' : 'Disconnected'),
                _buildInfoRow('Base URL', info['baseUrl']),
                _buildInfoRow('API URL', info['apiBaseUrl']),
                _buildInfoRow('WebSocket URL', info['websocketUrl']),
                if (info['serverInfo'] != null) ...[
                  SizedBox(height: 16),
                  Text('Server Info:', style: TextStyle(fontWeight: FontWeight.bold)),
                  _buildInfoRow('Server Status', info['serverInfo']['data']['status']),
                  _buildInfoRow('ROS2 Status', info['serverInfo']['data']['ros2Status']),
                  _buildInfoRow('Connected Devices', info['serverInfo']['data']['connectedDevices'].toString()),
                ],
                if (info['error'] != null) ...[
                  SizedBox(height: 16),
                  Text('Error:', style: TextStyle(fontWeight: FontWeight.bold, color: Colors.red)),
                  Text(info['error'], style: TextStyle(color: Colors.red)),
                ],
              ],
            ),
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: Text('Close'),
            ),
          ],
        ),
      );
    } catch (e) {
      _showErrorSnackBar('Failed to get connection info: $e');
    }
  }

  void _startMapping() async {
    try {
      // Use the corrected map management API
      await _apiService.saveMapData(
        deviceId: widget.deviceId,
        mapData: {
          'action': 'start_mapping',
          'timestamp': DateTime.now().toIso8601String(),
        },
        mapName: 'mapping_session_${DateTime.now().millisecondsSinceEpoch}',
      );
      // The mapping event will be received via WebSocket
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to start mapping: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to start mapping: $e');
      }
    }
  }

  void _stopMapping() async {
    try {
      await _apiService.saveMapData(
        deviceId: widget.deviceId,
        mapData: {
          'action': 'stop_mapping',
          'timestamp': DateTime.now().toIso8601String(),
        },
      );
      // The mapping event will be received via WebSocket
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to stop mapping: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to stop mapping: $e');
      }
    }
  }

  void _saveCurrentMap() async {
    try {
      final mapName = 'map_${DateTime.now().millisecondsSinceEpoch}';
      await _apiService.saveMapData(
        deviceId: widget.deviceId, 
        mapData: {
          'action': 'save_map',
          'timestamp': DateTime.now().toIso8601String(),
        },
        mapName: mapName,
      );
      // The save event will be received via WebSocket
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to save map: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to save map: $e');
      }
    }
  }

  void _toggleAutoMode(bool value) {
    setState(() {
      _autoMode = value;
    });
    
    if (!_autoMode) {
      _emergencyStop(); // Stop when switching to manual
    }
  }

  void _sendGoal() async {
    try {
      final x = double.tryParse(_goalXController.text) ?? 0.0;
      final y = double.tryParse(_goalYController.text) ?? 0.0;
      final orientation = (double.tryParse(_goalOrientationController.text) ?? 0.0) * (3.14159 / 180); // Convert to radians

      // Note: setDeviceGoal method would need to be implemented in the API service
      // For now, using the control/move endpoint as a placeholder
      await _apiService.moveDevice(
        deviceId: widget.deviceId,
        linear: 0.0, // Navigation goals don't use direct velocity
        angular: 0.0,
      );
      
      _showInfoSnackBar('Navigation goal sent: ($x, $y, ${orientation}rad)');
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to send goal: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to send goal: $e');
      }
    }
  }

  void _sendQuickGoal(double x, double y, double orientationDegrees) async {
    try {
      final orientation = orientationDegrees * (3.14159 / 180); // Convert to radians
      
      // Placeholder implementation - would need proper navigation goal API
      await _apiService.moveDevice(
        deviceId: widget.deviceId,
        linear: 0.0,
        angular: 0.0,
      );
      
      _showInfoSnackBar('Quick goal sent: ($x, $y)');
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to send quick goal: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to send quick goal: $e');
      }
    }
  }

  void _setCurrentAsInitialPose() async {
    if (_latestOdometry == null) {
      _showErrorSnackBar('No current position available');
      return;
    }

    try {
      // Placeholder implementation - would need proper initial pose API
      await _apiService.moveDevice(
        deviceId: widget.deviceId,
        linear: 0.0,
        angular: 0.0,
      );
      
      _showInfoSnackBar('Initial pose set to current position');
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to set initial pose: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to set initial pose: $e');
      }
    }
  }

  // Quick movement methods using the corrected API
  void _moveForward() async {
    try {
      await _apiService.moveDevice(
        deviceId: widget.deviceId,
        linear: 0.5,
        angular: 0.0,
      );
      
      // Stop after 500ms
      Future.delayed(Duration(milliseconds: 500), () async {
        await _apiService.stopDevice(widget.deviceId);
      });
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Move forward failed: ${e.message}');
      } else {
        _showErrorSnackBar('Move forward failed: $e');
      }
    }
  }

  void _moveBackward() async {
    try {
      await _apiService.moveDevice(
        deviceId: widget.deviceId,
        linear: -0.5,
        angular: 0.0,
      );
      
      Future.delayed(Duration(milliseconds: 500), () async {
        await _apiService.stopDevice(widget.deviceId);
      });
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Move backward failed: ${e.message}');
      } else {
        _showErrorSnackBar('Move backward failed: $e');
      }
    }
  }

  void _turnLeft() async {
    try {
      await _apiService.moveDevice(
        deviceId: widget.deviceId,
        linear: 0.0,
        angular: 1.0,
      );
      
      Future.delayed(Duration(milliseconds: 500), () async {
        await _apiService.stopDevice(widget.deviceId);
      });
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Turn left failed: ${e.message}');
      } else {
        _showErrorSnackBar('Turn left failed: $e');
      }
    }
  }

  void _turnRight() async {
    try {
      await _apiService.moveDevice(
        deviceId: widget.deviceId,
        linear: 0.0,
        angular: -1.0,
      );
      
      Future.delayed(Duration(milliseconds: 500), () async {
        await _apiService.stopDevice(widget.deviceId);
      });
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Turn right failed: ${e.message}');
      } else {
        _showErrorSnackBar('Turn right failed: $e');
      }
    }
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

  void _showInfoSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.blue,
        duration: Duration(seconds: 2),
      ),
    );
  }
}