// screens/control_page.dart - Enhanced with live mapping integration
import 'package:flutter/material.dart';
import 'dart:async';
import '../services/web_socket_service.dart';
import '../services/api_service.dart';
import '../widgets/joystick.dart';
import '../widgets/live_maping_canvas.dart';
import '../models/map_data.dart';
import '../models/odom.dart';

class ControlPage extends StatefulWidget {
  final String deviceId;

  const ControlPage({
    Key? key,
    required this.deviceId,
  }) : super(key: key);

  @override
  _ControlPageState createState() => _ControlPageState();
}

class _ControlPageState extends State<ControlPage>
    with TickerProviderStateMixin {
  final WebSocketService _webSocketService = WebSocketService();
  final ApiService _apiService = ApiService();

  // Subscriptions
  late StreamSubscription _realTimeDataSubscription;
  late StreamSubscription _mappingEventsSubscription;
  late StreamSubscription _controlEventsSubscription;
  late StreamSubscription _connectionStateSubscription;

  // Animation controller
  late AnimationController _statusAnimationController;

  // State variables
  bool _isConnected = false;
  bool _mappingActive = false;
  bool _controlEnabled = true;
  MapData? _currentMapData;
  OdometryData? _currentOdometry;
  List<Position> _robotTrail = [];

  // Control settings
  double _maxLinearSpeed = 1.0;
  double _maxAngularSpeed = 2.0;
  bool _useDeadmanSwitch = true;
  bool _showTrail = true;
  bool _autoCenter = true;

  // Status info
  String _connectionStatus = 'Disconnected';
  DateTime? _lastPositionUpdate;
  DateTime? _lastMapUpdate;
  int _messagesReceived = 0;

  @override
  void initState() {
    super.initState();
    _initializeAnimations();

    // Try to connect if not already connected
    if (!_webSocketService.isConnected) {
      _webSocketService.connect('ws://192.168.253.79:3000').then((_) {
        _checkConnection();
        _subscribeToTopics();
      });
    } else {
      _checkConnection();
      _subscribeToTopics();
    }
    _setupSubscriptions();
  }

  void _initializeAnimations() {
    _statusAnimationController = AnimationController(
      duration: Duration(milliseconds: 1000),
      vsync: this,
    );
  }

  void _checkConnection() {
    setState(() {
      _isConnected = _webSocketService.isConnected;
    });
  }

  void _setupSubscriptions() {
    // Connection state changes
    _connectionStateSubscription =
        _webSocketService.connectionState.listen((connected) {
      setState(() {
        _isConnected = connected;
        _connectionStatus = connected ? 'Connected' : 'Disconnected';
      });

      if (connected) {
        _subscribeToTopics();
        _statusAnimationController.forward();
      } else {
        _statusAnimationController.reverse();
      }
    });

    // Real-time data (position, map updates)
    _realTimeDataSubscription = _webSocketService.realTimeData.listen((data) {
      _handleRealTimeData(data);
    });

    // Mapping events
    _mappingEventsSubscription = _webSocketService.mappingEvents.listen((data) {
      _handleMappingEvent(data);
    });

    // Control events
    _controlEventsSubscription = _webSocketService.controlEvents.listen((data) {
      _handleControlEvent(data);
    });
  }

  void _subscribeToTopics() {
    if (_isConnected) {
      _webSocketService.subscribe('real_time_data', deviceId: widget.deviceId);
      _webSocketService.subscribe('mapping_events', deviceId: widget.deviceId);
      _webSocketService.subscribe('control_events', deviceId: widget.deviceId);

      // Request initial data
      _webSocketService.requestData('device_status', deviceId: widget.deviceId);
      _webSocketService.requestData('map', deviceId: widget.deviceId);
    }
  }

  void _handleRealTimeData(Map<String, dynamic> data) {
    setState(() {
      _messagesReceived++;
    });

    final messageType = data['type'];
    final deviceId = data['deviceId'];

    // Only process data for our device
    if (deviceId != null && deviceId != widget.deviceId) return;

    switch (messageType) {
      case 'position_update':
      case 'odometry_update':
        _handlePositionUpdate(data['data']);
        break;

      case 'map_update':
        _handleMapUpdate(data['data']);
        break;

      case 'mapping_odometry_update':
        // Special handling for mapping mode odometry
        _handleMappingPositionUpdate(data['data']);
        break;

      case 'velocity_feedback':
        _handleVelocityFeedback(data['data']);
        break;
    }
  }

  void _handlePositionUpdate(Map<String, dynamic> positionData) {
    try {
      final odometryData = OdometryData.fromJson(positionData);

      setState(() {
        _currentOdometry = odometryData;
        _lastPositionUpdate = DateTime.now();

        // Add to trail if mapping is active
        if (_mappingActive) {
          _addToRobotTrail(odometryData.position);
        }
      });
    } catch (e) {
      print('❌ Error processing position update: $e');
    }
  }

  void _handleMappingPositionUpdate(Map<String, dynamic> positionData) {
    try {
      final odometryData = OdometryData.fromJson(positionData);
      final mappingMode = positionData['mappingMode'] ?? false;

      setState(() {
        _currentOdometry = odometryData;
        _mappingActive = mappingMode;
        _lastPositionUpdate = DateTime.now();

        // Always add to trail when in mapping mode
        if (mappingMode) {
          _addToRobotTrail(odometryData.position);
        }
      });
    } catch (e) {
      print('❌ Error processing mapping position update: $e');
    }
  }

  void _handleMapUpdate(Map<String, dynamic> mapData) {
    try {
      final newMapData = MapData.fromJson(mapData);

      setState(() {
        _currentMapData = newMapData;
        _lastMapUpdate = DateTime.now();
      });

      print(
          '🗺️ Map updated: ${newMapData.info.width}x${newMapData.info.height}');
    } catch (e) {
      print('❌ Error processing map update: $e');
    }
  }

  void _handleVelocityFeedback(Map<String, dynamic> velocityData) {
    // Could be used to show actual robot velocity vs commanded
    // For now, just log
    if (DateTime.now().millisecondsSinceEpoch % 2000 < 100) {
      print(
          '🚗 Velocity feedback: ${velocityData['linear']?['x']?.toStringAsFixed(2) ?? '0.0'} m/s');
    }
  }

  void _handleMappingEvent(Map<String, dynamic> data) {
    final eventType = data['type'];

    switch (eventType) {
      case 'mapping_started':
        setState(() {
          _mappingActive = true;
          _robotTrail.clear(); // Start fresh trail
        });
        _showSnackBar('Mapping started', Colors.green);
        break;

      case 'mapping_stopped':
        setState(() {
          _mappingActive = false;
        });
        _showSnackBar('Mapping stopped', Colors.orange);
        break;

      case 'mapping_saved':
        _showSnackBar('Map saved successfully', Colors.blue);
        break;
    }
  }

  void _handleControlEvent(Map<String, dynamic> data) {
    final eventType = data['type'];

    switch (eventType) {
      case 'emergency_stop':
        _showSnackBar('Emergency stop activated!', Colors.red);
        break;

      case 'goal_set':
        final goalData = data['data'];
        _showSnackBar(
            'Goal set: (${goalData['x']}, ${goalData['y']})', Colors.blue);
        break;
    }
  }

  void _addToRobotTrail(Position position) {
    // Only add if robot moved significantly
    if (_robotTrail.isEmpty ||
        _distanceBetween(_robotTrail.last, position) > 0.05) {
      _robotTrail.add(position);

      // Keep trail length manageable
      if (_robotTrail.length > 500) {
        _robotTrail.removeAt(0);
      }
    }
  }

  double _distanceBetween(Position p1, Position p2) {
    return ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y))
        .abs();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Control - ${widget.deviceId}'),
        backgroundColor: _mappingActive ? Colors.green : Colors.blue,
        actions: [
          _buildConnectionIndicator(),
          _buildMappingIndicator(),
          PopupMenuButton<String>(
            onSelected: _handleMenuAction,
            itemBuilder: (BuildContext context) => [
              PopupMenuItem(value: 'settings', child: Text('Settings')),
              PopupMenuItem(value: 'save_map', child: Text('Save Map')),
              PopupMenuItem(value: 'clear_trail', child: Text('Clear Trail')),
            ],
          ),
        ],
      ),
      body: Column(
        children: [
          _buildStatusBar(),
          Expanded(
            child: Row(
              children: [
                // Left panel - Live mapping view
                Expanded(
                  flex: 3,
                  child: _buildMappingPanel(),
                ),

                // Right panel - Controls
                Expanded(
                  flex: 2,
                  child: _buildControlPanel(),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildConnectionIndicator() {
    return AnimatedBuilder(
      animation: _statusAnimationController,
      builder: (context, child) {
        return Container(
          margin: EdgeInsets.only(right: 8),
          padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
          decoration: BoxDecoration(
            color: _isConnected ? Colors.green : Colors.red,
            borderRadius: BorderRadius.circular(12),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(
                _isConnected ? Icons.wifi : Icons.wifi_off,
                size: 16,
                color: Colors.white,
              ),
              SizedBox(width: 4),
              Text(
                _isConnected ? 'Online' : 'Offline',
                style: TextStyle(
                  color: Colors.white,
                  fontSize: 12,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ],
          ),
        );
      },
    );
  }

  Widget _buildMappingIndicator() {
    if (!_mappingActive) return SizedBox.shrink();

    return Container(
      margin: EdgeInsets.only(right: 8),
      padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        color: Colors.green,
        borderRadius: BorderRadius.circular(12),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          SizedBox(
            width: 12,
            height: 12,
            child: CircularProgressIndicator(
              strokeWidth: 2,
              valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
            ),
          ),
          SizedBox(width: 4),
          Text(
            'MAPPING',
            style: TextStyle(
              color: Colors.white,
              fontSize: 12,
              fontWeight: FontWeight.bold,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildStatusBar() {
    return Container(
      height: 40,
      color: Colors.grey[100],
      padding: EdgeInsets.symmetric(horizontal: 16),
      child: Row(
        children: [
          Icon(Icons.location_on, size: 16, color: Colors.blue),
          SizedBox(width: 4),
          Text(_getPositionText()),
          SizedBox(width: 20),
          Icon(Icons.speed, size: 16, color: Colors.green),
          SizedBox(width: 4),
          Text(_getVelocityText()),
          SizedBox(width: 20),
          Icon(Icons.timeline, size: 16, color: Colors.orange),
          SizedBox(width: 4),
          Text('Trail: ${_robotTrail.length} points'),
          Spacer(),
          Text('Messages: $_messagesReceived'),
        ],
      ),
    );
  }

  String _getPositionText() {
    if (_currentOdometry?.position == null) return 'Position: N/A';

    final pos = _currentOdometry!.position;
    return 'X: ${pos.x.toStringAsFixed(2)}m  Y: ${pos.y.toStringAsFixed(2)}m';
  }

  String _getVelocityText() {
    if (_currentOdometry?.velocity?.linear == null) return 'Velocity: N/A';

    final vel = _currentOdometry!.velocity!.linear;
    final speed = (vel.x * vel.x + vel.y * vel.y).abs();
    return 'Speed: ${speed.toStringAsFixed(2)} m/s';
  }

  Widget _buildMappingPanel() {
    return Card(
      margin: EdgeInsets.all(8),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Padding(
            padding: EdgeInsets.all(16),
            child: Row(
              children: [
                Text(
                  'Live Mapping View',
                  style: TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                Spacer(),
                _buildMappingControls(),
              ],
            ),
          ),
          Expanded(
            child: LiveMappingCanvas(
              mapData: _currentMapData,
              currentOdometry: _currentOdometry,
              robotTrail: _showTrail ? _robotTrail : [],
              mappingActive: _mappingActive,
              deviceId: widget.deviceId,
              onMapChanged: (mapData) {
                setState(() {
                  _currentMapData = mapData;
                });
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMappingControls() {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        IconButton(
          onPressed: _isConnected ? _toggleMapping : null,
          icon: Icon(_mappingActive ? Icons.stop : Icons.play_arrow),
          color: _mappingActive ? Colors.red : Colors.green,
          tooltip: _mappingActive ? 'Stop Mapping' : 'Start Mapping',
        ),
        IconButton(
          onPressed: _isConnected && _currentMapData != null ? _saveMap : null,
          icon: Icon(Icons.save),
          color: Colors.blue,
          tooltip: 'Save Map',
        ),
        IconButton(
          onPressed: () {
            setState(() {
              _showTrail = !_showTrail;
            });
          },
          icon: Icon(_showTrail ? Icons.timeline : Icons.timeline_outlined),
          color: _showTrail ? Colors.orange : Colors.grey,
          tooltip: 'Toggle Trail',
        ),
      ],
    );
  }

  Widget _buildControlPanel() {
    return Card(
      margin: EdgeInsets.all(8),
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Robot Control',
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: 16),

            // Speed settings
            _buildSpeedSettings(),

            SizedBox(height: 20),

            // Joystick control
            Center(
              child: JoystickWidget(
                size: 200,
                maxLinearSpeed: _maxLinearSpeed,
                maxAngularSpeed: _maxAngularSpeed,
                enabled: _controlEnabled && _isConnected,
                requireDeadman: _useDeadmanSwitch,
                onChanged: _onJoystickChanged,
              ),
            ),

            SizedBox(height: 20),

            // Control buttons
            _buildControlButtons(),

            SizedBox(height: 20),

            // Settings
            _buildControlSettings(),
          ],
        ),
      ),
    );
  }

  Widget _buildSpeedSettings() {
    return Column(
      children: [
        Row(
          children: [
            Icon(Icons.speed, size: 16),
            SizedBox(width: 8),
            Text('Max Linear Speed:'),
            Expanded(
              child: Slider(
                value: _maxLinearSpeed,
                min: 0.1,
                max: 2.0,
                divisions: 19,
                label: '${_maxLinearSpeed.toStringAsFixed(1)} m/s',
                onChanged: (value) {
                  setState(() {
                    _maxLinearSpeed = value;
                  });
                },
              ),
            ),
            Text('${_maxLinearSpeed.toStringAsFixed(1)} m/s'),
          ],
        ),
        Row(
          children: [
            Icon(Icons.rotate_right, size: 16),
            SizedBox(width: 8),
            Text('Max Angular Speed:'),
            Expanded(
              child: Slider(
                value: _maxAngularSpeed,
                min: 0.1,
                max: 3.0,
                divisions: 29,
                label: '${_maxAngularSpeed.toStringAsFixed(1)} rad/s',
                onChanged: (value) {
                  setState(() {
                    _maxAngularSpeed = value;
                  });
                },
              ),
            ),
            Text('${_maxAngularSpeed.toStringAsFixed(1)} rad/s'),
          ],
        ),
      ],
    );
  }

  Widget _buildControlButtons() {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: [
        Expanded(
          child: ElevatedButton.icon(
            onPressed: _isConnected ? _toggleControl : null,
            icon: Icon(_controlEnabled ? Icons.pause : Icons.play_arrow),
            label: Text(_controlEnabled ? 'Disable' : 'Enable'),
            style: ElevatedButton.styleFrom(
              backgroundColor: _controlEnabled ? Colors.orange : Colors.green,
            ),
          ),
        ),
        SizedBox(width: 12),
        Expanded(
          child: ElevatedButton.icon(
            onPressed: _isConnected ? _emergencyStop : null,
            icon: Icon(Icons.stop),
            label: Text('EMERGENCY STOP'),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.red,
              foregroundColor: Colors.white,
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildControlSettings() {
    return Column(
      children: [
        SwitchListTile(
          title: Text('Deadman Switch'),
          subtitle: Text('Require holding joystick'),
          value: _useDeadmanSwitch,
          onChanged: (value) {
            setState(() {
              _useDeadmanSwitch = value;
            });
          },
        ),
        SwitchListTile(
          title: Text('Auto Center Map'),
          subtitle: Text('Center map on robot'),
          value: _autoCenter,
          onChanged: (value) {
            setState(() {
              _autoCenter = value;
            });
          },
        ),
      ],
    );
  }

  void _onJoystickChanged(double linear, double angular, bool deadmanActive) {
    if (_isConnected) {
      _webSocketService.sendMessage({
        'type': 'joystick_control',
        'deviceId': widget.deviceId,
        'x': angular / _maxAngularSpeed, // Normalize
        'y': linear / _maxLinearSpeed, // Normalize
        'deadman': deadmanActive,
      });
    }
  }

  void _toggleMapping() {
    if (_mappingActive) {
      _webSocketService.sendMessage({
        'type': 'mapping_command',
        'deviceId': widget.deviceId,
        'action': 'stop',
      });
    } else {
      _webSocketService.sendMessage({
        'type': 'mapping_command',
        'deviceId': widget.deviceId,
        'action': 'start',
      });
    }
  }

  void _saveMap() async {
    if (_currentMapData == null) {
      _showSnackBar('No map data to save', Colors.red);
      return;
    }

    try {
      final response = await _apiService.saveMapData(
        deviceId: widget.deviceId,
        mapData: _currentMapData!,
      );
      if (response['success'] == true) {
        _showSnackBar('Map saved and published!', Colors.green);
      } else {
        _showSnackBar('Failed to save map: ${response['error']}', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error saving map: $e', Colors.red);
    }
  }

  void _toggleControl() {
    setState(() {
      _controlEnabled = !_controlEnabled;
    });

    if (!_controlEnabled) {
      // Stop robot when disabling control
      _webSocketService.sendMessage({
        'type': 'control_command',
        'deviceId': widget.deviceId,
        'command': 'stop',
      });
    }
  }

  void _emergencyStop() {
    _webSocketService.sendMessage({
      'type': 'control_command',
      'deviceId': widget.deviceId,
      'command': 'stop',
    });

    _showSnackBar('Emergency stop activated!', Colors.red);
  }

  void _handleMenuAction(String action) {
    switch (action) {
      case 'save_map':
        _saveMap();
        break;
      case 'clear_trail':
        setState(() {
          _robotTrail.clear();
        });
        break;
      case 'settings':
        _showSettingsDialog();
        break;
    }
  }

  void _showSettingsDialog() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Control Settings'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text('Additional settings can be added here'),
            // Add more settings as needed
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Close'),
          ),
        ],
      ),
    );
  }

  void _showSnackBar(String message, Color color) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: color,
        duration: Duration(seconds: 2),
      ),
    );
  }

  @override
  void dispose() {
    _realTimeDataSubscription.cancel();
    _mappingEventsSubscription.cancel();
    _controlEventsSubscription.cancel();
    _connectionStateSubscription.cancel();
    _statusAnimationController.dispose();
    super.dispose();
  }

  void _navigateToControl(Map<String, dynamic> device) async {
    // Connect WebSocket if not already connected
    if (!_webSocketService.isConnected) {
      await _webSocketService.connect('ws://192.168.253.79:3000');
    }
    Navigator.push(
      context,
      MaterialPageRoute(
        builder: (context) => ControlPage(deviceId: device['id']),
      ),
    );
  }
}
