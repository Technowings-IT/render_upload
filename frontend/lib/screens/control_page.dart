// screens/control_page.dart - Fixed Responsive Design with Live Mapping Integration
// Compatible with provided map_data.dart structure
import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import '../services/web_socket_service.dart';
import '../services/api_service.dart';
import '../widgets/joystick.dart';
import '../widgets/live_maping_canvas.dart';
import '../models/map_data.dart';
import '../models/odom.dart' as odom;

class ControlPage extends StatefulWidget {
  final String deviceId;

  const ControlPage({
    Key? key,
    required this.deviceId,
  }) : super(key: key);

  @override
  State<ControlPage> createState() => _ControlPageState();
}

class _ControlPageState extends State<ControlPage>
    with TickerProviderStateMixin {
  final WebSocketService _webSocketService = WebSocketService();
  final ApiService _apiService = ApiService();

  // Subscriptions
  StreamSubscription<Map<String, dynamic>>? _realTimeDataSubscription;
  StreamSubscription<Map<String, dynamic>>? _mappingEventsSubscription;
  StreamSubscription<Map<String, dynamic>>? _controlEventsSubscription;
  StreamSubscription<bool>? _connectionStateSubscription;
  StreamSubscription<String>? _errorSubscription;

  // Animation controllers
  late AnimationController _statusAnimationController;
  late TabController _tabController;

  // State variables
  bool _isConnected = false;
  bool _mappingActive = false;
  bool _controlEnabled = true;
  MapData? _currentMapData; // Fixed: Added missing variable
  List<odom.Position> _robotTrail = []; // Using Position from odom.dart
  odom.OdometryData? _currentOdometry;

  // Control settings with mobile-friendly defaults
  double _maxLinearSpeed = 0.3;
  double _maxAngularSpeed = 0.8;
  bool _useDeadmanSwitch = true;
  bool _showTrail = true;
  bool _autoCenter = true;

  // UI state for mobile
  bool _showAdvancedSettings = false;
  bool _showStatusDetails = false;
  int _currentTabIndex = 0;

  // Current velocity tracking
  double _currentLinear = 0.0;
  double _currentAngular = 0.0;

  // Costmap data
  Map<String, dynamic>? _globalCostmap;
  Map<String, dynamic>? _localCostmap;

  // Map overlay settings
  bool _showGlobalCostmap = true;
  bool _showLocalCostmap = true;
  bool _showOccupancyGrid = true;
  double _costmapOpacity = 0.7;

  // Status info
  String _connectionStatus = 'Disconnected';
  DateTime? _lastPositionUpdate;
  DateTime? _lastMapUpdate;
  int _messagesReceived = 0;

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _initializeConnection();
    _setupSubscriptions();
  }

  void _initializeAnimations() {
    _statusAnimationController = AnimationController(
      duration: const Duration(milliseconds: 1000),
      vsync: this,
    );

    _tabController = TabController(
      length: 2, // EXACTLY 2 tabs: Live Control, Settings
      vsync: this,
    );

    // Debug check
    print('📋 TabController initialized with length: ${_tabController.length}');
  }

  void _initializeConnection() {
    setState(() {
      _isConnected = _webSocketService.isConnected;
    });

    if (!_webSocketService.isConnected) {
      _webSocketService.connect('ws://192.168.253.79:3000',
          deviceId: widget.deviceId,
          deviceInfo: {
            'type': 'mobile_controller',
            'name': 'Flutter Control App'
          }).then((connected) {
        if (connected) {
          _subscribeToTopics();
        }
      });
    } else {
      _subscribeToTopics();
    }
  }

  void _setupSubscriptions() {
    // Connection state changes
    _connectionStateSubscription =
        _webSocketService.connectionState.listen((connected) {
      if (mounted) {
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
      }
    });

    // Real-time data (position, map updates)
    _realTimeDataSubscription = _webSocketService.realTimeData.listen((data) {
      if (mounted) {
        _handleRealTimeData(data);
      }
    });

    // Mapping events
    _mappingEventsSubscription = _webSocketService.mappingEvents.listen((data) {
      if (mounted) {
        _handleMappingEvent(data);
      }
    });

    // Control events
    _controlEventsSubscription = _webSocketService.controlEvents.listen((data) {
      if (mounted) {
        _handleControlEvent(data);
      }
    });

    // Error handling
    _errorSubscription = _webSocketService.errors.listen((error) {
      if (mounted) {
        _showSnackBar('WebSocket Error: $error', Colors.red);
      }
    });
  }

  void _subscribeToTopics() {
    if (_isConnected) {
      _webSocketService.subscribe('real_time_data', deviceId: widget.deviceId);
      _webSocketService.subscribe('mapping_events', deviceId: widget.deviceId);
      _webSocketService.subscribe('control_events', deviceId: widget.deviceId);
      print('📡 Subscribed to topics for device: ${widget.deviceId}');
    }
  }

  // Data handling methods
  void _handleRealTimeData(Map<String, dynamic> data) {
    setState(() {
      _messagesReceived++;
    });
    final messageType = data['type'];
    final deviceId = data['deviceId'];
    if (deviceId != null && deviceId != widget.deviceId) return;

    switch (messageType) {
      case 'position_update':
      case 'odometry_update':
        _handlePositionUpdate(data['data']);
        break;
      case 'map_update':
        _handleMapUpdate(data['data']);
        break;
      case 'velocity_feedback':
        _handleVelocityFeedback(data['data']);
        break;
      case 'global_costmap_update':
        _handleGlobalCostmapUpdate(data['data']);
        break;
      case 'local_costmap_update':
        _handleLocalCostmapUpdate(data['data']);
        break;
    }
  }

  void _handlePositionUpdate(Map<String, dynamic> positionData) {
    try {
      final odometryData = odom.OdometryData.fromJson(positionData);
      setState(() {
        _currentOdometry = odometryData;
        _lastPositionUpdate = DateTime.now();
        if (_mappingActive) {
          // Convert odom.Position to odom.Position (they're the same type)
          _addToRobotTrail(odometryData.position);
        }
      });
    } catch (e) {
      debugPrint('❌ Error processing position update: $e');
    }
  }

  void _handleMapUpdate(Map<String, dynamic> mapData) {
    try {
      final newMapData = MapData.fromJson(mapData);
      setState(() {
        _currentMapData = newMapData;
        _lastMapUpdate = DateTime.now();
      });
    } catch (e) {
      debugPrint('❌ Error processing map update: $e');
    }
  }

  void _handleVelocityFeedback(Map<String, dynamic> velocityData) {
    if (mounted) {
      setState(() {
        _currentLinear = velocityData['linear']?['x']?.toDouble() ?? 0.0;
        _currentAngular = velocityData['angular']?['z']?.toDouble() ?? 0.0;
      });
    }
  }

  void _handleGlobalCostmapUpdate(Map<String, dynamic> costmapData) {
    setState(() {
      _globalCostmap = costmapData;
    });
  }

  void _handleLocalCostmapUpdate(Map<String, dynamic> costmapData) {
    setState(() {
      _localCostmap = costmapData;
    });
  }

  void _handleMappingEvent(Map<String, dynamic> data) {
    final eventType = data['type'];
    switch (eventType) {
      case 'mapping_started':
      case 'mapping_start':
        setState(() {
          _mappingActive = true;
          _robotTrail.clear();
        });
        _showSnackBar('Mapping started', Colors.green);
        break;
      case 'mapping_stopped':
      case 'mapping_stop':
        setState(() {
          _mappingActive = false;
        });
        _showSnackBar('Mapping stopped', Colors.orange);
        break;
      case 'mapping_result':
        final success = data['result']?['success'] ?? false;
        final command = data['command'] ?? 'unknown';
        _showSnackBar(
          'Mapping $command: ${success ? 'Success' : 'Failed'}',
          success ? Colors.green : Colors.red,
        );
        break;
    }
  }

  void _handleControlEvent(Map<String, dynamic> data) {
    final eventType = data['type'];
    switch (eventType) {
      case 'emergency_stop':
        _showSnackBar('Emergency stop activated!', Colors.red);
        break;
      case 'joystick_result':
        final success = data['result']?['success'] ?? true;
        if (!success) {
          _showSnackBar('Joystick command failed', Colors.orange);
        }
        break;
    }
  }

  void _addToRobotTrail(odom.Position position) {
    if (_robotTrail.isEmpty ||
        _distanceBetween(_robotTrail.last, position) > 0.05) {
      _robotTrail.add(position);
      if (_robotTrail.length > 500) {
        _robotTrail.removeAt(0);
      }
    }
  }

  double _distanceBetween(odom.Position p1, odom.Position p2) {
    return math
        .sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  }

  // Helper method to determine device type
  DeviceType _getDeviceType(BuildContext context) {
    final screenWidth = MediaQuery.of(context).size.width;

    if (screenWidth > 1024) {
      return DeviceType.desktop;
    } else if (screenWidth > 768) {
      return DeviceType.tablet;
    } else {
      return DeviceType.phone;
    }
  }

  @override
  Widget build(BuildContext context) {
    final deviceType = _getDeviceType(context);

    // Safety check: ensure TabController has correct length
    if (_tabController.length != 2) {
      print(
          '🚨 TabController length mismatch! Expected: 2, Actual: ${_tabController.length}');
      // Recreate TabController with correct length
      _tabController.dispose();
      _tabController = TabController(length: 2, vsync: this);
    }

    return Scaffold(
      appBar: AppBar(
        title: Text('Live Control - ${widget.deviceId}'),
        backgroundColor: _mappingActive ? Colors.green : Colors.blue,
        elevation: 2,
        actions: [
          _buildConnectionIndicator(),
          if (_mappingActive) _buildMappingIndicator(),
          IconButton(
            icon: const Icon(Icons.emergency, color: Colors.red),
            onPressed: _emergencyStop,
            tooltip: 'Emergency Stop',
          ),
        ],
        bottom: TabBar(
          controller: _tabController,
          indicatorColor: Colors.white,
          labelColor: Colors.white,
          unselectedLabelColor: Colors.white70,
          tabs: const [
            Tab(
                icon: Icon(Icons.control_camera),
                text: 'Live Control'), // Tab 1
            Tab(icon: Icon(Icons.settings), text: 'Settings'), // Tab 2
          ], // EXACTLY 2 tabs
        ),
      ),
      body: Column(
        children: [
          // Compact status bar
          _buildCompactStatusBar(),

          // Main content with tabs
          Expanded(
            child: TabBarView(
              controller: _tabController,
              children: [
                _buildLiveMappingTab(deviceType), // Child 1
                _buildSettingsTab(), // Child 2
              ], // EXACTLY 2 children
            ),
          ),
        ],
      ),
      // Floating action button for quick emergency stop
      floatingActionButton: FloatingActionButton(
        onPressed: _emergencyStop,
        backgroundColor: Colors.red,
        child: const Icon(Icons.stop, color: Colors.white),
        tooltip: 'Emergency Stop',
      ),
    );
  }

  Widget _buildLiveMappingTab(DeviceType deviceType) {
    switch (deviceType) {
      case DeviceType.desktop:
        return _buildDesktopLayout();
      case DeviceType.tablet:
        return _buildTabletLayout();
      case DeviceType.phone:
      default:
        return _buildPhoneLayout();
    }
  }

  Widget _buildDesktopLayout() {
    return Row(
      children: [
        // Map takes 60% of width
        Expanded(
          flex: 6,
          child: Container(
            margin: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              border: Border.all(color: Colors.grey.shade300),
              borderRadius: BorderRadius.circular(8),
            ),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(8),
              child: LiveMappingCanvas(
                mapData: _currentMapData,
                currentOdometry: _currentOdometry,
                robotTrail: _showTrail ? _robotTrail : [],
                mappingActive: _mappingActive,
                deviceId: widget.deviceId,
                globalCostmap: _showGlobalCostmap ? _globalCostmap : null,
                localCostmap: _showLocalCostmap ? _localCostmap : null,
                showOccupancyGrid: _showOccupancyGrid,
                costmapOpacity: _costmapOpacity,
                onMapChanged: (mapData) {
                  setState(() {
                    _currentMapData = mapData;
                  });
                },
                onTrailSaved: _saveTrailAsMap,
              ),
            ),
          ),
        ),

        // Controls take 40% of width
        Expanded(
          flex: 4,
          child: Container(
            padding: const EdgeInsets.all(16),
            child: _buildControlPanel(isCompact: false),
          ),
        ),
      ],
    );
  }

  Widget _buildTabletLayout() {
    return Row(
      children: [
        // Map takes 55% of width
        Expanded(
          flex: 55,
          child: Container(
            margin: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              border: Border.all(color: Colors.grey.shade300),
              borderRadius: BorderRadius.circular(8),
            ),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(8),
              child: LiveMappingCanvas(
                mapData: _currentMapData,
                currentOdometry: _currentOdometry,
                robotTrail: _showTrail ? _robotTrail : [],
                mappingActive: _mappingActive,
                deviceId: widget.deviceId,
                globalCostmap: _showGlobalCostmap ? _globalCostmap : null,
                localCostmap: _showLocalCostmap ? _localCostmap : null,
                showOccupancyGrid: _showOccupancyGrid,
                costmapOpacity: _costmapOpacity,
                onMapChanged: (mapData) {
                  setState(() {
                    _currentMapData = mapData;
                  });
                },
                onTrailSaved: _saveTrailAsMap,
              ),
            ),
          ),
        ),

        // Controls take 45% of width
        Expanded(
          flex: 45,
          child: Container(
            padding: const EdgeInsets.all(12),
            child: _buildControlPanel(isCompact: true),
          ),
        ),
      ],
    );
  }

  Widget _buildPhoneLayout() {
    final screenHeight = MediaQuery.of(context).size.height;
    final mapHeight = screenHeight * 0.45; // 45% for map

    return Column(
      children: [
        // Map section
        Container(
          height: mapHeight,
          margin: const EdgeInsets.all(8),
          decoration: BoxDecoration(
            border: Border.all(color: Colors.grey.shade300),
            borderRadius: BorderRadius.circular(8),
          ),
          child: ClipRRect(
            borderRadius: BorderRadius.circular(8),
            child: LiveMappingCanvas(
              mapData: _currentMapData,
              currentOdometry: _currentOdometry,
              robotTrail: _showTrail ? _robotTrail : [],
              mappingActive: _mappingActive,
              deviceId: widget.deviceId,
              globalCostmap: _showGlobalCostmap ? _globalCostmap : null,
              localCostmap: _showLocalCostmap ? _localCostmap : null,
              showOccupancyGrid: _showOccupancyGrid,
              costmapOpacity: _costmapOpacity,
              onMapChanged: (mapData) {
                setState(() {
                  _currentMapData = mapData;
                });
              },
              onTrailSaved: _saveTrailAsMap,
            ),
          ),
        ),

        // Controls section
        Expanded(
          child: SingleChildScrollView(
            padding: const EdgeInsets.symmetric(horizontal: 8),
            child: _buildControlPanel(isCompact: true),
          ),
        ),
      ],
    );
  }

  Widget _buildControlPanel({required bool isCompact}) {
    final screenWidth = MediaQuery.of(context).size.width;
    final joystickSize = isCompact
        ? math.min(screenWidth * 0.4, 200.0)
        : math.min(screenWidth * 0.3, 250.0);

    return Column(
      children: [
        // Speed controls - compact version
        _buildCompactSpeedControls(),

        SizedBox(height: isCompact ? 12 : 20),

        // Main joystick control
        Center(
          child: JoystickWidget(
            size: joystickSize,
            maxLinearSpeed: _maxLinearSpeed,
            maxAngularSpeed: _maxAngularSpeed,
            enabled: _controlEnabled && _isConnected,
            requireDeadman: _useDeadmanSwitch,
            onChanged: _onJoystickChanged,
          ),
        ),

        SizedBox(height: isCompact ? 12 : 20),

        // Current velocity display
        _buildVelocityDisplay(),

        SizedBox(height: isCompact ? 12 : 20),

        // Control buttons
        _buildMobileControlButtons(),

        SizedBox(height: isCompact ? 12 : 20),

        // Mapping quick controls
        _buildMappingQuickControls(),

        SizedBox(height: isCompact ? 12 : 20),

        // Map overlay controls
        _buildMapOverlayControls(),
      ],
    );
  }

  Widget _buildConnectionIndicator() {
    return Container(
      margin: const EdgeInsets.only(right: 8),
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        color: _isConnected ? Colors.green : Colors.red,
        borderRadius: BorderRadius.circular(12),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(
            _isConnected ? Icons.wifi : Icons.wifi_off,
            size: 14,
            color: Colors.white,
          ),
          const SizedBox(width: 4),
          Text(
            _isConnected ? 'Online' : 'Offline',
            style: const TextStyle(
              color: Colors.white,
              fontSize: 10,
              fontWeight: FontWeight.bold,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMappingIndicator() {
    return Container(
      margin: const EdgeInsets.only(right: 8),
      padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 4),
      decoration: BoxDecoration(
        color: Colors.green,
        borderRadius: BorderRadius.circular(12),
      ),
      child: const Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          SizedBox(
            width: 10,
            height: 10,
            child: CircularProgressIndicator(
              strokeWidth: 1.5,
              valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
            ),
          ),
          SizedBox(width: 4),
          Text(
            'MAPPING',
            style: TextStyle(
              color: Colors.white,
              fontSize: 9,
              fontWeight: FontWeight.bold,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildCompactStatusBar() {
    return Container(
      height: 40,
      color: Colors.grey[100],
      child: SingleChildScrollView(
        scrollDirection: Axis.horizontal,
        padding: const EdgeInsets.symmetric(horizontal: 16),
        child: Row(
          children: [
            _buildStatusItem(
                Icons.location_on, _getPositionText(), Colors.blue),
            const SizedBox(width: 16),
            _buildStatusItem(Icons.speed, _getVelocityText(), Colors.green),
            const SizedBox(width: 16),
            _buildStatusItem(
                Icons.timeline, 'Trail: ${_robotTrail.length}', Colors.orange),
            const SizedBox(width: 16),
            _buildStatusItem(
                Icons.layers,
                'Shapes: ${_currentMapData?.shapes.length ?? 0}',
                Colors.purple),
            const SizedBox(width: 16),
            _buildStatusItem(
                Icons.message, 'Msgs: $_messagesReceived', Colors.indigo),
            const SizedBox(width: 16),
            if (_globalCostmap != null)
              _buildStatusItem(Icons.public, 'Global', Colors.blue),
            if (_localCostmap != null) ...[
              const SizedBox(width: 16),
              _buildStatusItem(Icons.near_me, 'Local', Colors.red),
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildStatusItem(IconData icon, String text, Color color) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(icon, size: 14, color: color),
        const SizedBox(width: 4),
        Text(
          text,
          style: const TextStyle(fontSize: 12),
        ),
      ],
    );
  }

  Widget _buildCompactSpeedControls() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          children: [
            Row(
              children: [
                const Icon(Icons.info, size: 16, color: Colors.blue),
                const SizedBox(width: 8),
                const Expanded(
                  child: Text(
                    'Speed Limits',
                    style: TextStyle(fontWeight: FontWeight.bold),
                  ),
                ),
                TextButton(
                  onPressed: () {
                    setState(() {
                      _showAdvancedSettings = !_showAdvancedSettings;
                    });
                  },
                  child: Text(_showAdvancedSettings ? 'Hide' : 'Adjust'),
                ),
              ],
            ),

            // Always show current limits
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                _buildSpeedChip('Linear', _maxLinearSpeed, 'm/s', Colors.blue),
                _buildSpeedChip(
                    'Angular', _maxAngularSpeed, 'rad/s', Colors.orange),
              ],
            ),

            // Show sliders when expanded
            if (_showAdvancedSettings) ...[
              const SizedBox(height: 12),
              _buildSpeedSlider(
                'Linear Speed',
                _maxLinearSpeed,
                0.1,
                2.0,
                'm/s',
                (value) => setState(() => _maxLinearSpeed = value),
              ),
              _buildSpeedSlider(
                'Angular Speed',
                _maxAngularSpeed,
                0.1,
                3.0,
                'rad/s',
                (value) => setState(() => _maxAngularSpeed = value),
              ),
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildSpeedChip(String label, double value, String unit, Color color) {
    return Chip(
      avatar: CircleAvatar(
        backgroundColor: color,
        radius: 12,
        child: Text(
          label[0],
          style: const TextStyle(
            color: Colors.white,
            fontSize: 12,
            fontWeight: FontWeight.bold,
          ),
        ),
      ),
      label: Text('${value.toStringAsFixed(1)} $unit'),
      backgroundColor: color.withOpacity(0.1),
    );
  }

  Widget _buildSpeedSlider(
    String label,
    double value,
    double min,
    double max,
    String unit,
    Function(double) onChanged,
  ) {
    return Column(
      children: [
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Text(label, style: const TextStyle(fontSize: 14)),
            Text('${value.toStringAsFixed(1)} $unit',
                style: const TextStyle(fontWeight: FontWeight.bold)),
          ],
        ),
        Slider(
          value: value,
          min: min,
          max: max,
          divisions: ((max - min) / 0.1).round(),
          onChanged: onChanged,
        ),
      ],
    );
  }

  Widget _buildVelocityDisplay() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            _buildVelocityItem(
              'Linear',
              _currentLinear,
              _maxLinearSpeed,
              'm/s',
              Icons.arrow_upward,
            ),
            Container(
              width: 1,
              height: 40,
              color: Colors.grey.shade300,
            ),
            _buildVelocityItem(
              'Angular',
              _currentAngular,
              _maxAngularSpeed,
              'rad/s',
              Icons.rotate_right,
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildVelocityItem(
    String label,
    double value,
    double maxValue,
    String unit,
    IconData icon,
  ) {
    final percentage = maxValue > 0 ? (value.abs() / maxValue * 100) : 0.0;
    Color color = value > 0
        ? Colors.green
        : value < 0
            ? Colors.red
            : Colors.grey;

    return Column(
      children: [
        Icon(icon, color: color, size: 20),
        Text(label, style: const TextStyle(fontSize: 12)),
        Text(
          '${value.toStringAsFixed(2)} $unit',
          style: TextStyle(
            fontWeight: FontWeight.bold,
            color: value != 0 ? Colors.blue : Colors.grey,
            fontSize: 14,
          ),
        ),
        Text(
          '${percentage.toStringAsFixed(0)}%',
          style: TextStyle(
            fontSize: 10,
            color: Colors.grey.shade600,
          ),
        ),
      ],
    );
  }

  Widget _buildMobileControlButtons() {
    return Column(
      children: [
        Row(
          children: [
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _isConnected ? _toggleControl : null,
                icon: Icon(_controlEnabled ? Icons.pause : Icons.play_arrow),
                label: Text(_controlEnabled ? 'Disable' : 'Enable'),
                style: ElevatedButton.styleFrom(
                  backgroundColor:
                      _controlEnabled ? Colors.orange : Colors.green,
                  minimumSize: const Size(0, 48),
                ),
              ),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _isConnected ? _emergencyStop : null,
                icon: const Icon(Icons.stop),
                label: const Text('STOP'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.red,
                  foregroundColor: Colors.white,
                  minimumSize: const Size(0, 48),
                ),
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildMappingQuickControls() {
    if (!_isConnected) return const SizedBox.shrink();

    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Mapping Controls',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 8),
            Row(
              children: [
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: () => _toggleMapping(),
                    icon: Icon(_mappingActive ? Icons.stop : Icons.play_arrow),
                    label: Text(_mappingActive ? 'Stop' : 'Start'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor:
                          _mappingActive ? Colors.red : Colors.green,
                      foregroundColor: Colors.white,
                      minimumSize: const Size(0, 40),
                    ),
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _currentMapData != null ? _saveMap : null,
                    icon: const Icon(Icons.save),
                    label: const Text('Save'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.blue,
                      foregroundColor: Colors.white,
                      minimumSize: const Size(0, 40),
                    ),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 8),
            Row(
              children: [
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _robotTrail.isNotEmpty ? _saveTrailAsMap : null,
                    icon: const Icon(Icons.timeline),
                    label: const Text('Save Trail'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.purple,
                      foregroundColor: Colors.white,
                      minimumSize: const Size(0, 40),
                    ),
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _openMapEditor,
                    icon: const Icon(Icons.edit),
                    label: const Text('Edit'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.orange,
                      foregroundColor: Colors.white,
                      minimumSize: const Size(0, 40),
                    ),
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildMapOverlayControls() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Map Layers',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 8),

            // Occupancy grid toggle
            SwitchListTile(
              title: const Text('Occupancy Grid'),
              subtitle: const Text('Show main SLAM map'),
              value: _showOccupancyGrid,
              onChanged: (value) {
                setState(() {
                  _showOccupancyGrid = value;
                });
              },
              dense: true,
            ),

            // Global costmap toggle
            SwitchListTile(
              title: const Text('Global Costmap'),
              subtitle: const Text('Show global path planning layer'),
              value: _showGlobalCostmap,
              onChanged: (value) {
                setState(() {
                  _showGlobalCostmap = value;
                });
              },
              dense: true,
            ),

            // Local costmap toggle
            SwitchListTile(
              title: const Text('Local Costmap'),
              subtitle: const Text('Show local obstacle detection'),
              value: _showLocalCostmap,
              onChanged: (value) {
                setState(() {
                  _showLocalCostmap = value;
                });
              },
              dense: true,
            ),

            // Robot trail toggle
            SwitchListTile(
              title: const Text('Robot Trail'),
              subtitle:
                  Text('Show path history (${_robotTrail.length} points)'),
              value: _showTrail,
              onChanged: (value) {
                setState(() {
                  _showTrail = value;
                });
              },
              dense: true,
            ),

            // Costmap opacity slider
            const SizedBox(height: 8),
            Row(
              children: [
                const Text('Layer Opacity: '),
                Expanded(
                  child: Slider(
                    value: _costmapOpacity,
                    min: 0.1,
                    max: 1.0,
                    divisions: 9,
                    label: '${(_costmapOpacity * 100).round()}%',
                    onChanged: (value) {
                      setState(() {
                        _costmapOpacity = value;
                      });
                    },
                  ),
                ),
                Text('${(_costmapOpacity * 100).round()}%'),
              ],
            ),

            // Trail controls with enhanced options
            if (_robotTrail.isNotEmpty) ...[
              const SizedBox(height: 8),
              Row(
                children: [
                  Expanded(
                    child: OutlinedButton.icon(
                      onPressed: () {
                        setState(() {
                          _robotTrail.clear();
                        });
                        _showSnackBar('Trail cleared', Colors.orange);
                      },
                      icon: const Icon(Icons.clear),
                      label: const Text('Clear Trail'),
                    ),
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: OutlinedButton.icon(
                      onPressed: _convertTrailToWaypoints,
                      icon: const Icon(Icons.route),
                      label: const Text('To Waypoints'),
                    ),
                  ),
                ],
              ),
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildSettingsTab() {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Control Settings
          Card(
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text(
                    'Control Settings',
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                  const SizedBox(height: 16),
                  SwitchListTile(
                    title: const Text('Deadman Switch'),
                    subtitle: const Text('Require holding joystick to move'),
                    value: _useDeadmanSwitch,
                    onChanged: (value) {
                      setState(() {
                        _useDeadmanSwitch = value;
                      });
                    },
                  ),
                  SwitchListTile(
                    title: const Text('Auto Center Map'),
                    subtitle: const Text('Center map on robot position'),
                    value: _autoCenter,
                    onChanged: (value) {
                      setState(() {
                        _autoCenter = value;
                      });
                    },
                  ),
                  SwitchListTile(
                    title: const Text('Show Robot Trail'),
                    subtitle: const Text('Display path history on map'),
                    value: _showTrail,
                    onChanged: (value) {
                      setState(() {
                        _showTrail = value;
                      });
                    },
                  ),
                  SwitchListTile(
                    title: const Text('Show Global Costmap'),
                    subtitle: const Text('Display global planning layer'),
                    value: _showGlobalCostmap,
                    onChanged: (value) {
                      setState(() {
                        _showGlobalCostmap = value;
                      });
                    },
                  ),
                  SwitchListTile(
                    title: const Text('Show Local Costmap'),
                    subtitle: const Text('Display local obstacle layer'),
                    value: _showLocalCostmap,
                    onChanged: (value) {
                      setState(() {
                        _showLocalCostmap = value;
                      });
                    },
                  ),
                ],
              ),
            ),
          ),

          const SizedBox(height: 16),

          // Speed Settings
          Card(
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text(
                    'Speed Settings',
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                  const SizedBox(height: 16),
                  _buildSpeedSlider(
                    'Max Linear Speed',
                    _maxLinearSpeed,
                    0.1,
                    2.0,
                    'm/s',
                    (value) => setState(() => _maxLinearSpeed = value),
                  ),
                  const SizedBox(height: 16),
                  _buildSpeedSlider(
                    'Max Angular Speed',
                    _maxAngularSpeed,
                    0.1,
                    3.0,
                    'rad/s',
                    (value) => setState(() => _maxAngularSpeed = value),
                  ),
                ],
              ),
            ),
          ),

          const SizedBox(height: 16),

          // Map Analysis Card
          if (_currentMapData != null) ...[
            Card(
              child: Padding(
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Text(
                      'Map Analysis',
                      style:
                          TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                    ),
                    const SizedBox(height: 16),
                    _buildMapAnalysis(_currentMapData!),
                  ],
                ),
              ),
            ),
            const SizedBox(height: 16),
          ],

          // Status Information
          Card(
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text(
                    'Status Information',
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                  const SizedBox(height: 16),
                  _buildStatusRow('Connected', _isConnected ? 'Yes' : 'No'),
                  _buildStatusRow(
                      'Mapping Active', _mappingActive ? 'Yes' : 'No'),
                  _buildStatusRow(
                      'Control Enabled', _controlEnabled ? 'Yes' : 'No'),
                  _buildStatusRow(
                      'Messages Received', _messagesReceived.toString()),
                  _buildStatusRow(
                      'Trail Points', _robotTrail.length.toString()),
                  _buildStatusRow('Map Shapes',
                      _currentMapData?.shapes.length.toString() ?? '0'),
                  _buildStatusRow('Global Costmap',
                      _globalCostmap != null ? 'Available' : 'N/A'),
                  _buildStatusRow('Local Costmap',
                      _localCostmap != null ? 'Available' : 'N/A'),
                  if (_currentMapData != null) ...[
                    _buildStatusRow(
                        'Map Version', _currentMapData!.version.toString()),
                    _buildStatusRow('Map Size',
                        '${_currentMapData!.info.width}x${_currentMapData!.info.height}'),
                    _buildStatusRow('Resolution',
                        '${_currentMapData!.info.resolution.toStringAsFixed(3)}m/px'),
                  ],
                  if (_currentOdometry?.position != null)
                    _buildStatusRow(
                      'Position',
                      '(${_currentOdometry!.position.x.toStringAsFixed(2)}, ${_currentOdometry!.position.y.toStringAsFixed(2)})',
                    ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMapAnalysis(MapData mapData) {
    // Analyze the map data
    final shapesByType = <String, int>{};
    for (final shape in mapData.shapes) {
      shapesByType[shape.type] = (shapesByType[shape.type] ?? 0) + 1;
    }

    final occupiedCells =
        mapData.occupancyData.where((cell) => cell == 100).length;
    final freeCells = mapData.occupancyData.where((cell) => cell == 0).length;
    final unknownCells =
        mapData.occupancyData.where((cell) => cell == -1).length;
    final totalCells = mapData.occupancyData.length;

    // Map area calculation moved outside widget tree
    final mapAreaM2 = (mapData.info.width * mapData.info.resolution) *
        (mapData.info.height * mapData.info.resolution);

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        // Map coverage
        _buildAnalysisRow(
          'Map Coverage',
          '${((freeCells + occupiedCells) / totalCells * 100).toStringAsFixed(1)}%',
          Colors.blue,
        ),
        _buildAnalysisRow(
          'Free Space',
          '${(freeCells / totalCells * 100).toStringAsFixed(1)}%',
          Colors.green,
        ),
        _buildAnalysisRow(
          'Obstacles',
          '${(occupiedCells / totalCells * 100).toStringAsFixed(1)}%',
          Colors.red,
        ),

        if (shapesByType.isNotEmpty) ...[
          const SizedBox(height: 8),
          const Divider(),
          const SizedBox(height: 8),
          const Text('Shape Summary:',
              style: TextStyle(fontWeight: FontWeight.w600)),
          const SizedBox(height: 4),
          ...shapesByType.entries.map(
            (entry) => _buildAnalysisRow(
              '${entry.key.toUpperCase()}',
              '${entry.value}',
              _getShapeTypeColor(entry.key),
            ),
          ),
        ],

        const SizedBox(height: 8),
        const Divider(),
        const SizedBox(height: 8),

        _buildAnalysisRow(
          'Total Area',
          '${mapAreaM2.toStringAsFixed(1)} m²',
          Colors.purple,
        ),

        _buildAnalysisRow(
          'Last Updated',
          _formatTimestamp(mapData.timestamp),
          Colors.grey,
        ),
      ],
    );
  }

  Widget _buildAnalysisRow(String label, String value, Color color) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          Container(
            width: 8,
            height: 8,
            decoration: BoxDecoration(
              color: color,
              shape: BoxShape.circle,
            ),
          ),
          const SizedBox(width: 8),
          Expanded(child: Text(label)),
          Text(
            value,
            style: const TextStyle(fontWeight: FontWeight.bold),
          ),
        ],
      ),
    );
  }

  Color _getShapeTypeColor(String type) {
    switch (type) {
      case 'pickup':
        return Colors.orange;
      case 'drop':
        return Colors.green;
      case 'charging':
        return Colors.blue;
      case 'obstacle':
        return Colors.red;
      case 'waypoint':
        return Colors.purple;
      default:
        return Colors.grey;
    }
  }

  String _formatTimestamp(DateTime timestamp) {
    final now = DateTime.now();
    final difference = now.difference(timestamp);

    if (difference.inMinutes < 1) {
      return 'Just now';
    } else if (difference.inHours < 1) {
      return '${difference.inMinutes}m ago';
    } else if (difference.inDays < 1) {
      return '${difference.inHours}h ago';
    } else {
      return '${difference.inDays}d ago';
    }
  }

  Widget _buildStatusRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label),
          Text(
            value,
            style: const TextStyle(fontWeight: FontWeight.bold),
          ),
        ],
      ),
    );
  }

  String _getPositionText() {
    if (_currentOdometry?.position == null) return 'N/A';
    final pos = _currentOdometry!.position;
    return '${pos.x.toStringAsFixed(1)}, ${pos.y.toStringAsFixed(1)}';
  }

  String _getVelocityText() {
    final speed = math.sqrt(
        _currentLinear * _currentLinear + _currentAngular * _currentAngular);
    return '${speed.toStringAsFixed(2)} m/s';
  }

  // Control methods (UNCHANGED - as requested)
  void _onJoystickChanged(double linear, double angular, bool deadmanActive) {
    if (!_isConnected) return;

    final clampedLinear = linear.clamp(-_maxLinearSpeed, _maxLinearSpeed);
    final clampedAngular = angular.clamp(-_maxAngularSpeed, _maxAngularSpeed);

    setState(() {
      _currentLinear = clampedLinear;
      _currentAngular = clampedAngular;
    });

    _webSocketService.sendJoystickControl(
      widget.deviceId,
      clampedAngular / _maxAngularSpeed,
      clampedLinear / _maxLinearSpeed,
      deadmanActive,
      maxLinearSpeed: _maxLinearSpeed,
      maxAngularSpeed: _maxAngularSpeed,
    );
  }

  void _toggleMapping() {
    if (_mappingActive) {
      _webSocketService.sendMappingCommand(widget.deviceId, 'stop');
    } else {
      _webSocketService.sendMappingCommand(widget.deviceId, 'start');
    }
  }

  Future<void> _saveMap() async {
    if (_currentMapData == null) {
      _showSnackBar('No map data to save', Colors.red);
      return;
    }

    try {
      _webSocketService.sendMappingCommand(widget.deviceId, 'save');
      final response = await _apiService.saveMapData(
        deviceId: widget.deviceId,
        mapData: _currentMapData!,
      );

      if (response['success'] == true) {
        _showSnackBar('Map saved successfully!', Colors.green);
      } else {
        _showSnackBar('Failed to save map: ${response['error']}', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error saving map: $e', Colors.red);
    }
  }

  Future<void> _saveTrailAsMap() async {
    if (_robotTrail.isEmpty) {
      _showSnackBar('No trail data to save', Colors.orange);
      return;
    }

    try {
      // Convert trail to map shape for editing
      final trailShape = MapShape(
        id: 'trail_${DateTime.now().millisecondsSinceEpoch}',
        name:
            'Robot Trail ${DateTime.now().toLocal().toString().split('.')[0]}',
        type: 'waypoint',
        points: List<odom.Position>.from(_robotTrail), // Use all trail points
        color: 'FF9800', // Orange color for trails
        sides: {
          'left': '',
          'right': '',
          'front': '',
          'back': '',
        },
        createdAt: DateTime.now(),
      );

      // Create or update map data with trail
      MapData mapToSave;
      if (_currentMapData != null) {
        mapToSave =
            _currentMapData!.addShape(trailShape); // Use built-in method
      } else {
        // Create new map with trail
        mapToSave = MapData(
          deviceId: widget.deviceId,
          timestamp: DateTime.now(),
          info: MapInfo(
            resolution: 0.05,
            width: 1000,
            height: 1000,
            origin: odom.Position(x: -25.0, y: -25.0, z: 0.0),
            originOrientation: odom.Orientation(x: 0, y: 0, z: 0, w: 1),
          ),
          occupancyData: List.filled(1000 * 1000, -1),
          shapes: [trailShape],
          version: 1,
        );
      }

      final response = await _apiService.saveMapData(
        deviceId: widget.deviceId,
        mapData: mapToSave,
      );

      if (response['success'] == true) {
        setState(() {
          _currentMapData = mapToSave;
        });
        _showSnackBar('Trail saved as waypoint path!', Colors.green);
      } else {
        _showSnackBar('Failed to save trail: ${response['error']}', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error saving trail: $e', Colors.red);
    }
  }

  void _openMapEditor() {
    Navigator.of(context).pushNamed(
      '/map',
      arguments: {'deviceId': widget.deviceId},
    ).then((_) {
      // Refresh map data when returning from editor
      if (_currentMapData != null) {
        _loadCurrentMapData();
      }
    });
  }

  Future<void> _loadCurrentMapData() async {
    try {
      final response = await _apiService.getMapData(widget.deviceId);
      if (response['success'] == true && response['mapData'] != null) {
        setState(() {
          _currentMapData = MapData.fromJson(response['mapData']);
        });
      }
    } catch (e) {
      debugPrint('❌ Error loading map data: $e');
    }
  }

  void _convertTrailToWaypoints() async {
    if (_robotTrail.isEmpty) {
      _showSnackBar('No trail data to convert', Colors.orange);
      return;
    }

    try {
      // Convert trail points to individual waypoints with simplified trail
      // Take every 10th point to avoid too many waypoints
      final simplifiedTrail = <odom.Position>[];
      for (int i = 0; i < _robotTrail.length; i += 10) {
        simplifiedTrail.add(_robotTrail[i]);
      }

      // Always include the last point
      if (_robotTrail.length > 1 &&
          simplifiedTrail.isNotEmpty &&
          simplifiedTrail.last != _robotTrail.last) {
        simplifiedTrail.add(_robotTrail.last);
      }

      final waypoints = <MapShape>[];
      for (int i = 0; i < simplifiedTrail.length; i++) {
        final point = simplifiedTrail[i];
        final waypoint = MapShape(
          id: 'waypoint_${DateTime.now().millisecondsSinceEpoch}_$i',
          name: 'Waypoint ${i + 1}',
          type: 'waypoint',
          points: [point], // Single point waypoint
          color: 'FF2196F3', // Blue color for waypoints
          sides: {
            'left': '',
            'right': '',
            'front': '',
            'back': '',
          },
          createdAt: DateTime.now(),
        );
        waypoints.add(waypoint);
      }

      // Create or update map data with waypoints
      MapData mapToSave;
      if (_currentMapData != null) {
        mapToSave = _currentMapData!;
        for (final waypoint in waypoints) {
          mapToSave = mapToSave.addShape(waypoint);
        }
      } else {
        // Create new map with waypoints
        mapToSave = MapData(
          deviceId: widget.deviceId,
          timestamp: DateTime.now(),
          info: MapInfo(
            resolution: 0.05,
            width: 1000,
            height: 1000,
            origin: odom.Position(x: -25.0, y: -25.0, z: 0.0),
            originOrientation: odom.Orientation(x: 0, y: 0, z: 0, w: 1),
          ),
          occupancyData: List.filled(1000 * 1000, -1),
          shapes: waypoints,
          version: 1,
        );
      }

      final response = await _apiService.saveMapData(
        deviceId: widget.deviceId,
        mapData: mapToSave,
      );

      if (response['success'] == true) {
        setState(() {
          _currentMapData = mapToSave;
        });
        _showSnackBar(
            'Trail converted to ${waypoints.length} waypoints!', Colors.green);
      } else {
        _showSnackBar(
            'Failed to save waypoints: ${response['error']}', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error converting trail: $e', Colors.red);
    }
  }

  void _toggleControl() {
    setState(() {
      _controlEnabled = !_controlEnabled;
    });

    if (!_controlEnabled) {
      _webSocketService.stopRobot(widget.deviceId);
      setState(() {
        _currentLinear = 0.0;
        _currentAngular = 0.0;
      });
    }
  }

  void _emergencyStop() {
    _webSocketService.stopRobot(widget.deviceId);
    setState(() {
      _currentLinear = 0.0;
      _currentAngular = 0.0;
    });
    _showSnackBar('Emergency stop activated!', Colors.red);
  }

  void _showSnackBar(String message, Color color) {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(message),
          backgroundColor: color,
          duration: const Duration(seconds: 2),
        ),
      );
    }
  }

  @override
  void dispose() {
    _realTimeDataSubscription?.cancel();
    _mappingEventsSubscription?.cancel();
    _controlEventsSubscription?.cancel();
    _connectionStateSubscription?.cancel();
    _errorSubscription?.cancel();
    _statusAnimationController.dispose();
    _tabController.dispose();
    super.dispose();
  }
}

// Enum for device types
enum DeviceType { phone, tablet, desktop }
