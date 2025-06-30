// screens/control_page.dart - FIXED Enhanced Control Page with Complete Save Functions
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
  MapData? _currentMapData;
  List<odom.Position> _robotTrail = [];
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
      length: 2,
      vsync: this,
    );
  }

  void _initializeConnection() {
    setState(() {
      _isConnected = _webSocketService.isConnected;
    });

    if (!_webSocketService.isConnected) {
      _webSocketService.connect('ws://192.168.0.113:3000',
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

    _realTimeDataSubscription = _webSocketService.realTimeData.listen((data) {
      if (mounted) {
        _handleRealTimeData(data);
      }
    });

    _mappingEventsSubscription = _webSocketService.mappingEvents.listen((data) {
      if (mounted) {
        _handleMappingEvent(data);
      }
    });

    _controlEventsSubscription = _webSocketService.controlEvents.listen((data) {
      if (mounted) {
        _handleControlEvent(data);
      }
    });

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
    final theme = Theme.of(context);

    return Scaffold(
      appBar: AppBar(
        title: Text('Live Control - ${widget.deviceId}'),
        backgroundColor: _mappingActive ? Colors.green : theme.primaryColor,
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
            Tab(icon: Icon(Icons.control_camera), text: 'Live Control'),
            Tab(icon: Icon(Icons.settings), text: 'Settings'),
          ],
        ),
      ),
      body: Column(
        children: [
          _buildEnhancedStatusBar(),
          Expanded(
            child: TabBarView(
              controller: _tabController,
              children: [
                _buildLiveMappingTab(deviceType),
                _buildSettingsTab(),
              ],
            ),
          ),
        ],
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: _emergencyStop,
        backgroundColor: Colors.red,
        child: const Icon(Icons.stop, color: Colors.white),
        tooltip: 'Emergency Stop',
      ),
    );
  }

  Widget _buildEnhancedStatusBar() {
    final theme = Theme.of(context);
    return Container(
      height: 50,
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            theme.primaryColor.withOpacity(0.1),
            theme.primaryColor.withOpacity(0.05),
          ],
        ),
        border: Border(
          bottom: BorderSide(color: theme.dividerColor),
        ),
      ),
      child: SingleChildScrollView(
        scrollDirection: Axis.horizontal,
        padding: const EdgeInsets.symmetric(horizontal: 16),
        child: Row(
          children: [
            _buildEnhancedStatusItem(
                Icons.location_on, _getPositionText(), Colors.blue),
            const SizedBox(width: 20),
            _buildEnhancedStatusItem(
                Icons.speed, _getVelocityText(), Colors.green),
            const SizedBox(width: 20),
            _buildEnhancedStatusItem(
                Icons.timeline, 'Trail: ${_robotTrail.length}', Colors.orange),
            const SizedBox(width: 20),
            _buildEnhancedStatusItem(
                Icons.layers,
                'Shapes: ${_currentMapData?.shapes.length ?? 0}',
                Colors.purple),
            const SizedBox(width: 20),
            _buildEnhancedStatusItem(
                Icons.message, 'Msgs: $_messagesReceived', Colors.indigo),
            // --- ADD THIS FOR MAP INFO ---
            if (_currentMapData != null) ...[
              const SizedBox(width: 20),
              _buildEnhancedStatusItem(
                Icons.map,
                'Map: ${_currentMapData!.info.width}x${_currentMapData!.info.height} @ ${_currentMapData!.info.resolution.toStringAsFixed(3)}m/px',
                Colors.blueGrey,
              ),
            ],
            // --- END ADD ---
            if (_globalCostmap != null) ...[
              const SizedBox(width: 20),
              _buildEnhancedStatusItem(Icons.public, 'Global', Colors.blue),
            ],
            if (_localCostmap != null) ...[
              const SizedBox(width: 20),
              _buildEnhancedStatusItem(Icons.near_me, 'Local', Colors.red),
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedStatusItem(IconData icon, String text, Color color) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: color.withOpacity(0.3)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 16, color: color),
          const SizedBox(width: 6),
          Text(
            text,
            style: TextStyle(
              fontSize: 12,
              color: color,
              fontWeight: FontWeight.w600,
            ),
          ),
        ],
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
        Expanded(
          flex: 6,
          child: Container(
            margin: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(12),
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.1),
                  blurRadius: 10,
                  offset: const Offset(0, 4),
                ),
              ],
            ),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(12),
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
        Expanded(
          flex: 4,
          child: Container(
            padding: const EdgeInsets.all(16),
            child: SingleChildScrollView(
              child: _buildEnhancedControlPanel(isCompact: false),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildTabletLayout() {
    return Row(
      children: [
        Expanded(
          flex: 55,
          child: Container(
            margin: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(12),
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.1),
                  blurRadius: 8,
                  offset: const Offset(0, 3),
                ),
              ],
            ),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(12),
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
        Expanded(
          flex: 45,
          child: Container(
            padding: const EdgeInsets.all(12),
            child: SingleChildScrollView(
              child: _buildEnhancedControlPanel(isCompact: true),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildPhoneLayout() {
    final screenHeight = MediaQuery.of(context).size.height;
    final mapHeight = screenHeight * 0.45;

    return Column(
      children: [
        Container(
          height: mapHeight,
          margin: const EdgeInsets.all(8),
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(12),
            boxShadow: [
              BoxShadow(
                color: Colors.black.withOpacity(0.1),
                blurRadius: 6,
                offset: const Offset(0, 2),
              ),
            ],
          ),
          child: ClipRRect(
            borderRadius: BorderRadius.circular(12),
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
        Expanded(
          child: SingleChildScrollView(
            padding: const EdgeInsets.symmetric(horizontal: 8),
            child: _buildEnhancedControlPanel(isCompact: true),
          ),
        ),
      ],
    );
  }

  Widget _buildEnhancedControlPanel({required bool isCompact}) {
    final screenWidth = MediaQuery.of(context).size.width;
    final joystickSize = isCompact
        ? math.min(screenWidth * 0.4, 200.0)
        : math.min(screenWidth * 0.3, 250.0);

    return Column(
      children: [
        _buildGradientSpeedControls(),
        SizedBox(height: isCompact ? 12 : 20),
        Center(
          child: Container(
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.1),
                  blurRadius: 15,
                  offset: const Offset(0, 5),
                ),
              ],
            ),
            child: JoystickWidget(
              size: joystickSize,
              maxLinearSpeed: _maxLinearSpeed,
              maxAngularSpeed: _maxAngularSpeed,
              enabled: _controlEnabled && _isConnected,
              requireDeadman: _useDeadmanSwitch,
              onChanged: _onJoystickChanged,
            ),
          ),
        ),
        SizedBox(height: isCompact ? 12 : 20),
        _buildEnhancedVelocityDisplay(),
        SizedBox(height: isCompact ? 12 : 20),
        _buildGradientControlButtons(),
        SizedBox(height: isCompact ? 12 : 20),
        _buildEnhancedMappingControls(),
        SizedBox(height: isCompact ? 12 : 20),
        _buildEnhancedMapOverlayControls(),
      ],
    );
  }

  Widget _buildGradientSpeedControls() {
    final theme = Theme.of(context);
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            theme.primaryColor.withOpacity(0.1),
            theme.primaryColor.withOpacity(0.05),
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: theme.primaryColor.withOpacity(0.2)),
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    color: theme.primaryColor,
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: const Icon(Icons.speed, color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                const Expanded(
                  child: Text(
                    'Speed Controls',
                    style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
                  ),
                ),
                TextButton.icon(
                  onPressed: () {
                    setState(() {
                      _showAdvancedSettings = !_showAdvancedSettings;
                    });
                  },
                  icon: Icon(_showAdvancedSettings
                      ? Icons.expand_less
                      : Icons.expand_more),
                  label: Text(_showAdvancedSettings ? 'Hide' : 'Adjust'),
                ),
              ],
            ),
            const SizedBox(height: 12),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                _buildEnhancedSpeedChip(
                    'Linear', _maxLinearSpeed, 'm/s', Colors.blue),
                _buildEnhancedSpeedChip(
                    'Angular', _maxAngularSpeed, 'rad/s', Colors.orange),
              ],
            ),
            if (_showAdvancedSettings) ...[
              const SizedBox(height: 16),
              _buildEnhancedSpeedSlider(
                'Linear Speed',
                _maxLinearSpeed,
                0.1,
                2.0,
                'm/s',
                (value) => setState(() => _maxLinearSpeed = value),
              ),
              const SizedBox(height: 8),
              _buildEnhancedSpeedSlider(
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

  Widget _buildEnhancedSpeedChip(
      String label, double value, String unit, Color color) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [color.withOpacity(0.8), color],
        ),
        borderRadius: BorderRadius.circular(20),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.3),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            padding: const EdgeInsets.all(4),
            decoration: const BoxDecoration(
              color: Colors.white,
              shape: BoxShape.circle,
            ),
            child: Text(
              label[0],
              style: TextStyle(
                color: color,
                fontSize: 12,
                fontWeight: FontWeight.bold,
              ),
            ),
          ),
          const SizedBox(width: 8),
          Text(
            '${value.toStringAsFixed(1)} $unit',
            style: const TextStyle(
              color: Colors.white,
              fontWeight: FontWeight.bold,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEnhancedSpeedSlider(
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
            Text(label,
                style:
                    const TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
              decoration: BoxDecoration(
                color: Theme.of(context).primaryColor,
                borderRadius: BorderRadius.circular(12),
              ),
              child: Text(
                '${value.toStringAsFixed(1)} $unit',
                style: const TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  fontSize: 12,
                ),
              ),
            ),
          ],
        ),
        SliderTheme(
          data: SliderTheme.of(context).copyWith(
            trackHeight: 6,
            thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 8),
            overlayShape: const RoundSliderOverlayShape(overlayRadius: 16),
          ),
          child: Slider(
            value: value,
            min: min,
            max: max,
            divisions: ((max - min) / 0.1).round(),
            onChanged: onChanged,
          ),
        ),
      ],
    );
  }

  Widget _buildEnhancedVelocityDisplay() {
    final theme = Theme.of(context);
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.grey.shade50,
            Colors.grey.shade100,
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: theme.dividerColor),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.05),
            blurRadius: 8,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            _buildEnhancedVelocityItem(
              'Linear',
              _currentLinear,
              _maxLinearSpeed,
              'm/s',
              Icons.arrow_upward,
              Colors.blue,
            ),
            Container(
              width: 2,
              height: 50,
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  begin: Alignment.topCenter,
                  end: Alignment.bottomCenter,
                  colors: [
                    Colors.transparent,
                    theme.dividerColor,
                    Colors.transparent,
                  ],
                ),
              ),
            ),
            _buildEnhancedVelocityItem(
              'Angular',
              _currentAngular,
              _maxAngularSpeed,
              'rad/s',
              Icons.rotate_right,
              Colors.orange,
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedVelocityItem(
    String label,
    double value,
    double maxValue,
    String unit,
    IconData icon,
    Color color,
  ) {
    final percentage = maxValue > 0 ? (value.abs() / maxValue * 100) : 0.0;
    final displayColor = value > 0
        ? color
        : value < 0
            ? color.withRed(255)
            : Colors.grey;

    return Column(
      children: [
        Container(
          padding: const EdgeInsets.all(8),
          decoration: BoxDecoration(
            color: displayColor.withOpacity(0.1),
            shape: BoxShape.circle,
          ),
          child: Icon(icon, color: displayColor, size: 24),
        ),
        const SizedBox(height: 8),
        Text(
          label,
          style: const TextStyle(fontSize: 12, fontWeight: FontWeight.w600),
        ),
        const SizedBox(height: 4),
        Text(
          '${value.toStringAsFixed(2)} $unit',
          style: TextStyle(
            fontWeight: FontWeight.bold,
            color: value != 0 ? displayColor : Colors.grey,
            fontSize: 16,
          ),
        ),
        const SizedBox(height: 4),
        Container(
          width: 60,
          height: 4,
          decoration: BoxDecoration(
            color: Colors.grey.shade300,
            borderRadius: BorderRadius.circular(2),
          ),
          child: FractionallySizedBox(
            alignment: Alignment.centerLeft,
            widthFactor: percentage / 100,
            child: Container(
              decoration: BoxDecoration(
                color: displayColor,
                borderRadius: BorderRadius.circular(2),
              ),
            ),
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

  Widget _buildGradientControlButtons() {
    return Row(
      children: [
        Expanded(
          child: Container(
            height: 50,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: _controlEnabled
                    ? [Colors.orange.shade400, Colors.orange.shade600]
                    : [Colors.green.shade400, Colors.green.shade600],
              ),
              borderRadius: BorderRadius.circular(25),
              boxShadow: [
                BoxShadow(
                  color: (_controlEnabled ? Colors.orange : Colors.green)
                      .withOpacity(0.3),
                  blurRadius: 8,
                  offset: const Offset(0, 4),
                ),
              ],
            ),
            child: ElevatedButton.icon(
              onPressed: _isConnected ? _toggleControl : null,
              icon: Icon(_controlEnabled ? Icons.pause : Icons.play_arrow),
              label: Text(_controlEnabled ? 'Disable' : 'Enable'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.transparent,
                shadowColor: Colors.transparent,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(25),
                ),
              ),
            ),
          ),
        ),
        const SizedBox(width: 12),
        Expanded(
          child: Container(
            height: 50,
            decoration: BoxDecoration(
              gradient: const LinearGradient(
                colors: [Colors.red, Colors.redAccent],
              ),
              borderRadius: BorderRadius.circular(25),
              boxShadow: [
                BoxShadow(
                  color: Colors.red.withOpacity(0.3),
                  blurRadius: 8,
                  offset: const Offset(0, 4),
                ),
              ],
            ),
            child: ElevatedButton.icon(
              onPressed: _isConnected ? _emergencyStop : null,
              icon: const Icon(Icons.stop),
              label: const Text('STOP'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.transparent,
                shadowColor: Colors.transparent,
                foregroundColor: Colors.white,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(25),
                ),
              ),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildEnhancedMappingControls() {
    if (!_isConnected) return const SizedBox.shrink();

    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.green.shade50,
            Colors.green.shade100,
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.green.withOpacity(0.3)),
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    color: Colors.green,
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: const Icon(Icons.map, color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                const Text(
                  'Mapping Controls',
                  style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
                ),
              ],
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: Container(
                    height: 45,
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        colors: _mappingActive
                            ? [Colors.red.shade400, Colors.red.shade600]
                            : [Colors.green.shade400, Colors.green.shade600],
                      ),
                      borderRadius: BorderRadius.circular(22.5),
                      boxShadow: [
                        BoxShadow(
                          color: (_mappingActive ? Colors.red : Colors.green)
                              .withOpacity(0.3),
                          blurRadius: 6,
                          offset: const Offset(0, 3),
                        ),
                      ],
                    ),
                    child: ElevatedButton.icon(
                      onPressed: () => _toggleMapping(),
                      icon:
                          Icon(_mappingActive ? Icons.stop : Icons.play_arrow),
                      label: Text(_mappingActive ? 'Stop' : 'Start'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.transparent,
                        shadowColor: Colors.transparent,
                        foregroundColor: Colors.white,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(22.5),
                        ),
                      ),
                    ),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: Container(
                    height: 45,
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        colors: [Colors.blue.shade400, Colors.blue.shade600],
                      ),
                      borderRadius: BorderRadius.circular(22.5),
                      boxShadow: [
                        BoxShadow(
                          color: Colors.blue.withOpacity(0.3),
                          blurRadius: 6,
                          offset: const Offset(0, 3),
                        ),
                      ],
                    ),
                    child: ElevatedButton.icon(
                      onPressed: _isMapSaveable() ? _saveMap : null,
                      icon: const Icon(Icons.save),
                      label: const Text('Save'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.transparent,
                        shadowColor: Colors.transparent,
                        foregroundColor: Colors.white,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(22.5),
                        ),
                      ),
                    ),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 12),
            Row(
              children: [
                Expanded(
                  child: Container(
                    height: 45,
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        colors: [
                          Colors.purple.shade400,
                          Colors.purple.shade600
                        ],
                      ),
                      borderRadius: BorderRadius.circular(22.5),
                      boxShadow: [
                        BoxShadow(
                          color: Colors.purple.withOpacity(0.3),
                          blurRadius: 6,
                          offset: const Offset(0, 3),
                        ),
                      ],
                    ),
                    child: ElevatedButton.icon(
                      onPressed: _isTrailSaveable() ? _saveTrailAsMap : null,
                      icon: const Icon(Icons.timeline),
                      label: const Text('Save Trail'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.transparent,
                        shadowColor: Colors.transparent,
                        foregroundColor: Colors.white,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(22.5),
                        ),
                      ),
                    ),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: Container(
                    height: 45,
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        colors: [
                          Colors.orange.shade400,
                          Colors.orange.shade600
                        ],
                      ),
                      borderRadius: BorderRadius.circular(22.5),
                      boxShadow: [
                        BoxShadow(
                          color: Colors.orange.withOpacity(0.3),
                          blurRadius: 6,
                          offset: const Offset(0, 3),
                        ),
                      ],
                    ),
                    child: ElevatedButton.icon(
                      onPressed: _openMapEditor,
                      icon: const Icon(Icons.edit),
                      label: const Text('Edit'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.transparent,
                        shadowColor: Colors.transparent,
                        foregroundColor: Colors.white,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(22.5),
                        ),
                      ),
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

  // ✅ FIXED: Helper methods to determine if save buttons should be enabled
  bool _isMapSaveable() {
    return _currentMapData != null || _mappingActive;
  }

  bool _isTrailSaveable() {
    return _robotTrail.isNotEmpty;
  }

  Widget _buildEnhancedMapOverlayControls() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.blue.shade50,
            Colors.blue.shade100,
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.blue.withOpacity(0.3)),
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    color: Colors.blue,
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child:
                      const Icon(Icons.layers, color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                const Text(
                  'Map Layers',
                  style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
                ),
              ],
            ),
            const SizedBox(height: 12),
            _buildEnhancedSwitchTile(
              'Occupancy Grid',
              'Show main SLAM map',
              _showOccupancyGrid,
              Colors.blue,
              (value) => setState(() => _showOccupancyGrid = value),
            ),
            _buildEnhancedSwitchTile(
              'Global Costmap',
              'Show global path planning layer',
              _showGlobalCostmap,
              Colors.green,
              (value) => setState(() => _showGlobalCostmap = value),
            ),
            _buildEnhancedSwitchTile(
              'Local Costmap',
              'Show local obstacle detection',
              _showLocalCostmap,
              Colors.red,
              (value) => setState(() => _showLocalCostmap = value),
            ),
            _buildEnhancedSwitchTile(
              'Robot Trail',
              'Show path history (${_robotTrail.length} points)',
              _showTrail,
              Colors.orange,
              (value) => setState(() => _showTrail = value),
            ),
            const SizedBox(height: 16),
            Container(
              padding: const EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: Colors.white.withOpacity(0.7),
                borderRadius: BorderRadius.circular(12),
              ),
              child: Column(
                children: [
                  Row(
                    children: [
                      const Icon(Icons.opacity, color: Colors.blue, size: 20),
                      const SizedBox(width: 8),
                      const Text('Layer Opacity: '),
                      const Spacer(),
                      Container(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 8, vertical: 4),
                        decoration: BoxDecoration(
                          color: Colors.blue,
                          borderRadius: BorderRadius.circular(12),
                        ),
                        child: Text(
                          '${(_costmapOpacity * 100).round()}%',
                          style: const TextStyle(
                            color: Colors.white,
                            fontWeight: FontWeight.bold,
                            fontSize: 12,
                          ),
                        ),
                      ),
                    ],
                  ),
                  SliderTheme(
                    data: SliderTheme.of(context).copyWith(
                      trackHeight: 6,
                      thumbShape:
                          const RoundSliderThumbShape(enabledThumbRadius: 8),
                      overlayShape:
                          const RoundSliderOverlayShape(overlayRadius: 16),
                    ),
                    child: Slider(
                      value: _costmapOpacity,
                      min: 0.1,
                      max: 1.0,
                      divisions: 9,
                      onChanged: (value) {
                        setState(() {
                          _costmapOpacity = value;
                        });
                      },
                    ),
                  ),
                ],
              ),
            ),
            if (_robotTrail.isNotEmpty) ...[
              const SizedBox(height: 12),
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
                      style: OutlinedButton.styleFrom(
                        side: const BorderSide(color: Colors.orange),
                        foregroundColor: Colors.orange,
                      ),
                    ),
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: OutlinedButton.icon(
                      onPressed: _convertTrailToWaypoints,
                      icon: const Icon(Icons.route),
                      label: const Text('To Waypoints'),
                      style: OutlinedButton.styleFrom(
                        side: const BorderSide(color: Colors.purple),
                        foregroundColor: Colors.purple,
                      ),
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

  Widget _buildEnhancedSwitchTile(
    String title,
    String subtitle,
    bool value,
    Color color,
    Function(bool) onChanged,
  ) {
    return Container(
      margin: const EdgeInsets.symmetric(vertical: 4),
      padding: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.7),
        borderRadius: BorderRadius.circular(12),
      ),
      child: Row(
        children: [
          Container(
            width: 4,
            height: 40,
            decoration: BoxDecoration(
              color: color,
              borderRadius: BorderRadius.circular(2),
            ),
          ),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: const TextStyle(
                    fontWeight: FontWeight.w600,
                    fontSize: 14,
                  ),
                ),
                Text(
                  subtitle,
                  style: TextStyle(
                    fontSize: 12,
                    color: Colors.grey.shade600,
                  ),
                ),
              ],
            ),
          ),
          Transform.scale(
            scale: 0.8,
            child: Switch(
              value: value,
              onChanged: onChanged,
              activeColor: color,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildSettingsTab() {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildSettingsSection(
            'Control Settings',
            Icons.control_camera,
            Colors.blue,
            [
              _buildSettingsSwitchTile(
                'Deadman Switch',
                'Require holding joystick to move',
                _useDeadmanSwitch,
                (value) => setState(() => _useDeadmanSwitch = value),
              ),
              _buildSettingsSwitchTile(
                'Auto Center Map',
                'Center map on robot position',
                _autoCenter,
                (value) => setState(() => _autoCenter = value),
              ),
            ],
          ),
          const SizedBox(height: 16),
          _buildSettingsSection(
            'Speed Settings',
            Icons.speed,
            Colors.orange,
            [
              _buildEnhancedSpeedSlider(
                'Max Linear Speed',
                _maxLinearSpeed,
                0.1,
                2.0,
                'm/s',
                (value) => setState(() => _maxLinearSpeed = value),
              ),
              const SizedBox(height: 16),
              _buildEnhancedSpeedSlider(
                'Max Angular Speed',
                _maxAngularSpeed,
                0.1,
                3.0,
                'rad/s',
                (value) => setState(() => _maxAngularSpeed = value),
              ),
            ],
          ),
          const SizedBox(height: 16),
          if (_currentMapData != null) ...[
            _buildSettingsSection(
              'Map Analysis',
              Icons.analytics,
              Colors.green,
              [_buildMapAnalysis(_currentMapData!)],
            ),
            const SizedBox(height: 16),
          ],
          _buildSettingsSection(
            'Status Information',
            Icons.info,
            Colors.purple,
            [_buildStatusInformation()],
          ),
        ],
      ),
    );
  }

  Widget _buildSettingsSection(
    String title,
    IconData icon,
    Color color,
    List<Widget> children,
  ) {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            color.withOpacity(0.1),
            color.withOpacity(0.05),
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: color.withOpacity(0.3)),
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    color: color,
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Icon(icon, color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                Text(
                  title,
                  style: const TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            ...children,
          ],
        ),
      ),
    );
  }

  Widget _buildSettingsSwitchTile(
    String title,
    String subtitle,
    bool value,
    Function(bool) onChanged,
  ) {
    return Container(
      margin: const EdgeInsets.symmetric(vertical: 4),
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.7),
        borderRadius: BorderRadius.circular(12),
      ),
      child: Row(
        children: [
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: const TextStyle(
                    fontWeight: FontWeight.w600,
                    fontSize: 16,
                  ),
                ),
                Text(
                  subtitle,
                  style: TextStyle(
                    fontSize: 14,
                    color: Colors.grey.shade600,
                  ),
                ),
              ],
            ),
          ),
          Switch(
            value: value,
            onChanged: onChanged,
          ),
        ],
      ),
    );
  }

  Widget _buildMapAnalysis(MapData mapData) {
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

    final mapAreaM2 = (mapData.info.width * mapData.info.resolution) *
        (mapData.info.height * mapData.info.resolution);

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
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
          const SizedBox(height: 12),
          const Divider(),
          const SizedBox(height: 8),
          const Text('Shape Summary:',
              style: TextStyle(fontWeight: FontWeight.w600)),
          const SizedBox(height: 8),
          ...shapesByType.entries.map(
            (entry) => _buildAnalysisRow(
              '${entry.key.toUpperCase()}',
              '${entry.value}',
              _getShapeTypeColor(entry.key),
            ),
          ),
        ],
        const SizedBox(height: 12),
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
    return Container(
      margin: const EdgeInsets.symmetric(vertical: 2),
      padding: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.7),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        children: [
          Container(
            width: 4,
            height: 20,
            decoration: BoxDecoration(
              color: color,
              borderRadius: BorderRadius.circular(2),
            ),
          ),
          const SizedBox(width: 12),
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

  Widget _buildStatusInformation() {
    return Column(
      children: [
        _buildStatusRow('Connected', _isConnected ? 'Yes' : 'No'),
        _buildStatusRow('Mapping Active', _mappingActive ? 'Yes' : 'No'),
        _buildStatusRow('Control Enabled', _controlEnabled ? 'Yes' : 'No'),
        _buildStatusRow('Messages Received', _messagesReceived.toString()),
        _buildStatusRow('Trail Points', _robotTrail.length.toString()),
        _buildStatusRow(
            'Map Shapes', _currentMapData?.shapes.length.toString() ?? '0'),
        _buildStatusRow(
            'Global Costmap', _globalCostmap != null ? 'Available' : 'N/A'),
        _buildStatusRow(
            'Local Costmap', _localCostmap != null ? 'Available' : 'N/A'),
        if (_currentMapData != null) ...[
          _buildStatusRow('Map Version', _currentMapData!.version.toString()),
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
    );
  }

  Widget _buildStatusRow(String label, String value) {
    return Container(
      margin: const EdgeInsets.symmetric(vertical: 2),
      padding: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.7),
        borderRadius: BorderRadius.circular(8),
      ),
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

  Widget _buildConnectionIndicator() {
    return Container(
      margin: const EdgeInsets.only(right: 8),
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: _isConnected
              ? [Colors.green.shade400, Colors.green.shade600]
              : [Colors.red.shade400, Colors.red.shade600],
        ),
        borderRadius: BorderRadius.circular(12),
        boxShadow: [
          BoxShadow(
            color: (_isConnected ? Colors.green : Colors.red).withOpacity(0.3),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
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
        gradient: const LinearGradient(
          colors: [Colors.green, Colors.lightGreen],
        ),
        borderRadius: BorderRadius.circular(12),
        boxShadow: [
          BoxShadow(
            color: Colors.green.withOpacity(0.3),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: const Row(
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
          SizedBox(width: 6),
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

  // ==========================================
  // ✅ FIXED: COMPLETE CONTROL METHODS
  // ==========================================

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
    if (!_isMapSaveable()) {
      _showSnackBar('No map data to save', Colors.red);
      return;
    }

    try {
      _webSocketService.sendMappingCommand(widget.deviceId, 'save');

      MapData? mapToSave = _currentMapData;
      if (mapToSave == null && _mappingActive) {
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
          shapes: [],
          version: 1,
        );
      }

      if (mapToSave != null) {
        final response = await _apiService.saveMapData(
          deviceId: widget.deviceId,
          mapData: mapToSave,
        );

        if (response['success'] == true) {
          _showSnackBar('Map saved successfully!', Colors.green);
        } else {
          _showSnackBar('Failed to save map: ${response['error']}', Colors.red);
        }
      }
    } catch (e) {
      _showSnackBar('Error saving map: $e', Colors.red);
    }
  }

  Future<void> _saveTrailAsMap() async {
    if (!_isTrailSaveable()) {
      _showSnackBar('No trail data to save', Colors.orange);
      return;
    }

    try {
      final trailShape = MapShape(
        id: 'trail_${DateTime.now().millisecondsSinceEpoch}',
        name:
            'Robot Trail ${DateTime.now().toLocal().toString().split('.')[0]}',
        type: 'waypoint',
        points: List<odom.Position>.from(_robotTrail),
        color: 'FF9800',
        sides: {'left': '', 'right': '', 'front': '', 'back': ''},
        createdAt: DateTime.now(),
      );

      MapData mapToSave;
      if (_currentMapData != null) {
        mapToSave = _currentMapData!.addShape(trailShape);
      } else {
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
      final simplifiedTrail = <odom.Position>[];
      for (int i = 0; i < _robotTrail.length; i += 10) {
        simplifiedTrail.add(_robotTrail[i]);
      }

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
          points: [point],
          color: 'FF2196F3',
          sides: {'left': '', 'right': '', 'front': '', 'back': ''},
          createdAt: DateTime.now(),
        );
        waypoints.add(waypoint);
      }

      MapData mapToSave;
      if (_currentMapData != null) {
        mapToSave = _currentMapData!;
        for (final waypoint in waypoints) {
          mapToSave = mapToSave.addShape(waypoint);
        }
      } else {
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
          behavior: SnackBarBehavior.floating,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(8),
          ),
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

enum DeviceType { phone, tablet, desktop }
