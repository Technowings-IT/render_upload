// screens/control_page.dart - Simplified Mobile Layout with Core Controls Only
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'dart:async';
import 'dart:math' as math;
import '../services/web_socket_service.dart';
import '../services/api_service.dart';
import '../services/theme_service.dart';
import '../widgets/joystick.dart';
import '../widgets/live_maping_canvas.dart';
import '../widgets/modern_ui_components.dart';
import '../models/map_data.dart';
import '../models/odom.dart' as odom;


enum DeviceType { phone, tablet, desktop }
enum SnackBarType { success, error, warning, info }

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
  // Services
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
  late AnimationController _cardAnimationController;
  late AnimationController _joystickGlowController;
  late TabController _tabController;

  // Connection state
  bool _isConnected = false;
  String _connectionStatus = 'Disconnected';

  // Robot control state
  bool _mappingActive = false;
  bool _controlEnabled = true;
  bool _robotControlActive = false;
  bool _scriptExecutionInProgress = false;

  // Script status tracking
  Map<String, String> _scriptStatus = {
    'robot_control': 'stopped',
    'slam': 'stopped',
    'navigation': 'stopped'
  };

  // Map and robot data
  MapData? _currentMapData;
  List<odom.Position> _robotTrail = [];
  odom.OdometryData? _currentOdometry;

  // Control settings
  double _maxLinearSpeed = 0.3;
  double _maxAngularSpeed = 0.8;
  bool _useDeadmanSwitch = true;
  bool _showTrail = true;
  bool _autoCenter = true;

  // Current movement state
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

  // UI state
  bool _showAdvancedSettings = false;
  String? _customMapName;

  // Status tracking
  DateTime? _lastPositionUpdate;
  DateTime? _lastMapUpdate;
  int _messagesReceived = 0;

  // Map zoom control variables
  double _mapZoomLevel = 1.0;
  final double _minZoom = 0.5;
  final double _maxZoom = 3.0;

  // Joystick touch control variables
  bool _joystickTouchActive = false;
  final GlobalKey _joystickKey = GlobalKey();

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _initializeConnection();
    _setupSubscriptions();
  }

@override
void dispose() {
  _realTimeDataSubscription?.cancel();
  _mappingEventsSubscription?.cancel();
  _controlEventsSubscription?.cancel();
  _connectionStateSubscription?.cancel();
  _errorSubscription?.cancel();
  _statusAnimationController.dispose();
  _cardAnimationController.dispose();
  _joystickGlowController.dispose();
  _tabController.dispose();
  super.dispose();
}

  // ==========================================
  // INITIALIZATION METHODS
  // ==========================================

  // Added: Modern AppBar builder to fix missing method error
  PreferredSizeWidget _buildModernAppBar(ThemeService theme) {
    return AppBar(
      title: Text('Live Control - ${widget.deviceId}'),
      backgroundColor: _mappingActive ? Colors.green : Theme.of(context).primaryColor,
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
    );
  }

void _initializeAnimations() {
  _statusAnimationController = AnimationController(
    duration: const Duration(milliseconds: 1000),
    vsync: this,
  );

  // NEW: Additional animation controllers
  _cardAnimationController = AnimationController(
    duration: const Duration(milliseconds: 800),
    vsync: this,
  );

  _joystickGlowController = AnimationController(
    duration: const Duration(milliseconds: 2000),
    vsync: this,
  );

  _tabController = TabController(
    length: 2,
    vsync: this,
  );

  // NEW: Start card animations
  _cardAnimationController.forward();
}

// NEW: Enhanced snackbar types

void _showModernSnackBar(String message, SnackBarType type) {
  final theme = Provider.of<ThemeService>(context, listen: false);
  Color backgroundColor;
  IconData icon;

  switch (type) {
    case SnackBarType.success:
      backgroundColor = theme.successColor;
      icon = Icons.check_circle;
      break;
    case SnackBarType.error:
      backgroundColor = theme.errorColor;
      icon = Icons.error;
      break;
    case SnackBarType.warning:
      backgroundColor = theme.warningColor;
      icon = Icons.warning;
      break;
    case SnackBarType.info:
      backgroundColor = theme.infoColor;
      icon = Icons.info;
      break;
  }

  ScaffoldMessenger.of(context).showSnackBar(
    SnackBar(
      content: Row(
        children: [
          Icon(icon, color: Colors.white, size: 20),
          const SizedBox(width: 12),
          Expanded(
            child: Text(
              message,
              style: theme.bodyMedium.copyWith(
                color: Colors.white,
                fontWeight: FontWeight.w500,
              ),
            ),
          ),
        ],
      ),
      backgroundColor: backgroundColor,
      behavior: SnackBarBehavior.floating,
      shape: RoundedRectangleBorder(
        borderRadius: theme.borderRadiusMedium,
      ),
      margin: const EdgeInsets.all(16),
      duration: const Duration(seconds: 3),
    ),
  );
}

// Removed duplicate build method to resolve "The name 'build' is already defined" error.
  void _initializeConnection() {
    setState(() {
      _isConnected = _webSocketService.isConnected;
    });

    if (!_webSocketService.isConnected) {
      _webSocketService.connect(
        'ws://192.168.0.63:3000',
        deviceId: widget.deviceId,
        deviceInfo: {
          'type': 'mobile_controller',
          'name': 'Flutter Control App'
        },
      ).then((connected) {
        if (connected) {
          _subscribeToTopics();
        }
      });
    } else {
      _subscribeToTopics();
    }
  }

void _setupSubscriptions() {
  _connectionStateSubscription = _webSocketService.connectionState.listen(
    (connected) {
      if (mounted) {
        setState(() {
          _isConnected = connected;
          _connectionStatus = connected ? 'Connected' : 'Disconnected';
        });

        if (connected) {
          _subscribeToTopics();
          _statusAnimationController.forward();
          // NEW: Joystick glow animation when connected
          _joystickGlowController.repeat(reverse: true);
        } else {
          _statusAnimationController.reverse();
          // NEW: Stop glow animation when disconnected
          _joystickGlowController.stop();
        }
      }
    },
  );

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
        _showModernSnackBar('WebSocket Error: $error', SnackBarType.error);
      }
    });
  }

  void _subscribeToTopics() {
    if (_isConnected) {
      _webSocketService.subscribe('real_time_data', deviceId: widget.deviceId);
      _webSocketService.subscribe('control_events', deviceId: widget.deviceId);
      _webSocketService.subscribe('mapping_events', deviceId: widget.deviceId);
      _webSocketService.subscribe('script_status', deviceId: widget.deviceId);
      _webSocketService.subscribe('ros2_status', deviceId: widget.deviceId);

      print('📡 Subscribed to all topics for device: ${widget.deviceId}');
    }
  }

  // ==========================================
  // DATA HANDLING METHODS
  // ==========================================

  void _handleRealTimeData(Map<String, dynamic> data) {
    final dataType = data['type'];
    final payload = data['data'];

    switch (dataType) {
      case 'position_update':
        _handlePositionUpdate(payload);
        break;
      case 'navigation_feedback_update':
        _handleNavigationFeedback(payload);
        break;
      case 'navigation_status_update':
        _handleNavigationStatus(payload);
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
      case 'script_status_update':
        _handleScriptStatusUpdate(data['data']);
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

  void _handleScriptStatusUpdate(Map<String, dynamic> data) {
    if (mounted) {
      setState(() {
        final scriptType = data['script'] ?? data['process'];
        final status = data['status'];

        if (scriptType != null && status != null) {
          _scriptStatus[scriptType] = status;

          // Update specific flags
          if (scriptType == 'robot_control') {
            _robotControlActive = status == 'running';
          } else if (scriptType == 'slam') {
            _mappingActive = status == 'running';
          }
        }
      });

      // Show user feedback
      final message = data['message'] ?? 'Script status updated';
      final success = data['success'] ?? true;
      _showSnackBar(message, success ? Colors.green : Colors.red);
    }
  }

  void _handleNavigationFeedback(Map<String, dynamic> data) {
    if (mounted) {
      // Show progress updates
      final timeRemaining = data['estimated_time_remaining'];
      final distance = data['distance_remaining'];
      final recoveries = data['number_of_recoveries'];

      if (timeRemaining != null && distance != null) {
        _showSnackBar(
          'Navigation: ${timeRemaining}s remaining, ${distance.toStringAsFixed(1)}m left, ${recoveries} recoveries',
          Colors.blue,
        );
      }
    }
  }

  void _handleNavigationStatus(Map<String, dynamic> data) {
    if (mounted) {
      final status = data['status_text'];
      final success = data['result']?['success'];

      if (status == 'SUCCEEDED') {
        _showSnackBar('Navigation completed successfully!', Colors.green);
      } else if (status == 'ABORTED' || status == 'REJECTED') {
        _showSnackBar(
          'Navigation failed: ${data['result']?['error_msg'] ?? 'Unknown error'}',
          Colors.red,
        );
      }
    }
  }

  // ==========================================
  // ROBOT TRAIL MANAGEMENT
  // ==========================================

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
    return math.sqrt(
      (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y),
    );
  }

  // ==========================================
  // MAP GRID CONTROL METHODS
  // ==========================================

  void _handleMapZoom(double zoomDelta) {
    setState(() {
      _mapZoomLevel = (_mapZoomLevel + zoomDelta).clamp(_minZoom, _maxZoom);
    });
  }

  void _resetMapView() {
    setState(() {
      _mapZoomLevel = 1.0;
    });
  }

  // 🔧 NEW: Zoom Responsive Mapping Canvas for Mobile
  Widget _buildZoomResponsiveMappingCanvas() {
    return Stack(
      children: [
        // Main map canvas - size adjusts with container
        Positioned.fill(
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

        // Map controls overlay - positioned relative to container
        Positioned(
          top: 8,
          right: 8,
          child: Column(
            children: [
              // Zoom in button
              _buildZoomControlButton(
                icon: Icons.zoom_in,
                onPressed:
                    _mapZoomLevel < _maxZoom ? () => _handleMapZoom(0.1) : null,
              ),
              const SizedBox(height: 4),

              // Zoom out button
              _buildZoomControlButton(
                icon: Icons.zoom_out,
                onPressed: _mapZoomLevel > _minZoom
                    ? () => _handleMapZoom(-0.1)
                    : null,
              ),
              const SizedBox(height: 4),

              // Reset zoom button
              _buildZoomControlButton(
                icon: Icons.center_focus_strong,
                onPressed: _resetMapView,
              ),
            ],
          ),
        ),

        // Zoom level indicator - positioned relative to container
        Positioned(
          top: 8,
          left: 8,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
            decoration: BoxDecoration(
              color: Colors.blue.withOpacity(0.9),
              borderRadius: BorderRadius.circular(12),
            ),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                const Icon(
                  Icons.zoom_in,
                  size: 14,
                  color: Colors.white,
                ),
                const SizedBox(width: 4),
                Text(
                  'Zoom: ${(_mapZoomLevel * 100).toInt()}%',
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 11,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
          ),
        ),

        // 🔧 NEW: Map size indicator
        Positioned(
          bottom: 8,
          left: 8,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
            decoration: BoxDecoration(
              color: Colors.green.withOpacity(0.9),
              borderRadius: BorderRadius.circular(12),
            ),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                const Icon(
                  Icons.aspect_ratio,
                  size: 14,
                  color: Colors.white,
                ),
                const SizedBox(width: 4),
                Text(
                  'Size: ${(_mapZoomLevel * 100).toInt()}%',
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 11,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  // 🔧 NEW: Helper method for zoom control buttons
  Widget _buildZoomControlButton({
    required IconData icon,
    required VoidCallback? onPressed,
  }) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.9),
        borderRadius: BorderRadius.circular(8),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: IconButton(
        onPressed: onPressed,
        icon: Icon(icon),
        iconSize: 20,
      ),
    );
  }

  // ==========================================
  // JOYSTICK TOUCH CONTROL METHODS
  // ==========================================

  void _onJoystickTouchStart() {
    setState(() {
      _joystickTouchActive = true;
    });
  }

  void _onJoystickTouchEnd() {
    setState(() {
      _joystickTouchActive = false;
    });
  }

  bool _shouldAbsorbTouchEvents() {
    return _joystickTouchActive;
  }

  // ==========================================
  // UTILITY METHODS
  // ==========================================

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

  String _getVelocityText() {
    return 'L: ${_currentLinear.toStringAsFixed(2)} m/s, A: ${_currentAngular.toStringAsFixed(2)} rad/s';
  }

  String _getPositionText() {
    if (_currentOdometry?.position != null) {
      final pos = _currentOdometry!.position;
      return '(${pos.x.toStringAsFixed(2)}, ${pos.y.toStringAsFixed(2)})';
    }
    return 'N/A';
  }

  String _getSessionDuration() {
    final now = DateTime.now();
    final startTime = DateTime.now().subtract(const Duration(minutes: 30));
    final duration = now.difference(startTime);

    if (duration.inHours > 0) {
      return '${duration.inHours}h ${duration.inMinutes % 60}m';
    } else {
      return '${duration.inMinutes}m';
    }
  }

  void _showSnackBar(String message, Color color) {
    final scaffoldMessenger = ScaffoldMessenger.of(context);
    scaffoldMessenger.showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: color,
        duration: const Duration(seconds: 2),
      ),
    );
  }

  // ==========================================
  // CONTROL METHODS
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

  void _toggleMapping() {
    if (_mappingActive) {
      _webSocketService.sendMappingCommand(widget.deviceId, 'stop');
    } else {
      _webSocketService.sendMappingCommand(widget.deviceId, 'start');
    }
  }

  Future<void> _emergencyStop() async {
    // Immediate robot stop
    _webSocketService.stopRobot(widget.deviceId);

    // Execute kill.sh for complete system shutdown
    _webSocketService.sendScriptCommand(widget.deviceId, 'stop_all_scripts');

    setState(() {
      _mappingActive = false;
      _robotControlActive = false;
      _currentLinear = 0.0;
      _currentAngular = 0.0;

      // Reset all script statuses
      _scriptStatus.forEach((key, value) {
        _scriptStatus[key] = 'stopped';
      });
    });

    _showSnackBar('EMERGENCY STOP! All systems killed via kill.sh', Colors.red);
  }

  // ==========================================
  // SCRIPT CONTROL METHODS
  // ==========================================

  Future<void> _startRobotControl() async {
    if (_scriptExecutionInProgress) {
      _showSnackBar('Another script operation is in progress', Colors.orange);
      return;
    }

    setState(() {
      _scriptExecutionInProgress = true;
    });

    try {
      _showSnackBar(
          'Starting complete robot system (Robot Control + SLAM + Navigation)...',
          Colors.blue);

      // Start Robot Control first
      final robotSuccess = _webSocketService.sendScriptCommand(
        widget.deviceId,
        'start_robot_control',
      );

      if (robotSuccess) {
        _showSnackBar('Robot control started, starting SLAM...', Colors.blue);

        // Wait a bit for robot control to initialize
        await Future.delayed(Duration(seconds: 2));

        // Start SLAM
        final slamSuccess = _webSocketService.sendScriptCommand(
            widget.deviceId, 'start_slam', options: {
          'mapName': 'slam_map_${DateTime.now().millisecondsSinceEpoch}'
        });

        if (slamSuccess) {
          _showSnackBar('SLAM started, starting navigation...', Colors.blue);

          // Wait a bit for SLAM to initialize
          await Future.delayed(Duration(seconds: 2));

          // Start navigation
          final navSuccess = _webSocketService.sendScriptCommand(
              widget.deviceId, 'start_navigation',
              options: {'mapPath': 'current_map.yaml'});

          if (navSuccess) {
            _showSnackBar(
                'Complete robot system started! Waiting for confirmation...',
                Colors.blue);

            // Wait for all systems to come online
            await Future.delayed(Duration(seconds: 5));

            if (_scriptStatus['robot_control'] == 'running' ||
                _scriptStatus['slam'] == 'running') {
              _showSnackBar('Robot system started successfully!', Colors.green);
            } else {
              _showSnackBar(
                  'System start commands sent but no confirmation received',
                  Colors.orange);
            }
          } else {
            _showSnackBar('Failed to start navigation', Colors.red);
          }
        } else {
          _showSnackBar('Failed to start SLAM', Colors.red);
        }
      } else {
        _showSnackBar('Failed to start robot control', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error starting robot system: $e', Colors.red);
    } finally {
      setState(() {
        _scriptExecutionInProgress = false;
      });
    }
  }

  Future<void> _startSLAM() async {
    if (_scriptExecutionInProgress) {
      _showSnackBar('Another script operation is in progress', Colors.orange);
      return;
    }

    setState(() {
      _scriptExecutionInProgress = true;
    });

    try {
      _showSnackBar('Starting SLAM mapping system...', Colors.blue);

      final success = _webSocketService.sendScriptCommand(
          widget.deviceId, 'start_slam', options: {
        'mapName': 'slam_map_${DateTime.now().millisecondsSinceEpoch}'
      });

      if (success) {
        _showSnackBar(
            'SLAM command sent. System will auto-start robot control if needed...',
            Colors.blue);

        // Wait for SLAM to initialize
        await Future.delayed(Duration(seconds: 5));

        if (_scriptStatus['slam'] == 'running') {
          _showSnackBar('SLAM mapping started successfully!', Colors.green);
        } else {
          _showSnackBar(
              'SLAM command sent but no confirmation received', Colors.orange);
        }
      } else {
        _showSnackBar('Failed to send SLAM command', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error starting SLAM: $e', Colors.red);
    } finally {
      setState(() {
        _scriptExecutionInProgress = false;
      });
    }
  }

  Future<void> _startNavigation() async {
    if (_scriptExecutionInProgress) {
      _showSnackBar('Another script operation is in progress', Colors.orange);
      return;
    }

    setState(() {
      _scriptExecutionInProgress = true;
    });

    try {
      _showSnackBar('Starting navigation system...', Colors.blue);

      final success = _webSocketService.sendScriptCommand(
          widget.deviceId, 'start_navigation',
          options: {'mapPath': 'current_map.yaml'});

      if (success) {
        _showSnackBar(
            'Navigation command sent. System will auto-start robot control if needed...',
            Colors.blue);

        // Wait for navigation to initialize
        await Future.delayed(Duration(seconds: 5));

        if (_scriptStatus['navigation'] == 'running') {
          _showSnackBar('Navigation started successfully!', Colors.green);
        } else {
          _showSnackBar('Navigation command sent but no confirmation received',
              Colors.orange);
        }
      } else {
        _showSnackBar('Failed to send navigation command', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error starting navigation: $e', Colors.red);
    } finally {
      setState(() {
        _scriptExecutionInProgress = false;
      });
    }
  }

  Future<void> _stopAllScripts() async {
    if (_scriptExecutionInProgress) {
      _showSnackBar('Another script operation is in progress', Colors.orange);
      return;
    }

    setState(() {
      _scriptExecutionInProgress = true;
    });

    try {
      _showSnackBar(
          'Executing kill.sh to stop all processes...', Colors.orange);

      // Use the kill.sh script on Raspberry Pi to stop everything
      final success = _webSocketService.sendScriptCommand(
        widget.deviceId,
        'stop_all_scripts', // This will execute kill.sh
      );

      if (success) {
        _showSnackBar(
            'Kill script executed. Stopping all processes...', Colors.orange);

        // Wait for kill.sh to complete its work
        await Future.delayed(Duration(seconds: 8));

        // Update UI state to reflect all processes stopped
        setState(() {
          _scriptStatus.forEach((key, value) {
            _scriptStatus[key] = 'stopped';
          });
          _robotControlActive = false;
          _mappingActive = false;
          _currentLinear = 0.0;
          _currentAngular = 0.0;
        });

        _showSnackBar(
            'All processes killed successfully via kill.sh!', Colors.green);
      } else {
        _showSnackBar('Failed to execute kill.sh script', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error executing kill script: $e', Colors.red);
    } finally {
      setState(() {
        _scriptExecutionInProgress = false;
      });
    }
  }

  Future<void> _stopRobotControl() async {
    if (_scriptExecutionInProgress) {
      _showSnackBar('Another script operation is in progress', Colors.orange);
      return;
    }

    setState(() {
      _scriptExecutionInProgress = true;
    });

    try {
      final success = _webSocketService.sendScriptCommand(
        widget.deviceId,
        'stop_robot_control',
      );

      if (success) {
        _showSnackBar('Stopping robot control...', Colors.orange);
      } else {
        _showSnackBar('Failed to send stop command', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error stopping robot control: $e', Colors.red);
    } finally {
      setState(() {
        _scriptExecutionInProgress = false;
      });
    }
  }

  Future<void> _stopSLAM() async {
    if (_scriptExecutionInProgress) {
      _showSnackBar('Another script operation is in progress', Colors.orange);
      return;
    }

    setState(() {
      _scriptExecutionInProgress = true;
    });

    try {
      final success = _webSocketService.sendScriptCommand(
        widget.deviceId,
        'stop_slam',
      );

      if (success) {
        _showSnackBar('Stopping SLAM...', Colors.orange);
      } else {
        _showSnackBar('Failed to send stop command', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error stopping SLAM: $e', Colors.red);
    } finally {
      setState(() {
        _scriptExecutionInProgress = false;
      });
    }
  }

  Future<void> _stopNavigation() async {
    if (_scriptExecutionInProgress) {
      _showSnackBar('Another script operation is in progress', Colors.orange);
      return;
    }

    setState(() {
      _scriptExecutionInProgress = true;
    });

    try {
      final success = _webSocketService.sendScriptCommand(
        widget.deviceId,
        'stop_navigation',
      );

      if (success) {
        _showSnackBar('Stopping navigation...', Colors.orange);
      } else {
        _showSnackBar('Failed to send stop command', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error stopping navigation: $e', Colors.red);
    } finally {
      setState(() {
        _scriptExecutionInProgress = false;
      });
    }
  }

  // ==========================================
  // MAP SAVING METHODS
  // ==========================================

  bool _isTrailSaveable() {
    return _robotTrail.isNotEmpty;
  }

  bool _isMapSaveable() {
    return _currentMapData != null || _mappingActive;
  }

  bool _isCompleteMappingSaveable() {
    return _currentMapData != null ||
        _mappingActive ||
        _robotTrail.isNotEmpty ||
        _currentOdometry != null;
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
        mapToSave = _createEmptyMapData();
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
        mapToSave = _createEmptyMapData().addShape(trailShape);
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

  Future<void> _saveCompleteMap() async {
    if (!_isCompleteMappingSaveable()) {
      _showSnackBar('No complete map data available to save', Colors.red);
      return;
    }

    try {
      // Show save options dialog with ROS2 option
      _showEnhancedSaveOptionsDialog();
    } catch (e) {
      _showSnackBar('Error showing save options: $e', Colors.red);
    }
  }

  // NEW: Enhanced save options dialog with ROS2 option
  void _showEnhancedSaveOptionsDialog() {
    final TextEditingController nameController = TextEditingController();
    bool saveViaROS2 = true; // Default to ROS2 save
    bool includeTrail = _robotTrail.isNotEmpty;
    bool includePosition = _currentOdometry != null;
    String selectedDirectory = '/tmp/saved_maps';

    showDialog(
      context: context,
      builder: (context) => StatefulBuilder(
        builder: (context, setState) => AlertDialog(
          title: const Row(
            children: [
              Icon(Icons.save_alt, color: Colors.blue),
              SizedBox(width: 8),
              Text('Enhanced Save Options'),
            ],
          ),
          content: SingleChildScrollView(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                TextField(
                  controller: nameController,
                  decoration: const InputDecoration(
                    labelText: 'Map Name',
                    hintText: 'Enter map name (required for ROS2)',
                    border: OutlineInputBorder(),
                    prefixIcon: Icon(Icons.label),
                  ),
                  onChanged: (value) {
                    _customMapName = value.isEmpty ? null : value;
                  },
                ),
                const SizedBox(height: 16),

                // Save method selection
                Container(
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.blue.shade50,
                    borderRadius: BorderRadius.circular(8),
                    border: Border.all(color: Colors.blue.shade200),
                  ),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      const Text(
                        'Save Method:',
                        style: TextStyle(fontWeight: FontWeight.bold),
                      ),
                      RadioListTile<bool>(
                        title: const Text('ROS2 Map Saver (Recommended)'),
                        subtitle: const Text(
                            'Uses ros2 run nav2_map_server map_saver_cli'),
                        value: true,
                        groupValue: saveViaROS2,
                        onChanged: (value) =>
                            setState(() => saveViaROS2 = value!),
                        dense: true,
                      ),
                      RadioListTile<bool>(
                        title: const Text('Local JSON Storage'),
                        subtitle: const Text('Saves to app internal storage'),
                        value: false,
                        groupValue: saveViaROS2,
                        onChanged: (value) =>
                            setState(() => saveViaROS2 = value!),
                        dense: true,
                      ),
                    ],
                  ),
                ),

                const SizedBox(height: 16),

                // ROS2 specific options
                if (saveViaROS2) ...[
                  DropdownButtonFormField<String>(
                    value: selectedDirectory,
                    decoration: const InputDecoration(
                      labelText: 'Save Directory on Pi',
                      border: OutlineInputBorder(),
                      prefixIcon: Icon(Icons.folder),
                    ),
                    items: const [
                      DropdownMenuItem(
                          value: '/tmp/saved_maps',
                          child: Text('/tmp/saved_maps')),
                      DropdownMenuItem(
                          value: '/home/piros/maps',
                          child: Text('/home/piros/maps')),
                      DropdownMenuItem(
                          value: '/opt/ros/maps', child: Text('/opt/ros/maps')),
                    ],
                    onChanged: (value) =>
                        setState(() => selectedDirectory = value!),
                  ),
                  const SizedBox(height: 16),
                ],

                // Include options
                const Text(
                  'Include in Save:',
                  style: TextStyle(fontWeight: FontWeight.bold),
                ),
                CheckboxListTile(
                  title: const Text('Robot Trail'),
                  subtitle: Text('${_robotTrail.length} waypoints'),
                  value: includeTrail,
                  onChanged: _robotTrail.isNotEmpty
                      ? (value) => setState(() => includeTrail = value!)
                      : null,
                  dense: true,
                ),
                CheckboxListTile(
                  title: const Text('Current Position'),
                  subtitle: const Text('Robot location marker'),
                  value: includePosition,
                  onChanged: _currentOdometry != null
                      ? (value) => setState(() => includePosition = value!)
                      : null,
                  dense: true,
                ),
              ],
            ),
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: const Text('Cancel'),
            ),
            ElevatedButton.icon(
              onPressed: nameController.text.isNotEmpty || !saveViaROS2
                  ? () {
                      Navigator.of(context).pop();
                      if (saveViaROS2) {
                        _saveMapViaROS2(
                          mapName: nameController.text,
                          directory: selectedDirectory,
                          includeTrail: includeTrail,
                          includePosition: includePosition,
                        );
                      } else {
                        _saveCompleteMapTraditional();
                      }
                    }
                  : null,
              icon: Icon(saveViaROS2 ? Icons.rocket_launch : Icons.save),
              label: Text(saveViaROS2 ? 'Save via ROS2' : 'Save Locally'),
              style: ElevatedButton.styleFrom(
                backgroundColor: saveViaROS2 ? Colors.green : Colors.blue,
                foregroundColor: Colors.white,
              ),
            ),
          ],
        ),
      ),
    );
  }

  // NEW: Save map via ROS2 command
  Future<void> _saveMapViaROS2({
    required String mapName,
    required String directory,
    bool includeTrail = false,
    bool includePosition = false,
  }) async {
    try {
      // Show progress dialog
      showDialog(
        context: context,
        barrierDismissible: false,
        builder: (context) => AlertDialog(
          title: const Row(
            children: [
              CircularProgressIndicator(strokeWidth: 2),
              SizedBox(width: 16),
              Text('Saving via ROS2'),
            ],
          ),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text('Executing ROS2 map saver command...'),
              const SizedBox(height: 8),
              Text(
                  'Command: ros2 run nav2_map_server map_saver_cli -f $mapName'),
              Text('Directory: $directory'),
              const SizedBox(height: 16),
              const LinearProgressIndicator(),
            ],
          ),
        ),
      );

      // Execute ROS2 save via API
      final response = await _apiService.saveMapViaROS2(
        deviceId: widget.deviceId,
        mapName: mapName,
        directory: directory,
        includeTimestamp: true,
      );

      // Close progress dialog
      Navigator.of(context).pop();

      if (response['success'] == true) {
        _showROS2SaveSuccessDialog(response);
      } else {
        _showSnackBar('ROS2 save failed: ${response['error']}', Colors.red);
      }
    } catch (e) {
      // Close progress dialog
      Navigator.of(context).pop();
      _showSnackBar('Error saving via ROS2: $e', Colors.red);
    }
  }

  // NEW: Show ROS2 save success dialog with map loading option
  void _showROS2SaveSuccessDialog(Map<String, dynamic> response) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Row(
          children: [
            Icon(Icons.check_circle, color: Colors.green, size: 28),
            SizedBox(width: 12),
            Text('ROS2 Save Successful'),
          ],
        ),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Map saved successfully via ROS2 command!',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 16),
            const Text('Save Details:'),
            const SizedBox(height: 8),
            Text('Device: ${widget.deviceId}'),
            Text('Map Name: ${response['mapName']}'),
            Text('Directory: ${response['directory']}'),
            Text('Command: ${response['command']}'),
            Text(
                'Files: ${response['files']?.keys?.join(', ') ?? 'PGM + YAML'}'),
            Text('Saved At: ${response['savedAt']}'),
            const SizedBox(height: 16),
            Container(
              padding: const EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: Colors.blue.shade50,
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.blue.shade200),
              ),
              child: const Text(
                'The map is now saved on the Raspberry Pi and can be loaded for editing or navigation.',
                style: TextStyle(fontSize: 12),
              ),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: const Text('OK'),
          ),
          ElevatedButton.icon(
            onPressed: () {
              Navigator.of(context).pop();
              _navigateToMapEditorWithROS2Map(response['mapName']);
            },
            icon: const Icon(Icons.edit),
            label: const Text('Load & Edit'),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.purple,
              foregroundColor: Colors.white,
            ),
          ),
        ],
      ),
    );
  }

  // NEW: Navigate to map editor with ROS2 saved map
  void _navigateToMapEditorWithROS2Map(String mapName) {
    Navigator.pushNamed(
      context,
      '/map',
      arguments: {
        'deviceId': widget.deviceId,
        'loadROS2Map': true,
        'ros2MapName': mapName,
        'editMode': true,
      },
    );
  }

  // Keep existing traditional save method
  Future<void> _saveCompleteMapTraditional() async {
    try {
      final completeMapData = await _collectCompleteMapData();

      final response = await _apiService.saveCompleteMapData(
        deviceId: widget.deviceId,
        mapData: completeMapData,
        mapName: _customMapName,
      );

      if (response['success'] == true) {
        setState(() {
          _currentMapData = completeMapData;
        });
        _showSnackBar('Complete map saved successfully!', Colors.green);
        _showMapSaveSuccess(response);
      } else {
        _showSnackBar('Failed to save map: ${response['error']}', Colors.red);
      }
    } catch (e) {
      _showSnackBar('Error saving complete map: $e', Colors.red);
    }
  }

  Future<MapData> _collectCompleteMapData() async {
    MapData baseMapData = _currentMapData ?? _createEmptyMapData();

    // Add robot trail as waypoint path if available
    if (_robotTrail.isNotEmpty) {
      final trailShape = MapShape(
        id: 'robot_trail_${DateTime.now().millisecondsSinceEpoch}',
        name: 'Robot Trail Path',
        type: 'waypoint',
        points: List<odom.Position>.from(_robotTrail),
        color: 'FF9800',
        sides: {'left': '', 'right': '', 'front': '', 'back': ''},
        createdAt: DateTime.now(),
      );
      baseMapData = baseMapData.addShape(trailShape);
    }

    // Add current robot position as reference point
    if (_currentOdometry?.position != null) {
      final robotPosShape = MapShape(
        id: 'robot_position_${DateTime.now().millisecondsSinceEpoch}',
        name: 'Robot Current Position',
        type: 'waypoint',
        points: [_currentOdometry!.position],
        color: 'FF4CAF50',
        sides: {'left': '', 'right': '', 'front': '', 'back': ''},
        createdAt: DateTime.now(),
      );
      baseMapData = baseMapData.addShape(robotPosShape);
    }

    // Add session metadata
    final sessionMetadata = MapShape(
      id: 'session_metadata_${DateTime.now().millisecondsSinceEpoch}',
      name: 'Mapping Session Info',
      type: 'metadata',
      points: [],
      color: 'FF9C27B0',
      sides: {
        'mapping_active': _mappingActive.toString(),
        'trail_points': _robotTrail.length.toString(),
        'messages_received': _messagesReceived.toString(),
        'session_duration': _getSessionDuration(),
        'has_global_costmap': (_globalCostmap != null).toString(),
        'has_local_costmap': (_localCostmap != null).toString(),
      },
      createdAt: DateTime.now(),
    );

    return baseMapData.addShape(sessionMetadata);
  }

  MapData _createEmptyMapData() {
    return MapData(
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

  // ==========================================
  // NAVIGATION METHODS
  // ==========================================

  void _navigateToMapEditor() {
    Navigator.pushNamed(
      context,
      '/map',
      arguments: {
        'deviceId': widget.deviceId,
        'mapData': _currentMapData,
        'editMode': true,
      },
    );
  }

  // ==========================================
  // UI BUILDING METHODS
  // ==========================================

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
        heroTag: "emergency_stop",
        tooltip: 'Emergency Stop',
        child: const Icon(Icons.stop, color: Colors.white),
      ),
    );
  }

  Widget _buildConnectionIndicator() {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 12),
      child: Row(
        children: [
          Icon(
            _isConnected ? Icons.wifi : Icons.wifi_off,
            color: _isConnected ? Colors.green : Colors.red,
            size: 22,
          ),
          const SizedBox(width: 4),
          Text(
            _isConnected ? 'Online' : 'Offline',
            style: TextStyle(
              color: _isConnected ? Colors.green : Colors.red,
              fontWeight: FontWeight.bold,
              fontSize: 13,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMappingIndicator() {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 8),
      child: Row(
        children: [
          const Icon(Icons.map, color: Colors.green, size: 22),
          const SizedBox(width: 4),
          Text(
            'Mapping',
            style: TextStyle(
              color: Colors.green[700],
              fontWeight: FontWeight.bold,
              fontSize: 13,
            ),
          ),
        ],
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
              Icons.location_on,
              _getPositionText(),
              Colors.blue,
            ),
            const SizedBox(width: 20),
            _buildEnhancedStatusItem(
              Icons.speed,
              _getVelocityText(),
              Colors.green,
            ),
            const SizedBox(width: 20),
            _buildEnhancedStatusItem(
              Icons.timeline,
              'Trail: ${_robotTrail.length}',
              Colors.orange,
            ),
            const SizedBox(width: 20),
            _buildEnhancedStatusItem(
              Icons.layers,
              'Shapes: ${_currentMapData?.shapes.length ?? 0}',
              Colors.purple,
            ),
            const SizedBox(width: 20),
            _buildEnhancedStatusItem(
              Icons.message,
              'Msgs: $_messagesReceived',
              Colors.indigo,
            ),
            const SizedBox(width: 20),
            _buildEnhancedStatusItem(
              Icons.zoom_in,
              'Zoom: ${(_mapZoomLevel * 100).toInt()}%',
              Colors.teal,
            ),
            if (_currentMapData != null) ...[
              const SizedBox(width: 20),
              _buildEnhancedStatusItem(
                Icons.map,
                'Map: ${_currentMapData!.info.width}x${_currentMapData!.info.height} @ ${_currentMapData!.info.resolution.toStringAsFixed(3)}m/px',
                Colors.blueGrey,
              ),
            ],
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
        return _buildSimplifiedPhoneLayout(); // 🔧 NEW: Simplified mobile layout
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
              child: _buildFixedLiveMappingCanvas(),
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
              child: _buildFixedLiveMappingCanvas(),
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

  // 🔧 NEW: Simplified Phone Layout - Only 3 Core Components
  Widget _buildSimplifiedPhoneLayout() {
    final screenHeight = MediaQuery.of(context).size.height;
    final screenWidth = MediaQuery.of(context).size.width;

    // 🔧 NEW: Dynamic map height based on zoom level
    final baseMapHeight = screenHeight * 0.55;
    final dynamicMapHeight = baseMapHeight * _mapZoomLevel;
    final maxMapHeight = screenHeight * 0.75; // Cap maximum height
    final finalMapHeight = math.min(dynamicMapHeight, maxMapHeight);

    final joystickSize = math.min(screenWidth * 0.4, 180.0); // Compact joystick

    return Column(
      children: [
        // 1. MAP - Dynamic size based on zoom level
        Container(
          height: finalMapHeight,
          width: screenWidth - 16, // Full width minus margins
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
            child: _buildZoomResponsiveMappingCanvas(),
          ),
        ),

        // 2. SPEED CONTROLS - Compact version
        Expanded(
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 12),
            child: Column(
              children: [
                _buildCompactSpeedControls(),
                const SizedBox(height: 12),

                // 3. JOYSTICK - Centered and compact
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
                    child: AbsorbPointer(
                      absorbing: _shouldAbsorbTouchEvents(),
                      child: Listener(
                        onPointerDown: (_) => _onJoystickTouchStart(),
                        onPointerUp: (_) => _onJoystickTouchEnd(),
                        onPointerCancel: (_) => _onJoystickTouchEnd(),
                        child: Container(
                          key: _joystickKey,
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
                    ),
                  ),
                ),

                const SizedBox(height: 8),

                // Connection status indicator (minimal)
                if (!_isConnected)
                  Container(
                    padding:
                        const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                    decoration: BoxDecoration(
                      color: Colors.red.withOpacity(0.1),
                      borderRadius: BorderRadius.circular(16),
                      border: Border.all(color: Colors.red.withOpacity(0.3)),
                    ),
                    child: Row(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        const Icon(Icons.wifi_off, size: 16, color: Colors.red),
                        const SizedBox(width: 6),
                        const Text(
                          'Disconnected - Check Settings',
                          style: TextStyle(
                            fontSize: 12,
                            color: Colors.red,
                            fontWeight: FontWeight.w600,
                          ),
                        ),
                      ],
                    ),
                  ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  // 🔧 NEW: Compact Speed Controls for Mobile
  Widget _buildCompactSpeedControls() {
    final theme = Theme.of(context);
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            theme.primaryColor.withOpacity(0.1),
            theme.primaryColor.withOpacity(0.05),
          ],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: theme.primaryColor.withOpacity(0.2)),
      ),
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          children: [
            Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(6),
                  decoration: BoxDecoration(
                    color: theme.primaryColor,
                    borderRadius: BorderRadius.circular(6),
                  ),
                  child: const Icon(Icons.speed, color: Colors.white, size: 16),
                ),
                const SizedBox(width: 8),
                const Text(
                  'Speed Controls',
                  style: TextStyle(fontWeight: FontWeight.bold, fontSize: 14),
                ),
                const Spacer(),
                TextButton.icon(
                  onPressed: () {
                    setState(() {
                      _showAdvancedSettings = !_showAdvancedSettings;
                    });
                  },
                  icon: Icon(
                      _showAdvancedSettings ? Icons.expand_less : Icons.tune,
                      size: 16),
                  label: Text(_showAdvancedSettings ? 'Hide' : 'Adjust',
                      style: const TextStyle(fontSize: 12)),
                ),
              ],
            ),
            const SizedBox(height: 8),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                _buildCompactSpeedChip(
                    'Linear', _maxLinearSpeed, 'm/s', Colors.blue),
                _buildCompactSpeedChip(
                    'Angular', _maxAngularSpeed, 'rad/s', Colors.orange),
              ],
            ),
            if (_showAdvancedSettings) ...[
              const SizedBox(height: 12),
              _buildCompactSpeedSlider(
                'Linear',
                _maxLinearSpeed,
                0.1,
                2.0,
                'm/s',
                (value) => setState(() => _maxLinearSpeed = value),
              ),
              const SizedBox(height: 8),
              _buildCompactSpeedSlider(
                'Angular',
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

  // 🔧 NEW: Compact Speed Chip for Mobile
  Widget _buildCompactSpeedChip(
      String label, double value, String unit, Color color) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [color.withOpacity(0.8), color],
        ),
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.3),
            blurRadius: 3,
            offset: const Offset(0, 1),
          ),
        ],
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            padding: const EdgeInsets.all(3),
            decoration: const BoxDecoration(
              color: Colors.white,
              shape: BoxShape.circle,
            ),
            child: Text(
              label[0],
              style: TextStyle(
                color: color,
                fontSize: 10,
                fontWeight: FontWeight.bold,
              ),
            ),
          ),
          const SizedBox(width: 6),
          Text(
            '${value.toStringAsFixed(1)} $unit',
            style: const TextStyle(
              color: Colors.white,
              fontWeight: FontWeight.bold,
              fontSize: 12,
            ),
          ),
        ],
      ),
    );
  }

  // 🔧 NEW: Compact Speed Slider for Mobile
  Widget _buildCompactSpeedSlider(
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
            Text(
              label,
              style: const TextStyle(
                fontSize: 12,
                fontWeight: FontWeight.w600,
              ),
            ),
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
              decoration: BoxDecoration(
                color: Theme.of(context).primaryColor,
                borderRadius: BorderRadius.circular(8),
              ),
              child: Text(
                '${value.toStringAsFixed(1)} $unit',
                style: const TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  fontSize: 10,
                ),
              ),
            ),
          ],
        ),
        SliderTheme(
          data: SliderTheme.of(context).copyWith(
            trackHeight: 4,
            thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 6),
            overlayShape: const RoundSliderOverlayShape(overlayRadius: 12),
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

  // Fixed Live Mapping Canvas with zoom controls (Desktop/Tablet)
  Widget _buildFixedLiveMappingCanvas() {
    return Stack(
      children: [
        // Main map canvas with transform scale
        Transform.scale(
          scale: _mapZoomLevel,
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

        // Map controls overlay
        Positioned(
          top: 8,
          right: 8,
          child: Column(
            children: [
              // Zoom in button
              _buildZoomControlButton(
                icon: Icons.zoom_in,
                onPressed:
                    _mapZoomLevel < _maxZoom ? () => _handleMapZoom(0.1) : null,
              ),
              const SizedBox(height: 4),

              // Zoom out button
              _buildZoomControlButton(
                icon: Icons.zoom_out,
                onPressed: _mapZoomLevel > _minZoom
                    ? () => _handleMapZoom(-0.1)
                    : null,
              ),
              const SizedBox(height: 4),

              // Reset zoom button
              _buildZoomControlButton(
                icon: Icons.center_focus_strong,
                onPressed: _resetMapView,
              ),
            ],
          ),
        ),

        // Zoom level indicator
        Positioned(
          top: 8,
          left: 8,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
            decoration: BoxDecoration(
              color: Colors.blue.withOpacity(0.9),
              borderRadius: BorderRadius.circular(12),
            ),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                const Icon(
                  Icons.zoom_in,
                  size: 14,
                  color: Colors.white,
                ),
                const SizedBox(width: 4),
                Text(
                  'Zoom: ${(_mapZoomLevel * 100).toInt()}%',
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 11,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  // Enhanced Control Panel for Desktop/Tablet (unchanged)
  Widget _buildEnhancedControlPanel({required bool isCompact}) {
    final screenWidth = MediaQuery.of(context).size.width;
    final joystickSize = isCompact
        ? math.min(screenWidth * 0.4, 200.0)
        : math.min(screenWidth * 0.3, 250.0);

    return Column(
      children: [
        _buildGradientSpeedControls(),
        SizedBox(height: isCompact ? 12 : 20),

        // Enhanced joystick with touch absorption
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
            child: AbsorbPointer(
              absorbing: _shouldAbsorbTouchEvents(),
              child: Listener(
                onPointerDown: (_) => _onJoystickTouchStart(),
                onPointerUp: (_) => _onJoystickTouchEnd(),
                onPointerCancel: (_) => _onJoystickTouchEnd(),
                child: Container(
                  key: _joystickKey,
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
            Text(
              label,
              style: const TextStyle(
                fontSize: 14,
                fontWeight: FontWeight.w600,
              ),
            ),
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

    return Column(
      children: [
        // Existing robot controls container
        Container(
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
                // Header
                Row(
                  children: [
                    Container(
                      padding: const EdgeInsets.all(8),
                      decoration: BoxDecoration(
                        color: Colors.green,
                        borderRadius: BorderRadius.circular(8),
                      ),
                      child: const Icon(
                        Icons.smart_toy,
                        color: Colors.white,
                        size: 20,
                      ),
                    ),
                    const SizedBox(width: 12),
                    const Text(
                      'Robot & Mapping Controls',
                      style:
                          TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
                    ),
                    const Spacer(),
                    if (_scriptExecutionInProgress)
                      const SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(strokeWidth: 2),
                      ),
                  ],
                ),
                const SizedBox(height: 16),

                // Script Status Indicators
                Row(
                  children: [
                    _buildScriptStatusChip(
                        'Robot', _scriptStatus['robot_control'] ?? 'stopped'),
                    const SizedBox(width: 8),
                    _buildScriptStatusChip(
                        'SLAM', _scriptStatus['slam'] ?? 'stopped'),
                    const SizedBox(width: 8),
                    _buildScriptStatusChip(
                        'Nav', _scriptStatus['navigation'] ?? 'stopped'),
                  ],
                ),
                const SizedBox(height: 16),

                // Control Buttons
                _buildMappingControlButtons(),
              ],
            ),
          ),
        ),

        const SizedBox(height: 16),

        // Add a dedicated Save section
        Container(
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
                // Header
                Row(
                  children: [
                    Container(
                      padding: const EdgeInsets.all(8),
                      decoration: BoxDecoration(
                        color: Colors.blue,
                        borderRadius: BorderRadius.circular(8),
                      ),
                      child: const Icon(
                        Icons.save,
                        color: Colors.white,
                        size: 20,
                      ),
                    ),
                    const SizedBox(width: 12),
                    const Text(
                      'Save Map Data',
                      style:
                          TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
                    ),
                  ],
                ),
                const SizedBox(height: 16),

                // Save status indicators
                Row(
                  children: [
                    _buildSaveStatusChip(
                        'Trail', _isTrailSaveable(), _robotTrail.length),
                    const SizedBox(width: 8),
                    _buildSaveStatusChip('Map', _isMapSaveable(),
                        _currentMapData?.shapes.length ?? 0),
                    const SizedBox(width: 8),
                    _buildSaveStatusChip(
                        'Complete',
                        _isCompleteMappingSaveable(),
                        (_robotTrail.length +
                            (_currentMapData?.shapes.length ?? 0))),
                  ],
                ),
                const SizedBox(height: 16),

                // Save buttons
                _buildSaveButtons(),
              ],
            ),
          ),
        ),
      ],
    );
  }

  // Helper method for save status chips
  Widget _buildSaveStatusChip(String label, bool isAvailable, int count) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        color: isAvailable
            ? Colors.green.withOpacity(0.1)
            : Colors.grey.withOpacity(0.1),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(
          color: isAvailable
              ? Colors.green.withOpacity(0.3)
              : Colors.grey.withOpacity(0.3),
        ),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(
            isAvailable ? Icons.check_circle : Icons.cancel,
            size: 14,
            color: isAvailable ? Colors.green : Colors.grey,
          ),
          const SizedBox(width: 4),
          Text(
            '$label ($count)',
            style: TextStyle(
              fontSize: 11,
              fontWeight: FontWeight.bold,
              color: isAvailable ? Colors.green : Colors.grey,
            ),
          ),
        ],
      ),
    );
  }

  // Save buttons row
  Widget _buildSaveButtons() {
    return Column(
      children: [
        Row(
          children: [
            Expanded(
              child: ElevatedButton.icon(
                onPressed:
                    _isCompleteMappingSaveable() ? _saveCompleteMap : null,
                icon: const Icon(Icons.save_alt),
                label: const Text('Save Complete Map'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: _isCompleteMappingSaveable()
                      ? Colors.blue
                      : Colors.grey.shade300,
                  foregroundColor: Colors.white,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(12),
                  ),
                ),
              ),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _showSaveOptionsDialog,
                icon: const Icon(Icons.tune),
                label: const Text('Advanced Options'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.orange,
                  foregroundColor: Colors.white,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(12),
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
              child: OutlinedButton.icon(
                onPressed: _isTrailSaveable() ? _saveTrailAsMap : null,
                icon: const Icon(Icons.timeline),
                label: const Text('Save Trail Only'),
                style: OutlinedButton.styleFrom(
                  side: BorderSide(
                    color: _isTrailSaveable()
                        ? Colors.green.shade600
                        : Colors.grey.shade400,
                  ),
                  foregroundColor: _isTrailSaveable()
                      ? Colors.green.shade600
                      : Colors.grey.shade400,
                ),
              ),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: OutlinedButton.icon(
                onPressed: _isMapSaveable() ? _saveMap : null,
                icon: const Icon(Icons.map),
                label: const Text('Save Map Only'),
                style: OutlinedButton.styleFrom(
                  side: BorderSide(
                    color: _isMapSaveable()
                        ? Colors.indigo.shade600
                        : Colors.grey.shade400,
                  ),
                  foregroundColor: _isMapSaveable()
                      ? Colors.indigo.shade600
                      : Colors.grey.shade400,
                ),
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildScriptStatusChip(String label, String status) {
    Color statusColor;
    IconData statusIcon;

    switch (status) {
      case 'running':
        statusColor = Colors.green;
        statusIcon = Icons.play_circle_filled;
        break;
      case 'stopped':
        statusColor = Colors.grey;
        statusIcon = Icons.stop_circle;
        break;
      case 'starting':
        statusColor = Colors.orange;
        statusIcon = Icons.hourglass_empty;
        break;
      case 'error':
        statusColor = Colors.red;
        statusIcon = Icons.error;
        break;
      default:
        statusColor = Colors.grey;
        statusIcon = Icons.help;
    }

    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        color: statusColor.withOpacity(0.1),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: statusColor.withOpacity(0.3)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(statusIcon, size: 14, color: statusColor),
          const SizedBox(width: 4),
          Text(
            label,
            style: TextStyle(
              fontSize: 11,
              fontWeight: FontWeight.bold,
              color: statusColor,
            ),
          ),
        ],
      ),
    );
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
                  'Map Display Settings',
                  style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
                ),
              ],
            ),
            const SizedBox(height: 16),
            _buildMapOverlaySwitches(),
            const SizedBox(height: 12),
            _buildCostmapOpacitySlider(),
          ],
        ),
      ),
    );
  }

  Widget _buildMapOverlaySwitches() {
    return Column(
      children: [
        SwitchListTile(
          title: const Text('Show Robot Trail'),
          subtitle: Text('${_robotTrail.length} points'),
          value: _showTrail,
          onChanged: (value) {
            setState(() {
              _showTrail = value;
            });
          },
          dense: true,
        ),
        SwitchListTile(
          title: const Text('Show Occupancy Grid'),
          subtitle: const Text('SLAM map data'),
          value: _showOccupancyGrid,
          onChanged: (value) {
            setState(() {
              _showOccupancyGrid = value;
            });
          },
          dense: true,
        ),
        SwitchListTile(
          title: const Text('Show Global Costmap'),
          subtitle: const Text('Path planning overlay'),
          value: _showGlobalCostmap,
          onChanged: _globalCostmap != null
              ? (value) {
                  setState(() {
                    _showGlobalCostmap = value;
                  });
                }
              : null,
          dense: true,
        ),
        SwitchListTile(
          title: const Text('Show Local Costmap'),
          subtitle: const Text('Dynamic obstacles'),
          value: _showLocalCostmap,
          onChanged: _localCostmap != null
              ? (value) {
                  setState(() {
                    _showLocalCostmap = value;
                  });
                }
              : null,
          dense: true,
        ),
      ],
    );
  }

  Widget _buildCostmapOpacitySlider() {
    return Column(
      children: [
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            const Text(
              'Costmap Opacity',
              style: TextStyle(fontSize: 12),
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
    );
  }

  // 🔧 NEW: Comprehensive Settings Tab with All Moved Controls
  Widget _buildSettingsTab() {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // ROBOT CONTROL SECTION
          _buildSettingsSection(
            'Robot Control Settings',
            Icons.smart_toy,
            Colors.green,
            [
              _buildControlSettings(),
              const SizedBox(height: 16),
              _buildVelocityDisplaySection(),
              const SizedBox(height: 16),
              _buildControlButtonsSection(),
            ],
          ),

          const SizedBox(height: 24),

          // ROBOT & MAPPING CONTROLS SECTION
          _buildSettingsSection(
            'Robot & Mapping Controls',
            Icons.precision_manufacturing,
            Colors.orange,
            [
              if (_isConnected) _buildFullMappingControls(),
              if (!_isConnected) _buildConnectionRequiredMessage(),
            ],
          ),

          const SizedBox(height: 24),

          // SAVE MAP DATA SECTION
          _buildSettingsSection(
            'Save Map Data',
            Icons.save_alt,
            Colors.blue,
            [
              _buildFullSaveControls(),
            ],
          ),

          const SizedBox(height: 24),

          // MAP DISPLAY SECTION
          _buildSettingsSection(
            'Map Display Settings',
            Icons.layers,
            Colors.purple,
            [
              _buildMapSettings(),
            ],
          ),

          const SizedBox(height: 24),

          // CONNECTION SECTION
          _buildSettingsSection(
            'Connection & Statistics',
            Icons.wifi,
            Colors.indigo,
            [
              _buildConnectionSettings(),
            ],
          ),
        ],
      ),
    );
  }

  // Helper method to build consistent settings sections
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
            color.withOpacity(0.05),
            color.withOpacity(0.1),
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
                    fontWeight: FontWeight.bold,
                    fontSize: 18,
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

  Widget _buildControlSettings() {
    return Column(
      children: [
        SwitchListTile(
          title: const Text('Use Deadman Switch'),
          subtitle: const Text('Require continuous press for movement'),
          value: _useDeadmanSwitch,
          onChanged: (value) {
            setState(() {
              _useDeadmanSwitch = value;
            });
          },
        ),
        SwitchListTile(
          title: const Text('Auto Center Map'),
          subtitle: const Text('Keep robot centered in view'),
          value: _autoCenter,
          onChanged: (value) {
            setState(() {
              _autoCenter = value;
            });
          },
        ),
        const Divider(),
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
    );
  }

  // NEW: Velocity Display Section for Settings
  Widget _buildVelocityDisplaySection() {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.grey.shade50,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.grey.shade300),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Text(
            'Current Velocity',
            style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
          ),
          const SizedBox(height: 12),
          _buildEnhancedVelocityDisplay(),
        ],
      ),
    );
  }

  // NEW: Control Buttons Section for Settings
  Widget _buildControlButtonsSection() {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.grey.shade50,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.grey.shade300),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Text(
            'Control Actions',
            style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
          ),
          const SizedBox(height: 12),
          _buildGradientControlButtons(),
        ],
      ),
    );
  }

  // NEW: Full Mapping Controls for Settings
  Widget _buildFullMappingControls() {
    return Column(
      children: [
        // Script Status Indicators
        const Text(
          'System Status',
          style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
        ),
        const SizedBox(height: 8),
        Row(
          children: [
            _buildScriptStatusChip(
                'Robot', _scriptStatus['robot_control'] ?? 'stopped'),
            const SizedBox(width: 8),
            _buildScriptStatusChip('SLAM', _scriptStatus['slam'] ?? 'stopped'),
            const SizedBox(width: 8),
            _buildScriptStatusChip(
                'Nav', _scriptStatus['navigation'] ?? 'stopped'),
          ],
        ),

        if (_scriptExecutionInProgress) ...[
          const SizedBox(height: 12),
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Colors.orange.shade50,
              borderRadius: BorderRadius.circular(8),
              border: Border.all(color: Colors.orange.shade200),
            ),
            child: const Row(
              children: [
                SizedBox(
                  width: 16,
                  height: 16,
                  child: CircularProgressIndicator(strokeWidth: 2),
                ),
                SizedBox(width: 12),
                Text(
                  'Script operation in progress...',
                  style: TextStyle(
                    fontWeight: FontWeight.w600,
                    color: Colors.orange,
                  ),
                ),
              ],
            ),
          ),
        ],

        const SizedBox(height: 16),
        const Text(
          'System Control',
          style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
        ),
        const SizedBox(height: 12),

        // Control Buttons
        _buildMappingControlButtons(),
      ],
    );
  }

  // NEW: Connection Required Message
  Widget _buildConnectionRequiredMessage() {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.red.shade50,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.red.shade200),
      ),
      child: const Row(
        children: [
          Icon(Icons.wifi_off, color: Colors.red, size: 24),
          SizedBox(width: 12),
          Expanded(
            child: Text(
              'Robot control requires an active connection. Please check your connection settings.',
              style: TextStyle(
                color: Colors.red,
                fontWeight: FontWeight.w600,
              ),
            ),
          ),
        ],
      ),
    );
  }

  // NEW: Full Save Controls for Settings
  Widget _buildFullSaveControls() {
    return Column(
      children: [
        // Save status indicators
        const Text(
          'Data Available',
          style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
        ),
        const SizedBox(height: 8),
        Row(
          children: [
            _buildSaveStatusChip(
                'Trail', _isTrailSaveable(), _robotTrail.length),
            const SizedBox(width: 8),
            _buildSaveStatusChip(
                'Map', _isMapSaveable(), _currentMapData?.shapes.length ?? 0),
            const SizedBox(width: 8),
            _buildSaveStatusChip('Complete', _isCompleteMappingSaveable(),
                (_robotTrail.length + (_currentMapData?.shapes.length ?? 0))),
          ],
        ),
        const SizedBox(height: 16),

        const Text(
          'Save Options',
          style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
        ),
        const SizedBox(height: 12),

        // Save buttons
        _buildSaveButtons(),
      ],
    );
  }

  Widget _buildMapSettings() {
    return Column(
      children: [
        _buildCostmapOpacitySlider(),
        const Divider(),
        const Text(
          'Display Options',
          style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
        ),
        const SizedBox(height: 8),
        _buildMapOverlaySwitches(),
      ],
    );
  }

  Widget _buildConnectionSettings() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        ListTile(
          leading: Icon(
            _isConnected ? Icons.wifi : Icons.wifi_off,
            color: _isConnected ? Colors.green : Colors.red,
          ),
          title: Text('Connection Status: $_connectionStatus'),
          subtitle: Text('Device ID: ${widget.deviceId}'),
          contentPadding: EdgeInsets.zero,
        ),
        const Divider(),
        const Text(
          'Statistics',
          style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
        ),
        const SizedBox(height: 8),
        Container(
          padding: const EdgeInsets.all(12),
          decoration: BoxDecoration(
            color: Colors.grey.shade50,
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: Colors.grey.shade300),
          ),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text('Messages Received: $_messagesReceived'),
              const SizedBox(height: 4),
              Text('Robot Trail Points: ${_robotTrail.length}'),
              const SizedBox(height: 4),
              Text('Map Shapes: ${_currentMapData?.shapes.length ?? 0}'),
              const SizedBox(height: 4),
              if (_lastPositionUpdate != null)
                Text(
                    'Last Position Update: ${_lastPositionUpdate!.toLocal().toString().split('.')[0]}'),
              const SizedBox(height: 4),
              if (_lastMapUpdate != null)
                Text(
                    'Last Map Update: ${_lastMapUpdate!.toLocal().toString().split('.')[0]}'),
              const SizedBox(height: 4),
              Text('Session Duration: ${_getSessionDuration()}'),
            ],
          ),
        ),
      ],
    );
  }

  // ==========================================
  // MAPPING CONTROL BUTTONS
  // ==========================================

  Widget _buildMappingControlButtons() {
    return Column(
      children: [
        // Main control button - Start Complete System
        Container(
          width: double.infinity,
          margin: const EdgeInsets.only(bottom: 12),
          child: ElevatedButton.icon(
            onPressed: !_scriptExecutionInProgress
                ? (_robotControlActive ||
                        _mappingActive ||
                        _scriptStatus['navigation'] == 'running')
                    ? null // Disable if any system is running
                    : _startRobotControl // Start complete system
                : null,
            icon: const Icon(Icons.rocket_launch, size: 24),
            label: const Text(
              'START COMPLETE SYSTEM\n(Robot → SLAM → Navigation)',
              textAlign: TextAlign.center,
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.green,
              foregroundColor: Colors.white,
              padding: const EdgeInsets.all(16),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(12),
              ),
            ),
          ),
        ),

        // Stop All button with kill.sh
        Container(
          width: double.infinity,
          margin: const EdgeInsets.only(bottom: 12),
          child: ElevatedButton.icon(
            onPressed: !_scriptExecutionInProgress
                ? (_robotControlActive ||
                        _mappingActive ||
                        _scriptStatus['navigation'] == 'running')
                    ? _stopAllScripts // Stop all if any system is running
                    : null // Disable if nothing is running
                : null,
            icon: const Icon(Icons.stop_circle, size: 24),
            label: const Text(
              'STOP ALL (kill.sh)',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.red,
              foregroundColor: Colors.white,
              padding: const EdgeInsets.all(16),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(12),
              ),
            ),
          ),
        ),

        // Individual control buttons
        Row(
          children: [
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _robotControlActive
                    ? _stopRobotControl
                    : _startRobotControl,
                icon:
                    Icon(_robotControlActive ? Icons.pause : Icons.play_arrow),
                label: Text(_robotControlActive ? 'Stop Robot' : 'Start Robot'),
                style: ElevatedButton.styleFrom(
                  backgroundColor:
                      _robotControlActive ? Colors.orange : Colors.blue,
                  foregroundColor: Colors.white,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(12),
                  ),
                ),
              ),
            ),
            const SizedBox(width: 8),
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _mappingActive ? _stopSLAM : _startSLAM,
                icon: Icon(_mappingActive ? Icons.stop : Icons.play_arrow),
                label: Text(_mappingActive ? 'Stop SLAM' : 'Start SLAM'),
                style: ElevatedButton.styleFrom(
                  backgroundColor:
                      _mappingActive ? Colors.orange : Colors.purple,
                  foregroundColor: Colors.white,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(12),
                  ),
                ),
              ),
            ),
            const SizedBox(width: 8),
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _scriptStatus['navigation'] == 'running'
                    ? _stopNavigation
                    : _startNavigation,
                icon: Icon(_scriptStatus['navigation'] == 'running'
                    ? Icons.stop
                    : Icons.play_arrow),
                label: Text(_scriptStatus['navigation'] == 'running'
                    ? 'Stop Nav'
                    : 'Start Nav'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: _scriptStatus['navigation'] == 'running'
                      ? Colors.orange
                      : Colors.indigo,
                  foregroundColor: Colors.white,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(12),
                  ),
                ),
              ),
            ),
          ],
        ),
      ],
    );
  }

  // ==========================================
  // DIALOG METHODS
  // ==========================================

  void _showMapSaveSuccess(Map<String, dynamic> response) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Row(
          children: [
            Icon(Icons.check_circle, color: Colors.green, size: 28),
            SizedBox(width: 12),
            Text('Map Saved Successfully'),
          ],
        ),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Map Details:',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 8),
            Text('Device: ${widget.deviceId}'),
            Text('Map Name: ${response['mapName'] ?? 'Auto-generated'}'),
            Text('Shapes: ${_currentMapData?.shapes.length ?? 0}'),
            Text('Trail Points: ${_robotTrail.length}'),
            Text('File Size: ${response['fileSize'] ?? 'Unknown'}'),
            Text(
                'Saved At: ${DateTime.now().toLocal().toString().split('.')[0]}'),
            const SizedBox(height: 16),
            const Text(
                'The map is now available in the Dashboard > Maps tab for editing.'),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: const Text('OK'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _navigateToMapEditor();
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.purple,
              foregroundColor: Colors.white,
            ),
            child: const Text('Edit Now'),
          ),
        ],
      ),
    );
  }

  void _showSaveOptionsDialog() {
    final TextEditingController nameController = TextEditingController();
    bool includeTrail = _robotTrail.isNotEmpty;
    bool includePosition = _currentOdometry != null;
    bool includeCostmaps = _globalCostmap != null || _localCostmap != null;
    bool includeMetadata = true;

    showDialog(
      context: context,
      builder: (context) => StatefulBuilder(
        builder: (context, setState) => AlertDialog(
          title: const Text('Advanced Save Options'),
          content: SingleChildScrollView(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                TextField(
                  controller: nameController,
                  decoration: const InputDecoration(
                    labelText: 'Custom Map Name (Optional)',
                    hintText: 'Leave empty for auto-generated name',
                    border: OutlineInputBorder(),
                  ),
                  onChanged: (value) {
                    _customMapName = value.isEmpty ? null : value;
                  },
                ),
                const SizedBox(height: 16),
                CheckboxListTile(
                  title: const Text('Include Occupancy Grid'),
                  subtitle: const Text('SLAM-generated map data'),
                  value: true,
                  onChanged: null,
                  dense: true,
                ),
                CheckboxListTile(
                  title: const Text('Include Robot Trail'),
                  subtitle: Text('${_robotTrail.length} waypoints'),
                  value: includeTrail,
                  onChanged: _robotTrail.isNotEmpty
                      ? (value) => setState(() => includeTrail = value!)
                      : null,
                  dense: true,
                ),
                CheckboxListTile(
                  title: const Text('Include Current Position'),
                  subtitle: const Text('Robot location marker'),
                  value: includePosition,
                  onChanged: _currentOdometry != null
                      ? (value) => setState(() => includePosition = value!)
                      : null,
                  dense: true,
                ),
                CheckboxListTile(
                  title: const Text('Include Costmap Data'),
                  subtitle: const Text('Path planning layers'),
                  value: includeCostmaps,
                  onChanged: (_globalCostmap != null || _localCostmap != null)
                      ? (value) => setState(() => includeCostmaps = value!)
                      : null,
                  dense: true,
                ),
                CheckboxListTile(
                  title: const Text('Include Session Metadata'),
                  subtitle: const Text('Mapping session info'),
                  value: includeMetadata,
                  onChanged: (value) =>
                      setState(() => includeMetadata = value!),
                  dense: true,
                ),
              ],
            ),
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: const Text('Cancel'),
            ),
            ElevatedButton(
              onPressed: () {
                Navigator.of(context).pop();
                _saveCompleteMap();
              },
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.blue,
                foregroundColor: Colors.white,
              ),
              child: const Text('Save Complete Map'),
            ),
          ],
        ),
      ),
    );
  }
}
