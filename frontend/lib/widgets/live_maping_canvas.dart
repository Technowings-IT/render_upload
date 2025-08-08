// widgets/enhanced_live_mapping_canvas.dart - FIXED Map Display with ROS Colors
import 'package:flutter/material.dart';
import 'dart:math' as math;
import 'dart:async';
import '../models/map_data.dart';
import '../models/odom.dart';

class LiveMappingCanvas extends StatefulWidget {
  final MapData? mapData;
  final OdometryData? currentOdometry;
  final List<Position> robotTrail;
  final bool mappingActive;
  final String deviceId;
  final Map<String, dynamic>? globalCostmap;
  final Map<String, dynamic>? localCostmap;
  final bool showOccupancyGrid;
  final double costmapOpacity;
  final Function(MapData)? onMapChanged;
  final VoidCallback? onTrailSaved;

  const LiveMappingCanvas({
    Key? key,
    this.mapData,
    this.currentOdometry,
    this.robotTrail = const [],
    this.mappingActive = false,
    required this.deviceId,
    this.globalCostmap,
    this.localCostmap,
    this.showOccupancyGrid = true,
    this.costmapOpacity = 0.7,
    this.onMapChanged,
    this.onTrailSaved,
  }) : super(key: key);

  @override
  State<LiveMappingCanvas> createState() => _LiveMappingCanvasState();
}

class _LiveMappingCanvasState extends State<LiveMappingCanvas>
    with TickerProviderStateMixin {
  late TransformationController _transformationController;
  late AnimationController _robotAnimationController;
  late AnimationController _centeringAnimationController;

  // ✅ Keep original coordinate system
  double _scale = 1.5;
  Offset _translation = Offset.zero;

  // Auto-centering configuration
  bool _autoCenter = true;
  bool _aggressiveAutoCenter =
      false; // NEW: Option for more aggressive centering
  double _autoCenterThreshold = 0.1; // NEW: Configurable movement threshold
  Duration _userInteractionCooldown =
      Duration(seconds: 2); // NEW: Configurable cooldown

  // Auto-centering state tracking
  Position? _lastRobotPosition;
  DateTime? _lastUserInteraction;
  DateTime? _lastAutoCenter;
  bool _isUserPanning = false;
  bool _isUserZooming = false;
  bool _autoCenterPaused = false; // NEW: Manual pause state
  Timer? _userInteractionTimer;

  // ✅ Keep original visual settings
  static const double _gridSpacing = 20.0;
  static const double _robotDisplaySize = 12.0;
  static const double _trailWidth = 2.5;
  static const double _movementThreshold = 1.5; // Scale threshold for movement

  @override
  void initState() {
    super.initState();
    _transformationController = TransformationController();

    _robotAnimationController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );

    _centeringAnimationController = AnimationController(
      duration: const Duration(milliseconds: 800),
      vsync: this,
    );

    if (widget.mappingActive) {
      _robotAnimationController.repeat(reverse: true);
    }

    // ✅ Initial centering with delay
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (widget.currentOdometry?.position != null) {
        _centerOnRobotImmediate();
      }
      // 🐛 DEBUG: Print map data info
      _debugMapData();
    });
  }

  void _debugMapData() {
    print('🗺️ === MAP DEBUG INFO ===');
    print('🗺️ mapData: ${widget.mapData != null ? "Available" : "NULL"}');
    if (widget.mapData != null) {
      print(
          '🗺️ Map size: ${widget.mapData!.info.width}x${widget.mapData!.info.height}');
      print('🗺️ Map resolution: ${widget.mapData!.info.resolution}m/px');
      print(
          '🗺️ Map origin: (${widget.mapData!.info.origin.x}, ${widget.mapData!.info.origin.y})');
      print(
          '🗺️ Occupancy data length: ${widget.mapData!.occupancyData.length}');
      print(
          '🗺️ Expected length: ${widget.mapData!.info.width * widget.mapData!.info.height}');
    }

    print(
        '🏠 globalCostmap: ${widget.globalCostmap != null ? "Available" : "NULL"}');
    if (widget.globalCostmap != null) {
      print('🏠 Global costmap: ${widget.globalCostmap!['info']}');
    }

    print(
        '🏠 localCostmap: ${widget.localCostmap != null ? "Available" : "NULL"}');
    if (widget.localCostmap != null) {
      print('🏠 Local costmap: ${widget.localCostmap!['info']}');
    }
    print('🗺️ === END DEBUG ===');
  }

  @override
  void didUpdateWidget(LiveMappingCanvas oldWidget) {
    super.didUpdateWidget(oldWidget);

    // Update animation state
    if (widget.mappingActive && !oldWidget.mappingActive) {
      _robotAnimationController.repeat(reverse: true);
    } else if (!widget.mappingActive && oldWidget.mappingActive) {
      _robotAnimationController.stop();
      _robotAnimationController.reset();
    }

    // ✅ FIXED: Auto-centering logic with better coordinate handling
    if (_autoCenter && !_autoCenterPaused && widget.currentOdometry != null) {
      final currentPos = widget.currentOdometry!.position;

      // Debug logging to track robot movement
      if (_lastRobotPosition != null) {
        final distance = _distanceBetween(_lastRobotPosition!, currentPos);
        print(
            '🤖 Robot moved: ${distance.toStringAsFixed(3)}m from (${_lastRobotPosition!.x.toStringAsFixed(2)}, ${_lastRobotPosition!.y.toStringAsFixed(2)}) to (${currentPos.x.toStringAsFixed(2)}, ${currentPos.y.toStringAsFixed(2)})');
      }

      if (_shouldAutoCenter(currentPos)) {
        print('🎯 Auto-centering triggered');
        _centerOnRobotSmooth();
      }
      _lastRobotPosition = currentPos;
    }

    // Debug map data changes
    if (widget.mapData != oldWidget.mapData && widget.mapData != null) {
      print(
          '🗺️ Map data updated: ${widget.mapData!.info.width}x${widget.mapData!.info.height}');
    }
  }

  // ✅ UPDATED: Better auto-center logic using new settings
  bool _shouldAutoCenter(Position robotPos) {
    // Check if auto-center is enabled and not paused
    if (!_autoCenter || _autoCenterPaused) return false;

    // Don't auto-center if user is actively interacting
    if (_isUserPanning || _isUserZooming) return false;

    // Don't auto-center if user recently interacted (respects cooldown setting)
    if (_lastUserInteraction != null &&
        DateTime.now().difference(_lastUserInteraction!) <
            _userInteractionCooldown) {
      return false;
    }

    // Don't auto-center too frequently
    if (_lastAutoCenter != null &&
        DateTime.now().difference(_lastAutoCenter!) <
            Duration(milliseconds: 500)) {
      return false;
    }

    // Check movement threshold (configurable)
    if (_lastRobotPosition != null &&
        _distanceBetween(_lastRobotPosition!, robotPos) <
            _autoCenterThreshold) {
      return false;
    }

    // Check if robot is significantly out of view
    return _isRobotOutOfView(robotPos);
  }

  // ✅ IMPROVED: Better viewport detection
  bool _isRobotOutOfView(Position robotPos) {
    if (!mounted) return false;

    final size = MediaQuery.of(context).size;

    // Calculate robot position in screen coordinates
    final robotScreenPos =
        _mapToScreenCoordinates(Offset(robotPos.x, robotPos.y));

    // Define comfortable margins (20% of screen size)
    final marginX = size.width * 0.2;
    final marginY = size.height * 0.2;

    // Check if robot is outside the comfortable viewing area
    return robotScreenPos.dx < marginX ||
        robotScreenPos.dx > size.width - marginX ||
        robotScreenPos.dy < marginY ||
        robotScreenPos.dy > size.height - marginY;
  }

  double _distanceBetween(Position p1, Position p2) {
    return math
        .sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  }

  @override
  void dispose() {
    _transformationController.dispose();
    _robotAnimationController.dispose();
    _centeringAnimationController.dispose();
    _userInteractionTimer?.cancel();
    super.dispose();
  }

  void _centerOnRobotImmediate() {
    if (widget.currentOdometry?.position == null || !mounted) return;

    final robotPos = widget.currentOdometry!.position;
    final size = MediaQuery.of(context).size;
    final screenCenter = Offset(size.width / 2, size.height / 2);

    // ✅ FIXED: Correct coordinate transformation to match painter
    final targetTransform = Matrix4.identity()
      ..translate(screenCenter.dx, screenCenter.dy)
      ..scale(_scale)
      ..translate(-robotPos.x * _gridSpacing,
          -(-robotPos.y * _gridSpacing)); // Fixed Y coordinate

    _transformationController.value = targetTransform;
    _lastAutoCenter = DateTime.now();

    print(
        '🎯 Immediate center on robot: (${robotPos.x.toStringAsFixed(2)}, ${robotPos.y.toStringAsFixed(2)})');
  }

  void _centerOnRobotSmooth() {
    if (widget.currentOdometry?.position == null || !mounted) return;

    final robotPos = widget.currentOdometry!.position;
    final size = MediaQuery.of(context).size;
    final screenCenter = Offset(size.width / 2, size.height / 2);

    final currentTransform = _transformationController.value;

    // ✅ FIXED: Correct coordinate transformation to match painter
    final targetTransform = Matrix4.identity()
      ..translate(screenCenter.dx, screenCenter.dy)
      ..scale(_scale)
      ..translate(-robotPos.x * _gridSpacing,
          -(-robotPos.y * _gridSpacing)); // Fixed Y coordinate

    final animation = Matrix4Tween(
      begin: currentTransform,
      end: targetTransform,
    ).animate(CurvedAnimation(
      parent: _centeringAnimationController,
      curve: Curves.easeOutCubic,
    ));

    animation.addListener(() {
      if (mounted && !_isUserPanning && !_isUserZooming) {
        _transformationController.value = animation.value;
      }
    });

    _centeringAnimationController.reset();
    _centeringAnimationController.forward();
    _lastAutoCenter = DateTime.now();

    print(
        '🎯 Smooth center on robot: (${robotPos.x.toStringAsFixed(2)}, ${robotPos.y.toStringAsFixed(2)})');
  }

  // ✅ FIXED: Correct screen coordinate mapping
  Offset _mapToScreenCoordinates(Offset mapPoint) {
    final transform = _transformationController.value;

    // Use same coordinate system as the painter
    final worldPoint = Offset(mapPoint.dx * _gridSpacing,
        -mapPoint.dy * _gridSpacing // Fixed Y coordinate to match painter
        );

    return MatrixUtils.transformPoint(transform, worldPoint);
  }

  // ✅ IMPROVED: Less aggressive interaction detection
  void _onInteractionStart(ScaleStartDetails details) {
    _isUserPanning = true;
    _lastUserInteraction = DateTime.now();

    // Stop any ongoing centering animation
    if (_centeringAnimationController.isAnimating) {
      _centeringAnimationController.stop();
    }

    // Reset user interaction timer
    _userInteractionTimer?.cancel();
  }

  void _onInteractionUpdate(ScaleUpdateDetails details) {
    _lastUserInteraction = DateTime.now();

    // Track if user is zooming
    if (details.scale != 1.0) {
      _isUserZooming = true;
      _scale = (_scale * details.scale).clamp(0.2, 10.0);
    }
    // Remove the panning restriction - let users pan at any zoom level
    // The original code was blocking panning when zoomed out

    // Reset interaction timer
    _userInteractionTimer?.cancel();
    _userInteractionTimer = Timer(Duration(seconds: 3), () {
      setState(() {
        _isUserPanning = false;
        _isUserZooming = false;
      });
    });
  }

  void _onInteractionEnd(ScaleEndDetails details) {
    // Give user a moment before auto-centering can resume
    _userInteractionTimer?.cancel();
    _userInteractionTimer = Timer(Duration(seconds: 2), () {
      setState(() {
        _isUserPanning = false;
        _isUserZooming = false;
      });
    });
  }

  // ✅ ADDITIONAL: Force center button for manual override
  void _forceCenterOnRobot() {
    if (widget.currentOdometry?.position != null) {
      // Cancel any user interaction state
      _isUserPanning = false;
      _isUserZooming = false;
      _userInteractionTimer?.cancel();

      // Force immediate centering
      _centerOnRobotImmediate();

      print('🎯 Force center triggered by user');
    }
  }

  // HELPER: Get auto-center status
  String _getAutoCenterStatus() {
    if (!_autoCenter) return 'Disabled';
    if (_autoCenterPaused) return 'Paused';
    if (_isUserPanning || _isUserZooming) return 'User Interacting';
    if (_lastUserInteraction != null &&
        DateTime.now().difference(_lastUserInteraction!) <
            _userInteractionCooldown) {
      return 'Cooldown';
    }
    return 'Active';
  }

  // HELPER: Get status color
  Color _getAutoCenterStatusColor() {
    final status = _getAutoCenterStatus();
    switch (status) {
      case 'Active':
        return Colors.green;
      case 'User Interacting':
      case 'Cooldown':
        return Colors.orange;
      case 'Paused':
      case 'Disabled':
        return Colors.red;
      default:
        return Colors.grey;
    }
  }

  // IMPROVED: Widget for auto-center controls in your settings
  Widget _buildAutoCenterSettings() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.blue.withOpacity(0.05),
            Colors.blue.withOpacity(0.1),
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
                  child: const Icon(Icons.my_location,
                      color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                const Text(
                  'Auto-Center Settings',
                  style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
                ),
              ],
            ),
            const SizedBox(height: 16),

            // Main auto-center toggle
            SwitchListTile(
              title: const Text('Auto-Center on Robot'),
              subtitle: Text(_autoCenter
                  ? 'Map follows robot automatically'
                  : 'Manual centering only'),
              value: _autoCenter,
              onChanged: (value) {
                setState(() {
                  _autoCenter = value;
                  if (value && widget.currentOdometry?.position != null) {
                    _centerOnRobotSmooth();
                  }
                });
              },
              dense: true,
            ),

            if (_autoCenter) ...[
              // Aggressive mode toggle
              SwitchListTile(
                title: const Text('Aggressive Auto-Center'),
                subtitle:
                    const Text('Center more frequently for small movements'),
                value: _aggressiveAutoCenter,
                onChanged: (value) {
                  setState(() {
                    _aggressiveAutoCenter = value;
                    _autoCenterThreshold = value ? 0.05 : 0.1;
                  });
                },
                dense: true,
              ),

              const SizedBox(height: 8),

              // Movement threshold slider
              Text(
                'Movement Threshold: ${_autoCenterThreshold.toStringAsFixed(2)}m',
                style:
                    const TextStyle(fontSize: 14, fontWeight: FontWeight.w600),
              ),
              SliderTheme(
                data: SliderTheme.of(context).copyWith(
                  trackHeight: 4,
                  thumbShape:
                      const RoundSliderThumbShape(enabledThumbRadius: 6),
                ),
                child: Slider(
                  value: _autoCenterThreshold,
                  min: 0.01,
                  max: 0.5,
                  divisions: 49,
                  onChanged: (value) {
                    setState(() {
                      _autoCenterThreshold = value;
                    });
                  },
                ),
              ),

              const SizedBox(height: 8),

              // Cooldown duration slider
              Text(
                'User Interaction Cooldown: ${_userInteractionCooldown.inSeconds}s',
                style:
                    const TextStyle(fontSize: 14, fontWeight: FontWeight.w600),
              ),
              SliderTheme(
                data: SliderTheme.of(context).copyWith(
                  trackHeight: 4,
                  thumbShape:
                      const RoundSliderThumbShape(enabledThumbRadius: 6),
                ),
                child: Slider(
                  value: _userInteractionCooldown.inSeconds.toDouble(),
                  min: 1,
                  max: 10,
                  divisions: 9,
                  onChanged: (value) {
                    setState(() {
                      _userInteractionCooldown =
                          Duration(seconds: value.round());
                    });
                  },
                ),
              ),
            ],

            const SizedBox(height: 16),

            // Manual control buttons
            Row(
              children: [
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _forceCenterOnRobot,
                    icon: const Icon(Icons.center_focus_strong),
                    label: const Text('Center Now'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.green,
                      foregroundColor: Colors.white,
                    ),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: () {
                      setState(() {
                        _autoCenterPaused = !_autoCenterPaused;
                      });
                    },
                    icon: Icon(
                        _autoCenterPaused ? Icons.play_arrow : Icons.pause),
                    label: Text(_autoCenterPaused ? 'Resume' : 'Pause'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor:
                          _autoCenterPaused ? Colors.orange : Colors.blue,
                      foregroundColor: Colors.white,
                    ),
                  ),
                ),
              ],
            ),

            // Status indicator
            if (_autoCenter) ...[
              const SizedBox(height: 12),
              Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: Colors.grey.shade100,
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: Colors.grey.shade300),
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Auto-Center Status: ${_getAutoCenterStatus()}',
                      style: TextStyle(
                        fontWeight: FontWeight.bold,
                        color: _getAutoCenterStatusColor(),
                      ),
                    ),
                    if (_lastAutoCenter != null)
                      Text(
                        'Last centered: ${DateTime.now().difference(_lastAutoCenter!).inSeconds}s ago',
                        style: TextStyle(
                            fontSize: 12, color: Colors.grey.shade600),
                      ),
                    if (_lastUserInteraction != null)
                      Text(
                        'Last interaction: ${DateTime.now().difference(_lastUserInteraction!).inSeconds}s ago',
                        style: TextStyle(
                            fontSize: 12, color: Colors.grey.shade600),
                      ),
                  ],
                ),
              ),
            ],
          ],
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        // ✅ Dark background like ROS
        Container(
          decoration: BoxDecoration(
            color: Color(0xFF2B2B2B), // Dark grey background like ROS
          ),
          child: InteractiveViewer(
            transformationController: _transformationController,
            boundaryMargin: const EdgeInsets.all(100),
            minScale: 0.2,
            maxScale: 10.0,
            onInteractionStart: _onInteractionStart,
            onInteractionUpdate: _onInteractionUpdate,
            onInteractionEnd: _onInteractionEnd,
            child: Container(
              width: 4000,
              height: 4000,
              child: CustomPaint(
                painter: ROSStyleLiveMapPainter(
                  mapData: widget.mapData,
                  currentOdometry: widget.currentOdometry,
                  robotTrail: widget.robotTrail,
                  globalCostmap: widget.globalCostmap,
                  localCostmap: widget.localCostmap,
                  showOccupancyGrid: widget.showOccupancyGrid,
                  costmapOpacity: widget.costmapOpacity,
                  robotScale: _robotAnimationController.value,
                  mappingActive: widget.mappingActive,
                  theme: Theme.of(context),
                  scale: _scale,
                  movementThreshold: _movementThreshold,
                ),
                size: const Size(4000, 4000),
              ),
            ),
          ),
        ),

        // Enhanced controls overlay
        Positioned(
          top: 16,
          right: 16,
          child: Column(
            children: [
              // Auto-center toggle
              Container(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: _autoCenter && !_autoCenterPaused
                        ? [Colors.blue.shade400, Colors.blue.shade600]
                        : _autoCenterPaused
                            ? [Colors.orange.shade400, Colors.orange.shade600]
                            : [Colors.grey.shade400, Colors.grey.shade600],
                  ),
                  borderRadius: BorderRadius.circular(25),
                  boxShadow: [
                    BoxShadow(
                      color: (_autoCenter && !_autoCenterPaused
                              ? Colors.blue
                              : _autoCenterPaused
                                  ? Colors.orange
                                  : Colors.grey)
                          .withOpacity(0.3),
                      blurRadius: 8,
                      offset: const Offset(0, 4),
                    ),
                  ],
                ),
                child: FloatingActionButton(
                  mini: true,
                  backgroundColor: Colors.transparent,
                  elevation: 0,
                  heroTag: "auto_center",
                  onPressed: () {
                    setState(() {
                      if (_autoCenterPaused) {
                        _autoCenterPaused = false;
                      } else {
                        _autoCenter = !_autoCenter;
                      }
                    });
                    if (_autoCenter &&
                        !_autoCenterPaused &&
                        widget.currentOdometry?.position != null) {
                      _centerOnRobotSmooth();
                    }
                  },
                  child: Stack(
                    alignment: Alignment.center,
                    children: [
                      Icon(_autoCenterPaused ? Icons.pause : Icons.my_location,
                          color: Colors.white),
                      if (_autoCenter && _isUserPanning)
                        Positioned(
                          top: 2,
                          right: 2,
                          child: Container(
                            width: 8,
                            height: 8,
                            decoration: BoxDecoration(
                              color: Colors.orange,
                              shape: BoxShape.circle,
                            ),
                          ),
                        ),
                    ],
                  ),
                  tooltip: _autoCenterPaused
                      ? 'Auto Center: PAUSED'
                      : _autoCenter
                          ? 'Auto Center: ON'
                          : 'Auto Center: OFF',
                ),
              ),

              const SizedBox(height: 12),

              // Manual center button
              Container(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [Colors.green.shade400, Colors.green.shade600],
                  ),
                  borderRadius: BorderRadius.circular(25),
                  boxShadow: [
                    BoxShadow(
                      color: Colors.green.withOpacity(0.3),
                      blurRadius: 8,
                      offset: const Offset(0, 4),
                    ),
                  ],
                ),
                child: FloatingActionButton(
                  mini: true,
                  backgroundColor: Colors.transparent,
                  elevation: 0,
                  heroTag: "center_now",
                  onPressed: () {
                    if (widget.currentOdometry?.position != null) {
                      _forceCenterOnRobot();
                    }
                  },
                  child: const Icon(Icons.center_focus_strong,
                      color: Colors.white),
                  tooltip: 'Center Now',
                ),
              ),

              const SizedBox(height: 12),

              // Pause/Resume auto-center button
              Container(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: _autoCenterPaused
                        ? [Colors.orange.shade400, Colors.orange.shade600]
                        : [Colors.blue.shade400, Colors.blue.shade600],
                  ),
                  borderRadius: BorderRadius.circular(25),
                  boxShadow: [
                    BoxShadow(
                      color: (_autoCenterPaused ? Colors.orange : Colors.blue)
                          .withOpacity(0.3),
                      blurRadius: 8,
                      offset: const Offset(0, 4),
                    ),
                  ],
                ),
                child: FloatingActionButton(
                  mini: true,
                  backgroundColor: Colors.transparent,
                  elevation: 0,
                  heroTag: "pause_auto_center",
                  onPressed: () {
                    setState(() {
                      _autoCenterPaused = !_autoCenterPaused;
                    });
                  },
                  child: Icon(
                    _autoCenterPaused ? Icons.play_arrow : Icons.pause,
                    color: Colors.white,
                  ),
                  tooltip: _autoCenterPaused
                      ? 'Resume Auto-Center'
                      : 'Pause Auto-Center',
                ),
              ),

              const SizedBox(height: 12),

              // Enhanced zoom controls
              Container(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [
                      Colors.black.withOpacity(0.7),
                      Colors.black.withOpacity(0.5),
                    ],
                  ),
                  borderRadius: BorderRadius.circular(25),
                  boxShadow: [
                    BoxShadow(
                      color: Colors.black.withOpacity(0.2),
                      blurRadius: 10,
                      offset: const Offset(0, 4),
                    ),
                  ],
                ),
                child: Column(
                  children: [
                    IconButton(
                      onPressed: () => _zoomIn(),
                      icon: const Icon(Icons.zoom_in, color: Colors.white),
                      tooltip: 'Zoom In',
                    ),
                    Container(
                      width: 30,
                      height: 1,
                      color: Colors.white.withOpacity(0.3),
                    ),
                    IconButton(
                      onPressed: () => _zoomOut(),
                      icon: const Icon(Icons.zoom_out, color: Colors.white),
                      tooltip: 'Zoom Out',
                    ),
                  ],
                ),
              ),

              const SizedBox(height: 12),

              // Movement status indicator
              Container(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: _scale > _movementThreshold
                        ? [Colors.green.shade400, Colors.green.shade600]
                        : [Colors.red.shade400, Colors.red.shade600],
                  ),
                  borderRadius: BorderRadius.circular(25),
                  boxShadow: [
                    BoxShadow(
                      color: (_scale > _movementThreshold
                              ? Colors.green
                              : Colors.red)
                          .withOpacity(0.3),
                      blurRadius: 8,
                      offset: const Offset(0, 4),
                    ),
                  ],
                ),
                child: FloatingActionButton(
                  mini: true,
                  backgroundColor: Colors.transparent,
                  elevation: 0,
                  heroTag: "movement_status",
                  onPressed: null,
                  child: Icon(
                    _scale > _movementThreshold ? Icons.pan_tool : Icons.lock,
                    color: Colors.white,
                  ),
                  tooltip: _scale > _movementThreshold
                      ? 'Movement: Enabled'
                      : 'Movement: Locked',
                ),
              ),

              const SizedBox(height: 12),

              // Reset view button
              Container(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [Colors.purple.shade400, Colors.purple.shade600],
                  ),
                  borderRadius: BorderRadius.circular(25),
                  boxShadow: [
                    BoxShadow(
                      color: Colors.purple.withOpacity(0.3),
                      blurRadius: 8,
                      offset: const Offset(0, 4),
                    ),
                  ],
                ),
                child: FloatingActionButton(
                  mini: true,
                  backgroundColor: Colors.transparent,
                  elevation: 0,
                  heroTag: "reset_view",
                  onPressed: _resetView,
                  child: const Icon(Icons.refresh, color: Colors.white),
                  tooltip: 'Reset View',
                ),
              ),
            ],
          ),
        ),

        // Enhanced status overlay
        Positioned(
          bottom: 16,
          left: 16,
          child: Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [
                  Colors.black.withOpacity(0.8),
                  Colors.black.withOpacity(0.6),
                ],
              ),
              borderRadius: BorderRadius.circular(12),
              border:
                  Border.all(color: Colors.white.withOpacity(0.2), width: 1),
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.3),
                  blurRadius: 15,
                  offset: const Offset(0, 6),
                ),
              ],
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    Container(
                      width: 8,
                      height: 8,
                      decoration: BoxDecoration(
                        color:
                            widget.mappingActive ? Colors.green : Colors.blue,
                        shape: BoxShape.circle,
                        boxShadow: [
                          BoxShadow(
                            color: (widget.mappingActive
                                    ? Colors.green
                                    : Colors.blue)
                                .withOpacity(0.5),
                            blurRadius: 4,
                            spreadRadius: 1,
                          ),
                        ],
                      ),
                    ),
                    const SizedBox(width: 8),
                    Text(
                      'Live Map Status',
                      style: TextStyle(
                        color: Colors.white,
                        fontWeight: FontWeight.bold,
                        fontSize: 14,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 8),
                _buildStatusRow('Auto Center', _autoCenter ? 'ON' : 'OFF',
                    _autoCenter ? Colors.green : Colors.grey),
                if (_autoCenter)
                  _buildStatusRow('AC Status', _getAutoCenterStatus(),
                      _getAutoCenterStatusColor()),
                if (_autoCenterPaused)
                  _buildStatusRow('AC Paused', 'YES', Colors.orange),
                if (_autoCenter && _aggressiveAutoCenter)
                  _buildStatusRow('Aggressive', 'ON', Colors.purple),
                _buildStatusRow(
                    'Movement',
                    _scale > _movementThreshold ? 'ENABLED' : 'LOCKED',
                    _scale > _movementThreshold ? Colors.green : Colors.red),
                _buildStatusRow('Threshold',
                    '${_autoCenterThreshold.toStringAsFixed(2)}m', Colors.cyan),
                _buildStatusRow('Cooldown',
                    '${_userInteractionCooldown.inSeconds}s', Colors.cyan),
                if (_isUserPanning)
                  _buildStatusRow('User Input', 'ACTIVE', Colors.orange),
                _buildStatusRow('Trail', '${widget.robotTrail.length} points',
                    Colors.orange),
                if (widget.mapData != null)
                  _buildStatusRow(
                      'Map',
                      '${widget.mapData!.info.width}×${widget.mapData!.info.height}',
                      Colors.cyan),
                if (widget.globalCostmap != null)
                  _buildStatusRow(
                    'Global',
                    '${widget.globalCostmap!['info']?['width'] ?? 'N/A'}×${widget.globalCostmap!['info']?['height'] ?? 'N/A'}',
                    Colors.blue,
                  ),
                if (widget.localCostmap != null)
                  _buildStatusRow(
                    'Local',
                    '${widget.localCostmap!['info']?['width'] ?? 'N/A'}×${widget.localCostmap!['info']?['height'] ?? 'N/A'}',
                    Colors.red,
                  ),
                _buildStatusRow(
                    'Scale', '${_scale.toStringAsFixed(1)}×', Colors.grey),
                if (widget.currentOdometry?.position != null)
                  _buildStatusRow(
                    'Position',
                    '(${widget.currentOdometry!.position.x.toStringAsFixed(1)}, ${widget.currentOdometry!.position.y.toStringAsFixed(1)})',
                    Colors.cyan,
                  ),
              ],
            ),
          ),
        ),

        // Mapping status indicator
        if (widget.mappingActive)
          Positioned(
            top: 16,
            left: 16,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
              decoration: BoxDecoration(
                gradient: const LinearGradient(
                  colors: [Colors.green, Colors.lightGreen],
                ),
                borderRadius: BorderRadius.circular(20),
                boxShadow: [
                  BoxShadow(
                    color: Colors.green.withOpacity(0.4),
                    blurRadius: 10,
                    offset: const Offset(0, 4),
                  ),
                ],
              ),
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  SizedBox(
                    width: 16,
                    height: 16,
                    child: CircularProgressIndicator(
                      strokeWidth: 2,
                      valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
                    ),
                  ),
                  const SizedBox(width: 8),
                  const Text(
                    'MAPPING ACTIVE',
                    style: TextStyle(
                      color: Colors.white,
                      fontWeight: FontWeight.bold,
                      fontSize: 12,
                    ),
                  ),
                ],
              ),
            ),
          ),
      ],
    );
  }

  Widget _buildStatusRow(String label, String value, Color color) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 4,
            height: 12,
            decoration: BoxDecoration(
              color: color,
              borderRadius: BorderRadius.circular(2),
            ),
          ),
          const SizedBox(width: 6),
          Text(
            '$label: ',
            style: TextStyle(
              color: Colors.white.withOpacity(0.8),
              fontSize: 11,
            ),
          ),
          Text(
            value,
            style: TextStyle(
              color: Colors.white,
              fontSize: 11,
              fontWeight: FontWeight.bold,
            ),
          ),
        ],
      ),
    );
  }

  void _zoomIn() {
    _lastUserInteraction = DateTime.now();
    _isUserZooming = true;

    final currentTransform = _transformationController.value;
    final newScale = (_scale * 1.2).clamp(0.2, 10.0);
    final newTransform = currentTransform.clone()..scale(1.2);
    _transformationController.value = newTransform;
    setState(() {
      _scale = newScale;
    });
  }

  void _zoomOut() {
    _lastUserInteraction = DateTime.now();
    _isUserZooming = true;

    final currentTransform = _transformationController.value;
    final newScale = (_scale * 0.8).clamp(0.2, 10.0);
    final newTransform = currentTransform.clone()..scale(0.8);
    _transformationController.value = newTransform;
    setState(() {
      _scale = newScale;
    });
  }

  void _resetView() {
    _lastUserInteraction = DateTime.now();
    _scale = 1.5;
    _transformationController.value = Matrix4.identity()..scale(_scale);

    if (widget.currentOdometry?.position != null) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        _centerOnRobotSmooth();
      });
    }
  }
}

// ✅ FIXED: Keep original painter approach but with ROS colors
class ROSStyleLiveMapPainter extends CustomPainter {
  final MapData? mapData;
  final OdometryData? currentOdometry;
  final List<Position> robotTrail;
  final Map<String, dynamic>? globalCostmap;
  final Map<String, dynamic>? localCostmap;
  final bool showOccupancyGrid;
  final double costmapOpacity;
  final double robotScale;
  final bool mappingActive;
  final ThemeData theme;
  final double scale;
  final double movementThreshold;

  // ✅ Keep original visual constants
  static const double _gridSpacing = 20.0;
  static const double _robotSize = 12.0;
  static const double _trailWidth = 2.5;
  static const double _shapeOutlineWidth = 2.0;

  ROSStyleLiveMapPainter({
    this.mapData,
    this.currentOdometry,
    this.robotTrail = const [],
    this.globalCostmap,
    this.localCostmap,
    this.showOccupancyGrid = true,
    this.costmapOpacity = 0.7,
    this.robotScale = 1.0,
    this.mappingActive = false,
    required this.theme,
    required this.scale,
    required this.movementThreshold,
  });

  @override
  void paint(Canvas canvas, Size size) {
    // Draw subtle grid background
    _drawSubtleGrid(canvas, size);

    // ✅ FIXED: Draw ROS-style occupancy grid as the base layer
    if (showOccupancyGrid && mapData != null) {
      _drawROSStyleOccupancyGrid(canvas, size, mapData!);
    }

    // Draw costmap overlays
    if (globalCostmap != null) {
      _drawROSStyleCostmap(canvas, size, globalCostmap!, Colors.blue, 'global');
    }

    if (localCostmap != null) {
      _drawROSStyleCostmap(canvas, size, localCostmap!, Colors.red, 'local');
    }

    // Draw robot trail
    if (robotTrail.isNotEmpty) {
      _drawRobotTrail(canvas, size);
    }

    // Draw robot
    if (currentOdometry?.position != null) {
      _drawRobot(canvas, size, currentOdometry!);
    }

    // Draw map shapes
    if (mapData?.shapes != null) {
      _drawMapShapes(canvas, size, mapData!.shapes);
    }
  }

  void _drawSubtleGrid(Canvas canvas, Size size) {
    // ✅ Very subtle grid lines
    final majorGridPaint = Paint()
      ..color = Colors.grey.withOpacity(0.15)
      ..strokeWidth = 0.5;

    final minorGridPaint = Paint()
      ..color = Colors.grey.withOpacity(0.08)
      ..strokeWidth = 0.3;

    // Draw minor grid lines (every 0.5 meters)
    const minorSpacing = _gridSpacing / 2;
    for (double x = 0; x < size.width; x += minorSpacing) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), minorGridPaint);
    }
    for (double y = 0; y < size.height; y += minorSpacing) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), minorGridPaint);
    }

    // Draw major grid lines (every 1 meter)
    for (double x = 0; x < size.width; x += _gridSpacing) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), majorGridPaint);
    }
    for (double y = 0; y < size.height; y += _gridSpacing) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), majorGridPaint);
    }

    // Draw subtle axes
    final axisPaint = Paint()
      ..color = theme.primaryColor.withOpacity(0.3)
      ..strokeWidth = 1.0;

    final centerX = size.width / 2;
    final centerY = size.height / 2;

    canvas.drawLine(Offset(0, centerY), Offset(size.width, centerY), axisPaint);
    canvas.drawLine(
        Offset(centerX, 0), Offset(centerX, size.height), axisPaint);
  }

  // ✅ FIXED: ROS-style occupancy grid with NO cell skipping
  void _drawROSStyleOccupancyGrid(Canvas canvas, Size size, MapData mapData) {
    if (mapData.occupancyData.isEmpty) {
      print('⚠️ No occupancy data to render');
      return;
    }

    final info = mapData.info;
    final resolution = info.resolution;
    final origin = info.origin;

    final cellSize = resolution * _gridSpacing;

    // ✅ ROS standard colors
    final rosColors = {
      'free': Color(0xFF00FFFF), // Cyan for free space (value 0)
      'unknown': Color(0xFF808080), // Grey for unknown space (value -1)
      'occupied': Color(0xFFFF00FF), // Magenta for occupied space (value 100)
      'lowProb': Color(0xFF40E0D0), // Turquoise for low probability
      'medProb': Color(0xFF9370DB), // Medium violet for medium probability
      'highProb': Color(0xFFDA70D6), // Orchid for high probability
    };

    int renderedCells = 0;

    // ✅ FIXED: Render ALL cells (no skipping y += 2, x += 2)
    for (int y = 0; y < info.height; y++) {
      for (int x = 0; x < info.width; x++) {
        final index = y * info.width + x;
        if (index >= mapData.occupancyData.length) continue;

        final value = mapData.occupancyData[index];

        Color cellColor;

        // ✅ ROS-style color mapping
        if (value == -1) {
          cellColor = rosColors['unknown']!; // Grey for unknown
        } else if (value == 0) {
          cellColor = rosColors['free']!; // Cyan for free space
        } else if (value == 100) {
          cellColor = rosColors['occupied']!; // Magenta for occupied
        } else if (value < 30) {
          // Low probability occupied (turquoise blend)
          cellColor = Color.lerp(
              rosColors['free']!, rosColors['lowProb']!, value / 30.0)!;
        } else if (value < 70) {
          // Medium probability occupied (violet blend)
          cellColor = Color.lerp(rosColors['lowProb']!, rosColors['medProb']!,
              (value - 30) / 40.0)!;
        } else {
          // High probability occupied (magenta blend)
          cellColor = Color.lerp(rosColors['medProb']!, rosColors['occupied']!,
              (value - 70) / 30.0)!;
        }

        final paint = Paint()
          ..color = cellColor
          ..style = PaintingStyle.fill;

        // ✅ Keep original coordinate system
        final screenX =
            size.width / 2 + (origin.x + x * resolution) * _gridSpacing;
        final screenY =
            size.height / 2 - (origin.y + y * resolution) * _gridSpacing;

        canvas.drawRect(
          Rect.fromLTWH(screenX, screenY, cellSize, cellSize),
          paint,
        );

        renderedCells++;
      }
    }

    print('🗺️ Rendered $renderedCells ROS-style occupancy cells');
  }

  // ✅ FIXED: ROS-style costmap with proper overlay colors
  void _drawROSStyleCostmap(Canvas canvas, Size size,
      Map<String, dynamic> costmap, Color baseColor, String type) {
    final info = costmap['info'];
    final data = costmap['data'] as List?;

    if (info == null || data == null) {
      print('⚠️ No $type costmap data to render');
      return;
    }

    final width = info['width'] as int? ?? 0;
    final height = info['height'] as int? ?? 0;
    final resolution = info['resolution'] as double? ?? 0.05;
    final origin = info['origin'];

    if (width == 0 || height == 0 || origin == null) {
      print('⚠️ Invalid $type costmap dimensions or origin');
      return;
    }

    final originX = origin['position']?['x'] as double? ?? 0.0;
    final originY = origin['position']?['y'] as double? ?? 0.0;

    print(
        '🏠 Rendering ROS-style $type costmap: ${width}x$height, resolution: $resolution, origin: ($originX, $originY)');

    final cellSize = resolution * _gridSpacing;

    // ✅ ROS-style costmap colors
    final rosCostmapColors = {
      'global': {
        'low': Color(0x3300FF00), // Light green
        'medium': Color(0x66FFFF00), // Yellow
        'inscribed': Color(0x99FF8000), // Orange
        'lethal': Color(0xFFFF0000), // Red
      },
      'local': {
        'low': Color(0x330080FF), // Light blue
        'medium': Color(0x668000FF), // Purple
        'inscribed': Color(0x99FF0080), // Pink
        'lethal': Color(0xFFFF0000), // Red
      }
    };

    final colors = rosCostmapColors[type] ?? rosCostmapColors['global']!;

    int renderedCells = 0;

    // ✅ Render all costmap cells (no skipping)
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        final index = y * width + x;
        if (index >= data.length) continue;

        final value = data[index] as int? ?? 0;
        if (value == 0) continue; // Skip free space (transparent)

        Color cellColor;

        // ✅ ROS costmap color mapping
        if (value >= 100) {
          cellColor = colors['lethal']!; // Lethal obstacle (red)
        } else if (value >= 99) {
          cellColor = colors['inscribed']!; // Inscribed obstacle
        } else if (value >= 50) {
          cellColor = colors['medium']!; // Medium cost
        } else {
          cellColor = colors['low']!; // Low cost
        }

        // Apply global opacity setting
        cellColor = cellColor.withOpacity(cellColor.opacity * costmapOpacity);

        final paint = Paint()
          ..color = cellColor
          ..style = PaintingStyle.fill;

        // ✅ Keep original coordinate system
        final screenX =
            size.width / 2 + (originX + x * resolution) * _gridSpacing;
        final screenY =
            size.height / 2 - (originY + y * resolution) * _gridSpacing;

        canvas.drawRect(
          Rect.fromLTWH(screenX, screenY, cellSize, cellSize),
          paint,
        );

        renderedCells++;
      }
    }

    print('🏠 Rendered $renderedCells ROS-style $type costmap cells');
  }

  void _drawRobotTrail(Canvas canvas, Size size) {
    if (robotTrail.length < 2) return;

    // Create gradient trail effect
    final trailPaint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = _trailWidth
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;

    final path = Path();

    // Convert first point
    final firstPoint = robotTrail.first;
    final firstX = size.width / 2 + firstPoint.x * _gridSpacing;
    final firstY = size.height / 2 - firstPoint.y * _gridSpacing;
    path.moveTo(firstX, firstY);

    // Add remaining points
    for (int i = 1; i < robotTrail.length; i++) {
      final point = robotTrail[i];
      final x = size.width / 2 + point.x * _gridSpacing;
      final y = size.height / 2 - point.y * _gridSpacing;
      path.lineTo(x, y);
    }

    // Draw trail segments with fading effect
    final segmentCount = (robotTrail.length / 10).ceil();
    for (int segment = 0; segment < segmentCount; segment++) {
      final startIndex = segment * 10;
      final endIndex = math.min((segment + 1) * 10, robotTrail.length);

      if (endIndex - startIndex < 2) continue;

      // Calculate opacity based on segment age
      final opacity = 1.0 - (segment / segmentCount) * 0.7;
      trailPaint.color = Colors.orange.withOpacity(opacity);

      final segmentPath = Path();
      final startPoint = robotTrail[startIndex];
      final startX = size.width / 2 + startPoint.x * _gridSpacing;
      final startY = size.height / 2 - startPoint.y * _gridSpacing;
      segmentPath.moveTo(startX, startY);

      for (int i = startIndex + 1; i < endIndex; i++) {
        final point = robotTrail[i];
        final x = size.width / 2 + point.x * _gridSpacing;
        final y = size.width / 2 - point.y * _gridSpacing;
        segmentPath.lineTo(x, y);
      }

      canvas.drawPath(segmentPath, trailPaint);
    }
  }

  void _drawRobot(Canvas canvas, Size size, OdometryData odometry) {
    final position = odometry.position;
    final orientation = odometry.orientation;

    final x = size.width / 2 + position.x * _gridSpacing;
    final y = size.height / 2 - position.y * _gridSpacing;

    final robotSize = _robotSize * robotScale;

    // Robot glow effect for mapping mode
    if (mappingActive) {
      final glowPaint = Paint()
        ..color = Colors.green.withOpacity(0.3)
        ..style = PaintingStyle.fill
        ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 8.0);

      canvas.drawCircle(Offset(x, y), robotSize * 1.5, glowPaint);
    }

    // Robot body with gradient
    final bodyGradient = RadialGradient(
      colors: mappingActive
          ? [Colors.green.shade300, Colors.green.shade700]
          : [Colors.blue.shade300, Colors.blue.shade700],
    );

    final bodyPaint = Paint()
      ..shader = bodyGradient.createShader(
          Rect.fromCircle(center: Offset(x, y), radius: robotSize))
      ..style = PaintingStyle.fill;

    canvas.drawCircle(Offset(x, y), robotSize, bodyPaint);

    // Robot outline with enhanced styling
    final outlinePaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 2.5
      ..style = PaintingStyle.stroke;

    canvas.drawCircle(Offset(x, y), robotSize, outlinePaint);

    // Direction indicator with enhanced arrow
    final yaw = orientation.yaw;
    final directionLength = robotSize * 1.4;
    final directionX = x + math.cos(yaw) * directionLength;
    final directionY = y - math.sin(yaw) * directionLength;

    // Draw direction arrow with shadow
    final shadowPaint = Paint()
      ..color = Colors.black.withOpacity(0.3)
      ..strokeWidth = 4.0
      ..strokeCap = StrokeCap.round;

    canvas.drawLine(Offset(x + 1, y + 1),
        Offset(directionX + 1, directionY + 1), shadowPaint);

    final directionPaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 3.0
      ..strokeCap = StrokeCap.round;

    canvas.drawLine(
        Offset(x, y), Offset(directionX, directionY), directionPaint);

    // Draw arrowhead
    final arrowPaint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.fill;

    final arrowPath = Path();
    final arrowSize = robotSize * 0.3;
    arrowPath.moveTo(directionX, directionY);
    arrowPath.lineTo(directionX - arrowSize * math.cos(yaw - 2.5),
        directionY + arrowSize * math.sin(yaw - 2.5));
    arrowPath.lineTo(directionX - arrowSize * math.cos(yaw + 2.5),
        directionY + arrowSize * math.sin(yaw + 2.5));
    arrowPath.close();
    canvas.drawPath(arrowPath, arrowPaint);
  }

  void _drawMapShapes(Canvas canvas, Size size, List<MapShape> shapes) {
    for (final shape in shapes) {
      if (shape.points.isEmpty) continue;
      _drawShape(canvas, size, shape);
    }
  }

  void _drawShape(Canvas canvas, Size size, MapShape shape) {
    // Enhanced color parsing with fallback
    Color shapeColor;
    try {
      String colorString = shape.color;
      if (colorString.startsWith('#')) {
        colorString = colorString.substring(1);
      }
      if (colorString.length == 6) {
        colorString = 'FF$colorString';
      }
      final colorValue = int.tryParse('0x$colorString') ?? 0xFF6200EE;
      shapeColor = Color(colorValue);
    } catch (e) {
      shapeColor = theme.primaryColor;
    }

    final fillPaint = Paint()
      ..color = shapeColor.withOpacity(0.3)
      ..style = PaintingStyle.fill;

    final outlinePaint = Paint()
      ..color = shapeColor
      ..strokeWidth = _shapeOutlineWidth
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;

    if (shape.points.length == 1) {
      // Single point rendering
      final point = shape.points.first;
      final x = size.width / 2 + point.x * _gridSpacing;
      final y = size.height / 2 - point.y * _gridSpacing;

      canvas.drawCircle(Offset(x, y), 6, fillPaint);
      canvas.drawCircle(Offset(x, y), 6, outlinePaint);
    } else {
      // Multi-point shape
      final path = Path();

      final firstPoint = shape.points.first;
      final firstX = size.width / 2 + firstPoint.x * _gridSpacing;
      final firstY = size.height / 2 - firstPoint.y * _gridSpacing;
      path.moveTo(firstX, firstY);

      for (int i = 1; i < shape.points.length; i++) {
        final point = shape.points[i];
        final x = size.width / 2 + point.x * _gridSpacing;
        final y = size.height / 2 - point.y * _gridSpacing;
        path.lineTo(x, y);
      }

      if (shape.type != 'waypoint' && shape.points.length > 2) {
        path.close();
        canvas.drawPath(path, fillPaint);
      }

      canvas.drawPath(path, outlinePaint);

      // Draw point markers
      for (final point in shape.points) {
        final x = size.width / 2 + point.x * _gridSpacing;
        final y = size.height / 2 - point.y * _gridSpacing;

        final pointPaint = Paint()
          ..color = shapeColor
          ..style = PaintingStyle.fill;

        canvas.drawCircle(Offset(x, y), 4, pointPaint);

        final pointOutlinePaint = Paint()
          ..color = Colors.white
          ..strokeWidth = 1.5
          ..style = PaintingStyle.stroke;

        canvas.drawCircle(Offset(x, y), 4, pointOutlinePaint);
      }
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true; // Always repaint for live updates
  }
}
