// widgets/enhanced_live_mapping_canvas.dart - Enhanced Live Mapping Canvas with Auto-Centering Fix
import 'package:flutter/material.dart';
import 'dart:ui' as ui;
import 'dart:math' as math;
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
  late Animation<double> _robotPulseAnimation;
  late AnimationController _centeringAnimationController;
  
  // Map viewport settings with better defaults
  double _scale = 1.5; // Start with better zoom level
  Offset _translation = Offset.zero;
  bool _autoCenter = true;
  Position? _lastRobotPosition;
  
  // Drawing state
  bool _isDrawing = false;
  List<Offset> _currentStroke = [];

  // Enhanced visual settings
  static const double _gridSpacing = 20.0; // 1 meter = 20 pixels
  static const double _robotDisplaySize = 12.0;
  static const double _trailWidth = 2.5;

  @override
  void initState() {
    super.initState();
    _transformationController = TransformationController();
    
    // Robot pulse animation for live indication
    _robotAnimationController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );
    
    _robotPulseAnimation = Tween<double>(
      begin: 0.8,
      end: 1.2,
    ).animate(CurvedAnimation(
      parent: _robotAnimationController,
      curve: Curves.easeInOut,
    ));

    // Smooth centering animation
    _centeringAnimationController = AnimationController(
      duration: const Duration(milliseconds: 800),
      vsync: this,
    );
    
    if (widget.mappingActive) {
      _robotAnimationController.repeat(reverse: true);
    }

    // Initial centering
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (widget.currentOdometry?.position != null) {
        _centerOnRobotImmediate();
      }
    });
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
    
    // Enhanced auto-centering logic
    if (_autoCenter && widget.currentOdometry != null) {
      final currentPos = widget.currentOdometry!.position;
      
      // Check if robot moved significantly
      if (_lastRobotPosition == null || 
          _distanceBetween(_lastRobotPosition!, currentPos) > 0.1) {
        
        // Only center if robot is moving out of view
        if (_isRobotOutOfView(currentPos)) {
          _centerOnRobotSmooth();
        }
        
        _lastRobotPosition = currentPos;
      }
    }
  }

  bool _isRobotOutOfView(Position robotPos) {
    if (!mounted) return false;
    
    final size = MediaQuery.of(context).size;
    final screenCenter = Offset(size.width / 2, size.height / 2);
    
    // Calculate robot position on screen
    final robotScreenPos = _mapToScreenCoordinates(Offset(robotPos.x, robotPos.y));
    
    // Check if robot is within 80% of screen bounds
    final margin = math.min(size.width, size.height) * 0.1;
    return robotScreenPos.dx < margin || 
           robotScreenPos.dx > size.width - margin ||
           robotScreenPos.dy < margin || 
           robotScreenPos.dy > size.height - margin;
  }

  double _distanceBetween(Position p1, Position p2) {
    return math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  }

  @override
  void dispose() {
    _transformationController.dispose();
    _robotAnimationController.dispose();
    _centeringAnimationController.dispose();
    super.dispose();
  }

  void _centerOnRobotImmediate() {
    if (widget.currentOdometry?.position == null || !mounted) return;
    
    final robotPos = widget.currentOdometry!.position;
    final size = MediaQuery.of(context).size;
    final screenCenter = Offset(size.width / 2, size.height / 2);
    
    final targetTransform = Matrix4.identity()
      ..translate(screenCenter.dx - robotPos.x * _gridSpacing, 
                  screenCenter.dy + robotPos.y * _gridSpacing)
      ..scale(_scale);
    
    _transformationController.value = targetTransform;
  }

  void _centerOnRobotSmooth() {
    if (widget.currentOdometry?.position == null || !mounted) return;
    
    final robotPos = widget.currentOdometry!.position;
    final size = MediaQuery.of(context).size;
    final screenCenter = Offset(size.width / 2, size.height / 2);
    
    final currentTransform = _transformationController.value;
    final targetTransform = Matrix4.identity()
      ..translate(screenCenter.dx - robotPos.x * _gridSpacing, 
                  screenCenter.dy + robotPos.y * _gridSpacing)
      ..scale(_scale);

    // Animate to target transform
    final animation = Matrix4Tween(
      begin: currentTransform,
      end: targetTransform,
    ).animate(CurvedAnimation(
      parent: _centeringAnimationController,
      curve: Curves.easeOutCubic,
    ));

    animation.addListener(() {
      _transformationController.value = animation.value;
    });

    _centeringAnimationController.reset();
    _centeringAnimationController.forward();
  }

  Offset _mapToScreenCoordinates(Offset mapPoint) {
    final transform = _transformationController.value;
    final transformedPoint = MatrixUtils.transformPoint(transform, mapPoint * _gridSpacing);
    return transformedPoint;
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    
    return Stack(
      children: [
        // Main canvas with enhanced styling
        Container(
          decoration: BoxDecoration(
            gradient: LinearGradient(
              begin: Alignment.topLeft,
              end: Alignment.bottomRight,
              colors: [
                Colors.grey.shade100,
                Colors.grey.shade50,
              ],
            ),
          ),
          child: InteractiveViewer(
            transformationController: _transformationController,
            boundaryMargin: const EdgeInsets.all(100),
            minScale: 0.2,
            maxScale: 10.0,
            onInteractionUpdate: (details) {
              setState(() {
                _scale = _transformationController.value.getMaxScaleOnAxis();
              });
            },
            child: Container(
              width: 4000,
              height: 4000,
              child: CustomPaint(
                painter: EnhancedLiveMapPainter(
                  mapData: widget.mapData,
                  currentOdometry: widget.currentOdometry,
                  robotTrail: widget.robotTrail,
                  globalCostmap: widget.globalCostmap,
                  localCostmap: widget.localCostmap,
                  showOccupancyGrid: widget.showOccupancyGrid,
                  costmapOpacity: widget.costmapOpacity,
                  robotScale: _robotPulseAnimation.value,
                  mappingActive: widget.mappingActive,
                  theme: theme,
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
              // Auto-center toggle with enhanced styling
              Container(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: _autoCenter 
                        ? [Colors.blue.shade400, Colors.blue.shade600]
                        : [Colors.grey.shade400, Colors.grey.shade600],
                  ),
                  borderRadius: BorderRadius.circular(25),
                  boxShadow: [
                    BoxShadow(
                      color: (_autoCenter ? Colors.blue : Colors.grey).withOpacity(0.3),
                      blurRadius: 8,
                      offset: const Offset(0, 4),
                    ),
                  ],
                ),
                child: FloatingActionButton(
                  mini: true,
                  backgroundColor: Colors.transparent,
                  elevation: 0,
                  onPressed: () {
                    setState(() {
                      _autoCenter = !_autoCenter;
                    });
                    if (_autoCenter) {
                      _centerOnRobotSmooth();
                    }
                  },
                  child: const Icon(
                    Icons.my_location,
                    color: Colors.white,
                  ),
                  tooltip: 'Auto Center on Robot',
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
                  onPressed: _resetView,
                  child: const Icon(
                    Icons.center_focus_strong,
                    color: Colors.white,
                  ),
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
              border: Border.all(
                color: Colors.white.withOpacity(0.2),
                width: 1,
              ),
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
                        color: widget.mappingActive ? Colors.green : Colors.blue,
                        shape: BoxShape.circle,
                        boxShadow: [
                          BoxShadow(
                            color: (widget.mappingActive ? Colors.green : Colors.blue).withOpacity(0.5),
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
                _buildStatusRow('Trail', '${widget.robotTrail.length} points', Colors.orange),
                if (widget.mapData != null)
                  _buildStatusRow('Shapes', '${widget.mapData!.shapes.length}', Colors.purple),
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
                _buildStatusRow('Scale', '${_scale.toStringAsFixed(1)}×', Colors.grey),
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
    final currentTransform = _transformationController.value;
    final newScale = (_scale * 1.2).clamp(0.2, 10.0);
    final newTransform = currentTransform.clone()..scale(1.2);
    _transformationController.value = newTransform;
    setState(() {
      _scale = newScale;
    });
  }

  void _zoomOut() {
    final currentTransform = _transformationController.value;
    final newScale = (_scale * 0.8).clamp(0.2, 10.0);
    final newTransform = currentTransform.clone()..scale(0.8);
    _transformationController.value = newTransform;
    setState(() {
      _scale = newScale;
    });
  }

  void _resetView() {
    _scale = 1.5;
    _transformationController.value = Matrix4.identity()..scale(_scale);
    
    // Center on robot if available
    if (widget.currentOdometry?.position != null) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        _centerOnRobotSmooth();
      });
    }
  }
}

class EnhancedLiveMapPainter extends CustomPainter {
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

  // Enhanced visual constants
  static const double _gridSpacing = 20.0;
  static const double _robotSize = 12.0;
  static const double _trailWidth = 2.5;
  static const double _shapeOutlineWidth = 2.0;

  EnhancedLiveMapPainter({
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
  });

  @override
  void paint(Canvas canvas, Size size) {
    final centerX = size.width / 2;
    final centerY = size.height / 2;
    
    // Draw enhanced grid background
    _drawEnhancedGrid(canvas, size);
    
    // Draw occupancy grid map if available and enabled
    if (showOccupancyGrid && mapData != null) {
      _drawEnhancedOccupancyGrid(canvas, size, mapData!);
    }
    
    // Draw costmaps with enhanced styling
    if (globalCostmap != null) {
      _drawEnhancedCostmap(canvas, size, globalCostmap!, Colors.blue, 'global');
    }
    
    if (localCostmap != null) {
      _drawEnhancedCostmap(canvas, size, localCostmap!, Colors.red, 'local');
    }
    
    // Draw enhanced robot trail
    if (robotTrail.isNotEmpty) {
      _drawEnhancedRobotTrail(canvas, size);
    }
    
    // Draw current robot position with enhanced styling
    if (currentOdometry?.position != null) {
      _drawEnhancedRobot(canvas, size, currentOdometry!);
    }
    
    // Draw map shapes with enhanced styling
    if (mapData?.shapes != null) {
      _drawEnhancedMapShapes(canvas, size, mapData!.shapes);
    }
  }

  void _drawEnhancedGrid(Canvas canvas, Size size) {
    final majorGridPaint = Paint()
      ..color = Colors.grey.withOpacity(0.3)
      ..strokeWidth = 1.0;

    final minorGridPaint = Paint()
      ..color = Colors.grey.withOpacity(0.15)
      ..strokeWidth = 0.5;
    
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
    
    // Draw enhanced axes
    final axisPaint = Paint()
      ..color = theme.primaryColor.withOpacity(0.6)
      ..strokeWidth = 3.0;
    
    final centerX = size.width / 2;
    final centerY = size.height / 2;
    
    // X-axis with gradient effect
    canvas.drawLine(Offset(0, centerY), Offset(size.width, centerY), axisPaint);
    // Y-axis with gradient effect
    canvas.drawLine(Offset(centerX, 0), Offset(centerX, size.height), axisPaint);
  }

  void _drawEnhancedOccupancyGrid(Canvas canvas, Size size, MapData mapData) {
    if (mapData.occupancyData.isEmpty) return;
    
    final info = mapData.info;
    final resolution = info.resolution;
    final origin = info.origin;
    
    final cellSize = resolution * _gridSpacing;
    
    for (int y = 0; y < info.height; y += 2) { // Skip every other cell for performance
      for (int x = 0; x < info.width; x += 2) {
        final index = y * info.width + x;
        if (index >= mapData.occupancyData.length) continue;
        
        final value = mapData.occupancyData[index];
        Color? cellColor;
        
        if (value == -1) {
          continue; // Skip unknown space for better performance
        } else if (value == 0) {
          // Free space with subtle tint
          cellColor = Colors.white.withOpacity(0.8);
        } else if (value == 100) {
          // Occupied space with enhanced contrast
          cellColor = Colors.black87;
        } else {
          // Probabilistic occupancy with smooth gradation
          final gray = 255 - (value * 2.55).toInt();
          cellColor = Color.fromARGB(200, gray, gray, gray);
        }
        
        final paint = Paint()
          ..color = cellColor
          ..style = PaintingStyle.fill;
        
        final screenX = size.width / 2 + (origin.x + x * resolution) * _gridSpacing;
        final screenY = size.height / 2 - (origin.y + y * resolution) * _gridSpacing;
        
        canvas.drawRect(
          Rect.fromLTWH(screenX, screenY, cellSize * 2, cellSize * 2),
          paint,
        );
      }
    }
  }

  void _drawEnhancedCostmap(Canvas canvas, Size size, Map<String, dynamic> costmap, Color baseColor, String type) {
    final info = costmap['info'];
    final data = costmap['data'] as List?;
    
    if (info == null || data == null) return;
    
    final width = info['width'] as int? ?? 0;
    final height = info['height'] as int? ?? 0;
    final resolution = info['resolution'] as double? ?? 0.05;
    final origin = info['origin'];
    
    if (width == 0 || height == 0 || origin == null) return;
    
    final originX = origin['position']?['x'] as double? ?? 0.0;
    final originY = origin['position']?['y'] as double? ?? 0.0;
    
    final cellSize = resolution * _gridSpacing;
    
    // Create gradient colors for different cost values
    final gradientColors = [
      baseColor.withOpacity(0.1),
      baseColor.withOpacity(0.3),
      baseColor.withOpacity(0.6),
      baseColor.withOpacity(costmapOpacity),
    ];
    
    for (int y = 0; y < height && y < data.length ~/ width; y += 2) {
      for (int x = 0; x < width; x += 2) {
        final index = y * width + x;
        if (index >= data.length) continue;
        
        final value = data[index] as int? ?? 0;
        
        if (value == 0) continue; // Skip free space
        
        // Enhanced color mapping with gradients
        Color cellColor;
        if (value >= 100) {
          // Lethal obstacle with glow effect
          cellColor = gradientColors[3];
        } else if (value >= 99) {
          // Inscribed obstacle
          cellColor = gradientColors[2];
        } else if (value >= 50) {
          // High cost area
          cellColor = gradientColors[1];
        } else {
          // Low cost area
          cellColor = gradientColors[0];
        }
        
        final paint = Paint()
          ..color = cellColor
          ..style = PaintingStyle.fill;
        
        // Add slight blur for high-cost areas
        if (value >= 99) {
          paint.maskFilter = const MaskFilter.blur(BlurStyle.normal, 1.0);
        }
        
        final screenX = size.width / 2 + (originX + x * resolution) * _gridSpacing;
        final screenY = size.height / 2 - (originY + y * resolution) * _gridSpacing;
        
        canvas.drawRect(
          Rect.fromLTWH(screenX, screenY, cellSize * 2, cellSize * 2),
          paint,
        );
      }
    }
  }

  void _drawEnhancedRobotTrail(Canvas canvas, Size size) {
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
        final y = size.height / 2 - point.y * _gridSpacing;
        segmentPath.lineTo(x, y);
      }
      
      canvas.drawPath(segmentPath, trailPaint);
    }
    
    // Draw recent trail points with glow
    final recentCount = math.min(20, robotTrail.length);
    final pointPaint = Paint()
      ..style = PaintingStyle.fill;
    
    for (int i = robotTrail.length - recentCount; i < robotTrail.length; i++) {
      final point = robotTrail[i];
      final x = size.width / 2 + point.x * _gridSpacing;
      final y = size.height / 2 - point.y * _gridSpacing;
      
      final age = (robotTrail.length - i) / recentCount;
      final opacity = 1.0 - age * 0.8;
      final radius = 1.5 * (1.0 - age * 0.5);
      
      pointPaint.color = Colors.orange.withOpacity(opacity);
      canvas.drawCircle(Offset(x, y), radius, pointPaint);
    }
  }

  void _drawEnhancedRobot(Canvas canvas, Size size, OdometryData odometry) {
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
      ..shader = bodyGradient.createShader(Rect.fromCircle(center: Offset(x, y), radius: robotSize))
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
    
    canvas.drawLine(
      Offset(x + 1, y + 1), 
      Offset(directionX + 1, directionY + 1), 
      shadowPaint
    );
    
    final directionPaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 3.0
      ..strokeCap = StrokeCap.round;
    
    canvas.drawLine(Offset(x, y), Offset(directionX, directionY), directionPaint);
    
    // Draw arrowhead
    final arrowPaint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.fill;
    
    final arrowPath = Path();
    final arrowSize = robotSize * 0.3;
    arrowPath.moveTo(directionX, directionY);
    arrowPath.lineTo(
      directionX - arrowSize * math.cos(yaw - 2.5), 
      directionY + arrowSize * math.sin(yaw - 2.5)
    );
    arrowPath.lineTo(
      directionX - arrowSize * math.cos(yaw + 2.5), 
      directionY + arrowSize * math.sin(yaw + 2.5)
    );
    arrowPath.close();
    canvas.drawPath(arrowPath, arrowPaint);
    
    // Enhanced position text with background
    final textPainter = TextPainter(
      text: TextSpan(
        text: '(${position.x.toStringAsFixed(1)}, ${position.y.toStringAsFixed(1)})',
        style: TextStyle(
          color: Colors.white,
          fontSize: 11,
          fontWeight: FontWeight.bold,
          shadows: [
            Shadow(
              color: Colors.black.withOpacity(0.7),
              offset: const Offset(1, 1),
              blurRadius: 2,
            ),
          ],
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    
    textPainter.layout();
    
    // Draw text background
    final textBg = Paint()
      ..color = Colors.black.withOpacity(0.6)
      ..style = PaintingStyle.fill;
    
    final textRect = Rect.fromCenter(
      center: Offset(x, y + robotSize + 15),
      width: textPainter.width + 8,
      height: textPainter.height + 4,
    );
    
    canvas.drawRRect(
      RRect.fromRectAndRadius(textRect, const Radius.circular(4)),
      textBg,
    );
    
    textPainter.paint(
      canvas, 
      Offset(x - textPainter.width / 2, y + robotSize + 13)
    );
  }

  void _drawEnhancedMapShapes(Canvas canvas, Size size, List<MapShape> shapes) {
    for (final shape in shapes) {
      if (shape.points.isEmpty) continue;
      
      _drawEnhancedShape(canvas, size, shape);
    }
  }

  void _drawEnhancedShape(Canvas canvas, Size size, MapShape shape) {
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
    
    // Enhanced gradient fill
    final fillPaint = Paint()
      ..color = shapeColor.withOpacity(0.3)
      ..style = PaintingStyle.fill;
    
    // Enhanced outline with shadow
    final shadowPaint = Paint()
      ..color = Colors.black.withOpacity(0.2)
      ..strokeWidth = _shapeOutlineWidth + 1
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;
    
    final outlinePaint = Paint()
      ..color = shapeColor
      ..strokeWidth = _shapeOutlineWidth
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;
    
    if (shape.points.length == 1) {
      // Enhanced single point rendering
      final point = shape.points.first;
      final x = size.width / 2 + point.x * _gridSpacing;
      final y = size.height / 2 - point.y * _gridSpacing;
      
      _drawEnhancedShapeIcon(canvas, Offset(x, y), shape.type, shapeColor);
      
    } else if (shape.points.length == 2) {
      // Enhanced line rendering
      final start = shape.points.first;
      final end = shape.points.last;
      
      final startX = size.width / 2 + start.x * _gridSpacing;
      final startY = size.height / 2 - start.y * _gridSpacing;
      final endX = size.width / 2 + end.x * _gridSpacing;
      final endY = size.height / 2 - end.y * _gridSpacing;
      
      // Draw shadow
      canvas.drawLine(
        Offset(startX + 1, startY + 1), 
        Offset(endX + 1, endY + 1), 
        shadowPaint
      );
      
      // Draw line
      canvas.drawLine(Offset(startX, startY), Offset(endX, endY), outlinePaint);
      
      // Enhanced endpoints
      final endpointPaint = Paint()
        ..color = shapeColor
        ..style = PaintingStyle.fill;
      
      canvas.drawCircle(Offset(startX, startY), 6, endpointPaint);
      canvas.drawCircle(Offset(endX, endY), 6, endpointPaint);
      
    } else {
      // Enhanced polygon rendering
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
        // Draw shadow
        final shadowPath = Path.from(path);
        shadowPath.shift(const Offset(2, 2));
        canvas.drawPath(shadowPath, shadowPaint);
        
        // Draw fill
        canvas.drawPath(path, fillPaint);
      }
      
      // Draw outline
      canvas.drawPath(path, outlinePaint);
      
      // Enhanced point markers
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
    
    // Enhanced label rendering
    final center = shape.center;
    final centerX = size.width / 2 + center.x * _gridSpacing;
    final centerY = size.height / 2 - center.y * _gridSpacing;
    
    _drawEnhancedShapeLabel(canvas, Offset(centerX, centerY), shape.name, shapeColor);
  }

  void _drawEnhancedShapeIcon(Canvas canvas, Offset position, String type, Color color) {
    final iconSize = 14.0;
    
    // Enhanced shadow
    final shadowPaint = Paint()
      ..color = Colors.black.withOpacity(0.3)
      ..style = PaintingStyle.fill;
    
    final paint = Paint()
      ..color = color
      ..style = PaintingStyle.fill;
    
    final outlinePaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 2.0
      ..style = PaintingStyle.stroke;
    
    switch (type) {
      case 'pickup':
        // Enhanced square with gradient
        final rect = Rect.fromCenter(center: position, width: iconSize, height: iconSize);
        final shadowRect = rect.shift(const Offset(1, 1));
        
        canvas.drawRect(shadowRect, shadowPaint);
        canvas.drawRect(rect, paint);
        canvas.drawRect(rect, outlinePaint);
        break;
        
      case 'drop':
        // Enhanced circle with glow
        canvas.drawCircle(position.translate(1, 1), iconSize / 2, shadowPaint);
        canvas.drawCircle(position, iconSize / 2, paint);
        canvas.drawCircle(position, iconSize / 2, outlinePaint);
        break;
        
      case 'charging':
        // Enhanced triangle with gradient
        final path = Path()
          ..moveTo(position.dx, position.dy - iconSize / 2)
          ..lineTo(position.dx + iconSize / 2, position.dy + iconSize / 2)
          ..lineTo(position.dx - iconSize / 2, position.dy + iconSize / 2)
          ..close();
        
        final shadowPath = Path.from(path);
        shadowPath.shift(const Offset(1, 1));
        
        canvas.drawPath(shadowPath, shadowPaint);
        canvas.drawPath(path, paint);
        canvas.drawPath(path, outlinePaint);
        break;
        
      case 'obstacle':
        // Enhanced X with thicker lines
        final linePaint = Paint()
          ..color = color
          ..strokeWidth = 4.0
          ..strokeCap = StrokeCap.round;
        
        final shadowLinePaint = Paint()
          ..color = Colors.black.withOpacity(0.3)
          ..strokeWidth = 5.0
          ..strokeCap = StrokeCap.round;
        
        // Draw shadow
        canvas.drawLine(
          Offset(position.dx - iconSize / 2 + 1, position.dy - iconSize / 2 + 1),
          Offset(position.dx + iconSize / 2 + 1, position.dy + iconSize / 2 + 1),
          shadowLinePaint,
        );
        canvas.drawLine(
          Offset(position.dx - iconSize / 2 + 1, position.dy + iconSize / 2 + 1),
          Offset(position.dx + iconSize / 2 + 1, position.dy - iconSize / 2 + 1),
          shadowLinePaint,
        );
        
        // Draw X
        canvas.drawLine(
          Offset(position.dx - iconSize / 2, position.dy - iconSize / 2),
          Offset(position.dx + iconSize / 2, position.dy + iconSize / 2),
          linePaint,
        );
        canvas.drawLine(
          Offset(position.dx - iconSize / 2, position.dy + iconSize / 2),
          Offset(position.dx + iconSize / 2, position.dy - iconSize / 2),
          linePaint,
        );
        break;
        
      default: // waypoint
        // Enhanced diamond
        final path = Path()
          ..moveTo(position.dx, position.dy - iconSize / 2)
          ..lineTo(position.dx + iconSize / 2, position.dy)
          ..lineTo(position.dx, position.dy + iconSize / 2)
          ..lineTo(position.dx - iconSize / 2, position.dy)
          ..close();
        
        final shadowPath = Path.from(path);
        shadowPath.shift(const Offset(1, 1));
        
        canvas.drawPath(shadowPath, shadowPaint);
        canvas.drawPath(path, paint);
        canvas.drawPath(path, outlinePaint);
    }
  }

  void _drawEnhancedShapeLabel(Canvas canvas, Offset position, String label, Color color) {
    if (label.isEmpty) return;
    
    final textPainter = TextPainter(
      text: TextSpan(
        text: label,
        style: TextStyle(
          color: Colors.white,
          fontSize: 11,
          fontWeight: FontWeight.bold,
          shadows: [
            Shadow(
              color: Colors.black.withOpacity(0.8),
              offset: const Offset(1, 1),
              blurRadius: 3,
            ),
          ],
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    
    textPainter.layout();
    
    // Enhanced text background with rounded corners
    final textBounds = Rect.fromCenter(
      center: Offset(position.dx, position.dy + 25),
      width: textPainter.width + 12,
      height: textPainter.height + 6,
    );
    
    final bgPaint = Paint()
      ..color = color.withOpacity(0.9)
      ..style = PaintingStyle.fill;
    
    final bgOutlinePaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 1.0
      ..style = PaintingStyle.stroke;
    
    final rrect = RRect.fromRectAndRadius(textBounds, const Radius.circular(6));
    
    // Draw shadow
    final shadowRRect = rrect.shift(const Offset(1, 1));
    final shadowPaint = Paint()
      ..color = Colors.black.withOpacity(0.3)
      ..style = PaintingStyle.fill;
    canvas.drawRRect(shadowRRect, shadowPaint);
    
    // Draw background
    canvas.drawRRect(rrect, bgPaint);
    canvas.drawRRect(rrect, bgOutlinePaint);
    
    // Draw text
    textPainter.paint(
      canvas,
      Offset(
        position.dx - textPainter.width / 2,
        position.dy + 25 - textPainter.height / 2,
      ),
    );
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true; // Always repaint for live updates
  }
}