// widgets/live_maping_canvas.dart - Enhanced Live Mapping Canvas with Costmaps
// Compatible with provided map_data.dart structure
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
  
  // Map viewport settings
  double _scale = 1.0;
  Offset _translation = Offset.zero;
  bool _autoCenter = true;
  
  // Drawing state
  bool _isDrawing = false;
  List<Offset> _currentStroke = [];

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
    
    if (widget.mappingActive) {
      _robotAnimationController.repeat(reverse: true);
    }
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
    
    // Auto-center on robot if enabled and position changed
    if (_autoCenter && 
        widget.currentOdometry != null && 
        oldWidget.currentOdometry?.position != widget.currentOdometry?.position) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        _centerOnRobot();
      });
    }
  }

  @override
  void dispose() {
    _transformationController.dispose();
    _robotAnimationController.dispose();
    super.dispose();
  }

  void _centerOnRobot() {
    if (widget.currentOdometry?.position == null) return;
    
    final robotPos = widget.currentOdometry!.position;
    final screenCenter = MediaQuery.of(context).size.center(Offset.zero);
    
    // Convert robot position to screen coordinates and center
    final targetTransform = Matrix4.identity()
      ..translate(screenCenter.dx - robotPos.x * 20, screenCenter.dy + robotPos.y * 20)
      ..scale(_scale);
    
    _transformationController.value = targetTransform;
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        // Main canvas
        InteractiveViewer(
          transformationController: _transformationController,
          boundaryMargin: const EdgeInsets.all(100),
          minScale: 0.1,
          maxScale: 10.0,
          onInteractionUpdate: (details) {
            setState(() {
              _scale = _transformationController.value.getMaxScaleOnAxis();
            });
          },
          child: Container(
            width: 2000,
            height: 2000,
            child: CustomPaint(
              painter: LiveMapPainter(
                mapData: widget.mapData,
                currentOdometry: widget.currentOdometry,
                robotTrail: widget.robotTrail,
                globalCostmap: widget.globalCostmap,
                localCostmap: widget.localCostmap,
                showOccupancyGrid: widget.showOccupancyGrid,
                costmapOpacity: widget.costmapOpacity,
                robotScale: _robotPulseAnimation.value,
                mappingActive: widget.mappingActive,
              ),
              size: const Size(2000, 2000),
            ),
          ),
        ),
        
        // Controls overlay
        Positioned(
          top: 16,
          right: 16,
          child: Column(
            children: [
              // Auto-center toggle
              FloatingActionButton(
                mini: true,
                backgroundColor: _autoCenter ? Colors.blue : Colors.grey,
                onPressed: () {
                  setState(() {
                    _autoCenter = !_autoCenter;
                  });
                  if (_autoCenter) {
                    _centerOnRobot();
                  }
                },
                child: Icon(
                  Icons.my_location,
                  color: Colors.white,
                ),
                tooltip: 'Auto Center on Robot',
              ),
              
              const SizedBox(height: 8),
              
              // Zoom controls
              Container(
                decoration: BoxDecoration(
                  color: Colors.black54,
                  borderRadius: BorderRadius.circular(20),
                ),
                child: Column(
                  children: [
                    IconButton(
                      onPressed: () {
                        final currentTransform = _transformationController.value;
                        final newTransform = currentTransform.clone()..scale(1.2);
                        _transformationController.value = newTransform;
                      },
                      icon: const Icon(Icons.zoom_in, color: Colors.white),
                    ),
                    IconButton(
                      onPressed: () {
                        final currentTransform = _transformationController.value;
                        final newTransform = currentTransform.clone()..scale(0.8);
                        _transformationController.value = newTransform;
                      },
                      icon: const Icon(Icons.zoom_out, color: Colors.white),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
        
        // Status overlay
        Positioned(
          bottom: 16,
          left: 16,
          child: Container(
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              color: Colors.black54,
              borderRadius: BorderRadius.circular(8),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Live Map Status',
                  style: const TextStyle(
                    color: Colors.white,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                Text(
                  'Trail: ${widget.robotTrail.length} points',
                  style: const TextStyle(color: Colors.white70, fontSize: 12),
                ),
                if (widget.mapData != null)
                  Text(
                    'Shapes: ${widget.mapData!.shapes.length}',
                    style: const TextStyle(color: Colors.white70, fontSize: 12),
                  ),
                if (widget.globalCostmap != null)
                  Text(
                    'Global: ${widget.globalCostmap!['info']?['width'] ?? 'N/A'}x${widget.globalCostmap!['info']?['height'] ?? 'N/A'}',
                    style: const TextStyle(color: Colors.white70, fontSize: 12),
                  ),
                if (widget.localCostmap != null)
                  Text(
                    'Local: ${widget.localCostmap!['info']?['width'] ?? 'N/A'}x${widget.localCostmap!['info']?['height'] ?? 'N/A'}',
                    style: const TextStyle(color: Colors.white70, fontSize: 12),
                  ),
                Text(
                  'Scale: ${_scale.toStringAsFixed(2)}x',
                  style: const TextStyle(color: Colors.white70, fontSize: 12),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }
}

class LiveMapPainter extends CustomPainter {
  final MapData? mapData;
  final OdometryData? currentOdometry;
  final List<Position> robotTrail;
  final Map<String, dynamic>? globalCostmap;
  final Map<String, dynamic>? localCostmap;
  final bool showOccupancyGrid;
  final double costmapOpacity;
  final double robotScale;
  final bool mappingActive;

  LiveMapPainter({
    this.mapData,
    this.currentOdometry,
    this.robotTrail = const [],
    this.globalCostmap,
    this.localCostmap,
    this.showOccupancyGrid = true,
    this.costmapOpacity = 0.7,
    this.robotScale = 1.0,
    this.mappingActive = false,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final centerX = size.width / 2;
    final centerY = size.height / 2;
    
    // Draw grid background
    _drawGrid(canvas, size);
    
    // Draw occupancy grid map if available and enabled
    if (showOccupancyGrid && mapData != null) {
      _drawOccupancyGrid(canvas, size, mapData!);
    }
    
    // Draw global costmap if available
    if (globalCostmap != null) {
      _drawCostmap(canvas, size, globalCostmap!, Colors.blue, 'global');
    }
    
    // Draw local costmap if available
    if (localCostmap != null) {
      _drawCostmap(canvas, size, localCostmap!, Colors.red, 'local');
    }
    
    // Draw robot trail
    if (robotTrail.isNotEmpty) {
      _drawRobotTrail(canvas, size);
    }
    
    // Draw current robot position
    if (currentOdometry?.position != null) {
      _drawRobot(canvas, size, currentOdometry!);
    }
    
    // Draw map shapes/waypoints if available
    if (mapData?.shapes != null) {
      _drawMapShapes(canvas, size, mapData!.shapes);
    }
  }

  void _drawGrid(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.grey.withOpacity(0.2)
      ..strokeWidth = 0.5;
    
    const gridSpacing = 20.0; // 1 meter = 20 pixels
    
    // Draw vertical lines
    for (double x = 0; x < size.width; x += gridSpacing) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
    }
    
    // Draw horizontal lines
    for (double y = 0; y < size.height; y += gridSpacing) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paint);
    }
    
    // Draw axes
    final axisPaint = Paint()
      ..color = Colors.grey.withOpacity(0.5)
      ..strokeWidth = 2.0;
    
    final centerX = size.width / 2;
    final centerY = size.height / 2;
    
    // X-axis
    canvas.drawLine(Offset(0, centerY), Offset(size.width, centerY), axisPaint);
    // Y-axis
    canvas.drawLine(Offset(centerX, 0), Offset(centerX, size.height), axisPaint);
  }

  void _drawOccupancyGrid(Canvas canvas, Size size, MapData mapData) {
    if (mapData.occupancyData.isEmpty) return;
    
    final info = mapData.info;
    final resolution = info.resolution;
    final origin = info.origin;
    
    final cellSize = resolution * 20; // Scale factor for display
    
    for (int y = 0; y < info.height; y++) {
      for (int x = 0; x < info.width; x++) {
        final index = y * info.width + x;
        if (index >= mapData.occupancyData.length) continue;
        
        final value = mapData.occupancyData[index];
        Color? cellColor;
        
        if (value == -1) {
          // Unknown space - skip or draw light gray
          continue;
        } else if (value == 0) {
          // Free space
          cellColor = Colors.white;
        } else if (value == 100) {
          // Occupied space
          cellColor = Colors.black;
        } else {
          // Probabilistic occupancy
          final gray = 255 - (value * 2.55).toInt();
          cellColor = Color.fromARGB(255, gray, gray, gray);
        }
        
        final paint = Paint()..color = cellColor;
        
        final screenX = size.width / 2 + (origin.x + x * resolution) * 20;
        final screenY = size.height / 2 - (origin.y + y * resolution) * 20;
        
        canvas.drawRect(
          Rect.fromLTWH(screenX, screenY, cellSize, cellSize),
          paint,
        );
      }
    }
  }

  void _drawCostmap(Canvas canvas, Size size, Map<String, dynamic> costmap, Color baseColor, String type) {
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
    
    final cellSize = resolution * 20; // Scale factor for display
    
    for (int y = 0; y < height && y < data.length ~/ width; y++) {
      for (int x = 0; x < width; x++) {
        final index = y * width + x;
        if (index >= data.length) continue;
        
        final value = data[index] as int? ?? 0;
        
        if (value == 0) continue; // Skip free space
        
        // Color based on cost value
        Color cellColor;
        if (value == 100) {
          // Lethal obstacle
          cellColor = baseColor.withOpacity(costmapOpacity);
        } else if (value == 99) {
          // Inscribed obstacle
          cellColor = baseColor.withOpacity(costmapOpacity * 0.8);
        } else {
          // Inflated obstacle or other cost
          final opacity = (value / 100.0) * costmapOpacity * 0.6;
          cellColor = baseColor.withOpacity(opacity);
        }
        
        final paint = Paint()..color = cellColor;
        
        final screenX = size.width / 2 + (originX + x * resolution) * 20;
        final screenY = size.height / 2 - (originY + y * resolution) * 20;
        
        canvas.drawRect(
          Rect.fromLTWH(screenX, screenY, cellSize, cellSize),
          paint,
        );
      }
    }
  }

  void _drawRobotTrail(Canvas canvas, Size size) {
    if (robotTrail.length < 2) return;
    
    final paint = Paint()
      ..color = Colors.orange
      ..strokeWidth = 3.0
      ..style = PaintingStyle.stroke;
    
    final path = Path();
    
    // Convert first point
    final firstPoint = robotTrail.first;
    final firstX = size.width / 2 + firstPoint.x * 20;
    final firstY = size.height / 2 - firstPoint.y * 20;
    path.moveTo(firstX, firstY);
    
    // Add remaining points
    for (int i = 1; i < robotTrail.length; i++) {
      final point = robotTrail[i];
      final x = size.width / 2 + point.x * 20;
      final y = size.height / 2 - point.y * 20;
      path.lineTo(x, y);
    }
    
    canvas.drawPath(path, paint);
    
    // Draw trail points
    final pointPaint = Paint()
      ..color = Colors.orange.withOpacity(0.7)
      ..style = PaintingStyle.fill;
    
    for (final point in robotTrail) {
      final x = size.width / 2 + point.x * 20;
      final y = size.height / 2 - point.y * 20;
      canvas.drawCircle(Offset(x, y), 2.0, pointPaint);
    }
  }

  void _drawRobot(Canvas canvas, Size size, OdometryData odometry) {
    final position = odometry.position;
    final orientation = odometry.orientation;
    
    final x = size.width / 2 + position.x * 20;
    final y = size.height / 2 - position.y * 20;
    
    final robotSize = 15.0 * robotScale;
    
    // Robot body
    final bodyPaint = Paint()
      ..color = mappingActive ? Colors.green : Colors.blue
      ..style = PaintingStyle.fill;
    
    canvas.drawCircle(Offset(x, y), robotSize, bodyPaint);
    
    // Robot outline
    final outlinePaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 2.0
      ..style = PaintingStyle.stroke;
    
    canvas.drawCircle(Offset(x, y), robotSize, outlinePaint);
    
    // Direction indicator
    final yaw = orientation.yaw;
    final directionLength = robotSize * 1.2;
    final directionX = x + math.cos(yaw) * directionLength;
    final directionY = y - math.sin(yaw) * directionLength;
    
    final directionPaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 3.0
      ..strokeCap = StrokeCap.round;
    
    canvas.drawLine(Offset(x, y), Offset(directionX, directionY), directionPaint);
    
    // Position text
    final textPainter = TextPainter(
      text: TextSpan(
        text: '(${position.x.toStringAsFixed(1)}, ${position.y.toStringAsFixed(1)})',
        style: const TextStyle(
          color: Colors.black,
          fontSize: 12,
          backgroundColor: Colors.white,
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    
    textPainter.layout();
    textPainter.paint(canvas, Offset(x - textPainter.width / 2, y + robotSize + 5));
  }

  void _drawMapShapes(Canvas canvas, Size size, List<MapShape> shapes) {
    for (final shape in shapes) {
      if (shape.points.isEmpty) continue;
      
      // Parse color (handle both with and without # prefix)
      Color shapeColor;
      try {
        String colorString = shape.color;
        if (colorString.startsWith('#')) {
          colorString = colorString.substring(1);
        }
        if (colorString.length == 6) {
          colorString = 'FF$colorString'; // Add alpha if missing
        }
        final colorValue = int.tryParse('0x$colorString') ?? 0xFF000000;
        shapeColor = Color(colorValue);
      } catch (e) {
        shapeColor = Colors.purple;
      }
      
      // Draw shape based on number of points and type
      final paint = Paint()
        ..color = shapeColor.withOpacity(0.7)
        ..style = PaintingStyle.fill;
      
      final outlinePaint = Paint()
        ..color = shapeColor
        ..strokeWidth = 2.0
        ..style = PaintingStyle.stroke;
      
      if (shape.points.length == 1) {
        // Single point - draw as circle
        final point = shape.points.first;
        final x = size.width / 2 + point.x * 20;
        final y = size.height / 2 - point.y * 20;
        
        _drawShapeIcon(canvas, Offset(x, y), shape.type, shapeColor);
        
      } else if (shape.points.length == 2) {
        // Two points - draw as line
        final start = shape.points.first;
        final end = shape.points.last;
        
        final startX = size.width / 2 + start.x * 20;
        final startY = size.height / 2 - start.y * 20;
        final endX = size.width / 2 + end.x * 20;
        final endY = size.height / 2 - end.y * 20;
        
        final linePaint = Paint()
          ..color = shapeColor
          ..strokeWidth = 4.0
          ..strokeCap = StrokeCap.round;
        
        canvas.drawLine(Offset(startX, startY), Offset(endX, endY), linePaint);
        
        // Draw endpoints
        canvas.drawCircle(Offset(startX, startY), 6, paint);
        canvas.drawCircle(Offset(endX, endY), 6, paint);
        
      } else {
        // Multiple points - draw as path/polygon
        final path = Path();
        
        // Convert first point
        final firstPoint = shape.points.first;
        final firstX = size.width / 2 + firstPoint.x * 20;
        final firstY = size.height / 2 - firstPoint.y * 20;
        path.moveTo(firstX, firstY);
        
        // Add remaining points
        for (int i = 1; i < shape.points.length; i++) {
          final point = shape.points[i];
          final x = size.width / 2 + point.x * 20;
          final y = size.height / 2 - point.y * 20;
          path.lineTo(x, y);
        }
        
        // Close path if it's a polygon-like shape
        if (shape.type != 'waypoint' && shape.points.length > 2) {
          path.close();
          canvas.drawPath(path, paint);
        }
        
        // Draw path outline
        final pathOutlinePaint = Paint()
          ..color = shapeColor
          ..strokeWidth = 3.0
          ..style = PaintingStyle.stroke
          ..strokeCap = StrokeCap.round
          ..strokeJoin = StrokeJoin.round;
        
        canvas.drawPath(path, pathOutlinePaint);
        
        // Draw point markers
        for (final point in shape.points) {
          final x = size.width / 2 + point.x * 20;
          final y = size.height / 2 - point.y * 20;
          
          final pointPaint = Paint()
            ..color = shapeColor
            ..style = PaintingStyle.fill;
          
          canvas.drawCircle(Offset(x, y), 4, pointPaint);
        }
      }
      
      // Draw label at center of shape
      final center = shape.center;
      final centerX = size.width / 2 + center.x * 20;
      final centerY = size.height / 2 - center.y * 20;
      
      _drawShapeLabel(canvas, Offset(centerX, centerY), shape.name, shapeColor);
    }
  }

  void _drawShapeIcon(Canvas canvas, Offset position, String type, Color color) {
    final paint = Paint()
      ..color = color.withOpacity(0.8)
      ..style = PaintingStyle.fill;
    
    final outlinePaint = Paint()
      ..color = color
      ..strokeWidth = 2.0
      ..style = PaintingStyle.stroke;
    
    switch (type) {
      case 'pickup':
        // Draw a square
        canvas.drawRect(
          Rect.fromCenter(center: position, width: 16, height: 16),
          paint,
        );
        canvas.drawRect(
          Rect.fromCenter(center: position, width: 16, height: 16),
          outlinePaint,
        );
        break;
        
      case 'drop':
        // Draw a circle
        canvas.drawCircle(position, 8, paint);
        canvas.drawCircle(position, 8, outlinePaint);
        break;
        
      case 'charging':
        // Draw a triangle (charging symbol)
        final path = Path()
          ..moveTo(position.dx - 8, position.dy + 8)
          ..lineTo(position.dx + 8, position.dy + 8)
          ..lineTo(position.dx, position.dy - 8)
          ..close();
        canvas.drawPath(path, paint);
        canvas.drawPath(path, outlinePaint);
        break;
        
      case 'obstacle':
        // Draw an X
        final linePaint = Paint()
          ..color = color
          ..strokeWidth = 3.0
          ..strokeCap = StrokeCap.round;
        
        canvas.drawLine(
          Offset(position.dx - 6, position.dy - 6),
          Offset(position.dx + 6, position.dy + 6),
          linePaint,
        );
        canvas.drawLine(
          Offset(position.dx - 6, position.dy + 6),
          Offset(position.dx + 6, position.dy - 6),
          linePaint,
        );
        break;
        
      default: // waypoint
        // Draw a diamond
        final path = Path()
          ..moveTo(position.dx, position.dy - 8)
          ..lineTo(position.dx + 8, position.dy)
          ..lineTo(position.dx, position.dy + 8)
          ..lineTo(position.dx - 8, position.dy)
          ..close();
        canvas.drawPath(path, paint);
        canvas.drawPath(path, outlinePaint);
    }
  }

  void _drawShapeLabel(Canvas canvas, Offset position, String label, Color color) {
    if (label.isEmpty) return;
    
    final textPainter = TextPainter(
      text: TextSpan(
        text: label,
        style: TextStyle(
          color: Colors.black,
          fontSize: 10,
          fontWeight: FontWeight.bold,
          backgroundColor: Colors.white.withOpacity(0.8),
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    
    textPainter.layout();
    
    // Draw text background
    final textBounds = Rect.fromCenter(
      center: Offset(position.dx, position.dy + 20),
      width: textPainter.width + 4,
      height: textPainter.height + 2,
    );
    
    final bgPaint = Paint()
      ..color = Colors.white.withOpacity(0.9)
      ..style = PaintingStyle.fill;
    
    canvas.drawRRect(
      RRect.fromRectAndRadius(textBounds, const Radius.circular(2)),
      bgPaint,
    );
    
    // Draw text
    textPainter.paint(
      canvas,
      Offset(
        position.dx - textPainter.width / 2,
        position.dy + 20 - textPainter.height / 2,
      ),
    );
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true; // Always repaint for live updates
  }
}