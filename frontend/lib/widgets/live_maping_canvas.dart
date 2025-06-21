// Create new file: lib/widgets/live_mapping_canvas.dart
import 'package:flutter/material.dart';
import '../models/map_data.dart';
import '../models/odom.dart';
import 'dart:math' as math;

class LiveMappingCanvas extends StatefulWidget {
  final MapData? mapData;
  final OdometryData? currentOdometry;
  final List<Position> robotTrail;
  final bool mappingActive;
  final Function(MapData)? onMapChanged;
  final String? deviceId;

  const LiveMappingCanvas({
    Key? key,
    this.mapData,
    this.currentOdometry,
    this.robotTrail = const [],
    this.mappingActive = false,
    this.onMapChanged,
    this.deviceId,
  }) : super(key: key);

  @override
  _LiveMappingCanvasState createState() => _LiveMappingCanvasState();
}

class _LiveMappingCanvasState extends State<LiveMappingCanvas>
    with TickerProviderStateMixin {
  late AnimationController _pulseController;
  late Animation<double> _pulseAnimation;
  
  double _scale = 1.0;
  Offset _panOffset = Offset.zero;
  
  @override
  void initState() {
    super.initState();
    
    // Robot pulse animation when mapping
    _pulseController = AnimationController(
      duration: Duration(milliseconds: 1500),
      vsync: this,
    );
    _pulseAnimation = Tween<double>(begin: 0.8, end: 1.2).animate(
      CurvedAnimation(parent: _pulseController, curve: Curves.easeInOut),
    );
    
    if (widget.mappingActive) {
      _pulseController.repeat(reverse: true);
    }
  }

  @override
  void didUpdateWidget(LiveMappingCanvas oldWidget) {
    super.didUpdateWidget(oldWidget);
    
    // Start/stop pulse animation based on mapping state
    if (widget.mappingActive && !oldWidget.mappingActive) {
      _pulseController.repeat(reverse: true);
    } else if (!widget.mappingActive && oldWidget.mappingActive) {
      _pulseController.stop();
      _pulseController.reset();
    }
  }

  @override
  void dispose() {
    _pulseController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        border: Border.all(color: Colors.grey[300]!),
        borderRadius: BorderRadius.circular(8),
      ),
      child: ClipRRect(
        borderRadius: BorderRadius.circular(8),
        child: GestureDetector(
          onScaleStart: _onScaleStart,
          onScaleUpdate: _onScaleUpdate,
          child: CustomPaint(
            painter: LiveMappingPainter(
              mapData: widget.mapData,
              currentOdometry: widget.currentOdometry,
              robotTrail: widget.robotTrail,
              mappingActive: widget.mappingActive,
              scale: _scale,
              panOffset: _panOffset,
              pulseAnimation: _pulseAnimation,
            ),
            size: Size.infinite,
          ),
        ),
      ),
    );
  }

  void _onScaleStart(ScaleStartDetails details) {
    // Handle pan/zoom start
  }

  void _onScaleUpdate(ScaleUpdateDetails details) {
    setState(() {
      _scale = (_scale * details.scale).clamp(0.1, 5.0);
      _panOffset += details.focalPointDelta;
    });
  }
}

class LiveMappingPainter extends CustomPainter {
  final MapData? mapData;
  final OdometryData? currentOdometry;
  final List<Position> robotTrail;
  final bool mappingActive;
  final double scale;
  final Offset panOffset;
  final Animation<double> pulseAnimation;

  LiveMappingPainter({
    this.mapData,
    this.currentOdometry,
    this.robotTrail = const [],
    this.mappingActive = false,
    this.scale = 1.0,
    this.panOffset = Offset.zero,
    required this.pulseAnimation,
  }) : super(repaint: pulseAnimation);

  @override
  void paint(Canvas canvas, Size size) {
    // Transform canvas for pan/zoom
    canvas.save();
    canvas.translate(size.width / 2 + panOffset.dx, size.height / 2 + panOffset.dy);
    canvas.scale(scale);
    
    // Draw grid
    _drawGrid(canvas, size);
    
    // Draw map if available
    if (mapData != null) {
      _drawOccupancyGrid(canvas, size);
      _drawMapShapes(canvas, size);
    }
    
    // Draw robot trail
    _drawRobotTrail(canvas, size);
    
    // Draw current robot position
    if (currentOdometry != null) {
      _drawRobotPosition(canvas, size);
    }
    
    canvas.restore();
    
    // Draw UI overlay (not affected by pan/zoom)
    _drawUIOverlay(canvas, size);
  }

  void _drawGrid(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.grey[300]!.withOpacity(0.5)
      ..strokeWidth = 1;

    const gridSpacing = 50.0; // 1 meter = 50 pixels
    
    // Draw vertical lines
    for (double x = -size.width; x < size.width; x += gridSpacing) {
      canvas.drawLine(
        Offset(x, -size.height),
        Offset(x, size.height),
        paint,
      );
    }
    
    // Draw horizontal lines
    for (double y = -size.height; y < size.height; y += gridSpacing) {
      canvas.drawLine(
        Offset(-size.width, y),
        Offset(size.width, y),
        paint,
      );
    }
  }

  void _drawOccupancyGrid(Canvas canvas, Size size) {
    if (mapData == null) return;
    
    // Draw occupancy grid (simplified)
    final paint = Paint();
    const pixelSize = 2.0;
    
    for (int y = 0; y < mapData!.info.height && y < 500; y += 2) {
      for (int x = 0; x < mapData!.info.width && x < 500; x += 2) {
        final occupancyValue = mapData!.getOccupancyAt(x, y);
        
        if (occupancyValue >= 0) {
          // Convert map coordinates to canvas coordinates
          final canvasX = (x - mapData!.info.width / 2) * pixelSize;
          final canvasY = (y - mapData!.info.height / 2) * pixelSize;
          
          if (occupancyValue > 50) {
            paint.color = Colors.black.withOpacity(0.8); // Obstacle
          } else {
            paint.color = Colors.grey[200]!.withOpacity(0.3); // Free space
          }
          
          canvas.drawRect(
            Rect.fromLTWH(canvasX, canvasY, pixelSize, pixelSize),
            paint,
          );
        }
      }
    }
  }

  void _drawMapShapes(Canvas canvas, Size size) {
    if (mapData == null) return;
    
    for (final shape in mapData!.shapes) {
      _drawMapShape(canvas, shape);
    }
  }

  void _drawMapShape(Canvas canvas, MapShape shape) {
    final paint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;
      
    try {
      // Parse color safely
      final colorString = shape.color.replaceAll('#', '');
      final colorValue = int.tryParse('0xFF$colorString') ?? 0xFF0000FF;
      paint.color = Color(colorValue);
    } catch (e) {
      paint.color = Colors.blue;
    }

    if (shape.points.isNotEmpty) {
      final path = Path();
      
      // Convert world coordinates to canvas coordinates
      final firstPoint = _worldToCanvas(shape.points.first);
      path.moveTo(firstPoint.dx, firstPoint.dy);
      
      for (int i = 1; i < shape.points.length; i++) {
        final point = _worldToCanvas(shape.points[i]);
        path.lineTo(point.dx, point.dy);
      }
      
      if (shape.points.length > 2) {
        path.close();
      }
      
      canvas.drawPath(path, paint);
      
      // Draw shape label
      final center = _worldToCanvas(shape.center);
      _drawText(canvas, shape.name, center, Colors.black);
    }
  }

  void _drawRobotTrail(Canvas canvas, Size size) {
    if (robotTrail.length < 2) return;
    
    final paint = Paint()
      ..color = mappingActive ? Colors.green : Colors.blue
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;
    
    final path = Path();
    
    // Draw trail path
    final firstPoint = _worldToCanvas(robotTrail.first);
    path.moveTo(firstPoint.dx, firstPoint.dy);
    
    for (int i = 1; i < robotTrail.length; i++) {
      final point = _worldToCanvas(robotTrail[i]);
      path.lineTo(point.dx, point.dy);
    }
    
    canvas.drawPath(path, paint);
    
    // Draw trail points
    final pointPaint = Paint()
      ..color = mappingActive ? Colors.green[300]! : Colors.blue[300]!
      ..style = PaintingStyle.fill;
    
    for (int i = 0; i < robotTrail.length; i += 5) { // Every 5th point
      final point = _worldToCanvas(robotTrail[i]);
      canvas.drawCircle(point, 1, pointPaint);
    }
  }

  void _drawRobotPosition(Canvas canvas, Size size) {
    final position = _worldToCanvas(Position(
      x: currentOdometry!.position.x,
      y: currentOdometry!.position.y,
      z: 0,
    ));
    
    final yaw = currentOdometry!.orientation.yaw;
    
    // Robot body (circle)
    final bodyPaint = Paint()
      ..color = mappingActive ? Colors.green : Colors.blue
      ..style = PaintingStyle.fill;
    
    final bodyRadius = mappingActive ? 12.0 * pulseAnimation.value : 12.0;
    canvas.drawCircle(position, bodyRadius, bodyPaint);
    
    // Robot direction indicator (arrow)
    final arrowPaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;
    
    final arrowLength = 15.0;
    final arrowEnd = Offset(
      position.dx + arrowLength * math.cos(yaw),
      position.dy + arrowLength * math.sin(yaw),
    );
    
    canvas.drawLine(position, arrowEnd, arrowPaint);
    
    // Robot outline
    final outlinePaint = Paint()
      ..color = mappingActive ? Colors.green[700]! : Colors.blue[700]!
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;
    
    canvas.drawCircle(position, bodyRadius, outlinePaint);
    
    // Mapping indicator
    if (mappingActive) {
      final indicatorPaint = Paint()
        ..color = Colors.red
        ..style = PaintingStyle.fill;
      
      canvas.drawCircle(
        Offset(position.dx + 8, position.dy - 8),
        3,
        indicatorPaint,
      );
    }
  }

  void _drawUIOverlay(Canvas canvas, Size size) {
    // Draw scale indicator
    _drawScaleIndicator(canvas, size);
    
    // Draw coordinate info
    if (currentOdometry != null) {
      _drawCoordinateInfo(canvas, size);
    }
    
    // Draw mapping status
    _drawMappingStatus(canvas, size);
  }

  void _drawScaleIndicator(Canvas canvas, Size size) {
    const scaleLength = 100.0; // pixels
    const realDistance = 2.0; // meters
    
    final paint = Paint()
      ..color = Colors.black
      ..strokeWidth = 2;
    
    final startX = size.width - 120;
    final startY = size.height - 30;
    
    // Scale line
    canvas.drawLine(
      Offset(startX, startY),
      Offset(startX + scaleLength, startY),
      paint,
    );
    
    // Scale end markers
    canvas.drawLine(
      Offset(startX, startY - 5),
      Offset(startX, startY + 5),
      paint,
    );
    canvas.drawLine(
      Offset(startX + scaleLength, startY - 5),
      Offset(startX + scaleLength, startY + 5),
      paint,
    );
    
    // Scale text
    _drawText(
      canvas,
      '${realDistance}m',
      Offset(startX + scaleLength / 2, startY - 15),
      Colors.black,
    );
  }

  void _drawCoordinateInfo(Canvas canvas, Size size) {
    final x = currentOdometry!.position.x;
    final y = currentOdometry!.position.y;
    final yaw = currentOdometry!.orientation.yawDegrees;
    
    final info = 'X: ${x.toStringAsFixed(2)}m  '
                 'Y: ${y.toStringAsFixed(2)}m  '
                 'θ: ${yaw.toStringAsFixed(1)}°';
    
    _drawText(canvas, info, Offset(10, 10), Colors.black);
  }

  void _drawMappingStatus(Canvas canvas, Size size) {
    final status = mappingActive ? 'MAPPING ACTIVE' : 'MAPPING INACTIVE';
    final color = mappingActive ? Colors.green : Colors.grey;
    
    _drawText(canvas, status, Offset(10, size.height - 20), color);
    
    // Trail info
    final trailInfo = 'Trail: ${robotTrail.length} points';
    _drawText(canvas, trailInfo, Offset(10, size.height - 40), Colors.black);
  }

  void _drawText(Canvas canvas, String text, Offset position, Color color) {
    final textPainter = TextPainter(
      text: TextSpan(
        text: text,
        style: TextStyle(
          color: color,
          fontSize: 12,
          fontWeight: FontWeight.bold,
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    
    textPainter.layout();
    textPainter.paint(canvas, position);
  }

  Offset _worldToCanvas(Position worldPos) {
    // Convert world coordinates (meters) to canvas coordinates (pixels)
    const scale = 50.0; // 1 meter = 50 pixels
    return Offset(worldPos.x * scale, -worldPos.y * scale); // Flip Y axis
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}