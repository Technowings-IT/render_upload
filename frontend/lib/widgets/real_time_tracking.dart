//realtime_tracking.dart
import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import '../services/web_socket_service.dart';
import '../models/map_data.dart';
import 'joystick.dart';
import '/models/odom.dart';

// Real-Time Map Widget with Live AGV Tracking
class LiveMapWidget extends StatefulWidget {
  final String? deviceId;
  final Function(MapData)? onMapChanged;

  const LiveMapWidget({
    Key? key,
    this.deviceId,
    this.onMapChanged,
  }) : super(key: key);

  @override
  _LiveMapWidgetState createState() => _LiveMapWidgetState();
}

class _LiveMapWidgetState extends State<LiveMapWidget>
    with TickerProviderStateMixin {
  final WebSocketService _webSocketService = WebSocketService();

  // Live data
  Position? _currentPosition;
  MapData? _currentMap;
  List<Position> _robotPath = [];
  List<ScanPoint> _currentScan = [];

  // Animation controllers
  late AnimationController _robotAnimationController;
  late AnimationController _scanAnimationController;

  // Subscriptions
  StreamSubscription? _odometrySubscription;
  StreamSubscription? _mapSubscription;
  StreamSubscription? _scanSubscription;

  // Map editing
  bool _isEditing = false;
  List<MapShape> _editableShapes = [];

  // Map transformation
  double _scale = 1.0;
  Offset _panOffset = Offset.zero;

  @override
  void initState() {
    super.initState();

    // Initialize animation controllers
    _robotAnimationController = AnimationController(
      duration: Duration(milliseconds: 500),
      vsync: this,
    );

    _scanAnimationController = AnimationController(
      duration: Duration(milliseconds: 100),
      vsync: this,
    );

    // Subscribe to live data
    _subscribeToLiveData();

    // Request live data for device
    if (widget.deviceId != null) {
      _webSocketService.sendMessage({
        'type': 'subscribe_live_data',
        'deviceId': widget.deviceId,
      });
    }
  }

  @override
  void dispose() {
    _robotAnimationController.dispose();
    _scanAnimationController.dispose();
    _odometrySubscription?.cancel();
    _mapSubscription?.cancel();
    _scanSubscription?.cancel();
    super.dispose();
  }

  void _subscribeToLiveData() {
    // Subscribe to live odometry updates
    _odometrySubscription = _webSocketService.connectionState.listen((data) {
      // This is a simplified subscription - in reality you'd parse WebSocket messages
      // based on message type and deviceId
    });

    // In practice, you'd handle WebSocket messages like this:
    _webSocketService.sendMessage({
      'type': 'subscribe',
      'topic': 'live_odometry',
    });
  }

  void _updateRobotPosition(Position newPosition) {
    setState(() {
      _currentPosition = newPosition;

      // Add to path (limit path length)
      _robotPath.add(newPosition);
      if (_robotPath.length > 100) {
        _robotPath.removeAt(0);
      }
    });

    // Animate robot movement
    _robotAnimationController.reset();
    _robotAnimationController.forward();
  }

  void _updateMapData(MapData newMapData) {
    setState(() {
      _currentMap = newMapData;
    });

    // Notify parent widget
    if (widget.onMapChanged != null) {
      widget.onMapChanged!(newMapData);
    }
  }

  void _updateScanData(List<ScanPoint> scanData) {
    setState(() {
      _currentScan = scanData;
    });

    // Animate scan update
    _scanAnimationController.reset();
    _scanAnimationController.forward();
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        _buildControlBar(),
        Expanded(
          child: Container(
            decoration: BoxDecoration(
              border: Border.all(color: Colors.grey),
              color: Colors.grey[100],
            ),
            child: GestureDetector(
              onScaleUpdate: _handleScaleUpdate,
              onTapDown: _handleTapDown,
              child: CustomPaint(
                size: Size.infinite,
                painter: LiveMapPainter(
                  mapData: _currentMap,
                  robotPosition: _currentPosition,
                  robotPath: _robotPath,
                  scanData: _currentScan,
                  editableShapes: _editableShapes,
                  scale: _scale,
                  panOffset: _panOffset,
                  robotAnimation: _robotAnimationController,
                  scanAnimation: _scanAnimationController,
                ),
              ),
            ),
          ),
        ),
        _buildStatusBar(),
      ],
    );
  }

  Widget _buildControlBar() {
    return Container(
      padding: EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.grey[200],
        border: Border(bottom: BorderSide(color: Colors.grey)),
      ),
      child: Row(
        children: [
          // Live status indicator
          Container(
            width: 12,
            height: 12,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: _currentPosition != null ? Colors.green : Colors.red,
            ),
          ),
          SizedBox(width: 8),
          Text(
            _currentPosition != null ? 'Live Tracking' : 'No Signal',
            style: TextStyle(fontWeight: FontWeight.bold),
          ),

          Spacer(),

          // Map editing toggle
          ElevatedButton.icon(
            onPressed: () {
              setState(() {
                _isEditing = !_isEditing;
              });
            },
            icon: Icon(_isEditing ? Icons.edit : Icons.edit_off),
            label: Text(_isEditing ? 'Stop Editing' : 'Edit Map'),
            style: ElevatedButton.styleFrom(
              backgroundColor: _isEditing ? Colors.orange : Colors.blue,
            ),
          ),

          SizedBox(width: 8),

          // Center on robot
          ElevatedButton.icon(
            onPressed: _centerOnRobot,
            icon: Icon(Icons.my_location),
            label: Text('Center'),
          ),
        ],
      ),
    );
  }

  Widget _buildStatusBar() {
    return Container(
      padding: EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.grey[100],
        border: Border(top: BorderSide(color: Colors.grey)),
      ),
      child: Row(
        children: [
          // Position info
          if (_currentPosition != null) ...[
            Text('X: ${_currentPosition!.x.toStringAsFixed(2)}m'),
            SizedBox(width: 16),
            Text('Y: ${_currentPosition!.y.toStringAsFixed(2)}m'),
            SizedBox(width: 16),
          ],

          // Map info
          if (_currentMap != null) ...[
            Text('Map: ${_currentMap!.info.width}×${_currentMap!.info.height}'),
            SizedBox(width: 16),
            Text(
                'Resolution: ${_currentMap!.info.resolution.toStringAsFixed(3)}m/px'),
          ],

          Spacer(),

          // Scale info
          Text('Scale: ${(_scale * 100).toInt()}%'),
        ],
      ),
    );
  }

  void _handleScaleUpdate(ScaleUpdateDetails details) {
    setState(() {
      _scale = (_scale * details.scale).clamp(0.1, 5.0);
      if (details.scale == 1.0) {
        // Pure pan
        _panOffset += details.focalPointDelta;
      }
    });
  }

  void _handleTapDown(TapDownDetails details) {
    if (!_isEditing) return;

    // Convert screen coordinates to map coordinates
    final mapPoint = _screenToMapCoordinates(details.localPosition);

    // Add a waypoint or edit existing shape
    _addWaypoint(mapPoint);
  }

  void _addWaypoint(Offset mapPoint) {
    if (_currentMap == null) return;

    final waypoint = MapShape(
      id: DateTime.now().millisecondsSinceEpoch.toString(),
      type: 'waypoint',
      name: 'Waypoint ${_editableShapes.length + 1}',
      points: [Position(x: mapPoint.dx, y: mapPoint.dy, z: 0)],
      sides: {'front': '', 'back': '', 'left': '', 'right': ''},
      color: Colors.purple.value.toRadixString(16),
      createdAt: DateTime.now(),
    );

    setState(() {
      _editableShapes.add(waypoint);
    });

    // Send real-time map edit
    _webSocketService.sendMessage({
      'type': 'map_edit',
      'deviceId': widget.deviceId,
      'editData': {
        'type': 'add_waypoint',
        'shape': waypoint.toJson(),
      },
    });
  }

  void _centerOnRobot() {
    if (_currentPosition == null) return;

    setState(() {
      _panOffset = Offset(
        -_currentPosition!.x * _scale + 200, // Center horizontally
        -_currentPosition!.y * _scale + 200, // Center vertically
      );
    });
  }

  Offset _screenToMapCoordinates(Offset screenPoint) {
    return (screenPoint - _panOffset) / _scale;
  }
}

// Custom painter for real-time map rendering
class LiveMapPainter extends CustomPainter {
  final MapData? mapData;
  final Position? robotPosition;
  final List<Position> robotPath;
  final List<ScanPoint> scanData;
  final List<MapShape> editableShapes;
  final double scale;
  final Offset panOffset;
  final AnimationController robotAnimation;
  final AnimationController scanAnimation;

  LiveMapPainter({
    this.mapData,
    this.robotPosition,
    required this.robotPath,
    required this.scanData,
    required this.editableShapes,
    required this.scale,
    required this.panOffset,
    required this.robotAnimation,
    required this.scanAnimation,
  }) : super(repaint: Listenable.merge([robotAnimation, scanAnimation]));

  @override
  void paint(Canvas canvas, Size size) {
    // Apply transformations
    canvas.save();
    canvas.translate(panOffset.dx, panOffset.dy);
    canvas.scale(scale);

    // Draw map if available
    if (mapData != null) {
      _drawOccupancyGrid(canvas);
    }

    // Draw robot path
    _drawRobotPath(canvas);

    // Draw LIDAR scan data
    _drawScanData(canvas);

    // Draw robot
    if (robotPosition != null) {
      _drawRobot(canvas, robotPosition!);
    }

    // Draw editable shapes
    _drawEditableShapes(canvas);

    canvas.restore();
  }

  void _drawOccupancyGrid(Canvas canvas) {
    if (mapData == null) return;

    final paint = Paint();
    final info = mapData!.info;

    for (int y = 0; y < info.height; y++) {
      for (int x = 0; x < info.width; x++) {
        final value = mapData!.getOccupancyAt(x, y);

        Color color;
        if (value == -1) {
          color = Colors.grey; // Unknown
        } else if (value == 0) {
          color = Colors.white; // Free
        } else {
          color = Colors.black; // Occupied
        }

        paint.color = color;

        final rect = Rect.fromLTWH(
          info.origin.x + (x * info.resolution),
          info.origin.y + (y * info.resolution),
          info.resolution,
          info.resolution,
        );

        canvas.drawRect(rect, paint);
      }
    }
  }

  void _drawRobotPath(Canvas canvas) {
    if (robotPath.length < 2) return;

    final paint = Paint()
      ..color = Colors.blue.withOpacity(0.6)
      ..strokeWidth = 2.0
      ..style = PaintingStyle.stroke;

    final path = Path();

    if (robotPath.isNotEmpty) {
      path.moveTo(robotPath.first.x, robotPath.first.y);

      for (int i = 1; i < robotPath.length; i++) {
        path.lineTo(robotPath[i].x, robotPath[i].y);
      }
    }

    canvas.drawPath(path, paint);
  }

  void _drawScanData(Canvas canvas) {
    if (scanData.isEmpty || robotPosition == null) return;

    final paint = Paint()
      ..color = Colors.red.withOpacity(0.3 + 0.4 * scanAnimation.value)
      ..strokeWidth = 1.0;

    final robotX = robotPosition!.x;
    final robotY = robotPosition!.y;

    for (final point in scanData) {
      final endX = robotX + point.range * math.cos(point.angle);
      final endY = robotY + point.range * math.sin(point.angle);

      canvas.drawLine(
        Offset(robotX, robotY),
        Offset(endX, endY),
        paint,
      );

      // Draw scan point
      canvas.drawCircle(
        Offset(endX, endY),
        1.0,
        paint..color = Colors.red,
      );
    }
  }

  void _drawRobot(Canvas canvas, Position position) {
    final paint = Paint()
      ..color = Colors.green
      ..style = PaintingStyle.fill;

    final outlinePaint = Paint()
      ..color = Color(0xFF006400)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;

    // Robot body (animated size based on animation)
    final animatedSize = 0.3 + 0.1 * robotAnimation.value;
    final robotRect = Rect.fromCenter(
      center: Offset(position.x, position.y),
      width: animatedSize,
      height: animatedSize,
    );

    canvas.drawOval(robotRect, paint);
    canvas.drawOval(robotRect, outlinePaint);

    // Robot direction indicator
    final directionPaint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.fill;

    final directionOffset = Offset(
      position.x + 0.1 * math.cos(0), // Assuming 0 orientation for now
      position.y + 0.1 * math.sin(0),
    );

    canvas.drawCircle(directionOffset, 0.05, directionPaint);

    // Robot ID text
    final textPainter = TextPainter(
      text: TextSpan(
        text: 'AGV',
        style: TextStyle(
          color: Colors.white,
          fontSize: 8,
          fontWeight: FontWeight.bold,
        ),
      ),
      textDirection: TextDirection.ltr,
    );

    textPainter.layout();
    textPainter.paint(
      canvas,
      Offset(
        position.x - textPainter.width / 2,
        position.y - textPainter.height / 2,
      ),
    );
  }

  void _drawEditableShapes(Canvas canvas) {
    for (final shape in editableShapes) {
      _drawShape(canvas, shape);
    }
  }

  void _drawShape(Canvas canvas, MapShape shape) {
    if (shape.points.isEmpty) return;

    final color = Color(int.parse('0xFF${shape.color}'));
    final paint = Paint()
      ..color = color.withOpacity(0.3)
      ..style = PaintingStyle.fill;

    final outlinePaint = Paint()
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;

    if (shape.points.length == 1) {
      // Single point (waypoint)
      canvas.drawCircle(
        Offset(shape.points.first.x, shape.points.first.y),
        0.2,
        paint,
      );
      canvas.drawCircle(
        Offset(shape.points.first.x, shape.points.first.y),
        0.2,
        outlinePaint,
      );
    } else {
      // Multi-point shape
      final path = Path();
      path.moveTo(shape.points.first.x, shape.points.first.y);

      for (int i = 1; i < shape.points.length; i++) {
        path.lineTo(shape.points[i].x, shape.points[i].y);
      }
      path.close();

      canvas.drawPath(path, paint);
      canvas.drawPath(path, outlinePaint);
    }

    // Draw shape name
    final textPainter = TextPainter(
      text: TextSpan(
        text: shape.name,
        style: TextStyle(
          color: Colors.black,
          fontSize: 6,
          fontWeight: FontWeight.bold,
        ),
      ),
      textDirection: TextDirection.ltr,
    );

    textPainter.layout();
    final center = shape.center;
    textPainter.paint(
      canvas,
      Offset(
        center.x - textPainter.width / 2,
        center.y - textPainter.height / 2,
      ),
    );
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true; // Always repaint for live updates
  }
}

// Data classes for scan data
class ScanPoint {
  final double angle;
  final double range;

  ScanPoint({required this.angle, required this.range});
}

// Real-time joystick widget for live control
class RealTimeJoystickWidget extends StatefulWidget {
  final String deviceId;
  final Function(double, double) onVelocityChanged;

  const RealTimeJoystickWidget({
    Key? key,
    required this.deviceId,
    required this.onVelocityChanged,
  }) : super(key: key);

  @override
  _RealTimeJoystickWidgetState createState() => _RealTimeJoystickWidgetState();
}

class _RealTimeJoystickWidgetState extends State<RealTimeJoystickWidget> {
  final WebSocketService _webSocketService = WebSocketService();
  Timer? _controlTimer;
  double _currentLinear = 0.0;
  double _currentAngular = 0.0;

  void _sendRealTimeControl(double linear, double angular) {
    setState(() {
      _currentLinear = linear;
      _currentAngular = angular;
    });

    // Send real-time control via WebSocket
    _webSocketService.sendMessage({
      'type': 'joystick_control',
      'deviceId': widget.deviceId,
      'linear': linear,
      'angular': angular,
    });

    widget.onVelocityChanged(linear, angular);
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: EdgeInsets.all(16),
      child: Column(
        children: [
          // Real-time feedback
          Container(
            padding: EdgeInsets.all(8),
            decoration: BoxDecoration(
              color: Colors.blue.withOpacity(0.1),
              borderRadius: BorderRadius.circular(8),
            ),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                Text('Linear: ${_currentLinear.toStringAsFixed(2)} m/s'),
                Text('Angular: ${_currentAngular.toStringAsFixed(2)} rad/s'),
              ],
            ),
          ),

          SizedBox(height: 16),

          // Joystick control
          JoystickWidget(
            size: 200,
            maxLinearSpeed: 1.0,
            maxAngularSpeed: 2.0,
            enabled: true,
            onChanged: (double linear, double angular, bool isPressed) {
              _sendRealTimeControl(linear, angular);
            },
          ),
        ],
      ),
    );
  }
}
