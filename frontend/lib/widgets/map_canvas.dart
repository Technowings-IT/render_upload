//widgets/map_canvas.dart - Enhanced Map Canvas with Real-time Updates
import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../models/map_data.dart';
import '../models/odom.dart';
import '../services/web_socket_service.dart';
import '../services/api_service.dart';

enum MapEditTool {
  none,
  pencil,
  eraser,
  rectangle,
  circle,
  line,
  polygon,
}

enum LocationType {
  pickup,
  drop,
  charging,
  waypoint,
  obstacle,
}

class MapCanvasWidget extends StatefulWidget {
  final MapData? mapData;
  final Function(MapData) onMapChanged;
  final double width;
  final double height;
  final String? deviceId;
  final bool enableRealTimeUpdates;

  const MapCanvasWidget({
    Key? key,
    this.mapData,
    required this.onMapChanged,
    this.width = 800,
    this.height = 600,
    this.deviceId,
    this.enableRealTimeUpdates = true,
  }) : super(key: key);

  @override
  _MapCanvasWidgetState createState() => _MapCanvasWidgetState();
}

class _MapCanvasWidgetState extends State<MapCanvasWidget> {
  final WebSocketService _webSocketService = WebSocketService();
  final ApiService _apiService = ApiService();
  
  MapEditTool _currentTool = MapEditTool.none;
  LocationType _currentLocationType = LocationType.waypoint;
  List<Offset> _currentPath = [];
  List<Offset> _tempShapePoints = [];
  MapShape? _selectedShape;
  bool _isDrawing = false;
  double _brushSize = 5.0;
  double _scale = 1.0;
  Offset _panOffset = Offset.zero;
  Offset _lastPanPoint = Offset.zero;
  
  // Real-time data
  OdometryData? _currentOdometry;
  List<Offset> _robotTrail = [];
  List<Offset> _obstacles = [];
  MapData? _liveMapData;
  bool _showRealTimeData = true;
  bool _showRobotTrail = true;
  bool _showObstacles = true;

  static const Map<LocationType, Color> _locationColors = {
    LocationType.pickup: Colors.green,
    LocationType.drop: Colors.blue,
    LocationType.charging: Colors.orange,
    LocationType.waypoint: Colors.purple,
    LocationType.obstacle: Colors.red,
  };

  static const Map<LocationType, String> _locationNames = {
    LocationType.pickup: 'PICKUP_LOCATION',
    LocationType.drop: 'DROP_LOCATION', 
    LocationType.charging: 'CHARGING_LOCATION',
    LocationType.waypoint: 'WAYPOINT',
    LocationType.obstacle: 'NO_GO_ZONE',
  };

  @override
  void initState() {
    super.initState();
    if (widget.enableRealTimeUpdates && widget.deviceId != null) {
      _subscribeToRealTimeUpdates();
    }
  }

  @override
  void dispose() {
    super.dispose();
  }

  void _subscribeToRealTimeUpdates() {
    // Subscribe to real-time data for this device
    _webSocketService.realTimeData.listen((data) {
      if (data['deviceId'] == widget.deviceId) {
        _handleRealTimeData(data);
      }
    });

    // Subscribe to map events
    _webSocketService.mapEvents.listen((data) {
      if (data['deviceId'] == widget.deviceId) {
        _handleMapEvent(data);
      }
    });
  }

  void _handleRealTimeData(Map<String, dynamic> data) {
    setState(() {
      switch (data['type']) {
        case 'odometry_update':
          _currentOdometry = OdometryData.fromJson(data['data']);
          
          // Update robot trail
          if (_currentOdometry != null) {
            final robotPos = Offset(_currentOdometry!.position.x, _currentOdometry!.position.y);
            _robotTrail.add(robotPos);
            if (_robotTrail.length > 200) {
              _robotTrail.removeAt(0);
            }
          }
          break;
          
        case 'map_update':
          try {
            _liveMapData = MapData.fromJson(data['data']);
          } catch (e) {
            print('❌ Error parsing live map data: $e');
          }
          break;
          
        case 'laser_scan':
          final obstacles = data['data']['obstacles'] as List?;
          if (obstacles != null) {
            try {
              _obstacles = obstacles.map((obs) => Offset(
                (obs['x'] as num).toDouble(), 
                (obs['y'] as num).toDouble()
              )).toList();
            } catch (e) {
              print('❌ Error parsing obstacle data: $e');
              _obstacles = [];
            }
          }
          break;
      }
    });
  }

  void _handleMapEvent(Map<String, dynamic> data) {
    switch (data['type']) {
      case 'map_edited':
        // Handle real-time map edits from other clients
        print('🗺️ Map edited by another client: ${data['editType']}');
        _showInfoSnackBar('Map edited by another user');
        break;
        
      case 'shape_added':
      case 'shape_updated':
      case 'shape_removed':
        // Refresh map data
        _refreshMapData();
        break;
    }
  }

  void _refreshMapData() async {
    if (widget.deviceId == null) return;
    
    try {
      final response = await _apiService.getMapData(widget.deviceId!);
      if (response['success'] == true) {
        final updatedMap = MapData.fromJson(response['mapData']);
        widget.onMapChanged(updatedMap);
      }
    } catch (e) {
      print('❌ Error refreshing map data: $e');
      if (e is ApiException) {
        _showErrorSnackBar('Failed to refresh map: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to refresh map: $e');
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        _buildToolbar(),
        Expanded(
          child: Container(
            decoration: BoxDecoration(
              border: Border.all(color: Colors.grey),
              color: Colors.grey[100],
            ),
            child: ClipRect(
              child: GestureDetector(
                onScaleStart: _onScaleStart,
                onScaleUpdate: _onScaleUpdate,
                onScaleEnd: _onScaleEnd,
                onTapDown: _currentTool == MapEditTool.none ? _onTapDown : null,
                onPanStart: _currentTool != MapEditTool.none ? _onPanStart : null,
                onPanUpdate: _currentTool != MapEditTool.none ? _onPanUpdate : null,
                onPanEnd: _currentTool != MapEditTool.none ? _onPanEnd : null,
                child: CustomPaint(
                  size: Size(widget.width, widget.height),
                  painter: MapCanvasPainter(
                    mapData: widget.mapData,
                    liveMapData: _liveMapData,
                    currentPath: _currentPath,
                    tempShapePoints: _tempShapePoints,
                    selectedShape: _selectedShape,
                    scale: _scale,
                    panOffset: _panOffset,
                    brushSize: _brushSize,
                    currentTool: _currentTool,
                    currentColor: _locationColors[_currentLocationType]!,
                    // Real-time data
                    currentOdometry: _currentOdometry,
                    robotTrail: _showRobotTrail ? _robotTrail : [],
                    obstacles: _showObstacles ? _obstacles : [],
                    showRealTimeData: _showRealTimeData,
                  ),
                ),
              ),
            ),
          ),
        ),
        _buildRealTimeControls(),
        _buildShapeProperties(),
      ],
    );
  }

  Widget _buildToolbar() {
    return Container(
      padding: EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.grey[200],
        border: Border(bottom: BorderSide(color: Colors.grey)),
      ),
      child: Column(
        children: [
          // Drawing tools
          Row(
            children: [
              Text('Tools: '),
              SizedBox(width: 8),
              ...MapEditTool.values.map((tool) => _buildToolButton(tool)),
              Spacer(),
              Text('Brush Size: '),
              SizedBox(
                width: 100,
                child: Slider(
                  value: _brushSize,
                  min: 1.0,
                  max: 20.0,
                  onChanged: (value) {
                    setState(() {
                      _brushSize = value;
                    });
                  },
                ),
              ),
            ],
          ),
          SizedBox(height: 8),
          // Location types
          Row(
            children: [
              Text('Location Type: '),
              SizedBox(width: 8),
              ...LocationType.values.map((type) => _buildLocationTypeButton(type)),
              Spacer(),
              ElevatedButton.icon(
                onPressed: _clearMap,
                icon: Icon(Icons.clear_all),
                label: Text('Clear'),
              ),
              SizedBox(width: 8),
              ElevatedButton.icon(
                onPressed: _saveMap,
                icon: Icon(Icons.save),
                label: Text('Save'),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildRealTimeControls() {
    if (!widget.enableRealTimeUpdates) return SizedBox.shrink();
    
    return Container(
      padding: EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.blue[50],
        border: Border(top: BorderSide(color: Colors.blue[200]!)),
      ),
      child: Row(
        children: [
          Icon(Icons.online_prediction, color: Colors.blue),
          SizedBox(width: 8),
          Text('Real-time Data:', style: TextStyle(fontWeight: FontWeight.bold)),
          SizedBox(width: 16),
          _buildToggleSwitch('Show Live Data', _showRealTimeData, (value) {
            setState(() {
              _showRealTimeData = value;
            });
          }),
          SizedBox(width: 16),
          _buildToggleSwitch('Robot Trail', _showRobotTrail, (value) {
            setState(() {
              _showRobotTrail = value;
            });
          }),
          SizedBox(width: 16),
          _buildToggleSwitch('Obstacles', _showObstacles, (value) {
            setState(() {
              _showObstacles = value;
            });
          }),
          Spacer(),
          if (_currentOdometry != null) ...[
            Text('Robot: '),
            Text('(${_currentOdometry!.position.x.toStringAsFixed(2)}, ${_currentOdometry!.position.y.toStringAsFixed(2)})',
                 style: TextStyle(fontWeight: FontWeight.bold)),
            SizedBox(width: 8),
            Text('${_currentOdometry!.orientation.yawDegrees.toStringAsFixed(1)}°'),
          ],
        ],
      ),
    );
  }

  Widget _buildToggleSwitch(String label, bool value, Function(bool) onChanged) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Text(label, style: TextStyle(fontSize: 12)),
        SizedBox(width: 4),
        Switch(
          value: value,
          onChanged: onChanged,
          materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
        ),
      ],
    );
  }

  Widget _buildToolButton(MapEditTool tool) {
    final isSelected = _currentTool == tool;
    IconData icon;
    String tooltip;

    switch (tool) {
      case MapEditTool.none:
        icon = Icons.pan_tool;
        tooltip = 'Select/Pan';
        break;
      case MapEditTool.pencil:
        icon = Icons.brush;
        tooltip = 'Draw';
        break;
      case MapEditTool.eraser:
        icon = Icons.auto_fix_off;
        tooltip = 'Erase';
        break;
      case MapEditTool.rectangle:
        icon = Icons.crop_square;
        tooltip = 'Rectangle';
        break;
      case MapEditTool.circle:
        icon = Icons.circle_outlined;
        tooltip = 'Circle';
        break;
      case MapEditTool.line:
        icon = Icons.timeline;
        tooltip = 'Line';
        break;
      case MapEditTool.polygon:
        icon = Icons.polyline;
        tooltip = 'Polygon';
        break;
    }

    return Tooltip(
      message: tooltip,
      child: Padding(
        padding: EdgeInsets.only(right: 4),
        child: ElevatedButton(
          onPressed: () {
            setState(() {
              _currentTool = tool;
              _tempShapePoints.clear();
              _selectedShape = null;
            });
          },
          style: ElevatedButton.styleFrom(
            backgroundColor: isSelected ? Colors.blue : Colors.grey[300],
            foregroundColor: isSelected ? Colors.white : Colors.black,
            minimumSize: Size(40, 40),
            padding: EdgeInsets.all(8),
          ),
          child: Icon(icon, size: 20),
        ),
      ),
    );
  }

  Widget _buildLocationTypeButton(LocationType type) {
    final isSelected = _currentLocationType == type;
    final color = _locationColors[type]!;
    final name = type.toString().split('.').last;

    return Padding(
      padding: EdgeInsets.only(right: 4),
      child: ElevatedButton(
        onPressed: () {
          setState(() {
            _currentLocationType = type;
          });
        },
        style: ElevatedButton.styleFrom(
          backgroundColor: isSelected ? color : Colors.grey[300],
          foregroundColor: isSelected ? Colors.white : Colors.black,
          padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        ),
        child: Text(name.toUpperCase()),
      ),
    );
  }

  Widget _buildShapeProperties() {
    if (_selectedShape == null) return SizedBox.shrink();

    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.grey[100],
        border: Border(top: BorderSide(color: Colors.grey)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Shape Properties',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          ),
          SizedBox(height: 12),
          Row(
            children: [
              Expanded(
                child: TextFormField(
                  initialValue: _selectedShape!.name,
                  decoration: InputDecoration(
                    labelText: 'Name',
                    border: OutlineInputBorder(),
                  ),
                  onChanged: (value) {
                    _updateSelectedShapeName(value);
                  },
                ),
              ),
              SizedBox(width: 16),
              Expanded(
                child: DropdownButtonFormField<LocationType>(
                  value: LocationType.values.firstWhere(
                    (t) => t.toString().split('.').last == _selectedShape!.type,
                    orElse: () => LocationType.waypoint,
                  ),
                  decoration: InputDecoration(
                    labelText: 'Type',
                    border: OutlineInputBorder(),
                  ),
                  items: LocationType.values.map((type) {
                    final name = type.toString().split('.').last;
                    return DropdownMenuItem(
                      value: type,
                      child: Text(name.toUpperCase()),
                    );
                  }).toList(),
                  onChanged: (value) {
                    if (value != null) {
                      _updateSelectedShapeType(value.toString().split('.').last);
                    }
                  },
                ),
              ),
            ],
          ),
          SizedBox(height: 12),
          Text('Side Definitions:', style: TextStyle(fontWeight: FontWeight.bold)),
          SizedBox(height: 8),
          Row(
            children: [
              Expanded(child: _buildSideInput('Front', 'front')),
              SizedBox(width: 8),
              Expanded(child: _buildSideInput('Back', 'back')),
              SizedBox(width: 8),
              Expanded(child: _buildSideInput('Left', 'left')),
              SizedBox(width: 8),
              Expanded(child: _buildSideInput('Right', 'right')),
            ],
          ),
          SizedBox(height: 12),
          Row(
            children: [
              ElevatedButton.icon(
                onPressed: _deleteSelectedShape,
                icon: Icon(Icons.delete),
                label: Text('Delete'),
                style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
              ),
              SizedBox(width: 8),
              ElevatedButton.icon(
                onPressed: _duplicateSelectedShape,
                icon: Icon(Icons.copy),
                label: Text('Duplicate'),
              ),
              SizedBox(width: 8),
              ElevatedButton.icon(
                onPressed: _navigateToShape,
                icon: Icon(Icons.navigation),
                label: Text('Navigate Here'),
                style: ElevatedButton.styleFrom(backgroundColor: Colors.blue),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildSideInput(String label, String side) {
    return TextFormField(
      initialValue: _selectedShape!.sides[side] ?? '',
      decoration: InputDecoration(
        labelText: label,
        border: OutlineInputBorder(),
        isDense: true,
      ),
      onChanged: (value) {
        _updateSelectedShapeSide(side, value);
      },
    );
  }

  // Event handlers (keeping existing ones and adding new ones)
  void _onScaleStart(ScaleStartDetails details) {
    _lastPanPoint = details.localFocalPoint;
  }

  void _onScaleUpdate(ScaleUpdateDetails details) {
    if (_currentTool == MapEditTool.none) {
      setState(() {
        _scale = (_scale * details.scale).clamp(0.1, 5.0);
        if (details.scale == 1.0) {
          final delta = details.localFocalPoint - _lastPanPoint;
          _panOffset += delta;
          _lastPanPoint = details.localFocalPoint;
        }
      });
    }
  }

  void _onScaleEnd(ScaleEndDetails details) {
    // Handle scale end if needed
  }

  void _onTapDown(TapDownDetails details) {
    if (_currentTool == MapEditTool.none) {
      _selectShapeAtPoint(details.localPosition);
    }
  }

  void _onPanStart(DragStartDetails details) {
    if (_currentTool == MapEditTool.none) return;

    setState(() {
      _isDrawing = true;
      _currentPath.clear();
      _tempShapePoints.clear();
    });

    final point = _screenToMapCoordinates(details.localPosition);
    _tempShapePoints.add(point);
  }

  void _onPanUpdate(DragUpdateDetails details) {
    if (!_isDrawing || _currentTool == MapEditTool.none) return;

    final point = _screenToMapCoordinates(details.localPosition);

    setState(() {
      switch (_currentTool) {
        case MapEditTool.pencil:
        case MapEditTool.eraser:
          _currentPath.add(point);
          break;
        case MapEditTool.rectangle:
        case MapEditTool.circle:
        case MapEditTool.line:
          if (_tempShapePoints.isNotEmpty) {
            _tempShapePoints[_tempShapePoints.length - 1] = point;
          }
          break;
        case MapEditTool.polygon:
          if (_tempShapePoints.length > 1) {
            _tempShapePoints[_tempShapePoints.length - 1] = point;
          }
          break;
        default:
          break;
      }
    });
  }

  void _onPanEnd(DragEndDetails details) {
    if (!_isDrawing || _currentTool == MapEditTool.none) return;

    setState(() {
      _isDrawing = false;
    });

    _createShapeFromTempPoints();
  }

  void _createShapeFromTempPoints() async {
    if (_tempShapePoints.isEmpty) return;

    List<Position> points = _tempShapePoints
        .map((offset) => Position(x: offset.dx, y: offset.dy, z: 0))
        .toList();

    if (_currentTool == MapEditTool.rectangle && points.length >= 2) {
      final p1 = points[0];
      final p2 = points[1];
      points = [
        Position(x: p1.x, y: p1.y, z: 0),
        Position(x: p2.x, y: p1.y, z: 0),
        Position(x: p2.x, y: p2.y, z: 0),
        Position(x: p1.x, y: p2.y, z: 0),
      ];
    } else if (_currentTool == MapEditTool.circle && points.length >= 2) {
      final center = points[0];
      final radiusPoint = points[1];
      final radius = math.sqrt(
        math.pow(radiusPoint.x - center.x, 2) + 
        math.pow(radiusPoint.y - center.y, 2)
      );
      
      points = [];
      for (int i = 0; i < 32; i++) {
        final angle = (i / 32) * 2 * math.pi;
        points.add(Position(
          x: center.x + radius * math.cos(angle),
          y: center.y + radius * math.sin(angle),
          z: 0,
        ));
      }
    }

    final shape = MapShape(
      id: DateTime.now().millisecondsSinceEpoch.toString(),
      type: _currentLocationType.toString().split('.').last,
      name: '${_currentLocationType.toString().split('.').last}_${DateTime.now().millisecondsSinceEpoch}',
      points: points,
      sides: {'front': '', 'back': '', 'left': '', 'right': ''},
      color: _locationColors[_currentLocationType]!.value.toRadixString(16),
      createdAt: DateTime.now(),
    );

    // Send to backend via WebSocket for real-time updates
    if (widget.deviceId != null && widget.enableRealTimeUpdates) {
      try {
        _webSocketService.addMapShape(widget.deviceId!, {
          'type': shape.type,
          'name': shape.name,
          'coordinates': shape.points.map((p) => {'x': p.x, 'y': p.y, 'z': p.z}).toList(),
          'properties': {
            'color': shape.color,
            'sides': shape.sides,
          },
        });
      } catch (e) {
        print('❌ Error sending shape via WebSocket: $e');
        _showWarningSnackBar('Real-time sync failed. Shape saved locally only.');
      }
    }

    // Also update local state
    if (widget.mapData != null) {
      final updatedMap = widget.mapData!.addShape(shape);
      widget.onMapChanged(updatedMap);
    }

    setState(() {
      _tempShapePoints.clear();
      _currentPath.clear();
    });
  }

  void _selectShapeAtPoint(Offset screenPoint) {
    if (widget.mapData == null) return;

    final mapPoint = _screenToMapCoordinates(screenPoint);
    final position = Position(x: mapPoint.dx, y: mapPoint.dy, z: 0);

    for (final shape in widget.mapData!.shapes.reversed) {
      if (shape.containsPoint(position)) {
        setState(() {
          _selectedShape = shape;
        });
        return;
      }
    }

    setState(() {
      _selectedShape = null;
    });
  }

  Offset _screenToMapCoordinates(Offset screenPoint) {
    return (screenPoint - _panOffset) / _scale;
  }

  Offset _mapToScreenCoordinates(Offset mapPoint) {
    return mapPoint * _scale + _panOffset;
  }

  void _updateSelectedShapeName(String name) async {
    if (_selectedShape == null || widget.mapData == null) return;

    // Send update via WebSocket if enabled
    if (widget.deviceId != null && widget.enableRealTimeUpdates) {
      try {
        _webSocketService.updateMapShape(widget.deviceId!, _selectedShape!.id, {
          'name': name,
        });
      } catch (e) {
        print('❌ Error updating shape name via WebSocket: $e');
        _showWarningSnackBar('Real-time sync failed. Changes saved locally only.');
      }
    }

    // Update local state
    final updatedShape = MapShape(
      id: _selectedShape!.id,
      type: _selectedShape!.type,
      name: name,
      points: _selectedShape!.points,
      sides: _selectedShape!.sides,
      color: _selectedShape!.color,
      createdAt: _selectedShape!.createdAt,
    );

    _updateShapeInMap(updatedShape);
  }

  void _updateSelectedShapeType(String type) async {
    if (_selectedShape == null || widget.mapData == null) return;

    final locationType = LocationType.values.firstWhere(
      (t) => t.toString().split('.').last == type,
      orElse: () => LocationType.waypoint,
    );

    // Send update via WebSocket if enabled
    if (widget.deviceId != null && widget.enableRealTimeUpdates) {
      try {
        _webSocketService.updateMapShape(widget.deviceId!, _selectedShape!.id, {
          'type': type,
          'properties': {
            'color': _locationColors[locationType]!.value.toRadixString(16),
          },
        });
      } catch (e) {
        print('❌ Error updating shape type via WebSocket: $e');
        _showWarningSnackBar('Real-time sync failed. Changes saved locally only.');
      }
    }

    final updatedShape = MapShape(
      id: _selectedShape!.id,
      type: type,
      name: _selectedShape!.name,
      points: _selectedShape!.points,
      sides: _selectedShape!.sides,
      color: _locationColors[locationType]!.value.toRadixString(16),
      createdAt: _selectedShape!.createdAt,
    );

    _updateShapeInMap(updatedShape);
  }

  void _updateSelectedShapeSide(String side, String value) async {
    if (_selectedShape == null || widget.mapData == null) return;

    final newSides = Map<String, String>.from(_selectedShape!.sides);
    newSides[side] = value;

    // Send update via WebSocket if enabled
    if (widget.deviceId != null && widget.enableRealTimeUpdates) {
      try {
        _webSocketService.updateMapShape(widget.deviceId!, _selectedShape!.id, {
          'properties': {
            'sides': newSides,
          },
        });
      } catch (e) {
        print('❌ Error updating shape sides via WebSocket: $e');
        _showWarningSnackBar('Real-time sync failed. Changes saved locally only.');
      }
    }

    final updatedShape = MapShape(
      id: _selectedShape!.id,
      type: _selectedShape!.type,
      name: _selectedShape!.name,
      points: _selectedShape!.points,
      sides: newSides,
      color: _selectedShape!.color,
      createdAt: _selectedShape!.createdAt,
    );

    _updateShapeInMap(updatedShape);
  }

  void _updateShapeInMap(MapShape updatedShape) {
    final updatedShapes = widget.mapData!.shapes.map((shape) {
      return shape.id == updatedShape.id ? updatedShape : shape;
    }).toList();

    final updatedMap = MapData(
      deviceId: widget.mapData!.deviceId,
      timestamp: DateTime.now(),
      info: widget.mapData!.info,
      occupancyData: widget.mapData!.occupancyData,
      shapes: updatedShapes,
      version: widget.mapData!.version + 1,
    );

    setState(() {
      _selectedShape = updatedShape;
    });

    widget.onMapChanged(updatedMap);
  }

  void _deleteSelectedShape() async {
    if (_selectedShape == null || widget.mapData == null) return;

    // Send delete via WebSocket if enabled
    if (widget.deviceId != null && widget.enableRealTimeUpdates) {
      try {
        _webSocketService.deleteMapShape(widget.deviceId!, _selectedShape!.id);
      } catch (e) {
        print('❌ Error deleting shape via WebSocket: $e');
        _showWarningSnackBar('Real-time sync failed. Shape deleted locally only.');
      }
    }

    // Update local state
    final updatedMap = widget.mapData!.removeShape(_selectedShape!.id);
    widget.onMapChanged(updatedMap);

    setState(() {
      _selectedShape = null;
    });
  }

  void _duplicateSelectedShape() async {
    if (_selectedShape == null || widget.mapData == null) return;

    final duplicatedShape = MapShape(
      id: DateTime.now().millisecondsSinceEpoch.toString(),
      type: _selectedShape!.type,
      name: '${_selectedShape!.name}_copy',
      points: _selectedShape!.points.map((p) => Position(
        x: p.x + 0.5, // Offset slightly
        y: p.y + 0.5,
        z: p.z,
      )).toList(),
      sides: Map<String, String>.from(_selectedShape!.sides),
      color: _selectedShape!.color,
      createdAt: DateTime.now(),
    );

    // Send via WebSocket if enabled
    if (widget.deviceId != null && widget.enableRealTimeUpdates) {
      try {
        _webSocketService.addMapShape(widget.deviceId!, {
          'type': duplicatedShape.type,
          'name': duplicatedShape.name,
          'coordinates': duplicatedShape.points.map((p) => {'x': p.x, 'y': p.y, 'z': p.z}).toList(),
          'properties': {
            'color': duplicatedShape.color,
            'sides': duplicatedShape.sides,
          },
        });
      } catch (e) {
        print('❌ Error duplicating shape via WebSocket: $e');
        _showWarningSnackBar('Real-time sync failed. Shape duplicated locally only.');
      }
    }

    final updatedMap = widget.mapData!.addShape(duplicatedShape);
    widget.onMapChanged(updatedMap);
  }

  void _navigateToShape() async {
    if (_selectedShape == null || widget.deviceId == null) return;

    final center = _selectedShape!.center;
    
    try {
      // Use the corrected movement API for navigation placeholder
      await _apiService.moveDevice(
        deviceId: widget.deviceId!,
        linear: 0.0, // Stop movement, just for endpoint test
        angular: 0.0,
      );
      
      _showInfoSnackBar('Navigation goal set to ${_selectedShape!.name}');
      
      // Note: In a real implementation, you would need a proper navigation endpoint
      // This is just using the movement API as a placeholder
      print('🧭 Navigation requested to: ${center.x}, ${center.y}');
      
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to set navigation goal: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to set navigation goal: $e');
      }
    }
  }

  void _clearMap() {
    if (widget.mapData == null) return;

    final clearedMap = MapData(
      deviceId: widget.mapData!.deviceId,
      timestamp: DateTime.now(),
      info: widget.mapData!.info,
      occupancyData: widget.mapData!.occupancyData,
      shapes: [],
      version: widget.mapData!.version + 1,
    );

    widget.onMapChanged(clearedMap);
    setState(() {
      _selectedShape = null;
    });
  }

  void _saveMap() async {
    if (widget.mapData == null || widget.deviceId == null) return;

    try {
      await _apiService.saveMapData(
        deviceId: widget.deviceId!,
        mapData: widget.mapData!.toJson(),
      );
      
      _showInfoSnackBar('Map saved successfully');
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to save map: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to save map: $e');
      }
    }
  }

  // Helper methods for showing messages
  void _showInfoSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.blue,
        duration: Duration(seconds: 2),
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

  void _showErrorSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.red,
        duration: Duration(seconds: 4),
      ),
    );
  }
}

class MapCanvasPainter extends CustomPainter {
  final MapData? mapData;
  final MapData? liveMapData;
  final List<Offset> currentPath;
  final List<Offset> tempShapePoints;
  final MapShape? selectedShape;
  final double scale;
  final Offset panOffset;
  final double brushSize;
  final MapEditTool currentTool;
  final Color currentColor;
  
  // Real-time data
  final OdometryData? currentOdometry;
  final List<Offset> robotTrail;
  final List<Offset> obstacles;
  final bool showRealTimeData;

  MapCanvasPainter({
    this.mapData,
    this.liveMapData,
    required this.currentPath,
    required this.tempShapePoints,
    this.selectedShape,
    required this.scale,
    required this.panOffset,
    required this.brushSize,
    required this.currentTool,
    required this.currentColor,
    this.currentOdometry,
    required this.robotTrail,
    required this.obstacles,
    required this.showRealTimeData,
  });

  @override
  void paint(Canvas canvas, Size size) {
    // Draw background
    canvas.drawRect(
      Rect.fromLTWH(0, 0, size.width, size.height),
      Paint()..color = Colors.white,
    );

    // Apply transformations
    canvas.save();
    canvas.translate(panOffset.dx, panOffset.dy);
    canvas.scale(scale);

    // Draw occupancy grid (prioritize live data)
    final currentMapData = liveMapData ?? mapData;
    if (currentMapData != null) {
      _drawOccupancyGrid(canvas, currentMapData);
      _drawShapes(canvas, currentMapData);
    }

    // Draw real-time data
    if (showRealTimeData) {
      _drawObstacles(canvas);
      _drawRobotTrail(canvas);
      _drawRobot(canvas);
    }

    // Draw current drawing
    _drawCurrentPath(canvas);
    _drawTempShape(canvas);

    canvas.restore();

    // Draw UI elements that shouldn't be scaled
    _drawOverlay(canvas, size);
  }

  void _drawOccupancyGrid(Canvas canvas, MapData mapData) {
    final paint = Paint();
    final info = mapData.info;
    
    // Sample the grid for performance (draw every 4th pixel)
    for (int y = 0; y < info.height; y += 4) {
      for (int x = 0; x < info.width; x += 4) {
        final value = mapData.getOccupancyAt(x, y);
        
        Color color;
        if (value == -1) {
          color = Colors.grey[300]!; // Unknown
        } else if (value == 0) {
          color = Colors.white; // Free
        } else {
          color = Colors.black; // Occupied
        }
        
        paint.color = color;
        canvas.drawRect(
          Rect.fromLTWH(
            x * info.resolution,
            y * info.resolution,
            info.resolution * 4,
            info.resolution * 4,
          ),
          paint,
        );
      }
    }
  }

  void _drawShapes(Canvas canvas, MapData mapData) {
    for (final shape in mapData.shapes) {
      _drawShape(canvas, shape, shape == selectedShape);
    }
  }

  void _drawShape(Canvas canvas, MapShape shape, bool isSelected) {
    if (shape.points.isEmpty) return;

    Color shapeColor;
    try {
      // Safe color parsing with better error handling
      final colorString = shape.color.replaceAll('#', '');
      final colorValue = int.tryParse('0xFF$colorString');
      if (colorValue != null) {
        shapeColor = Color(colorValue);
      } else {
        // Fallback color based on type
        shapeColor = _getDefaultColorForType(shape.type);
      }
    } catch (e) {
      shapeColor = _getDefaultColorForType(shape.type);
    }

    final paint = Paint()
      ..color = shapeColor
      ..style = PaintingStyle.fill
      ..strokeWidth = 2;

    final outlinePaint = Paint()
      ..color = isSelected ? Colors.red : Colors.black
      ..style = PaintingStyle.stroke
      ..strokeWidth = isSelected ? 3 : 1;

    final path = Path();
    final firstPoint = shape.points.first;
    path.moveTo(firstPoint.x, firstPoint.y);

    for (int i = 1; i < shape.points.length; i++) {
      final point = shape.points[i];
      path.lineTo(point.x, point.y);
    }
    path.close();

    // Draw filled shape
    canvas.drawPath(path, paint..color = paint.color.withOpacity(0.3));
    
    // Draw outline
    canvas.drawPath(path, outlinePaint);

    // Draw shape name
    final textPainter = TextPainter(
      text: TextSpan(
        text: shape.name,
        style: TextStyle(
          color: Colors.black,
          fontSize: 10 / scale, // Scale text appropriately
          fontWeight: FontWeight.bold,
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    textPainter.layout();

    final center = shape.center;
    textPainter.paint(
      canvas,
      Offset(center.x - textPainter.width / 2, center.y - textPainter.height / 2),
    );
  }

  Color _getDefaultColorForType(String type) {
    switch (type) {
      case 'pickup':
        return Colors.green;
      case 'drop':
        return Colors.blue;
      case 'charging':
        return Colors.orange;
      case 'waypoint':
        return Colors.purple;
      case 'obstacle':
        return Colors.red;
      default:
        return Colors.grey;
    }
  }

  void _drawRobot(Canvas canvas) {
    if (currentOdometry == null) return;

    final robotPos = Offset(currentOdometry!.position.x, currentOdometry!.position.y);
    final robotYaw = currentOdometry!.orientation.yaw;

    // Draw robot body
    final robotPaint = Paint()
      ..color = Colors.blue
      ..style = PaintingStyle.fill;

    canvas.drawCircle(robotPos, 0.3, robotPaint);

    // Draw robot orientation arrow
    final arrowPaint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.stroke
      ..strokeWidth = 0.05;

    final arrowEnd = Offset(
      robotPos.dx + 0.2 * math.cos(robotYaw),
      robotPos.dy + 0.2 * math.sin(robotYaw),
    );

    canvas.drawLine(robotPos, arrowEnd, arrowPaint);
  }

  void _drawRobotTrail(Canvas canvas) {
    if (robotTrail.length < 2) return;

    final trailPaint = Paint()
      ..color = Colors.blue.withOpacity(0.5)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 0.02;

    final path = Path();
    path.moveTo(robotTrail.first.dx, robotTrail.first.dy);

    for (int i = 1; i < robotTrail.length; i++) {
      path.lineTo(robotTrail[i].dx, robotTrail[i].dy);
    }

    canvas.drawPath(path, trailPaint);
  }

  void _drawObstacles(Canvas canvas) {
    if (obstacles.isEmpty) return;

    final obstaclePaint = Paint()
      ..color = Colors.red.withOpacity(0.7)
      ..style = PaintingStyle.fill;

    for (final obstacle in obstacles) {
      canvas.drawCircle(obstacle, 0.05, obstaclePaint);
    }
  }

  void _drawCurrentPath(Canvas canvas) {
    if (currentPath.isEmpty) return;

    final paint = Paint()
      ..color = currentTool == MapEditTool.eraser ? Colors.white : currentColor
      ..style = PaintingStyle.stroke
      ..strokeWidth = brushSize / 100 // Scale brush size for map coordinates
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;

    final path = Path();
    if (currentPath.isNotEmpty) {
      path.moveTo(currentPath.first.dx, currentPath.first.dy);
      for (int i = 1; i < currentPath.length; i++) {
        path.lineTo(currentPath[i].dx, currentPath[i].dy);
      }
    }

    canvas.drawPath(path, paint);
  }

  void _drawTempShape(Canvas canvas) {
    if (tempShapePoints.isEmpty) return;

    final paint = Paint()
      ..color = currentColor.withOpacity(0.5)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 0.02;

    switch (currentTool) {
      case MapEditTool.rectangle:
        if (tempShapePoints.length >= 2) {
          final rect = Rect.fromPoints(tempShapePoints[0], tempShapePoints[1]);
          canvas.drawRect(rect, paint);
        }
        break;
      case MapEditTool.circle:
        if (tempShapePoints.length >= 2) {
          final center = tempShapePoints[0];
          final radius = (tempShapePoints[1] - center).distance;
          canvas.drawCircle(center, radius, paint);
        }
        break;
      case MapEditTool.line:
        if (tempShapePoints.length >= 2) {
          canvas.drawLine(tempShapePoints[0], tempShapePoints[1], paint);
        }
        break;
      case MapEditTool.polygon:
        if (tempShapePoints.length >= 2) {
          final path = Path();
          path.moveTo(tempShapePoints[0].dx, tempShapePoints[0].dy);
          for (int i = 1; i < tempShapePoints.length; i++) {
            path.lineTo(tempShapePoints[i].dx, tempShapePoints[i].dy);
          }
          canvas.drawPath(path, paint);
        }
        break;
      default:
        break;
    }
  }

  void _drawOverlay(Canvas canvas, Size size) {
    // Draw scale indicator
    final textPainter = TextPainter(
      text: TextSpan(
        text: 'Scale: ${(scale * 100).toInt()}%',
        style: TextStyle(
          color: Colors.black,
          fontSize: 14,
          fontWeight: FontWeight.bold,
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    textPainter.layout();
    textPainter.paint(canvas, Offset(10, size.height - 30));

    // Draw coordinate indicator if robot is present
    if (showRealTimeData && currentOdometry != null) {
      final coordText = TextPainter(
        text: TextSpan(
          text: 'Robot: (${currentOdometry!.position.x.toStringAsFixed(2)}, ${currentOdometry!.position.y.toStringAsFixed(2)})',
          style: TextStyle(
            color: Colors.blue,
            fontSize: 12,
            fontWeight: FontWeight.bold,
          ),
        ),
        textDirection: TextDirection.ltr,
      );
      coordText.layout();
      coordText.paint(canvas, Offset(size.width - coordText.width - 10, size.height - 30));
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true;
  }
}