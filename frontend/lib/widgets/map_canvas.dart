// widgets/enhanced_map_canvas.dart - Complete costmap visualization support
import 'package:flutter/material.dart';
import 'dart:math' as math;
import 'dart:typed_data';
import '../models/map_data.dart';
import '../models/odom.dart';
import '../services/web_socket_service.dart';
import '../services/api_service.dart';

class EnhancedMapCanvas extends StatefulWidget {
  final MapData? mapData;
  final Function(MapData) onMapChanged;
  final String? deviceId;
  final bool enableRealTimeUpdates;

  const EnhancedMapCanvas({
    Key? key,
    this.mapData,
    required this.onMapChanged,
    this.deviceId,
    this.enableRealTimeUpdates = true,
  }) : super(key: key);

  @override
  _EnhancedMapCanvasState createState() => _EnhancedMapCanvasState();
}

class _EnhancedMapCanvasState extends State<EnhancedMapCanvas> 
    with TickerProviderStateMixin {
  
  final WebSocketService _webSocketService = WebSocketService();
  final ApiService _apiService = ApiService();
  
  // Enhanced costmap data storage
  Map<String, dynamic>? _globalCostmap;
  Map<String, dynamic>? _localCostmap;
  Map<String, dynamic>? _staticMap;
  
  // Real-time data
  OdometryData? _currentOdometry;
  List<Position> _robotTrail = [];
  
  // Display controls
  bool _showGlobalCostmap = true;
  bool _showLocalCostmap = true;
  bool _showStaticMap = true;
  bool _showRobotTrail = true;
  double _globalCostmapOpacity = 0.6;
  double _localCostmapOpacity = 0.8;
  double _staticMapOpacity = 0.9;
  
  // Map editing
  MapEditTool _currentTool = MapEditTool.none;
  LocationType _currentLocationType = LocationType.waypoint;
  List<Offset> _tempShapePoints = [];
  MapShape? _selectedShape;
  bool _isDrawing = false;
  
  // View controls
  double _scale = 1.0;
  Offset _panOffset = Offset.zero;
  Offset _lastPanPoint = Offset.zero;
  late AnimationController _robotAnimationController;

  @override
  void initState() {
    super.initState();
    
    _robotAnimationController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    )..repeat(reverse: true);
    
    if (widget.enableRealTimeUpdates) {
      _subscribeToRealTimeUpdates();
    }
  }

  @override
  void dispose() {
    _robotAnimationController.dispose();
    super.dispose();
  }

  void _subscribeToRealTimeUpdates() {
    // Subscribe to real-time data streams
    _webSocketService.realTimeData.listen((data) {
      if (data['deviceId'] == widget.deviceId || data['deviceId'] == null) {
        _handleRealTimeData(data);
      }
    });
  }

  void _handleRealTimeData(Map<String, dynamic> data) {
    if (!mounted) return;
    
    setState(() {
      switch (data['type']) {
        case 'position_update':
        case 'odometry_update':
          _handleOdometryUpdate(data['data']);
          break;
          
        case 'map_update':
          _handleMapUpdate(data['data']);
          break;
          
        case 'global_costmap_update':
          _handleGlobalCostmapUpdate(data['data']);
          break;
          
        case 'local_costmap_update':
          _handleLocalCostmapUpdate(data['data']);
          break;
      }
    });
  }

  void _handleOdometryUpdate(Map<String, dynamic> data) {
    try {
      _currentOdometry = OdometryData.fromJson(data);
      
      // Update robot trail
      if (_currentOdometry != null && _showRobotTrail) {
        _robotTrail.add(_currentOdometry!.position);
        if (_robotTrail.length > 500) {
          _robotTrail.removeAt(0);
        }
      }
    } catch (e) {
      print('❌ Error parsing odometry data: $e');
    }
  }

  void _handleMapUpdate(Map<String, dynamic> data) {
    try {
      _staticMap = data;
      print('🗺️ Updated static map: ${data['info']?['width']}x${data['info']?['height']}');
    } catch (e) {
      print('❌ Error parsing map data: $e');
    }
  }

  void _handleGlobalCostmapUpdate(Map<String, dynamic> data) {
    try {
      _globalCostmap = data;
      final info = data['info'];
      if (info != null) {
        print('🌍 Updated global costmap: ${info['width']}x${info['height']} @ ${info['resolution']}m/px');
      }
    } catch (e) {
      print('❌ Error parsing global costmap: $e');
    }
  }

  void _handleLocalCostmapUpdate(Map<String, dynamic> data) {
    try {
      _localCostmap = data;
      final info = data['info'];
      if (info != null) {
        print('🏠 Updated local costmap: ${info['width']}x${info['height']} @ ${info['resolution']}m/px');
      }
    } catch (e) {
      print('❌ Error parsing local costmap: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        _buildEnhancedToolbar(),
        Expanded(
          child: Container(
            decoration: BoxDecoration(
              border: Border.all(color: Colors.grey.shade300),
              borderRadius: BorderRadius.circular(8),
            ),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(8),
              child: Stack(
                children: [
                  // Main canvas
                  GestureDetector(
                    onScaleStart: _onScaleStart,
                    onScaleUpdate: _onScaleUpdate,
                    onTapDown: _onTapDown,
                    child: CustomPaint(
                      size: Size.infinite,
                      painter: EnhancedCostmapPainter(
                        staticMap: _staticMap,
                        globalCostmap: _globalCostmap,
                        localCostmap: _localCostmap,
                        currentOdometry: _currentOdometry,
                        robotTrail: _robotTrail,
                        selectedShape: _selectedShape,
                        tempShapePoints: _tempShapePoints,
                        scale: _scale,
                        panOffset: _panOffset,
                        currentTool: _currentTool,
                        currentLocationType: _currentLocationType,
                        // Display options
                        showStaticMap: _showStaticMap,
                        showGlobalCostmap: _showGlobalCostmap,
                        showLocalCostmap: _showLocalCostmap,
                        showRobotTrail: _showRobotTrail,
                        staticMapOpacity: _staticMapOpacity,
                        globalCostmapOpacity: _globalCostmapOpacity,
                        localCostmapOpacity: _localCostmapOpacity,
                        robotAnimationValue: _robotAnimationController.value,
                      ),
                    ),
                  ),
                  
                  // Status overlay
                  _buildStatusOverlay(),
                ],
              ),
            ),
          ),
        ),
        _buildCostmapControls(),
      ],
    );
  }

  Widget _buildEnhancedToolbar() {
    return Container(
      padding: EdgeInsets.all(12),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.blue.shade50, Colors.blue.shade100],
        ),
        borderRadius: BorderRadius.vertical(top: Radius.circular(8)),
        border: Border.all(color: Colors.blue.shade200),
      ),
      child: Column(
        children: [
          // Main controls
          Row(
            children: [
              Icon(Icons.map, color: Colors.blue.shade700),
              SizedBox(width: 8),
              Text(
                'Enhanced Map View',
                style: TextStyle(
                  fontSize: 16,
                  fontWeight: FontWeight.bold,
                  color: Colors.blue.shade700,
                ),
              ),
              Spacer(),
              
              // PGM Export Button
              ElevatedButton.icon(
                onPressed: _exportToPGM,
                icon: Icon(Icons.file_download),
                label: Text('Export PGM'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.green,
                  foregroundColor: Colors.white,
                ),
              ),
              SizedBox(width: 8),
              
              // Upload to AGV Button
              ElevatedButton.icon(
                onPressed: _uploadToAGV,
                icon: Icon(Icons.upload),
                label: Text('Upload to AGV'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.purple,
                  foregroundColor: Colors.white,
                ),
              ),
            ],
          ),
          
          SizedBox(height: 12),
          
          // Drawing tools
          Row(
            children: [
              Text('Tools: '),
              SizedBox(width: 8),
              ...MapEditTool.values.map((tool) => _buildToolButton(tool)),
              Spacer(),
              Text('Location: '),
              ...LocationType.values.take(3).map((type) => _buildLocationButton(type)),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildCostmapControls() {
    return Container(
      padding: EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.grey.shade50,
        border: Border.all(color: Colors.grey.shade300),
        borderRadius: BorderRadius.vertical(bottom: Radius.circular(8)),
      ),
      child: Column(
        children: [
          // Layer visibility controls
          Row(
            children: [
              Icon(Icons.layers, color: Colors.grey.shade700),
              SizedBox(width: 8),
              Text(
                'Layer Controls:',
                style: TextStyle(
                  fontWeight: FontWeight.bold,
                  color: Colors.grey.shade700,
                ),
              ),
              SizedBox(width: 16),
              
              _buildLayerToggle(
                'Static Map',
                _showStaticMap,
                Colors.grey.shade600,
                (value) => setState(() => _showStaticMap = value),
              ),
              
              _buildLayerToggle(
                'Global Costmap',
                _showGlobalCostmap,
                Colors.blue,
                (value) => setState(() => _showGlobalCostmap = value),
              ),
              
              _buildLayerToggle(
                'Local Costmap',
                _showLocalCostmap,
                Colors.red,
                (value) => setState(() => _showLocalCostmap = value),
              ),
              
              _buildLayerToggle(
                'Robot Trail',
                _showRobotTrail,
                Colors.orange,
                (value) => setState(() => _showRobotTrail = value),
              ),
            ],
          ),
          
          SizedBox(height: 12),
          
          // Opacity controls
          Row(
            children: [
              Text('Opacity:', style: TextStyle(fontWeight: FontWeight.bold)),
              SizedBox(width: 16),
              
              if (_showStaticMap) ...[
                Text('Map: '),
                SizedBox(
                  width: 100,
                  child: Slider(
                    value: _staticMapOpacity,
                    onChanged: (value) => setState(() => _staticMapOpacity = value),
                    activeColor: Colors.grey.shade600,
                  ),
                ),
                SizedBox(width: 16),
              ],
              
              if (_showGlobalCostmap) ...[
                Text('Global: '),
                SizedBox(
                  width: 100,
                  child: Slider(
                    value: _globalCostmapOpacity,
                    onChanged: (value) => setState(() => _globalCostmapOpacity = value),
                    activeColor: Colors.blue,
                  ),
                ),
                SizedBox(width: 16),
              ],
              
              if (_showLocalCostmap) ...[
                Text('Local: '),
                SizedBox(
                  width: 100,
                  child: Slider(
                    value: _localCostmapOpacity,
                    onChanged: (value) => setState(() => _localCostmapOpacity = value),
                    activeColor: Colors.red,
                  ),
                ),
              ],
            ],
          ),
          
          // Status info
          if (_currentOdometry != null || _globalCostmap != null || _localCostmap != null)
            Container(
              margin: EdgeInsets.only(top: 8),
              padding: EdgeInsets.all(8),
              decoration: BoxDecoration(
                color: Colors.blue.shade50,
                borderRadius: BorderRadius.circular(6),
                border: Border.all(color: Colors.blue.shade200),
              ),
              child: Row(
                children: [
                  if (_currentOdometry != null) ...[
                    Icon(Icons.location_on, size: 16, color: Colors.blue),
                    SizedBox(width: 4),
                    Text(
                      'Robot: (${_currentOdometry!.position.x.toStringAsFixed(2)}, ${_currentOdometry!.position.y.toStringAsFixed(2)})',
                      style: TextStyle(fontSize: 12, fontWeight: FontWeight.bold),
                    ),
                    SizedBox(width: 16),
                  ],
                  
                  if (_globalCostmap != null) ...[
                    Container(
                      width: 8,
                      height: 8,
                      decoration: BoxDecoration(
                        color: Colors.blue,
                        shape: BoxShape.circle,
                      ),
                    ),
                    SizedBox(width: 4),
                    Text(
                      'Global: ${_globalCostmap!['info']?['width'] ?? 'N/A'}×${_globalCostmap!['info']?['height'] ?? 'N/A'}',
                      style: TextStyle(fontSize: 12),
                    ),
                    SizedBox(width: 12),
                  ],
                  
                  if (_localCostmap != null) ...[
                    Container(
                      width: 8,
                      height: 8,
                      decoration: BoxDecoration(
                        color: Colors.red,
                        shape: BoxShape.circle,
                      ),
                    ),
                    SizedBox(width: 4),
                    Text(
                      'Local: ${_localCostmap!['info']?['width'] ?? 'N/A'}×${_localCostmap!['info']?['height'] ?? 'N/A'}',
                      style: TextStyle(fontSize: 12),
                    ),
                  ],
                ],
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildLayerToggle(
    String label,
    bool value,
    Color color,
    Function(bool) onChanged,
  ) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          width: 12,
          height: 12,
          decoration: BoxDecoration(
            color: value ? color : Colors.grey.shade300,
            shape: BoxShape.circle,
            border: Border.all(color: color),
          ),
        ),
        SizedBox(width: 4),
        Text(label, style: TextStyle(fontSize: 12)),
        SizedBox(width: 4),
        Switch(
          value: value,
          onChanged: onChanged,
          activeColor: color,
          materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
        ),
        SizedBox(width: 16),
      ],
    );
  }

  Widget _buildToolButton(MapEditTool tool) {
    final isSelected = _currentTool == tool;
    IconData icon;
    
    switch (tool) {
      case MapEditTool.none:
        icon = Icons.pan_tool;
        break;
      case MapEditTool.pencil:
        icon = Icons.brush;
        break;
      case MapEditTool.rectangle:
        icon = Icons.crop_square;
        break;
      case MapEditTool.circle:
        icon = Icons.circle_outlined;
        break;
      default:
        icon = Icons.edit;
    }

    return Padding(
      padding: EdgeInsets.only(right: 4),
      child: ElevatedButton(
        onPressed: () => setState(() => _currentTool = tool),
        style: ElevatedButton.styleFrom(
          backgroundColor: isSelected ? Colors.blue : Colors.grey.shade300,
          foregroundColor: isSelected ? Colors.white : Colors.black,
          minimumSize: Size(32, 32),
          padding: EdgeInsets.all(4),
        ),
        child: Icon(icon, size: 16),
      ),
    );
  }

  Widget _buildLocationButton(LocationType type) {
    final isSelected = _currentLocationType == type;
    final colors = {
      LocationType.pickup: Colors.green,
      LocationType.drop: Colors.blue,
      LocationType.charging: Colors.orange,
      LocationType.waypoint: Colors.purple,
      LocationType.obstacle: Colors.red,
    };
    
    final color = colors[type]!;
    final name = type.toString().split('.').last.toUpperCase();

    return Padding(
      padding: EdgeInsets.only(right: 4),
      child: ElevatedButton(
        onPressed: () => setState(() => _currentLocationType = type),
        style: ElevatedButton.styleFrom(
          backgroundColor: isSelected ? color : Colors.grey.shade300,
          foregroundColor: isSelected ? Colors.white : Colors.black,
          padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
        ),
        child: Text(name, style: TextStyle(fontSize: 10)),
      ),
    );
  }

  Widget _buildStatusOverlay() {
    return Positioned(
      top: 16,
      right: 16,
      child: Container(
        padding: EdgeInsets.all(8),
        decoration: BoxDecoration(
          color: Colors.black.withOpacity(0.7),
          borderRadius: BorderRadius.circular(8),
        ),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Live Data Status',
              style: TextStyle(
                color: Colors.white,
                fontWeight: FontWeight.bold,
                fontSize: 12,
              ),
            ),
            SizedBox(height: 4),
            _buildStatusIndicator('Static Map', _staticMap != null, Colors.grey),
            _buildStatusIndicator('Global Costmap', _globalCostmap != null, Colors.blue),
            _buildStatusIndicator('Local Costmap', _localCostmap != null, Colors.red),
            _buildStatusIndicator('Robot Position', _currentOdometry != null, Colors.green),
          ],
        ),
      ),
    );
  }

  Widget _buildStatusIndicator(String label, bool active, Color color) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 1),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 6,
            height: 6,
            decoration: BoxDecoration(
              color: active ? color : Colors.grey,
              shape: BoxShape.circle,
            ),
          ),
          SizedBox(width: 4),
          Text(
            label,
            style: TextStyle(
              color: Colors.white,
              fontSize: 10,
            ),
          ),
        ],
      ),
    );
  }

  // Event handlers
  void _onScaleStart(ScaleStartDetails details) {
    _lastPanPoint = details.localFocalPoint;
  }

  void _onScaleUpdate(ScaleUpdateDetails details) {
    setState(() {
      _scale = (_scale * details.scale).clamp(0.1, 5.0);
      if (details.scale == 1.0) {
        final delta = details.localFocalPoint - _lastPanPoint;
        _panOffset += delta;
        _lastPanPoint = details.localFocalPoint;
      }
    });
  }

  void _onTapDown(TapDownDetails details) {
    // Handle shape selection or creation
    if (_currentTool != MapEditTool.none) {
      _handleShapeCreation(details.localPosition);
    }
  }

  void _handleShapeCreation(Offset position) {
    // Convert screen coordinates to map coordinates
    final mapPoint = _screenToMapCoordinates(position);
    
    // Add shape creation logic here
    setState(() {
      _tempShapePoints.add(mapPoint);
    });
  }

  Offset _screenToMapCoordinates(Offset screenPoint) {
    return (screenPoint - _panOffset) / _scale;
  }

  // PGM Export and Upload functionality
  Future<void> _exportToPGM() async {
    if (widget.deviceId == null) {
      _showErrorSnackBar('No device selected');
      return;
    }

    try {
      _showInfoSnackBar('Exporting map to PGM format...');
      
      final response = await _apiService.exportMapToPGM(
        deviceId: widget.deviceId!,
        includeGlobalCostmap: _showGlobalCostmap,
        includeLocalCostmap: _showLocalCostmap,
        includeStaticMap: _showStaticMap,
      );

      if (response['success'] == true) {
        _showSuccessSnackBar('Map exported to PGM successfully! Files: ${response['files']}');
      } else {
        _showErrorSnackBar('Export failed: ${response['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Export failed: $e');
    }
  }

  Future<void> _uploadToAGV() async {
    if (widget.deviceId == null) {
      _showErrorSnackBar('No device selected');
      return;
    }

    try {
      _showInfoSnackBar('Uploading map to AGV...');
      
      final response = await _apiService.uploadMapToAGV(
        deviceId: widget.deviceId!,
        mapName: 'edited_map_${DateTime.now().millisecondsSinceEpoch}',
      );

      if (response['success'] == true) {
        _showSuccessSnackBar('Map uploaded to AGV successfully!');
      } else {
        _showErrorSnackBar('Upload failed: ${response['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Upload failed: $e');
    }
  }

  // Helper methods
  void _showInfoSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.blue,
        duration: Duration(seconds: 2),
      ),
    );
  }

  void _showSuccessSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.green,
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

// Enhanced costmap painter
class EnhancedCostmapPainter extends CustomPainter {
  final Map<String, dynamic>? staticMap;
  final Map<String, dynamic>? globalCostmap;
  final Map<String, dynamic>? localCostmap;
  final OdometryData? currentOdometry;
  final List<Position> robotTrail;
  final MapShape? selectedShape;
  final List<Offset> tempShapePoints;
  final double scale;
  final Offset panOffset;
  final MapEditTool currentTool;
  final LocationType currentLocationType;
  
  final bool showStaticMap;
  final bool showGlobalCostmap;
  final bool showLocalCostmap;
  final bool showRobotTrail;
  final double staticMapOpacity;
  final double globalCostmapOpacity;
  final double localCostmapOpacity;
  final double robotAnimationValue;

  EnhancedCostmapPainter({
    this.staticMap,
    this.globalCostmap,
    this.localCostmap,
    this.currentOdometry,
    this.robotTrail = const [],
    this.selectedShape,
    this.tempShapePoints = const [],
    required this.scale,
    required this.panOffset,
    required this.currentTool,
    required this.currentLocationType,
    required this.showStaticMap,
    required this.showGlobalCostmap,
    required this.showLocalCostmap,
    required this.showRobotTrail,
    required this.staticMapOpacity,
    required this.globalCostmapOpacity,
    required this.localCostmapOpacity,
    required this.robotAnimationValue,
  });

  @override
  void paint(Canvas canvas, Size size) {
    // Draw background grid
    _drawGrid(canvas, size);
    
    // Apply transformations
    canvas.save();
    canvas.translate(panOffset.dx + size.width / 2, panOffset.dy + size.height / 2);
    canvas.scale(scale);

    // Draw layers in order (bottom to top)
    if (showStaticMap && staticMap != null) {
      _drawOccupancyGrid(canvas, staticMap!, Colors.grey, staticMapOpacity);
    }
    
    if (showGlobalCostmap && globalCostmap != null) {
      _drawCostmap(canvas, globalCostmap!, Colors.blue, globalCostmapOpacity);
    }
    
    if (showLocalCostmap && localCostmap != null) {
      _drawCostmap(canvas, localCostmap!, Colors.red, localCostmapOpacity);
    }
    
    // Draw robot trail
    if (showRobotTrail && robotTrail.isNotEmpty) {
      _drawRobotTrail(canvas);
    }
    
    // Draw current robot position
    if (currentOdometry != null) {
      _drawRobot(canvas);
    }
    
    // Draw temporary shapes
    _drawTempShapes(canvas);

    canvas.restore();
  }

  void _drawGrid(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = Colors.grey.withOpacity(0.2)
      ..strokeWidth = 0.5;

    const gridSpacing = 50.0; // 1 meter = 50 pixels at scale 1.0
    
    for (double x = 0; x < size.width; x += gridSpacing) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
    }
    
    for (double y = 0; y < size.height; y += gridSpacing) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);
    }
    
    // Draw axes
    final axisPaint = Paint()
      ..color = Colors.grey.withOpacity(0.5)
      ..strokeWidth = 2.0;
    
    canvas.drawLine(
      Offset(size.width / 2, 0),
      Offset(size.width / 2, size.height),
      axisPaint,
    );
    canvas.drawLine(
      Offset(0, size.height / 2),
      Offset(size.width, size.height / 2),
      axisPaint,
    );
  }

  void _drawOccupancyGrid(Canvas canvas, Map<String, dynamic> mapData, Color baseColor, double opacity) {
    final info = mapData['info'];
    final data = mapData['data'] as List?;
    
    if (info == null || data == null) return;
    
    final width = info['width'] as int? ?? 0;
    final height = info['height'] as int? ?? 0;
    final resolution = info['resolution'] as double? ?? 0.05;
    final origin = info['origin'];
    
    if (width == 0 || height == 0) return;
    
    final originX = origin?['position']?['x'] as double? ?? 0.0;
    final originY = origin?['position']?['y'] as double? ?? 0.0;
    
    final cellSize = resolution * 50.0; // Scale factor
    
    for (int y = 0; y < height; y += 2) {
      for (int x = 0; x < width; x += 2) {
        final index = y * width + x;
        if (index >= data.length) continue;
        
        final value = data[index] as int? ?? -1;
        
        Color cellColor;
        if (value == -1) {
          continue; // Skip unknown
        } else if (value == 0) {
          cellColor = Colors.white.withOpacity(opacity);
        } else if (value == 100) {
          cellColor = Colors.black.withOpacity(opacity);
        } else {
          final intensity = (value / 100.0);
          cellColor = Color.lerp(Colors.white, Colors.black, intensity)!.withOpacity(opacity);
        }
        
        final paint = Paint()
          ..color = cellColor
          ..style = PaintingStyle.fill;
        
        final screenX = originX + x * resolution;
        final screenY = -(originY + y * resolution);
        
        canvas.drawRect(
          Rect.fromLTWH(screenX * 50, screenY * 50, cellSize * 2, cellSize * 2),
          paint,
        );
      }
    }
  }

  void _drawCostmap(Canvas canvas, Map<String, dynamic> costmapData, Color baseColor, double opacity) {
    final info = costmapData['info'];
    final data = costmapData['data'] as List?;
    
    if (info == null || data == null) return;
    
    final width = info['width'] as int? ?? 0;
    final height = info['height'] as int? ?? 0;
    final resolution = info['resolution'] as double? ?? 0.05;
    final origin = info['origin'];
    
    if (width == 0 || height == 0) return;
    
    final originX = origin?['position']?['x'] as double? ?? 0.0;
    final originY = origin?['position']?['y'] as double? ?? 0.0;
    
    final cellSize = resolution * 50.0;
    
    for (int y = 0; y < height; y += 2) {
      for (int x = 0; x < width; x += 2) {
        final index = y * width + x;
        if (index >= data.length) continue;
        
        final value = data[index] as int? ?? 0;
        
        if (value == 0) continue; // Skip free space
        
        // Enhanced costmap visualization
        Color cellColor;
        if (value >= 100) {
          // Lethal obstacle - full intensity
          cellColor = baseColor.withOpacity(opacity);
        } else if (value >= 99) {
          // Inscribed obstacle - high intensity
          cellColor = baseColor.withOpacity(opacity * 0.8);
        } else if (value >= 50) {
          // Medium cost
          cellColor = baseColor.withOpacity(opacity * 0.5);
        } else {
          // Low cost
          cellColor = baseColor.withOpacity(opacity * 0.3);
        }
        
        final paint = Paint()
          ..color = cellColor
          ..style = PaintingStyle.fill;
        
        final screenX = originX + x * resolution;
        final screenY = -(originY + y * resolution);
        
        canvas.drawRect(
          Rect.fromLTWH(screenX * 50, screenY * 50, cellSize * 2, cellSize * 2),
          paint,
        );
      }
    }
  }

  void _drawRobotTrail(Canvas canvas) {
    if (robotTrail.length < 2) return;
    
    final trailPaint = Paint()
      ..color = Colors.orange.withOpacity(0.6)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0
      ..strokeCap = StrokeCap.round;
    
    final path = Path();
    final firstPoint = robotTrail.first;
    path.moveTo(firstPoint.x * 50, -firstPoint.y * 50);
    
    for (int i = 1; i < robotTrail.length; i++) {
      final point = robotTrail[i];
      path.lineTo(point.x * 50, -point.y * 50);
    }
    
    canvas.drawPath(path, trailPaint);
    
    // Draw trail points
    for (int i = robotTrail.length - 20; i < robotTrail.length; i++) {
      if (i < 0) continue;
      
      final point = robotTrail[i];
      final age = (robotTrail.length - i) / 20.0;
      final opacity = 1.0 - age;
      
      final pointPaint = Paint()
        ..color = Colors.orange.withOpacity(opacity * 0.8)
        ..style = PaintingStyle.fill;
      
      canvas.drawCircle(
        Offset(point.x * 50, -point.y * 50),
        2.0 * (1.0 - age * 0.5),
        pointPaint,
      );
    }
  }

  void _drawRobot(Canvas canvas) {
    final position = currentOdometry!.position;
    final orientation = currentOdometry!.orientation;
    
    final x = position.x * 50;
    final y = -position.y * 50;
    final yaw = orientation.yaw;
    
    // Robot body with animation
    final robotSize = 8.0 + (robotAnimationValue * 2.0);
    
    final robotPaint = Paint()
      ..color = Colors.blue
      ..style = PaintingStyle.fill;
    
    final robotOutlinePaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 2.0
      ..style = PaintingStyle.stroke;
    
    canvas.drawCircle(Offset(x, y), robotSize, robotPaint);
    canvas.drawCircle(Offset(x, y), robotSize, robotOutlinePaint);
    
    // Direction indicator
    final directionLength = robotSize * 1.5;
    final directionX = x + math.cos(yaw) * directionLength;
    final directionY = y - math.sin(yaw) * directionLength;
    
    final directionPaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 3.0
      ..strokeCap = StrokeCap.round;
    
    canvas.drawLine(Offset(x, y), Offset(directionX, directionY), directionPaint);
    
    // Draw arrow head
    final arrowPaint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.fill;
    
    final arrowPath = Path();
    final arrowSize = 4.0;
    arrowPath.moveTo(directionX, directionY);
    arrowPath.lineTo(
      directionX - arrowSize * math.cos(yaw - 2.5),
      directionY + arrowSize * math.sin(yaw - 2.5),
    );
    arrowPath.lineTo(
      directionX - arrowSize * math.cos(yaw + 2.5),
      directionY + arrowSize * math.sin(yaw + 2.5),
    );
    arrowPath.close();
    canvas.drawPath(arrowPath, arrowPaint);
  }

  void _drawTempShapes(Canvas canvas) {
    if (tempShapePoints.isEmpty) return;
    
    final shapePaint = Paint()
      ..color = _getLocationColor().withOpacity(0.6)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;
    
    if (tempShapePoints.length == 1) {
      // Single point
      canvas.drawCircle(
        Offset(tempShapePoints[0].dx * 50, -tempShapePoints[0].dy * 50),
        5.0,
        shapePaint,
      );
    } else if (tempShapePoints.length >= 2) {
      // Multiple points - draw shape
      final path = Path();
      path.moveTo(tempShapePoints[0].dx * 50, -tempShapePoints[0].dy * 50);
      
      for (int i = 1; i < tempShapePoints.length; i++) {
        path.lineTo(tempShapePoints[i].dx * 50, -tempShapePoints[i].dy * 50);
      }
      
      if (currentTool == MapEditTool.rectangle || currentTool == MapEditTool.circle) {
        path.close();
      }
      
      canvas.drawPath(path, shapePaint);
    }
  }

  Color _getLocationColor() {
    switch (currentLocationType) {
      case LocationType.pickup:
        return Colors.green;
      case LocationType.drop:
        return Colors.blue;
      case LocationType.charging:
        return Colors.orange;
      case LocationType.waypoint:
        return Colors.purple;
      case LocationType.obstacle:
        return Colors.red;
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true;
  }
}

// Enums (if not already defined)
enum MapEditTool { none, pencil, rectangle, circle }
enum LocationType { pickup, drop, charging, waypoint, obstacle }