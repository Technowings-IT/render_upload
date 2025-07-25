// widgets/enhanced_pgm_map_editor.dart - GIMP-like Map Editor with Advanced Tools
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' as ui;
import 'dart:async';
import '../models/map_data.dart';
import '../models/odom.dart' as odom;
import '../services/api_service.dart';
// Ensure Position is imported from the correct file

// Enhanced editing tools (GIMP-like)
enum EditTool {
  select, // Selection tool
  pan, // Pan/move view
  zoom, // Zoom tool
  pencil, // Freehand drawing
  brush, // Brush with pressure
  eraser, // Eraser tool
  bucket, // Flood fill
  line, // Straight line
  rectangle, // Rectangle shape
  circle, // Circle/ellipse
  polygon, // Polygon shape
  text, // Text tool
}

// Map layers (GIMP-like layer system)
enum MapLayer {
  obstacles(0, 'Obstacles', Colors.black),
  free(254, 'Free Space', Colors.white),
  unknown(128, 'Unknown', Colors.grey),
  restricted(64, 'Restricted', Colors.red),
  preferred(192, 'Preferred Path', Colors.green);

  const MapLayer(this.value, this.name, this.color);
  final int value;
  final String name;
  final Color color;
}

// Location point types (your 4 required types + extras)
enum LocationPointType {
  pickup('Pickup Point', '📦', Colors.green),
  drop('Drop Point', '📍', Colors.blue),
  home('Home Position', '🏠', Colors.orange),
  charging('Charging Station', '🔋', Colors.yellow),
  waypoint('Waypoint', '📌', Colors.purple),
  obstacle('Obstacle', '🚫', Colors.red);

  const LocationPointType(this.displayName, this.icon, this.color);
  final String displayName;
  final String icon;
  final Color color;
}

// Enhanced PGM Map Editor
class EnhancedPGMMapEditor extends StatefulWidget {
  final MapData? mapData;
  final Function(MapData) onMapChanged;
  final String? deviceId;

  const EnhancedPGMMapEditor({
    Key? key,
    this.mapData,
    required this.onMapChanged,
    this.deviceId,
  }) : super(key: key);

  @override
  _EnhancedPGMMapEditorState createState() => _EnhancedPGMMapEditorState();
}

class _EnhancedPGMMapEditorState extends State<EnhancedPGMMapEditor>
    with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();

  // Enhanced image data management
  Uint8List? _originalPgmData;
  Uint8List? _currentPgmData;
  ui.Image? _mapImage;
  ui.Image? _overlayImage; // For temporary drawings
  int _mapWidth = 0;
  int _mapHeight = 0;

  // Tool state
  EditTool _currentTool = EditTool.pencil;
  MapLayer _currentLayer = MapLayer.obstacles;
  LocationPointType _currentPointType = LocationPointType.pickup;

  // Brush settings
  double _brushSize = 5.0;
  double _brushOpacity = 1.0;
  double _brushHardness = 0.8;

  // Drawing state
  List<DrawingStroke> _currentStroke = [];
  List<DrawingStroke> _undoStack = [];
  List<DrawingStroke> _redoStack = [];
  bool _isDrawing = false;

  // Location points
  List<LocationPoint> _locationPoints = [];
  LocationPoint? _selectedPoint;

  // Selection state
  Rect? _selectionRect;
  bool _hasSelection = false;
  Uint8List? _clipboardData;

  // View transformation
  double _scale = 1.0;
  Offset _panOffset = Offset.zero;
  final double _minScale = 0.1;
  final double _maxScale = 20.0;

  // Animation controllers
  late AnimationController _brushPreviewController;
  late AnimationController _toolSwitchController;

  // Layer management
  List<EditorLayer> _layers = [];
  int _activeLayerIndex = 0;
  bool _showLayerPanel = false;

  // State management
  bool _isLoading = false;
  String? _error;
  bool _hasUnsavedChanges = false;

  @override
  void initState() {
    super.initState();

    _brushPreviewController = AnimationController(
      duration: Duration(milliseconds: 200),
      vsync: this,
    );

    _toolSwitchController = AnimationController(
      duration: Duration(milliseconds: 150),
      vsync: this,
    );

    _initializeLayers();

    if (widget.mapData != null) {
      _loadMapData();
    } else {
      _createNewMap();
    }
  }

  @override
  void dispose() {
    _brushPreviewController.dispose();
    _toolSwitchController.dispose();
    super.dispose();
  }

  void _initializeLayers() {
    _layers = [
      EditorLayer(
        id: 'background',
        name: 'Background',
        type: LayerType.background,
        opacity: 1.0,
        visible: true,
        locked: false,
      ),
      EditorLayer(
        id: 'obstacles',
        name: 'Obstacles',
        type: LayerType.obstacles,
        opacity: 1.0,
        visible: true,
        locked: false,
      ),
      EditorLayer(
        id: 'locations',
        name: 'Location Points',
        type: LayerType.locations,
        opacity: 1.0,
        visible: true,
        locked: false,
      ),
    ];
  }

  Future<void> _loadMapData() async {
    setState(() {
      _isLoading = true;
      _error = null;
    });

    try {
      await _convertMapToPGM();
    } catch (e) {
      setState(() {
        _error = 'Failed to load map: $e';
      });
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  Future<void> _createNewMap() async {
    setState(() {
      _isLoading = true;
    });

    try {
      // Create a new blank map
      const width = 500;
      const height = 500;
      final data = Uint8List(width * height);

      // Fill with unknown (gray) initially
      for (int i = 0; i < data.length; i++) {
        data[i] = MapLayer.unknown.value;
      }

      await _loadPGMImage(data, width, height);
    } catch (e) {
      setState(() {
        _error = 'Failed to create new map: $e';
      });
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  Future<void> _convertMapToPGM() async {
    if (widget.mapData == null) return;

    try {
      // Use existing conversion logic but enhanced
      final mapData = widget.mapData!;
      _mapWidth = mapData.info.width;
      _mapHeight = mapData.info.height;

      final pgmData = Uint8List(_mapWidth * _mapHeight);

      for (int i = 0;
          i < pgmData.length && i < mapData.occupancyData.length;
          i++) {
        final occupancy = mapData.occupancyData[i];

        if (occupancy == -1) {
          pgmData[i] = MapLayer.unknown.value;
        } else if (occupancy >= 65) {
          pgmData[i] = MapLayer.obstacles.value;
        } else {
          pgmData[i] = MapLayer.free.value;
        }
      }

      await _loadPGMImage(pgmData, _mapWidth, _mapHeight);

      // Load existing shapes as location points
      _loadExistingShapes();
    } catch (e) {
      throw Exception('PGM conversion failed: $e');
    }
  }

  void _loadExistingShapes() {
    if (widget.mapData?.shapes == null) return;

    _locationPoints.clear();

    for (final shape in widget.mapData!.shapes) {
      if (shape.points.isNotEmpty) {
        final point = LocationPoint(
          id: shape.id,
          name: shape.name,
          type: _parseLocationPointType(shape.type),
          position: Offset(shape.points.first.x, shape.points.first.y),
          properties: {'color': shape.color},
        );
        _locationPoints.add(point);
      }
    }
  }

  LocationPointType _parseLocationPointType(String type) {
    switch (type.toLowerCase()) {
      case 'pickup':
        return LocationPointType.pickup;
      case 'drop':
        return LocationPointType.drop;
      case 'home':
        return LocationPointType.home;
      case 'charging':
        return LocationPointType.charging;
      case 'waypoint':
        return LocationPointType.waypoint;
      default:
        return LocationPointType.waypoint;
    }
  }

  Future<void> _loadPGMImage(Uint8List pgmData, int width, int height) async {
    try {
      _mapWidth = width;
      _mapHeight = height;
      _originalPgmData = Uint8List.fromList(pgmData);
      _currentPgmData = Uint8List.fromList(pgmData);

      // Convert to RGBA for display
      final rgbaData = Uint8List(width * height * 4);
      for (int i = 0; i < pgmData.length; i++) {
        final grayValue = pgmData[i];
        final rgbaIndex = i * 4;
        rgbaData[rgbaIndex] = grayValue; // R
        rgbaData[rgbaIndex + 1] = grayValue; // G
        rgbaData[rgbaIndex + 2] = grayValue; // B
        rgbaData[rgbaIndex + 3] = 255; // A
      }

      final completer = Completer<ui.Image>();
      ui.decodeImageFromPixels(
        rgbaData,
        width,
        height,
        ui.PixelFormat.rgba8888,
        completer.complete,
      );

      final image = await completer.future;
      setState(() {
        _mapImage = image;
        _error = null;
      });
    } catch (e) {
      throw Exception('Failed to load PGM image: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.grey[100],
      body: Column(
        children: [
          _buildEnhancedToolbar(),
          Expanded(
            child: Row(
              children: [
                _buildToolPanel(),
                Expanded(child: _buildCanvasArea()),
                if (_showLayerPanel) _buildLayerPanel(),
                _buildPropertiesPanel(),
              ],
            ),
          ),
          _buildStatusBar(),
        ],
      ),
    );
  }

  Widget _buildEnhancedToolbar() {
    return Container(
      height: 60,
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.blue[800]!, Colors.blue[600]!],
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.2),
            blurRadius: 4,
            offset: Offset(0, 2),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Row(
          children: [
            // Logo/Title
            Icon(Icons.edit, color: Colors.white, size: 24),
            SizedBox(width: 8),
            Text(
              'Enhanced Map Editor',
              style: TextStyle(
                color: Colors.white,
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),

            SizedBox(width: 24),

            // File operations
            _buildToolbarSection([
              _buildToolbarButton(Icons.folder_open, 'Open', _openMap),
              _buildToolbarButton(Icons.save, 'Save', _saveMap),
              _buildToolbarButton(Icons.save_as, 'Export', _exportMap),
            ]),

            _buildToolbarDivider(),

            // Edit operations
            _buildToolbarSection([
              _buildToolbarButton(
                Icons.undo,
                'Undo',
                _undoStack.isNotEmpty ? _undo : null,
              ),
              _buildToolbarButton(
                Icons.redo,
                'Redo',
                _redoStack.isNotEmpty ? _redo : null,
              ),
              _buildToolbarButton(
                  Icons.content_cut, 'Cut', _hasSelection ? _cut : null),
              _buildToolbarButton(
                  Icons.content_copy, 'Copy', _hasSelection ? _copy : null),
              _buildToolbarButton(Icons.content_paste, 'Paste',
                  _clipboardData != null ? _paste : null),
            ]),

            _buildToolbarDivider(),

            // View operations
            _buildToolbarSection([
              _buildToolbarButton(Icons.zoom_in, 'Zoom In', _zoomIn),
              _buildToolbarButton(Icons.zoom_out, 'Zoom Out', _zoomOut),
              _buildToolbarButton(
                  Icons.center_focus_strong, 'Fit', _fitToScreen),
            ]),

            _buildToolbarDivider(),

            // Layer operations
            _buildToolbarSection([
              _buildToolbarToggle(
                Icons.layers,
                'Layers',
                _showLayerPanel,
                () => setState(() => _showLayerPanel = !_showLayerPanel),
              ),
            ]),

            Spacer(),

            // Scale display
            Container(
              padding: EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              decoration: BoxDecoration(
                color: Colors.white.withOpacity(0.2),
                borderRadius: BorderRadius.circular(4),
              ),
              child: Text(
                '${(_scale * 100).toInt()}%',
                style:
                    TextStyle(color: Colors.white, fontWeight: FontWeight.bold),
              ),
            ),

            SizedBox(width: 16),

            // Deploy operations
            _buildToolbarSection([
              ElevatedButton.icon(
                onPressed: _deployToRaspberryPi,
                icon: Icon(Icons.upload, size: 18),
                label: Text('Deploy to Pi'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.green,
                  foregroundColor: Colors.white,
                ),
              ),
            ]),
          ],
        ),
      ),
    );
  }

  Widget _buildToolbarSection(List<Widget> children) {
    return Row(children: children);
  }

  Widget _buildToolbarDivider() {
    return Container(
      width: 1,
      height: 24,
      color: Colors.white.withOpacity(0.3),
      margin: EdgeInsets.symmetric(horizontal: 12),
    );
  }

  Widget _buildToolbarButton(
      IconData icon, String tooltip, VoidCallback? onPressed) {
    return Tooltip(
      message: tooltip,
      child: IconButton(
        onPressed: onPressed,
        icon: Icon(icon,
            color: onPressed != null ? Colors.white : Colors.white54),
        iconSize: 20,
      ),
    );
  }

  Widget _buildToolbarToggle(
      IconData icon, String tooltip, bool isActive, VoidCallback onPressed) {
    return Tooltip(
      message: tooltip,
      child: IconButton(
        onPressed: onPressed,
        icon: Icon(icon, color: isActive ? Colors.yellow : Colors.white),
        iconSize: 20,
      ),
    );
  }

  Widget _buildToolPanel() {
    return Container(
      width: 100,
      decoration: BoxDecoration(
        color: Colors.grey[800],
        border: Border(right: BorderSide(color: Colors.grey[600]!)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.2),
            blurRadius: 4,
            offset: Offset(2, 0),
          ),
        ],
      ),
      child: SingleChildScrollView(
        child: Column(
          children: [
            // Tool selection
            Container(
              padding: EdgeInsets.all(8),
              child: Column(
                children: [
                  Text(
                    'Tools',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 12,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  SizedBox(height: 8),
                  GridView.count(
                    shrinkWrap: true,
                    physics: NeverScrollableScrollPhysics(),
                    crossAxisCount: 2,
                    crossAxisSpacing: 4,
                    mainAxisSpacing: 4,
                    children: EditTool.values
                        .map((tool) => _buildToolButton(tool))
                        .toList(),
                  ),
                ],
              ),
            ),

            Divider(color: Colors.grey[600]),

            // Layer selection
            Container(
              padding: EdgeInsets.all(8),
              child: Column(
                children: [
                  Text(
                    'Layer',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 12,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  SizedBox(height: 8),
                  ...MapLayer.values.map((layer) => _buildLayerButton(layer)),
                ],
              ),
            ),

            Divider(color: Colors.grey[600]),

            // Point type selection
            Container(
              padding: EdgeInsets.all(8),
              child: Column(
                children: [
                  Text(
                    'Points',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 12,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  SizedBox(height: 8),
                  ...LocationPointType.values
                      .map((type) => _buildPointTypeButton(type)),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildToolButton(EditTool tool) {
    final isSelected = _currentTool == tool;
    final icon = _getToolIcon(tool);

    return AnimatedContainer(
      duration: Duration(milliseconds: 150),
      margin: EdgeInsets.all(2),
      decoration: BoxDecoration(
        color: isSelected ? Colors.blue : Colors.grey[700],
        borderRadius: BorderRadius.circular(6),
        border:
            isSelected ? Border.all(color: Colors.blue[300]!, width: 2) : null,
      ),
      child: InkWell(
        onTap: () => _selectTool(tool),
        borderRadius: BorderRadius.circular(6),
        child: Container(
          height: 36,
          child: Icon(
            icon,
            color: Colors.white,
            size: 20,
          ),
        ),
      ),
    );
  }

  Widget _buildLayerButton(MapLayer layer) {
    final isSelected = _currentLayer == layer;

    return Container(
      margin: EdgeInsets.only(bottom: 4),
      child: InkWell(
        onTap: () => setState(() => _currentLayer = layer),
        borderRadius: BorderRadius.circular(4),
        child: Container(
          padding: EdgeInsets.symmetric(horizontal: 8, vertical: 6),
          decoration: BoxDecoration(
            color:
                isSelected ? Colors.blue.withOpacity(0.3) : Colors.transparent,
            borderRadius: BorderRadius.circular(4),
            border: isSelected ? Border.all(color: Colors.blue) : null,
          ),
          child: Row(
            children: [
              Container(
                width: 12,
                height: 12,
                decoration: BoxDecoration(
                  color: layer.color,
                  shape: BoxShape.circle,
                  border: Border.all(color: Colors.white),
                ),
              ),
              SizedBox(width: 8),
              Expanded(
                child: Text(
                  layer.name,
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 10,
                  ),
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildPointTypeButton(LocationPointType type) {
    final isSelected = _currentPointType == type;

    return Container(
      margin: EdgeInsets.only(bottom: 4),
      child: InkWell(
        onTap: () => setState(() => _currentPointType = type),
        borderRadius: BorderRadius.circular(4),
        child: Container(
          padding: EdgeInsets.symmetric(horizontal: 6, vertical: 4),
          decoration: BoxDecoration(
            color:
                isSelected ? type.color.withOpacity(0.3) : Colors.transparent,
            borderRadius: BorderRadius.circular(4),
            border: isSelected ? Border.all(color: type.color) : null,
          ),
          child: Row(
            children: [
              Text(
                type.icon,
                style: TextStyle(fontSize: 12),
              ),
              SizedBox(width: 4),
              Expanded(
                child: Text(
                  type.displayName,
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 9,
                  ),
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildCanvasArea() {
    if (_isLoading) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            CircularProgressIndicator(),
            SizedBox(height: 16),
            Text('Loading map data...'),
          ],
        ),
      );
    }

    if (_error != null) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.error, size: 64, color: Colors.red),
            SizedBox(height: 16),
            Text(
              'Error: $_error',
              textAlign: TextAlign.center,
              style: TextStyle(color: Colors.red[700]),
            ),
            SizedBox(height: 16),
            ElevatedButton(
              onPressed: _createNewMap,
              child: Text('Create New Map'),
            ),
          ],
        ),
      );
    }

    if (_mapImage == null) {
      return Center(child: Text('No map data available'));
    }

    return Container(
      decoration: BoxDecoration(
        color: Colors.grey[300],
        border: Border.all(color: Colors.grey[400]!),
      ),
      child: ClipRect(
        child: GestureDetector(
          onScaleStart: _handleScaleStart,
          onScaleUpdate: _handleScaleUpdate,
          onScaleEnd: _handleScaleEnd,
          onTapDown: _handleTapDown,
          child: CustomPaint(
            size: Size.infinite,
            painter: EnhancedMapPainter(
              mapImage: _mapImage!,
              overlayImage: _overlayImage,
              scale: _scale,
              panOffset: _panOffset,
              currentStroke: _currentStroke,
              locationPoints: _locationPoints,
              selectedPoint: _selectedPoint,
              selectionRect: _selectionRect,
              currentTool: _currentTool,
              currentLayer: _currentLayer,
              currentPointType: _currentPointType,
              brushSize: _brushSize,
              layers: _layers,
              activeLayerIndex: _activeLayerIndex,
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildLayerPanel() {
    return Container(
      width: 200,
      decoration: BoxDecoration(
        color: Colors.grey[800],
        border: Border(left: BorderSide(color: Colors.grey[600]!)),
      ),
      child: Column(
        children: [
          Container(
            padding: EdgeInsets.all(16),
            decoration: BoxDecoration(
              border: Border(bottom: BorderSide(color: Colors.grey[600]!)),
            ),
            child: Row(
              children: [
                Icon(Icons.layers, color: Colors.white),
                SizedBox(width: 8),
                Text(
                  'Layers',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                Spacer(),
                IconButton(
                  onPressed: () => setState(() => _showLayerPanel = false),
                  icon: Icon(Icons.close, color: Colors.white),
                ),
              ],
            ),
          ),
          Expanded(
            child: ListView.builder(
              itemCount: _layers.length,
              itemBuilder: (context, index) {
                final layer = _layers[index];
                final isActive = index == _activeLayerIndex;

                return _buildLayerItem(layer, index, isActive);
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildLayerItem(EditorLayer layer, int index, bool isActive) {
    return Container(
      margin: EdgeInsets.symmetric(horizontal: 8, vertical: 2),
      decoration: BoxDecoration(
        color: isActive ? Colors.blue.withOpacity(0.3) : Colors.transparent,
        borderRadius: BorderRadius.circular(4),
      ),
      child: ListTile(
        dense: true,
        leading: Icon(
          layer.visible ? Icons.visibility : Icons.visibility_off,
          color: Colors.white,
          size: 18,
        ),
        title: Text(
          layer.name,
          style: TextStyle(color: Colors.white, fontSize: 14),
        ),
        trailing: Icon(
          layer.locked ? Icons.lock : Icons.lock_open,
          color: Colors.white54,
          size: 16,
        ),
        onTap: () => setState(() => _activeLayerIndex = index),
      ),
    );
  }

  Widget _buildPropertiesPanel() {
    return Container(
      width: 250,
      decoration: BoxDecoration(
        color: Colors.grey[100],
        border: Border(left: BorderSide(color: Colors.grey[300]!)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildPropertiesHeader(),
          Expanded(
            child: SingleChildScrollView(
              child: Column(
                children: [
                  _buildBrushProperties(),
                  _buildMapProperties(),
                  _buildLocationProperties(),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildPropertiesHeader() {
    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        border: Border(bottom: BorderSide(color: Colors.grey[300]!)),
      ),
      child: Row(
        children: [
          Icon(Icons.tune, color: Colors.grey[700]),
          SizedBox(width: 8),
          Text(
            'Properties',
            style: TextStyle(
              fontSize: 16,
              fontWeight: FontWeight.bold,
              color: Colors.grey[700],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildBrushProperties() {
    return ExpansionTile(
      title: Text('Brush Settings'),
      initiallyExpanded: true,
      children: [
        Padding(
          padding: EdgeInsets.all(16),
          child: Column(
            children: [
              _buildSliderProperty(
                'Size',
                _brushSize,
                1.0,
                100.0,
                (value) => setState(() => _brushSize = value),
              ),
              _buildSliderProperty(
                'Opacity',
                _brushOpacity,
                0.1,
                1.0,
                (value) => setState(() => _brushOpacity = value),
              ),
              _buildSliderProperty(
                'Hardness',
                _brushHardness,
                0.0,
                1.0,
                (value) => setState(() => _brushHardness = value),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildMapProperties() {
    return ExpansionTile(
      title: Text('Map Info'),
      children: [
        Padding(
          padding: EdgeInsets.all(16),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              _buildInfoRow('Dimensions', '${_mapWidth}×${_mapHeight}'),
              _buildInfoRow('Scale', '${(_scale * 100).toInt()}%'),
              if (widget.mapData != null)
                _buildInfoRow('Resolution',
                    '${widget.mapData!.info.resolution.toStringAsFixed(3)} m/px'),
              _buildInfoRow('Location Points', '${_locationPoints.length}'),
              _buildInfoRow('Modified', _hasUnsavedChanges ? 'Yes' : 'No'),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildLocationProperties() {
    return ExpansionTile(
      title: Text('Location Points'),
      children: [
        Container(
          height: 200,
          child: ListView.builder(
            itemCount: _locationPoints.length,
            itemBuilder: (context, index) {
              final point = _locationPoints[index];
              return _buildLocationPointItem(point, index);
            },
          ),
        ),
      ],
    );
  }

  Widget _buildLocationPointItem(LocationPoint point, int index) {
    final isSelected = _selectedPoint == point;

    return Container(
      margin: EdgeInsets.symmetric(horizontal: 8, vertical: 2),
      decoration: BoxDecoration(
        color: isSelected ? Colors.blue.withOpacity(0.1) : Colors.transparent,
        borderRadius: BorderRadius.circular(4),
        border: isSelected ? Border.all(color: Colors.blue) : null,
      ),
      child: ListTile(
        dense: true,
        leading: Text(point.type.icon),
        title: Text(point.name, style: TextStyle(fontSize: 12)),
        subtitle: Text(
          '(${point.position.dx.toStringAsFixed(1)}, ${point.position.dy.toStringAsFixed(1)})',
          style: TextStyle(fontSize: 10),
        ),
        trailing: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            IconButton(
              onPressed: () => _editLocationPoint(point),
              icon: Icon(Icons.edit, size: 16),
            ),
            IconButton(
              onPressed: () => _deleteLocationPoint(index),
              icon: Icon(Icons.delete, size: 16, color: Colors.red),
            ),
          ],
        ),
        onTap: () => setState(() => _selectedPoint = point),
      ),
    );
  }

  Widget _buildSliderProperty(String label, double value, double min,
      double max, Function(double) onChanged) {
    return Column(
      children: [
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Text(label),
            Text(value.toStringAsFixed(1)),
          ],
        ),
        Slider(
          value: value,
          min: min,
          max: max,
          divisions: ((max - min) * 10).round(),
          onChanged: onChanged,
        ),
        SizedBox(height: 8),
      ],
    );
  }

  Widget _buildInfoRow(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          Text('$label: ', style: TextStyle(fontWeight: FontWeight.w500)),
          Expanded(
              child: Text(value, style: TextStyle(color: Colors.grey[600]))),
        ],
      ),
    );
  }

  Widget _buildStatusBar() {
    return Container(
      height: 30,
      decoration: BoxDecoration(
        color: Colors.grey[800],
        border: Border(top: BorderSide(color: Colors.grey[600]!)),
      ),
      child: Padding(
        padding: EdgeInsets.symmetric(horizontal: 16),
        child: Row(
          children: [
            Text(
              'Tool: ${_currentTool.name}',
              style: TextStyle(color: Colors.white, fontSize: 12),
            ),
            SizedBox(width: 16),
            Text(
              'Layer: ${_currentLayer.name}',
              style: TextStyle(color: Colors.white, fontSize: 12),
            ),
            SizedBox(width: 16),
            Text(
              'Points: ${_currentPointType.displayName}',
              style: TextStyle(color: Colors.white, fontSize: 12),
            ),
            Spacer(),
            if (_hasUnsavedChanges)
              Text(
                'Modified',
                style: TextStyle(color: Colors.orange, fontSize: 12),
              ),
            SizedBox(width: 16),
            Text(
              'Ready',
              style: TextStyle(color: Colors.green, fontSize: 12),
            ),
          ],
        ),
      ),
    );
  }

  // Event handlers and tool implementations
  void _selectTool(EditTool tool) {
    setState(() {
      _currentTool = tool;
    });
    _toolSwitchController.forward().then((_) {
      _toolSwitchController.reverse();
    });
  }

  IconData _getToolIcon(EditTool tool) {
    switch (tool) {
      case EditTool.select:
        return Icons.crop_free;
      case EditTool.pan:
        return Icons.pan_tool;
      case EditTool.zoom:
        return Icons.zoom_in;
      case EditTool.pencil:
        return Icons.edit;
      case EditTool.brush:
        return Icons.brush;
      case EditTool.eraser:
        return Icons.cleaning_services;
      case EditTool.bucket:
        return Icons.format_color_fill;
      case EditTool.line:
        return Icons.show_chart;
      case EditTool.rectangle:
        return Icons.crop_din;
      case EditTool.circle:
        return Icons.radio_button_unchecked;
      case EditTool.polygon:
        return Icons.polyline;
      case EditTool.text:
        return Icons.text_format;
    }
  }

  // Gesture handlers
  void _handleScaleStart(ScaleStartDetails details) {
    if (_currentTool == EditTool.pencil ||
        _currentTool == EditTool.brush ||
        _currentTool == EditTool.eraser) {
      _startDrawing(details.localFocalPoint);
    }
  }

  void _handleScaleUpdate(ScaleUpdateDetails details) {
    setState(() {
      if (details.scale != 1.0) {
        // Handle scaling
        _scale = (_scale * details.scale).clamp(_minScale, _maxScale);
      } else {
        // Handle single finger pan
        final delta = details.focalPointDelta;
        if (_isDrawing) {
          // If we're drawing, continue the drawing operation
          _continueDrawing(details.localFocalPoint);
        } else if (_currentTool == EditTool.pan) {
          // If using pan tool, update pan offset
          _panOffset += delta;
        }
      }
    });
  }

  void _handleScaleEnd(ScaleEndDetails details) {
    if (_isDrawing) {
      _finishDrawing();
    }
  }

  void _handleTapDown(TapDownDetails details) {
    final localPosition = details.localPosition;
    final mapPosition = _screenToMapCoordinates(localPosition);

    switch (_currentTool) {
      case EditTool.select:
        _handleSelection(localPosition);
        break;
      default:
        _handleLocationPointCreation(mapPosition);
        break;
    }
  }

  void _handleSelection(Offset position) {
    // Check if clicking on a location point
    for (final point in _locationPoints) {
      final pointScreenPos = _mapToScreenCoordinates(point.position);
      final distance = (pointScreenPos - position).distance;

      if (distance < 20) {
        // 20 pixel selection radius
        setState(() {
          _selectedPoint = point;
        });
        return;
      }
    }

    // Clear selection if not clicking on a point
    setState(() {
      _selectedPoint = null;
    });
  }

  void _handleLocationPointCreation(Offset mapPosition) {
    if (_currentTool == EditTool.select) {
      // Create new location point
      final point = LocationPoint(
        id: DateTime.now().millisecondsSinceEpoch.toString(),
        name: '${_currentPointType.displayName} ${_locationPoints.length + 1}',
        type: _currentPointType,
        position: mapPosition,
        properties: {},
      );

      setState(() {
        _locationPoints.add(point);
        _selectedPoint = point;
        _hasUnsavedChanges = true;
      });
    }
  }

  void _startDrawing(Offset position) {
    final mapPosition = _screenToMapCoordinates(position);

    setState(() {
      _isDrawing = true;
      _currentStroke = [
        DrawingStroke(
          points: [mapPosition],
          tool: _currentTool,
          layer: _currentLayer,
          brushSize: _brushSize,
          opacity: _brushOpacity,
        ),
      ];
    });
  }

  void _continueDrawing(Offset position) {
    if (!_isDrawing || _currentStroke.isEmpty) return;

    final mapPosition = _screenToMapCoordinates(position);

    setState(() {
      _currentStroke.last.points.add(mapPosition);
    });

    _applyStrokeToImage();
  }

  void _finishDrawing() {
    if (!_isDrawing || _currentStroke.isEmpty) return;

    setState(() {
      _isDrawing = false;
      _undoStack.addAll(_currentStroke);
      _redoStack.clear();
      _currentStroke.clear();
      _hasUnsavedChanges = true;
    });

    // Limit undo stack size
    if (_undoStack.length > 100) {
      _undoStack.removeRange(0, _undoStack.length - 100);
    }
  }

  void _applyStrokeToImage() {
    if (_currentPgmData == null || _currentStroke.isEmpty) return;

    final stroke = _currentStroke.last;

    for (final point in stroke.points) {
      _applyBrushAtPoint(point, stroke);
    }
  }

  void _applyBrushAtPoint(Offset point, DrawingStroke stroke) {
    final x = point.dx.round();
    final y = point.dy.round();
    final radius = (stroke.brushSize / 2).round();

    for (int dy = -radius; dy <= radius; dy++) {
      for (int dx = -radius; dx <= radius; dx++) {
        final px = x + dx;
        final py = y + dy;

        if (px >= 0 && px < _mapWidth && py >= 0 && py < _mapHeight) {
          final distance = math.sqrt(dx * dx + dy * dy);

          if (distance <= radius) {
            final index = py * _mapWidth + px;

            if (index < _currentPgmData!.length) {
              int newValue;

              switch (stroke.tool) {
                case EditTool.pencil:
                case EditTool.brush:
                  newValue = stroke.layer.value;
                  break;
                case EditTool.eraser:
                  newValue = MapLayer.free.value;
                  break;
                default:
                  continue;
              }

              // Apply brush hardness and opacity
              final intensity = _calculateBrushIntensity(
                  distance, radius.toDouble(), stroke.opacity, _brushHardness);
              final currentValue = _currentPgmData![index];

              _currentPgmData![index] =
                  _blendValues(currentValue, newValue, intensity);
            }
          }
        }
      }
    }

    // Update the display image
    _updateDisplayImage();
  }

  double _calculateBrushIntensity(
      double distance, double radius, double opacity, double hardness) {
    if (distance > radius) return 0.0;

    final normalizedDistance = distance / radius;
    final falloff = math.pow(1.0 - normalizedDistance, hardness * 3.0 + 0.5);

    return (falloff * opacity).clamp(0.0, 1.0);
  }

  int _blendValues(int current, int target, double intensity) {
    return (current + (target - current) * intensity).round().clamp(0, 255);
  }

  void _updateDisplayImage() {
    if (_currentPgmData == null) return;

    // Convert PGM data to RGBA and update the display image
    final rgbaData = Uint8List(_mapWidth * _mapHeight * 4);

    for (int i = 0; i < _currentPgmData!.length; i++) {
      final grayValue = _currentPgmData![i];
      final rgbaIndex = i * 4;
      rgbaData[rgbaIndex] = grayValue;
      rgbaData[rgbaIndex + 1] = grayValue;
      rgbaData[rgbaIndex + 2] = grayValue;
      rgbaData[rgbaIndex + 3] = 255;
    }

    ui.decodeImageFromPixels(
      rgbaData,
      _mapWidth,
      _mapHeight,
      ui.PixelFormat.rgba8888,
      (ui.Image image) {
        setState(() {
          _mapImage?.dispose();
          _mapImage = image;
        });
      },
    );
  }

  Offset _screenToMapCoordinates(Offset screenPoint) {
    return (screenPoint - _panOffset) / _scale;
  }

  Offset _mapToScreenCoordinates(Offset mapPoint) {
    return mapPoint * _scale + _panOffset;
  }

  // Tool operations
  void _undo() {
    if (_undoStack.isNotEmpty) {
      final stroke = _undoStack.removeLast();
      _redoStack.add(stroke);
      _replayAllStrokes();
    }
  }

  void _redo() {
    if (_redoStack.isNotEmpty) {
      final stroke = _redoStack.removeLast();
      _undoStack.add(stroke);
      _replayAllStrokes();
    }
  }

  void _replayAllStrokes() {
    // Reset to original data
    if (_originalPgmData != null) {
      _currentPgmData = Uint8List.fromList(_originalPgmData!);

      // Replay all strokes in undo stack
      for (final stroke in _undoStack) {
        for (final point in stroke.points) {
          _applyBrushAtPoint(point, stroke);
        }
      }

      _updateDisplayImage();
      setState(() {
        _hasUnsavedChanges = true;
      });
    }
  }

  void _cut() {
    if (_hasSelection) {
      _copy();
      // Clear selection area
      _hasUnsavedChanges = true;
    }
  }

  void _copy() {
    if (_hasSelection && _selectionRect != null) {
      // Copy selected area to clipboard
      _clipboardData = _extractSelectionData();
    }
  }

  void _paste() {
    if (_clipboardData != null) {
      // Paste clipboard data at current position
      _hasUnsavedChanges = true;
    }
  }

  Uint8List? _extractSelectionData() {
    // Extract image data from selection rectangle
    // Implementation depends on selection rectangle
    return null;
  }

  void _zoomIn() {
    setState(() {
      _scale = (_scale * 1.2).clamp(_minScale, _maxScale);
    });
  }

  void _zoomOut() {
    setState(() {
      _scale = (_scale / 1.2).clamp(_minScale, _maxScale);
    });
  }

  void _fitToScreen() {
    // Calculate scale to fit map in viewport
    // Implementation depends on viewport size
  }

  // File operations
  void _openMap() async {
    // Show file picker dialog
  }

  Future<void> _saveMap() async {
    if (_currentPgmData == null) return;

    try {
      // Convert current PGM data back to map format
      final occupancyData = List<int>.filled(_mapWidth * _mapHeight, -1);

      for (int i = 0;
          i < _currentPgmData!.length && i < occupancyData.length;
          i++) {
        final pgmValue = _currentPgmData![i];

        if (pgmValue == MapLayer.obstacles.value) {
          occupancyData[i] = 100;
        } else if (pgmValue == MapLayer.free.value) {
          occupancyData[i] = 0;
        } else {
          occupancyData[i] = -1;
        }
      }

      // Create updated map data with location points as shapes
      final shapes = _locationPoints.map((point) {
        return MapShape(
          id: point.id,
          type: point.type.name,
          name: point.name,
          points: [
            odom.Position(x: point.position.dx, y: point.position.dy, z: 0)
          ],
          sides: {'left': '', 'right': '', 'front': '', 'back': ''},
          color: 'FF${point.type.color.value.toRadixString(16).substring(2)}',
          createdAt: DateTime.now(),
        );
      }).toList();

      final updatedMapData = MapData(
        deviceId: widget.mapData?.deviceId ?? 'unknown',
        timestamp: DateTime.now(),
        info: widget.mapData?.info ??
            MapInfo(
              resolution: 0.05,
              width: _mapWidth,
              height: _mapHeight,
              origin: odom.Position(x: 0, y: 0, z: 0),
              originOrientation: odom.Orientation(x: 0, y: 0, z: 0, w: 1),
            ),
        occupancyData: occupancyData,
        shapes: shapes,
        version: (widget.mapData?.version ?? 0) + 1,
      );

      widget.onMapChanged(updatedMapData);

      setState(() {
        _hasUnsavedChanges = false;
      });

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Map saved successfully!')),
      );
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Error saving map: $e')),
      );
    }
  }

  void _exportMap() async {
    // Export to various formats (PGM, PNG, etc.)
    try {
      if (widget.deviceId != null) {
        final response = await _apiService.exportMapToPGM(
          deviceId: widget.deviceId!,
          mapName: 'edited_map_${DateTime.now().millisecondsSinceEpoch}',
        );

        if (response['success'] == true) {
          ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(content: Text('Map exported successfully!')),
          );
        }
      }
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Export failed: $e')),
      );
    }
  }

  void _deployToRaspberryPi() async {
    // Deploy to Raspberry Pi
    try {
      await _saveMap(); // Save first

      if (widget.deviceId != null) {
        final response = await _apiService.deployMapToRaspberryPi(
          deviceId: widget.deviceId!,
          mapName: 'edited_map_${DateTime.now().millisecondsSinceEpoch}',
          autoLoad: true,
        );

        if (response['success'] == true) {
          ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(
              content: Text('Map deployed to Raspberry Pi successfully!'),
              backgroundColor: Colors.green,
            ),
          );
        }
      }
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Deployment failed: $e')),
      );
    }
  }

  // Location point operations
  void _editLocationPoint(LocationPoint point) {
    // Show edit dialog
    showDialog(
      context: context,
      builder: (context) => _LocationPointEditDialog(
        point: point,
        onSave: (updatedPoint) {
          final index = _locationPoints.indexOf(point);
          if (index != -1) {
            setState(() {
              _locationPoints[index] = updatedPoint;
              _hasUnsavedChanges = true;
            });
          }
        },
      ),
    );
  }

  void _deleteLocationPoint(int index) {
    setState(() {
      _locationPoints.removeAt(index);
      _selectedPoint = null;
      _hasUnsavedChanges = true;
    });
  }
}

// Supporting classes
class DrawingStroke {
  List<Offset> points;
  EditTool tool;
  MapLayer layer;
  double brushSize;
  double opacity;

  DrawingStroke({
    required this.points,
    required this.tool,
    required this.layer,
    required this.brushSize,
    required this.opacity,
  });
}

class LocationPoint {
  String id;
  String name;
  LocationPointType type;
  Offset position;
  Map<String, dynamic> properties;

  LocationPoint({
    required this.id,
    required this.name,
    required this.type,
    required this.position,
    required this.properties,
  });
}

enum LayerType {
  background,
  obstacles,
  locations,
  annotations,
}

class EditorLayer {
  String id;
  String name;
  LayerType type;
  double opacity;
  bool visible;
  bool locked;

  EditorLayer({
    required this.id,
    required this.name,
    required this.type,
    required this.opacity,
    required this.visible,
    required this.locked,
  });
}

// Enhanced map painter
class EnhancedMapPainter extends CustomPainter {
  final ui.Image mapImage;
  final ui.Image? overlayImage;
  final double scale;
  final Offset panOffset;
  final List<DrawingStroke> currentStroke;
  final List<LocationPoint> locationPoints;
  final LocationPoint? selectedPoint;
  final Rect? selectionRect;
  final EditTool currentTool;
  final MapLayer currentLayer;
  final LocationPointType currentPointType;
  final double brushSize;
  final List<EditorLayer> layers;
  final int activeLayerIndex;

  EnhancedMapPainter({
    required this.mapImage,
    this.overlayImage,
    required this.scale,
    required this.panOffset,
    required this.currentStroke,
    required this.locationPoints,
    this.selectedPoint,
    this.selectionRect,
    required this.currentTool,
    required this.currentLayer,
    required this.currentPointType,
    required this.brushSize,
    required this.layers,
    required this.activeLayerIndex,
  });

  @override
  void paint(Canvas canvas, Size size) {
    // Apply transformations
    canvas.save();
    canvas.translate(size.width / 2, size.height / 2);
    canvas.translate(panOffset.dx, panOffset.dy);
    canvas.scale(scale);
    canvas.translate(-mapImage.width / 2, -mapImage.height / 2);

    // Draw map layers
    _drawMapImage(canvas);
    _drawLocationPoints(canvas);
    _drawCurrentStroke(canvas);
    _drawSelection(canvas);

    canvas.restore();

    // Draw UI overlays (not transformed)
    _drawUIOverlays(canvas, size);
  }

  void _drawMapImage(Canvas canvas) {
    canvas.drawImage(mapImage, Offset.zero, Paint());

    if (overlayImage != null) {
      canvas.drawImage(overlayImage!, Offset.zero, Paint());
    }
  }

  void _drawLocationPoints(Canvas canvas) {
    for (final point in locationPoints) {
      _drawLocationPoint(canvas, point, point == selectedPoint);
    }
  }

  void _drawLocationPoint(Canvas canvas, LocationPoint point, bool isSelected) {
    final paint = Paint()
      ..color = point.type.color
      ..style = PaintingStyle.fill;

    final outlinePaint = Paint()
      ..color = isSelected ? Colors.yellow : Colors.white
      ..style = PaintingStyle.stroke
      ..strokeWidth = isSelected ? 3.0 : 2.0;

    final radius = isSelected ? 12.0 : 10.0;

    // Draw point circle
    canvas.drawCircle(point.position, radius, paint);
    canvas.drawCircle(point.position, radius, outlinePaint);

    // Draw icon
    final textPainter = TextPainter(
      text: TextSpan(
        text: point.type.icon,
        style: TextStyle(
          fontSize: 16,
          color: Colors.white,
        ),
      ),
      textDirection: TextDirection.ltr,
    );

    textPainter.layout();
    textPainter.paint(
      canvas,
      point.position - Offset(textPainter.width / 2, textPainter.height / 2),
    );

    // Draw name label
    if (isSelected) {
      final namePainter = TextPainter(
        text: TextSpan(
          text: point.name,
          style: TextStyle(
            fontSize: 12,
            color: Colors.black,
            backgroundColor: Colors.white.withOpacity(0.8),
          ),
        ),
        textDirection: TextDirection.ltr,
      );

      namePainter.layout();
      namePainter.paint(
        canvas,
        point.position + Offset(-namePainter.width / 2, radius + 5),
      );
    }
  }

  void _drawCurrentStroke(Canvas canvas) {
    if (currentStroke.isEmpty) return;

    for (final stroke in currentStroke) {
      if (stroke.points.length < 2) continue;

      final paint = Paint()
        ..color = stroke.layer.color.withOpacity(stroke.opacity)
        ..style = PaintingStyle.stroke
        ..strokeWidth = stroke.brushSize
        ..strokeCap = StrokeCap.round
        ..strokeJoin = StrokeJoin.round;

      final path = Path();
      path.moveTo(stroke.points.first.dx, stroke.points.first.dy);

      for (int i = 1; i < stroke.points.length; i++) {
        path.lineTo(stroke.points[i].dx, stroke.points[i].dy);
      }

      canvas.drawPath(path, paint);
    }
  }

  void _drawSelection(Canvas canvas) {
    if (selectionRect != null) {
      final paint = Paint()
        ..color = Colors.blue.withOpacity(0.3)
        ..style = PaintingStyle.fill;

      final borderPaint = Paint()
        ..color = Colors.blue
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.0;

      canvas.drawRect(selectionRect!, paint);
      _drawDashedRect(canvas, selectionRect!, borderPaint);
    }
  }

  void _drawUIOverlays(Canvas canvas, Size size) {
    // Draw tool cursor preview
    // Draw grid
    // Draw rulers
    // etc.
  }

  // Dashed line effect for selection rectangle is not supported in Flutter's Canvas API.
  // Draw a dashed rectangle for selection
  void _drawDashedRect(Canvas canvas, Rect rect, Paint paint) {
    const dashWidth = 6.0;
    const dashSpace = 4.0;

    // Top
    double startX = rect.left;
    while (startX < rect.right) {
      final endX = (startX + dashWidth).clamp(rect.left, rect.right);
      canvas.drawLine(
        Offset(startX, rect.top),
        Offset(endX, rect.top),
        paint,
      );
      startX = endX + dashSpace;
    }

    // Bottom
    startX = rect.left;
    while (startX < rect.right) {
      final endX = (startX + dashWidth).clamp(rect.left, rect.right);
      canvas.drawLine(
        Offset(startX, rect.bottom),
        Offset(endX, rect.bottom),
        paint,
      );
      startX = endX + dashSpace;
    }

    // Left
    double startY = rect.top;
    while (startY < rect.bottom) {
      final endY = (startY + dashWidth).clamp(rect.top, rect.bottom);
      canvas.drawLine(
        Offset(rect.left, startY),
        Offset(rect.left, endY),
        paint,
      );
      startY = endY + dashSpace;
    }

    // Right
    startY = rect.top;
    while (startY < rect.bottom) {
      final endY = (startY + dashWidth).clamp(rect.top, rect.bottom);
      canvas.drawLine(
        Offset(rect.right, startY),
        Offset(rect.right, endY),
        paint,
      );
      startY = endY + dashSpace;
    }
  }

  @override
  bool shouldRepaint(EnhancedMapPainter oldDelegate) {
    return oldDelegate.scale != scale ||
        oldDelegate.panOffset != panOffset ||
        oldDelegate.currentStroke != currentStroke ||
        oldDelegate.locationPoints != locationPoints ||
        oldDelegate.selectedPoint != selectedPoint ||
        oldDelegate.selectionRect != selectionRect;
  }
}

// Location point edit dialog
class _LocationPointEditDialog extends StatefulWidget {
  final LocationPoint point;
  final Function(LocationPoint) onSave;

  const _LocationPointEditDialog({
    required this.point,
    required this.onSave,
  });

  @override
  _LocationPointEditDialogState createState() =>
      _LocationPointEditDialogState();
}

class _LocationPointEditDialogState extends State<_LocationPointEditDialog> {
  late TextEditingController _nameController;
  late LocationPointType _selectedType;

  @override
  void initState() {
    super.initState();
    _nameController = TextEditingController(text: widget.point.name);
    _selectedType = widget.point.type;
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: Text('Edit Location Point'),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          TextField(
            controller: _nameController,
            decoration: InputDecoration(
              labelText: 'Name',
              border: OutlineInputBorder(),
            ),
          ),
          SizedBox(height: 16),
          DropdownButtonFormField<LocationPointType>(
            value: _selectedType,
            decoration: InputDecoration(
              labelText: 'Type',
              border: OutlineInputBorder(),
            ),
            items: LocationPointType.values.map((type) {
              return DropdownMenuItem(
                value: type,
                child: Row(
                  children: [
                    Text(type.icon),
                    SizedBox(width: 8),
                    Text(type.displayName),
                  ],
                ),
              );
            }).toList(),
            onChanged: (value) {
              setState(() {
                _selectedType = value!;
              });
            },
          ),
        ],
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.of(context).pop(),
          child: Text('Cancel'),
        ),
        ElevatedButton(
          onPressed: () {
            final updatedPoint = LocationPoint(
              id: widget.point.id,
              name: _nameController.text,
              type: _selectedType,
              position: widget.point.position,
              properties: widget.point.properties,
            );
            widget.onSave(updatedPoint);
            Navigator.of(context).pop();
          },
          child: Text('Save'),
        ),
      ],
    );
  }
}
