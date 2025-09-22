// widgets/enhanced_pgm_map_editor.dart - Simplified PGM Map Editor (Fixed Rendering Issues)
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'dart:math' as math;
import 'dart:ui' as ui;
import 'dart:async';
import '../models/map_data.dart';
import '../models/odom.dart' as odom;
import '../services/api_service.dart';

// Simplified editing tools
enum EditTool {
  pencil,
  brush,
  eraser,
  select,
  pan,
  zoom,
}

// Map layers
enum MapLayer {
  obstacles(0, 'Obstacles', Colors.black),
  free(254, 'Free Space', Colors.white),
  unknown(128, 'Unknown', Colors.grey),
  restricted(64, 'Restricted', Colors.red);

  const MapLayer(this.value, this.name, this.color);
  final int value;
  final String name;
  final Color color;
}

// Location point types
enum LocationPointType {
  pickup('Pickup Point', '', Colors.green),
  drop('Drop Point', '', Colors.blue),
  home('Home Position', '', Colors.orange),
  charging('Charging Station', '', Colors.yellow),
  waypoint('Waypoint', '', Colors.purple),
  obstacle('Obstacle', '', Colors.red);

  const LocationPointType(this.displayName, this.icon, this.color);
  final String displayName;
  final String icon;
  final Color color;
}

// Simplified PGM Map Editor
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

class _EnhancedPGMMapEditorState extends State<EnhancedPGMMapEditor> {
  final ApiService _apiService = ApiService();

  // Image data
  Uint8List? _originalPgmData;
  Uint8List? _currentPgmData;
  ui.Image? _mapImage;
  int _mapWidth = 0;
  int _mapHeight = 0;

  // Tool state
  EditTool _currentTool = EditTool.pencil;
  MapLayer _currentLayer = MapLayer.obstacles;
  LocationPointType _currentPointType = LocationPointType.pickup;

  // Brush settings
  double _brushSize = 5.0;
  double _brushOpacity = 1.0;
  Color _brushColor = Colors.black;

  // Layer visibility
  Map<MapLayer, bool> _layerVisibility = {
    MapLayer.obstacles: true,
    MapLayer.free: true,
    MapLayer.unknown: true,
  };

  // Drawing state
  List<DrawingStroke> _undoStack = [];
  List<DrawingStroke> _redoStack = [];
  bool _isDrawing = false;
  List<Offset> _currentStrokePoints = [];

  // Location points
  List<LocationPoint> _locationPoints = [];
  LocationPoint? _selectedPoint;

  // View transformation
  double _scale = 1.0;
  Offset _panOffset = Offset.zero;
  final double _minScale = 0.1;
  final double _maxScale = 20.0;

  // State
  bool _isLoading = false;
  String? _error;
  bool _hasUnsavedChanges = false;

  @override
  void initState() {
    super.initState();

    if (widget.mapData != null) {
      _loadMapData();
    } else {
      _createNewMap();
    }
  }

  @override
  void didUpdateWidget(EnhancedPGMMapEditor oldWidget) {
    super.didUpdateWidget(oldWidget);

    // If the map data changed, reload and re-center
    if (widget.mapData != oldWidget.mapData) {
      if (widget.mapData != null) {
        _loadMapData();
      } else {
        _createNewMap();
      }
    }
  }

  @override
  void dispose() {
    _mapImage?.dispose();
    super.dispose();
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
      const width = 500;
      const height = 500;
      final data = Uint8List(width * height);

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

      final rgbaData = Uint8List(width * height * 4);
      for (int i = 0; i < pgmData.length; i++) {
        final grayValue = pgmData[i];
        final rgbaIndex = i * 4;
        rgbaData[rgbaIndex] = grayValue;
        rgbaData[rgbaIndex + 1] = grayValue;
        rgbaData[rgbaIndex + 2] = grayValue;
        rgbaData[rgbaIndex + 3] = 255;
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
      if (mounted) {
        setState(() {
          _mapImage?.dispose();
          _mapImage = image;
          _error = null;
        });

        // Auto-center and fit the map when it loads
        WidgetsBinding.instance.addPostFrameCallback((_) {
          // Add a small delay to ensure the widget is fully rendered
          Future.delayed(const Duration(milliseconds: 100), () {
            if (mounted) {
              _fitToScreen();
            }
          });
        });
      }
    } catch (e) {
      throw Exception('Failed to load PGM image: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    final screenSize = MediaQuery.of(context).size;
    final isSmallScreen = screenSize.width < 768; // Mobile and small tablets

    // Schedule centering if map is loaded but not centered yet
    if (_mapImage != null && _panOffset == Offset.zero) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        Future.delayed(const Duration(milliseconds: 200), () {
          if (mounted) {
            _fitToScreen();
          }
        });
      });
    }

    return Scaffold(
      backgroundColor: Colors.grey[100],
      body: isSmallScreen ? _buildMobileLayout() : _buildDesktopLayout(),
    );
  }

  Widget _buildMobileLayout() {
    return Column(
      children: [
        _buildMobileToolbar(),
        Expanded(child: _buildCanvasArea()),
        _buildMobileBottomSheet(),
      ],
    );
  }

  Widget _buildDesktopLayout() {
    return Column(
      children: [
        _buildToolbar(),
        Expanded(
          child: Row(
            children: [
              _buildToolPanel(),
              Expanded(child: _buildCanvasArea()),
              _buildPropertiesPanel(),
            ],
          ),
        ),
        _buildStatusBar(),
      ],
    );
  }

  Widget _buildMobileToolbar() {
    return Container(
      height: 56,
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.blue[800]!, Colors.blue[600]!],
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.2),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        child: Row(
          children: [
            const Icon(Icons.edit, color: Colors.white, size: 20),
            const SizedBox(width: 8),
            const Text(
              'PGM Editor',
              style: TextStyle(
                color: Colors.white,
                fontSize: 16,
                fontWeight: FontWeight.bold,
              ),
            ),
            const Spacer(),
            // Quick tool selector for mobile
            _buildMobileQuickTools(),
          ],
        ),
      ),
    );
  }

  Widget _buildMobileQuickTools() {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        // Current tool indicator
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
          decoration: BoxDecoration(
            color: Colors.white.withOpacity(0.2),
            borderRadius: BorderRadius.circular(12),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(
                _getToolIcon(_currentTool),
                color: Colors.white,
                size: 16,
              ),
              const SizedBox(width: 4),
              Text(
                _getToolName(_currentTool),
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 12,
                ),
              ),
            ],
          ),
        ),
        const SizedBox(width: 8),
        // Deploy to Pi button
        ElevatedButton.icon(
          onPressed: _deployToRaspberryPi,
          icon: const Icon(Icons.upload, size: 16),
          label: const Text(
            'Deploy to Pi',
            style: TextStyle(fontSize: 12),
          ),
          style: ElevatedButton.styleFrom(
            backgroundColor: Colors.green,
            foregroundColor: Colors.white,
            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
            minimumSize: const Size(0, 32),
          ),
        ),
        const SizedBox(width: 8),
        // Tools menu button
        IconButton(
          onPressed: _showMobileToolsDialog,
          icon: const Icon(Icons.build, color: Colors.white, size: 20),
          tooltip: 'Tools & Settings',
        ),
      ],
    );
  }

  Widget _buildMobileBottomSheet() {
    return Container(
      height: 80,
      decoration: BoxDecoration(
        color: Colors.white,
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 4,
            offset: const Offset(0, -2),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Row(
          children: [
            // Tool shortcuts
            Expanded(
              child: SingleChildScrollView(
                scrollDirection: Axis.horizontal,
                child: Row(
                  children: [
                    _buildMobileToolButton(EditTool.pencil),
                    _buildMobileToolButton(EditTool.brush),
                    _buildMobileToolButton(EditTool.eraser),
                    _buildMobileToolButton(EditTool.pan),
                    _buildMobileToolButton(EditTool.zoom),
                  ],
                ),
              ),
            ),
            // Divider
            Container(
              width: 1,
              height: 40,
              color: Colors.grey[300],
              margin: const EdgeInsets.symmetric(horizontal: 8),
            ),
            // Action buttons
            Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Row(
                  children: [
                    IconButton(
                      onPressed: _undo,
                      icon: const Icon(Icons.undo),
                      iconSize: 20,
                      tooltip: 'Undo',
                    ),
                    IconButton(
                      onPressed: _redo,
                      icon: const Icon(Icons.redo),
                      iconSize: 20,
                      tooltip: 'Redo',
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildMobileToolButton(EditTool tool) {
    final isSelected = _currentTool == tool;
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 2),
      child: InkWell(
        onTap: () => setState(() => _currentTool = tool),
        borderRadius: BorderRadius.circular(8),
        child: Container(
          width: 48,
          height: 48,
          decoration: BoxDecoration(
            color: isSelected ? Colors.blue[600] : Colors.grey[200],
            borderRadius: BorderRadius.circular(8),
            border: isSelected
                ? Border.all(color: Colors.blue[800]!, width: 2)
                : null,
          ),
          child: Icon(
            _getToolIcon(tool),
            color: isSelected ? Colors.white : Colors.grey[700],
            size: 20,
          ),
        ),
      ),
    );
  }

  void _showMobileToolsDialog() {
    showModalBottomSheet(
      context: context,
      isScrollControlled: true,
      backgroundColor: Colors.transparent,
      builder: (context) => Container(
        height: MediaQuery.of(context).size.height * 0.7,
        decoration: const BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.only(
            topLeft: Radius.circular(20),
            topRight: Radius.circular(20),
          ),
        ),
        child: Column(
          children: [
            // Handle bar
            Container(
              width: 40,
              height: 4,
              margin: const EdgeInsets.symmetric(vertical: 8),
              decoration: BoxDecoration(
                color: Colors.grey[300],
                borderRadius: BorderRadius.circular(2),
              ),
            ),
            // Title
            Padding(
              padding: const EdgeInsets.all(16),
              child: Text(
                'Tools & Settings',
                style: TextStyle(
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                  color: Colors.grey[800],
                ),
              ),
            ),
            // Content
            Expanded(
              child: SingleChildScrollView(
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    _buildMobileToolsSection(),
                    const SizedBox(height: 24),
                    _buildMobileLayersSection(),
                    const SizedBox(height: 24),
                    _buildMobileBrushSettings(),
                    const SizedBox(height: 24),
                    _buildMobileLocationPointsSection(),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildToolbar() {
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
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Row(
          children: [
            const Icon(Icons.edit, color: Colors.white, size: 24),
            const SizedBox(width: 8),
            const Text(
              'PGM Map Editor',
              style: TextStyle(
                color: Colors.white,
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(width: 24),

            // File operations
            _buildToolbarButton(Icons.folder_open, 'Open', _openMap),
            _buildToolbarButton(Icons.save, 'Save', _saveMap),
            _buildToolbarButton(Icons.save_as, 'Export', _exportMap),

            const SizedBox(width: 16),

            // Edit operations
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

            const SizedBox(width: 16),

            // View operations
            _buildToolbarButton(Icons.zoom_in, 'Zoom In', _zoomIn),
            _buildToolbarButton(Icons.zoom_out, 'Zoom Out', _zoomOut),
            _buildToolbarButton(Icons.center_focus_strong, 'Fit', _fitToScreen),
            _buildToolbarButton(Icons.my_location, 'Center', _centerMap),

            const Spacer(),

            // Scale display
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              decoration: BoxDecoration(
                color: Colors.white.withOpacity(0.2),
                borderRadius: BorderRadius.circular(4),
              ),
              child: Text(
                '${(_scale * 100).toInt()}%',
                style: const TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  fontSize: 14,
                ),
              ),
            ),

            const SizedBox(width: 16),

            // Deploy button
            ElevatedButton.icon(
              onPressed: _deployToRaspberryPi,
              icon: const Icon(Icons.upload, size: 20),
              label: const Text('Deploy to Pi'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.green,
                foregroundColor: Colors.white,
                padding:
                    const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildToolbarButton(
      IconData icon, String tooltip, VoidCallback? onPressed) {
    return Tooltip(
      message: tooltip,
      child: IconButton(
        onPressed: onPressed,
        icon: Icon(
          icon,
          color: onPressed != null ? Colors.white : Colors.white54,
        ),
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
      ),
      child: SingleChildScrollView(
        child: Column(
          children: [
            // Tools
            Container(
              padding: const EdgeInsets.all(8),
              child: Column(
                children: [
                  const Text(
                    'Tools',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 12,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 8),
                  GridView.count(
                    shrinkWrap: true,
                    physics: const NeverScrollableScrollPhysics(),
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

            // Layers
            Container(
              padding: const EdgeInsets.all(8),
              child: Column(
                children: [
                  const Text(
                    'Layer',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 12,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 8),
                  ...MapLayer.values.map((layer) => _buildLayerButton(layer)),
                ],
              ),
            ),

            Divider(color: Colors.grey[600]),

            // Point types
            Container(
              padding: const EdgeInsets.all(8),
              child: Column(
                children: [
                  const Text(
                    'Points',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 12,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 8),
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

    return Container(
      margin: const EdgeInsets.all(2),
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
          child: Icon(icon, color: Colors.white, size: 20),
        ),
      ),
    );
  }

  Widget _buildLayerButton(MapLayer layer) {
    final isSelected = _currentLayer == layer;

    return Container(
      margin: const EdgeInsets.only(bottom: 4),
      child: InkWell(
        onTap: () => setState(() => _currentLayer = layer),
        borderRadius: BorderRadius.circular(4),
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 6),
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
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  layer.name,
                  style: const TextStyle(color: Colors.white, fontSize: 10),
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
      margin: const EdgeInsets.only(bottom: 4),
      child: InkWell(
        onTap: () => setState(() => _currentPointType = type),
        borderRadius: BorderRadius.circular(4),
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 4),
          decoration: BoxDecoration(
            color:
                isSelected ? type.color.withOpacity(0.3) : Colors.transparent,
            borderRadius: BorderRadius.circular(4),
            border: isSelected ? Border.all(color: type.color) : null,
          ),
          child: Row(
            children: [
              Text(type.icon, style: const TextStyle(fontSize: 12)),
              const SizedBox(width: 4),
              Expanded(
                child: Text(
                  type.displayName,
                  style: const TextStyle(color: Colors.white, fontSize: 9),
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
      return const Center(
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
            const SizedBox(height: 16),
            Text(
              'Error: $_error',
              textAlign: TextAlign.center,
              style: TextStyle(color: Colors.red[700]),
            ),
            const SizedBox(height: 16),
            ElevatedButton(
              onPressed: _createNewMap,
              child: const Text('Create New Map'),
            ),
          ],
        ),
      );
    }

    if (_mapImage == null) {
      return const Center(child: Text('No map data available'));
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
            painter: SimplifiedMapPainter(
              mapImage: _mapImage!,
              scale: _scale,
              panOffset: _panOffset,
              locationPoints: _locationPoints,
              selectedPoint: _selectedPoint,
              currentTool: _currentTool,
              currentLayer: _currentLayer,
              brushSize: _brushSize,
              currentStrokePoints: _currentStrokePoints,
            ),
          ),
        ),
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
          Container(
            padding: const EdgeInsets.all(16),
            decoration: BoxDecoration(
              border: Border(bottom: BorderSide(color: Colors.grey[300]!)),
            ),
            child: const Row(
              children: [
                Icon(Icons.tune, color: Colors.grey),
                SizedBox(width: 8),
                Text(
                  'Properties',
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
          ),
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

  Widget _buildBrushProperties() {
    return ExpansionTile(
      title: const Text('Brush Settings'),
      initiallyExpanded: true,
      children: [
        Padding(
          padding: const EdgeInsets.all(16),
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
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildMapProperties() {
    return ExpansionTile(
      title: const Text('Map Info'),
      children: [
        Padding(
          padding: const EdgeInsets.all(16),
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
      title: const Text('Location Points'),
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
      margin: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
      decoration: BoxDecoration(
        color: isSelected ? Colors.blue.withOpacity(0.1) : Colors.transparent,
        borderRadius: BorderRadius.circular(4),
        border: isSelected ? Border.all(color: Colors.blue) : null,
      ),
      child: ListTile(
        dense: true,
        leading: Text(point.type.icon),
        title: Text(point.name, style: const TextStyle(fontSize: 12)),
        subtitle: Text(
          '(${point.position.dx.toStringAsFixed(1)}, ${point.position.dy.toStringAsFixed(1)})',
          style: const TextStyle(fontSize: 10),
        ),
        trailing: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            IconButton(
              onPressed: () => _editLocationPoint(point),
              icon: const Icon(Icons.edit, size: 16),
            ),
            IconButton(
              onPressed: () => _deleteLocationPoint(index),
              icon: const Icon(Icons.delete, size: 16, color: Colors.red),
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
        const SizedBox(height: 8),
      ],
    );
  }

  Widget _buildInfoRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          Text('$label: ', style: const TextStyle(fontWeight: FontWeight.w500)),
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
        padding: const EdgeInsets.symmetric(horizontal: 16),
        child: Row(
          children: [
            Text(
              'Tool: ${_currentTool.name}',
              style: const TextStyle(color: Colors.white, fontSize: 12),
            ),
            const SizedBox(width: 16),
            Text(
              'Layer: ${_currentLayer.name}',
              style: const TextStyle(color: Colors.white, fontSize: 12),
            ),
            const SizedBox(width: 16),
            Text(
              'Points: ${_currentPointType.displayName}',
              style: const TextStyle(color: Colors.white, fontSize: 12),
            ),
            const Spacer(),
            if (_hasUnsavedChanges)
              const Text(
                'Modified',
                style: TextStyle(color: Colors.orange, fontSize: 12),
              ),
            const SizedBox(width: 16),
            const Text(
              'Ready',
              style: TextStyle(color: Colors.green, fontSize: 12),
            ),
          ],
        ),
      ),
    );
  }

  // Event handlers
  void _selectTool(EditTool tool) {
    setState(() {
      _currentTool = tool;
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
    }
  }

  String _getToolName(EditTool tool) {
    switch (tool) {
      case EditTool.select:
        return 'Select';
      case EditTool.pan:
        return 'Pan';
      case EditTool.zoom:
        return 'Zoom';
      case EditTool.pencil:
        return 'Pencil';
      case EditTool.brush:
        return 'Brush';
      case EditTool.eraser:
        return 'Eraser';
    }
  }

  Widget _buildMobileToolsSection() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          'Drawing Tools',
          style: TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
            color: Colors.grey[800],
          ),
        ),
        const SizedBox(height: 12),
        Wrap(
          spacing: 12,
          runSpacing: 12,
          children: EditTool.values.map((tool) {
            final isSelected = _currentTool == tool;
            return InkWell(
              onTap: () {
                setState(() => _currentTool = tool);
                Navigator.pop(context);
              },
              borderRadius: BorderRadius.circular(12),
              child: Container(
                width: 80,
                height: 80,
                decoration: BoxDecoration(
                  color: isSelected ? Colors.blue[50] : Colors.grey[100],
                  borderRadius: BorderRadius.circular(12),
                  border: Border.all(
                    color: isSelected ? Colors.blue[600]! : Colors.grey[300]!,
                    width: isSelected ? 2 : 1,
                  ),
                ),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Icon(
                      _getToolIcon(tool),
                      color: isSelected ? Colors.blue[600] : Colors.grey[600],
                      size: 24,
                    ),
                    const SizedBox(height: 4),
                    Text(
                      _getToolName(tool),
                      style: TextStyle(
                        fontSize: 10,
                        color: isSelected ? Colors.blue[600] : Colors.grey[600],
                        fontWeight:
                            isSelected ? FontWeight.bold : FontWeight.normal,
                      ),
                    ),
                  ],
                ),
              ),
            );
          }).toList(),
        ),
      ],
    );
  }

  Widget _buildMobileLayersSection() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          'Map Layers',
          style: TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
            color: Colors.grey[800],
          ),
        ),
        const SizedBox(height: 12),
        ...MapLayer.values.map((layer) {
          final isVisible = _layerVisibility[layer] ?? true;
          return Container(
            margin: const EdgeInsets.only(bottom: 8),
            child: InkWell(
              onTap: () {
                setState(() {
                  _layerVisibility[layer] = !isVisible;
                });
              },
              borderRadius: BorderRadius.circular(8),
              child: Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: Colors.grey[50],
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: Colors.grey[300]!),
                ),
                child: Row(
                  children: [
                    Icon(
                      isVisible ? Icons.visibility : Icons.visibility_off,
                      color: isVisible ? Colors.green : Colors.grey,
                      size: 20,
                    ),
                    const SizedBox(width: 12),
                    Expanded(
                      child: Text(
                        layer.toString().split('.').last.toUpperCase(),
                        style: TextStyle(
                          color: Colors.grey[700],
                          fontWeight: FontWeight.w500,
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          );
        }).toList(),
      ],
    );
  }

  Widget _buildMobileBrushSettings() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          'Brush Settings',
          style: TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
            color: Colors.grey[800],
          ),
        ),
        const SizedBox(height: 12),
        Container(
          padding: const EdgeInsets.all(16),
          decoration: BoxDecoration(
            color: Colors.grey[50],
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: Colors.grey[300]!),
          ),
          child: Column(
            children: [
              Row(
                children: [
                  const Text('Size:',
                      style: TextStyle(fontWeight: FontWeight.w500)),
                  const SizedBox(width: 12),
                  Expanded(
                    child: Slider(
                      value: _brushSize,
                      min: 1,
                      max: 20,
                      divisions: 19,
                      label: _brushSize.round().toString(),
                      onChanged: (value) {
                        setState(() => _brushSize = value);
                      },
                    ),
                  ),
                  Text(
                    _brushSize.round().toString(),
                    style: const TextStyle(fontWeight: FontWeight.bold),
                  ),
                ],
              ),
              const SizedBox(height: 16),
              Row(
                children: [
                  const Text('Color:',
                      style: TextStyle(fontWeight: FontWeight.w500)),
                  const SizedBox(width: 12),
                  Expanded(
                    child: Wrap(
                      spacing: 8,
                      children: [
                        Colors.black,
                        Colors.white,
                        Colors.grey,
                        Colors.red,
                        Colors.blue,
                        Colors.green,
                      ].map((color) {
                        final isSelected = _brushColor == color;
                        return InkWell(
                          onTap: () => setState(() => _brushColor = color),
                          child: Container(
                            width: 32,
                            height: 32,
                            decoration: BoxDecoration(
                              color: color,
                              shape: BoxShape.circle,
                              border: Border.all(
                                color: isSelected
                                    ? Colors.blue[600]!
                                    : Colors.grey[400]!,
                                width: isSelected ? 3 : 1,
                              ),
                            ),
                          ),
                        );
                      }).toList(),
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildMobileLocationPointsSection() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          'Location Points',
          style: TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
            color: Colors.grey[800],
          ),
        ),
        const SizedBox(height: 12),
        Container(
          padding: const EdgeInsets.all(16),
          decoration: BoxDecoration(
            color: Colors.grey[50],
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: Colors.grey[300]!),
          ),
          child: Column(
            children: [
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton.icon(
                      onPressed: () {
                        // Add waypoint functionality
                        Navigator.pop(context);
                      },
                      icon: const Icon(Icons.location_on, size: 18),
                      label: const Text('Add Waypoint'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.blue[600],
                        foregroundColor: Colors.white,
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 8),
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton.icon(
                      onPressed: () {
                        // Add charging station functionality
                        Navigator.pop(context);
                      },
                      icon: const Icon(Icons.battery_charging_full, size: 18),
                      label: const Text('Add Charging Station'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.green[600],
                        foregroundColor: Colors.white,
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 8),
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton.icon(
                      onPressed: () {
                        // Clear all points functionality
                        Navigator.pop(context);
                      },
                      icon: const Icon(Icons.clear_all, size: 18),
                      label: const Text('Clear All Points'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.red[600],
                        foregroundColor: Colors.white,
                      ),
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
      ],
    );
  }

  void _handleScaleStart(ScaleStartDetails details) {
    if (_currentTool == EditTool.pencil ||
        _currentTool == EditTool.brush ||
        _currentTool == EditTool.eraser) {
      _startDrawing(details.localFocalPoint);
    }
  }

  void _handleScaleUpdate(ScaleUpdateDetails details) {
    if (details.scale != 1.0) {
      setState(() {
        _scale = (_scale * details.scale).clamp(_minScale, _maxScale);
      });
    } else {
      final delta = details.focalPointDelta;
      if (_isDrawing) {
        _continueDrawing(details.localFocalPoint);
      } else if (_currentTool == EditTool.pan) {
        setState(() {
          _panOffset += delta;
        });
      }
    }
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
        if (_currentTool == EditTool.select) {
          _handleLocationPointCreation(mapPosition);
        }
        break;
    }
  }

  void _handleSelection(Offset position) {
    for (final point in _locationPoints) {
      final pointScreenPos = _mapToScreenCoordinates(point.position);
      final distance = (pointScreenPos - position).distance;

      if (distance < 20) {
        setState(() {
          _selectedPoint = point;
        });
        return;
      }
    }

    setState(() {
      _selectedPoint = null;
    });
  }

  void _handleLocationPointCreation(Offset mapPosition) {
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

  void _startDrawing(Offset position) {
    final mapPosition = _screenToMapCoordinates(position);
    setState(() {
      _isDrawing = true;
      _currentStrokePoints = [mapPosition];
    });
  }

  void _continueDrawing(Offset position) {
    if (!_isDrawing) return;

    final mapPosition = _screenToMapCoordinates(position);
    setState(() {
      _currentStrokePoints.add(mapPosition);
    });

    _applyStrokeToImage();
  }

  void _finishDrawing() {
    if (!_isDrawing) return;

    final stroke = DrawingStroke(
      points: List.from(_currentStrokePoints),
      tool: _currentTool,
      layer: _currentLayer,
      brushSize: _brushSize,
      opacity: _brushOpacity,
    );

    setState(() {
      _isDrawing = false;
      _undoStack.add(stroke);
      _redoStack.clear();
      _currentStrokePoints.clear();
      _hasUnsavedChanges = true;
    });

    if (_undoStack.length > 100) {
      _undoStack.removeAt(0);
    }
  }

  void _applyStrokeToImage() {
    if (_currentPgmData == null || _currentStrokePoints.isEmpty) return;

    for (final point in _currentStrokePoints) {
      _applyBrushAtPoint(point);
    }
  }

  void _applyBrushAtPoint(Offset point) {
    final x = point.dx.round();
    final y = point.dy.round();
    final radius = (_brushSize / 2).round();

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

              switch (_currentTool) {
                case EditTool.pencil:
                case EditTool.brush:
                  newValue = _currentLayer.value;
                  break;
                case EditTool.eraser:
                  newValue = MapLayer.free.value;
                  break;
                default:
                  continue;
              }

              final intensity = _brushOpacity * (1.0 - (distance / radius));
              final currentValue = _currentPgmData![index];

              _currentPgmData![index] =
                  (currentValue + (newValue - currentValue) * intensity)
                      .round()
                      .clamp(0, 255);
            }
          }
        }
      }
    }

    _updateDisplayImage();
  }

  void _updateDisplayImage() {
    if (_currentPgmData == null) return;

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
        if (mounted) {
          setState(() {
            _mapImage?.dispose();
            _mapImage = image;
          });
        }
      },
    );
  }

  Offset _screenToMapCoordinates(Offset screenPoint) {
    return (screenPoint - _panOffset) / _scale;
  }

  Offset _mapToScreenCoordinates(Offset mapPoint) {
    return mapPoint * _scale + _panOffset;
  }

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
    if (_originalPgmData != null) {
      _currentPgmData = Uint8List.fromList(_originalPgmData!);

      for (final stroke in _undoStack) {
        for (final point in stroke.points) {
          _applyBrushAtPointFromStroke(point, stroke);
        }
      }

      _updateDisplayImage();
      setState(() {
        _hasUnsavedChanges = true;
      });
    }
  }

  void _applyBrushAtPointFromStroke(Offset point, DrawingStroke stroke) {
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

              final intensity = stroke.opacity * (1.0 - (distance / radius));
              final currentValue = _currentPgmData![index];

              _currentPgmData![index] =
                  (currentValue + (newValue - currentValue) * intensity)
                      .round()
                      .clamp(0, 255);
            }
          }
        }
      }
    }
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
    if (_mapImage == null) return;

    final renderBox = context.findRenderObject() as RenderBox?;
    if (renderBox == null) return;

    final size = renderBox.size;
    print(
        '️ Fitting map to screen. Canvas size: ${size.width}x${size.height}');

    // Add some padding to prevent the map from touching the edges
    final padding = 40.0;
    final availableWidth = size.width - padding * 2;
    final availableHeight = size.height - padding * 2;

    final scaleX = availableWidth / _mapImage!.width;
    final scaleY = availableHeight / _mapImage!.height;

    setState(() {
      _scale = math.min(scaleX, scaleY).clamp(_minScale, _maxScale);

      // Center the map in the viewport
      final scaledMapWidth = _mapImage!.width * _scale;
      final scaledMapHeight = _mapImage!.height * _scale;

      final centerX = (size.width - scaledMapWidth) / 2;
      final centerY = (size.height - scaledMapHeight) / 2;

      _panOffset = Offset(centerX, centerY);

      print(
          ' Map centered at offset: (${centerX.toStringAsFixed(1)}, ${centerY.toStringAsFixed(1)}) with scale: ${_scale.toStringAsFixed(2)}');
    });
  }

  void _centerMap() {
    if (_mapImage == null) return;

    final renderBox = context.findRenderObject() as RenderBox?;
    if (renderBox == null) return;

    final size = renderBox.size;
    print(' Centering map. Canvas size: ${size.width}x${size.height}');

    setState(() {
      // Center the map in the viewport without changing scale
      final scaledMapWidth = _mapImage!.width * _scale;
      final scaledMapHeight = _mapImage!.height * _scale;

      final centerX = (size.width - scaledMapWidth) / 2;
      final centerY = (size.height - scaledMapHeight) / 2;

      _panOffset = Offset(centerX, centerY);

      print(
          ' Map recentered at offset: (${centerX.toStringAsFixed(1)}, ${centerY.toStringAsFixed(1)})');
    });
  }

  // Public method to center the map from external calls
  void centerMapView() {
    _centerMap();
  }

  void _openMap() async {
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('Open map functionality coming soon')),
    );
  }

  Future<void> _saveMap() async {
    if (_currentPgmData == null) return;

    try {
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
        const SnackBar(content: Text('Map saved successfully!')),
      );
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Error saving map: $e')),
      );
    }
  }

  void _exportMap() async {
    try {
      if (widget.deviceId != null) {
        final response = await _apiService.exportMapToPGM(
          deviceId: widget.deviceId!,
          mapName: 'edited_map_${DateTime.now().millisecondsSinceEpoch}',
        );

        if (response['success'] == true) {
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(content: Text('Map exported successfully!')),
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
    try {
      await _saveMap();

      if (widget.deviceId != null) {
        // First export the map to backend to ensure files exist
        final mapName = 'edited_map_${DateTime.now().millisecondsSinceEpoch}';
        final exportResponse = await _apiService.exportMapToPGM(
          deviceId: widget.deviceId!,
          mapName: mapName,
        );

        if (exportResponse['success'] != true) {
          throw Exception('Failed to export map to backend');
        }

        // Now deploy the exported map
        final response = await _apiService.deployMapToRaspberryPi(
          deviceId: widget.deviceId!,
          mapName: mapName,
          autoLoad: true,
        );

        if (response['success'] == true) {
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(
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

  void _editLocationPoint(LocationPoint point) {
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

// Simplified map painter
class SimplifiedMapPainter extends CustomPainter {
  final ui.Image mapImage;
  final double scale;
  final Offset panOffset;
  final List<LocationPoint> locationPoints;
  final LocationPoint? selectedPoint;
  final EditTool currentTool;
  final MapLayer currentLayer;
  final double brushSize;
  final List<Offset> currentStrokePoints;

  SimplifiedMapPainter({
    required this.mapImage,
    required this.scale,
    required this.panOffset,
    required this.locationPoints,
    this.selectedPoint,
    required this.currentTool,
    required this.currentLayer,
    required this.brushSize,
    required this.currentStrokePoints,
  });

  @override
  void paint(Canvas canvas, Size size) {
    canvas.save();
    canvas.translate(size.width / 2, size.height / 2);
    canvas.translate(panOffset.dx, panOffset.dy);
    canvas.scale(scale);
    canvas.translate(-mapImage.width / 2, -mapImage.height / 2);

    // Draw map
    canvas.drawImage(mapImage, Offset.zero, Paint());

    // Draw location points
    for (final point in locationPoints) {
      _drawLocationPoint(canvas, point, point == selectedPoint);
    }

    // Draw current stroke
    if (currentStrokePoints.isNotEmpty) {
      _drawCurrentStroke(canvas);
    }

    canvas.restore();
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

    canvas.drawCircle(point.position, radius, paint);
    canvas.drawCircle(point.position, radius, outlinePaint);

    final textPainter = TextPainter(
      text: TextSpan(
        text: point.type.icon,
        style: const TextStyle(
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
  }

  void _drawCurrentStroke(Canvas canvas) {
    if (currentStrokePoints.length < 2) return;

    final paint = Paint()
      ..color = currentLayer.color
      ..style = PaintingStyle.stroke
      ..strokeWidth = brushSize
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;

    final path = Path();
    path.moveTo(currentStrokePoints.first.dx, currentStrokePoints.first.dy);

    for (int i = 1; i < currentStrokePoints.length; i++) {
      path.lineTo(currentStrokePoints[i].dx, currentStrokePoints[i].dy);
    }

    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(SimplifiedMapPainter oldDelegate) {
    return oldDelegate.scale != scale ||
        oldDelegate.panOffset != panOffset ||
        oldDelegate.locationPoints != locationPoints ||
        oldDelegate.selectedPoint != selectedPoint ||
        oldDelegate.currentStrokePoints != currentStrokePoints;
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
      title: const Text('Edit Location Point'),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          TextField(
            controller: _nameController,
            decoration: const InputDecoration(
              labelText: 'Name',
              border: OutlineInputBorder(),
            ),
          ),
          const SizedBox(height: 16),
          DropdownButtonFormField<LocationPointType>(
            value: _selectedType,
            decoration: const InputDecoration(
              labelText: 'Type',
              border: OutlineInputBorder(),
            ),
            items: LocationPointType.values.map((type) {
              return DropdownMenuItem(
                value: type,
                child: Row(
                  children: [
                    Text(type.icon),
                    const SizedBox(width: 8),
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
          child: const Text('Cancel'),
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
          child: const Text('Save'),
        ),
      ],
    );
  }
}
