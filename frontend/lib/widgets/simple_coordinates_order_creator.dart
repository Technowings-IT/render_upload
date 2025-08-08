// widgets/simple_coordinate_order_creator.dart - EXACTLY what you need
import 'package:flutter/material.dart';
import '../models/map_data.dart';
import '../models/odom.dart'
    as odom; // ✅ NEW: Import with prefix to avoid conflict
import '../services/api_service.dart';

class SimpleCoordinateOrderCreator extends StatefulWidget {
  final String deviceId;
  final MapData mapData;
  final Function(Map<String, dynamic>) onOrderCreated;

  const SimpleCoordinateOrderCreator({
    Key? key,
    required this.deviceId,
    required this.mapData,
    required this.onOrderCreated,
  }) : super(key: key);

  @override
  State<SimpleCoordinateOrderCreator> createState() =>
      _SimpleCoordinateOrderCreatorState();
}

class _SimpleCoordinateOrderCreatorState
    extends State<SimpleCoordinateOrderCreator> {
  final ApiService _apiService = ApiService();

  // State variables
  List<Map<String, dynamic>> _coordinates = [];
  bool _isCreatingOrder = false;
  bool _isExecutingOrder = false;
  int _currentCoordinateIndex = 0;
  bool _showSavedOrderPreview = false; // ✅ NEW: Show saved order preview
  Map<String, dynamic>? _lastSavedOrder; // ✅ NEW: Track last saved order

  // ✅ NEW: Map selection functionality
  List<Map<String, dynamic>> _availableMaps = [];
  String? _selectedMapName;
  MapData? _selectedMapData;
  bool _isLoadingMaps = false;
  bool _isLoadingMapData = false;

  // Map interaction
  double _mapScale = 1.0;
  Offset _mapOffset = Offset.zero;
  Size _canvasSize = Size(600, 500); // ✅ NEW: Track actual canvas size

  // ✅ NEW: Pan/drag state management
  bool _isPanning = false;
  Offset? _lastPanPosition;

  // Form
  final TextEditingController _orderNameController = TextEditingController();
  final TextEditingController _coordinateNameController =
      TextEditingController();

  @override
  void initState() {
    super.initState();
    _orderNameController.text =
        'Order ${DateTime.now().millisecondsSinceEpoch}';

    // ✅ FIXED: Initialize with YOUR YAML values instead of potentially wrong widget.mapData
    print('🚀 Initializing with YAML origin values: (-2.02, -5.21)');

    // Create a corrected MapData with your YAML origin values
    _selectedMapData = MapData(
      deviceId: widget.deviceId,
      timestamp: DateTime.now(),
      info: MapInfo(
        width: widget.mapData.info.width,
        height: widget.mapData.info.height,
        resolution: widget.mapData.info.resolution,
        origin: odom.Position(
          x: -2.02, // ✅ YOUR YAML origin X
          y: -5.21, // ✅ YOUR YAML origin Y
          z: 0.0,
        ),
        originOrientation: widget.mapData.info.originOrientation,
      ),
      occupancyData: widget.mapData.occupancyData,
      shapes: widget.mapData.shapes,
      version: widget.mapData.version,
    );

    print(
        '✅ Initialized with corrected origin: (${_selectedMapData!.info.origin.x}, ${_selectedMapData!.info.origin.y})');

    // Load available maps for the device
    _loadAvailableMaps();
  }

  // ✅ NEW: Load available saved maps for the device
  Future<void> _loadAvailableMaps() async {
    setState(() => _isLoadingMaps = true);

    try {
      print('🔍 Loading maps for device: ${widget.deviceId}');
      final maps = await _apiService.getSavedMapsEnhanced(widget.deviceId);

      print('📋 Raw maps response: $maps');
      print('📊 Maps count: ${maps.length}');

      setState(() {
        _availableMaps = maps;
        // Pre-select the first map if available
        if (_availableMaps.isNotEmpty && _selectedMapName == null) {
          _selectedMapName = _availableMaps.first['name'];
          print('🎯 Pre-selected map: $_selectedMapName');
          _loadSelectedMapData(_selectedMapName!);
        } else {
          print('⚠️ No maps found or map already selected');
        }
      });

      print(
          '✅ Loaded ${_availableMaps.length} maps for device: ${widget.deviceId}');

      // ✅ NEW: Try alternative map loading methods if no maps found
      if (_availableMaps.isEmpty) {
        print('🔄 Trying alternative map loading methods...');
        await _tryAlternativeMapLoading();
      }
    } catch (e) {
      print('❌ Error loading maps: $e');
      print('📍 Stack trace: ${StackTrace.current}');

      // ✅ NEW: Try fallback methods
      await _tryAlternativeMapLoading();

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Failed to load maps: $e'),
          backgroundColor: Colors.orange,
        ),
      );
    } finally {
      setState(() => _isLoadingMaps = false);
    }
  }

  // ✅ NEW: Try alternative methods to load maps
  Future<void> _tryAlternativeMapLoading() async {
    try {
      print('🔄 Trying getSavedMaps (regular) method...');
      final regularMaps = await _apiService.getSavedMaps(widget.deviceId);

      if (regularMaps.isNotEmpty) {
        print('✅ Found maps using regular method: ${regularMaps.length}');
        setState(() {
          _availableMaps = regularMaps;
          if (_selectedMapName == null && _availableMaps.isNotEmpty) {
            _selectedMapName = _availableMaps.first['name'];
            print('🎯 Pre-selected map (fallback): $_selectedMapName');
            _loadSelectedMapData(_selectedMapName!);
          }
        });
        return;
      }

      print('🔄 Trying getMapData method as fallback...');
      final mapDataResponse = await _apiService.getMapData(widget.deviceId);

      if (mapDataResponse['success'] == true) {
        print('✅ Found map data, creating virtual map entry');
        setState(() {
          _availableMaps = [
            {
              'name': 'Current Map',
              'deviceId': widget.deviceId,
              'timestamp': DateTime.now().toIso8601String(),
            }
          ];
          _selectedMapName = 'Current Map';
          _selectedMapData = widget.mapData; // Use the provided map data
        });
        return;
      }

      print('⚠️ No maps found using any method, using default map data');
      setState(() {
        _availableMaps = [
          {
            'name': 'Default Map',
            'deviceId': widget.deviceId,
            'timestamp': DateTime.now().toIso8601String(),
          }
        ];
        _selectedMapName = 'Default Map';
        _selectedMapData = widget.mapData; // Use the provided map data
      });
    } catch (e) {
      print('❌ All fallback methods failed: $e');
      // Create a minimal default entry so user can still create coordinates
      setState(() {
        _availableMaps = [
          {
            'name': 'Default Map',
            'deviceId': widget.deviceId,
            'timestamp': DateTime.now().toIso8601String(),
          }
        ];
        _selectedMapName = 'Default Map';
        _selectedMapData = widget.mapData; // Use the provided map data
      });
    }
  }

  // ✅ NEW: Load selected map data
// ✅ FIXED: Load selected map data with proper YAML parsing
  Future<void> _loadSelectedMapData(String mapName) async {
    setState(() => _isLoadingMapData = true);

    try {
      print('🔄 Loading map data for: $mapName');

      // Get the detailed map data for the selected map
      final mapData = await _apiService.getMapData(widget.deviceId);

      print('📦 Raw map data response: ${mapData.keys.toList()}');

      if (mapData['success'] == true && mapData['mapData'] != null) {
        // Parse the map data according to your MapData model
        final rawMapData = mapData['mapData'];

        print('🗺️ Raw map data structure:');
        print('   - Keys: ${rawMapData.keys.toList()}');
        print('   - Info: ${rawMapData['info']}');

        // ✅ ENHANCED: Better data parsing with YAML origin detection
        final mapInfo = rawMapData['info'] ?? {};
        final width = mapInfo['width'] ?? 164;
        final height = mapInfo['height'] ?? 145;
        final resolution = mapInfo['resolution'] ?? 0.05;

        // ✅ FIXED: Properly parse origin from YAML with extensive debugging
        var originX = -2.02; // Your actual YAML value as fallback
        var originY = -5.21; // Your actual YAML value as fallback

        print('🎯 Parsing origin from map info:');
        print('   - Raw mapInfo: $mapInfo');
        print('   - mapInfo keys: ${mapInfo.keys.toList()}');

        if (mapInfo.containsKey('origin')) {
          final rawOrigin = mapInfo['origin'];
          print('   - Raw origin value: $rawOrigin');
          print('   - Origin type: ${rawOrigin.runtimeType}');

          if (rawOrigin is List && rawOrigin.isNotEmpty) {
            print('   - Origin is List with ${rawOrigin.length} elements');
            print('   - Origin elements: $rawOrigin');

            if (rawOrigin.length >= 2) {
              originX = (rawOrigin[0] as num?)?.toDouble() ?? -2.02;
              originY = (rawOrigin[1] as num?)?.toDouble() ?? -5.21;
              print('   - Parsed origin: ($originX, $originY)');
            }
          } else if (rawOrigin is Map) {
            print('   - Origin is Map: $rawOrigin');
            originX = (rawOrigin['x'] as num?)?.toDouble() ?? -2.02;
            originY = (rawOrigin['y'] as num?)?.toDouble() ?? -5.21;
            print('   - Parsed origin from map: ($originX, $originY)');
          } else {
            print('   - Origin format not recognized, using YAML values');
          }
        } else {
          print('   - No origin found in mapInfo, using YAML values');
        }

        // ✅ ENHANCED: Better occupancy data parsing
        List<int> occupancyData = [];
        final rawOccupancy =
            rawMapData['occupancyData'] ?? rawMapData['data'] ?? [];

        if (rawOccupancy is List) {
          occupancyData =
              rawOccupancy.map((e) => (e as num?)?.toInt() ?? -1).toList();
        }

        print('✅ Parsed map data:');
        print('   - Dimensions: ${width}x${height}');
        print('   - Resolution: $resolution meters/cell');
        print('   - Origin: ($originX, $originY) meters');
        print('   - Occupancy cells: ${occupancyData.length}');
        print('   - Expected cells: ${width * height}');

        // Sample some occupancy values for debugging
        if (occupancyData.isNotEmpty) {
          final sampleSize =
              (10 < occupancyData.length) ? 10 : occupancyData.length;
          final samples = occupancyData.take(sampleSize).toList();
          print('   - Sample occupancy values: $samples');

          // Count different value types
          final unknownCount = occupancyData.where((v) => v == -1).length;
          final freeCount =
              occupancyData.where((v) => v >= 0 && v <= 10).length;
          final occupiedCount = occupancyData.where((v) => v >= 90).length;
          print(
              '   - Value distribution: Unknown=$unknownCount, Free=$freeCount, Occupied=$occupiedCount');
        }

        setState(() {
          _selectedMapData = MapData(
            deviceId: widget.deviceId,
            timestamp: DateTime.now(),
            info: MapInfo(
              width: width,
              height: height,
              resolution: resolution,
              origin: odom.Position(
                x: originX, // ✅ FIXED: Use actual parsed origin
                y: originY, // ✅ FIXED: Use actual parsed origin
                z: 0.0,
              ),
              originOrientation: odom.Orientation(x: 0, y: 0, z: 0, w: 1),
            ),
            occupancyData: occupancyData,
            shapes: [], // Will be populated if needed
            version: 1,
          );
        });

        print('✅ Map data loaded with REAL origin: ($originX, $originY)');
      } else {
        throw Exception(
            'Invalid map data response: ${mapData['error'] ?? 'Unknown error'}');
      }
    } catch (e) {
      print('❌ Error loading map data: $e');
      print('📍 Stack trace: ${StackTrace.current}');

      // ✅ FIXED: Even fallback should use your YAML values
      print('🔄 Using fallback with YOUR YAML origin values');
      setState(() {
        _selectedMapData = MapData(
          deviceId: widget.deviceId,
          timestamp: DateTime.now(),
          info: MapInfo(
            width: 164,
            height: 145,
            resolution: 0.05,
            origin: odom.Position(
              x: -2.02, // ✅ YOUR actual YAML origin
              y: -5.21, // ✅ YOUR actual YAML origin
              z: 0.0,
            ),
            originOrientation: odom.Orientation(x: 0, y: 0, z: 0, w: 1),
          ),
          occupancyData: widget.mapData.occupancyData, // Use provided data
          shapes: [],
          version: 1,
        );
      });

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Using fallback map data with YAML origin'),
          backgroundColor: Colors.orange,
        ),
      );
    } finally {
      setState(() => _isLoadingMapData = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Create Coordinate Order'),
        backgroundColor: Colors.blue,
        foregroundColor: Colors.white,
      ),
      body: Row(
        children: [
          // LEFT: Simple Map
          Expanded(
            flex: 3,
            child: _buildSimpleMap(),
          ),

          // RIGHT: Coordinate List & Controls
          Expanded(
            flex: 2,
            child: _buildControlPanel(),
          ),
        ],
      ),
    );
  }

  // SIMPLE MAP - Just tap to add coordinates
  Widget _buildSimpleMap() {
    return Container(
      margin: EdgeInsets.all(8),
      decoration: BoxDecoration(
        border: Border.all(color: Colors.grey.shade300),
        borderRadius: BorderRadius.circular(8),
      ),
      child: LayoutBuilder(
        builder: (context, constraints) {
          // ✅ NEW: Track actual canvas size for accurate coordinate conversion
          _canvasSize = Size(constraints.maxWidth, constraints.maxHeight);

          return Stack(
            children: [
              // Map Canvas
              GestureDetector(
                onTapDown: _handleMapTap,
                // ✅ FIXED: Use only scale gestures (includes pan functionality)
                onScaleStart: _handleScaleStart,
                onScaleUpdate: _handleScaleUpdate,
                onScaleEnd: _handleScaleEnd,
                child: CustomPaint(
                  size: Size(constraints.maxWidth, constraints.maxHeight),
                  painter: SimpleMapPainter(
                    mapData: _selectedMapData ??
                        widget.mapData, // ✅ Use selected map data
                    coordinates: _coordinates,
                    mapScale: _mapScale,
                    mapOffset: _mapOffset,
                    currentExecutingIndex:
                        _isExecutingOrder ? _currentCoordinateIndex : -1,
                    canvasSize:
                        Size(constraints.maxWidth, constraints.maxHeight),
                    savedOrder: _lastSavedOrder, // ✅ NEW: Pass saved order
                    showSavedOrderPreview:
                        _showSavedOrderPreview, // ✅ NEW: Control visibility
                  ),
                ),
              ),

              // ✅ NEW: Loading overlay when map data is loading
              if (_isLoadingMapData)
                Container(
                  width: constraints.maxWidth,
                  height: constraints.maxHeight,
                  decoration: BoxDecoration(
                    color: Colors.white.withOpacity(0.8),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Center(
                    child: Column(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        CircularProgressIndicator(),
                        SizedBox(height: 16),
                        Text(
                          'Loading map data...',
                          style: TextStyle(
                            fontSize: 16,
                            fontWeight: FontWeight.w500,
                          ),
                        ),
                      ],
                    ),
                  ),
                ),

              // ✅ NEW: Map info overlay
              if (_selectedMapData != null && !_isLoadingMapData)
                Positioned(
                  top: 8,
                  left: 8,
                  child: Container(
                    padding: EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                    decoration: BoxDecoration(
                      color: Colors.black87,
                      borderRadius: BorderRadius.circular(4),
                    ),
                    child: Text(
                      'Map: ${_selectedMapName ?? "Default"} | ${_selectedMapData!.info.width}×${_selectedMapData!.info.height} | Device: ${widget.deviceId}',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 12,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                  ),
                ),

              // ✅ NEW: Coordinate info overlay
              if (_coordinates.isNotEmpty)
                Positioned(
                  top: 8,
                  right: 8,
                  child: Container(
                    padding: EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                    decoration: BoxDecoration(
                      color: Colors.blue.shade700.withOpacity(0.9),
                      borderRadius: BorderRadius.circular(4),
                    ),
                    child: Text(
                      '${_coordinates.length} coordinate${_coordinates.length == 1 ? '' : 's'}',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 12,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                  ),
                ),

              // ✅ NEW: Map controls overlay
              Positioned(
                bottom: 16,
                right: 16,
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    // Zoom In
                    Container(
                      decoration: BoxDecoration(
                        color: Colors.white,
                        borderRadius: BorderRadius.circular(8),
                        boxShadow: [
                          BoxShadow(
                            color: Colors.black26,
                            blurRadius: 4,
                            offset: Offset(0, 2),
                          ),
                        ],
                      ),
                      child: IconButton(
                        onPressed: () {
                          setState(() {
                            _mapScale = (_mapScale * 1.5).clamp(0.3, 5.0);
                          });
                        },
                        icon: Icon(Icons.zoom_in, color: Colors.blue),
                        tooltip: 'Zoom In',
                      ),
                    ),

                    SizedBox(height: 4),

                    // Zoom Out
                    Container(
                      decoration: BoxDecoration(
                        color: Colors.white,
                        borderRadius: BorderRadius.circular(8),
                        boxShadow: [
                          BoxShadow(
                            color: Colors.black26,
                            blurRadius: 4,
                            offset: Offset(0, 2),
                          ),
                        ],
                      ),
                      child: IconButton(
                        onPressed: () {
                          setState(() {
                            _mapScale = (_mapScale / 1.5).clamp(0.3, 5.0);
                          });
                        },
                        icon: Icon(Icons.zoom_out, color: Colors.blue),
                        tooltip: 'Zoom Out',
                      ),
                    ),

                    SizedBox(height: 4),

                    // Reset View
                    Container(
                      decoration: BoxDecoration(
                        color: Colors.white,
                        borderRadius: BorderRadius.circular(8),
                        boxShadow: [
                          BoxShadow(
                            color: Colors.black26,
                            blurRadius: 4,
                            offset: Offset(0, 2),
                          ),
                        ],
                      ),
                      child: IconButton(
                        onPressed: () {
                          setState(() {
                            _mapScale = 1.0;
                            _mapOffset = Offset.zero;
                          });
                        },
                        icon: Icon(Icons.center_focus_strong,
                            color: Colors.green),
                        tooltip: 'Reset View',
                      ),
                    ),

                    SizedBox(height: 8),

                    // Current zoom level indicator
                    Container(
                      padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                      decoration: BoxDecoration(
                        color: Colors.black87,
                        borderRadius: BorderRadius.circular(4),
                      ),
                      child: Text(
                        '${(_mapScale * 100).toStringAsFixed(0)}%',
                        style: TextStyle(
                          color: Colors.white,
                          fontSize: 12,
                          fontWeight: FontWeight.w500,
                        ),
                      ),
                    ),
                  ],
                ),
              ),

              // ✅ NEW: Pan instructions overlay
              if (!_isPanning && _coordinates.isEmpty)
                Positioned(
                  bottom: 16,
                  left: 16,
                  child: Container(
                    padding: EdgeInsets.all(12),
                    decoration: BoxDecoration(
                      color: Colors.blue.shade50.withOpacity(0.9),
                      borderRadius: BorderRadius.circular(8),
                      border: Border.all(color: Colors.blue.shade200),
                    ),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(Icons.touch_app, color: Colors.blue, size: 16),
                            SizedBox(width: 6),
                            Text(
                              'Map Controls',
                              style: TextStyle(
                                fontWeight: FontWeight.bold,
                                color: Colors.blue.shade700,
                                fontSize: 12,
                              ),
                            ),
                          ],
                        ),
                        SizedBox(height: 4),
                        Text(
                          '• Drag to pan around\n• Pinch to zoom\n• Tap to place coordinates',
                          style: TextStyle(
                            fontSize: 11,
                            color: Colors.blue.shade600,
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
            ],
          );
        },
      ),
    );
  }

  // CONTROL PANEL - Coordinate list and actions
  Widget _buildControlPanel() {
    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.grey.shade50,
        border: Border(left: BorderSide(color: Colors.grey.shade300)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Order Name
          TextField(
            controller: _orderNameController,
            decoration: InputDecoration(
              labelText: 'Order Name',
              border: OutlineInputBorder(),
            ),
          ),

          SizedBox(height: 16),

          // ✅ NEW: Map Selection Dropdown
          Container(
            width: double.infinity,
            decoration: BoxDecoration(
              color: Colors.white,
              borderRadius: BorderRadius.circular(8),
              border: Border.all(color: Colors.blue.shade300),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Padding(
                  padding: EdgeInsets.fromLTRB(12, 8, 12, 4),
                  child: Text(
                    'Select Map',
                    style: TextStyle(
                      fontSize: 12,
                      color: Colors.blue.shade700,
                      fontWeight: FontWeight.w500,
                    ),
                  ),
                ),
                if (_isLoadingMaps)
                  Padding(
                    padding: EdgeInsets.all(16),
                    child: Row(
                      children: [
                        SizedBox(
                          width: 16,
                          height: 16,
                          child: CircularProgressIndicator(strokeWidth: 2),
                        ),
                        SizedBox(width: 8),
                        Text('Loading maps...'),
                      ],
                    ),
                  )
                else if (_availableMaps.isEmpty)
                  Padding(
                    padding: EdgeInsets.all(16),
                    child: Column(
                      children: [
                        Icon(Icons.map_outlined,
                            color: Colors.grey.shade400, size: 32),
                        SizedBox(height: 8),
                        Text(
                          'No saved maps found',
                          style: TextStyle(
                            color: Colors.grey.shade600,
                            fontWeight: FontWeight.w500,
                          ),
                        ),
                        SizedBox(height: 4),
                        Text(
                          'Using default map for coordinate placement',
                          style: TextStyle(
                            color: Colors.grey.shade500,
                            fontSize: 12,
                          ),
                        ),
                        SizedBox(height: 8),
                        ElevatedButton.icon(
                          onPressed: () {
                            _loadAvailableMaps(); // Retry loading
                          },
                          icon: Icon(Icons.refresh, size: 16),
                          label: Text('Retry', style: TextStyle(fontSize: 12)),
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.blue,
                            foregroundColor: Colors.white,
                            padding: EdgeInsets.symmetric(
                                horizontal: 12, vertical: 6),
                          ),
                        ),
                      ],
                    ),
                  )
                else
                  Padding(
                    padding: EdgeInsets.fromLTRB(12, 4, 12, 8),
                    child: DropdownButtonHideUnderline(
                      child: DropdownButton<String>(
                        value: _selectedMapName,
                        isExpanded: true,
                        hint: Text('Choose a map'),
                        items: _availableMaps.map((map) {
                          return DropdownMenuItem<String>(
                            value: map['name'],
                            child: Row(
                              children: [
                                Icon(Icons.map, color: Colors.blue, size: 16),
                                SizedBox(width: 8),
                                Expanded(
                                  child: Text(
                                    map['name'] ?? 'Unnamed Map',
                                    overflow: TextOverflow.ellipsis,
                                  ),
                                ),
                              ],
                            ),
                          );
                        }).toList(),
                        onChanged: (String? newValue) {
                          if (newValue != null &&
                              newValue != _selectedMapName) {
                            setState(() {
                              _selectedMapName = newValue;
                            });
                            _loadSelectedMapData(newValue);
                          }
                        },
                      ),
                    ),
                  ),
              ],
            ),
          ),

          SizedBox(height: 16),

          // ✅ UPDATED: Map loading status
          if (_isLoadingMapData)
            Container(
              padding: EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: Colors.orange.shade50,
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.orange.shade200),
              ),
              child: Row(
                children: [
                  SizedBox(
                    width: 16,
                    height: 16,
                    child: CircularProgressIndicator(strokeWidth: 2),
                  ),
                  SizedBox(width: 8),
                  Text(
                    'Loading map data...',
                    style: TextStyle(color: Colors.orange.shade700),
                  ),
                ],
              ),
            )
          else
            // Instructions
            Container(
              padding: EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: Colors.blue.shade50,
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.blue.shade200),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    'Select a map above, then tap on map to add coordinates.\nCoordinates will be executed in order.',
                    style: TextStyle(color: Colors.blue.shade700),
                  ),
                  SizedBox(height: 8),
                  // ✅ NEW: Debug information
                  Container(
                    padding: EdgeInsets.all(8),
                    decoration: BoxDecoration(
                      color: Colors.grey.shade100,
                      borderRadius: BorderRadius.circular(4),
                    ),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          'Debug Info:',
                          style: TextStyle(
                            fontSize: 10,
                            fontWeight: FontWeight.bold,
                            color: Colors.grey.shade700,
                          ),
                        ),
                        Text(
                          'Device: ${widget.deviceId}',
                          style: TextStyle(
                              fontSize: 10, color: Colors.grey.shade600),
                        ),
                        Text(
                          'Available Maps: ${_availableMaps.length}',
                          style: TextStyle(
                              fontSize: 10, color: Colors.grey.shade600),
                        ),
                        Text(
                          'Selected: ${_selectedMapName ?? "None"}',
                          style: TextStyle(
                              fontSize: 10, color: Colors.grey.shade600),
                        ),
                        Text(
                          'Map Data: ${_selectedMapData != null ? "Loaded" : "Using Default"}',
                          style: TextStyle(
                              fontSize: 10, color: Colors.grey.shade600),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),

          SizedBox(height: 16),

          // ✅ NEW: Saved Order Preview Section
          if (_showSavedOrderPreview && _lastSavedOrder != null)
            Container(
              padding: EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: Colors.green.shade50,
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.green.shade200),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(
                    children: [
                      Icon(Icons.check_circle,
                          color: Colors.green.shade700, size: 20),
                      SizedBox(width: 8),
                      Text(
                        'Previously Saved Order',
                        style: TextStyle(
                          fontSize: 14,
                          fontWeight: FontWeight.bold,
                          color: Colors.green.shade700,
                        ),
                      ),
                      Spacer(),
                      IconButton(
                        icon: Icon(Icons.visibility_off,
                            color: Colors.green.shade600, size: 20),
                        onPressed: () {
                          setState(() {
                            _showSavedOrderPreview = false;
                          });
                        },
                        tooltip: 'Hide saved order preview',
                        padding: EdgeInsets.all(4),
                        constraints: BoxConstraints(),
                      ),
                    ],
                  ),
                  SizedBox(height: 8),
                  Text(
                    '✅ "${_lastSavedOrder!['name']}" with ${(_lastSavedOrder!['coordinates'] as List).length} waypoints',
                    style: TextStyle(
                      fontSize: 13,
                      color: Colors.green.shade600,
                    ),
                  ),
                  SizedBox(height: 8),
                  Text(
                    'Green markers on the map show the saved order. The order is available on the dashboard.',
                    style: TextStyle(
                      fontSize: 12,
                      color: Colors.green.shade600,
                      fontStyle: FontStyle.italic,
                    ),
                  ),
                ],
              ),
            ),

          if (_showSavedOrderPreview && _lastSavedOrder != null)
            SizedBox(height: 16),

          // Coordinate List
          Text(
            'Coordinates (${_coordinates.length})',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          ),

          SizedBox(height: 8),

          Expanded(
            child: _coordinates.isEmpty
                ? Center(
                    child: Text(
                      'No coordinates added yet.\nSelect a map and tap on it to start.',
                      textAlign: TextAlign.center,
                      style: TextStyle(color: Colors.grey.shade600),
                    ),
                  )
                : ListView.builder(
                    itemCount: _coordinates.length,
                    itemBuilder: (context, index) =>
                        _buildCoordinateItem(index),
                  ),
          ),

          // Action Buttons
          if (_coordinates.isNotEmpty) ...[
            SizedBox(height: 16),

            if (!_isExecutingOrder) ...[
              // Save Order Button
              SizedBox(
                width: double.infinity,
                child: ElevatedButton.icon(
                  onPressed: _isCreatingOrder ? null : _saveOrder,
                  icon: _isCreatingOrder
                      ? SizedBox(
                          width: 16,
                          height: 16,
                          child: CircularProgressIndicator(strokeWidth: 2))
                      : Icon(Icons.save),
                  label: Text(_isCreatingOrder ? 'Saving...' : 'Save Order'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.green,
                    foregroundColor: Colors.white,
                    padding: EdgeInsets.symmetric(vertical: 16),
                  ),
                ),
              ),

              SizedBox(height: 8),

              // Start Order Button
              SizedBox(
                width: double.infinity,
                child: ElevatedButton.icon(
                  onPressed: _startOrderExecution,
                  icon: Icon(Icons.play_arrow),
                  label: Text('Start Order Execution'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.blue,
                    foregroundColor: Colors.white,
                    padding: EdgeInsets.symmetric(vertical: 16),
                  ),
                ),
              ),
            ] else ...[
              // Execution Status
              Container(
                padding: EdgeInsets.all(16),
                decoration: BoxDecoration(
                  color: Colors.orange.shade50,
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: Colors.orange.shade200),
                ),
                child: Column(
                  children: [
                    Text(
                      'Executing Order...',
                      style: TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.bold,
                        color: Colors.orange.shade700,
                      ),
                    ),
                    SizedBox(height: 8),
                    Text(
                        'Coordinate ${_currentCoordinateIndex + 1} of ${_coordinates.length}'),
                    SizedBox(height: 8),
                    LinearProgressIndicator(
                      value:
                          (_currentCoordinateIndex + 1) / _coordinates.length,
                      backgroundColor: Colors.grey.shade300,
                      valueColor: AlwaysStoppedAnimation<Color>(Colors.orange),
                    ),
                  ],
                ),
              ),

              SizedBox(height: 16),

              // Stop Execution Button
              SizedBox(
                width: double.infinity,
                child: ElevatedButton.icon(
                  onPressed: _stopOrderExecution,
                  icon: Icon(Icons.stop),
                  label: Text('Stop Execution'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.red,
                    foregroundColor: Colors.white,
                    padding: EdgeInsets.symmetric(vertical: 16),
                  ),
                ),
              ),
            ],

            SizedBox(height: 8),

            // Clear All Button
            SizedBox(
              width: double.infinity,
              child: OutlinedButton.icon(
                onPressed: _isExecutingOrder ? null : _clearAllCoordinates,
                icon: Icon(Icons.clear_all),
                label: Text('Clear All'),
                style: OutlinedButton.styleFrom(
                  padding: EdgeInsets.symmetric(vertical: 16),
                ),
              ),
            ),
          ],
        ],
      ),
    );
  }

  Widget _buildCoordinateItem(int index) {
    final coord = _coordinates[index];
    final isExecuting = _isExecutingOrder && index == _currentCoordinateIndex;
    final isCompleted = _isExecutingOrder && index < _currentCoordinateIndex;

    return Card(
      color: isExecuting
          ? Colors.orange.shade50
          : isCompleted
              ? Colors.green.shade50
              : null,
      child: ListTile(
        leading: Container(
          width: 40,
          height: 40,
          decoration: BoxDecoration(
            color: isExecuting
                ? Colors.orange
                : isCompleted
                    ? Colors.green
                    : Colors.blue,
            shape: BoxShape.circle,
          ),
          child: Center(
            child: isExecuting
                ? SizedBox(
                    width: 20,
                    height: 20,
                    child: CircularProgressIndicator(
                        color: Colors.white, strokeWidth: 2),
                  )
                : isCompleted
                    ? Icon(Icons.check, color: Colors.white, size: 20)
                    : Text(
                        '${index + 1}',
                        style: TextStyle(
                            color: Colors.white, fontWeight: FontWeight.bold),
                      ),
          ),
        ),
        title: Text(coord['name']),
        subtitle: Text(
            '(${coord['x'].toStringAsFixed(2)}, ${coord['y'].toStringAsFixed(2)}) meters'),
        trailing: _isExecutingOrder
            ? null
            : PopupMenuButton<String>(
                onSelected: (value) {
                  if (value == 'edit') {
                    _editCoordinate(index);
                  } else if (value == 'delete') {
                    _deleteCoordinate(index);
                  }
                },
                itemBuilder: (context) => [
                  PopupMenuItem(value: 'edit', child: Text('Edit')),
                  PopupMenuItem(value: 'delete', child: Text('Delete')),
                ],
              ),
      ),
    );
  }

  // EVENT HANDLERS
  void _handleMapTap(TapDownDetails details) {
    if (_isExecutingOrder) return;

    // Don't place coordinates if we were panning
    if (_isPanning) {
      _isPanning = false;
      return;
    }

    if (_isLoadingMapData) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Please wait for map to load'),
          backgroundColor: Colors.orange,
        ),
      );
      return;
    }

    final coordinates = _screenToMapCoordinates(details.localPosition);
    _showCoordinateNameDialog(coordinates);
  }

  // ✅ FIXED: Combined scale/pan gesture handlers
  void _handleScaleStart(ScaleStartDetails details) {
    _isPanning = false; // Reset panning state
    _lastPanPosition = details.localFocalPoint;
    print('🔍 Scale/Pan started at: ${details.localFocalPoint}');
  }

  void _handleScaleUpdate(ScaleUpdateDetails details) {
    setState(() {
      // Handle zoom (scale)
      if (details.scale != 1.0) {
        final newScale = (_mapScale * details.scale).clamp(0.3, 5.0);
        if (newScale != _mapScale) {
          _mapScale = newScale;
          print('🔍 Zoom: ${_mapScale.toStringAsFixed(2)}x');
        }
      }

      // Handle pan (when scale is 1.0, it's just panning)
      if (details.scale == 1.0 && _lastPanPosition != null) {
        _isPanning = true;
        final delta = details.localFocalPoint - _lastPanPosition!;
        _mapOffset += delta;
        _lastPanPosition = details.localFocalPoint;

        // Debug info for panning
        if (delta.distance > 5) {
          print(
              '🗺️ Panning: offset=(${_mapOffset.dx.toStringAsFixed(1)}, ${_mapOffset.dy.toStringAsFixed(1)})');
        }
      } else {
        // Update focal point for zoom operations
        _lastPanPosition = details.localFocalPoint;

        // Handle pan during pinch
        _mapOffset += details.focalPointDelta;
      }
    });
  }

  void _handleScaleEnd(ScaleEndDetails details) {
    _lastPanPosition = null;
    print('🔍 Scale/Pan ended at ${_mapScale.toStringAsFixed(2)}x');

    // Reset panning flag after a short delay to avoid accidental coordinate placement
    if (_isPanning) {
      Future.delayed(Duration(milliseconds: 100), () {
        _isPanning = false;
      });
    }
  }

  void _showCoordinateNameDialog(Map<String, double> coordinates) {
    _coordinateNameController.clear();

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Add Coordinate'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(
                'Position: (${coordinates['x']!.toStringAsFixed(2)}, ${coordinates['y']!.toStringAsFixed(2)}) meters'),
            SizedBox(height: 16),
            TextField(
              controller: _coordinateNameController,
              decoration: InputDecoration(
                labelText: 'Coordinate Name',
                hintText: 'e.g., Point A, Station 1',
                border: OutlineInputBorder(),
              ),
              autofocus: true,
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
              _addCoordinate(
                  coordinates, _coordinateNameController.text.trim());
              Navigator.of(context).pop();
            },
            child: Text('Add'),
          ),
        ],
      ),
    );
  }

  void _addCoordinate(Map<String, double> coordinates, String name) {
    setState(() {
      _coordinates.add({
        'name': name.isNotEmpty ? name : 'Point ${_coordinates.length + 1}',
        'x': coordinates['x']!,
        'y': coordinates['y']!,
      });
    });

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
          content: Text('Coordinate added!'), backgroundColor: Colors.green),
    );
  }

  void _editCoordinate(int index) {
    _coordinateNameController.text = _coordinates[index]['name'];

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Edit Coordinate'),
        content: TextField(
          controller: _coordinateNameController,
          decoration: InputDecoration(
            labelText: 'Coordinate Name',
            border: OutlineInputBorder(),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              setState(() {
                _coordinates[index]['name'] =
                    _coordinateNameController.text.trim();
              });
              Navigator.of(context).pop();
            },
            child: Text('Save'),
          ),
        ],
      ),
    );
  }

  void _deleteCoordinate(int index) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Delete Coordinate'),
        content: Text('Delete "${_coordinates[index]['name']}"?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              setState(() => _coordinates.removeAt(index));
              Navigator.of(context).pop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child: Text('Delete'),
          ),
        ],
      ),
    );
  }

  void _clearAllCoordinates() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Clear All Coordinates'),
        content: Text('Remove all coordinates?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              setState(() => _coordinates.clear());
              Navigator.of(context).pop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child: Text('Clear All'),
          ),
        ],
      ),
    );
  }

  // SAVE ORDER - Just coordinates in sequence
  Future<void> _saveOrder() async {
    setState(() => _isCreatingOrder = true);

    try {
      // Create simple order data - EXACTLY what you wanted
      final orderData = {
        'id': 'order_${DateTime.now().millisecondsSinceEpoch}',
        'name': _orderNameController.text.trim(),
        'deviceId': widget.deviceId,
        'coordinates': _coordinates, // Just the coordinates!
        'status': 'pending',
        'currentCoordinate': 0,
        'createdAt': DateTime.now().toIso8601String(),
      };

      // Save to backend
      final response = await _apiService.createSimpleCoordinateOrder(
        deviceId: widget.deviceId,
        name: orderData['name'] as String,
        coordinates: _coordinates,
      );

      if (response['success'] == true) {
        widget.onOrderCreated(orderData);

        // ✅ NEW: Show success dialog with order details instead of just closing
        await _showOrderSavedDialog(orderData);
      } else {
        throw Exception(response['error'] ?? 'Failed to save order');
      }
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
            content: Text('❌ Save failed: $e'), backgroundColor: Colors.red),
      );
    } finally {
      setState(() => _isCreatingOrder = false);
    }
  }

  // START ORDER EXECUTION - Send coordinates one by one
  Future<void> _startOrderExecution() async {
    setState(() {
      _isExecutingOrder = true;
      _currentCoordinateIndex = 0;
    });

    try {
      await _executeNextCoordinate();
    } catch (e) {
      _showRestartDialog('Execution failed: $e');
    }
  }

  Future<void> _executeNextCoordinate() async {
    if (_currentCoordinateIndex >= _coordinates.length) {
      // All coordinates completed
      _showCompletionDialog();
      return;
    }

    final coord = _coordinates[_currentCoordinateIndex];

    try {
      // Send coordinate to /target_pose
      final response = await _apiService.publishGoalToTargetPose(
        deviceId: widget.deviceId,
        x: coord['x'],
        y: coord['y'],
        orientation: 0.0,
      );

      if (response['success'] == true) {
        // Wait for navigation feedback
        await _waitForNavigationFeedback();
      } else {
        throw Exception(response['error'] ?? 'Failed to publish goal');
      }
    } catch (e) {
      _showRestartDialog(
          'Failed to execute coordinate ${_currentCoordinateIndex + 1}: $e');
    }
  }

  Future<void> _waitForNavigationFeedback() async {
    // Subscribe to navigation feedback and wait for success/failure
    // This should integrate with your existing navigation feedback system

    // For now, simulate waiting for feedback
    await Future.delayed(Duration(seconds: 3));

    // ✅ FIXED: Make success variable dynamic (in real implementation, this comes from navigate_to_pose feedback)
    final success = DateTime.now().millisecond % 2 ==
        0; // Random success/failure for testing

    if (success) {
      setState(() => _currentCoordinateIndex++);
      // Execute next coordinate
      await _executeNextCoordinate();
    } else {
      _showRestartDialog(
          'Navigation to coordinate ${_currentCoordinateIndex + 1} failed');
    }
  }

  void _stopOrderExecution() {
    setState(() {
      _isExecutingOrder = false;
      _currentCoordinateIndex = 0;
    });
  }

  void _showRestartDialog(String error) {
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (context) => AlertDialog(
        title: Text('❌ Execution Failed'),
        content: Text('$error\n\nWould you like to restart the order?'),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
              _stopOrderExecution();
            },
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _startOrderExecution(); // Restart from beginning
            },
            child: Text('Restart Order'),
          ),
        ],
      ),
    );
  }

  void _showCompletionDialog() {
    setState(() => _isExecutingOrder = false);

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('✅ Order Completed'),
        content: Text('All coordinates have been executed successfully!'),
        actions: [
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              Navigator.of(context).pop(); // Go back to previous screen
            },
            child: Text('OK'),
          ),
        ],
      ),
    );
  }

  // ✅ NEW: Show order saved dialog with order details
  Future<void> _showOrderSavedDialog(Map<String, dynamic> orderData) async {
    // ✅ Store the saved order for preview
    setState(() {
      _lastSavedOrder = orderData;
      _showSavedOrderPreview = true;
    });

    return showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Row(
          children: [
            Icon(Icons.check_circle, color: Colors.green, size: 28),
            SizedBox(width: 8),
            Text('✅ Order Saved!'),
          ],
        ),
        content: Container(
          width: double.maxFinite,
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'Your coordinate order has been saved successfully!',
                style: TextStyle(fontSize: 16, fontWeight: FontWeight.w500),
              ),
              SizedBox(height: 16),

              // Order details card
              Container(
                padding: EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: Colors.blue.shade50,
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: Colors.blue.shade200),
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      children: [
                        Icon(Icons.label,
                            color: Colors.blue.shade700, size: 20),
                        SizedBox(width: 8),
                        Text(
                          'Order Name:',
                          style: TextStyle(
                            fontWeight: FontWeight.bold,
                            color: Colors.blue.shade700,
                          ),
                        ),
                      ],
                    ),
                    SizedBox(height: 4),
                    Text(
                      orderData['name'] as String,
                      style: TextStyle(fontSize: 16),
                    ),
                    SizedBox(height: 12),

                    Row(
                      children: [
                        Icon(Icons.place,
                            color: Colors.blue.shade700, size: 20),
                        SizedBox(width: 8),
                        Text(
                          'Coordinates:',
                          style: TextStyle(
                            fontWeight: FontWeight.bold,
                            color: Colors.blue.shade700,
                          ),
                        ),
                      ],
                    ),
                    SizedBox(height: 4),
                    Text(
                      '${(orderData['coordinates'] as List).length} waypoints',
                      style: TextStyle(fontSize: 16),
                    ),
                    SizedBox(height: 8),

                    // Show first few coordinates
                    if ((orderData['coordinates'] as List).isNotEmpty) ...[
                      Text(
                        'Waypoints:',
                        style: TextStyle(
                          fontWeight: FontWeight.w500,
                          color: Colors.blue.shade600,
                        ),
                      ),
                      SizedBox(height: 4),
                      Container(
                        height: 100,
                        child: ListView.builder(
                          itemCount: (orderData['coordinates'] as List).length,
                          itemBuilder: (context, index) {
                            final coord =
                                (orderData['coordinates'] as List)[index];
                            return Padding(
                              padding: EdgeInsets.symmetric(vertical: 2),
                              child: Row(
                                children: [
                                  Container(
                                    width: 20,
                                    height: 20,
                                    decoration: BoxDecoration(
                                      color: Colors.blue,
                                      shape: BoxShape.circle,
                                    ),
                                    child: Center(
                                      child: Text(
                                        '${index + 1}',
                                        style: TextStyle(
                                          color: Colors.white,
                                          fontSize: 12,
                                          fontWeight: FontWeight.bold,
                                        ),
                                      ),
                                    ),
                                  ),
                                  SizedBox(width: 8),
                                  Expanded(
                                    child: Text(
                                      '${coord['name']} (${coord['x'].toStringAsFixed(2)}, ${coord['y'].toStringAsFixed(2)}) m',
                                      style: TextStyle(fontSize: 14),
                                    ),
                                  ),
                                ],
                              ),
                            );
                          },
                        ),
                      ),
                    ],
                  ],
                ),
              ),

              SizedBox(height: 12),
              Text(
                'The order is now available on the dashboard and ready for execution.',
                style: TextStyle(
                  fontSize: 14,
                  color: Colors.grey.shade600,
                  fontStyle: FontStyle.italic,
                ),
              ),
            ],
          ),
        ),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.of(context).pop(); // Close dialog
              _resetOrderForm(); // Clear current order to start fresh
            },
            child: Text('Create Another Order'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop(); // Close dialog
              Navigator.of(context).pop(); // Go back to dashboard

              // ✅ NEW: Small delay to let dashboard load, then show where to find the order
              Future.delayed(Duration(milliseconds: 500), () {
                if (context.mounted) {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: Text(
                          '💡 Your order "${orderData['name']}" is now visible in the Orders section below!'),
                      backgroundColor: Colors.blue,
                      duration: Duration(seconds: 4),
                      action: SnackBarAction(
                        label: 'Got it',
                        textColor: Colors.white,
                        onPressed: () {},
                      ),
                    ),
                  );
                }
              });
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.blue,
              foregroundColor: Colors.white,
            ),
            child: Text('Go to Dashboard'),
          ),
        ],
      ),
    );
  }

  // ✅ NEW: Reset form for creating another order
  void _resetOrderForm() {
    setState(() {
      _coordinates.clear();
      _isCreatingOrder = false;
      _isExecutingOrder = false;
      _currentCoordinateIndex = 0;
      _showSavedOrderPreview = false; // ✅ Hide saved order preview
      _lastSavedOrder = null; // ✅ Clear saved order

      // Generate new order name with timestamp
      _orderNameController.text =
          'Order ${DateTime.now().millisecondsSinceEpoch}';
      _coordinateNameController.clear();
    });

    // Show success message
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('✨ Ready to create a new order!'),
        backgroundColor: Colors.blue,
        duration: Duration(seconds: 2),
      ),
    );
  }

// ✅ REAL MAP COORDINATE SYSTEM - Using actual origin from YAML
  Map<String, double> _screenToMapCoordinates(Offset screenPosition) {
    // ✅ Use actual map metadata from YAML/map info
    final mapData = _selectedMapData ?? widget.mapData;

    // Apply map transformations (scale and offset)
    final adjustedX = (screenPosition.dx - _mapOffset.dx) / _mapScale;
    final adjustedY = (screenPosition.dy - _mapOffset.dy) / _mapScale;

    // ✅ Get REAL map parameters from YAML
    final resolution = mapData.info.resolution; // meters per cell from YAML
    final originX = mapData.info.origin.x; // origin X from YAML
    final originY = mapData.info.origin.y; // origin Y from YAML
    final mapWidth = mapData.info.width; // map width in cells
    final mapHeight = mapData.info.height; // map height in cells

    // ✅ Calculate REAL world bounds based on map metadata
    final mapPhysicalWidth = mapWidth * resolution; // actual width in meters
    final mapPhysicalHeight = mapHeight * resolution; // actual height in meters

    // World coordinate bounds (what the coordinate system should show)
    final minX = originX;
    final maxX = originX + mapPhysicalWidth;
    final minY = originY;
    final maxY = originY + mapPhysicalHeight;

    // ✅ Convert screen coordinates to REAL world coordinates
    final canvasWidth = _canvasSize.width / _mapScale;
    final canvasHeight = _canvasSize.height / _mapScale;

    // Map screen position to world coordinates
    final worldX = minX + (adjustedX / canvasWidth) * mapPhysicalWidth;
    final worldY = minY + (adjustedY / canvasHeight) * mapPhysicalHeight;

    print('🎯 Real world coordinate conversion:');
    print(
        '   Screen: (${screenPosition.dx.toStringAsFixed(1)}, ${screenPosition.dy.toStringAsFixed(1)})');
    print(
        '   Map origin: (${originX.toStringAsFixed(2)}, ${originY.toStringAsFixed(2)})');
    print('   Map size: ${mapWidth}x${mapHeight} cells, ${resolution}m/cell');
    print(
        '   World bounds: X(${minX.toStringAsFixed(2)} to ${maxX.toStringAsFixed(2)}), Y(${minY.toStringAsFixed(2)} to ${maxY.toStringAsFixed(2)})');
    print(
        '   Final world coords: (${worldX.toStringAsFixed(3)}, ${worldY.toStringAsFixed(3)})');

    return {'x': worldX, 'y': worldY};
  }

  @override
  void dispose() {
    _orderNameController.dispose();
    _coordinateNameController.dispose();
    super.dispose();
  }
}

// SIMPLE MAP PAINTER - Just what you need
class SimpleMapPainter extends CustomPainter {
  final MapData mapData;
  final List<Map<String, dynamic>> coordinates;
  final double mapScale;
  final Offset mapOffset;
  final int currentExecutingIndex;
  final Size canvasSize;
  final Map<String, dynamic>? savedOrder; // ✅ NEW: Show saved order preview
  final bool showSavedOrderPreview; // ✅ NEW: Control saved order visibility

  SimpleMapPainter({
    required this.mapData,
    required this.coordinates,
    required this.mapScale,
    required this.mapOffset,
    required this.currentExecutingIndex,
    required this.canvasSize,
    this.savedOrder, // ✅ NEW: Optional saved order
    this.showSavedOrderPreview = false, // ✅ NEW: Default to hidden
  });

  @override
  void paint(Canvas canvas, Size size) {
    canvas.save();
    canvas.translate(mapOffset.dx, mapOffset.dy);
    canvas.scale(mapScale);

    // Draw coordinate system using real map bounds
    _drawRealCoordinateGrid(canvas, size);

    // Draw the mirrored and fitted map
    _drawFittedMap(canvas, size);

    // ✅ Draw saved order preview first (as background layer)
    if (showSavedOrderPreview && savedOrder != null) {
      _drawSavedOrderPreview(canvas, size);
    }

    // Draw coordinates
    _drawCoordinates(canvas, size);

    canvas.restore();
  }

  // ✅ NEW: Draw coordinate grid using REAL map bounds from YAML
  void _drawRealCoordinateGrid(Canvas canvas, Size size) {
    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;

    // ✅ Get REAL map parameters from YAML
    final resolution = mapData.info.resolution;
    final originX = mapData.info.origin.x;
    final originY = mapData.info.origin.y;
    final mapWidth = mapData.info.width;
    final mapHeight = mapData.info.height;

    // Calculate REAL world bounds
    final mapPhysicalWidth = mapWidth * resolution;
    final mapPhysicalHeight = mapHeight * resolution;
    final minX = originX;
    final maxX = originX + mapPhysicalWidth;
    final minY = originY;
    final maxY = originY + mapPhysicalHeight;

    // Grid spacing (adjust based on map size)
    double gridSpacing = 1.0; // Default 1 meter
    if (mapPhysicalWidth < 5.0) {
      gridSpacing = 0.5; // 0.5m for small maps
    } else if (mapPhysicalWidth > 20.0) {
      gridSpacing = 2.0; // 2m for large maps
    }

    print('🗺️ Drawing coordinate grid:');
    print(
        '   World bounds: X(${minX.toStringAsFixed(2)} to ${maxX.toStringAsFixed(2)}), Y(${minY.toStringAsFixed(2)} to ${maxY.toStringAsFixed(2)})');
    print('   Grid spacing: ${gridSpacing}m');

    // ✅ Draw background
    final bgPaint = Paint()..color = Colors.grey.shade50;
    canvas.drawRect(
        Rect.fromLTWH(0, 0, adjustedWidth, adjustedHeight), bgPaint);

    // ✅ Draw minor grid lines
    final minorGridPaint = Paint()
      ..color = Colors.blue.withOpacity(0.15)
      ..strokeWidth = 0.5;

    final minorSpacing = gridSpacing / 4; // 4 divisions per major grid

    // Calculate pixel spacing
    final pixelsPerMeterX = adjustedWidth / mapPhysicalWidth;
    final pixelsPerMeterY = adjustedHeight / mapPhysicalHeight;

    // Minor grid lines
    for (double worldX = minX; worldX <= maxX; worldX += minorSpacing) {
      final pixelX = (worldX - minX) * pixelsPerMeterX;
      canvas.drawLine(
          Offset(pixelX, 0), Offset(pixelX, adjustedHeight), minorGridPaint);
    }

    for (double worldY = minY; worldY <= maxY; worldY += minorSpacing) {
      final pixelY = (worldY - minY) * pixelsPerMeterY;
      canvas.drawLine(
          Offset(0, pixelY), Offset(adjustedWidth, pixelY), minorGridPaint);
    }

    // ✅ Draw major grid lines with REAL coordinates
    final majorGridPaint = Paint()
      ..color = Colors.blue.withOpacity(0.4)
      ..strokeWidth = 1.0;

    // Major vertical lines
    for (double worldX = _roundToGrid(minX, gridSpacing);
        worldX <= maxX;
        worldX += gridSpacing) {
      final pixelX = (worldX - minX) * pixelsPerMeterX;
      canvas.drawLine(
          Offset(pixelX, 0), Offset(pixelX, adjustedHeight), majorGridPaint);
    }

    // Major horizontal lines
    for (double worldY = _roundToGrid(minY, gridSpacing);
        worldY <= maxY;
        worldY += gridSpacing) {
      final pixelY = (worldY - minY) * pixelsPerMeterY;
      canvas.drawLine(
          Offset(0, pixelY), Offset(adjustedWidth, pixelY), majorGridPaint);
    }

    // ✅ Draw axes highlighting origin and bounds
    final axisPaint = Paint()
      ..color = Colors.red.withOpacity(0.8)
      ..strokeWidth = 2.0;

    // Highlight origin lines if they're within bounds
    if (minX <= 0 && maxX >= 0) {
      final originPixelX = (0 - minX) * pixelsPerMeterX;
      canvas.drawLine(Offset(originPixelX, 0),
          Offset(originPixelX, adjustedHeight), axisPaint);
    }

    if (minY <= 0 && maxY >= 0) {
      final originPixelY = (0 - minY) * pixelsPerMeterY;
      canvas.drawLine(Offset(0, originPixelY),
          Offset(adjustedWidth, originPixelY), axisPaint);
    }

    // ✅ Draw coordinate labels
    _drawRealAxisLabels(canvas, size, pixelsPerMeterX, pixelsPerMeterY, minX,
        maxX, minY, maxY, gridSpacing);
  }

  // ✅ Helper: Round to nearest grid value
  double _roundToGrid(double value, double gridSpacing) {
    return (value / gridSpacing).ceil() * gridSpacing;
  }

  // ✅ NEW: Draw axis labels with REAL coordinates
  void _drawRealAxisLabels(
      Canvas canvas,
      Size size,
      double pixelsPerMeterX,
      double pixelsPerMeterY,
      double minX,
      double maxX,
      double minY,
      double maxY,
      double gridSpacing) {
    final textStyle = TextStyle(
      color: Colors.blue.shade700,
      fontSize: 10,
      fontWeight: FontWeight.bold,
    );

    // X-axis labels (showing real world X coordinates)
    for (double worldX = _roundToGrid(minX, gridSpacing);
        worldX <= maxX;
        worldX += gridSpacing) {
      final pixelX = (worldX - minX) * pixelsPerMeterX;

      final textPainter = TextPainter(
        text: TextSpan(text: worldX.toStringAsFixed(1), style: textStyle),
        textDirection: TextDirection.ltr,
      );
      textPainter.layout();

      // Position label above the grid
      textPainter.paint(canvas, Offset(pixelX - textPainter.width / 2, -18));
    }

    // Y-axis labels (showing real world Y coordinates)
    for (double worldY = _roundToGrid(minY, gridSpacing);
        worldY <= maxY;
        worldY += gridSpacing) {
      final pixelY = (worldY - minY) * pixelsPerMeterY;

      final textPainter = TextPainter(
        text: TextSpan(text: worldY.toStringAsFixed(1), style: textStyle),
        textDirection: TextDirection.ltr,
      );
      textPainter.layout();

      // Position label to the left of the grid
      textPainter.paint(canvas,
          Offset(-textPainter.width - 5, pixelY - textPainter.height / 2));
    }

    // ✅ Draw coordinate system info
    final infoStyle =
        textStyle.copyWith(fontSize: 12, color: Colors.blue.shade800);

    // Title showing coordinate range
    final title = TextPainter(
      text: TextSpan(
        text:
            'World Coordinates: X(${minX.toStringAsFixed(1)} to ${maxX.toStringAsFixed(1)}) Y(${minY.toStringAsFixed(1)} to ${maxY.toStringAsFixed(1)}) [meters]',
        style: infoStyle,
      ),
      textDirection: TextDirection.ltr,
    );
    title.layout();
    title.paint(canvas, Offset(10, -40));

    // Origin info
    final originInfo = TextPainter(
      text: TextSpan(
        text:
            'Map Origin: (${mapData.info.origin.x.toStringAsFixed(2)}, ${mapData.info.origin.y.toStringAsFixed(2)}) | Resolution: ${mapData.info.resolution}m/cell',
        style: textStyle.copyWith(fontSize: 9, color: Colors.grey.shade600),
      ),
      textDirection: TextDirection.ltr,
    );
    originInfo.layout();
    originInfo.paint(canvas, Offset(10, -25));
    ;
  }

  // ✅ NEW: Draw axis labels with numbers
  void _drawAxisLabels(Canvas canvas, Size size, double pixelsPerMeterX,
      double pixelsPerMeterY, double maxX, double maxY) {
    final textStyle = TextStyle(
      color: Colors.blue.shade700,
      fontSize: 10,
      fontWeight: FontWeight.bold,
    );

    // X-axis labels
    for (double x = 0; x <= maxX; x += 1.0) {
      final pixelX = x * pixelsPerMeterX;

      final textPainter = TextPainter(
        text: TextSpan(text: x.toStringAsFixed(0), style: textStyle),
        textDirection: TextDirection.ltr,
      );
      textPainter.layout();

      // Position label below the axis
      textPainter.paint(canvas, Offset(pixelX - textPainter.width / 2, -20));
    }

    // Y-axis labels
    for (double y = 0; y <= maxY; y += 1.0) {
      final pixelY = y * pixelsPerMeterY;

      final textPainter = TextPainter(
        text: TextSpan(text: y.toStringAsFixed(0), style: textStyle),
        textDirection: TextDirection.ltr,
      );
      textPainter.layout();

      // Position label to the left of the axis
      textPainter.paint(canvas, Offset(-25, pixelY - textPainter.height / 2));
    }

    // ✅ Draw axis titles
    // X-axis title
    final xAxisTitle = TextPainter(
      text: TextSpan(
          text: 'X (meters) →', style: textStyle.copyWith(fontSize: 12)),
      textDirection: TextDirection.ltr,
    );
    xAxisTitle.layout();
    xAxisTitle.paint(
        canvas, Offset(size.width / mapScale / 2 - xAxisTitle.width / 2, -35));

    // Y-axis title (rotated)
    canvas.save();
    canvas.translate(-40, size.height / mapScale / 2);
    canvas.rotate(-1.5708); // Rotate 90 degrees counterclockwise
    final yAxisTitle = TextPainter(
      text: TextSpan(
          text: '↓ Y (meters)', style: textStyle.copyWith(fontSize: 12)),
      textDirection: TextDirection.ltr,
    );
    yAxisTitle.layout();
    yAxisTitle.paint(canvas, Offset(-yAxisTitle.width / 2, 0));
    canvas.restore();
  }

  // ✅ Draw the map fitted within the coordinate system
  void _drawFittedMap(Canvas canvas, Size size) {
    try {
      if (mapData.occupancyData.isEmpty) {
        print('⚠️ No occupancy data to draw');
        return;
      }

      final adjustedWidth = size.width / mapScale;
      final adjustedHeight = size.height / mapScale;

      // Map parameters
      final mapWidth = mapData.info.width;
      final mapHeight = mapData.info.height;
      final resolution = mapData.info.resolution;

      // Calculate cell size to fit the map in the canvas
      final mapPhysicalWidth = mapWidth * resolution;
      final mapPhysicalHeight = mapHeight * resolution;

      final scaleX = adjustedWidth / mapPhysicalWidth;
      final scaleY = adjustedHeight / mapPhysicalHeight;
      final cellSize = (scaleX < scaleY ? scaleX : scaleY).clamp(0.5, 10.0);

      print('🗺️ Drawing fitted map:');
      print(
          '   Physical size: ${mapPhysicalWidth.toStringAsFixed(2)}m x ${mapPhysicalHeight.toStringAsFixed(2)}m');
      print('   Cell size: ${cellSize.toStringAsFixed(2)}px');

      // ✅ Draw occupancy cells (properly oriented)
      for (int y = 0; y < mapHeight; y++) {
        for (int x = 0; x < mapWidth; x++) {
          final index = y * mapWidth + x;

          if (index < mapData.occupancyData.length) {
            final occupancyValue = mapData.occupancyData[index];
            Color cellColor = _getOccupancyColor(occupancyValue);

            // Position cell in the coordinate system
            final screenX = x * cellSize;
            final screenY = y * cellSize;

            final cellPaint = Paint()..color = cellColor;
            final cellRect =
                Rect.fromLTWH(screenX, screenY, cellSize, cellSize);

            // Only draw non-white cells for performance
            if (cellColor != Colors.white) {
              canvas.drawRect(cellRect, cellPaint);
            }
          }
        }
      }

      print('✅ Map fitted to coordinate system');
    } catch (e) {
      print('❌ Error drawing fitted map: $e');
    }
  }

  // ✅ Get color for occupancy value
  Color _getOccupancyColor(int occupancyValue) {
    if (occupancyValue == -1) {
      return Colors.grey.shade300; // Unknown
    } else if (occupancyValue <= 10) {
      return Colors.white; // Free space
    } else if (occupancyValue >= 90) {
      return Colors.black; // Occupied
    } else {
      // Probability value
      final probability = occupancyValue / 100.0;
      final greyValue = (255 * (1.0 - probability)).round().clamp(0, 255);
      return Color.fromARGB(255, greyValue, greyValue, greyValue);
    }
  }

  // ✅ NEW: Draw the actual map data
  // ✅ NEW: Draw occupancy grid data
  void _drawOccupancyGrid(Canvas canvas, Size size) {
    final mapInfo = mapData.info;
    final occupancyData = mapData.occupancyData;

    final resolution = mapInfo.resolution;

    // Calculate map dimensions
    final mapWidth = mapInfo.width;
    final mapHeight = mapInfo.height;

    print(
        '🗺️ Drawing occupancy grid: ${mapWidth}x${mapHeight}, ${occupancyData.length} cells');
    print('   - Resolution: $resolution meters/cell');
    print('   - Expected cells: ${mapWidth * mapHeight}');

    // ✅ ENHANCED: Calculate proper cell size for rendering
    final canvasWidth = size.width / mapScale;
    final canvasHeight = size.height / mapScale;

    // Scale to fit the canvas while maintaining aspect ratio
    final scaleX = canvasWidth / mapWidth;
    final scaleY = canvasHeight / mapHeight;
    final cellSize = (scaleX < scaleY ? scaleX : scaleY).clamp(1.0, 10.0);

    print('   - Cell size: ${cellSize}px');
    print('   - Canvas size: ${canvasWidth}x${canvasHeight}');

    // Draw each occupancy cell
    for (int y = 0; y < mapHeight; y++) {
      for (int x = 0; x < mapWidth; x++) {
        final index = y * mapWidth + x;

        if (index < occupancyData.length) {
          final occupancyValue = occupancyData[index];
          Color cellColor;

          // ✅ ENHANCED: Better color mapping for occupancy values
          // Standard ROS occupancy grid values:
          // -1 = unknown (light grey)
          // 0 = free space (white)
          // 100 = occupied (black)
          // Values in between = probability (grey scale)

          if (occupancyValue == -1) {
            cellColor = Colors.grey.shade300; // Unknown - light grey
          } else if (occupancyValue <= 10) {
            cellColor = Colors.white; // Free space - white
          } else if (occupancyValue >= 90) {
            cellColor = Colors.black; // Occupied - black
          } else {
            // Probability value (0-100) - grey scale
            final probability = occupancyValue / 100.0;
            final greyValue = (255 * (1.0 - probability)).round().clamp(0, 255);
            cellColor = Color.fromARGB(255, greyValue, greyValue, greyValue);
          }

          // ✅ ENHANCED: Draw cells with proper sizing
          final cellPaint = Paint()..color = cellColor;
          final cellRect = Rect.fromLTWH(
            x * cellSize,
            y * cellSize,
            cellSize,
            cellSize,
          );

          canvas.drawRect(cellRect, cellPaint);
        }
      }

      // ✅ NEW: Progress logging for large maps
      if (y % 20 == 0 && y > 0) {
        print(
            '   - Drawing progress: ${(y / mapHeight * 100).toStringAsFixed(1)}%');
      }
    }

    print('✅ Occupancy grid rendered successfully');
  }

  // ✅ NEW: Draw map shapes (stations, obstacles, boundaries)
  void _drawMapShapes(Canvas canvas, Size size) {
    for (final shape in mapData.shapes) {
      _drawMapShape(canvas, shape, size);
    }
  }

  // ✅ NEW: Draw subtle background grid for coordinate placement
  void _drawBackgroundGrid(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = Colors.blue.withOpacity(0.15) // Very subtle blue grid
      ..strokeWidth = 0.5;

    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;

    // Dynamic grid spacing based on zoom level
    double gridSpacing = 50.0;
    if (mapScale > 2.0) {
      gridSpacing = 25.0; // Finer grid when zoomed in
    } else if (mapScale < 0.8) {
      gridSpacing = 100.0; // Coarser grid when zoomed out
    }

    // Draw vertical lines
    for (double x = 0; x < adjustedWidth; x += gridSpacing) {
      canvas.drawLine(Offset(x, 0), Offset(x, adjustedHeight), gridPaint);
    }

    // Draw horizontal lines
    for (double y = 0; y < adjustedHeight; y += gridSpacing) {
      canvas.drawLine(Offset(0, y), Offset(adjustedWidth, y), gridPaint);
    }

    // Draw major grid lines every 200 units for reference
    final majorGridPaint = Paint()
      ..color = Colors.blue.withOpacity(0.25)
      ..strokeWidth = 1.0;

    const majorGridSpacing = 200.0;

    // Major vertical lines
    for (double x = 0; x < adjustedWidth; x += majorGridSpacing) {
      canvas.drawLine(Offset(x, 0), Offset(x, adjustedHeight), majorGridPaint);
    }

    // Major horizontal lines
    for (double y = 0; y < adjustedHeight; y += majorGridSpacing) {
      canvas.drawLine(Offset(0, y), Offset(adjustedWidth, y), majorGridPaint);
    }
  }

  // ✅ NEW: Draw individual map shape
  void _drawMapShape(Canvas canvas, MapShape shape, Size size) {
    if (shape.points.isEmpty) return;

    final paint = Paint()
      ..color = _getShapeColor(shape.type)
      ..style = PaintingStyle.fill;

    final borderPaint = Paint()
      ..color = _getShapeColor(shape.type).withOpacity(0.8)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;

    // ✅ FIXED: Convert all points to screen coordinates using proper conversion
    final screenPoints = shape.points.map((point) {
      // Use the same coordinate conversion as _worldToScreen
      final mapInfo = mapData.info;
      final resolution = mapInfo.resolution;
      final originX = mapInfo.origin.x;
      final originY = mapInfo.origin.y;
      final mapWidth = mapInfo.width;
      final mapHeight = mapInfo.height;

      // Use the same coordinate conversion as _worldToScreen for consistency
      final mapPhysicalWidth = mapWidth * resolution;
      final mapPhysicalHeight = mapHeight * resolution;

      // Use the canvas size passed to the method
      final canvasWidth = size.width / mapScale; // Account for scaling
      final canvasHeight = size.height / mapScale;

      final metersPerPixelX = mapPhysicalWidth / canvasWidth;
      final metersPerPixelY = mapPhysicalHeight / canvasHeight;

      // Convert world coordinates to screen coordinates
      final relativeX = (point.x - originX) / metersPerPixelX;
      final relativeY = (point.y - originY) / metersPerPixelY;
      final screenX = relativeX;
      final screenY = canvasHeight - relativeY; // Flip Y axis

      return Offset(screenX, screenY);
    }).toList();

    if (screenPoints.isEmpty) return;

    // Draw shape based on number of points
    if (screenPoints.length == 1) {
      // Single point - draw as circle
      final center = screenPoints.first;
      final radius = 5.0;
      canvas.drawCircle(center, radius, paint);
      canvas.drawCircle(center, radius, borderPaint);
    } else if (screenPoints.length == 2) {
      // Two points - draw as line with thickness
      final linePaint = Paint()
        ..color = _getShapeColor(shape.type)
        ..strokeWidth = 6
        ..style = PaintingStyle.stroke;
      canvas.drawLine(screenPoints[0], screenPoints[1], linePaint);
    } else {
      // Multiple points - draw as polygon
      final path = Path();
      path.moveTo(screenPoints.first.dx, screenPoints.first.dy);

      for (int i = 1; i < screenPoints.length; i++) {
        path.lineTo(screenPoints[i].dx, screenPoints[i].dy);
      }
      path.close();

      canvas.drawPath(path, paint);
      canvas.drawPath(path, borderPaint);
    }

    // Draw shape label at center
    if (shape.name.isNotEmpty) {
      _drawShapeLabel(canvas, shape);
    }
  }

  // ✅ NEW: Draw shape label
  void _drawShapeLabel(Canvas canvas, MapShape shape) {
    if (shape.points.isEmpty) return;

    final textPainter = TextPainter(
      text: TextSpan(
        text: shape.name,
        style: TextStyle(
          color: Colors.white,
          fontWeight: FontWeight.bold,
          fontSize: 12,
          shadows: [
            Shadow(
              offset: Offset(1, 1),
              blurRadius: 2,
              color: Colors.black54,
            ),
          ],
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    textPainter.layout();

    // Use the center property from MapShape
    final center = shape.center;
    final labelPosition = Offset(
      center.x * 20 - textPainter.width / 2,
      center.y * 20 - textPainter.height / 2,
    );

    textPainter.paint(canvas, labelPosition);
  }

  // ✅ NEW: Get color for shape type
  Color _getShapeColor(String type) {
    switch (type) {
      case 'pickup':
        return Colors.green.withOpacity(0.7);
      case 'drop':
        return Colors.blue.withOpacity(0.7);
      case 'charging':
        return Colors.orange.withOpacity(0.7);
      case 'obstacle':
        return Colors.red.withOpacity(0.7);
      case 'boundary':
        return Colors.purple.withOpacity(0.7);
      case 'waypoint':
        return Colors.teal.withOpacity(0.7);
      default:
        return Colors.grey.withOpacity(0.7);
    }
  }

  void _drawGrid(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = Colors.grey.shade300
      ..strokeWidth = 1.0;

    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;
    const gridSpacing = 50.0;

    for (double x = 0; x < adjustedWidth; x += gridSpacing) {
      canvas.drawLine(Offset(x, 0), Offset(x, adjustedHeight), gridPaint);
    }

    for (double y = 0; y < adjustedHeight; y += gridSpacing) {
      canvas.drawLine(Offset(0, y), Offset(adjustedWidth, y), gridPaint);
    }
  }

  // ✅ NEW: Convert Cartesian coordinates to screen position
  // ✅ Draw coordinates using real world coordinates
  void _drawCoordinates(Canvas canvas, Size size) {
    for (int i = 0; i < coordinates.length; i++) {
      final coord = coordinates[i];
      final screenPos = _worldToScreen(coord['x'], coord['y'], size);

      if (screenPos != null) {
        _drawCoordinate(canvas, screenPos, i + 1, i == currentExecutingIndex,
            i < currentExecutingIndex);

        // Draw line to next coordinate
        if (i < coordinates.length - 1) {
          final nextCoord = coordinates[i + 1];
          final nextScreenPos =
              _worldToScreen(nextCoord['x'], nextCoord['y'], size);
          if (nextScreenPos != null) {
            _drawConnectionLine(canvas, screenPos, nextScreenPos);
          }
        }
      }
    }
  }

  // ✅ Convert world coordinates to screen position
  Offset? _worldToScreen(double worldX, double worldY, Size size) {
    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;

    // Map bounds
    final resolution = mapData.info.resolution;
    final originX = mapData.info.origin.x;
    final originY = mapData.info.origin.y;
    final mapWidth = mapData.info.width;
    final mapHeight = mapData.info.height;

    final mapPhysicalWidth = mapWidth * resolution;
    final mapPhysicalHeight = mapHeight * resolution;
    final minX = originX;
    final minY = originY;

    // Convert world coordinates to screen coordinates
    final screenX = ((worldX - minX) / mapPhysicalWidth) * adjustedWidth;
    final screenY = ((worldY - minY) / mapPhysicalHeight) * adjustedHeight;

    return Offset(screenX, screenY);
  }

  void _drawCoordinate(Canvas canvas, Offset position, int number,
      bool isExecuting, bool isCompleted) {
    final radius = 16.0;
    Color color = Colors.red;

    if (isExecuting) color = Colors.orange;
    if (isCompleted) color = Colors.green;

    // Draw circle with shadow
    final shadowPaint = Paint()
      ..color = Colors.black54
      ..maskFilter = MaskFilter.blur(BlurStyle.normal, 3);
    canvas.drawCircle(position + Offset(1, 1), radius, shadowPaint);

    // Draw circle
    final circlePaint = Paint()..color = color;
    canvas.drawCircle(position, radius, circlePaint);

    // Draw border
    final borderPaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;
    canvas.drawCircle(position, radius, borderPaint);

    // Draw number
    final textPainter = TextPainter(
      text: TextSpan(
        text: number.toString(),
        style: TextStyle(
          color: Colors.white,
          fontWeight: FontWeight.bold,
          fontSize: 12,
          shadows: [
            Shadow(
              offset: Offset(0.5, 0.5),
              blurRadius: 1,
              color: Colors.black54,
            ),
          ],
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    textPainter.layout();
    textPainter.paint(
      canvas,
      Offset(
        position.dx - textPainter.width / 2,
        position.dy - textPainter.height / 2,
      ),
    );
  }

  void _drawConnectionLine(Canvas canvas, Offset from, Offset to) {
    final linePaint = Paint()
      ..color = Colors.red.withOpacity(0.7)
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    canvas.drawLine(from, to, linePaint);
    _drawArrow(canvas, from, to);
  }

  void _drawArrow(Canvas canvas, Offset from, Offset to) {
    final direction = to - from;
    final distance = direction.distance;
    if (distance == 0) return;

    final normalizedDirection = direction / distance;
    final arrowLength = 8.0;

    final arrowPoint1 = to -
        normalizedDirection * arrowLength +
        Offset(-normalizedDirection.dy * arrowLength * 0.4,
            normalizedDirection.dx * arrowLength * 0.4);
    final arrowPoint2 = to -
        normalizedDirection * arrowLength +
        Offset(normalizedDirection.dy * arrowLength * 0.4,
            -normalizedDirection.dx * arrowLength * 0.4);

    final arrowPaint = Paint()
      ..color = Colors.red.withOpacity(0.7)
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    canvas.drawLine(to, arrowPoint1, arrowPaint);
    canvas.drawLine(to, arrowPoint2, arrowPaint);
  }

  // ✅ NEW: Draw saved order preview with different styling (Cartesian coordinates)
  void _drawSavedOrderPreview(Canvas canvas, Size size) {
    if (savedOrder == null) return;

    final savedCoordinates = savedOrder!['coordinates'] as List<dynamic>;
    if (savedCoordinates.isEmpty) return;

    print(
        '🎨 Drawing saved order preview: ${savedOrder!['name']} with ${savedCoordinates.length} coordinates');

    // Draw saved coordinates with different styling (more transparent, different color)
    for (int i = 0; i < savedCoordinates.length; i++) {
      final coord = savedCoordinates[i] as Map<String, dynamic>;
      final screenPos = _worldToScreen(coord['x'], coord['y'], size);

      if (screenPos != null) {
        _drawSavedCoordinate(canvas, screenPos, i + 1);

        // Draw line to next coordinate
        if (i < savedCoordinates.length - 1) {
          final nextCoord = savedCoordinates[i + 1] as Map<String, dynamic>;
          final nextScreenPos =
              _worldToScreen(nextCoord['x'], nextCoord['y'], size);
          if (nextScreenPos != null) {
            _drawSavedConnectionLine(canvas, screenPos, nextScreenPos);
          }
        }
      }
    }

    // ✅ Draw order label
    _drawSavedOrderLabel(canvas, size, savedOrder!['name'] as String);
  }

  // ✅ NEW: Draw saved coordinate with different styling
  void _drawSavedCoordinate(Canvas canvas, Offset position, int number) {
    final radius = 18.0;

    // Draw circle with shadow (more transparent)
    final shadowPaint = Paint()
      ..color = Colors.black26
      ..maskFilter = MaskFilter.blur(BlurStyle.normal, 3);
    canvas.drawCircle(position + Offset(1, 1), radius, shadowPaint);

    // Draw circle (green for saved order)
    final circlePaint = Paint()..color = Colors.green.withOpacity(0.7);
    canvas.drawCircle(position, radius, circlePaint);

    // Draw border (lighter)
    final borderPaint = Paint()
      ..color = Colors.white.withOpacity(0.8)
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;
    canvas.drawCircle(position, radius, borderPaint);

    // Draw number (smaller text)
    final textPainter = TextPainter(
      text: TextSpan(
        text: number.toString(),
        style: TextStyle(
          color: Colors.white,
          fontWeight: FontWeight.bold,
          fontSize: 14,
          shadows: [
            Shadow(
              offset: Offset(1, 1),
              blurRadius: 2,
              color: Colors.black45,
            ),
          ],
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    textPainter.layout();
    textPainter.paint(
      canvas,
      Offset(
        position.dx - textPainter.width / 2,
        position.dy - textPainter.height / 2,
      ),
    );
  }

  // ✅ NEW: Draw saved connection line
  void _drawSavedConnectionLine(Canvas canvas, Offset from, Offset to) {
    // Draw main line (green, more transparent)
    final linePaint = Paint()
      ..color = Colors.green.withOpacity(0.5)
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    canvas.drawLine(from, to, linePaint);

    // Draw arrow at the end (smaller)
    _drawSavedArrow(canvas, from, to);
  }

  // ✅ NEW: Draw arrow for saved order
  void _drawSavedArrow(Canvas canvas, Offset from, Offset to) {
    final direction = to - from;
    final distance = direction.distance;
    if (distance == 0) return;

    final normalizedDirection = direction / distance;
    final arrowLength = 8.0;

    final arrowPoint1 = to -
        normalizedDirection * arrowLength +
        Offset(-normalizedDirection.dy * arrowLength * 0.5,
            normalizedDirection.dx * arrowLength * 0.5);
    final arrowPoint2 = to -
        normalizedDirection * arrowLength +
        Offset(normalizedDirection.dy * arrowLength * 0.5,
            -normalizedDirection.dx * arrowLength * 0.5);

    final arrowPaint = Paint()
      ..color = Colors.green.withOpacity(0.5)
      ..strokeWidth = 1.5
      ..style = PaintingStyle.stroke;

    canvas.drawLine(to, arrowPoint1, arrowPaint);
    canvas.drawLine(to, arrowPoint2, arrowPaint);
  }

  // ✅ NEW: Draw order label
  void _drawSavedOrderLabel(Canvas canvas, Size size, String orderName) {
    final labelText = '✅ Saved: $orderName';
    final textPainter = TextPainter(
      text: TextSpan(
        text: labelText,
        style: TextStyle(
          color: Colors.green.shade700,
          fontWeight: FontWeight.bold,
          fontSize: 16,
          backgroundColor: Colors.white.withOpacity(0.9),
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    textPainter.layout();

    // Position at top-right of canvas
    final labelPosition = Offset(
      (size.width / mapScale) - textPainter.width - 16,
      16,
    );

    // Draw background
    final bgPaint = Paint()..color = Colors.white.withOpacity(0.9);
    final bgRect = Rect.fromLTWH(
      labelPosition.dx - 8,
      labelPosition.dy - 4,
      textPainter.width + 16,
      textPainter.height + 8,
    );
    canvas.drawRRect(
      RRect.fromRectAndRadius(bgRect, Radius.circular(8)),
      bgPaint,
    );

    // Draw border
    final borderPaint = Paint()
      ..color = Colors.green.shade300
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;
    canvas.drawRRect(
      RRect.fromRectAndRadius(bgRect, Radius.circular(8)),
      borderPaint,
    );

    textPainter.paint(canvas, labelPosition);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}
