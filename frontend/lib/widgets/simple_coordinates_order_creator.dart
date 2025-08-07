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

  // SIMPLE STATE - Just what you need
  List<Map<String, dynamic>> _coordinates = [];
  bool _isCreatingOrder = false;
  bool _isExecutingOrder = false;
  int _currentCoordinateIndex = 0;

  // ✅ NEW: Map selection functionality
  List<Map<String, dynamic>> _availableMaps = [];
  String? _selectedMapName;
  MapData? _selectedMapData;
  bool _isLoadingMaps = false;
  bool _isLoadingMapData = false;

  // Map interaction
  double _mapScale = 1.0;
  Offset _mapOffset = Offset.zero;

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

    // ✅ NEW: Load available maps for the device
    _loadAvailableMaps();

    // ✅ NEW: Initialize with the provided map data as default
    _selectedMapData = widget.mapData;
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
        print(
            '   - Occupancy data type: ${rawMapData['occupancyData']?.runtimeType}');
        print(
            '   - Occupancy data length: ${rawMapData['occupancyData']?.length ?? 0}');

        // ✅ ENHANCED: Better data parsing with more fallbacks
        final mapInfo = rawMapData['info'] ?? {};
        final width =
            mapInfo['width'] ?? 164; // Use the size from your screenshot
        final height = mapInfo['height'] ?? 145;
        final resolution = mapInfo['resolution'] ?? 0.05;
        final origin = mapInfo['origin'] ?? [-25.0, -25.0];

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
        print('   - Resolution: $resolution');
        print('   - Origin: $origin');
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
                x: origin is List && origin.isNotEmpty
                    ? origin[0].toDouble()
                    : -25.0,
                y: origin is List && origin.length > 1
                    ? origin[1].toDouble()
                    : -25.0,
                z: 0.0,
              ),
              originOrientation: odom.Orientation(x: 0, y: 0, z: 0, w: 1),
            ),
            occupancyData: occupancyData,
            shapes: [], // Will be populated if needed
            version: 1,
          );
        });

        print('✅ Map data loaded and set for: $mapName');
      } else {
        throw Exception(
            'Invalid map data response: ${mapData['error'] ?? 'Unknown error'}');
      }
    } catch (e) {
      print('❌ Error loading map data: $e');
      print('📍 Stack trace: ${StackTrace.current}');

      // Fallback to the provided mapData
      setState(() {
        _selectedMapData = widget.mapData;
      });

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Failed to load map data, using default'),
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
            '(${coord['x'].toStringAsFixed(2)}, ${coord['y'].toStringAsFixed(2)})'),
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
                'Position: (${coordinates['x']!.toStringAsFixed(2)}, ${coordinates['y']!.toStringAsFixed(2)})'),
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

        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
              content: Text('✅ Order saved!'), backgroundColor: Colors.green),
        );

        Navigator.of(context).pop();
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

  Map<String, double> _screenToMapCoordinates(Offset screenPosition) {
    // ✅ IMPROVED: Use selected map data for accurate coordinate conversion
    final mapData = _selectedMapData ?? widget.mapData;

    // Apply map transformations (scale and offset)
    final adjustedX = (screenPosition.dx - _mapOffset.dx) / _mapScale;
    final adjustedY = (screenPosition.dy - _mapOffset.dy) / _mapScale;

    // Convert screen coordinates to map coordinates using the map's resolution and origin
    final resolution = mapData.info.resolution;
    final originX = mapData.info.origin.x;
    final originY = mapData.info.origin.y;

    // Convert pixel coordinates to world coordinates (meters)
    // Note: We're using a 20:1 pixel to meter ratio in the painter
    final worldX = originX + (adjustedX / 20.0) * resolution;
    final worldY = originY + (adjustedY / 20.0) * resolution;

    print('🎯 Screen to world conversion:');
    print(
        '   Screen: (${screenPosition.dx.toStringAsFixed(1)}, ${screenPosition.dy.toStringAsFixed(1)})');
    print(
        '   Adjusted: (${adjustedX.toStringAsFixed(1)}, ${adjustedY.toStringAsFixed(1)})');
    print(
        '   World: (${worldX.toStringAsFixed(3)}, ${worldY.toStringAsFixed(3)})');
    print('   Resolution: ${resolution}, Origin: (${originX}, ${originY})');

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

  SimpleMapPainter({
    required this.mapData,
    required this.coordinates,
    required this.mapScale,
    required this.mapOffset,
    required this.currentExecutingIndex,
    required this.canvasSize,
  });

  @override
  void paint(Canvas canvas, Size size) {
    canvas.save();
    canvas.translate(mapOffset.dx, mapOffset.dy);
    canvas.scale(mapScale);

    // ✅ ENHANCED: Draw actual map instead of just background
    _drawMap(canvas, size);

    // Draw coordinates
    _drawCoordinates(canvas, size);

    canvas.restore();
  }

  // ✅ NEW: Draw the actual map data
  void _drawMap(Canvas canvas, Size size) {
    try {
      // Draw map background
      final bgPaint = Paint()..color = Colors.white;
      canvas.drawRect(
          Rect.fromLTWH(0, 0, size.width / mapScale, size.height / mapScale),
          bgPaint);

      print(
          '🗺️ Drawing map: ${mapData.info.width}x${mapData.info.height}, occupancy: ${mapData.occupancyData.length} cells');

      // ✅ ENHANCED: Always try to draw occupancy grid first
      if (mapData.occupancyData.isNotEmpty &&
          mapData.info.width > 0 &&
          mapData.info.height > 0) {
        print('✅ Drawing occupancy grid data...');
        _drawOccupancyGrid(canvas, size);
      } else {
        print('⚠️ No occupancy data available, drawing grid fallback');
        print('   - Occupancy data length: ${mapData.occupancyData.length}');
        print(
            '   - Map dimensions: ${mapData.info.width}x${mapData.info.height}');
        // Fallback to grid when no occupancy data
        _drawGrid(canvas, size);
      }

      // ✅ NEW: Draw background grid overlay for coordinate placement
      _drawBackgroundGrid(canvas, size);

      // ✅ NEW: Draw map shapes (stations, obstacles, etc.)
      _drawMapShapes(canvas, size);
    } catch (e) {
      print('❌ Error drawing map: $e');
      print('📍 Stack trace: ${StackTrace.current}');
      // Fallback to simple grid
      _drawGrid(canvas, size);
    }
  }

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
      _drawMapShape(canvas, shape);
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
  void _drawMapShape(Canvas canvas, MapShape shape) {
    if (shape.points.isEmpty) return;

    final paint = Paint()
      ..color = _getShapeColor(shape.type)
      ..style = PaintingStyle.fill;

    final borderPaint = Paint()
      ..color = _getShapeColor(shape.type).withOpacity(0.8)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;

    // Convert all points to screen coordinates
    final screenPoints = shape.points.map((point) {
      return Offset(point.x * 20, point.y * 20); // Convert meters to pixels
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

  void _drawCoordinate(Canvas canvas, Offset position, int number,
      bool isExecuting, bool isCompleted) {
    final radius = 20.0;
    Color color = Colors.blue;

    if (isExecuting) color = Colors.orange;
    if (isCompleted) color = Colors.green;

    // Draw circle with shadow for better visibility on map
    final shadowPaint = Paint()
      ..color = Colors.black54
      ..maskFilter = MaskFilter.blur(BlurStyle.normal, 4);
    canvas.drawCircle(position + Offset(2, 2), radius, shadowPaint);

    // Draw circle
    final circlePaint = Paint()..color = color;
    canvas.drawCircle(position, radius, circlePaint);

    // Draw border
    final borderPaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 3
      ..style = PaintingStyle.stroke;
    canvas.drawCircle(position, radius, borderPaint);

    // Draw number
    final textPainter = TextPainter(
      text: TextSpan(
        text: number.toString(),
        style: TextStyle(
          color: Colors.white,
          fontWeight: FontWeight.bold,
          fontSize: 16,
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
    textPainter.paint(
      canvas,
      Offset(
        position.dx - textPainter.width / 2,
        position.dy - textPainter.height / 2,
      ),
    );
  }

  void _drawConnectionLine(Canvas canvas, Offset from, Offset to) {
    // Draw shadow line
    final shadowPaint = Paint()
      ..color = Colors.black26
      ..strokeWidth = 5
      ..style = PaintingStyle.stroke;
    canvas.drawLine(from + Offset(1, 1), to + Offset(1, 1), shadowPaint);

    // Draw main line
    final linePaint = Paint()
      ..color = Colors.blue.withOpacity(0.8)
      ..strokeWidth = 3
      ..style = PaintingStyle.stroke;

    canvas.drawLine(from, to, linePaint);

    // Draw arrow at the end
    _drawArrow(canvas, from, to);
  }

  // ✅ NEW: Draw arrow to show direction
  void _drawArrow(Canvas canvas, Offset from, Offset to) {
    final direction = to - from;
    final distance = direction.distance;
    if (distance == 0) return;

    final normalizedDirection = direction / distance;
    final arrowLength = 10.0;

    final arrowPoint1 = to -
        normalizedDirection * arrowLength +
        Offset(-normalizedDirection.dy * arrowLength * 0.5,
            normalizedDirection.dx * arrowLength * 0.5);
    final arrowPoint2 = to -
        normalizedDirection * arrowLength +
        Offset(normalizedDirection.dy * arrowLength * 0.5,
            -normalizedDirection.dx * arrowLength * 0.5);

    final arrowPaint = Paint()
      ..color = Colors.blue.withOpacity(0.8)
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    canvas.drawLine(to, arrowPoint1, arrowPaint);
    canvas.drawLine(to, arrowPoint2, arrowPaint);
  }

  Offset? _worldToScreen(double worldX, double worldY, Size size) {
    // ✅ IMPROVED: Use actual map coordinate conversion
    final mapInfo = mapData.info;

    // Convert world coordinates to map pixel coordinates
    final mapPixelX = (worldX - mapInfo.origin.x) / mapInfo.resolution;
    final mapPixelY = (worldY - mapInfo.origin.y) / mapInfo.resolution;

    // Scale to screen coordinates (using 20:1 pixel to meter ratio)
    final screenX = mapPixelX * 20;
    final screenY = mapPixelY * 20;

    return Offset(screenX, screenY);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}
