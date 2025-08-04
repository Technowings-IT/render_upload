// widgets/interactive_map_order_creator.dart - Interactive Map-based Order Creation
import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../models/map_data.dart';
import '../services/api_service.dart';

class InteractiveMapOrderCreator extends StatefulWidget {
  final String deviceId;
  final MapData mapData;
  final Function(Map<String, dynamic>) onOrderCreated;

  const InteractiveMapOrderCreator({
    Key? key,
    required this.deviceId,
    required this.mapData,
    required this.onOrderCreated,
  }) : super(key: key);

  @override
  State<InteractiveMapOrderCreator> createState() =>
      _InteractiveMapOrderCreatorState();
}

class _InteractiveMapOrderCreatorState extends State<InteractiveMapOrderCreator>
    with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();

  // Animation controllers
  late AnimationController _waypointAnimationController;
  late AnimationController _pulseAnimationController;

  // Order creation state
  List<Map<String, dynamic>> _waypoints = [];
  bool _isCreatingOrder = false;

  // Map interaction state
  Offset? _selectedCoordinate;
  bool _isAddingWaypoint = false;
  String _selectedWaypointType = 'waypoint';

  // Map display settings
  double _mapScale = 1.0;
  Offset _mapOffset = Offset.zero;
  bool _showGrid = true;
  bool _showCoordinates = true;

  // ✅ NEW: Map loading state
  MapData? _loadedMapData;
  bool _isLoadingMap = false;
  String? _mapLoadError;
  String? _mapLoadSource;
  
  // ✅ NEW: Enhanced map visualization
  bool _showMapShapes = true;
  bool _showOccupancyGrid = true;
  double _mapOpacity = 0.7;

  // Form controllers
  final TextEditingController _orderNameController = TextEditingController();
  final TextEditingController _priorityController =
      TextEditingController(text: '0');
  final TextEditingController _waypointNameController = TextEditingController();

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _orderNameController.text =
        'Order ${DateTime.now().millisecondsSinceEpoch}';

    // Debug map data
    print('🗺️ Interactive Map Order Creator initialized');
    print(
        '📊 Map dimensions: ${widget.mapData.info.width}x${widget.mapData.info.height}');
    print('📍 Map shapes: ${widget.mapData.shapes.length}');
    print('🎯 Device ID: ${widget.deviceId}');
    
    // ✅ NEW: Load actual map data for the device
    _loadDeviceMapData();
    
    // ✅ Test layout calculations
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _testLayout();
    });
  }

  // ✅ NEW: Load actual map data from API
  Future<void> _loadDeviceMapData() async {
    setState(() {
      _isLoadingMap = true;
      _mapLoadError = null;
    });

    try {
      print('🔄 Loading actual map data for device: ${widget.deviceId}');
      
      final response = await _apiService.getMapData(widget.deviceId);
      
      if (response['success'] == true && response['mapData'] != null) {
        try {
          final mapData = MapData.fromJson(response['mapData']);
          setState(() {
            _loadedMapData = mapData;
            _mapLoadSource = response['source'] ?? 'api';
            _mapLoadError = null;
          });
          
          print('✅ Loaded actual map data: ${mapData.shapes.length} shapes, ${mapData.info.width}x${mapData.info.height}');
        } catch (parseError) {
          print('❌ Error parsing loaded map data: $parseError');
          setState(() {
            _loadedMapData = widget.mapData; // Fallback to provided map
            _mapLoadSource = 'fallback_provided';
            _mapLoadError = 'Parse error, using provided map';
          });
        }
      } else {
        print('⚠️ No map data found, using provided map');
        setState(() {
          _loadedMapData = widget.mapData; // Use provided map data
          _mapLoadSource = 'provided_default';
          _mapLoadError = null;
        });
      }
    } catch (e) {
      print('❌ Error loading map data: $e');
      setState(() {
        _loadedMapData = widget.mapData; // Fallback to provided map
        _mapLoadSource = 'error_fallback';
        _mapLoadError = 'Load failed: $e';
      });
    } finally {
      setState(() {
        _isLoadingMap = false;
      });
    }
  }

  void _initializeAnimations() {
    _waypointAnimationController = AnimationController(
      duration: const Duration(milliseconds: 500),
      vsync: this,
    );

    _pulseAnimationController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );
    _pulseAnimationController.repeat();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Interactive Order Creator'),
        backgroundColor: Colors.blue,
        foregroundColor: Colors.white,
        actions: [
          // ✅ NEW: Map source indicator
          if (_mapLoadSource != null)
            Container(
              margin: EdgeInsets.only(right: 8),
              padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
              decoration: BoxDecoration(
                color: _getMapSourceColor(_mapLoadSource!),
                borderRadius: BorderRadius.circular(12),
              ),
              child: Text(
                _mapLoadSource!.toUpperCase(),
                style: TextStyle(fontSize: 10, color: Colors.white),
              ),
            ),
          IconButton(
            icon: Icon(_showGrid ? Icons.grid_on : Icons.grid_off),
            onPressed: () => setState(() => _showGrid = !_showGrid),
            tooltip: 'Toggle Grid',
          ),
          IconButton(
            icon: Icon(_showCoordinates ? Icons.gps_fixed : Icons.gps_off),
            onPressed: () =>
                setState(() => _showCoordinates = !_showCoordinates),
            tooltip: 'Toggle Coordinates',
          ),
          // ✅ NEW: Map visualization toggles
          PopupMenuButton<String>(
            icon: Icon(Icons.layers),
            tooltip: 'Map Layers',
            onSelected: (value) {
              setState(() {
                switch (value) {
                  case 'shapes':
                    _showMapShapes = !_showMapShapes;
                    break;
                  case 'occupancy':
                    _showOccupancyGrid = !_showOccupancyGrid;
                    break;
                  case 'reload':
                    _loadDeviceMapData();
                    break;
                }
              });
            },
            itemBuilder: (context) => [
              PopupMenuItem(
                value: 'shapes',
                child: Row(
                  children: [
                    Icon(_showMapShapes ? Icons.check_box : Icons.check_box_outline_blank),
                    SizedBox(width: 8),
                    Text('Map Shapes'),
                  ],
                ),
              ),
              PopupMenuItem(
                value: 'occupancy',
                child: Row(
                  children: [
                    Icon(_showOccupancyGrid ? Icons.check_box : Icons.check_box_outline_blank),
                    SizedBox(width: 8),
                    Text('Occupancy Grid'),
                  ],
                ),
              ),
              PopupMenuDivider(),
              PopupMenuItem(
                value: 'reload',
                child: Row(
                  children: [
                    Icon(Icons.refresh),
                    SizedBox(width: 8),
                    Text('Reload Map'),
                  ],
                ),
              ),
            ],
          ),
        ],
      ),
      body: SafeArea(
        child: Container(
          width: double.infinity,
          height: double.infinity,
          child: Column(
            children: [
              // ✅ NEW: Map loading status
              if (_isLoadingMap) _buildMapLoadingBanner(),
              if (_mapLoadError != null) _buildMapErrorBanner(),
              
              Expanded(
                child: Row(
                  children: [
                    // Map Canvas
                    Expanded(
                      flex: 3,
                      child: Container(
                        width: double.infinity,
                        height: double.infinity,
                        color: Colors.grey.shade100,
                        child: _buildDirectMapCanvas(),
                      ),
                    ),

                    // Control Panel
                    Container(
                      width: 400,
                      decoration: BoxDecoration(
                        border: Border(left: BorderSide(color: Colors.grey.shade300)),
                      ),
                      child: Column(
                        children: [
                          Expanded(child: _buildControlPanel()),
                          Container(
                            height: 80,
                            child: _buildBottomActionBar(),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  // ✅ NEW: Map loading status banner
  Widget _buildMapLoadingBanner() {
    return Container(
      width: double.infinity,
      padding: EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.blue.shade100,
        border: Border(bottom: BorderSide(color: Colors.blue.shade300)),
      ),
      child: Row(
        children: [
          SizedBox(
            width: 16,
            height: 16,
            child: CircularProgressIndicator(strokeWidth: 2),
          ),
          SizedBox(width: 12),
          Text('Loading map data for ${widget.deviceId}...'),
        ],
      ),
    );
  }

  // ✅ NEW: Map error banner
  Widget _buildMapErrorBanner() {
    return Container(
      width: double.infinity,
      padding: EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.orange.shade100,
        border: Border(bottom: BorderSide(color: Colors.orange.shade300)),
      ),
      child: Row(
        children: [
          Icon(Icons.warning, size: 16, color: Colors.orange.shade700),
          SizedBox(width: 8),
          Expanded(
            child: Text(
              _mapLoadError!,
              style: TextStyle(color: Colors.orange.shade700),
            ),
          ),
          TextButton(
            onPressed: _loadDeviceMapData,
            child: Text('Retry'),
          ),
        ],
      ),
    );
  }

  Widget _buildDirectMapCanvas() {
    return LayoutBuilder(
      builder: (context, constraints) {
        final width = constraints.maxWidth;
        final height = constraints.maxHeight;
        
        final canvasWidth = width.isInfinite || width <= 0 ? 800.0 : width;
        final canvasHeight = height.isInfinite || height <= 0 ? 600.0 : height;
        
        return Container(
          width: canvasWidth,
          height: canvasHeight,
          child: Stack(
            children: [
              // ✅ ENHANCED: Map widget with loaded data
              Positioned.fill(
                child: GestureDetector(
                  onTapDown: _handleMapTap,
                  onScaleStart: _handleScaleStart,
                  onScaleUpdate: _handleScaleUpdate,
                  child: RepaintBoundary(
                    child: CustomPaint(
                      painter: EnhancedInteractiveMapPainter(
                        mapData: widget.mapData, // Original for basic structure
                        loadedMapData: _loadedMapData, // Actual loaded map data
                        waypoints: _waypoints,
                        selectedCoordinate: _selectedCoordinate,
                        mapScale: _mapScale,
                        mapOffset: _mapOffset,
                        showGrid: _showGrid,
                        showCoordinates: _showCoordinates,
                        showMapShapes: _showMapShapes,
                        showOccupancyGrid: _showOccupancyGrid,
                        mapOpacity: _mapOpacity,
                        pulseAnimation: _pulseAnimationController,
                        canvasSize: Size(canvasWidth, canvasHeight),
                        isLoadingMap: _isLoadingMap,
                      ),
                    ),
                  ),
                ),
              ),
              
              // Debug info overlay
              Positioned(
                top: 10,
                right: 10,
                child: Container(
                  padding: EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    color: Colors.black54,
                    borderRadius: BorderRadius.circular(4),
                  ),
                  child: Text(
                    'Canvas: ${canvasWidth.toInt()}x${canvasHeight.toInt()}\n'
                    'Map: ${(_loadedMapData ?? widget.mapData).info.width}x${(_loadedMapData ?? widget.mapData).info.height}\n'
                    'Scale: ${(_mapScale * 100).toInt()}%\n'
                    'Waypoints: ${_waypoints.length}\n'
                    'Map Shapes: ${(_loadedMapData?.shapes.length ?? 0)}\n'
                    'Source: ${_mapLoadSource ?? 'none'}',
                    style: TextStyle(color: Colors.white, fontSize: 10),
                  ),
                ),
              ),
              
              // Overlays
              if (_selectedCoordinate != null && _showCoordinates)
                Positioned(
                  left: (_selectedCoordinate!.dx + 10).clamp(0.0, canvasWidth - 120),
                  top: (_selectedCoordinate!.dy - 30).clamp(0.0, canvasHeight - 30),
                  child: Container(
                    padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                    decoration: BoxDecoration(
                      color: Colors.black87,
                      borderRadius: BorderRadius.circular(4),
                    ),
                    child: Text(
                      _getMapCoordinatesText(_selectedCoordinate!),
                      style: TextStyle(color: Colors.white, fontSize: 12),
                    ),
                  ),
                ),

              if (_isAddingWaypoint)
                Positioned.fill(
                  child: Container(
                    color: Colors.blue.withOpacity(0.1),
                    child: Center(
                      child: Container(
                        padding: EdgeInsets.all(16),
                        decoration: BoxDecoration(
                          color: Colors.white,
                          borderRadius: BorderRadius.circular(8),
                          boxShadow: [
                            BoxShadow(color: Colors.black26, blurRadius: 8)
                          ],
                        ),
                        child: Column(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Text(
                              'Click on map to add ${_selectedWaypointType.toUpperCase()}',
                              style: TextStyle(
                                  fontSize: 16, fontWeight: FontWeight.bold),
                            ),
                            SizedBox(height: 8),
                            ElevatedButton(
                              onPressed: () =>
                                  setState(() => _isAddingWaypoint = false),
                              child: Text('Cancel'),
                            ),
                          ],
                        ),
                      ),
                    ),
                  ),
                ),
            ],
          ),
        );
      },
    );
  }

  Widget _buildControlPanel() {
    return Column(
      children: [
        // Scrollable content section
        Expanded(
          child: SingleChildScrollView(
            padding: EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisSize: MainAxisSize.min,
              children: [
                // Order Details Section
                _buildOrderDetailsSection(),

                SizedBox(height: 16),

                // Waypoint Types Section
                _buildWaypointTypesSection(),

                SizedBox(height: 16),

                // Waypoints List Section - Use intrinsic height
                _buildWaypointsListSection(),

                SizedBox(height: 16),

                // Actions
                _buildActionButtons(),
              ],
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildOrderDetailsSection() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Order Details',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 12),

            TextField(
              controller: _orderNameController,
              decoration: InputDecoration(
                labelText: 'Order Name',
                border: OutlineInputBorder(),
                prefixIcon: Icon(Icons.assignment),
              ),
            ),

            SizedBox(height: 12),

            TextField(
              controller: _priorityController,
              decoration: InputDecoration(
                labelText: 'Priority (0-10)',
                border: OutlineInputBorder(),
                prefixIcon: Icon(Icons.priority_high),
              ),
              keyboardType: TextInputType.number,
              onChanged: (value) {
                // Validate numeric input
                if (value.isNotEmpty && int.tryParse(value) == null) {
                  _priorityController.text = '0';
                  _priorityController.selection = TextSelection.fromPosition(
                    TextPosition(offset: _priorityController.text.length),
                  );
                }
              },
            ),

            SizedBox(height: 12),

            // Order summary
            Container(
              padding: EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: Colors.blue.shade50,
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.blue.shade200),
              ),
              child: Row(
                children: [
                  Icon(Icons.info, color: Colors.blue.shade700),
                  SizedBox(width: 8),
                  Expanded(
                    child: Text(
                      'Device: ${widget.deviceId}\nWaypoints: ${_waypoints.length}',
                      style: TextStyle(color: Colors.blue.shade700),
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildWaypointTypesSection() {
    final waypointTypes = [
      {
        'type': 'home',
        'label': 'Home',
        'icon': Icons.home,
        'color': Colors.green
      },
      {
        'type': 'pickup',
        'label': 'Pickup',
        'icon': Icons.outbox,
        'color': Colors.blue
      },
      {
        'type': 'drop',
        'label': 'Drop',
        'icon': Icons.inbox,
        'color': Colors.orange
      },
      {
        'type': 'charging',
        'label': 'Charge',
        'icon': Icons.battery_charging_full,
        'color': Colors.yellow.shade700
      },
      {
        'type': 'waypoint',
        'label': 'Point',
        'icon': Icons.place,
        'color': Colors.purple
      },
    ];

    return Card(
      child: Padding(
        padding: EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(
              'Add Waypoint',
              style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            Text(
              'Select type and click on map:',
              style: TextStyle(color: Colors.grey.shade600, fontSize: 12),
            ),
            SizedBox(height: 8),
            // Fix wrap overflow by constraining width
            LayoutBuilder(
              builder: (context, constraints) {
                return Wrap(
                  spacing: 4,
                  runSpacing: 4,
                  children: waypointTypes
                      .map((type) => _buildWaypointTypeButton(type, constraints.maxWidth))
                      .toList(),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildWaypointTypeButton(Map<String, dynamic> type, double maxWidth) {
    final isSelected = _selectedWaypointType == type['type'];
    final color = type['color'] as Color;
    
    // Calculate button width to prevent overflow
    final buttonWidth = (maxWidth - 32) / 3; // 3 buttons per row with padding

    return GestureDetector(
      onTap: () {
        setState(() {
          _selectedWaypointType = type['type'];
          _isAddingWaypoint = true;
        });
      },
      child: Container(
        width: buttonWidth,
        padding: EdgeInsets.symmetric(horizontal: 6, vertical: 4),
        decoration: BoxDecoration(
          color: isSelected ? color : color.withOpacity(0.1),
          border: Border.all(color: color),
          borderRadius: BorderRadius.circular(12),
        ),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(
              type['icon'] as IconData,
              size: 12,
              color: isSelected ? Colors.white : color,
            ),
            SizedBox(width: 2),
            Flexible(
              child: Text(
                type['label'],
                style: TextStyle(
                  color: isSelected ? Colors.white : color,
                  fontWeight: FontWeight.w600,
                  fontSize: 9,
                ),
                overflow: TextOverflow.ellipsis,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildWaypointsListSection() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          mainAxisSize: MainAxisSize.min,
          children: [
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Expanded(
                  child: Text(
                    'Waypoints (${_waypoints.length})',
                    style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
                    overflow: TextOverflow.ellipsis,
                  ),
                ),
                if (_waypoints.isNotEmpty)
                  IconButton(
                    icon: Icon(Icons.clear_all, size: 18),
                    onPressed: _clearAllWaypoints,
                    tooltip: 'Clear All',
                    padding: EdgeInsets.all(2),
                    constraints: BoxConstraints(minWidth: 28, minHeight: 28),
                  ),
              ],
            ),
            SizedBox(height: 8),
            // Fixed height container for waypoints list
            Container(
              height: 200, // Fixed height to prevent overflow
              child: _waypoints.isEmpty
                  ? Center(
                      child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          Icon(Icons.touch_app, size: 24, color: Colors.grey),
                          SizedBox(height: 6),
                          Text(
                            'No waypoints added yet',
                            style: TextStyle(color: Colors.grey.shade600, fontSize: 11),
                          ),
                          Text(
                            'Select a type and click on the map',
                            style: TextStyle(color: Colors.grey.shade500, fontSize: 9),
                          ),
                        ],
                      ),
                    )
                  : ListView.builder(
                      itemCount: _waypoints.length,
                      itemBuilder: (context, index) => _buildCompactWaypointListItem(index),
                    ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildCompactWaypointListItem(int index) {
    final waypoint = _waypoints[index];
    final color = _getWaypointTypeColor(waypoint['type']);

    return Card(
      key: ValueKey(waypoint['id']),
      margin: EdgeInsets.symmetric(vertical: 1),
      child: ListTile(
        dense: true,
        contentPadding: EdgeInsets.symmetric(horizontal: 6, vertical: 1),
        leading: Container(
          width: 28,
          height: 28,
          decoration: BoxDecoration(
            color: color,
            borderRadius: BorderRadius.circular(4),
          ),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Text(
                '${index + 1}',
                style: TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  fontSize: 8,
                ),
              ),
              Icon(
                _getWaypointTypeIcon(waypoint['type']),
                color: Colors.white,
                size: 10,
              ),
            ],
          ),
        ),
        title: Text(
          waypoint['name'],
          style: TextStyle(fontWeight: FontWeight.w600, fontSize: 11),
          maxLines: 1,
          overflow: TextOverflow.ellipsis,
        ),
        subtitle: Text(
          '${waypoint['type'].toUpperCase()}\n(${waypoint['coordinates']['x'].toStringAsFixed(1)}, ${waypoint['coordinates']['y'].toStringAsFixed(1)})',
          style: TextStyle(fontSize: 9),
          maxLines: 2,
          overflow: TextOverflow.ellipsis,
        ),
        trailing: PopupMenuButton<String>(
          onSelected: (value) => _handleWaypointAction(index, value),
          itemBuilder: (context) => [
            PopupMenuItem(value: 'edit', child: Text('Edit', style: TextStyle(fontSize: 11))),
            PopupMenuItem(value: 'duplicate', child: Text('Duplicate', style: TextStyle(fontSize: 11))),
            PopupMenuItem(value: 'delete', child: Text('Delete', style: TextStyle(fontSize: 11))),
          ],
          padding: EdgeInsets.zero,
          icon: Icon(Icons.more_vert, size: 14),
        ),
        isThreeLine: false,
      ),
    );
  }

  Widget _buildActionButtons() {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        SizedBox(
          width: double.infinity,
          child: ElevatedButton.icon(
            onPressed: _waypoints.isNotEmpty && !_isCreatingOrder
                ? _createOrder
                : null,
            icon: _isCreatingOrder
                ? SizedBox(
                    width: 12,
                    height: 12,
                    child: CircularProgressIndicator(strokeWidth: 2))
                : Icon(Icons.send, size: 14),
            label: Text(_isCreatingOrder ? 'Creating...' : 'Create Order',
                style: TextStyle(fontSize: 11)),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.green,
              foregroundColor: Colors.white,
              padding: EdgeInsets.symmetric(vertical: 6),
            ),
          ),
        ),
        SizedBox(height: 4),
        Row(
          children: [
            Expanded(
              child: OutlinedButton.icon(
                onPressed: () => _previewOrderExecution(),
                icon: Icon(Icons.preview, size: 12),
                label: Text('Preview', style: TextStyle(fontSize: 10)),
                style: OutlinedButton.styleFrom(
                  padding: EdgeInsets.symmetric(vertical: 4),
                ),
              ),
            ),
            SizedBox(width: 4),
            Expanded(
              child: OutlinedButton.icon(
                onPressed: () => _saveOrderTemplate(),
                icon: Icon(Icons.save, size: 12),
                label: Text('Template', style: TextStyle(fontSize: 10)),
                style: OutlinedButton.styleFrom(
                  padding: EdgeInsets.symmetric(vertical: 4),
                ),
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildBottomActionBar() {
    return Container(
      padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      decoration: BoxDecoration(
        border: Border(top: BorderSide(color: Colors.grey.shade300)),
        color: Colors.white,
      ),
      child: Row(
        children: [
          // Map controls
          Text('Scale: ${(_mapScale * 100).toInt()}%', style: TextStyle(fontSize: 11)),
          SizedBox(width: 8),
          Expanded(
            child: Slider(
              value: _mapScale,
              min: 0.5,
              max: 3.0,
              divisions: 25,
              onChanged: (value) => setState(() => _mapScale = value),
            ),
          ),

          // Quick actions
          IconButton(
            icon: Icon(Icons.center_focus_strong, size: 18),
            onPressed: _centerMap,
            tooltip: 'Center Map',
            padding: EdgeInsets.all(4),
            constraints: BoxConstraints(minWidth: 32, minHeight: 32),
          ),
          IconButton(
            icon: Icon(Icons.refresh, size: 18),
            onPressed: _resetMapView,
            tooltip: 'Reset View',
            padding: EdgeInsets.all(4),
            constraints: BoxConstraints(minWidth: 32, minHeight: 32),
          ),
        ],
      ),
    );
  }

  // ==========================================
  // EVENT HANDLERS
  // ==========================================

  void _handleMapTap(TapDownDetails details) {
    if (!_isAddingWaypoint) {
      setState(() {
        _selectedCoordinate = details.localPosition;
      });
      return;
    }

    // Convert screen coordinates to map coordinates
    final mapCoordinates = _screenToMapCoordinates(details.localPosition);
    _showWaypointCreationDialog(mapCoordinates);
  }

  void _handleScaleStart(ScaleStartDetails details) {
    // Store initial values for scaling
  }

  void _handleScaleUpdate(ScaleUpdateDetails details) {
    setState(() {
      _mapScale = (_mapScale * details.scale).clamp(0.5, 3.0);
      _mapOffset += details.focalPointDelta;
    });
  }

  void _handleWaypointAction(int index, String action) {
    switch (action) {
      case 'edit':
        _editWaypoint(index);
        break;
      case 'duplicate':
        _duplicateWaypoint(index);
        break;
      case 'delete':
        _deleteWaypoint(index);
        break;
    }
  }

  // ==========================================
  // WAYPOINT MANAGEMENT
  // ==========================================

  void _showWaypointCreationDialog(Map<String, double> coordinates) {
    _waypointNameController.clear();

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Add ${_selectedWaypointType.toUpperCase()}'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
                'Coordinates: (${coordinates['x']!.toStringAsFixed(2)}, ${coordinates['y']!.toStringAsFixed(2)})'),
            SizedBox(height: 16),
            TextField(
              controller: _waypointNameController,
              decoration: InputDecoration(
                labelText: 'Waypoint Name',
                hintText: 'e.g., Pickup Station A',
                border: OutlineInputBorder(),
              ),
              autofocus: true,
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
              setState(() => _isAddingWaypoint = false);
            },
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              _addWaypoint(coordinates, _waypointNameController.text.trim());
              Navigator.of(context).pop();
              setState(() => _isAddingWaypoint = false);
            },
            child: Text('Add'),
          ),
        ],
      ),
    );
  }

  void _addWaypoint(Map<String, double> coordinates, String name) {
    // Enhanced coordinate validation
    if (!_isValidMapCoordinate(coordinates['x']!, coordinates['y']!)) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Warning: Coordinates (${coordinates['x']!.toStringAsFixed(3)}, ${coordinates['y']!.toStringAsFixed(3)}) may be outside valid map bounds'),
          backgroundColor: Colors.orange,
          duration: Duration(seconds: 4),
        ),
      );
    }
    
    final waypoint = {
      'id': DateTime.now().millisecondsSinceEpoch.toString(),
      'name': name.isNotEmpty
          ? name
          : '${_selectedWaypointType.toUpperCase()} ${_waypoints.length + 1}',
      'type': _selectedWaypointType,
      'coordinates': coordinates,
      'orientation': 0.0, // Default orientation in radians for ROS
      'metadata': {
        'createdAt': DateTime.now().toIso8601String(),
        'color': '#${_getWaypointTypeColor(_selectedWaypointType).value.toRadixString(16).padLeft(8, '0')}',
        'mapResolution': widget.mapData.info.resolution,
        'coordinateFrame': 'map', // ROS coordinate frame
        'validationStatus': _isValidMapCoordinate(coordinates['x']!, coordinates['y']!) ? 'valid' : 'warning',
      },
    };

    setState(() {
      _waypoints.add(waypoint);
    });

    _waypointAnimationController.forward().then((_) {
      _waypointAnimationController.reset();
    });

    print('✅ Waypoint added: ${waypoint['name']}');
    print('📍 ROS coordinates: (${coordinates['x']!.toStringAsFixed(3)}, ${coordinates['y']!.toStringAsFixed(3)})');
    print('🎯 Type: ${_selectedWaypointType}');

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('${waypoint['name']} added at (${coordinates['x']!.toStringAsFixed(3)}, ${coordinates['y']!.toStringAsFixed(3)})'),
        duration: Duration(seconds: 3),
        backgroundColor: _isValidMapCoordinate(coordinates['x']!, coordinates['y']!) 
            ? Colors.green 
            : Colors.orange,
      ),
    );
  }

  void _editWaypoint(int index) {
    final waypoint = _waypoints[index];
    _waypointNameController.text = waypoint['name'];

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Edit Waypoint'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextField(
              controller: _waypointNameController,
              decoration: InputDecoration(
                labelText: 'Waypoint Name',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 16),
            Text(
                'Coordinates: (${waypoint['coordinates']['x'].toStringAsFixed(2)}, ${waypoint['coordinates']['y'].toStringAsFixed(2)})'),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              setState(() {
                _waypoints[index]['name'] = _waypointNameController.text.trim();
              });
              Navigator.of(context).pop();
            },
            child: Text('Save'),
          ),
        ],
      ),
    );
  }

  void _duplicateWaypoint(int index) {
    final originalWaypoint = _waypoints[index];
    final duplicatedWaypoint = Map<String, dynamic>.from(originalWaypoint);
    duplicatedWaypoint['id'] = DateTime.now().millisecondsSinceEpoch.toString();
    duplicatedWaypoint['name'] = '${originalWaypoint['name']} (Copy)';

    setState(() {
      _waypoints.insert(index + 1, duplicatedWaypoint);
    });
  }

  void _deleteWaypoint(int index) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Delete Waypoint'),
        content: Text(
            'Are you sure you want to delete "${_waypoints[index]['name']}"?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              setState(() {
                _waypoints.removeAt(index);
              });
              Navigator.of(context).pop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child: Text('Delete'),
          ),
        ],
      ),
    );
  }

  void _reorderWaypoints(int oldIndex, int newIndex) {
    setState(() {
      if (newIndex > oldIndex) {
        newIndex -= 1;
      }
      final waypoint = _waypoints.removeAt(oldIndex);
      _waypoints.insert(newIndex, waypoint);
    });
  }

  void _clearAllWaypoints() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Clear All Waypoints'),
        content: Text('Are you sure you want to remove all waypoints?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              setState(() {
                _waypoints.clear();
              });
              Navigator.of(context).pop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child: Text('Clear All'),
          ),
        ],
      ),
    );
  }

  // ==========================================
  // ORDER CREATION & EXECUTION
  // ==========================================

  Future<void> _createOrder() async {
    setState(() {
      _isCreatingOrder = true;
    });

    try {
      int priority = 0;
      final priorityText = _priorityController.text.trim();
      if (priorityText.isNotEmpty) {
        priority = int.tryParse(priorityText) ?? 0;
      }

      // Validate all waypoints before creating order
      for (int i = 0; i < _waypoints.length; i++) {
        final wp = _waypoints[i];
        final coords = wp['coordinates'] as Map<String, dynamic>;
        if (!_isValidMapCoordinate(coords['x'] as double, coords['y'] as double)) {
          throw Exception('Waypoint ${i + 1} (${wp['name']}) has invalid coordinates');
        }
      }

      final orderData = {
        'deviceId': widget.deviceId,
        'name': _orderNameController.text.trim(),
        'priority': priority,
        'description': 'Created from interactive map with ${_waypoints.length} waypoints. Map resolution: ${widget.mapData.info.resolution}m/pixel',
        'waypoints': _waypoints.map((wp) => {
          'name': wp['name'],
          'type': wp['type'],
          'position': {
            'x': wp['coordinates']['x'], // Already in ROS world coordinates (meters)
            'y': wp['coordinates']['y'], // Already in ROS world coordinates (meters)
            'z': 0.0, // Assuming 2D navigation
          },
          'orientation': wp['orientation'] ?? 0.0, // Radians for ROS
          'metadata': {
            ...wp['metadata'],
            'coordinateSystem': 'map', // ROS coordinate frame
            'createdFromMap': widget.mapData.info.width.toString() + 'x' + widget.mapData.info.height.toString(),
            'mapOrigin': {
              'x': widget.mapData.info.origin.x,
              'y': widget.mapData.info.origin.y,
            },
            'mapResolution': widget.mapData.info.resolution,
          },
        }).toList(),
      };

      print('🚀 Creating order with ROS-compatible coordinates:');
      final waypointsList = orderData['waypoints'] as List<Map<String, dynamic>>;
      for (int i = 0; i < waypointsList.length; i++) {
        final wp = waypointsList[i];
        final pos = wp['position'] as Map<String, dynamic>;
        print('  ${i + 1}. ${wp['name']} (${wp['type']}): (${pos['x']}, ${pos['y']}) meters');
      }

      final response = await _apiService.createOrder(
        deviceId: widget.deviceId,
        name: orderData['name'] as String,
        waypoints: orderData['waypoints'] as List<Map<String, dynamic>>,
        priority: orderData['priority'] as int,
        description: orderData['description'] as String,
      );

      if (response['success'] == true) {
        widget.onOrderCreated(orderData);

        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('✅ Order created successfully with ${_waypoints.length} waypoints!\nReady for execution with 3-second intervals.'),
            backgroundColor: Colors.green,
            duration: Duration(seconds: 4),
          ),
        );

        Navigator.of(context).pop();
      } else {
        throw Exception(response['error'] ?? 'Failed to create order');
      }
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('❌ Failed to create order: $e'),
          backgroundColor: Colors.red,
          duration: Duration(seconds: 5),
        ),
      );
    } finally {
      setState(() {
        _isCreatingOrder = false;
      });
    }
  }

  void _previewOrderExecution() {
    if (_waypoints.isEmpty) return;

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Order Execution Preview'),
        content: Container(
          width: 400,
          height: 300,
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text('Execution Sequence:',
                  style: TextStyle(fontWeight: FontWeight.bold)),
              SizedBox(height: 8),
              Expanded(
                child: ListView.builder(
                  itemCount: _waypoints.length,
                  itemBuilder: (context, index) {
                    final waypoint = _waypoints[index];
                    return ListTile(
                      leading: CircleAvatar(
                        backgroundColor:
                            _getWaypointTypeColor(waypoint['type']),
                        child: Text('${index + 1}'),
                      ),
                      title: Text(waypoint['name']),
                      subtitle: Text(
                        '${waypoint['type'].toUpperCase()}\n→ (${waypoint['coordinates']['x'].toStringAsFixed(1)}, ${waypoint['coordinates']['y'].toStringAsFixed(1)})',
                      ),
                      trailing: Icon(Icons.arrow_downward),
                    );
                  },
                ),
              ),
            ],
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Close'),
          ),
        ],
      ),
    );
  }

  void _saveOrderTemplate() {
    // Implementation for saving order template
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text('Order template save feature coming soon!')),
    );
  }

  // Add this method to convert world coordinates back to screen for displaying waypoints
  Offset _worldToScreenCoordinates(double worldX, double worldY) {
    try {
      final renderBox = context.findRenderObject() as RenderBox?;
      final canvasSize = renderBox?.size ?? Size(800, 600);
      final canvasWidth = canvasSize.width;
      final canvasHeight = canvasSize.height;
      
      final mapWidth = widget.mapData.info.width.toDouble();
      final mapHeight = widget.mapData.info.height.toDouble();
      final resolution = widget.mapData.info.resolution;
      final originX = widget.mapData.info.origin.x;
      final originY = widget.mapData.info.origin.y;
      
      // Convert world coordinates to map pixel coordinates
      final mapPixelX = (worldX - originX) / resolution;
      final mapPixelY = mapHeight - ((worldY - originY) / resolution); // Flip Y axis
      
      // Convert map pixel coordinates to canvas coordinates
      final canvasX = (mapPixelX / mapWidth) * canvasWidth;
      final canvasY = (mapPixelY / mapHeight) * canvasHeight;
      
      // Apply current transform (scale and offset)
      final screenX = (canvasX * _mapScale) + _mapOffset.dx;
      final screenY = (canvasY * _mapScale) + _mapOffset.dy;
      
      return Offset(screenX, screenY);
    } catch (e) {
      print('❌ Error converting world to screen coordinates: $e');
      return Offset.zero;
    }
  }

  // ==========================================
  // MAP UTILITIES
  // ==========================================
  bool _isValidMapCoordinate(double x, double y) {
    // Define reasonable bounds based on typical indoor/outdoor maps
    // These bounds should match your actual map area
    final maxBound = 100.0; // 100 meters from origin
    final minBound = -100.0; // -100 meters from origin
    
    final isValid = x >= minBound && x <= maxBound && 
                    y >= minBound && y <= maxBound &&
                    x.isFinite && y.isFinite && !x.isNaN && !y.isNaN;
                    
    if (!isValid) {
      print('⚠️ Invalid coordinates: x=$x, y=$y (bounds: $minBound to $maxBound)');
    }
    
    return isValid;
  }

  Map<String, double> _screenToMapCoordinates(Offset screenPosition) {
    try {
      // Use loaded map data if available, otherwise fall back to provided map
      final mapData = _loadedMapData ?? widget.mapData;
      
      final renderBox = context.findRenderObject() as RenderBox?;
      final canvasSize = renderBox?.size ?? Size(800, 600);
      final canvasWidth = canvasSize.width;
      final canvasHeight = canvasSize.height;
      
      final mapWidth = mapData.info.width.toDouble();
      final mapHeight = mapData.info.height.toDouble();
      final resolution = mapData.info.resolution;
      final originX = mapData.info.origin.x;
      final originY = mapData.info.origin.y;
      
      print('🗺️ Using ${_mapLoadSource ?? 'provided'} map for coordinates: ${mapWidth}x${mapHeight}, resolution: ${resolution}m/pixel');
      
      // Apply current map transformations
      final adjustedX = (screenPosition.dx - _mapOffset.dx) / _mapScale;
      final adjustedY = (screenPosition.dy - _mapOffset.dy) / _mapScale;
      
      // Convert canvas coordinates to map pixel coordinates
      final mapPixelX = (adjustedX / canvasWidth) * mapWidth;
      final mapPixelY = (adjustedY / canvasHeight) * mapHeight;
      
      // Convert to world coordinates
      final worldX = originX + (mapPixelX * resolution);
      final worldY = originY + ((mapHeight - mapPixelY) * resolution);
      
      print('🔄 Enhanced coordinate conversion:');
      print('  Screen: (${screenPosition.dx.toStringAsFixed(1)}, ${screenPosition.dy.toStringAsFixed(1)})');
      print('  World (ROS): (${worldX.toStringAsFixed(3)}, ${worldY.toStringAsFixed(3)})');
      
      return {
        'x': worldX,
        'y': worldY,
      };
    } catch (e) {
      print('❌ Error in enhanced coordinate conversion: $e');
      return {'x': 0.0, 'y': 0.0};
    }
  }

  String _getMapCoordinatesText(Offset screenPosition) {
    try {
      final coords = _screenToMapCoordinates(screenPosition);
      // Show coordinates in meters with 3 decimal places for precision
      return '(${coords['x']!.toStringAsFixed(3)}m, ${coords['y']!.toStringAsFixed(3)}m)';
    } catch (e) {
      print('❌ Error getting map coordinates text: $e');
      return '(0.000m, 0.000m)';
    }
  }

  void _centerMap() {
    setState(() {
      _mapOffset = Offset.zero;
      _mapScale = 1.0;
    });
  }

  void _resetMapView() {
    setState(() {
      _mapOffset = Offset.zero;
      _mapScale = 1.0;
      _selectedCoordinate = null;
    });
  }

  // ==========================================
  // HELPER METHODS
  // ==========================================

  Color _getWaypointTypeColor(String type) {
    switch (type) {
      case 'home':
        return Colors.green;
      case 'pickup':
        return Colors.blue;
      case 'drop':
        return Colors.orange;
      case 'charging':
        return Colors.yellow.shade700;
      case 'waypoint':
        return Colors.purple;
      default:
        return Colors.grey;
    }
  }

  IconData _getWaypointTypeIcon(String type) {
    switch (type) {
      case 'home':
        return Icons.home;
      case 'pickup':
        return Icons.outbox;
      case 'drop':
        return Icons.inbox;
      case 'charging':
        return Icons.battery_charging_full;
      case 'waypoint':
        return Icons.place;
      default:
        return Icons.location_on;
    }
  }

  /// Get color for map source indicator
  Color _getMapSourceColor(String source) {
    switch (source.toLowerCase()) {
      case 'api':
      case 'loaded':
        return Colors.green;
      case 'provided_default':
      case 'provided':
        return Colors.blue;
      case 'fallback_provided':
      case 'fallback':
        return Colors.orange;
      case 'error_fallback':
      case 'error':
        return Colors.red;
      default:
        return Colors.grey;
    }
  }

  @override
  void dispose() {
    _waypointAnimationController.dispose();
    _pulseAnimationController.dispose();
    _orderNameController.dispose();
    _priorityController.dispose();
    _waypointNameController.dispose();
    super.dispose();
  }

  // ✅ FIX 10: Test method to verify layout
  void _testLayout() {
    final screenSize = MediaQuery.of(context).size;
    final appBarHeight = AppBar().preferredSize.height;
    final statusBarHeight = MediaQuery.of(context).padding.top;
    final bottomPadding = MediaQuery.of(context).padding.bottom;
    
    print('🧪 Layout test:');
    print('  Screen: ${screenSize.width}x${screenSize.height}');
    print('  AppBar: $appBarHeight');
    print('  Status: $statusBarHeight');
    print('  Bottom: $bottomPadding');
    print('  Available: ${screenSize.height - appBarHeight - statusBarHeight - bottomPadding}');
  }
}

// ==========================================
// CUSTOM PAINTER FOR INTERACTIVE MAP
// ==========================================

// ✅ ENHANCED CustomPainter with better validation
class EnhancedInteractiveMapPainter extends CustomPainter {
  final MapData mapData;
  final MapData? loadedMapData; // Actual loaded map data
  final List<Map<String, dynamic>> waypoints;
  final Offset? selectedCoordinate;
  final double mapScale;
  final Offset mapOffset;
  final bool showGrid;
  final bool showCoordinates;
  final bool showMapShapes;
  final bool showOccupancyGrid;
  final double mapOpacity;
  final AnimationController pulseAnimation;
  final Size canvasSize;
  final bool isLoadingMap;

  EnhancedInteractiveMapPainter({
    required this.mapData,
    this.loadedMapData,
    required this.waypoints,
    this.selectedCoordinate,
    required this.mapScale,
    required this.mapOffset,
    required this.showGrid,
    required this.showCoordinates,
    required this.showMapShapes,
    required this.showOccupancyGrid,
    required this.mapOpacity,
    required this.pulseAnimation,
    required this.canvasSize,
    required this.isLoadingMap,
  }) : super(repaint: pulseAnimation);

  @override
  void paint(Canvas canvas, Size size) {
    try {
      final workingMapData = loadedMapData ?? mapData;
      
      Size effectiveSize = size.width > 0 && size.height > 0 ? size : canvasSize;
      if (effectiveSize.width <= 0 || effectiveSize.height <= 0) {
        effectiveSize = Size(800, 600);
      }

      // Validate canvas and size
      if (!effectiveSize.width.isFinite || !effectiveSize.height.isFinite ||
          effectiveSize.width <= 0 || effectiveSize.height <= 0) {
        print('❌ Invalid canvas size: $effectiveSize');
        _drawErrorState(canvas, size);
        return;
      }

      canvas.save();
      
      // Validate map offset and scale
      if (!mapOffset.dx.isFinite || !mapOffset.dy.isFinite || 
          !mapScale.isFinite || mapScale <= 0) {
        print('❌ Invalid map transform: offset=$mapOffset, scale=$mapScale');
        canvas.restore();
        _drawErrorState(canvas, size);
        return;
      }
      
      canvas.translate(mapOffset.dx, mapOffset.dy);
      canvas.scale(mapScale);

      // Draw map background
      _drawMapBackground(canvas, effectiveSize, workingMapData);

      // ✅ NEW: Draw occupancy grid if enabled and available
      if (showOccupancyGrid && workingMapData.occupancyData.isNotEmpty) {
        try {
          _drawOccupancyGrid(canvas, effectiveSize, workingMapData);
        } catch (occupancyError) {
          print('❌ Error drawing occupancy grid: $occupancyError');
        }
      }

      // Draw coordinate grid
      if (showGrid) {
        try {
          _drawCoordinateGrid(canvas, effectiveSize, workingMapData);
        } catch (gridError) {
          print('❌ Error drawing coordinate grid: $gridError');
        }
      }

      // ✅ NEW: Draw map shapes if enabled
      if (showMapShapes) {
        try {
          _drawMapShapes(canvas, workingMapData);
        } catch (shapeError) {
          print('❌ Critical error in shape drawing: $shapeError');
          // Continue without shapes to prevent app crash
        }
      }

      // Draw waypoints with enhanced visualization
      try {
        _drawWaypoints(canvas, effectiveSize, workingMapData);
      } catch (waypointError) {
        print('❌ Error drawing waypoints: $waypointError');
      }

      // Draw selected coordinate
      if (selectedCoordinate != null) {
        try {
          _drawSelectedCoordinate(canvas);
        } catch (coordError) {
          print('❌ Error drawing selected coordinate: $coordError');
        }
      }

      // ✅ NEW: Draw loading overlay
      if (isLoadingMap) {
        try {
          _drawLoadingOverlay(canvas, effectiveSize);
        } catch (loadingError) {
          print('❌ Error drawing loading overlay: $loadingError');
        }
      }

      canvas.restore();
      
    } catch (e, stackTrace) {
      print('❌ Enhanced paint error: $e');
      print('Stack trace: $stackTrace');
      try {
        canvas.restore(); // Try to restore canvas state
      } catch (restoreError) {
        print('❌ Error restoring canvas: $restoreError');
      }
      _drawErrorState(canvas, size);
    }
  }

  // ✅ NEW: Draw occupancy grid
  void _drawOccupancyGrid(Canvas canvas, Size size, MapData mapData) {
    try {
      final adjustedWidth = size.width / mapScale;
      final adjustedHeight = size.height / mapScale;
      
      final mapWidth = mapData.info.width;
      final mapHeight = mapData.info.height;
      final occupancyData = mapData.occupancyData;
      
      if (occupancyData.isEmpty) return;
      
      // Calculate cell size
      final cellWidth = adjustedWidth / mapWidth;
      final cellHeight = adjustedHeight / mapHeight;
      
      // Only draw if cells are large enough to be visible
      if (cellWidth < 0.5 || cellHeight < 0.5) return;
      
      for (int y = 0; y < mapHeight; y++) {
        for (int x = 0; x < mapWidth; x++) {
          final index = y * mapWidth + x;
          if (index >= occupancyData.length) continue;
          
          final value = occupancyData[index];
          Color? cellColor;
          
          if (value == 100) {
            // Occupied (obstacles)
            cellColor = Colors.black.withOpacity(mapOpacity);
          } else if (value == 0) {
            // Free space
            cellColor = Colors.white.withOpacity(mapOpacity * 0.3);
          } else if (value == -1) {
            // Unknown space - don't draw
            continue;
          }
          
          if (cellColor != null) {
            final paint = Paint()..color = cellColor;
            
            canvas.drawRect(
              Rect.fromLTWH(
                x * cellWidth,
                y * cellHeight,
                cellWidth,
                cellHeight,
              ),
              paint,
            );
          }
        }
      }
      
      print('🎨 Drew occupancy grid: ${mapWidth}x${mapHeight} cells');
    } catch (e) {
      print('❌ Error drawing occupancy grid: $e');
    }
  }

  // ✅ ENHANCED: Draw map shapes with comprehensive validation
  void _drawMapShapes(Canvas canvas, MapData mapData) {
    if (mapData.shapes.isEmpty) return;

    print('🎨 Drawing ${mapData.shapes.length} shapes...');

    for (int shapeIndex = 0; shapeIndex < mapData.shapes.length; shapeIndex++) {
      final shape = mapData.shapes[shapeIndex];
      
      try {
        print('🔹 Processing shape ${shapeIndex}: ${shape.name} with ${shape.points.length} points');
        
        // Skip shapes with no points
        if (shape.points.isEmpty) {
          print('⚠️ Skipping shape ${shape.name} - no points');
          continue;
        }

        final color = _parseShapeColor(shape.color);
        print('🎨 Shape ${shape.name} color parsed: ${color.toString()}');
        
        // Validate color
        if (color.alpha < 0 || color.alpha > 255 || 
            color.red < 0 || color.red > 255 ||
            color.green < 0 || color.green > 255 ||
            color.blue < 0 || color.blue > 255) {
          print('❌ Invalid color values for shape ${shape.name}');
          continue;
        }
        
        // Validate and clamp opacity
        final validOpacity = mapOpacity.clamp(0.0, 1.0);
        if (!validOpacity.isFinite) {
          print('❌ Invalid opacity for shape ${shape.name}');
          continue;
        }
        
        final shapeColor = Color.fromARGB(
          (color.alpha * validOpacity).round().clamp(0, 255),
          color.red,
          color.green,
          color.blue,
        );
        
        // Create and validate Paint objects
        final paint = Paint();
        paint.color = shapeColor;
        paint.style = PaintingStyle.fill;
        
        final outlinePaint = Paint();
        final outlineOpacity = (validOpacity * 0.8).clamp(0.0, 1.0);
        outlinePaint.color = Color.fromARGB(
          (color.alpha * outlineOpacity).round().clamp(0, 255),
          color.red,
          color.green,
          color.blue,
        );
        outlinePaint.style = PaintingStyle.stroke;
        outlinePaint.strokeWidth = 1.5; // Fixed stroke width

        // Collect valid points
        final validPoints = <Offset>[];
        
        for (int i = 0; i < shape.points.length; i++) {
          final point = _worldToCanvasCoordinates(
            shape.points[i].x,
            shape.points[i].y,
            Size(canvasSize.width / mapScale, canvasSize.height / mapScale),
            mapData
          );
          
          if (point != null && 
              point.dx.isFinite && point.dy.isFinite &&
              point.dx.abs() < 100000 && point.dy.abs() < 100000) {
            validPoints.add(point);
          } else {
            print('⚠️ Invalid point ${i} for shape ${shape.name}: $point');
          }
        }
        
        print('🔹 Shape ${shape.name}: ${validPoints.length}/${shape.points.length} valid points');
        
        // Need at least 3 points for a shape
        if (validPoints.length < 3) {
          print('⚠️ Skipping shape ${shape.name} - insufficient valid points');
          continue;
        }
        
        // Create path with validated points
        final path = Path();
        bool pathCreated = false;
        
        try {
          path.moveTo(validPoints[0].dx, validPoints[0].dy);
          
          for (int i = 1; i < validPoints.length; i++) {
            path.lineTo(validPoints[i].dx, validPoints[i].dy);
          }
          path.close();
          pathCreated = true;
        } catch (pathError) {
          print('❌ Error creating path for shape ${shape.name}: $pathError');
          pathCreated = false;
        }
        
        if (!pathCreated) {
          // Fallback: draw simple bounding rectangle
          try {
            final minX = validPoints.map((p) => p.dx).reduce((a, b) => a < b ? a : b);
            final maxX = validPoints.map((p) => p.dx).reduce((a, b) => a > b ? a : b);
            final minY = validPoints.map((p) => p.dy).reduce((a, b) => a < b ? a : b);
            final maxY = validPoints.map((p) => p.dy).reduce((a, b) => a > b ? a : b);
            
            final rect = Rect.fromLTRB(minX, minY, maxX, maxY);
            if (rect.isFinite && !rect.isEmpty) {
              canvas.drawRect(rect, paint);
              canvas.drawRect(rect, outlinePaint);
              print('✅ Drew fallback rectangle for shape ${shape.name}');
            }
          } catch (rectError) {
            print('❌ Error drawing fallback rectangle for shape ${shape.name}: $rectError');
          }
          continue;
        }
        
        // Validate path bounds
        final bounds = path.getBounds();
        if (!bounds.isFinite || bounds.isEmpty) {
          print('❌ Invalid path bounds for shape ${shape.name}: $bounds');
          continue;
        }
        
        print('🎨 Drawing shape ${shape.name} with bounds: $bounds');
        
        // Draw with error handling
        try {
          canvas.drawPath(path, paint);
          print('✅ Filled shape ${shape.name}');
        } catch (fillError) {
          print('❌ Error drawing filled shape ${shape.name}: $fillError');
        }
        
        try {
          canvas.drawPath(path, outlinePaint);
          print('✅ Outlined shape ${shape.name}');
        } catch (outlineError) {
          print('❌ Error drawing outline for shape ${shape.name}: $outlineError');
        }
        
        // Draw label safely
        try {
          _drawShapeLabel(canvas, shape, validPoints[0]);
          print('✅ Labeled shape ${shape.name}');
        } catch (labelError) {
          print('❌ Error drawing label for shape ${shape.name}: $labelError');
        }
        
      } catch (e, stackTrace) {
        print('❌ Error processing shape ${shape.name}: $e');
        print('Stack trace: $stackTrace');
      }
    }
    
    print('🎨 Finished drawing shapes');
  }

  // ✅ NEW: Draw shape labels with validation
  void _drawShapeLabel(Canvas canvas, MapShape shape, Offset position) {
    try {
      // Validate position coordinates
      if (!position.dx.isFinite || !position.dy.isFinite) {
        print('⚠️ Invalid position for shape label: ${shape.name}');
        return;
      }

      final textPainter = TextPainter(
        text: TextSpan(
          text: shape.name.isNotEmpty ? shape.name : 'Shape',
          style: TextStyle(
            color: Colors.black87,
            fontSize: 10.0.clamp(8.0, 12.0), // Ensure valid font size
            fontWeight: FontWeight.bold,
          ),
        ),
        textDirection: TextDirection.ltr,
      );

      textPainter.layout();
      
      // Validate text dimensions
      if (!textPainter.width.isFinite || !textPainter.height.isFinite || 
          textPainter.width <= 0 || textPainter.height <= 0) {
        return;
      }
      
      // Calculate background rectangle with validation
      final bgX = (position.dx - 2).clamp(-1000, 10000);
      final bgY = (position.dy - textPainter.height - 2).clamp(-1000, 10000);
      final bgWidth = (textPainter.width + 4).clamp(1, 1000);
      final bgHeight = (textPainter.height + 4).clamp(1, 100);
      
      final bgRect = Rect.fromLTWH(bgX, bgY, bgWidth, bgHeight);
      
      // Draw background
      canvas.drawRect(
        bgRect,
        Paint()..color = Colors.white.withOpacity(0.9),
      );
      
      // Calculate text position with validation
      final textX = position.dx.clamp(-1000, 10000);
      final textY = (position.dy - textPainter.height).clamp(-1000, 10000);
      
      // Draw text
      textPainter.paint(canvas, Offset(textX, textY));
    } catch (e) {
      print('❌ Error drawing shape label for ${shape.name}: $e');
    }
  }

  // ✅ ENHANCED: World to canvas coordinate conversion with validation
  Offset? _worldToCanvasCoordinates(double worldX, double worldY, Size size, MapData mapData) {
    try {
      // Validate input coordinates
      if (!worldX.isFinite || !worldY.isFinite) {
        print('⚠️ Invalid world coordinates: ($worldX, $worldY)');
        return null;
      }

      final canvasWidth = size.width;
      final canvasHeight = size.height;
      
      // Validate canvas size
      if (!canvasWidth.isFinite || !canvasHeight.isFinite || canvasWidth <= 0 || canvasHeight <= 0) {
        print('⚠️ Invalid canvas size: ${canvasWidth}x${canvasHeight}');
        return null;
      }
      
      final mapWidth = mapData.info.width.toDouble();
      final mapHeight = mapData.info.height.toDouble();
      final resolution = mapData.info.resolution;
      final originX = mapData.info.origin.x;
      final originY = mapData.info.origin.y;
      
      // Validate map data
      if (!mapWidth.isFinite || !mapHeight.isFinite || !resolution.isFinite || 
          !originX.isFinite || !originY.isFinite || 
          mapWidth <= 0 || mapHeight <= 0 || resolution <= 0) {
        print('⚠️ Invalid map data: ${mapWidth}x${mapHeight}, res: $resolution, origin: ($originX, $originY)');
        return null;
      }
      
      // Convert world coordinates to map pixel coordinates
      final mapPixelX = (worldX - originX) / resolution;
      final mapPixelY = mapHeight - ((worldY - originY) / resolution);
      
      // Validate intermediate calculations
      if (!mapPixelX.isFinite || !mapPixelY.isFinite) {
        print('⚠️ Invalid map pixel coordinates: ($mapPixelX, $mapPixelY)');
        return null;
      }
      
      // Convert map pixel coordinates to canvas coordinates
      final canvasX = (mapPixelX / mapWidth) * canvasWidth;
      final canvasY = (mapPixelY / mapHeight) * canvasHeight;
      
      // Validate final coordinates
      if (!canvasX.isFinite || !canvasY.isFinite) {
        print('⚠️ Invalid canvas coordinates: ($canvasX, $canvasY)');
        return null;
      }
      
      return Offset(canvasX, canvasY);
    } catch (e) {
      print('❌ Error in coordinate conversion: $e');
      return null;
    }
  }

  // ✅ NEW: Draw loading overlay
  void _drawLoadingOverlay(Canvas canvas, Size size) {
    final overlayPaint = Paint()
      ..color = Colors.white.withOpacity(0.7);
    
    canvas.drawRect(
      Rect.fromLTWH(0, 0, size.width / mapScale, size.height / mapScale),
      overlayPaint,
    );
    
    // Draw loading spinner
    final center = Offset(
      (size.width / mapScale) / 2,
      (size.height / mapScale) / 2,
    );
    
    final spinnerPaint = Paint()
      ..color = Colors.blue
      ..style = PaintingStyle.stroke
      ..strokeWidth = 3.0;
    
    canvas.drawCircle(center, 20, spinnerPaint);
  }

  // ✅ ENHANCED: Draw map background with better info
  void _drawMapBackground(Canvas canvas, Size size, MapData mapData) {
    final backgroundPaint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.fill;

    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;

    canvas.drawRect(
      Rect.fromLTWH(0, 0, adjustedWidth, adjustedHeight),
      backgroundPaint,
    );

    // Draw border
    final borderPaint = Paint()
      ..color = Colors.grey.shade400
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;

    canvas.drawRect(
      Rect.fromLTWH(0, 0, adjustedWidth, adjustedHeight),
      borderPaint,
    );

    // Draw enhanced info text
    final textPainter = TextPainter(
      text: TextSpan(
        text: 'Interactive Map\n'
               '${mapData.info.width}x${mapData.info.height} @ ${mapData.info.resolution.toStringAsFixed(3)}m/px\n'
               '${mapData.shapes.length} locations • ${waypoints.length} waypoints\n'
               'Click to add waypoints',
        style: TextStyle(
          color: Colors.grey.shade600,
          fontSize: 14,
        ),
      ),
      textAlign: TextAlign.center,
      textDirection: TextDirection.ltr,
    );

    textPainter.layout();
    textPainter.paint(
      canvas,
      Offset(
        adjustedWidth / 2 - textPainter.width / 2,
        adjustedHeight / 2 - textPainter.height / 2,
      ),
    );
  }

  // ✅ ENHANCED: Coordinate grid with real-world measurements
  void _drawCoordinateGrid(Canvas canvas, Size size, MapData mapData) {
    final paint = Paint()
      ..color = Colors.grey.shade300
      ..strokeWidth = 0.5;

    const gridSpacingMeters = 1.0; // 1 meter grid
    final gridSpacingPixels = gridSpacingMeters / mapData.info.resolution;
    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;

    // Draw vertical lines
    for (double x = 0; x < adjustedWidth; x += gridSpacingPixels) {
      canvas.drawLine(
        Offset(x, 0),
        Offset(x, adjustedHeight),
        paint,
      );
    }

    // Draw horizontal lines
    for (double y = 0; y < adjustedHeight; y += gridSpacingPixels) {
      canvas.drawLine(
        Offset(0, y),
        Offset(adjustedWidth, y),
        paint,
      );
    }

    // Draw coordinate labels every 5 meters
    final textPaint = TextPainter(textDirection: TextDirection.ltr);

    for (double x = 0; x < adjustedWidth; x += gridSpacingPixels * 5) {
      for (double y = 0; y < adjustedHeight; y += gridSpacingPixels * 5) {
        // Convert to world coordinates for labeling
        final worldX = mapData.info.origin.x + (x * mapData.info.resolution);
        final worldY = mapData.info.origin.y + ((mapData.info.height - y) * mapData.info.resolution);
        
        textPaint.text = TextSpan(
          text: '(${worldX.toStringAsFixed(1)},${worldY.toStringAsFixed(1)})',
          style: TextStyle(color: Colors.grey.shade600, fontSize: 8),
        );
        textPaint.layout();
        textPaint.paint(canvas, Offset(x + 2, y + 2));
      }
    }
  }

  void _drawWaypoints(Canvas canvas, Size size, MapData mapData) {
    for (int i = 0; i < waypoints.length; i++) {
      try {
        final waypoint = waypoints[i];
        final coords = waypoint['coordinates'] as Map<String, dynamic>;

        final worldX = coords['x'] is double
            ? coords['x'] as double
            : (coords['x'] as num?)?.toDouble() ?? 0.0;
        final worldY = coords['y'] is double
            ? coords['y'] as double
            : (coords['y'] as num?)?.toDouble() ?? 0.0;

        final canvasPos = _worldToCanvasCoordinates(worldX, worldY, size, mapData);
        
        if (canvasPos != null) {
          _drawWaypoint(canvas, canvasPos, waypoint, i + 1);
        }
      } catch (e) {
        print('❌ Error drawing waypoint $i: $e');
      }
    }
  }

  void _drawWaypoint(Canvas canvas, Offset center, Map<String, dynamic> waypoint, int stepNumber) {
    final type = waypoint['type'] as String;
    final color = _getWaypointTypeColor(type);

    // Pulsing outer circle
    final pulseScale = 1.0 + (pulseAnimation.value * 0.3);
    final outerPaint = Paint()
      ..color = color.withOpacity(0.3)
      ..style = PaintingStyle.fill;

    canvas.drawCircle(center, 20 * pulseScale, outerPaint);

    // Main circle
    final mainPaint = Paint()
      ..color = color
      ..style = PaintingStyle.fill;

    canvas.drawCircle(center, 15, mainPaint);

    // Step number
    final textPainter = TextPainter(
      text: TextSpan(
        text: stepNumber.toString(),
        style: TextStyle(
          color: Colors.white,
          fontWeight: FontWeight.bold,
          fontSize: 12,
        ),
      ),
      textDirection: TextDirection.ltr,
    );

    textPainter.layout();
    textPainter.paint(
      canvas,
      Offset(
        center.dx - textPainter.width / 2,
        center.dy - textPainter.height / 2,
      ),
    );

    // Draw connection line to next waypoint
    if (stepNumber < waypoints.length) {
      try {
        final nextWaypoint = waypoints[stepNumber];
        final nextCoords = nextWaypoint['coordinates'] as Map<String, dynamic>;

        // Safely convert next coordinates
        final nextWorldX = nextCoords['x'] is double
            ? nextCoords['x'] as double
            : (nextCoords['x'] as num?)?.toDouble() ?? 0.0;
        final nextWorldY = nextCoords['y'] is double
            ? nextCoords['y'] as double
            : (nextCoords['y'] as num?)?.toDouble() ?? 0.0;

        // Convert next waypoint world coordinates to canvas coordinates
        final nextCanvasPos = _worldToCanvasCoordinates(nextWorldX, nextWorldY, canvasSize, mapData);
        
        if (nextCanvasPos != null) {
          final linePaint = Paint()
            ..color = color.withOpacity(0.6)
            ..strokeWidth = 2
            ..style = PaintingStyle.stroke;

          canvas.drawLine(center, nextCanvasPos, linePaint);

          // Draw arrow at the end
          _drawArrow(canvas, center, nextCanvasPos, color);
        }
      } catch (e) {
        print('❌ Error drawing connection line: $e');
      }
    }
  }

  void _drawArrow(Canvas canvas, Offset start, Offset end, Color color) {
    final direction = (end - start).direction;
    const arrowLength = 10.0;
    const arrowAngle = math.pi / 6;

    final arrowPoint1 = end +
        Offset(
          arrowLength * math.cos(direction + math.pi - arrowAngle),
          arrowLength * math.sin(direction + math.pi - arrowAngle),
        );

    final arrowPoint2 = end +
        Offset(
          arrowLength * math.cos(direction + math.pi + arrowAngle),
          arrowLength * math.sin(direction + math.pi + arrowAngle),
        );

    final arrowPaint = Paint()
      ..color = color
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    canvas.drawLine(end, arrowPoint1, arrowPaint);
    canvas.drawLine(end, arrowPoint2, arrowPaint);
  }

  void _drawSelectedCoordinate(Canvas canvas) {
    if (selectedCoordinate == null) return;

    final paint = Paint()
      ..color = Colors.red
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;

    canvas.drawCircle(selectedCoordinate!, 10, paint);

    // Crosshair
    canvas.drawLine(
      Offset(selectedCoordinate!.dx - 15, selectedCoordinate!.dy),
      Offset(selectedCoordinate!.dx + 15, selectedCoordinate!.dy),
      paint,
    );
    canvas.drawLine(
      Offset(selectedCoordinate!.dx, selectedCoordinate!.dy - 15),
      Offset(selectedCoordinate!.dx, selectedCoordinate!.dy + 15),
      paint,
    );
  }

  Color _getWaypointTypeColor(String type) {
    switch (type) {
      case 'home':
        return Colors.green;
      case 'pickup':
        return Colors.blue;
      case 'drop':
        return Colors.orange;
      case 'charging':
        return Colors.yellow.shade700;
      case 'waypoint':
        return Colors.purple;
      default:
        return Colors.grey;
    }
  }

  /// Draw error state when painting fails
  void _drawErrorState(Canvas canvas, Size size) {
    // Clear canvas with error background
    final errorPaint = Paint()
      ..color = Colors.red.shade50
      ..style = PaintingStyle.fill;
    
    canvas.drawRect(
      Rect.fromLTWH(0, 0, size.width, size.height),
      errorPaint,
    );
    
    // Draw error border
    final borderPaint = Paint()
      ..color = Colors.red
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;
    
    canvas.drawRect(
      Rect.fromLTWH(0, 0, size.width, size.height),
      borderPaint,
    );
    
    // Draw error icon and text
    final center = Offset(size.width / 2, size.height / 2);
    
    // Error icon (triangle with exclamation)
    final iconPaint = Paint()
      ..color = Colors.red
      ..style = PaintingStyle.fill;
    
    final iconPath = Path();
    iconPath.moveTo(center.dx, center.dy - 30);
    iconPath.lineTo(center.dx - 25, center.dy + 20);
    iconPath.lineTo(center.dx + 25, center.dy + 20);
    iconPath.close();
    
    canvas.drawPath(iconPath, iconPaint);
    
    // Exclamation mark
    final exclamationPaint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.fill;
    
    // Exclamation line
    canvas.drawRect(
      Rect.fromCenter(
        center: Offset(center.dx, center.dy - 5),
        width: 4,
        height: 20,
      ),
      exclamationPaint,
    );
    
    // Exclamation dot
    canvas.drawCircle(
      Offset(center.dx, center.dy + 12),
      2,
      exclamationPaint,
    );
    
    // Error text
    final textPainter = TextPainter(
      text: TextSpan(
        text: 'Map Rendering Error\nCheck console for details',
        style: TextStyle(
          color: Colors.red.shade700,
          fontSize: 16,
          fontWeight: FontWeight.bold,
        ),
      ),
      textAlign: TextAlign.center,
      textDirection: TextDirection.ltr,
    );
    
    textPainter.layout();
    textPainter.paint(
      canvas,
      Offset(
        center.dx - textPainter.width / 2,
        center.dy + 40,
      ),
    );
  }

  /// Parse color from string format (hex, rgb, or color name)
  Color _parseShapeColor(String colorString) {
    try {
      final cleanColor = colorString.trim();
      
      // Handle hex colors with # prefix
      if (cleanColor.startsWith('#')) {
        final hexString = cleanColor.substring(1);
        if (hexString.length == 6) {
          // #RRGGBB format
          final colorValue = int.parse('FF$hexString', radix: 16);
          return Color(colorValue);
        } else if (hexString.length == 8) {
          // #AARRGGBB format
          final colorValue = int.parse(hexString, radix: 16);
          return Color(colorValue);
        }
      }
      
      // Handle hex colors without # prefix (like FF9C27B0)
      if (RegExp(r'^[0-9A-Fa-f]{6}
      
      // Handle RGB format rgb(r,g,b)
      final lowerColor = cleanColor.toLowerCase();
      if (lowerColor.startsWith('rgb(') && lowerColor.endsWith(')')) {
        final rgbString = lowerColor.substring(4, lowerColor.length - 1);
        final parts = rgbString.split(',').map((s) => s.trim()).toList();
        if (parts.length == 3) {
          final r = (int.tryParse(parts[0]) ?? 0).clamp(0, 255);
          final g = (int.tryParse(parts[1]) ?? 0).clamp(0, 255);
          final b = (int.tryParse(parts[2]) ?? 0).clamp(0, 255);
          return Color.fromARGB(255, r, g, b);
        }
      }
      
      // Handle RGBA format rgba(r,g,b,a)
      if (lowerColor.startsWith('rgba(') && lowerColor.endsWith(')')) {
        final rgbaString = lowerColor.substring(5, lowerColor.length - 1);
        final parts = rgbaString.split(',').map((s) => s.trim()).toList();
        if (parts.length == 4) {
          final r = (int.tryParse(parts[0]) ?? 0).clamp(0, 255);
          final g = (int.tryParse(parts[1]) ?? 0).clamp(0, 255);
          final b = (int.tryParse(parts[2]) ?? 0).clamp(0, 255);
          final a = ((double.tryParse(parts[3]) ?? 1.0) * 255).clamp(0, 255);
          return Color.fromARGB(a.toInt(), r, g, b);
        }
      }
      
      // Handle named colors
      switch (lowerColor) {
        case 'red':
          return Colors.red;
        case 'green':
          return Colors.green;
        case 'blue':
          return Colors.blue;
        case 'yellow':
          return Colors.yellow;
        case 'orange':
          return Colors.orange;
        case 'purple':
          return Colors.purple;
        case 'pink':
          return Colors.pink;
        case 'cyan':
          return Colors.cyan;
        case 'brown':
          return Colors.brown;
        case 'grey':
        case 'gray':
          return Colors.grey;
        case 'black':
          return Colors.black;
        case 'white':
          return Colors.white;
        case 'transparent':
          return Colors.transparent;
        default:
          print('⚠️ Unknown color format: $colorString, using default blue');
          return Colors.blue;
      }
    } catch (e) {
      print('❌ Error parsing color "$colorString": $e, using default blue');
      return Colors.blue;
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}).hasMatch(cleanColor)) {
        // RRGGBB format without #
        final colorValue = int.parse('FF$cleanColor', radix: 16);
        return Color(colorValue);
      } else if (RegExp(r'^[0-9A-Fa-f]{8}
      
      // Handle RGB format rgb(r,g,b)
      if (cleanColor.startsWith('rgb(') && cleanColor.endsWith(')')) {
        final rgbString = cleanColor.substring(4, cleanColor.length - 1);
        final parts = rgbString.split(',').map((s) => s.trim()).toList();
        if (parts.length == 3) {
          final r = int.tryParse(parts[0]) ?? 0;
          final g = int.tryParse(parts[1]) ?? 0;
          final b = int.tryParse(parts[2]) ?? 0;
          return Color.fromARGB(255, r, g, b);
        }
      }
      
      // Handle RGBA format rgba(r,g,b,a)
      if (cleanColor.startsWith('rgba(') && cleanColor.endsWith(')')) {
        final rgbaString = cleanColor.substring(5, cleanColor.length - 1);
        final parts = rgbaString.split(',').map((s) => s.trim()).toList();
        if (parts.length == 4) {
          final r = int.tryParse(parts[0]) ?? 0;
          final g = int.tryParse(parts[1]) ?? 0;
          final b = int.tryParse(parts[2]) ?? 0;
          final a = (double.tryParse(parts[3]) ?? 1.0) * 255;
          return Color.fromARGB(a.toInt(), r, g, b);
        }
      }
      
      // Handle named colors
      switch (cleanColor) {
        case 'red':
          return Colors.red;
        case 'green':
          return Colors.green;
        case 'blue':
          return Colors.blue;
        case 'yellow':
          return Colors.yellow;
        case 'orange':
          return Colors.orange;
        case 'purple':
          return Colors.purple;
        case 'pink':
          return Colors.pink;
        case 'cyan':
          return Colors.cyan;
        case 'brown':
          return Colors.brown;
        case 'grey':
        case 'gray':
          return Colors.grey;
        case 'black':
          return Colors.black;
        case 'white':
          return Colors.white;
        case 'transparent':
          return Colors.transparent;
        default:
          print('⚠️ Unknown color format: $colorString, using default blue');
          return Colors.blue;
      }
    } catch (e) {
      print('❌ Error parsing color "$colorString": $e, using default blue');
      return Colors.blue;
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}).hasMatch(cleanColor)) {
        // AARRGGBB format without #
        final colorValue = int.parse(cleanColor, radix: 16);
        return Color(colorValue);
      }
      
      // Handle 0x format
      if (cleanColor.toLowerCase().startsWith('0x')) {
        final colorValue = int.parse(cleanColor, radix: 16);
        return Color(colorValue);
      }
      
      // Handle RGB format rgb(r,g,b)
      if (cleanColor.startsWith('rgb(') && cleanColor.endsWith(')')) {
        final rgbString = cleanColor.substring(4, cleanColor.length - 1);
        final parts = rgbString.split(',').map((s) => s.trim()).toList();
        if (parts.length == 3) {
          final r = int.tryParse(parts[0]) ?? 0;
          final g = int.tryParse(parts[1]) ?? 0;
          final b = int.tryParse(parts[2]) ?? 0;
          return Color.fromARGB(255, r, g, b);
        }
      }
      
      // Handle RGBA format rgba(r,g,b,a)
      if (cleanColor.startsWith('rgba(') && cleanColor.endsWith(')')) {
        final rgbaString = cleanColor.substring(5, cleanColor.length - 1);
        final parts = rgbaString.split(',').map((s) => s.trim()).toList();
        if (parts.length == 4) {
          final r = int.tryParse(parts[0]) ?? 0;
          final g = int.tryParse(parts[1]) ?? 0;
          final b = int.tryParse(parts[2]) ?? 0;
          final a = (double.tryParse(parts[3]) ?? 1.0) * 255;
          return Color.fromARGB(a.toInt(), r, g, b);
        }
      }
      
      // Handle named colors
      switch (cleanColor) {
        case 'red':
          return Colors.red;
        case 'green':
          return Colors.green;
        case 'blue':
          return Colors.blue;
        case 'yellow':
          return Colors.yellow;
        case 'orange':
          return Colors.orange;
        case 'purple':
          return Colors.purple;
        case 'pink':
          return Colors.pink;
        case 'cyan':
          return Colors.cyan;
        case 'brown':
          return Colors.brown;
        case 'grey':
        case 'gray':
          return Colors.grey;
        case 'black':
          return Colors.black;
        case 'white':
          return Colors.white;
        case 'transparent':
          return Colors.transparent;
        default:
          print('⚠️ Unknown color format: $colorString, using default blue');
          return Colors.blue;
      }
    } catch (e) {
      print('❌ Error parsing color "$colorString": $e, using default blue');
      return Colors.blue;
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}