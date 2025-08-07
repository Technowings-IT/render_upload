// widgets/interactive_map_order_creator_tablet.dart - Tablet Optimized Interactive Map-based Order Creation
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

  // Map loading state
  MapData? _loadedMapData;
  bool _isLoadingMap = false;
  String? _mapLoadError;
  String? _mapLoadSource;
  
  // Enhanced map visualization
  bool _showMapShapes = true;
  bool _showOccupancyGrid = true;
  double _mapOpacity = 0.7;

  // Form controllers
  final TextEditingController _orderNameController = TextEditingController();
  final TextEditingController _priorityController = TextEditingController(text: '0');
  final TextEditingController _waypointNameController = TextEditingController();

  // Tablet detection
  bool get isTablet => MediaQuery.of(context).size.width >= 768;

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _orderNameController.text = 'Order ${DateTime.now().millisecondsSinceEpoch}';

    print('🗺️ Interactive Map Order Creator initialized');
    print('📊 Map dimensions: ${widget.mapData.info.width}x${widget.mapData.info.height}');
    print('📍 Map shapes: ${widget.mapData.shapes.length}');
    print('🎯 Device ID: ${widget.deviceId}');
    
    _loadDeviceMapData();
    
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _testLayout();
    });
  }

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
            _loadedMapData = widget.mapData;
            _mapLoadSource = 'fallback_provided';
            _mapLoadError = 'Parse error, using provided map';
          });
        }
      } else {
        print('⚠️ No map data found, using provided map');
        setState(() {
          _loadedMapData = widget.mapData;
          _mapLoadSource = 'provided_default';
          _mapLoadError = null;
        });
      }
    } catch (e) {
      print('❌ Error loading map data: $e');
      setState(() {
        _loadedMapData = widget.mapData;
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
        title: Text(
          'Interactive Order Creator', 
          style: TextStyle(fontSize: isTablet ? 22 : 18),
        ),
        backgroundColor: Colors.blue,
        foregroundColor: Colors.white,
        toolbarHeight: isTablet ? 70 : 56,
        actions: _buildAppBarActions(),
      ),
      body: SafeArea(
        child: Column(
          children: [
            if (_isLoadingMap) _buildMapLoadingBanner(),
            if (_mapLoadError != null) _buildMapErrorBanner(),
            
            Expanded(
              child: Row(
                children: [
                  // Map Canvas
                  Expanded(
                    flex: isTablet ? 7 : 3,
                    child: Container(
                      color: Colors.grey.shade100,
                      child: _buildDirectMapCanvas(),
                    ),
                  ),

                  // Control Panel
                  Expanded(
                    flex: isTablet ? 4 : 2,
                    child: Container(
                      decoration: BoxDecoration(
                        border: Border(left: BorderSide(color: Colors.grey.shade300)),
                      ),
                      child: Column(
                        children: [
                          Expanded(child: _buildControlPanel()),
                          Container(
                            height: isTablet ? 100 : 80,
                            child: _buildBottomActionBar(),
                          ),
                        ],
                      ),
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

  List<Widget> _buildAppBarActions() {
    return [
      if (_mapLoadSource != null)
        Container(
          margin: EdgeInsets.only(right: 12),
          padding: EdgeInsets.symmetric(horizontal: 12, vertical: 6),
          decoration: BoxDecoration(
            color: _getMapSourceColor(_mapLoadSource!),
            borderRadius: BorderRadius.circular(16),
          ),
          child: Text(
            _mapLoadSource!.toUpperCase(),
            style: TextStyle(fontSize: isTablet ? 12 : 10, color: Colors.white),
          ),
        ),
      IconButton(
        iconSize: isTablet ? 28 : 24,
        icon: Icon(_showGrid ? Icons.grid_on : Icons.grid_off),
        onPressed: () => setState(() => _showGrid = !_showGrid),
        tooltip: 'Toggle Grid',
      ),
      IconButton(
        iconSize: isTablet ? 28 : 24,
        icon: Icon(_showCoordinates ? Icons.gps_fixed : Icons.gps_off),
        onPressed: () => setState(() => _showCoordinates = !_showCoordinates),
        tooltip: 'Toggle Coordinates',
      ),
      PopupMenuButton<String>(
        iconSize: isTablet ? 28 : 24,
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
                SizedBox(width: 12),
                Text('Map Shapes', style: TextStyle(fontSize: isTablet ? 16 : 14)),
              ],
            ),
          ),
          PopupMenuItem(
            value: 'occupancy',
            child: Row(
              children: [
                Icon(_showOccupancyGrid ? Icons.check_box : Icons.check_box_outline_blank),
                SizedBox(width: 12),
                Text('Occupancy Grid', style: TextStyle(fontSize: isTablet ? 16 : 14)),
              ],
            ),
          ),
          PopupMenuDivider(),
          PopupMenuItem(
            value: 'reload',
            child: Row(
              children: [
                Icon(Icons.refresh),
                SizedBox(width: 12),
                Text('Reload Map', style: TextStyle(fontSize: isTablet ? 16 : 14)),
              ],
            ),
          ),
        ],
      ),
      SizedBox(width: isTablet ? 16 : 8),
    ];
  }

  Widget _buildMapLoadingBanner() {
    return Container(
      width: double.infinity,
      padding: EdgeInsets.all(isTablet ? 16 : 8),
      decoration: BoxDecoration(
        color: Colors.blue.shade100,
        border: Border(bottom: BorderSide(color: Colors.blue.shade300)),
      ),
      child: Row(
        children: [
          SizedBox(
            width: isTablet ? 20 : 16,
            height: isTablet ? 20 : 16,
            child: CircularProgressIndicator(strokeWidth: 2),
          ),
          SizedBox(width: isTablet ? 16 : 12),
          Text(
            'Loading map data for ${widget.deviceId}...',
            style: TextStyle(fontSize: isTablet ? 16 : 14),
          ),
        ],
      ),
    );
  }

  Widget _buildMapErrorBanner() {
    return Container(
      width: double.infinity,
      padding: EdgeInsets.all(isTablet ? 16 : 8),
      decoration: BoxDecoration(
        color: Colors.orange.shade100,
        border: Border(bottom: BorderSide(color: Colors.orange.shade300)),
      ),
      child: Row(
        children: [
          Icon(
            Icons.warning, 
            size: isTablet ? 20 : 16, 
            color: Colors.orange.shade700
          ),
          SizedBox(width: isTablet ? 12 : 8),
          Expanded(
            child: Text(
              _mapLoadError!,
              style: TextStyle(
                color: Colors.orange.shade700,
                fontSize: isTablet ? 16 : 14,
              ),
            ),
          ),
          TextButton(
            onPressed: _loadDeviceMapData,
            style: TextButton.styleFrom(
              padding: EdgeInsets.symmetric(
                horizontal: isTablet ? 16 : 12,
                vertical: isTablet ? 12 : 8,
              ),
            ),
            child: Text(
              'Retry',
              style: TextStyle(fontSize: isTablet ? 16 : 14),
            ),
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
              Positioned.fill(
                child: GestureDetector(
                  onTapDown: _handleMapTap,
                  onScaleStart: _handleScaleStart,
                  onScaleUpdate: _handleScaleUpdate,
                  child: RepaintBoundary(
                    child: CustomPaint(
                      painter: TabletOptimizedMapPainter(
                        mapData: widget.mapData,
                        loadedMapData: _loadedMapData,
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
                        isTablet: isTablet,
                      ),
                    ),
                  ),
                ),
              ),
              
              // Debug info overlay
              _buildDebugOverlay(canvasWidth, canvasHeight),
              
              // Coordinate overlay
              if (_selectedCoordinate != null && _showCoordinates)
                _buildCoordinateOverlay(canvasWidth, canvasHeight),

              // Adding waypoint overlay
              if (_isAddingWaypoint) _buildAddingWaypointOverlay(),
            ],
          ),
        );
      },
    );
  }

  Widget _buildDebugOverlay(double canvasWidth, double canvasHeight) {
    return Positioned(
      top: isTablet ? 16 : 10,
      right: isTablet ? 16 : 10,
      child: Container(
        padding: EdgeInsets.all(isTablet ? 12 : 8),
        decoration: BoxDecoration(
          color: Colors.black54,
          borderRadius: BorderRadius.circular(8),
        ),
        child: Text(
          'Canvas: ${canvasWidth.toInt()}x${canvasHeight.toInt()}\n'
          'Map: ${(_loadedMapData ?? widget.mapData).info.width}x${(_loadedMapData ?? widget.mapData).info.height}\n'
          'Scale: ${(_mapScale * 100).toInt()}%\n'
          'Waypoints: ${_waypoints.length}\n'
          'Map Shapes: ${(_loadedMapData?.shapes.length ?? 0)}\n'
          'Source: ${_mapLoadSource ?? 'none'}',
          style: TextStyle(
            color: Colors.white, 
            fontSize: isTablet ? 14 : 10,
          ),
        ),
      ),
    );
  }

  Widget _buildCoordinateOverlay(double canvasWidth, double canvasHeight) {
    return Positioned(
      left: (_selectedCoordinate!.dx + (isTablet ? 16 : 10)).clamp(0.0, canvasWidth - 160),
      top: (_selectedCoordinate!.dy - (isTablet ? 40 : 30)).clamp(0.0, canvasHeight - 40),
      child: Container(
        padding: EdgeInsets.symmetric(
          horizontal: isTablet ? 12 : 8,
          vertical: isTablet ? 8 : 4,
        ),
        decoration: BoxDecoration(
          color: Colors.black87,
          borderRadius: BorderRadius.circular(6),
        ),
        child: Text(
          _getMapCoordinatesText(_selectedCoordinate!),
          style: TextStyle(
            color: Colors.white, 
            fontSize: isTablet ? 14 : 12,
          ),
        ),
      ),
    );
  }

  Widget _buildAddingWaypointOverlay() {
    return Positioned.fill(
      child: Container(
        color: Colors.blue.withOpacity(0.1),
        child: Center(
          child: Container(
            padding: EdgeInsets.all(isTablet ? 24 : 16),
            decoration: BoxDecoration(
              color: Colors.white,
              borderRadius: BorderRadius.circular(12),
              boxShadow: [BoxShadow(color: Colors.black26, blurRadius: 8)],
            ),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Text(
                  'Tap on map to add ${_selectedWaypointType.toUpperCase()}',
                  style: TextStyle(
                    fontSize: isTablet ? 20 : 16,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                SizedBox(height: isTablet ? 16 : 8),
                ElevatedButton(
                  onPressed: () => setState(() => _isAddingWaypoint = false),
                  style: ElevatedButton.styleFrom(
                    padding: EdgeInsets.symmetric(
                      horizontal: isTablet ? 32 : 24,
                      vertical: isTablet ? 16 : 12,
                    ),
                  ),
                  child: Text(
                    'Cancel',
                    style: TextStyle(fontSize: isTablet ? 18 : 14),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildControlPanel() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(isTablet ? 20 : 16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildOrderDetailsSection(),
          SizedBox(height: isTablet ? 20 : 16),
          _buildWaypointTypesSection(),
          SizedBox(height: isTablet ? 20 : 16),
          _buildWaypointsListSection(),
          SizedBox(height: isTablet ? 20 : 16),
          _buildActionButtons(),
        ],
      ),
    );
  }

  Widget _buildOrderDetailsSection() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(isTablet ? 20 : 16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Order Details',
              style: TextStyle(
                fontSize: isTablet ? 22 : 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: isTablet ? 16 : 12),

            TextField(
              controller: _orderNameController,
              style: TextStyle(fontSize: isTablet ? 16 : 14),
              decoration: InputDecoration(
                labelText: 'Order Name',
                labelStyle: TextStyle(fontSize: isTablet ? 16 : 14),
                border: OutlineInputBorder(),
                prefixIcon: Icon(Icons.assignment, size: isTablet ? 24 : 20),
                contentPadding: EdgeInsets.symmetric(
                  horizontal: isTablet ? 16 : 12,
                  vertical: isTablet ? 16 : 12,
                ),
              ),
            ),

            SizedBox(height: isTablet ? 16 : 12),

            TextField(
              controller: _priorityController,
              style: TextStyle(fontSize: isTablet ? 16 : 14),
              decoration: InputDecoration(
                labelText: 'Priority (0-10)',
                labelStyle: TextStyle(fontSize: isTablet ? 16 : 14),
                border: OutlineInputBorder(),
                prefixIcon: Icon(Icons.priority_high, size: isTablet ? 24 : 20),
                contentPadding: EdgeInsets.symmetric(
                  horizontal: isTablet ? 16 : 12,
                  vertical: isTablet ? 16 : 12,
                ),
              ),
              keyboardType: TextInputType.number,
              onChanged: (value) {
                if (value.isNotEmpty && int.tryParse(value) == null) {
                  _priorityController.text = '0';
                  _priorityController.selection = TextSelection.fromPosition(
                    TextPosition(offset: _priorityController.text.length),
                  );
                }
              },
            ),

            SizedBox(height: isTablet ? 16 : 12),

            Container(
              padding: EdgeInsets.all(isTablet ? 16 : 12),
              decoration: BoxDecoration(
                color: Colors.blue.shade50,
                borderRadius: BorderRadius.circular(12),
                border: Border.all(color: Colors.blue.shade200),
              ),
              child: Row(
                children: [
                  Icon(
                    Icons.info, 
                    color: Colors.blue.shade700,
                    size: isTablet ? 24 : 20,
                  ),
                  SizedBox(width: isTablet ? 12 : 8),
                  Expanded(
                    child: Text(
                      'Device: ${widget.deviceId}\nWaypoints: ${_waypoints.length}',
                      style: TextStyle(
                        color: Colors.blue.shade700,
                        fontSize: isTablet ? 16 : 14,
                      ),
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
      {'type': 'home', 'label': 'Home', 'icon': Icons.home, 'color': Colors.green},
      {'type': 'pickup', 'label': 'Pickup', 'icon': Icons.outbox, 'color': Colors.blue},
      {'type': 'drop', 'label': 'Drop', 'icon': Icons.inbox, 'color': Colors.orange},
      {'type': 'charging', 'label': 'Charge', 'icon': Icons.battery_charging_full, 'color': Colors.yellow.shade700},
      {'type': 'waypoint', 'label': 'Point', 'icon': Icons.place, 'color': Colors.purple},
    ];

    return Card(
      child: Padding(
        padding: EdgeInsets.all(isTablet ? 20 : 12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Add Waypoint',
              style: TextStyle(
                fontSize: isTablet ? 20 : 16,
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: isTablet ? 12 : 8),
            Text(
              'Select type and tap on map:',
              style: TextStyle(
                color: Colors.grey.shade600, 
                fontSize: isTablet ? 16 : 12,
              ),
            ),
            SizedBox(height: isTablet ? 16 : 8),
            
            LayoutBuilder(
              builder: (context, constraints) {
                final buttonsPerRow = isTablet ? 2 : 3;
                final buttonWidth = (constraints.maxWidth - (buttonsPerRow - 1) * 8) / buttonsPerRow;
                
                return Wrap(
                  spacing: 8,
                  runSpacing: 8,
                  children: waypointTypes.map((type) => _buildWaypointTypeButton(type, buttonWidth)).toList(),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildWaypointTypeButton(Map<String, dynamic> type, double buttonWidth) {
    final isSelected = _selectedWaypointType == type['type'];
    final color = type['color'] as Color;

    return GestureDetector(
      onTap: () {
        setState(() {
          _selectedWaypointType = type['type'];
          _isAddingWaypoint = true;
        });
      },
      child: Container(
        width: buttonWidth,
        padding: EdgeInsets.symmetric(
          horizontal: isTablet ? 12 : 6,
          vertical: isTablet ? 12 : 4,
        ),
        decoration: BoxDecoration(
          color: isSelected ? color : color.withOpacity(0.1),
          border: Border.all(color: color),
          borderRadius: BorderRadius.circular(16),
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(
              type['icon'] as IconData,
              size: isTablet ? 20 : 12,
              color: isSelected ? Colors.white : color,
            ),
            SizedBox(width: isTablet ? 8 : 2),
            Flexible(
              child: Text(
                type['label'],
                style: TextStyle(
                  color: isSelected ? Colors.white : color,
                  fontWeight: FontWeight.w600,
                  fontSize: isTablet ? 16 : 9,
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
        padding: EdgeInsets.all(isTablet ? 20 : 12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Expanded(
                  child: Text(
                    'Waypoints (${_waypoints.length})',
                    style: TextStyle(
                      fontSize: isTablet ? 20 : 16,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                ),
                if (_waypoints.isNotEmpty)
                  IconButton(
                    icon: Icon(Icons.clear_all, size: isTablet ? 24 : 18),
                    onPressed: _clearAllWaypoints,
                    tooltip: 'Clear All',
                    padding: EdgeInsets.all(isTablet ? 8 : 2),
                    constraints: BoxConstraints(
                      minWidth: isTablet ? 40 : 28,
                      minHeight: isTablet ? 40 : 28,
                    ),
                  ),
              ],
            ),
            SizedBox(height: isTablet ? 12 : 8),
            
            Container(
              height: isTablet ? 300 : 200,
              child: _waypoints.isEmpty
                  ? Center(
                      child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          Icon(Icons.touch_app, size: isTablet ? 36 : 24, color: Colors.grey),
                          SizedBox(height: isTablet ? 12 : 6),
                          Text(
                            'No waypoints added yet',
                            style: TextStyle(
                              color: Colors.grey.shade600,
                              fontSize: isTablet ? 16 : 11,
                            ),
                          ),
                          Text(
                            'Select a type and tap on the map',
                            style: TextStyle(
                              color: Colors.grey.shade500,
                              fontSize: isTablet ? 14 : 9,
                            ),
                          ),
                        ],
                      ),
                    )
                  : ListView.builder(
                      itemCount: _waypoints.length,
                      itemBuilder: (context, index) => _buildWaypointListItem(index),
                    ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildWaypointListItem(int index) {
    final waypoint = _waypoints[index];
    final color = _getWaypointTypeColor(waypoint['type']);

    return Card(
      margin: EdgeInsets.symmetric(vertical: isTablet ? 4 : 1),
      child: ListTile(
        dense: !isTablet,
        contentPadding: EdgeInsets.symmetric(
          horizontal: isTablet ? 16 : 6,
          vertical: isTablet ? 8 : 1,
        ),
        leading: Container(
          width: isTablet ? 44 : 28,
          height: isTablet ? 44 : 28,
          decoration: BoxDecoration(
            color: color,
            borderRadius: BorderRadius.circular(8),
          ),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Text(
                '${index + 1}',
                style: TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  fontSize: isTablet ? 12 : 8,
                ),
              ),
              Icon(
                _getWaypointTypeIcon(waypoint['type']),
                color: Colors.white,
                size: isTablet ? 16 : 10,
              ),
            ],
          ),
        ),
        title: Text(
          waypoint['name'],
          style: TextStyle(
            fontWeight: FontWeight.w600,
            fontSize: isTablet ? 16 : 11,
          ),
          maxLines: 1,
          overflow: TextOverflow.ellipsis,
        ),
        subtitle: Text(
          '${waypoint['type'].toUpperCase()}\n(${waypoint['coordinates']['x'].toStringAsFixed(1)}, ${waypoint['coordinates']['y'].toStringAsFixed(1)})',
          style: TextStyle(fontSize: isTablet ? 14 : 9),
          maxLines: 2,
        ),
        trailing: PopupMenuButton<String>(
          onSelected: (value) => _handleWaypointAction(index, value),
          itemBuilder: (context) => [
            PopupMenuItem(
              value: 'edit', 
              child: Text('Edit', style: TextStyle(fontSize: isTablet ? 16 : 11)),
            ),
            PopupMenuItem(
              value: 'duplicate', 
              child: Text('Duplicate', style: TextStyle(fontSize: isTablet ? 16 : 11)),
            ),
            PopupMenuItem(
              value: 'delete', 
              child: Text('Delete', style: TextStyle(fontSize: isTablet ? 16 : 11)),
            ),
          ],
          icon: Icon(Icons.more_vert, size: isTablet ? 20 : 14),
        ),
      ),
    );
  }

  Widget _buildActionButtons() {
    return Column(
      children: [
        SizedBox(
          width: double.infinity,
          child: ElevatedButton.icon(
            onPressed: _waypoints.isNotEmpty && !_isCreatingOrder ? _createOrder : null,
            icon: _isCreatingOrder
                ? SizedBox(
                    width: isTablet ? 16 : 12,
                    height: isTablet ? 16 : 12,
                    child: CircularProgressIndicator(strokeWidth: 2))
                : Icon(Icons.send, size: isTablet ? 20 : 14),
            label: Text(
              _isCreatingOrder ? 'Creating...' : 'Create Order',
              style: TextStyle(fontSize: isTablet ? 18 : 11),
            ),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.green,
              foregroundColor: Colors.white,
              padding: EdgeInsets.symmetric(vertical: isTablet ? 16 : 6),
            ),
          ),
        ),
        SizedBox(height: isTablet ? 12 : 4),
        Row(
          children: [
            Expanded(
              child: OutlinedButton.icon(
                onPressed: _previewOrderExecution,
                icon: Icon(Icons.preview, size: isTablet ? 18 : 12),
                label: Text('Preview', style: TextStyle(fontSize: isTablet ? 16 : 10)),
                style: OutlinedButton.styleFrom(
                  padding: EdgeInsets.symmetric(vertical: isTablet ? 12 : 4),
                ),
              ),
            ),
            SizedBox(width: isTablet ? 12 : 4),
            Expanded(
              child: OutlinedButton.icon(
                onPressed: _saveOrderTemplate,
                icon: Icon(Icons.save, size: isTablet ? 18 : 12),
                label: Text('Template', style: TextStyle(fontSize: isTablet ? 16 : 10)),
                style: OutlinedButton.styleFrom(
                  padding: EdgeInsets.symmetric(vertical: isTablet ? 12 : 4),
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
      padding: EdgeInsets.symmetric(
        horizontal: isTablet ? 20 : 12,
        vertical: isTablet ? 16 : 8,
      ),
      decoration: BoxDecoration(
        border: Border(top: BorderSide(color: Colors.grey.shade300)),
        color: Colors.white,
      ),
      child: Row(
        children: [
          Text(
            'Scale: ${(_mapScale * 100).toInt()}%', 
            style: TextStyle(fontSize: isTablet ? 16 : 11),
          ),
          SizedBox(width: isTablet ? 16 : 8),
          Expanded(
            child: Slider(
              value: _mapScale,
              min: 0.5,
              max: 3.0,
              divisions: 25,
              onChanged: (value) => setState(() => _mapScale = value),
            ),
          ),
          IconButton(
            icon: Icon(Icons.center_focus_strong, size: isTablet ? 24 : 18),
            onPressed: _centerMap,
            tooltip: 'Center Map',
            padding: EdgeInsets.all(isTablet ? 8 : 4),
            constraints: BoxConstraints(
              minWidth: isTablet ? 48 : 32,
              minHeight: isTablet ? 48 : 32,
            ),
          ),
          IconButton(
            icon: Icon(Icons.refresh, size: isTablet ? 24 : 18),
            onPressed: _resetMapView,
            tooltip: 'Reset View',
            padding: EdgeInsets.all(isTablet ? 8 : 4),
            constraints: BoxConstraints(
              minWidth: isTablet ? 48 : 32,
              minHeight: isTablet ? 48 : 32,
            ),
          ),
        ],
      ),
    );
  }

  // Event Handlers
  void _handleMapTap(TapDownDetails details) {
    if (!_isAddingWaypoint) {
      setState(() => _selectedCoordinate = details.localPosition);
      return;
    }
    final mapCoordinates = _screenToMapCoordinates(details.localPosition);
    _showWaypointCreationDialog(mapCoordinates);
  }

  void _handleScaleStart(ScaleStartDetails details) {}

  void _handleScaleUpdate(ScaleUpdateDetails details) {
    setState(() {
      _mapScale = (_mapScale * details.scale).clamp(0.5, 3.0);
      _mapOffset += details.focalPointDelta;
    });
  }

  void _handleWaypointAction(int index, String action) {
    switch (action) {
      case 'edit': _editWaypoint(index); break;
      case 'duplicate': _duplicateWaypoint(index); break;
      case 'delete': _deleteWaypoint(index); break;
    }
  }

  // Waypoint Management
  void _showWaypointCreationDialog(Map<String, double> coordinates) {
    _waypointNameController.clear();
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text(
          'Add ${_selectedWaypointType.toUpperCase()}',
          style: TextStyle(fontSize: isTablet ? 22 : 18),
        ),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Coordinates: (${coordinates['x']!.toStringAsFixed(2)}, ${coordinates['y']!.toStringAsFixed(2)})',
              style: TextStyle(fontSize: isTablet ? 16 : 14),
            ),
            SizedBox(height: isTablet ? 20 : 16),
            TextField(
              controller: _waypointNameController,
              style: TextStyle(fontSize: isTablet ? 16 : 14),
              decoration: InputDecoration(
                labelText: 'Waypoint Name',
                hintText: 'e.g., Pickup Station A',
                border: OutlineInputBorder(),
                contentPadding: EdgeInsets.symmetric(
                  horizontal: isTablet ? 16 : 12,
                  vertical: isTablet ? 16 : 12,
                ),
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
            child: Text('Cancel', style: TextStyle(fontSize: isTablet ? 16 : 14)),
          ),
          ElevatedButton(
            onPressed: () {
              _addWaypoint(coordinates, _waypointNameController.text.trim());
              Navigator.of(context).pop();
              setState(() => _isAddingWaypoint = false);
            },
            child: Text('Add', style: TextStyle(fontSize: isTablet ? 16 : 14)),
          ),
        ],
      ),
    );
  }

  void _addWaypoint(Map<String, double> coordinates, String name) {
    if (!_isValidMapCoordinate(coordinates['x']!, coordinates['y']!)) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Warning: Coordinates may be outside valid map bounds'),
          backgroundColor: Colors.orange,
        ),
      );
    }
    
    final waypoint = {
      'id': DateTime.now().millisecondsSinceEpoch.toString(),
      'name': name.isNotEmpty ? name : '${_selectedWaypointType.toUpperCase()} ${_waypoints.length + 1}',
      'type': _selectedWaypointType,
      'coordinates': coordinates,
      'orientation': 0.0,
      'metadata': {
        'createdAt': DateTime.now().toIso8601String(),
        'color': '#${_getWaypointTypeColor(_selectedWaypointType).value.toRadixString(16).padLeft(8, '0')}',
        'mapResolution': widget.mapData.info.resolution,
        'coordinateFrame': 'map',
        'validationStatus': _isValidMapCoordinate(coordinates['x']!, coordinates['y']!) ? 'valid' : 'warning',
      },
    };

    setState(() => _waypoints.add(waypoint));
    _waypointAnimationController.forward().then((_) => _waypointAnimationController.reset());

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('${waypoint['name']} added'),
        backgroundColor: Colors.green,
      ),
    );
  }

  void _editWaypoint(int index) {
    final waypoint = _waypoints[index];
    _waypointNameController.text = waypoint['name'];

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Edit Waypoint', style: TextStyle(fontSize: isTablet ? 22 : 18)),
        content: TextField(
          controller: _waypointNameController,
          decoration: InputDecoration(labelText: 'Waypoint Name', border: OutlineInputBorder()),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              setState(() => _waypoints[index]['name'] = _waypointNameController.text.trim());
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
    setState(() => _waypoints.insert(index + 1, duplicatedWaypoint));
  }

  void _deleteWaypoint(int index) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Delete Waypoint'),
        content: Text('Are you sure you want to delete "${_waypoints[index]['name']}"?'),
        actions: [
          TextButton(onPressed: () => Navigator.of(context).pop(), child: Text('Cancel')),
          ElevatedButton(
            onPressed: () {
              setState(() => _waypoints.removeAt(index));
              Navigator.of(context).pop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child: Text('Delete'),
          ),
        ],
      ),
    );
  }

  void _clearAllWaypoints() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Clear All Waypoints'),
        content: Text('Are you sure you want to remove all waypoints?'),
        actions: [
          TextButton(onPressed: () => Navigator.of(context).pop(), child: Text('Cancel')),
          ElevatedButton(
            onPressed: () {
              setState(() => _waypoints.clear());
              Navigator.of(context).pop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child: Text('Clear All'),
          ),
        ],
      ),
    );
  }

  // Order Creation
  Future<void> _createOrder() async {
    setState(() => _isCreatingOrder = true);

    try {
      final priority = int.tryParse(_priorityController.text.trim()) ?? 0;
      
      for (int i = 0; i < _waypoints.length; i++) {
        final wp = _waypoints[i];
        final coords = wp['coordinates'] as Map<String, dynamic>;
        if (!_isValidMapCoordinate(coords['x'] as double, coords['y'] as double)) {
          throw Exception('Waypoint ${i + 1} has invalid coordinates');
        }
      }

      final orderData = {
        'deviceId': widget.deviceId,
        'name': _orderNameController.text.trim(),
        'priority': priority,
        'description': 'Created from interactive map with ${_waypoints.length} waypoints',
        'waypoints': _waypoints.map((wp) => {
          'name': wp['name'],
          'type': wp['type'],
          'position': {
            'x': wp['coordinates']['x'],
            'y': wp['coordinates']['y'],
            'z': 0.0,
          },
          'orientation': wp['orientation'] ?? 0.0,
          'metadata': wp['metadata'],
        }).toList(),
      };

      final response = await _apiService.createOrder(
        deviceId: widget.deviceId,
        name: orderData['name'] as String,
        waypoints: orderData['waypoints'] as List<Map<String, dynamic>>,
        priority: priority,
        description: orderData['description'] as String,
      );

      if (response['success'] == true) {
        widget.onOrderCreated(orderData);
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('✅ Order created successfully!'), backgroundColor: Colors.green),
        );
        Navigator.of(context).pop();
      } else {
        throw Exception(response['error'] ?? 'Failed to create order');
      }
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('❌ Failed to create order: $e'), backgroundColor: Colors.red),
      );
    } finally {
      setState(() => _isCreatingOrder = false);
    }
  }

  void _previewOrderExecution() {
    if (_waypoints.isEmpty) return;

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Order Execution Preview', style: TextStyle(fontSize: isTablet ? 22 : 18)),
        content: Container(
          width: isTablet ? 600 : 400,
          height: isTablet ? 450 : 300,
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'Execution Sequence:',
                style: TextStyle(fontWeight: FontWeight.bold, fontSize: isTablet ? 18 : 16),
              ),
              SizedBox(height: isTablet ? 16 : 8),
              Expanded(
                child: ListView.builder(
                  itemCount: _waypoints.length,
                  itemBuilder: (context, index) {
                    final waypoint = _waypoints[index];
                    return ListTile(
                      leading: CircleAvatar(
                        backgroundColor: _getWaypointTypeColor(waypoint['type']),
                        child: Text('${index + 1}'),
                      ),
                      title: Text(waypoint['name']),
                      subtitle: Text('${waypoint['type'].toUpperCase()}'),
                      trailing: Icon(Icons.arrow_downward),
                    );
                  },
                ),
              ),
            ],
          ),
        ),
        actions: [
          TextButton(onPressed: () => Navigator.of(context).pop(), child: Text('Close')),
        ],
      ),
    );
  }

  void _saveOrderTemplate() {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text('Order template save feature coming soon!')),
    );
  }

  // Map Utilities
  bool _isValidMapCoordinate(double x, double y) {
    const maxBound = 100.0;
    const minBound = -100.0;
    return x >= minBound && x <= maxBound && y >= minBound && y <= maxBound && x.isFinite && y.isFinite;
  }

  Map<String, double> _screenToMapCoordinates(Offset screenPosition) {
    try {
      final mapData = _loadedMapData ?? widget.mapData;
      final renderBox = context.findRenderObject() as RenderBox?;
      final canvasSize = renderBox?.size ?? Size(800, 600);
      
      final adjustedX = (screenPosition.dx - _mapOffset.dx) / _mapScale;
      final adjustedY = (screenPosition.dy - _mapOffset.dy) / _mapScale;
      
      final mapPixelX = (adjustedX / canvasSize.width) * mapData.info.width;
      final mapPixelY = (adjustedY / canvasSize.height) * mapData.info.height;
      
      final worldX = mapData.info.origin.x + (mapPixelX * mapData.info.resolution);
      final worldY = mapData.info.origin.y + ((mapData.info.height - mapPixelY) * mapData.info.resolution);
      
      return {'x': worldX, 'y': worldY};
    } catch (e) {
      return {'x': 0.0, 'y': 0.0};
    }
  }

  String _getMapCoordinatesText(Offset screenPosition) {
    final coords = _screenToMapCoordinates(screenPosition);
    return '(${coords['x']!.toStringAsFixed(3)}m, ${coords['y']!.toStringAsFixed(3)}m)';
  }

  void _centerMap() => setState(() {
    _mapOffset = Offset.zero;
    _mapScale = 1.0;
  });

  void _resetMapView() => setState(() {
    _mapOffset = Offset.zero;
    _mapScale = 1.0;
    _selectedCoordinate = null;
  });

  // Helper Methods
  Color _getWaypointTypeColor(String type) {
    switch (type) {
      case 'home': return Colors.green;
      case 'pickup': return Colors.blue;
      case 'drop': return Colors.orange;
      case 'charging': return Colors.yellow.shade700;
      case 'waypoint': return Colors.purple;
      default: return Colors.grey;
    }
  }

  IconData _getWaypointTypeIcon(String type) {
    switch (type) {
      case 'home': return Icons.home;
      case 'pickup': return Icons.outbox;
      case 'drop': return Icons.inbox;
      case 'charging': return Icons.battery_charging_full;
      case 'waypoint': return Icons.place;
      default: return Icons.location_on;
    }
  }

  Color _getMapSourceColor(String source) {
    switch (source.toLowerCase()) {
      case 'api': case 'loaded': return Colors.green;
      case 'provided_default': case 'provided': return Colors.blue;
      case 'fallback_provided': case 'fallback': return Colors.orange;
      case 'error_fallback': case 'error': return Colors.red;
      default: return Colors.grey;
    }
  }

  void _testLayout() {
    final screenSize = MediaQuery.of(context).size;
    print('📱 Layout: ${isTablet ? 'TABLET' : 'PHONE'} - ${screenSize.width}x${screenSize.height}');
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
}

// Custom Painter for Tablet-Optimized Map
class TabletOptimizedMapPainter extends CustomPainter {
  final MapData mapData;
  final MapData? loadedMapData;
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
  final bool isTablet;

  TabletOptimizedMapPainter({
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
    this.isTablet = false,
  }) : super(repaint: pulseAnimation);

  @override
  void paint(Canvas canvas, Size size) {
    try {
      final workingMapData = loadedMapData ?? mapData;
      final effectiveSize = size.width > 0 && size.height > 0 ? size : canvasSize;

      canvas.save();
      canvas.translate(mapOffset.dx, mapOffset.dy);
      canvas.scale(mapScale);

      _drawMapBackground(canvas, effectiveSize, workingMapData);
      
      if (showOccupancyGrid && workingMapData.occupancyData.isNotEmpty) {
        _drawOccupancyGrid(canvas, effectiveSize, workingMapData);
      }
      
      if (showGrid) _drawCoordinateGrid(canvas, effectiveSize, workingMapData);
      if (showMapShapes) _drawMapShapes(canvas, workingMapData);
      
      _drawWaypoints(canvas, effectiveSize, workingMapData);
      
      if (selectedCoordinate != null) _drawSelectedCoordinate(canvas);
      if (isLoadingMap) _drawLoadingOverlay(canvas, effectiveSize);

      canvas.restore();
    } catch (e) {
      _drawErrorState(canvas, size);
    }
  }

  void _drawMapBackground(Canvas canvas, Size size, MapData mapData) {
    final backgroundPaint = Paint()..color = Colors.white;
    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;

    canvas.drawRect(Rect.fromLTWH(0, 0, adjustedWidth, adjustedHeight), backgroundPaint);

    final borderPaint = Paint()
      ..color = Colors.grey.shade400
      ..style = PaintingStyle.stroke
      ..strokeWidth = isTablet ? 3.0 : 2.0;

    canvas.drawRect(Rect.fromLTWH(0, 0, adjustedWidth, adjustedHeight), borderPaint);

    final textPainter = TextPainter(
      text: TextSpan(
        text: 'Interactive Map\n'
               '${mapData.info.width}x${mapData.info.height} @ ${mapData.info.resolution.toStringAsFixed(3)}m/px\n'
               '${mapData.shapes.length} locations • ${waypoints.length} waypoints\n'
               '${isTablet ? 'Tap' : 'Click'} to add waypoints',
        style: TextStyle(color: Colors.grey.shade600, fontSize: isTablet ? 18 : 14),
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

  void _drawOccupancyGrid(Canvas canvas, Size size, MapData mapData) {
    // Simplified occupancy grid drawing for tablet
    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;
    final cellWidth = adjustedWidth / mapData.info.width;
    final cellHeight = adjustedHeight / mapData.info.height;
    
    if (cellWidth < (isTablet ? 0.3 : 0.5) || cellHeight < (isTablet ? 0.3 : 0.5)) return;
    
    // Basic grid visualization
    final gridPaint = Paint()..color = Colors.black.withOpacity(0.1);
    for (int x = 0; x < mapData.info.width; x += isTablet ? 5 : 10) {
      canvas.drawLine(
        Offset(x * cellWidth, 0),
        Offset(x * cellWidth, adjustedHeight),
        gridPaint,
      );
    }
  }

  void _drawCoordinateGrid(Canvas canvas, Size size, MapData mapData) {
    final paint = Paint()
      ..color = Colors.grey.shade300
      ..strokeWidth = isTablet ? 0.8 : 0.5;

    const gridSpacingMeters = 1.0;
    final gridSpacingPixels = gridSpacingMeters / mapData.info.resolution;
    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;

    for (double x = 0; x < adjustedWidth; x += gridSpacingPixels) {
      canvas.drawLine(Offset(x, 0), Offset(x, adjustedHeight), paint);
    }

    for (double y = 0; y < adjustedHeight; y += gridSpacingPixels) {
      canvas.drawLine(Offset(0, y), Offset(adjustedWidth, y), paint);
    }
  }

  void _drawMapShapes(Canvas canvas, MapData mapData) {
    for (final shape in mapData.shapes) {
      if (shape.points.isEmpty) continue;

      final color = _parseShapeColor(shape.color);
      final paint = Paint()
        ..color = color.withOpacity(mapOpacity)
        ..style = PaintingStyle.fill;

      final path = Path();
      final firstPoint = _worldToCanvasCoordinates(shape.points[0].x, shape.points[0].y, mapData);
      if (firstPoint != null) {
        path.moveTo(firstPoint.dx, firstPoint.dy);
        
        for (int i = 1; i < shape.points.length; i++) {
          final point = _worldToCanvasCoordinates(shape.points[i].x, shape.points[i].y, mapData);
          if (point != null) path.lineTo(point.dx, point.dy);
        }
        
        path.close();
        canvas.drawPath(path, paint);
      }
    }
  }

  void _drawWaypoints(Canvas canvas, Size size, MapData mapData) {
    for (int i = 0; i < waypoints.length; i++) {
      final waypoint = waypoints[i];
      final coords = waypoint['coordinates'] as Map<String, dynamic>;
      final worldX = (coords['x'] as num).toDouble();
      final worldY = (coords['y'] as num).toDouble();

      final canvasPos = _worldToCanvasCoordinates(worldX, worldY, mapData);
      if (canvasPos != null) {
        _drawWaypoint(canvas, canvasPos, waypoint, i + 1);
      }
    }
  }

  void _drawWaypoint(Canvas canvas, Offset center, Map<String, dynamic> waypoint, int stepNumber) {
    final color = _getWaypointTypeColor(waypoint['type']);
    final waypointRadius = isTablet ? 20.0 : 15.0;
    final pulseRadius = isTablet ? 30.0 : 20.0;

    // Pulsing outer circle
    final pulseScale = 1.0 + (pulseAnimation.value * 0.3);
    final outerPaint = Paint()..color = color.withOpacity(0.3);
    canvas.drawCircle(center, pulseRadius * pulseScale, outerPaint);

    // Main circle
    final mainPaint = Paint()..color = color;
    canvas.drawCircle(center, waypointRadius, mainPaint);

    // Step number
    final textPainter = TextPainter(
      text: TextSpan(
        text: stepNumber.toString(),
        style: TextStyle(
          color: Colors.white,
          fontWeight: FontWeight.bold,
          fontSize: isTablet ? 16 : 12,
        ),
      ),
      textDirection: TextDirection.ltr,
    );

    textPainter.layout();
    textPainter.paint(
      canvas,
      Offset(center.dx - textPainter.width / 2, center.dy - textPainter.height / 2),
    );

    // Connection line to next waypoint
    if (stepNumber < waypoints.length) {
      final nextWaypoint = waypoints[stepNumber];
      final nextCoords = nextWaypoint['coordinates'] as Map<String, dynamic>;
      final nextCanvasPos = _worldToCanvasCoordinates(
        (nextCoords['x'] as num).toDouble(),
        (nextCoords['y'] as num).toDouble(),
        mapData,
      );
      
      if (nextCanvasPos != null) {
        final linePaint = Paint()
          ..color = color.withOpacity(0.6)
          ..strokeWidth = isTablet ? 3 : 2
          ..style = PaintingStyle.stroke;

        canvas.drawLine(center, nextCanvasPos, linePaint);
        _drawArrow(canvas, center, nextCanvasPos, color);
      }
    }
  }

  void _drawArrow(Canvas canvas, Offset start, Offset end, Color color) {
    final direction = (end - start).direction;
    final arrowLength = isTablet ? 15.0 : 10.0;
    const arrowAngle = math.pi / 6;

    final arrowPaint = Paint()
      ..color = color
      ..strokeWidth = isTablet ? 3 : 2
      ..style = PaintingStyle.stroke;

    final arrowPoint1 = end + Offset(
      arrowLength * math.cos(direction + math.pi - arrowAngle),
      arrowLength * math.sin(direction + math.pi - arrowAngle),
    );

    final arrowPoint2 = end + Offset(
      arrowLength * math.cos(direction + math.pi + arrowAngle),
      arrowLength * math.sin(direction + math.pi + arrowAngle),
    );

    canvas.drawLine(end, arrowPoint1, arrowPaint);
    canvas.drawLine(end, arrowPoint2, arrowPaint);
  }

  void _drawSelectedCoordinate(Canvas canvas) {
    if (selectedCoordinate == null) return;

    final paint = Paint()
      ..color = Colors.red
      ..style = PaintingStyle.stroke
      ..strokeWidth = isTablet ? 3 : 2;

    final radius = isTablet ? 15.0 : 10.0;
    canvas.drawCircle(selectedCoordinate!, radius, paint);

    final crosshairLength = isTablet ? 20.0 : 15.0;
    canvas.drawLine(
      Offset(selectedCoordinate!.dx - crosshairLength, selectedCoordinate!.dy),
      Offset(selectedCoordinate!.dx + crosshairLength, selectedCoordinate!.dy),
      paint,
    );
    canvas.drawLine(
      Offset(selectedCoordinate!.dx, selectedCoordinate!.dy - crosshairLength),
      Offset(selectedCoordinate!.dx, selectedCoordinate!.dy + crosshairLength),
      paint,
    );
  }

  void _drawLoadingOverlay(Canvas canvas, Size size) {
    final overlayPaint = Paint()..color = Colors.white.withOpacity(0.7);
    canvas.drawRect(
      Rect.fromLTWH(0, 0, size.width / mapScale, size.height / mapScale),
      overlayPaint,
    );

    final center = Offset((size.width / mapScale) / 2, (size.height / mapScale) / 2);
    final spinnerPaint = Paint()
      ..color = Colors.blue
      ..style = PaintingStyle.stroke
      ..strokeWidth = isTablet ? 4.0 : 3.0;

    canvas.drawCircle(center, isTablet ? 30 : 20, spinnerPaint);
  }

  void _drawErrorState(Canvas canvas, Size size) {
    final errorPaint = Paint()..color = Colors.red.shade50;
    canvas.drawRect(Rect.fromLTWH(0, 0, size.width, size.height), errorPaint);

    final center = Offset(size.width / 2, size.height / 2);
    final textPainter = TextPainter(
      text: TextSpan(
        text: 'Map Rendering Error',
        style: TextStyle(
          color: Colors.red.shade700,
          fontSize: isTablet ? 20 : 16,
          fontWeight: FontWeight.bold,
        ),
      ),
      textAlign: TextAlign.center,
      textDirection: TextDirection.ltr,
    );

    textPainter.layout();
    textPainter.paint(canvas, Offset(center.dx - textPainter.width / 2, center.dy));
  }

  Offset? _worldToCanvasCoordinates(double worldX, double worldY, MapData mapData) {
    try {
      final canvasWidth = canvasSize.width / mapScale;
      final canvasHeight = canvasSize.height / mapScale;
      
      final mapPixelX = (worldX - mapData.info.origin.x) / mapData.info.resolution;
      final mapPixelY = mapData.info.height - ((worldY - mapData.info.origin.y) / mapData.info.resolution);
      
      final canvasX = (mapPixelX / mapData.info.width) * canvasWidth;
      final canvasY = (mapPixelY / mapData.info.height) * canvasHeight;
      
      return Offset(canvasX, canvasY);
    } catch (e) {
      return null;
    }
  }

  Color _parseShapeColor(String colorString) {
    try {
      if (colorString.startsWith('#')) {
        final hexString = colorString.substring(1);
        if (hexString.length == 6) {
          return Color(int.parse('FF$hexString', radix: 16));
        } else if (hexString.length == 8) {
          return Color(int.parse(hexString, radix: 16));
        }
      }
      
      switch (colorString.toLowerCase()) {
        case 'red': return Colors.red;
        case 'green': return Colors.green;
        case 'blue': return Colors.blue;
        case 'orange': return Colors.orange;
        case 'purple': return Colors.purple;
        default: return Colors.blue;
      }
    } catch (e) {
      return Colors.blue;
    }
  }

  Color _getWaypointTypeColor(String type) {
    switch (type) {
      case 'home': return Colors.green;
      case 'pickup': return Colors.blue;
      case 'drop': return Colors.orange;
      case 'charging': return Colors.yellow.shade700;
      case 'waypoint': return Colors.purple;
      default: return Colors.grey;
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}