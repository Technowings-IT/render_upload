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
        ],
      ),
      body: Column(
        children: [
          // Main content area with explicit height
          Expanded(
            child: Container(
              height: double.infinity,
              child: Row(
                children: [
                  // Map Canvas
                  Expanded(
                    flex: 3,
                    child: Container(
                      height: double.infinity,
                      child: _buildInteractiveMapCanvas(),
                    ),
                  ),

                  // Control Panel
                  Container(
                    width: 400,
                    height: double.infinity,
                    decoration: BoxDecoration(
                      border:
                          Border(left: BorderSide(color: Colors.grey.shade300)),
                    ),
                    child: _buildControlPanel(),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
      bottomNavigationBar: _buildBottomActionBar(),
    );
  }

  Widget _buildInteractiveMapCanvas() {
    return Container(
      color: Colors.grey.shade100,
      width: double.infinity,
      height: double.infinity,
      child: Stack(
        children: [
          // Map display with explicit sizing
          Positioned.fill(
            child: ClipRect(
              child: GestureDetector(
                onTapDown: _handleMapTap,
                onScaleStart: _handleScaleStart,
                onScaleUpdate: _handleScaleUpdate,
                child: CustomPaint(
                  painter: InteractiveMapPainter(
                    mapData: widget.mapData,
                    waypoints: _waypoints,
                    selectedCoordinate: _selectedCoordinate,
                    mapScale: _mapScale,
                    mapOffset: _mapOffset,
                    showGrid: _showGrid,
                    showCoordinates: _showCoordinates,
                    pulseAnimation: _pulseAnimationController,
                  ),
                  size: Size.infinite,
                  child: Container(), // Provide a child to ensure proper layout
                ),
              ),
            ),
          ),

          // Coordinate display overlay
          if (_selectedCoordinate != null && _showCoordinates)
            Positioned(
              left: _selectedCoordinate!.dx + 10,
              top: _selectedCoordinate!.dy - 30,
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

          // Waypoint addition mode overlay
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
  }

  Widget _buildControlPanel() {
    return Padding(
      padding: EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Order Details Section
          _buildOrderDetailsSection(),

          SizedBox(height: 20),

          // Waypoint Types Section
          _buildWaypointTypesSection(),

          SizedBox(height: 20),

          // Waypoints List Section
          _buildWaypointsListSection(),

          Spacer(),

          // Actions
          _buildActionButtons(),
        ],
      ),
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
        'label': 'Home Position',
        'icon': Icons.home,
        'color': Colors.green
      },
      {
        'type': 'pickup',
        'label': 'Pickup Point',
        'icon': Icons.outbox,
        'color': Colors.blue
      },
      {
        'type': 'drop',
        'label': 'Drop Point',
        'icon': Icons.inbox,
        'color': Colors.orange
      },
      {
        'type': 'charging',
        'label': 'Charging Station',
        'icon': Icons.battery_charging_full,
        'color': Colors.yellow.shade700
      },
      {
        'type': 'waypoint',
        'label': 'Waypoint',
        'icon': Icons.place,
        'color': Colors.purple
      },
    ];

    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Add Waypoint',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 12),
            Text(
              'Select type and click on map:',
              style: TextStyle(color: Colors.grey.shade600),
            ),
            SizedBox(height: 12),
            Wrap(
              spacing: 8,
              runSpacing: 8,
              children: waypointTypes
                  .map((type) => _buildWaypointTypeButton(type))
                  .toList(),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildWaypointTypeButton(Map<String, dynamic> type) {
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
        padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        decoration: BoxDecoration(
          color: isSelected ? color : color.withOpacity(0.1),
          border: Border.all(color: color),
          borderRadius: BorderRadius.circular(20),
        ),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(
              type['icon'] as IconData,
              size: 16,
              color: isSelected ? Colors.white : color,
            ),
            SizedBox(width: 6),
            Text(
              type['label'],
              style: TextStyle(
                color: isSelected ? Colors.white : color,
                fontWeight: FontWeight.w600,
                fontSize: 12,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildWaypointsListSection() {
    return Expanded(
      child: Card(
        child: Padding(
          padding: EdgeInsets.all(16),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Text(
                    'Waypoints (${_waypoints.length})',
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                  if (_waypoints.isNotEmpty)
                    IconButton(
                      icon: Icon(Icons.clear_all),
                      onPressed: _clearAllWaypoints,
                      tooltip: 'Clear All',
                    ),
                ],
              ),
              SizedBox(height: 12),
              if (_waypoints.isEmpty)
                Expanded(
                  child: Center(
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Icon(Icons.touch_app, size: 48, color: Colors.grey),
                        SizedBox(height: 8),
                        Text(
                          'No waypoints added yet',
                          style: TextStyle(color: Colors.grey.shade600),
                        ),
                        Text(
                          'Select a type and click on the map',
                          style: TextStyle(
                              color: Colors.grey.shade500, fontSize: 12),
                        ),
                      ],
                    ),
                  ),
                )
              else
                Expanded(
                  child: ReorderableListView.builder(
                    itemCount: _waypoints.length,
                    onReorder: _reorderWaypoints,
                    itemBuilder: (context, index) =>
                        _buildWaypointListItem(index),
                  ),
                ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildWaypointListItem(int index) {
    final waypoint = _waypoints[index];
    final color = _getWaypointTypeColor(waypoint['type']);

    return Card(
      key: ValueKey(waypoint['id']),
      margin: EdgeInsets.symmetric(vertical: 2),
      child: ListTile(
        leading: Container(
          width: 40,
          height: 40,
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
                  fontSize: 12,
                ),
              ),
              Icon(
                _getWaypointTypeIcon(waypoint['type']),
                color: Colors.white,
                size: 16,
              ),
            ],
          ),
        ),
        title: Text(
          waypoint['name'],
          style: TextStyle(fontWeight: FontWeight.w600),
        ),
        subtitle: Text(
          '${waypoint['type'].toUpperCase()}\n(${waypoint['coordinates']['x'].toStringAsFixed(2)}, ${waypoint['coordinates']['y'].toStringAsFixed(2)})',
        ),
        trailing: PopupMenuButton<String>(
          onSelected: (value) => _handleWaypointAction(index, value),
          itemBuilder: (context) => [
            PopupMenuItem(value: 'edit', child: Text('Edit')),
            PopupMenuItem(value: 'duplicate', child: Text('Duplicate')),
            PopupMenuItem(value: 'delete', child: Text('Delete')),
          ],
        ),
        isThreeLine: true,
      ),
    );
  }

  Widget _buildActionButtons() {
    return Column(
      children: [
        SizedBox(
          width: double.infinity,
          child: ElevatedButton.icon(
            onPressed: _waypoints.isNotEmpty && !_isCreatingOrder
                ? _createOrder
                : null,
            icon: _isCreatingOrder
                ? SizedBox(
                    width: 16,
                    height: 16,
                    child: CircularProgressIndicator(strokeWidth: 2))
                : Icon(Icons.send),
            label: Text(_isCreatingOrder ? 'Creating...' : 'Create Order'),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.green,
              foregroundColor: Colors.white,
              padding: EdgeInsets.symmetric(vertical: 12),
            ),
          ),
        ),
        SizedBox(height: 8),
        Row(
          children: [
            Expanded(
              child: OutlinedButton.icon(
                onPressed: () => _previewOrderExecution(),
                icon: Icon(Icons.preview),
                label: Text('Preview'),
              ),
            ),
            SizedBox(width: 8),
            Expanded(
              child: OutlinedButton.icon(
                onPressed: () => _saveOrderTemplate(),
                icon: Icon(Icons.save),
                label: Text('Save Template'),
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildBottomActionBar() {
    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        border: Border(top: BorderSide(color: Colors.grey.shade300)),
        color: Colors.white,
      ),
      child: Row(
        children: [
          // Map controls
          Text('Map Scale: ${(_mapScale * 100).toInt()}%'),
          SizedBox(width: 16),
          Slider(
            value: _mapScale,
            min: 0.5,
            max: 3.0,
            divisions: 25,
            onChanged: (value) => setState(() => _mapScale = value),
          ),

          Spacer(),

          // Quick actions
          IconButton(
            icon: Icon(Icons.center_focus_strong),
            onPressed: _centerMap,
            tooltip: 'Center Map',
          ),
          IconButton(
            icon: Icon(Icons.refresh),
            onPressed: _resetMapView,
            tooltip: 'Reset View',
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
    for (int i = 0; i < orderData['waypoints'].length; i++) {
      final wp = (orderData['waypoints'] as List)[i];
      final pos = wp['position'];
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
    // Enhanced PGM map coordinate conversion
    // This properly converts screen coordinates to ROS map coordinates
    
    // Get the actual canvas size
    final renderBox = context.findRenderObject() as RenderBox?;
    final canvasSize = renderBox?.size ?? Size(800, 600);
    final canvasWidth = canvasSize.width;
    final canvasHeight = canvasSize.height;
    
    // PGM map parameters from MapData
    final mapWidth = widget.mapData.info.width.toDouble();
    final mapHeight = widget.mapData.info.height.toDouble();
    final resolution = widget.mapData.info.resolution; // meters per pixel
    final originX = widget.mapData.info.origin.x; // map origin in meters
    final originY = widget.mapData.info.origin.y;
    
    print('🗺️ Map info: ${mapWidth}x${mapHeight}, resolution: ${resolution}m/pixel');
    print('🌍 Origin: (${originX}, ${originY})');
    print('🖥️ Canvas: ${canvasWidth}x${canvasHeight}');
    
    // Apply current map transformations (scale and offset)
    final adjustedX = (screenPosition.dx - _mapOffset.dx) / _mapScale;
    final adjustedY = (screenPosition.dy - _mapOffset.dy) / _mapScale;
    
    // Convert canvas coordinates to map pixel coordinates
    // Map the entire canvas to the map dimensions
    final mapPixelX = (adjustedX / canvasWidth) * mapWidth;
    final mapPixelY = (adjustedY / canvasHeight) * mapHeight;
    
    // Convert map pixel coordinates to world coordinates (meters)
    // PGM maps have origin at bottom-left, but screen coordinates start at top-left
    // So we need to flip the Y coordinate
    final worldX = originX + (mapPixelX * resolution);
    final worldY = originY + ((mapHeight - mapPixelY) * resolution);
    
    // Validate coordinates are within reasonable bounds
    if (!_isValidMapCoordinate(worldX, worldY)) {
      print('⚠️ Coordinates may be outside valid map bounds: ($worldX, $worldY)');
    }
    
    print('🔄 Coordinate conversion:');
    print('  Screen: (${screenPosition.dx.toStringAsFixed(1)}, ${screenPosition.dy.toStringAsFixed(1)})');
    print('  Adjusted: (${adjustedX.toStringAsFixed(1)}, ${adjustedY.toStringAsFixed(1)})');
    print('  Map Pixel: (${mapPixelX.toStringAsFixed(1)}, ${mapPixelY.toStringAsFixed(1)})');
    print('  World (ROS): (${worldX.toStringAsFixed(3)}, ${worldY.toStringAsFixed(3)})');
    
    return {
      'x': worldX,
      'y': worldY,
    };
  } catch (e) {
    print('❌ Error converting screen to map coordinates: $e');
    // Return safe fallback coordinates
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

// ==========================================
// CUSTOM PAINTER FOR INTERACTIVE MAP
// ==========================================
  void _drawCoordinateGrid(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.grey.shade300
      ..strokeWidth = 0.5;

    // Calculate grid spacing in meters (adjust based on map resolution)
    final gridSpacingMeters = 1.0; // 1 meter grid
    final gridSpacingPixels = gridSpacingMeters / mapData.info.resolution;

    // Draw vertical lines
    for (double x = 0; x < size.width / mapScale; x += gridSpacingPixels) {
      canvas.drawLine(
        Offset(x, 0),
        Offset(x, size.height / mapScale),
        paint,
      );
    }

    // Draw horizontal lines
    for (double y = 0; y < size.height / mapScale; y += gridSpacingPixels) {
      canvas.drawLine(
        Offset(0, y),
        Offset(size.width / mapScale, y),
        paint,
      );
    }

    // Draw coordinate labels every 5 meters
    final textPaint = TextPainter(
      textDirection: TextDirection.ltr,
    );

    for (double x = 0; x < size.width / mapScale; x += gridSpacingPixels * 5) {
      for (double y = 0; y < size.height / mapScale; y += gridSpacingPixels * 5) {
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

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}

class InteractiveMapPainter extends CustomPainter {
  final MapData mapData;
  final List<Map<String, dynamic>> waypoints;
  final Offset? selectedCoordinate;
  final double mapScale;
  final Offset mapOffset;
  final bool showGrid;
  final bool showCoordinates;
  final AnimationController pulseAnimation;

  InteractiveMapPainter({
    required this.mapData,
    required this.waypoints,
    this.selectedCoordinate,
    required this.mapScale,
    required this.mapOffset,
    required this.showGrid,
    required this.showCoordinates,
    required this.pulseAnimation,
  }) : super(repaint: pulseAnimation);

  @override
  void paint(Canvas canvas, Size size) {
    try {
      print('🎨 Painting map canvas - Size: ${size.width}x${size.height}');
      print('📏 Map scale: $mapScale, Offset: $mapOffset');
      print('🗺️ Map dimensions: ${mapData.info.width}x${mapData.info.height}');
      print('📍 Map resolution: ${mapData.info.resolution}m/pixel');
      print('🌍 Map origin: (${mapData.info.origin.x}, ${mapData.info.origin.y})');

      // Handle zero height case
      if (size.height <= 0 || size.width <= 0) {
        print('❌ Invalid canvas size, skipping paint');
        return;
      }

      canvas.save();

      // Apply transformations
      canvas.translate(mapOffset.dx, mapOffset.dy);
      canvas.scale(mapScale);

      // Draw map background with coordinate system info
      _drawMapBackground(canvas, size);

      // Draw coordinate grid if enabled
      if (showGrid) {
        _drawCoordinateGrid(canvas, size);
      }

      // Draw existing map shapes
      _drawMapShapes(canvas);

      // Draw waypoints with ROS coordinates
      _drawWaypointsWithCoordinates(canvas);

      // Draw selected coordinate with world coordinates
      if (selectedCoordinate != null) {
        _drawSelectedCoordinateWithWorldPos(canvas);
      }

      canvas.restore();
    } catch (e) {
      print('❌ Error painting canvas: $e');
      // Draw error message
      final textPainter = TextPainter(
        text: TextSpan(
          text: 'Error rendering map: $e',
          style: TextStyle(color: Colors.red, fontSize: 14),
        ),
        textDirection: TextDirection.ltr,
      );
      textPainter.layout();
      textPainter.paint(canvas, Offset(10, 10));
    }
  }

  void _drawMapBackground(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.fill;

    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;

    canvas.drawRect(
      Rect.fromLTWH(0, 0, adjustedWidth, adjustedHeight),
      paint,
    );

    // Draw a border to make the map area visible
    final borderPaint = Paint()
      ..color = Colors.grey.shade400
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;

    canvas.drawRect(
      Rect.fromLTWH(0, 0, adjustedWidth, adjustedHeight),
      borderPaint,
    );

    // If no map data, draw a placeholder
    if (mapData.shapes.isEmpty) {
      _drawMapPlaceholder(canvas, Size(adjustedWidth, adjustedHeight));
    }
  }

  void _drawMapPlaceholder(Canvas canvas, Size size) {
    final centerX = size.width / 2;
    final centerY = size.height / 2;

    // Draw placeholder text
    final textPainter = TextPainter(
      text: TextSpan(
        text: 'No Map Data\nClick to add waypoints',
        style: TextStyle(
          color: Colors.grey.shade600,
          fontSize: 16,
          fontWeight: FontWeight.w500,
        ),
      ),
      textAlign: TextAlign.center,
      textDirection: TextDirection.ltr,
    );

    textPainter.layout();
    textPainter.paint(
      canvas,
      Offset(
        centerX - textPainter.width / 2,
        centerY - textPainter.height / 2,
      ),
    );

    // Draw some sample grid points
    final pointPaint = Paint()
      ..color = Colors.grey.shade300
      ..style = PaintingStyle.fill;

    for (double x = 50; x < size.width; x += 100) {
      for (double y = 50; y < size.height; y += 100) {
        canvas.drawCircle(Offset(x, y), 2, pointPaint);
      }
    }
  }

  void _drawGrid(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.grey.shade300
      ..strokeWidth = 0.5;

    const gridSpacing = 50.0;

    // Vertical lines
    for (double x = 0; x < size.width / mapScale; x += gridSpacing) {
      canvas.drawLine(
        Offset(x, 0),
        Offset(x, size.height / mapScale),
        paint,
      );
    }

    // Horizontal lines
    for (double y = 0; y < size.height / mapScale; y += gridSpacing) {
      canvas.drawLine(
        Offset(0, y),
        Offset(size.width / mapScale, y),
        paint,
      );
    }
  }

  void _drawMapShapes(Canvas canvas) {
    if (mapData.shapes.isEmpty) {
      // No shapes to draw
      return;
    }

    for (final shape in mapData.shapes) {
      try {
        final paint = Paint()
          ..color = _parseShapeColor(shape.color)
          ..style = PaintingStyle.fill;

        if (shape.points.isNotEmpty) {
          final path = Path();
          path.moveTo(shape.points.first.x, shape.points.first.y);
          for (int i = 1; i < shape.points.length; i++) {
            path.lineTo(shape.points[i].x, shape.points[i].y);
          }
          path.close();
          canvas.drawPath(path, paint);
        }
      } catch (e) {
        print('❌ Error drawing shape: $e');
        // Continue with next shape
      }
    }
  }

  Color _parseShapeColor(String colorString) {
    try {
      // Handle different color formats
      String cleanColor = colorString.trim();

      // Remove # if present
      if (cleanColor.startsWith('#')) {
        cleanColor = cleanColor.substring(1);
      }

      // Ensure we have 8 characters (AARRGGBB)
      if (cleanColor.length == 6) {
        cleanColor = 'FF$cleanColor'; // Add alpha if missing
      }

      // Parse as hexadecimal
      final colorValue = int.parse(cleanColor, radix: 16);
      return Color(colorValue);
    } catch (e) {
      print('❌ Error parsing color "$colorString": $e');
      return Colors.grey; // Fallback color
    }
  }

  void _drawWaypoints(Canvas canvas) {
    for (int i = 0; i < waypoints.length; i++) {
      try {
        final waypoint = waypoints[i];
        final coords = waypoint['coordinates'] as Map<String, dynamic>;

        // Safely convert coordinates to double
        final x = coords['x'] is double
            ? coords['x'] as double
            : (coords['x'] as num?)?.toDouble() ?? 0.0;
        final y = coords['y'] is double
            ? coords['y'] as double
            : (coords['y'] as num?)?.toDouble() ?? 0.0;

        final center = Offset(x, y);
        _drawWaypoint(canvas, center, waypoint, i + 1);
      } catch (e) {
        print('❌ Error drawing waypoint $i: $e');
        // Skip this waypoint and continue with the next one
      }
    }
  }

  void _drawWaypoint(Canvas canvas, Offset center,
      Map<String, dynamic> waypoint, int stepNumber) {
    try {
      final type = waypoint['type'] as String;
      final color = _getWaypointTypeColor(type);

      // Draw outer circle with pulse effect
      final pulseScale = 1.0 + (pulseAnimation.value * 0.3);
      final outerPaint = Paint()
        ..color = color.withOpacity(0.3)
        ..style = PaintingStyle.fill;

      canvas.drawCircle(center, 20 * pulseScale, outerPaint);

      // Draw main circle
      final mainPaint = Paint()
        ..color = color
        ..style = PaintingStyle.fill;

      canvas.drawCircle(center, 15, mainPaint);

      // Draw step number
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
          final nextCoords =
              nextWaypoint['coordinates'] as Map<String, dynamic>;

          // Safely convert next coordinates
          final nextX = nextCoords['x'] is double
              ? nextCoords['x'] as double
              : (nextCoords['x'] as num?)?.toDouble() ?? 0.0;
          final nextY = nextCoords['y'] is double
              ? nextCoords['y'] as double
              : (nextCoords['y'] as num?)?.toDouble() ?? 0.0;

          final nextCenter = Offset(nextX, nextY);

          final linePaint = Paint()
            ..color = color.withOpacity(0.6)
            ..strokeWidth = 2
            ..style = PaintingStyle.stroke;

          canvas.drawLine(center, nextCenter, linePaint);

          // Draw arrow at the end
          _drawArrow(canvas, center, nextCenter, color);
        } catch (e) {
          print('❌ Error drawing connection line: $e');
        }
      }
    } catch (e) {
      print('❌ Error drawing waypoint: $e');
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

    // Draw crosshair
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

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}
