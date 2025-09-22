// widgets/complete_fleet_management_widget.dart - Complete Fleet Management System
import 'package:flutter/material.dart';
import '../models/map_data.dart';
import '../services/api_service.dart';
// import 'enhanced_order_creation_dialog.dart';
import 'order_table_widget.dart';
// import 'interactive_map_order_creator.dart';

class CompleteFleetManagementWidget extends StatefulWidget {
  final String? deviceId; // null for all devices view
  final Map<String, MapData> availableMaps;
  final List<Map<String, dynamic>> availableDevices;

  const CompleteFleetManagementWidget({
    Key? key,
    this.deviceId,
    required this.availableMaps,
    required this.availableDevices,
  }) : super(key: key);

  @override
  State<CompleteFleetManagementWidget> createState() =>
      _CompleteFleetManagementWidgetState();
}

class _CompleteFleetManagementWidgetState
    extends State<CompleteFleetManagementWidget> with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();

  late TabController _tabController;

  // State
  List<Map<String, dynamic>> _orders = [];
  Map<String, List<MapShape>> _deviceStations = {};
  bool _isLoading = false;
  String? _selectedDeviceId;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 3, vsync: this);
    _selectedDeviceId = widget.deviceId ?? widget.availableDevices.first['id'];
    _loadStations();
    _loadOrders();
  }

  Future<void> _loadStations() async {
    setState(() => _isLoading = true);

    try {
      for (final deviceId in widget.availableMaps.keys) {
        final mapData = widget.availableMaps[deviceId];
        if (mapData != null) {
          _deviceStations[deviceId] = mapData.shapes
              .where((shape) => [
                    'pickup',
                    'drop',
                    'charging',
                    'waypoint',
                    'pick_up',
                    'charge_1',
                    'charge_2'
                  ].contains(shape.type))
              .toList();
        }
      }
      print(' Loaded stations for ${_deviceStations.length} devices');
    } catch (e) {
      print(' Error loading stations: $e');
    } finally {
      setState(() => _isLoading = false);
    }
  }

  Future<void> _loadOrders() async {
    try {
      final response = widget.deviceId != null
          ? await _apiService.getOrders(widget.deviceId!)
          : await _apiService.getAllOrders();

      final responseMap = response as Map<String, dynamic>;
      if (responseMap['success'] == true) {
        setState(() {
          _orders =
              List<Map<String, dynamic>>.from(responseMap['orders'] ?? []);
        });
      }
    } catch (e) {
      print(' Error loading orders: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Column(
        children: [
          _buildHeader(),
          Expanded(
            child: TabBarView(
              controller: _tabController,
              children: [
                _buildOrdersView(),
                _buildStationsView(),
                // _buildMapView(),
              ],
            ),
          ),
        ],
      ),
      floatingActionButton: _buildFloatingActionButtons(),
    );
  }

  Widget _buildHeader() {
    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.blue.shade700, Colors.blue.shade900],
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 8,
            offset: Offset(0, 2),
          ),
        ],
      ),
      child: Column(
        children: [
          // Title and device selector
          Row(
            children: [
              Icon(Icons.engineering, color: Colors.white, size: 32),
              SizedBox(width: 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Fleet Management System',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 24,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                    Text(
                      'Complete order and station management',
                      style: TextStyle(
                        color: Colors.white.withOpacity(0.9),
                        fontSize: 14,
                      ),
                    ),
                  ],
                ),
              ),
              _buildDeviceSelector(),
            ],
          ),

          SizedBox(height: 16),

          // Tab bar
          Material(
            color: Colors.transparent,
            child: TabBar(
              controller: _tabController,
              indicatorColor: Colors.white,
              labelColor: Colors.white,
              unselectedLabelColor: Colors.white.withOpacity(0.7),
              tabs: [
                Tab(
                  icon: Icon(Icons.list_alt),
                  text: 'Orders (${_orders.length})',
                ),
                Tab(
                  icon: Icon(Icons.location_on),
                  text: 'Stations (${_getStationCount()})',
                ),
                Tab(
                  icon: Icon(Icons.map),
                  text: 'Map View',
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDeviceSelector() {
    if (widget.deviceId != null) {
      // Single device mode
      return Container(
        padding: EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        decoration: BoxDecoration(
          color: Colors.white.withOpacity(0.2),
          borderRadius: BorderRadius.circular(20),
        ),
        child: Text(
          'Device: ${widget.deviceId}',
          style: TextStyle(
            color: Colors.white,
            fontWeight: FontWeight.w600,
          ),
        ),
      );
    }

    // Multi-device selector
    return Container(
      padding: EdgeInsets.symmetric(horizontal: 12, vertical: 4),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.1),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: Colors.white.withOpacity(0.3)),
      ),
      child: DropdownButtonHideUnderline(
        child: DropdownButton<String>(
          value: _selectedDeviceId,
          dropdownColor: Colors.blue.shade800,
          style: TextStyle(color: Colors.white),
          items: [
            DropdownMenuItem<String>(
              value: 'all',
              child: Text('All Devices'),
            ),
            ...widget.availableDevices.map((device) {
              return DropdownMenuItem<String>(
                value: device['id'],
                child: Text(device['name'] ?? device['id']),
              );
            }),
          ],
          onChanged: (value) {
            setState(() {
              _selectedDeviceId = value;
            });
            _loadOrders();
          },
        ),
      ),
    );
  }

  Widget _buildOrdersView() {
    return OrdersTableWidget(
      deviceId: _selectedDeviceId == 'all' ? null : _selectedDeviceId,
      onOrderUpdated: (order) => _loadOrders(),
      onOrderExecuted: (order) => _executeOrder(order),
    );
  }

  Widget _buildStationsView() {
    return Container(
      padding: EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Stations header
          Container(
            padding: EdgeInsets.all(16),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [Colors.purple.shade50, Colors.purple.shade100],
              ),
              borderRadius: BorderRadius.circular(12),
              border: Border.all(color: Colors.purple.shade200),
            ),
            child: Row(
              children: [
                Icon(Icons.location_city, color: Colors.purple.shade700),
                SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Station Management',
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: Colors.purple.shade700,
                        ),
                      ),
                      Text(
                        'Manage pickup, drop, charging, and waypoint stations',
                        style: TextStyle(color: Colors.purple.shade600),
                      ),
                    ],
                  ),
                ),
                _buildStationSummary(),
              ],
            ),
          ),

          SizedBox(height: 16),

          // Station list
          Expanded(
            child: _buildStationList(),
          ),
        ],
      ),
    );
  }

  Widget _buildStationSummary() {
    final stationCounts = <String, int>{};

    for (final stations in _deviceStations.values) {
      for (final station in stations) {
        stationCounts[station.type] = (stationCounts[station.type] ?? 0) + 1;
      }
    }

    return Container(
      padding: EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.7),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Column(
        children: [
          _buildStationTypeBadge(
              'pickup', stationCounts['pickup'] ?? 0, Colors.green),
          SizedBox(height: 4),
          _buildStationTypeBadge(
              'drop', stationCounts['drop'] ?? 0, Colors.blue),
          SizedBox(height: 4),
          _buildStationTypeBadge(
              'charging', stationCounts['charging'] ?? 0, Colors.orange),
        ],
      ),
    );
  }

  Widget _buildStationTypeBadge(String type, int count, Color color) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          width: 12,
          height: 12,
          decoration: BoxDecoration(
            color: color,
            shape: BoxShape.circle,
          ),
        ),
        SizedBox(width: 6),
        Text(
          '$type: $count',
          style: TextStyle(
            fontSize: 12,
            fontWeight: FontWeight.w600,
          ),
        ),
      ],
    );
  }

  Widget _buildStationList() {
    if (_deviceStations.isEmpty) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.location_off, size: 64, color: Colors.grey),
            SizedBox(height: 16),
            Text(
              'No Stations Available',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            Text('Load maps to view stations'),
          ],
        ),
      );
    }

    final stationsToShow =
        _selectedDeviceId == 'all' || _selectedDeviceId == null
            ? _deviceStations.values.expand((stations) => stations).toList()
            : _deviceStations[_selectedDeviceId] ?? [];

    return ListView.builder(
      itemCount: stationsToShow.length,
      itemBuilder: (context, index) {
        final station = stationsToShow[index];
        return _buildStationCard(station, index + 1);
      },
    );
  }

  Widget _buildStationCard(MapShape station, int index) {
    final color = _getStationTypeColor(station.type);

    return Card(
      margin: EdgeInsets.symmetric(vertical: 4),
      child: ListTile(
        leading: Container(
          width: 50,
          height: 50,
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [color, color.withOpacity(0.7)],
            ),
            borderRadius: BorderRadius.circular(8),
          ),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Text(
                index.toString(),
                style: TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  fontSize: 16,
                ),
              ),
              Icon(
                _getStationTypeIcon(station.type),
                color: Colors.white,
                size: 16,
              ),
            ],
          ),
        ),
        title: Text(
          station.name,
          style: TextStyle(fontWeight: FontWeight.w600),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Type: ${station.type.toUpperCase()}'),
            Text(
                'Position: (${station.center.x.toStringAsFixed(2)}, ${station.center.y.toStringAsFixed(2)})'),
          ],
        ),
        trailing: PopupMenuButton<String>(
          onSelected: (value) => _handleStationAction(station, value),
          itemBuilder: (context) => [
            PopupMenuItem(value: 'edit', child: Text('Edit Station')),
            PopupMenuItem(value: 'orders', child: Text('View Orders')),
            PopupMenuItem(value: 'test', child: Text('Test Navigation')),
            PopupMenuDivider(),
            PopupMenuItem(
                value: 'delete',
                child: Text('Delete', style: TextStyle(color: Colors.red))),
          ],
        ),
      ),
    );
  }

  // Widget _buildMapView() {
  //   if (widget.availableMaps.isEmpty) {
  //     return Center(
  //       child: Column(
  //         mainAxisAlignment: MainAxisAlignment.center,
  //         children: [
  //           Icon(Icons.map_outlined, size: 64, color: Colors.grey),
  //           SizedBox(height: 16),
  //           Text(
  //             'No Maps Available',
  //             style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
  //           ),
  //           Text('Load device maps to view station layout'),
  //         ],
  //       ),
  //     );
  //   }

  //   final deviceId = _selectedDeviceId == 'all'
  //       ? widget.availableMaps.keys.first
  //       : _selectedDeviceId;
  //   final mapData = widget.availableMaps[deviceId];

  //   if (mapData == null) {
  //     return Center(child: Text('No map data for selected device'));
  //   }

  //   // return InteractiveMapOrderCreator(
  //   //   deviceId: deviceId!,
  //   //   mapData: mapData,
  //   //   onOrderCreated: (orderData) {
  //   //     _loadOrders();
  //   //     _showMessage('Order created successfully!', Colors.green);
  //   //   },
  //   // );
  // }

  Widget _buildFloatingActionButtons() {
    return Column(
      mainAxisAlignment: MainAxisAlignment.end,
      children: [
        // Quick create order button
        FloatingActionButton.extended(
          heroTag: "createOrder",
          onPressed: _showOrderCreationOptions,
          icon: Icon(Icons.add_task),
          label: Text('Create Order'),
          backgroundColor: Colors.green,
        ),

        SizedBox(height: 12),

        // Emergency stop button
        FloatingActionButton(
          heroTag: "emergencyStop",
          onPressed: _showEmergencyStopDialog,
          child: Icon(Icons.emergency, color: Colors.white),
          backgroundColor: Colors.red,
          mini: true,
        ),
      ],
    );
  }

  void _showOrderCreationOptions() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Create Order'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            ListTile(
              leading: Icon(Icons.touch_app, color: Colors.blue),
              title: Text('Station-Based Order'),
              subtitle: Text('Select from and to stations'),
              onTap: () {
                Navigator.of(context).pop();
                _showStationBasedOrderDialog();
              },
            ),
            ListTile(
              leading: Icon(Icons.map, color: Colors.green),
              title: Text('Interactive Map Order'),
              subtitle: Text('Click coordinates on map'),
              onTap: () {
                Navigator.of(context).pop();
                _showInteractiveMapDialog();
              },
            ),
          ],
        ),
      ),
    );
  }

  void _showStationBasedOrderDialog() {
    final selectedDeviceId = _selectedDeviceId == 'all'
        ? widget.availableDevices.first['id']
        : _selectedDeviceId;
    final mapData = widget.availableMaps[selectedDeviceId];

    if (mapData == null) {
      _showMessage('No map available for selected device', Colors.red);
      return;
    }

    // showDialog(
    //   context: context,
    //   builder: (context) => EnhancedOrderCreationDialog(
    //     deviceId: selectedDeviceId!,
    //     mapData: mapData,
    //     onOrderCreated: (orderData) {
    //       _loadOrders();
    //       _showMessage('Order created successfully!', Colors.green);
    //     },
    //   ),
    // );
  }

  void _showInteractiveMapDialog() {
    final selectedDeviceId = _selectedDeviceId == 'all'
        ? widget.availableDevices.first['id']
        : _selectedDeviceId;
    final mapData = widget.availableMaps[selectedDeviceId];

    if (mapData == null) {
      _showMessage('No map available for selected device', Colors.red);
      return;
    }

    // Navigator.of(context).push(
    //   MaterialPageRoute(
    //     builder: (context) => InteractiveMapOrderCreator(
    //       deviceId: selectedDeviceId!,
    //       mapData: mapData,
    //       onOrderCreated: (orderData) {
    //         _loadOrders();
    //         _showMessage('Order created successfully!', Colors.green);
    //       },
    //     ),
    //   ),
    // );
  }

  void _showEmergencyStopDialog() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Row(
          children: [
            Icon(Icons.warning, color: Colors.red),
            SizedBox(width: 8),
            Text('EMERGENCY STOP'),
          ],
        ),
        content: Text(
            'This will immediately halt all AMR operations across the fleet. Confirm emergency stop?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _performEmergencyStop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child:
                Text('EMERGENCY STOP', style: TextStyle(color: Colors.white)),
          ),
        ],
      ),
    );
  }

  // Helper methods
  int _getStationCount() {
    if (_selectedDeviceId == 'all' || _selectedDeviceId == null) {
      return _deviceStations.values
          .fold(0, (total, stations) => total + stations.length);
    }
    return _deviceStations[_selectedDeviceId]?.length ?? 0;
  }

  Color _getStationTypeColor(String type) {
    switch (type.toLowerCase()) {
      case 'pickup':
      case 'pick_up':
        return Colors.green;
      case 'drop':
        return Colors.blue;
      case 'charging':
      case 'charge_1':
      case 'charge_2':
        return Colors.orange;
      case 'waypoint':
        return Colors.purple;
      default:
        return Colors.grey;
    }
  }

  IconData _getStationTypeIcon(String type) {
    switch (type.toLowerCase()) {
      case 'pickup':
      case 'pick_up':
        return Icons.outbox;
      case 'drop':
        return Icons.inbox;
      case 'charging':
      case 'charge_1':
      case 'charge_2':
        return Icons.battery_charging_full;
      case 'waypoint':
        return Icons.place;
      default:
        return Icons.location_on;
    }
  }

  void _handleStationAction(MapShape station, String action) {
    switch (action) {
      case 'edit':
        _editStation(station);
        break;
      case 'orders':
        _viewStationOrders(station);
        break;
      case 'test':
        _testStationNavigation(station);
        break;
      case 'delete':
        _deleteStation(station);
        break;
    }
  }

  void _editStation(MapShape station) {
    _showMessage('Station editing feature coming soon!', Colors.blue);
  }

  void _viewStationOrders(MapShape station) {
    // Show orders that use this station
    final stationOrders = _orders.where((order) {
      final waypoints = order['waypoints'] as List<dynamic>? ?? [];
      return waypoints.any((wp) =>
          wp['metadata']?['stationId'] == station.id ||
          wp['name'] == station.name);
    }).toList();

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Orders for ${station.name}'),
        content: Container(
          width: 400,
          height: 300,
          child: stationOrders.isEmpty
              ? Center(child: Text('No orders use this station'))
              : ListView.builder(
                  itemCount: stationOrders.length,
                  itemBuilder: (context, index) {
                    final order = stationOrders[index];
                    return ListTile(
                      title: Text(order['name'] ?? 'Order'),
                      subtitle: Text('Status: ${order['status']}'),
                      trailing:
                          Text(order['createdAt']?.substring(0, 10) ?? ''),
                    );
                  },
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

  void _testStationNavigation(MapShape station) async {
    try {
      // Send test navigation command to station coordinates
      final response = await _apiService.publishGoal(
        _selectedDeviceId!,
        station.center.x,
        station.center.y,
      );

      if (response['success'] == true) {
        _showMessage('Test navigation sent to ${station.name}', Colors.green);
      } else {
        throw Exception(response['error'] ?? 'Navigation test failed');
      }
    } catch (e) {
      _showMessage('Navigation test failed: $e', Colors.red);
    }
  }

  void _deleteStation(MapShape station) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Delete Station'),
        content: Text(
            'Are you sure you want to delete "${station.name}"? This action cannot be undone.'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _showMessage(
                  'Station deletion feature coming soon!', Colors.orange);
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child: Text('Delete'),
          ),
        ],
      ),
    );
  }

  Future<void> _executeOrder(Map<String, dynamic> order) async {
    try {
      // Get waypoints and publish navigation goals
      final waypoints = order['waypoints'] as List<dynamic>? ?? [];

      for (int i = 0; i < waypoints.length; i++) {
        final waypoint = waypoints[i];
        final position = waypoint['position'] as Map<String, dynamic>? ?? {};

        final x = position['x']?.toDouble() ?? 0.0;
        final y = position['y']?.toDouble() ?? 0.0;

        print(
            ' Publishing navigation goal ${i + 1}/${waypoints.length}: ($x, $y)');

        // Publish to target_pose topic
        final response = await _apiService.publishGoal(order['deviceId'], x, y);

        if (response['success'] != true) {
          throw Exception(
              'Failed to publish waypoint ${i + 1}: ${response['error']}');
        }

        // Wait between waypoints (3 seconds as mentioned in requirements)
        if (i < waypoints.length - 1) {
          await Future.delayed(Duration(seconds: 3));
        }
      }

      _showMessage('Order execution completed successfully!', Colors.green);
    } catch (e) {
      _showMessage('Order execution failed: $e', Colors.red);
    }
  }

  Future<void> _performEmergencyStop() async {
    try {
      // Send emergency stop to all devices
      final devices = _selectedDeviceId == 'all'
          ? widget.availableDevices.map((d) => d['id'] as String).toList()
          : [_selectedDeviceId!];

      for (final deviceId in devices) {
        await _apiService.emergencyStop(deviceId);
      }

      _showMessage('EMERGENCY STOP activated for all devices!', Colors.red);
    } catch (e) {
      _showMessage('Emergency stop failed: $e', Colors.red);
    }
  }

  void _showMessage(String message, Color color) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: color,
        duration: Duration(seconds: 3),
      ),
    );
  }

  @override
  void dispose() {
    _tabController.dispose();
    super.dispose();
  }
}
