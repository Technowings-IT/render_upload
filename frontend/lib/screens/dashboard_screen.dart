// screens/dashboard_screen.dart - Enhanced with Order Sequence Management
import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import '../models/odom.dart';
import '../models/map_data.dart';
import 'profile_screen.dart';
import 'map_page.dart'; // Make sure this import points to the file where MapPage is defined
import 'control_page.dart';
import '../widgets/order_creation_dialog.dart';

class DashboardScreen extends StatefulWidget {
  @override
  _DashboardScreenState createState() => _DashboardScreenState();
}

class _DashboardScreenState extends State<DashboardScreen> 
    with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();
  
  // Animation controllers
  late TabController _mainTabController;
  late AnimationController _refreshAnimationController;
  late AnimationController _orderAnimationController;
  
  // Data
  List<Map<String, dynamic>> _connectedDevices = [];
  Map<String, OdometryData> _latestOdometry = {};
  Map<String, Map<String, dynamic>> _batteryStates = {};
  Map<String, Map<String, dynamic>> _robotStates = {};
  Map<String, List<Map<String, dynamic>>> _deviceOrders = {};
  Map<String, MapData> _availableMaps = {};
  List<Map<String, dynamic>> _recentAlerts = [];
  Map<String, dynamic> _systemStats = {};
  
  // UI State
  bool _isLoading = true;
  bool _isRefreshing = false;
  int _currentTabIndex = 0;
  Timer? _refreshTimer;
  String? _selectedDeviceFilter;
  
  // Stream subscriptions
  late StreamSubscription _realTimeSubscription;
  late StreamSubscription _deviceEventsSubscription;
  late StreamSubscription _orderEventsSubscription;

  @override
  void initState() {
    super.initState();
    _initializeControllers();
    _initializeDashboard();
    _subscribeToUpdates();
    _startPeriodicRefresh();
  }

  void _initializeControllers() {
    _mainTabController = TabController(length: 4, vsync: this);
    _refreshAnimationController = AnimationController(
      duration: const Duration(milliseconds: 1000),
      vsync: this,
    );
    _orderAnimationController = AnimationController(
      duration: const Duration(milliseconds: 300),
      vsync: this,
    );
  }

  void _initializeDashboard() async {
    try {
      await Future.wait([
        _loadDevices(),
        _loadMapsForAllDevices(),
        _loadOrdersForAllDevices(),
        _loadSystemStats(),
      ]);
    } catch (e) {
      print('❌ Error initializing dashboard: $e');
      _showErrorSnackBar('Failed to load dashboard data: $e');
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  void _subscribeToUpdates() {
    _realTimeSubscription = _webSocketService.realTimeData.listen((data) {
      _handleRealTimeData(data);
    });

    _deviceEventsSubscription = _webSocketService.deviceEvents.listen((event) {
      _handleDeviceEvent(event);
    });

    _orderEventsSubscription = _webSocketService.orderEvents.listen((event) {
      _handleOrderEvent(event);
    });
  }

  void _handleRealTimeData(Map<String, dynamic> data) {
    final deviceId = data['deviceId'];
    if (deviceId == null) return;

    setState(() {
      switch (data['type']) {
        case 'odometry_update':
          try {
            _latestOdometry[deviceId] = OdometryData.fromJson(data['data']);
          } catch (e) {
            print('❌ Error parsing odometry data: $e');
          }
          break;
        case 'battery_update':
          _batteryStates[deviceId] = data['data'];
          break;
        case 'robot_state':
          _robotStates[deviceId] = data['data'];
          break;
      }
    });
  }

  void _handleDeviceEvent(Map<String, dynamic> event) {
    switch (event['type']) {
      case 'device_connected':
      case 'device_disconnected':
        _loadDevices();
        break;
    }
  }

  void _handleOrderEvent(Map<String, dynamic> event) {
    switch (event['type']) {
      case 'order_created':
      case 'order_status_changed':
      case 'order_completed':
        _loadOrdersForAllDevices();
        _loadSystemStats();
        break;
    }
  }

  void _startPeriodicRefresh() {
    _refreshTimer = Timer.periodic(Duration(seconds: 30), (timer) {
      if (mounted) {
        _refreshData();
      }
    });
  }

  Future<void> _loadDevices() async {
    try {
      final devices = await _apiService.getDevices();
      setState(() {
        _connectedDevices = devices;
      });
    } catch (e) {
      print('❌ Error loading devices: $e');
    }
  }

  Future<void> _loadMapsForAllDevices() async {
    for (final device in _connectedDevices) {
      try {
        final response = await _apiService.getMapData(device['id']);
        if (response['success'] == true && response['mapData'] != null) {
          final mapData = MapData.fromJson(response['mapData']);
          setState(() {
            _availableMaps[device['id']] = mapData;
          });
        }
      } catch (e) {
        print('❌ Error loading map for ${device['id']}: $e');
      }
    }
  }

  Future<void> _loadOrdersForAllDevices() async {
    for (final device in _connectedDevices) {
      try {
        final orders = await _apiService.getOrders(device['id']);
        setState(() {
          _deviceOrders[device['id']] = orders;
        });
      } catch (e) {
        print('❌ Error loading orders for ${device['id']}: $e');
      }
    }
  }

Future<void> _loadSystemStats() async {
  try {
    final response = await _apiService.getSystemOrderStats();
    if (response['success'] == true && response['stats'] != null && response['stats'] is Map<String, dynamic>) {
      setState(() {
        _systemStats = response['stats'];
      });
    } else {
      setState(() {
        _systemStats = {}; // fallback to empty map
      });
      print('⚠️ System stats missing or invalid: $response');
    }
  } catch (e) {
    print('❌ Error loading system stats: $e');
    setState(() {
      _systemStats = {}; // fallback to empty map on error
    });
  }
}

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('AGV Fleet Dashboard'),
        backgroundColor: Theme.of(context).primaryColor,
        elevation: 2,
        actions: [
          IconButton(
            icon: Stack(
              children: [
                Icon(Icons.notifications),
                if (_recentAlerts.isNotEmpty)
                  Positioned(
                    right: 0,
                    top: 0,
                    child: Container(
                      padding: EdgeInsets.all(2),
                      decoration: BoxDecoration(
                        color: Colors.red,
                        borderRadius: BorderRadius.circular(6),
                      ),
                      constraints: BoxConstraints(minWidth: 12, minHeight: 12),
                      child: Text(
                        '${_recentAlerts.length}',
                        style: TextStyle(color: Colors.white, fontSize: 8),
                        textAlign: TextAlign.center,
                      ),
                    ),
                  ),
              ],
            ),
            onPressed: _showAlertsDialog,
          ),
          RotationTransition(
            turns: _refreshAnimationController,
            child: IconButton(
              icon: Icon(Icons.refresh),
              onPressed: _isRefreshing ? null : _refreshDashboard,
            ),
          ),
          IconButton(
            icon: Icon(Icons.person),
            onPressed: () => Navigator.push(
              context,
              MaterialPageRoute(builder: (context) => ProfileScreen()),
            ),
          ),
        ],
        bottom: TabBar(
          controller: _mainTabController,
          indicatorColor: Colors.white,
          labelColor: Colors.white,
          unselectedLabelColor: Colors.white70,
          isScrollable: false,
          onTap: (index) {
            setState(() {
              _currentTabIndex = index;
            });
          },
          tabs: [
            Tab(icon: Icon(Icons.dashboard, size: 20), text: 'Overview'),
            Tab(icon: Icon(Icons.devices, size: 20), text: 'Devices'),
            Tab(icon: Icon(Icons.assignment, size: 20), text: 'Orders'),
            Tab(icon: Icon(Icons.map, size: 20), text: 'Maps'),
          ],
        ),
      ),
      body: _isLoading
          ? Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  CircularProgressIndicator(),
                  SizedBox(height: 16),
                  Text('Loading dashboard...'),
                ],
              ),
            )
          : TabBarView(
              controller: _mainTabController,
              children: [
                _buildOverviewTab(),
                _buildDevicesTab(),
                _buildOrdersTab(),
                _buildMapsTab(),
              ],
            ),
      floatingActionButton: _buildFloatingActionButton(),
    );
  }

  Widget? _buildFloatingActionButton() {
    switch (_currentTabIndex) {
      case 2: // Orders tab
        return FloatingActionButton.extended(
          onPressed: _showCreateOrderDialog,
          icon: Icon(Icons.add),
          label: Text('Create Order'),
          backgroundColor: Colors.green,
          heroTag: "create_order",
        );
      case 1: // Devices tab
        return FloatingActionButton(
          onPressed: () => Navigator.pushNamed(context, '/connect'),
          child: Icon(Icons.add),
          tooltip: 'Add Device',
          backgroundColor: Colors.blue,
          heroTag: "add_device",
        );
      case 3: // Maps tab
        return FloatingActionButton(
          onPressed: () => Navigator.push(
            context,
            MaterialPageRoute(builder: (context) => EnhancedMapPage()),
          ),
          child: Icon(Icons.edit),
          tooltip: 'Edit Map',
          backgroundColor: Colors.purple,
          heroTag: "edit_map",
        );
      default:
        return null;
    }
  }

  Widget _buildOverviewTab() {
    return RefreshIndicator(
      onRefresh: _refreshDashboard,
      child: SingleChildScrollView(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            _buildOverviewCards(),
            SizedBox(height: 20),
            _buildSystemHealthStatus(),
            SizedBox(height: 20),
            _buildQuickActions(),
            SizedBox(height: 20),
            _buildActiveOrdersSummary(),
            SizedBox(height: 20),
            _buildRecentActivity(),
          ],
        ),
      ),
    );
  }

  Widget _buildDevicesTab() {
    return Column(
      children: [
        // Device filter and controls
        Container(
          padding: EdgeInsets.all(16),
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [Colors.blue.shade50, Colors.blue.shade100],
            ),
            border: Border(bottom: BorderSide(color: Colors.blue.shade200)),
          ),
          child: Row(
            children: [
              Icon(Icons.devices, color: Colors.blue.shade700, size: 28),
              SizedBox(width: 12),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Device Management',
                      style: TextStyle(
                        fontSize: 20,
                        fontWeight: FontWeight.bold,
                        color: Colors.blue.shade700,
                      ),
                    ),
                    Text(
                      '${_connectedDevices.length} devices available',
                      style: TextStyle(
                        fontSize: 14,
                        color: Colors.blue.shade600,
                      ),
                    ),
                  ],
                ),
              ),
              _buildDeviceFilter(),
            ],
          ),
        ),
        
        // Device grid
        Expanded(
          child: _buildDeviceGrid(),
        ),
      ],
    );
  }

  Widget _buildOrdersTab() {
    return Column(
      children: [
        // Order management header
        Container(
          padding: EdgeInsets.all(16),
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [Colors.green.shade50, Colors.green.shade100],
            ),
            border: Border(bottom: BorderSide(color: Colors.green.shade200)),
          ),
          child: Column(
            children: [
              Row(
                children: [
                  Icon(Icons.assignment, color: Colors.green.shade700, size: 28),
                  SizedBox(width: 12),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          'Order Management',
                          style: TextStyle(
                            fontSize: 20,
                            fontWeight: FontWeight.bold,
                            color: Colors.green.shade700,
                          ),
                        ),
                        Text(
                          'Create A→B→C→D sequences: Pickup → Drop → Charging → Next Drop',
                          style: TextStyle(
                            fontSize: 14,
                            color: Colors.green.shade600,
                          ),
                        ),
                      ],
                    ),
                  ),
                  ElevatedButton.icon(
                    onPressed: _showCreateOrderDialog,
                    icon: Icon(Icons.add),
                    label: Text('New Order'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.green,
                      foregroundColor: Colors.white,
                    ),
                  ),
                ],
              ),
              SizedBox(height: 12),
              _buildOrderStatsRow(),
            ],
          ),
        ),
        
        // Order list
        Expanded(
          child: _buildOrdersList(),
        ),
      ],
    );
  }

  Widget _buildMapsTab() {
    return Column(
      children: [
        // Maps header
        Container(
          padding: EdgeInsets.all(16),
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [Colors.purple.shade50, Colors.purple.shade100],
            ),
            border: Border(bottom: BorderSide(color: Colors.purple.shade200)),
          ),
          child: Row(
            children: [
              Icon(Icons.map, color: Colors.purple.shade700, size: 28),
              SizedBox(width: 12),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Map & Station Management',
                      style: TextStyle(
                        fontSize: 20,
                        fontWeight: FontWeight.bold,
                        color: Colors.purple.shade700,
                      ),
                    ),
                    Text(
                      'Define pickup, drop, and charging stations for order sequences',
                      style: TextStyle(
                        fontSize: 14,
                        color: Colors.purple.shade600,
                      ),
                    ),
                  ],
                ),
              ),
              ElevatedButton.icon(
                onPressed: () => Navigator.push(
                  context,
                  MaterialPageRoute(builder: (context) => EnhancedMapPage()),
                ),
                icon: Icon(Icons.edit),
                label: Text('Edit Maps'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.purple,
                  foregroundColor: Colors.white,
                ),
              ),
            ],
          ),
        ),
        
        // Maps list
        Expanded(
          child: _buildMapsList(),
        ),
      ],
    );
  }

  Widget _buildOverviewCards() {
    final totalDevices = _connectedDevices.length;
    final onlineDevices = _connectedDevices.where((d) => d['status'] == 'connected').length;
    final totalOrders = _systemStats['total'] ?? 0;
    final activeOrders = _systemStats['byStatus']?['active'] ?? 0;

    return LayoutBuilder(
      builder: (context, constraints) {
        final isWideScreen = constraints.maxWidth > 600;
        
        if (isWideScreen) {
          return Row(
            children: [
              Expanded(child: _buildOverviewCard('Total Devices', totalDevices.toString(), Icons.devices, Colors.blue)),
              SizedBox(width: 12),
              Expanded(child: _buildOverviewCard('Online', '$onlineDevices/$totalDevices', Icons.wifi, onlineDevices == totalDevices ? Colors.green : Colors.orange)),
              SizedBox(width: 12),
              Expanded(child: _buildOverviewCard('Total Orders', totalOrders.toString(), Icons.list_alt, Colors.orange)),
              SizedBox(width: 12),
              Expanded(child: _buildOverviewCard('Active', activeOrders.toString(), Icons.play_circle, Colors.purple)),
            ],
          );
        } else {
          return Column(
            children: [
              Row(
                children: [
                  Expanded(child: _buildOverviewCard('Devices', totalDevices.toString(), Icons.devices, Colors.blue)),
                  SizedBox(width: 12),
                  Expanded(child: _buildOverviewCard('Online', '$onlineDevices/$totalDevices', Icons.wifi, onlineDevices == totalDevices ? Colors.green : Colors.orange)),
                ],
              ),
              SizedBox(height: 12),
              Row(
                children: [
                  Expanded(child: _buildOverviewCard('Orders', totalOrders.toString(), Icons.list_alt, Colors.orange)),
                  SizedBox(width: 12),
                  Expanded(child: _buildOverviewCard('Active', activeOrders.toString(), Icons.play_circle, Colors.purple)),
                ],
              ),
            ],
          );
        }
      },
    );
  }

  Widget _buildOverviewCard(String title, String value, IconData icon, Color color) {
    return Card(
      elevation: 4,
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          children: [
            Icon(icon, size: 32, color: color),
            SizedBox(height: 8),
            FittedBox(
              child: Text(
                value,
                style: TextStyle(
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                  color: color,
                ),
              ),
            ),
            SizedBox(height: 4),
            FittedBox(
              child: Text(
                title,
                style: TextStyle(
                  fontSize: 11,
                  color: Colors.grey[600],
                ),
                textAlign: TextAlign.center,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSystemHealthStatus() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'System Health',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 12),
            Row(
              children: [
                Expanded(child: _buildHealthIndicator('Backend', true, 'Online')),
                SizedBox(width: 16),
                Expanded(child: _buildHealthIndicator('ROS2', true, 'Connected')),
                SizedBox(width: 16),
                Expanded(child: _buildHealthIndicator('WebSocket', true, 'Active')),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildHealthIndicator(String name, bool isHealthy, String status) {
    final color = isHealthy ? Colors.green : Colors.red;
    return Row(
      children: [
        Container(
          width: 12,
          height: 12,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            color: color,
          ),
        ),
        SizedBox(width: 8),
        Expanded(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                name,
                style: TextStyle(fontWeight: FontWeight.bold, fontSize: 12),
              ),
              Text(
                status,
                style: TextStyle(color: color, fontSize: 10),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildQuickActions() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Quick Actions',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 12),
            Wrap(
              spacing: 12,
              runSpacing: 8,
              children: [
                _buildQuickActionButton(
                  'Create Order',
                  Icons.add_task,
                  Colors.green,
                  _showCreateOrderDialog,
                ),
                _buildQuickActionButton(
                  'Add Device',
                  Icons.add,
                  Colors.blue,
                  () => Navigator.pushNamed(context, '/connect'),
                ),
                _buildQuickActionButton(
                  'Edit Maps',
                  Icons.edit,
                  Colors.purple,
                  () => Navigator.push(
                    context,
                    MaterialPageRoute(builder: (context) => EnhancedMapPage()),
                  ),
                ),
                _buildQuickActionButton(
                  'Emergency Stop',
                  Icons.emergency,
                  Colors.red,
                  _showEmergencyStopDialog,
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildQuickActionButton(String label, IconData icon, Color color, VoidCallback onPressed) {
    return ElevatedButton.icon(
      onPressed: onPressed,
      icon: Icon(icon, size: 18),
      label: Text(label, style: TextStyle(fontSize: 12)),
      style: ElevatedButton.styleFrom(
        backgroundColor: color,
        foregroundColor: Colors.white,
        padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      ),
    );
  }

  Widget _buildOrderStatsRow() {
    final stats = _systemStats['byStatus'] ?? {};
    return Container(
      padding: EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.8),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        children: [
          Expanded(child: _buildOrderStatItem('Pending', stats['pending'] ?? 0, Colors.orange)),
          _buildStatDivider(),
          Expanded(child: _buildOrderStatItem('Active', stats['active'] ?? 0, Colors.blue)),
          _buildStatDivider(),
          Expanded(child: _buildOrderStatItem('Completed', stats['completed'] ?? 0, Colors.green)),
          _buildStatDivider(),
          Expanded(child: _buildOrderStatItem('Failed', stats['failed'] ?? 0, Colors.red)),
        ],
      ),
    );
  }

  Widget _buildOrderStatItem(String label, int count, Color color) {
    return Column(
      children: [
        Text(
          count.toString(),
          style: TextStyle(
            fontSize: 18,
            fontWeight: FontWeight.bold,
            color: color,
          ),
        ),
        Text(
          label,
          style: TextStyle(
            fontSize: 11,
            color: Colors.grey[600],
          ),
        ),
      ],
    );
  }

  Widget _buildStatDivider() {
    return Container(
      width: 1,
      height: 30,
      color: Colors.grey.shade300,
      margin: EdgeInsets.symmetric(horizontal: 8),
    );
  }

  Widget _buildDeviceFilter() {
    return Container(
      padding: EdgeInsets.symmetric(horizontal: 12),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.blue.shade300),
      ),
      child: DropdownButtonHideUnderline(
        child: DropdownButton<String>(
          value: _selectedDeviceFilter,
          hint: Text('Filter'),
          items: [
            DropdownMenuItem(value: null, child: Text('All Devices')),
            DropdownMenuItem(value: 'online', child: Text('Online Only')),
            DropdownMenuItem(value: 'with_orders', child: Text('With Orders')),
            DropdownMenuItem(value: 'mapping', child: Text('Mapping')),
          ],
          onChanged: (value) {
            setState(() {
              _selectedDeviceFilter = value;
            });
          },
        ),
      ),
    );
  }

  Widget _buildDeviceGrid() {
    var filteredDevices = _connectedDevices;
    
    if (_selectedDeviceFilter != null) {
      switch (_selectedDeviceFilter) {
        case 'online':
          filteredDevices = _connectedDevices.where((d) => d['status'] == 'connected').toList();
          break;
        case 'with_orders':
          filteredDevices = _connectedDevices.where((d) => (_deviceOrders[d['id']]?.length ?? 0) > 0).toList();
          break;
        case 'mapping':
          filteredDevices = _connectedDevices.where((d) => d['mappingStatus']?['active'] == true).toList();
          break;
      }
    }

    if (filteredDevices.isEmpty) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.devices_other, size: 64, color: Colors.grey),
            SizedBox(height: 16),
            Text(
              _connectedDevices.isEmpty ? 'No Devices Connected' : 'No devices match the filter',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            Text(_connectedDevices.isEmpty ? 'Connect your AGV devices to get started' : 'Try changing the filter'),
            SizedBox(height: 16),
            ElevatedButton(
              onPressed: () => Navigator.pushNamed(context, '/connect'),
              child: Text('Connect Device'),
            ),
          ],
        ),
      );
    }

    return ListView.builder(
      padding: EdgeInsets.all(16),
      itemCount: filteredDevices.length,
      itemBuilder: (context, index) {
        final device = filteredDevices[index];
        return _buildDeviceCard(device);
      },
    );
  }

  Widget _buildDeviceCard(Map<String, dynamic> device) {
    final deviceId = device['id'];
    final isOnline = device['status'] == 'connected';
    final hasMap = _availableMaps.containsKey(deviceId);
    final orderCount = _deviceOrders[deviceId]?.length ?? 0;
    final activeOrders = _deviceOrders[deviceId]?.where((o) => o['status'] == 'active').length ?? 0;
    
    return Card(
      margin: EdgeInsets.symmetric(vertical: 4),
      child: InkWell(
        onTap: () => _navigateToControl(device),
        child: Padding(
          padding: EdgeInsets.all(16),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              // Header
              Row(
                children: [
                  Container(
                    width: 50,
                    height: 50,
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        colors: isOnline 
                            ? [Colors.green.shade400, Colors.green.shade600]
                            : [Colors.grey.shade400, Colors.grey.shade600],
                      ),
                      borderRadius: BorderRadius.circular(8),
                    ),
                    child: Icon(Icons.smart_toy, color: Colors.white),
                  ),
                  SizedBox(width: 16),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          device['name'] ?? device['id'],
                          style: TextStyle(
                            fontWeight: FontWeight.bold,
                            fontSize: 16,
                          ),
                        ),
                        Text(
                          'ID: ${device['id']}',
                          style: TextStyle(color: Colors.grey[600]),
                        ),
                        Text(
                          'IP: ${device['ipAddress'] ?? 'Unknown'}',
                          style: TextStyle(color: Colors.grey[600], fontSize: 12),
                        ),
                      ],
                    ),
                  ),
                  Container(
                    width: 12,
                    height: 12,
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: isOnline ? Colors.green : Colors.red,
                    ),
                  ),
                ],
              ),
              
              SizedBox(height: 12),
              
              // Status indicators
              Row(
                children: [
                  _buildStatusChip(
                    hasMap ? 'Map Available' : 'No Map',
                    hasMap ? Colors.green : Colors.orange,
                    hasMap ? Icons.check_circle : Icons.warning,
                  ),
                  SizedBox(width: 8),
                  _buildStatusChip(
                    '$orderCount Orders',
                    orderCount > 0 ? Colors.blue : Colors.grey,
                    Icons.assignment,
                  ),
                  if (activeOrders > 0) ...[
                    SizedBox(width: 8),
                    _buildStatusChip(
                      '$activeOrders Active',
                      Colors.purple,
                      Icons.play_circle,
                    ),
                  ],
                ],
              ),
              
              SizedBox(height: 12),
              
              // Action buttons
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton.icon(
                      onPressed: () => _showCreateOrderForDevice(deviceId),
                      icon: Icon(Icons.add_task, size: 16),
                      label: Text('Create Order'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.green,
                        foregroundColor: Colors.white,
                        padding: EdgeInsets.symmetric(vertical: 8),
                      ),
                    ),
                  ),
                  SizedBox(width: 8),
                  Expanded(
                    child: OutlinedButton.icon(
                      onPressed: () => _navigateToControl(device),
                      icon: Icon(Icons.gamepad, size: 16),
                      label: Text('Control'),
                      style: OutlinedButton.styleFrom(
                        padding: EdgeInsets.symmetric(vertical: 8),
                      ),
                    ),
                  ),
                  SizedBox(width: 8),
                  Expanded(
                    child: OutlinedButton.icon(
                      onPressed: () => _navigateToMap(device),
                      icon: Icon(Icons.map, size: 16),
                      label: Text('Map'),
                      style: OutlinedButton.styleFrom(
                        padding: EdgeInsets.symmetric(vertical: 8),
                      ),
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildStatusChip(String text, Color color, IconData icon) {
    return Container(
      padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.3)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 12, color: color),
          SizedBox(width: 4),
          Text(
            text,
            style: TextStyle(
              fontSize: 10,
              color: color,
              fontWeight: FontWeight.w600,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildOrdersList() {
    final allOrders = <Map<String, dynamic>>[];
    _deviceOrders.forEach((deviceId, orders) {
      for (final order in orders) {
        allOrders.add({
          ...order,
          'deviceId': deviceId,
          'deviceName': _connectedDevices
              .firstWhere((d) => d['id'] == deviceId, orElse: () => {'name': deviceId})['name']
        });
      }
    });

    // Sort by creation time (newest first)
    allOrders.sort((a, b) {
      final aTime = DateTime.tryParse(a['createdAt'] ?? '') ?? DateTime.now();
      final bTime = DateTime.tryParse(b['createdAt'] ?? '') ?? DateTime.now();
      return bTime.compareTo(aTime);
    });

    if (allOrders.isEmpty) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.assignment, size: 64, color: Colors.grey),
            SizedBox(height: 16),
            Text(
              'No Orders Created',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            Text('Create your first A→B→C→D sequence to get started'),
            SizedBox(height: 16),
            ElevatedButton.icon(
              onPressed: _showCreateOrderDialog,
              icon: Icon(Icons.add),
              label: Text('Create Order Sequence'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.green,
                foregroundColor: Colors.white,
              ),
            ),
          ],
        ),
      );
    }

    return ListView.builder(
      padding: EdgeInsets.all(16),
      itemCount: allOrders.length,
      itemBuilder: (context, index) {
        final order = allOrders[index];
        return _buildDetailedOrderCard(order);
      },
    );
  }

  Widget _buildDetailedOrderCard(Map<String, dynamic> order) {
    final status = order['status'] ?? 'pending';
    final statusColor = _getOrderStatusColor(status);
    final waypoints = order['waypoints'] as List? ?? [];
    final progress = order['progress'] as Map<String, dynamic>? ?? {};

    return Card(
      margin: EdgeInsets.symmetric(vertical: 4),
      child: ExpansionTile(
        leading: CircleAvatar(
          backgroundColor: statusColor,
          child: Icon(_getOrderStatusIcon(status), color: Colors.white),
        ),
        title: Text(
          order['name'] ?? 'Order',
          style: TextStyle(fontWeight: FontWeight.bold),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Device: ${order['deviceName'] ?? order['deviceId']}'),
            Text('Sequence: ${waypoints.length} stations • ${progress['percentage'] ?? 0}% complete'),
            SizedBox(height: 4),
            LinearProgressIndicator(
              value: (progress['percentage'] ?? 0) / 100.0,
              backgroundColor: Colors.grey.shade300,
              valueColor: AlwaysStoppedAnimation<Color>(statusColor),
            ),
          ],
        ),
        trailing: Container(
          padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
          decoration: BoxDecoration(
            color: statusColor,
            borderRadius: BorderRadius.circular(12),
          ),
          child: Text(
            status.toUpperCase(),
            style: TextStyle(
              color: Colors.white,
              fontSize: 10,
              fontWeight: FontWeight.bold,
            ),
          ),
        ),
        children: [
          Container(
            padding: EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                if (waypoints.isNotEmpty) ...[
                  Text(
                    'Order Sequence (A→B→C→D):',
                    style: TextStyle(fontWeight: FontWeight.bold),
                  ),
                  SizedBox(height: 8),
                  ...waypoints.asMap().entries.map((entry) {
                    final index = entry.key;
                    final waypoint = entry.value;
                    return _buildWaypointSequenceItem(index + 1, waypoint, index < (order['currentWaypoint'] ?? 0));
                  }),
                  SizedBox(height: 16),
                ],
                _buildOrderActionButtons(order),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildWaypointSequenceItem(int step, Map<String, dynamic> waypoint, bool isCompleted) {
    final type = waypoint['type'] ?? 'waypoint';
    final name = waypoint['name'] ?? 'Station $step';
    final color = _getStationTypeColor(type);
    final icon = _getStationTypeIcon(type);
    final stepLabel = _getStepLabel(step, type);

    return Container(
      margin: EdgeInsets.symmetric(vertical: 2),
      padding: EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: isCompleted ? color.withOpacity(0.2) : Colors.grey.shade50,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(
          color: isCompleted ? color : Colors.grey.shade300,
          width: isCompleted ? 2 : 1,
        ),
      ),
      child: Row(
        children: [
          Container(
            width: 28,
            height: 28,
            decoration: BoxDecoration(
              color: isCompleted ? color : Colors.grey.shade400,
              shape: BoxShape.circle,
            ),
            child: Center(
              child: isCompleted 
                  ? Icon(Icons.check, color: Colors.white, size: 16)
                  : Text(
                      stepLabel,
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 12,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
            ),
          ),
          SizedBox(width: 12),
          Icon(icon, color: color, size: 20),
          SizedBox(width: 8),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  name,
                  style: TextStyle(
                    fontWeight: FontWeight.w600,
                    color: isCompleted ? color : Colors.black87,
                  ),
                ),
                Text(
                  '${type.toUpperCase()} STATION',
                  style: TextStyle(
                    fontSize: 11,
                    color: color,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
          ),
          if (step < 4 && step < waypoint.length) // Show arrow if not last step
            Icon(Icons.arrow_forward, color: Colors.grey.shade400),
        ],
      ),
    );
  }

  String _getStepLabel(int step, String type) {
    switch (step) {
      case 1: return 'A';
      case 2: return 'B';
      case 3: return 'C';
      case 4: return 'D';
      default: return step.toString();
    }
  }

  Widget _buildOrderActionButtons(Map<String, dynamic> order) {
    final status = order['status'] ?? 'pending';
    
    return Row(
      children: [
        Expanded(
          child: OutlinedButton.icon(
            onPressed: () => _showOrderDetails(order),
            icon: Icon(Icons.info),
            label: Text('Details'),
          ),
        ),
        SizedBox(width: 8),
        if (status == 'pending')
          Expanded(
            child: ElevatedButton.icon(
              onPressed: () => _executeOrder(order),
              icon: Icon(Icons.play_arrow),
              label: Text('Execute'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.green,
                foregroundColor: Colors.white,
              ),
            ),
          ),
        if (status == 'active')
          Expanded(
            child: ElevatedButton.icon(
              onPressed: () => _pauseOrder(order),
              icon: Icon(Icons.pause),
              label: Text('Pause'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.orange,
                foregroundColor: Colors.white,
              ),
            ),
          ),
        if (status == 'paused')
          Expanded(
            child: ElevatedButton.icon(
              onPressed: () => _executeOrder(order),
              icon: Icon(Icons.play_arrow),
              label: Text('Resume'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.blue,
                foregroundColor: Colors.white,
              ),
            ),
          ),
      ],
    );
  }

  Widget _buildMapsList() {
    if (_availableMaps.isEmpty) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.map, size: 64, color: Colors.grey),
            SizedBox(height: 16),
            Text(
              'No Maps Available',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            Text('Create maps and define stations for order sequences'),
            SizedBox(height: 16),
            ElevatedButton.icon(
              onPressed: () => Navigator.push(
                context,
                MaterialPageRoute(builder: (context) => EnhancedMapPage()),
              ),
              icon: Icon(Icons.edit),
              label: Text('Create Map'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.purple,
                foregroundColor: Colors.white,
              ),
            ),
          ],
        ),
      );
    }

    return ListView.builder(
      padding: EdgeInsets.all(16),
      itemCount: _availableMaps.length,
      itemBuilder: (context, index) {
        final deviceId = _availableMaps.keys.elementAt(index);
        final mapData = _availableMaps[deviceId]!;
        return _buildMapCard(deviceId, mapData);
      },
    );
  }

  Widget _buildMapCard(String deviceId, MapData mapData) {
    final device = _connectedDevices.firstWhere(
      (d) => d['id'] == deviceId,
      orElse: () => {'name': deviceId, 'id': deviceId},
    );
    
    final stationCounts = <String, int>{};
    for (final shape in mapData.shapes) {
      stationCounts[shape.type] = (stationCounts[shape.type] ?? 0) + 1;
    }

    final totalStations = stationCounts.values.fold(0, (sum, count) => sum + count);

    return Card(
      margin: EdgeInsets.symmetric(vertical: 4),
      child: ListTile(
        leading: Container(
          width: 50,
          height: 50,
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [Colors.purple.shade400, Colors.purple.shade600],
            ),
            borderRadius: BorderRadius.circular(8),
          ),
          child: Icon(Icons.map, color: Colors.white),
        ),
        title: Text(
          'Map for ${device['name']}',
          style: TextStyle(fontWeight: FontWeight.bold),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Size: ${mapData.info.width}×${mapData.info.height}'),
            Text('Stations: $totalStations available for sequences'),
            if (stationCounts.isNotEmpty)
              Wrap(
                spacing: 4,
                children: stationCounts.entries.map((entry) {
                  final color = _getStationTypeColor(entry.key);
                  return Chip(
                    label: Text(
                      '${entry.key}: ${entry.value}',
                      style: TextStyle(fontSize: 10),
                    ),
                    backgroundColor: color.withOpacity(0.2),
                    side: BorderSide(color: color),
                  );
                }).toList(),
              ),
          ],
        ),
        trailing: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            IconButton(
              onPressed: () => _navigateToMap({'id': deviceId}),
              icon: Icon(Icons.edit),
              tooltip: 'Edit Map & Stations',
            ),
            IconButton(
              onPressed: () => _showCreateOrderForDevice(deviceId),
              icon: Icon(Icons.add_task),
              tooltip: 'Create Order Sequence',
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildActiveOrdersSummary() {
    final activeOrders = <Map<String, dynamic>>[];
    _deviceOrders.forEach((deviceId, orders) {
      for (final order in orders) {
        if (order['status'] == 'active' || order['status'] == 'pending') {
          activeOrders.add({
            ...order,
            'deviceId': deviceId,
            'deviceName': _connectedDevices
                .firstWhere((d) => d['id'] == deviceId, orElse: () => {'name': deviceId})['name']
          });
        }
      }
    });

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Text(
              'Active Orders',
              style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
            ),
            Spacer(),
            TextButton(
              onPressed: () {
                setState(() {
                  _currentTabIndex = 2;
                  _mainTabController.animateTo(2);
                });
              },
              child: Text('View All'),
            ),
          ],
        ),
        SizedBox(height: 12),
        if (activeOrders.isEmpty)
          Card(
            child: Padding(
              padding: EdgeInsets.all(24),
              child: Row(
                children: [
                  Icon(Icons.check_circle, color: Colors.green, size: 32),
                  SizedBox(width: 16),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          'No active orders',
                          style: TextStyle(fontWeight: FontWeight.bold),
                        ),
                        Text('All AGVs are ready for new order sequences'),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          )
        else
          ListView.builder(
            shrinkWrap: true,
            physics: NeverScrollableScrollPhysics(),
            itemCount: math.min(activeOrders.length, 3), // Show max 3
            itemBuilder: (context, index) {
              final order = activeOrders[index];
              return _buildCompactOrderCard(order);
            },
          ),
      ],
    );
  }

  Widget _buildCompactOrderCard(Map<String, dynamic> order) {
    final status = order['status'] ?? 'pending';
    final statusColor = _getOrderStatusColor(status);
    final progress = order['progress'] as Map<String, dynamic>? ?? {};

    return Card(
      child: ListTile(
        leading: CircleAvatar(
          backgroundColor: statusColor,
          child: Icon(_getOrderStatusIcon(status), color: Colors.white),
        ),
        title: Text(
          order['name'] ?? 'Order',
          overflow: TextOverflow.ellipsis,
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Device: ${order['deviceName'] ?? order['deviceId']}'),
            Text('Progress: ${progress['percentage'] ?? 0}% (${progress['completedWaypoints'] ?? 0}/${progress['totalWaypoints'] ?? 0})'),
            SizedBox(height: 4),
            LinearProgressIndicator(
              value: (progress['percentage'] ?? 0) / 100.0,
              backgroundColor: Colors.grey.shade300,
              valueColor: AlwaysStoppedAnimation<Color>(statusColor),
            ),
          ],
        ),
        trailing: _buildOrderActionIcon(order),
        onTap: () => _showOrderDetails(order),
      ),
    );
  }

  Widget _buildOrderActionIcon(Map<String, dynamic> order) {
    final status = order['status'] ?? 'pending';
    
    switch (status) {
      case 'pending':
        return IconButton(
          icon: Icon(Icons.play_arrow, color: Colors.green),
          onPressed: () => _executeOrder(order),
          tooltip: 'Execute Order',
        );
      case 'active':
        return IconButton(
          icon: Icon(Icons.pause, color: Colors.orange),
          onPressed: () => _pauseOrder(order),
          tooltip: 'Pause Order',
        );
      default:
        return Icon(Icons.schedule, color: Colors.grey);
    }
  }

  Widget _buildRecentActivity() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Recent Activity',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 12),
            if (_recentAlerts.isEmpty)
              Text('No recent activity')
            else
              ListView.builder(
                shrinkWrap: true,
                physics: NeverScrollableScrollPhysics(),
                itemCount: math.min(_recentAlerts.length, 3),
                itemBuilder: (context, index) {
                  final alert = _recentAlerts[index];
                  return ListTile(
                    leading: Icon(Icons.info_outline, color: Colors.blue),
                    title: Text(alert['message'] ?? 'Activity'),
                    subtitle: Text(alert['timestamp'] ?? ''),
                    dense: true,
                  );
                },
              ),
          ],
        ),
      ),
    );
  }

  // Helper methods for colors and icons
  Color _getOrderStatusColor(String status) {
    switch (status) {
      case 'pending': return Colors.orange;
      case 'active': return Colors.blue;
      case 'completed': return Colors.green;
      case 'paused': return Colors.grey;
      case 'failed': return Colors.red;
      case 'cancelled': return Colors.red.shade300;
      default: return Colors.grey;
    }
  }

  IconData _getOrderStatusIcon(String status) {
    switch (status) {
      case 'pending': return Icons.pending;
      case 'active': return Icons.play_circle;
      case 'completed': return Icons.check_circle;
      case 'paused': return Icons.pause_circle;
      case 'failed': return Icons.error;
      case 'cancelled': return Icons.cancel;
      default: return Icons.help;
    }
  }

  Color _getStationTypeColor(String type) {
    switch (type) {
      case 'pickup': return Colors.green;
      case 'drop': return Colors.blue;
      case 'charging': return Colors.orange;
      case 'waypoint': return Colors.purple;
      default: return Colors.grey;
    }
  }

  IconData _getStationTypeIcon(String type) {
    switch (type) {
      case 'pickup': return Icons.outbox;
      case 'drop': return Icons.inbox;
      case 'charging': return Icons.battery_charging_full;
      case 'waypoint': return Icons.place;
      default: return Icons.location_on;
    }
  }

  // Navigation and action methods
  void _navigateToControl(Map<String, dynamic> device) {
    Navigator.push(
      context,
      MaterialPageRoute(
        builder: (context) => ControlPage(deviceId: device['id']),
        settings: RouteSettings(
          arguments: {
            'deviceId': device['id'],
            'deviceName': device['name'],
          },
        ),
      ),
    );
  }

  void _navigateToMap(Map<String, dynamic> device) {
    Navigator.push(
      context,
      MaterialPageRoute(
        builder: (context) => EnhancedMapPage(),
        settings: RouteSettings(
          arguments: {
            'deviceId': device['id'],
          },
        ),
      ),
    );
  }

  void _showCreateOrderDialog() {
    if (_connectedDevices.isEmpty) {
      _showErrorSnackBar('No devices available. Connect a device first.');
      return;
    }

    showDialog(
      context: context,
      builder: (context) => OrderCreationDialog(
        availableDevices: _connectedDevices,
        availableMaps: _availableMaps,
        onOrderCreated: (order) {
          _createOrder(order);
        },
      ),
    );
  }

  void _showCreateOrderForDevice(String deviceId) {
    if (!_availableMaps.containsKey(deviceId)) {
      _showErrorSnackBar('No map available for this device. Create a map with stations first.');
      return;
    }

    final device = _connectedDevices.firstWhere(
      (d) => d['id'] == deviceId,
      orElse: () => {'id': deviceId, 'name': deviceId},
    );

    showDialog(
      context: context,
      builder: (context) => OrderCreationDialog(
        availableDevices: [device],
        availableMaps: {deviceId: _availableMaps[deviceId]!},
        preSelectedDevice: deviceId,
        onOrderCreated: (order) {
          _createOrder(order);
        },
      ),
    );
  }

  Future<void> _createOrder(Map<String, dynamic> orderData) async {
    try {
      final response = await _apiService.createOrder(
        deviceId: orderData['deviceId'],
        name: orderData['name'],
        waypoints: orderData['waypoints'],
        priority: orderData['priority'] ?? 0,
      );

      if (response['success'] == true) {
        _showSuccessSnackBar('Order sequence created successfully!');
        _loadOrdersForAllDevices();
        _loadSystemStats();
      } else {
        _showErrorSnackBar('Failed to create order: ${response['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Error creating order: $e');
    }
  }

  Future<void> _executeOrder(Map<String, dynamic> order) async {
    try {
      final response = await _apiService.updateOrderStatus(
        orderId: order['id'],
        status: 'active',
      );

      if (response['success'] == true) {
        _showSuccessSnackBar('Order execution started!');
        _loadOrdersForAllDevices();
        _loadSystemStats();
      } else {
        _showErrorSnackBar('Failed to execute order: ${response['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Error executing order: $e');
    }
  }

  Future<void> _pauseOrder(Map<String, dynamic> order) async {
    try {
      final response = await _apiService.updateOrderStatus(
        orderId: order['id'],
        status: 'paused',
      );

      if (response['success'] == true) {
        _showSuccessSnackBar('Order paused!');
        _loadOrdersForAllDevices();
        _loadSystemStats();
      } else {
        _showErrorSnackBar('Failed to pause order: ${response['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Error pausing order: $e');
    }
  }

  void _showOrderDetails(Map<String, dynamic> order) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text(order['name'] ?? 'Order Details'),
        content: SingleChildScrollView(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text('Device: ${order['deviceName'] ?? order['deviceId']}'),
              SizedBox(height: 8),
              Text('Status: ${order['status']?.toUpperCase() ?? 'UNKNOWN'}'),
              SizedBox(height: 8),
              Text('Priority: ${order['priority'] ?? 0}'),
              SizedBox(height: 8),
              Text('Sequence Steps: ${order['waypoints']?.length ?? 0}'),
              if (order['createdAt'] != null) ...[
                SizedBox(height: 8),
                Text('Created: ${order['createdAt']}'),
              ],
              if (order['waypoints'] != null && order['waypoints'].isNotEmpty) ...[
                SizedBox(height: 16),
                Text('A→B→C→D Sequence:', style: TextStyle(fontWeight: FontWeight.bold)),
                SizedBox(height: 8),
                ...order['waypoints'].asMap().entries.map((entry) {
                  final index = entry.key;
                  final waypoint = entry.value;
                  return _buildWaypointSequenceItem(index + 1, waypoint, false);
                }),
              ],
            ],
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Close'),
          ),
          if (order['status'] == 'pending')
            ElevatedButton(
              onPressed: () {
                Navigator.of(context).pop();
                _executeOrder(order);
              },
              child: Text('Execute Sequence'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.green,
                foregroundColor: Colors.white,
              ),
            ),
        ],
      ),
    );
  }

  void _showAlertsDialog() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Recent Alerts'),
        content: Container(
          width: double.maxFinite,
          height: 400,
          child: _recentAlerts.isEmpty
              ? Center(child: Text('No alerts'))
              : ListView.builder(
                  itemCount: _recentAlerts.length,
                  itemBuilder: (context, index) {
                    final alert = _recentAlerts[index];
                    return ListTile(
                      leading: Icon(Icons.warning, color: Colors.orange),
                      title: Text(alert['message'] ?? 'Alert'),
                      subtitle: Text(alert['timestamp'] ?? ''),
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

  void _showEmergencyStopDialog() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Emergency Stop'),
        content: Text('Stop all active orders and AGV movement immediately?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _emergencyStopAll();
            },
            child: Text('Emergency Stop'),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.red,
              foregroundColor: Colors.white,
            ),
          ),
        ],
      ),
    );
  }

  void _emergencyStopAll() async {
    try {
      for (final device in _connectedDevices) {
        await _apiService.emergencyStop(device['id']);
      }
      _showSuccessSnackBar('Emergency stop sent to all devices');
    } catch (e) {
      _showErrorSnackBar('Error sending emergency stop: $e');
    }
  }

  Future<void> _refreshDashboard() async {
    setState(() {
      _isRefreshing = true;
    });
    
    _refreshAnimationController.repeat();
    
    try {
      await Future.wait([
        _loadDevices(),
        _loadMapsForAllDevices(), 
        _loadOrdersForAllDevices(),
        _loadSystemStats(),
      ]);
    } catch (e) {
      _showErrorSnackBar('Failed to refresh dashboard: $e');
    } finally {
      setState(() {
        _isRefreshing = false;
      });
      _refreshAnimationController.stop();
      _refreshAnimationController.reset();
    }
  }

  Future<void> _refreshData() async {
    // Background refresh without showing loading
    try {
      await Future.wait([
        _loadOrdersForAllDevices(),
        _loadSystemStats(),
      ]);
    } catch (e) {
      print('❌ Background refresh failed: $e');
    }
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

  @override
  void dispose() {
    _refreshTimer?.cancel();
    _realTimeSubscription.cancel();
    _deviceEventsSubscription.cancel();
    _orderEventsSubscription.cancel();
    _mainTabController.dispose();
    _refreshAnimationController.dispose();
    _orderAnimationController.dispose();
    super.dispose();
  }
}