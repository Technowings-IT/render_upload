//screens/dashboard_screen.dart - Updated for Backend Integration
import 'package:flutter/material.dart';
import 'dart:async';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import '../models/odom.dart';

class DashboardScreen extends StatefulWidget {
  @override
  _DashboardScreenState createState() => _DashboardScreenState();
}

class _DashboardScreenState extends State<DashboardScreen> {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();
  
  List<Map<String, dynamic>> _connectedDevices = [];
  Map<String, OdometryData> _latestOdometry = {};
  Map<String, Map<String, dynamic>> _batteryStates = {};
  Map<String, Map<String, dynamic>> _robotStates = {};
  Map<String, List<Map<String, dynamic>>> _deviceOrders = {};
  List<Map<String, dynamic>> _recentAlerts = [];
  Map<String, dynamic>? _systemHealth;
  
  bool _isLoading = true;
  Timer? _refreshTimer;
  late StreamSubscription _realTimeSubscription;
  late StreamSubscription _deviceEventsSubscription;
  late StreamSubscription _orderEventsSubscription;

  @override
  void initState() {
    super.initState();
    _initializeDashboard();
    _subscribeToUpdates();
    _startPeriodicRefresh();
  }

  @override
  void dispose() {
    _refreshTimer?.cancel();
    _realTimeSubscription.cancel();
    _deviceEventsSubscription.cancel();
    _orderEventsSubscription.cancel();
    super.dispose();
  }

  void _initializeDashboard() async {
    try {
      await _loadSystemHealth();
      await _loadDevices();
      await _loadOrdersForAllDevices();
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
    // Subscribe to real-time data updates
    _realTimeSubscription = _webSocketService.realTimeData.listen((data) {
      _handleRealTimeData(data);
    });

    // Subscribe to device events
    _deviceEventsSubscription = _webSocketService.deviceEvents.listen((event) {
      _handleDeviceEvent(event);
    });

    // Subscribe to order events
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
          _latestOdometry[deviceId] = OdometryData.fromJson(data['data']);
          break;
        case 'battery_update':
          _batteryStates[deviceId] = data['data'];
          _checkBatteryAlert(deviceId, data['data']);
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
      case 'initial_data':
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
        break;
    }
  }

  void _checkBatteryAlert(String deviceId, Map<String, dynamic> batteryData) {
    final percentage = batteryData['percentage'];
    if (percentage != null && percentage < 20) {
      final alert = {
        'type': 'low_battery',
        'deviceId': deviceId,
        'message': 'Low battery warning',
        'percentage': percentage,
        'timestamp': DateTime.now().toIso8601String(),
      };
      
      setState(() {
        _recentAlerts.insert(0, alert);
        if (_recentAlerts.length > 10) {
          _recentAlerts.removeLast();
        }
      });
    }
  }

  void _startPeriodicRefresh() {
    _refreshTimer = Timer.periodic(Duration(seconds: 30), (timer) {
      if (mounted) {
        _refreshSystemHealth();
      }
    });
  }

  Future<void> _loadSystemHealth() async {
    try {
      final health = await _apiService.healthCheck();
      setState(() {
        _systemHealth = health;
      });
    } catch (e) {
      print('❌ Error loading system health: $e');
    }
  }

  Future<void> _refreshSystemHealth() async {
    try {
      await _loadSystemHealth();
    } catch (e) {
      // Silent fail for background refresh
    }
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

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Fleet Management Dashboard'),
        actions: [
          IconButton(
            icon: Icon(Icons.refresh),
            onPressed: _refreshDashboard,
          ),
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
                      constraints: BoxConstraints(
                        minWidth: 12,
                        minHeight: 12,
                      ),
                      child: Text(
                        '${_recentAlerts.length}',
                        style: TextStyle(
                          color: Colors.white,
                          fontSize: 8,
                        ),
                        textAlign: TextAlign.center,
                      ),
                    ),
                  ),
              ],
            ),
            onPressed: _showAlertsDialog,
          ),
        ],
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
          : RefreshIndicator(
              onRefresh: _refreshDashboard,
              child: SingleChildScrollView(
                padding: EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    _buildSystemHealthCard(),
                    SizedBox(height: 20),
                    _buildOverviewCards(),
                    SizedBox(height: 20),
                    _buildDeviceGrid(),
                    SizedBox(height: 20),
                    _buildActiveOrdersSection(),
                    SizedBox(height: 20),
                    _buildRecentAlertsSection(),
                  ],
                ),
              ),
            ),
      floatingActionButton: FloatingActionButton(
        onPressed: () => Navigator.pushNamed(context, '/connect'),
        child: Icon(Icons.add),
        tooltip: 'Add Device',
      ),
    );
  }

  Widget _buildSystemHealthCard() {
    if (_systemHealth == null) return SizedBox.shrink();

    final isHealthy = _systemHealth!['success'] == true;
    final data = _systemHealth!['data'] ?? {};
    
    return Card(
      color: isHealthy ? Colors.green[50] : Colors.red[50],
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(
                  isHealthy ? Icons.check_circle : Icons.error,
                  color: isHealthy ? Colors.green : Colors.red,
                  size: 24,
                ),
                SizedBox(width: 8),
                Text(
                  'System Status: ${data['status']?.toString().toUpperCase() ?? 'UNKNOWN'}',
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                    color: isHealthy ? Colors.green[700] : Colors.red[700],
                  ),
                ),
                Spacer(),
                Text(
                  'ROS2: ${data['ros2Status']?.toString().toUpperCase() ?? 'UNKNOWN'}',
                  style: TextStyle(
                    fontSize: 12,
                    color: Colors.grey[600],
                  ),
                ),
              ],
            ),
            SizedBox(height: 8),
            Row(
              children: [
                Text('Uptime: ${_formatUptime(data['uptime'])}'),
                Spacer(),
                Text('Version: ${data['version'] ?? 'Unknown'}'),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildOverviewCards() {
    final totalDevices = _connectedDevices.length;
    final onlineDevices = _connectedDevices.where((d) => d['status'] == 'connected').length;
    final totalOrders = _deviceOrders.values.fold<int>(0, (sum, orders) => sum + orders.length);
    final activeOrders = _deviceOrders.values
        .expand((orders) => orders)
        .where((order) => order['status'] == 'active' || order['status'] == 'pending')
        .length;

    return Row(
      children: [
        Expanded(
          child: _buildOverviewCard(
            'Total Devices',
            totalDevices.toString(),
            Icons.devices,
            Colors.blue,
          ),
        ),
        SizedBox(width: 12),
        Expanded(
          child: _buildOverviewCard(
            'Online',
            '$onlineDevices/$totalDevices',
            Icons.wifi,
            onlineDevices == totalDevices ? Colors.green : Colors.orange,
          ),
        ),
        SizedBox(width: 12),
        Expanded(
          child: _buildOverviewCard(
            'Total Orders',
            totalOrders.toString(),
            Icons.list_alt,
            Colors.orange,
          ),
        ),
        SizedBox(width: 12),
        Expanded(
          child: _buildOverviewCard(
            'Active',
            activeOrders.toString(),
            Icons.play_circle,
            Colors.purple,
          ),
        ),
      ],
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
            Text(
              value,
              style: TextStyle(
                fontSize: 24,
                fontWeight: FontWeight.bold,
                color: color,
              ),
            ),
            SizedBox(height: 4),
            Text(
              title,
              style: TextStyle(
                fontSize: 12,
                color: Colors.grey[600],
              ),
              textAlign: TextAlign.center,
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildDeviceGrid() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Text(
              'Device Status',
              style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
            ),
            Spacer(),
            TextButton(
              onPressed: () => Navigator.pushNamed(context, '/connect'),
              child: Text('Manage Devices'),
            ),
          ],
        ),
        SizedBox(height: 12),
        if (_connectedDevices.isEmpty)
          Card(
            child: Padding(
              padding: EdgeInsets.all(32),
              child: Column(
                children: [
                  Icon(Icons.devices_other, size: 64, color: Colors.grey),
                  SizedBox(height: 16),
                  Text(
                    'No Devices Connected',
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                  SizedBox(height: 8),
                  Text('Connect your AGV devices to get started'),
                  SizedBox(height: 16),
                  ElevatedButton(
                    onPressed: () => Navigator.pushNamed(context, '/connect'),
                    child: Text('Connect Devices'),
                  ),
                ],
              ),
            ),
          )
        else
          GridView.builder(
            shrinkWrap: true,
            physics: NeverScrollableScrollPhysics(),
            gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
              crossAxisCount: MediaQuery.of(context).size.width > 800 ? 3 : 2,
              crossAxisSpacing: 12,
              mainAxisSpacing: 12,
              childAspectRatio: 1.2,
            ),
            itemCount: _connectedDevices.length,
            itemBuilder: (context, index) {
              final device = _connectedDevices[index];
              return _buildDeviceCard(device);
            },
          ),
      ],
    );
  }

  Widget _buildDeviceCard(Map<String, dynamic> device) {
    final deviceId = device['id'];
    final isOnline = device['status'] == 'connected';
    final odometry = _latestOdometry[deviceId];
    final battery = _batteryStates[deviceId];
    final robotState = _robotStates[deviceId];
    
    final batteryPercentage = battery?['percentage']?.toDouble() ?? 0.0;
    final batteryColor = batteryPercentage > 30 ? Colors.green : 
                        batteryPercentage > 15 ? Colors.orange : Colors.red;

    return Card(
      elevation: 4,
      child: InkWell(
        onTap: () => _navigateToControl(device),
        child: Padding(
          padding: EdgeInsets.all(12),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                children: [
                  Container(
                    width: 12,
                    height: 12,
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: isOnline ? Colors.green : Colors.red,
                    ),
                  ),
                  SizedBox(width: 8),
                  Expanded(
                    child: Text(
                      device['name'] ?? device['id'],
                      style: TextStyle(
                        fontWeight: FontWeight.bold,
                        fontSize: 16,
                      ),
                      overflow: TextOverflow.ellipsis,
                    ),
                  ),
                  PopupMenuButton(
                    itemBuilder: (context) => [
                      PopupMenuItem(
                        value: 'control',
                        child: Text('Control'),
                      ),
                      PopupMenuItem(
                        value: 'map',
                        child: Text('Map Editor'),
                      ),
                      PopupMenuItem(
                        value: 'orders',
                        child: Text('Orders'),
                      ),
                    ],
                    onSelected: (value) => _handleDeviceAction(device, value),
                  ),
                ],
              ),
              SizedBox(height: 8),
              Text(
                'ID: ${device['id']}',
                style: TextStyle(
                  color: Colors.grey[600],
                  fontSize: 12,
                ),
              ),
              Spacer(),
              
              // Battery indicator
              if (battery != null) ...[
                Row(
                  children: [
                    Icon(Icons.battery_std, size: 16, color: batteryColor),
                    SizedBox(width: 4),
                    Text(
                      '${batteryPercentage.toStringAsFixed(0)}%',
                      style: TextStyle(
                        color: batteryColor,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                  ],
                ),
                SizedBox(height: 4),
              ],
              
              // Robot state
              if (robotState != null) ...[
                Row(
                  children: [
                    Icon(
                      Icons.smart_toy,
                      size: 16,
                      color: _getRobotStateColor(robotState['status']),
                    ),
                    SizedBox(width: 4),
                    Text(
                      robotState['mode']?.toString().toUpperCase() ?? 'UNKNOWN',
                      style: TextStyle(
                        fontSize: 12,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                  ],
                ),
                SizedBox(height: 4),
              ],
              
              // Position info
              if (odometry != null)
                Text(
                  'X: ${odometry.position.x.toStringAsFixed(1)}, Y: ${odometry.position.y.toStringAsFixed(1)}',
                  style: TextStyle(
                    fontSize: 12,
                    color: Colors.grey[600],
                  ),
                ),
              
              SizedBox(height: 8),
              
              // Quick action buttons
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton(
                      onPressed: isOnline ? () => _navigateToControl(device) : null,
                      child: Text('Control', style: TextStyle(fontSize: 10)),
                      style: ElevatedButton.styleFrom(
                        padding: EdgeInsets.symmetric(vertical: 4),
                      ),
                    ),
                  ),
                  SizedBox(width: 4),
                  Expanded(
                    child: OutlinedButton(
                      onPressed: () => _navigateToMap(device),
                      child: Text('Map', style: TextStyle(fontSize: 10)),
                      style: OutlinedButton.styleFrom(
                        padding: EdgeInsets.symmetric(vertical: 4),
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

  Widget _buildActiveOrdersSection() {
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
              onPressed: () => Navigator.pushNamed(context, '/map'),
              child: Text('Manage Orders'),
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
                  Text('No active orders'),
                ],
              ),
            ),
          )
        else
          ListView.builder(
            shrinkWrap: true,
            physics: NeverScrollableScrollPhysics(),
            itemCount: activeOrders.length,
            itemBuilder: (context, index) {
              final order = activeOrders[index];
              return _buildOrderCard(order);
            },
          ),
      ],
    );
  }

  Widget _buildOrderCard(Map<String, dynamic> order) {
    final status = order['status'] ?? 'pending';
    final statusColor = status == 'active' ? Colors.blue : Colors.orange;
    final statusIcon = status == 'active' ? Icons.play_circle : Icons.pending;

    return Card(
      child: ListTile(
        leading: CircleAvatar(
          backgroundColor: statusColor,
          child: Icon(statusIcon, color: Colors.white),
        ),
        title: Text(order['name'] ?? 'Order'),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Device: ${order['deviceName'] ?? order['deviceId']}'),
            Text('Waypoints: ${order['waypoints']?.length ?? 0}'),
            Text('Status: ${status.toUpperCase()}'),
          ],
        ),
        trailing: status == 'pending'
            ? IconButton(
                icon: Icon(Icons.play_arrow, color: Colors.green),
                onPressed: () => _executeOrder(order),
              )
            : Icon(Icons.schedule, color: Colors.blue),
        onTap: () => _showOrderDetails(order),
      ),
    );
  }

  Widget _buildRecentAlertsSection() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Text(
              'Recent Alerts',
              style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
            ),
            Spacer(),
            TextButton(
              onPressed: _showAlertsDialog,
              child: Text('View All'),
            ),
          ],
        ),
        SizedBox(height: 12),
        if (_recentAlerts.isEmpty)
          Card(
            child: Padding(
              padding: EdgeInsets.all(24),
              child: Row(
                children: [
                  Icon(Icons.check_circle, color: Colors.green, size: 32),
                  SizedBox(width: 16),
                  Text('No recent alerts'),
                ],
              ),
            ),
          )
        else
          ListView.builder(
            shrinkWrap: true,
            physics: NeverScrollableScrollPhysics(),
            itemCount: _recentAlerts.take(3).length,
            itemBuilder: (context, index) {
              final alert = _recentAlerts[index];
              return _buildAlertCard(alert);
            },
          ),
      ],
    );
  }

  Widget _buildAlertCard(Map<String, dynamic> alert) {
    final type = alert['type'] ?? 'info';
    Color alertColor;
    IconData alertIcon;

    switch (type) {
      case 'low_battery':
        alertColor = Colors.orange;
        alertIcon = Icons.battery_alert;
        break;
      case 'error':
        alertColor = Colors.red;
        alertIcon = Icons.error;
        break;
      case 'warning':
        alertColor = Colors.orange;
        alertIcon = Icons.warning;
        break;
      default:
        alertColor = Colors.blue;
        alertIcon = Icons.info;
    }

    return Card(
      child: ListTile(
        leading: Icon(alertIcon, color: alertColor),
        title: Text(_getAlertTitle(alert)),
        subtitle: Text(_getAlertDescription(alert)),
        trailing: Text(
          _formatAlertTime(alert['timestamp']),
          style: TextStyle(fontSize: 12, color: Colors.grey[600]),
        ),
      ),
    );
  }

  String _getAlertTitle(Map<String, dynamic> alert) {
    switch (alert['type']) {
      case 'low_battery':
        return 'Low Battery Warning';
      case 'error':
        return 'System Error';
      case 'warning':
        return 'System Warning';
      default:
        return 'System Notification';
    }
  }

  String _getAlertDescription(Map<String, dynamic> alert) {
    switch (alert['type']) {
      case 'low_battery':
        return 'Device ${alert['deviceId']} battery at ${alert['percentage']}%';
      default:
        return alert['message'] ?? 'No description available';
    }
  }

  String _formatAlertTime(String? timestamp) {
    if (timestamp == null) return '';
    try {
      final dateTime = DateTime.parse(timestamp);
      final now = DateTime.now();
      final difference = now.difference(dateTime);
      
      if (difference.inMinutes < 1) {
        return 'Just now';
      } else if (difference.inHours < 1) {
        return '${difference.inMinutes}m ago';
      } else if (difference.inDays < 1) {
        return '${difference.inHours}h ago';
      } else {
        return '${difference.inDays}d ago';
      }
    } catch (e) {
      return '';
    }
  }

  String _formatUptime(dynamic uptime) {
    if (uptime == null) return 'Unknown';
    final seconds = uptime is num ? uptime.toInt() : 0;
    final hours = seconds ~/ 3600;
    final minutes = (seconds % 3600) ~/ 60;
    return '${hours}h ${minutes}m';
  }

  Color _getRobotStateColor(String? status) {
    switch (status) {
      case 'ok':
        return Colors.green;
      case 'warning':
        return Colors.orange;
      case 'error':
        return Colors.red;
      default:
        return Colors.grey;
    }
  }

  Future<void> _refreshDashboard() async {
    setState(() {
      _isLoading = true;
    });
    
    try {
      await _loadSystemHealth();
      await _loadDevices();
      await _loadOrdersForAllDevices();
    } catch (e) {
      _showErrorSnackBar('Failed to refresh dashboard: $e');
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  void _navigateToControl(Map<String, dynamic> device) {
    Navigator.pushNamed(
      context,
      '/control',
      arguments: {
        'deviceId': device['id'],
        'deviceName': device['name'],
      },
    );
  }

  void _navigateToMap(Map<String, dynamic> device) {
    Navigator.pushNamed(
      context,
      '/map',
      arguments: {
        'deviceId': device['id'],
      },
    );
  }

  void _handleDeviceAction(Map<String, dynamic> device, String action) {
    switch (action) {
      case 'control':
        _navigateToControl(device);
        break;
      case 'map':
        _navigateToMap(device);
        break;
      case 'orders':
        _navigateToMap(device);
        break;
    }
  }

  void _executeOrder(Map<String, dynamic> order) async {
    try {
      await _apiService.updateOrderStatus(
        orderId: order['id'],
        status: 'active',
      );
      
      _loadOrdersForAllDevices();
      
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Order execution started'),
          backgroundColor: Colors.green,
        ),
      );
    } catch (e) {
      _showErrorSnackBar('Failed to execute order: $e');
    }
  }

  void _showOrderDetails(Map<String, dynamic> order) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text(order['name'] ?? 'Order Details'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Device: ${order['deviceName'] ?? order['deviceId']}'),
            SizedBox(height: 8),
            Text('Status: ${order['status']?.toUpperCase() ?? 'UNKNOWN'}'),
            SizedBox(height: 8),
            Text('Waypoints: ${order['waypoints']?.length ?? 0}'),
            if (order['createdAt'] != null) ...[
              SizedBox(height: 8),
              Text('Created: ${DateTime.parse(order['createdAt']).toString()}'),
            ],
          ],
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
              child: Text('Execute'),
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
                    return _buildAlertCard(alert);
                  },
                ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Close'),
          ),
          TextButton(
            onPressed: () {
              setState(() {
                _recentAlerts.clear();
              });
              Navigator.of(context).pop();
            },
            child: Text('Clear All'),
          ),
        ],
      ),
    );
  }

  void _showErrorSnackBar(String message) {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(message),
          backgroundColor: Colors.red,
        ),
      );
    }
  }
}