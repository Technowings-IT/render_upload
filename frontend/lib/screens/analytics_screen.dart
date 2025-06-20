//analysis_screen.dart
import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../services/api_service.dart';
import '../services/web_socket_service.dart';

class AnalyticsScreen extends StatefulWidget {
  @override
  _AnalyticsScreenState createState() => _AnalyticsScreenState();
}

class _AnalyticsScreenState extends State<AnalyticsScreen> with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();
  
  late TabController _tabController;
  
  String _selectedTimeRange = '24h';
  String _selectedDeviceId = 'all';
  List<Map<String, dynamic>> _connectedDevices = [];
  
  // Analytics data
  Map<String, List<double>> _batteryHistory = {};
  Map<String, List<Map<String, dynamic>>> _orderHistory = {};
  Map<String, Map<String, dynamic>> _deviceStats = {};
  List<Map<String, dynamic>> _systemEvents = [];
  
  bool _isLoading = true;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 4, vsync: this);
    _loadDevices();
    _loadAnalyticsData();
  }

  @override
  void dispose() {
    _tabController.dispose();
    super.dispose();
  }

  void _loadDevices() async {
    try {
      final devices = await _apiService.getDevices();
      setState(() {
        _connectedDevices = devices;
        if (_connectedDevices.isNotEmpty && _selectedDeviceId == 'all') {
          // Keep 'all' selected
        }
      });
    } catch (e) {
      print('Error loading devices: $e');
    }
  }

  void _loadAnalyticsData() async {
    setState(() {
      _isLoading = true;
    });

    // Simulate loading analytics data
    await Future.delayed(Duration(seconds: 1));
    
    // Generate mock data for demonstration
    _generateMockData();
    
    setState(() {
      _isLoading = false;
    });
  }

  void _generateMockData() {
    final random = math.Random();
    
    // Generate battery history
    for (final device in _connectedDevices) {
      final deviceId = device['id'];
      final batteryData = <double>[];
      
      // Generate 24 hours of battery data (hourly)
      double currentBattery = 80.0 + random.nextDouble() * 20;
      for (int i = 0; i < 24; i++) {
        currentBattery += (random.nextDouble() - 0.6) * 3; // Gradual discharge with some variance
        currentBattery = currentBattery.clamp(10.0, 100.0);
        batteryData.add(currentBattery);
      }
      
      _batteryHistory[deviceId] = batteryData;
    }
    
    // Generate order history
    for (final device in _connectedDevices) {
      final deviceId = device['id'];
      final orders = <Map<String, dynamic>>[];
      
      // Generate random orders for the past week
      for (int i = 0; i < 15; i++) {
        final createdAt = DateTime.now().subtract(Duration(hours: random.nextInt(168))); // Past week
        final duration = Duration(minutes: 15 + random.nextInt(45)); // 15-60 minutes
        final completedAt = createdAt.add(duration);
        
        orders.add({
          'id': 'order_${deviceId}_$i',
          'name': 'Order ${i + 1}',
          'createdAt': createdAt.toIso8601String(),
          'completedAt': completedAt.toIso8601String(),
          'status': 'completed',
          'duration': duration.inMinutes,
          'waypoints': random.nextInt(5) + 2,
        });
      }
      
      _orderHistory[deviceId] = orders;
    }
    
    // Generate device statistics
    for (final device in _connectedDevices) {
      final deviceId = device['id'];
      
      _deviceStats[deviceId] = {
        'totalOrders': _orderHistory[deviceId]?.length ?? 0,
        'completedOrders': _orderHistory[deviceId]?.where((o) => o['status'] == 'completed').length ?? 0,
        'averageOrderTime': _calculateAverageOrderTime(deviceId),
        'totalUptime': Duration(hours: 180 + random.nextInt(24)).inHours, // ~7 days
        'totalDistance': (50.0 + random.nextDouble() * 100).toStringAsFixed(1), // km
        'currentBattery': _batteryHistory[deviceId]?.last ?? 0.0,
        'averageBattery': _batteryHistory[deviceId] != null && _batteryHistory[deviceId]!.isNotEmpty
            ? _batteryHistory[deviceId]!.fold(0.0, (a, b) => a + b) / _batteryHistory[deviceId]!.length
            : 0.0,
      };
    }
    
    // Generate system events
    _systemEvents = List.generate(20, (index) {
      final eventTypes = ['info', 'warning', 'error', 'success'];
      final messages = [
        'Device connected successfully',
        'Low battery warning',
        'Order completed',
        'Mapping session started',
        'Navigation goal reached',
        'System maintenance required',
        'Communication timeout',
        'Charging session completed',
      ];
      
      return {
        'timestamp': DateTime.now().subtract(Duration(hours: random.nextInt(72))).toIso8601String(),
        'type': eventTypes[random.nextInt(eventTypes.length)],
        'message': messages[random.nextInt(messages.length)],
        'deviceId': _connectedDevices.isNotEmpty ? _connectedDevices[random.nextInt(_connectedDevices.length)]['id'] : 'unknown',
      };
    })..sort((a, b) => DateTime.parse(b['timestamp']).compareTo(DateTime.parse(a['timestamp'])));
  }

  double _calculateAverageOrderTime(String deviceId) {
    final orders = _orderHistory[deviceId] ?? [];
    if (orders.isEmpty) return 0.0;
    
    final totalDuration = orders.fold<int>(0, (sum, order) => sum + ((order['duration'] ?? 0) as int));
    return totalDuration / orders.length;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Analytics & Reports'),
        actions: [
          _buildTimeRangeSelector(),
          SizedBox(width: 8),
          _buildDeviceSelector(),
          IconButton(
            icon: Icon(Icons.refresh),
            onPressed: _loadAnalyticsData,
          ),
        ],
        bottom: TabBar(
          controller: _tabController,
          tabs: [
            Tab(icon: Icon(Icons.dashboard), text: 'Overview'),
            Tab(icon: Icon(Icons.battery_std), text: 'Performance'),
            Tab(icon: Icon(Icons.list_alt), text: 'Orders'),
            Tab(icon: Icon(Icons.event), text: 'Events'),
          ],
        ),
      ),
      body: _isLoading
          ? Center(child: CircularProgressIndicator())
          : TabBarView(
              controller: _tabController,
              children: [
                _buildOverviewTab(),
                _buildPerformanceTab(),
                _buildOrdersTab(),
                _buildEventsTab(),
              ],
            ),
    );
  }

  Widget _buildTimeRangeSelector() {
    return Container(
      margin: EdgeInsets.symmetric(horizontal: 8),
      child: DropdownButton<String>(
        value: _selectedTimeRange,
        dropdownColor: Colors.grey[800],
        style: TextStyle(color: Colors.white),
        underline: Container(),
        items: [
          DropdownMenuItem(value: '24h', child: Text('Last 24h')),
          DropdownMenuItem(value: '7d', child: Text('Last 7 days')),
          DropdownMenuItem(value: '30d', child: Text('Last 30 days')),
        ],
        onChanged: (value) {
          if (value != null) {
            setState(() {
              _selectedTimeRange = value;
            });
            _loadAnalyticsData();
          }
        },
      ),
    );
  }

  Widget _buildDeviceSelector() {
    return Container(
      margin: EdgeInsets.symmetric(horizontal: 8),
      child: DropdownButton<String>(
        value: _selectedDeviceId,
        dropdownColor: Colors.grey[800],
        style: TextStyle(color: Colors.white),
        underline: Container(),
        items: [
          DropdownMenuItem(value: 'all', child: Text('All Devices')),
          ..._connectedDevices.map((device) => DropdownMenuItem(
            value: device['id'],
            child: Text(device['name'] ?? device['id']),
          )),
        ],
        onChanged: (value) {
          if (value != null) {
            setState(() {
              _selectedDeviceId = value;
            });
            _loadAnalyticsData();
          }
        },
      ),
    );
  }

  Widget _buildOverviewTab() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildOverviewMetrics(),
          SizedBox(height: 20),
          _buildFleetHealthCard(),
          SizedBox(height: 20),
          _buildQuickStatsGrid(),
        ],
      ),
    );
  }

  Widget _buildOverviewMetrics() {
    final totalDevices = _connectedDevices.length;
    final activeDevices = _connectedDevices.where((d) => d['status'] == 'connected').length;
    final totalOrders = _deviceStats.values.fold(0, (sum, stats) => sum + (stats['totalOrders'] as int));
    final averageUptime = _deviceStats.values.isNotEmpty 
        ? _deviceStats.values.fold(0, (sum, stats) => sum + (stats['totalUptime'] as int)) / _deviceStats.length
        : 0;

    return Row(
      children: [
        Expanded(child: _buildMetricCard('Fleet Size', totalDevices.toString(), Icons.devices, Colors.blue)),
        SizedBox(width: 12),
        Expanded(child: _buildMetricCard('Active Devices', activeDevices.toString(), Icons.power, Colors.green)),
        SizedBox(width: 12),
        Expanded(child: _buildMetricCard('Total Orders', totalOrders.toString(), Icons.assignment, Colors.orange)),
        SizedBox(width: 12),
        Expanded(child: _buildMetricCard('Avg Uptime', '${averageUptime.toStringAsFixed(0)}h', Icons.schedule, Colors.purple)),
      ],
    );
  }

  Widget _buildMetricCard(String title, String value, IconData icon, Color color) {
    return Card(
      elevation: 4,
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          children: [
            Icon(icon, size: 36, color: color),
            SizedBox(height: 12),
            Text(
              value,
              style: TextStyle(
                fontSize: 28,
                fontWeight: FontWeight.bold,
                color: color,
              ),
            ),
            SizedBox(height: 8),
            Text(
              title,
              style: TextStyle(
                fontSize: 14,
                color: Colors.grey[600],
              ),
              textAlign: TextAlign.center,
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildFleetHealthCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Fleet Health Overview',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            if (_connectedDevices.isEmpty)
              Center(child: Text('No devices available'))
            else
              ..._connectedDevices.map((device) {
                final deviceId = device['id'];
                final stats = _deviceStats[deviceId];
                final batteryLevel = stats?['currentBattery'] ?? 0.0;
                final isOnline = device['status'] == 'connected';
                
                return Padding(
                  padding: EdgeInsets.symmetric(vertical: 8),
                  child: Row(
                    children: [
                      Container(
                        width: 12,
                        height: 12,
                        decoration: BoxDecoration(
                          shape: BoxShape.circle,
                          color: isOnline ? Colors.green : Colors.red,
                        ),
                      ),
                      SizedBox(width: 12),
                      Expanded(
                        flex: 2,
                        child: Text(device['name'] ?? device['id']),
                      ),
                      Expanded(
                        flex: 3,
                        child: LinearProgressIndicator(
                          value: batteryLevel / 100.0,
                          backgroundColor: Colors.grey[300],
                          valueColor: AlwaysStoppedAnimation<Color>(
                            batteryLevel > 30 ? Colors.green : 
                            batteryLevel > 15 ? Colors.orange : Colors.red,
                          ),
                        ),
                      ),
                      SizedBox(width: 12),
                      Text('${batteryLevel.toStringAsFixed(0)}%'),
                    ],
                  ),
                );
              }),
          ],
        ),
      ),
    );
  }

  Widget _buildQuickStatsGrid() {
    return GridView.count(
      shrinkWrap: true,
      physics: NeverScrollableScrollPhysics(),
      crossAxisCount: 2,
      crossAxisSpacing: 12,
      mainAxisSpacing: 12,
      childAspectRatio: 1.5,
      children: [
        _buildStatCard('Total Distance', _getTotalDistance(), Icons.route, Colors.blue),
        _buildStatCard('Avg Order Time', _getAverageOrderTime(), Icons.timer, Colors.green),
        _buildStatCard('Success Rate', _getSuccessRate(), Icons.check_circle, Colors.orange),
        _buildStatCard('Active Orders', _getActiveOrders(), Icons.play_circle, Colors.purple),
      ],
    );
  }

  Widget _buildStatCard(String title, String value, IconData icon, Color color) {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, size: 32, color: color),
            SizedBox(height: 12),
            Text(
              value,
              style: TextStyle(
                fontSize: 20,
                fontWeight: FontWeight.bold,
                color: color,
              ),
            ),
            SizedBox(height: 8),
            Text(
              title,
              style: TextStyle(fontSize: 12),
              textAlign: TextAlign.center,
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildPerformanceTab() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(16),
      child: Column(
        children: [
          _buildBatteryChart(),
          SizedBox(height: 20),
          _buildPerformanceMetrics(),
        ],
      ),
    );
  }

  Widget _buildBatteryChart() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Battery Levels (Last 24h)',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            Container(
              height: 200,
              child: CustomPaint(
                size: Size.infinite,
                painter: BatteryChartPainter(_batteryHistory, _selectedDeviceId),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildPerformanceMetrics() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Performance Metrics',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            if (_selectedDeviceId == 'all')
              Text('Select a specific device to view detailed performance metrics')
            else if (_deviceStats[_selectedDeviceId] != null)
              _buildDevicePerformanceDetails(_deviceStats[_selectedDeviceId]!)
            else
              Text('No performance data available for this device'),
          ],
        ),
      ),
    );
  }

  Widget _buildDevicePerformanceDetails(Map<String, dynamic> stats) {
    return Column(
      children: [
        _buildPerformanceRow('Total Orders', stats['totalOrders'].toString()),
        _buildPerformanceRow('Completed Orders', stats['completedOrders'].toString()),
        _buildPerformanceRow('Average Order Time', '${stats['averageOrderTime'].toStringAsFixed(1)} min'),
        _buildPerformanceRow('Total Uptime', '${stats['totalUptime']} hours'),
        _buildPerformanceRow('Total Distance', '${stats['totalDistance']} km'),
        _buildPerformanceRow('Current Battery', '${stats['currentBattery'].toStringAsFixed(1)}%'),
        _buildPerformanceRow('Average Battery', '${stats['averageBattery'].toStringAsFixed(1)}%'),
      ],
    );
  }

  Widget _buildPerformanceRow(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 8),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label),
          Text(
            value,
            style: TextStyle(fontWeight: FontWeight.bold),
          ),
        ],
      ),
    );
  }

  Widget _buildOrdersTab() {
    final relevantOrders = _selectedDeviceId == 'all' 
        ? _orderHistory.values.expand((orders) => orders).toList()
        : _orderHistory[_selectedDeviceId] ?? [];

    // Sort by completion time
    relevantOrders.sort((a, b) => DateTime.parse(b['completedAt']).compareTo(DateTime.parse(a['completedAt'])));

    return SingleChildScrollView(
      padding: EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Order History',
            style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
          ),
          SizedBox(height: 16),
          if (relevantOrders.isEmpty)
            Card(
              child: Padding(
                padding: EdgeInsets.all(24),
                child: Center(child: Text('No order history available')),
              ),
            )
          else
            ListView.builder(
              shrinkWrap: true,
              physics: NeverScrollableScrollPhysics(),
              itemCount: relevantOrders.length,
              itemBuilder: (context, index) {
                final order = relevantOrders[index];
                return _buildOrderHistoryCard(order);
              },
            ),
        ],
      ),
    );
  }

  Widget _buildOrderHistoryCard(Map<String, dynamic> order) {
    final createdAt = DateTime.parse(order['createdAt']);
    final completedAt = DateTime.parse(order['completedAt']);
    final duration = order['duration'];
    
    return Card(
      child: ListTile(
        leading: CircleAvatar(
          backgroundColor: Colors.green,
          child: Icon(Icons.check, color: Colors.white),
        ),
        title: Text(order['name']),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Completed: ${_formatDateTime(completedAt)}'),
            Text('Duration: ${duration} minutes'),
            Text('Waypoints: ${order['waypoints']}'),
          ],
        ),
        trailing: Text(
          '${duration}m',
          style: TextStyle(
            fontWeight: FontWeight.bold,
            color: duration < 30 ? Colors.green : 
                   duration < 60 ? Colors.orange : Colors.red,
          ),
        ),
      ),
    );
  }

  Widget _buildEventsTab() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'System Events',
            style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
          ),
          SizedBox(height: 16),
          ListView.builder(
            shrinkWrap: true,
            physics: NeverScrollableScrollPhysics(),
            itemCount: _systemEvents.length,
            itemBuilder: (context, index) {
              final event = _systemEvents[index];
              return _buildEventCard(event);
            },
          ),
        ],
      ),
    );
  }

  Widget _buildEventCard(Map<String, dynamic> event) {
    final type = event['type'];
    Color color;
    IconData icon;

    switch (type) {
      case 'error':
        color = Colors.red;
        icon = Icons.error;
        break;
      case 'warning':
        color = Colors.orange;
        icon = Icons.warning;
        break;
      case 'success':
        color = Colors.green;
        icon = Icons.check_circle;
        break;
      default:
        color = Colors.blue;
        icon = Icons.info;
    }

    return Card(
      child: ListTile(
        leading: Icon(icon, color: color),
        title: Text(event['message']),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Device: ${event['deviceId']}'),
            Text(_formatDateTime(DateTime.parse(event['timestamp']))),
          ],
        ),
        trailing: Container(
          padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
          decoration: BoxDecoration(
            color: color.withOpacity(0.1),
            border: Border.all(color: color),
            borderRadius: BorderRadius.circular(12),
          ),
          child: Text(
            type.toUpperCase(),
            style: TextStyle(
              color: color,
              fontSize: 10,
              fontWeight: FontWeight.bold,
            ),
          ),
        ),
      ),
    );
  }

  String _getTotalDistance() {
    if (_selectedDeviceId == 'all') {
      final total = _deviceStats.values.fold(0.0, (sum, stats) => sum + double.parse(stats['totalDistance']));
      return '${total.toStringAsFixed(1)} km';
    } else {
      return '${_deviceStats[_selectedDeviceId]?['totalDistance'] ?? '0'} km';
    }
  }

  String _getAverageOrderTime() {
    if (_selectedDeviceId == 'all') {
      final times = _deviceStats.values.map((stats) => stats['averageOrderTime'] as double).toList();
      if (times.isEmpty) return '0 min';
      final average = times.fold(0.0, (sum, time) => sum + time) / times.length;
      return '${average.toStringAsFixed(1)} min';
    } else {
      return '${_deviceStats[_selectedDeviceId]?['averageOrderTime']?.toStringAsFixed(1) ?? '0'} min';
    }
  }

  String _getSuccessRate() {
    // Mock success rate calculation
    return '98.5%';
  }

  String _getActiveOrders() {
    // Mock active orders count
    return '3';
  }

  String _formatDateTime(DateTime dateTime) {
    return '${dateTime.day}/${dateTime.month}/${dateTime.year} ${dateTime.hour.toString().padLeft(2, '0')}:${dateTime.minute.toString().padLeft(2, '0')}';
  }
}

class BatteryChartPainter extends CustomPainter {
  final Map<String, List<double>> batteryHistory;
  final String selectedDeviceId;

  BatteryChartPainter(this.batteryHistory, this.selectedDeviceId);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    final textPainter = TextPainter(
      textDirection: TextDirection.ltr,
    );

    // Draw grid lines
    final gridPaint = Paint()
      ..color = Colors.grey[300]!
      ..strokeWidth = 1;

    // Horizontal grid lines (battery levels)
    for (int i = 0; i <= 5; i++) {
      final y = size.height * (1 - i / 5);
      canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);
      
      // Draw battery level labels
      textPainter.text = TextSpan(
        text: '${i * 20}%',
        style: TextStyle(color: Colors.grey[600], fontSize: 10),
      );
      textPainter.layout();
      textPainter.paint(canvas, Offset(-30, y - textPainter.height / 2));
    }

    // Vertical grid lines (time)
    for (int i = 0; i <= 6; i++) {
      final x = size.width * (i / 6);
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
    }

    // Draw battery data
    if (selectedDeviceId == 'all') {
      final colors = [Colors.blue, Colors.green, Colors.orange, Colors.purple, Colors.red];
      int colorIndex = 0;
      
      batteryHistory.forEach((deviceId, data) {
        paint.color = colors[colorIndex % colors.length];
        _drawBatteryLine(canvas, size, data, paint);
        colorIndex++;
      });
    } else if (batteryHistory.containsKey(selectedDeviceId)) {
      paint.color = Colors.blue;
      _drawBatteryLine(canvas, size, batteryHistory[selectedDeviceId]!, paint);
    }
  }

  void _drawBatteryLine(Canvas canvas, Size size, List<double> data, Paint paint) {
    if (data.length < 2) return;

    final path = Path();
    for (int i = 0; i < data.length; i++) {
      final x = size.width * (i / (data.length - 1));
      final y = size.height * (1 - data[i] / 100);
      
      if (i == 0) {
        path.moveTo(x, y);
      } else {
        path.lineTo(x, y);
      }
    }
    
    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true;
  }
}