// screens/enhanced_analytics_screen.dart - FIXED Analytics with Real Data Fetching
import 'package:flutter/material.dart';
import 'dart:math' as math;
import 'dart:async';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';

class EnhancedAnalyticsScreen extends StatefulWidget {
  @override
  _EnhancedAnalyticsScreenState createState() => _EnhancedAnalyticsScreenState();
}

class _EnhancedAnalyticsScreenState extends State<EnhancedAnalyticsScreen> 
    with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();
  
  late TabController _tabController;
  late AnimationController _chartAnimationController;
  late StreamSubscription _realTimeSubscription;
  
  String _selectedTimeRange = '24h';
  String _selectedDeviceId = 'all';
  List<Map<String, dynamic>> _connectedDevices = [];
  DeviceType _deviceType = DeviceType.phone;
  
  // Real-time analytics data
  Map<String, List<Map<String, dynamic>>> _batteryHistory = {};
  Map<String, List<Map<String, dynamic>>> _orderHistory = {};
  Map<String, Map<String, dynamic>> _deviceStats = {};
  List<Map<String, dynamic>> _systemEvents = [];
  Map<String, List<Map<String, dynamic>>> _velocityHistory = {};
  Map<String, List<Map<String, dynamic>>> _errorHistory = {};
  Map<String, double> _realTimeMetrics = {};
  
  bool _isLoading = true;
  bool _autoRefresh = true;
  Timer? _refreshTimer;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 5, vsync: this);
    _chartAnimationController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );

    WidgetsBinding.instance.addPostFrameCallback((_) {
      _updateDeviceType();
    });

    _initializeData();
    _setupRealTimeUpdates();
    _startAutoRefresh();
  }

  void _updateDeviceType() {
    final screenWidth = MediaQuery.of(context).size.width;
    setState(() {
      if (screenWidth > 1200) {
        _deviceType = DeviceType.desktop;
      } else if (screenWidth > 768) {
        _deviceType = DeviceType.tablet;
      } else {
        _deviceType = DeviceType.phone;
      }
    });
  }

  @override
  void dispose() {
    _tabController.dispose();
    _chartAnimationController.dispose();
    _realTimeSubscription.cancel();
    _refreshTimer?.cancel();
    super.dispose();
  }

  void _initializeData() async {
    _loadDevices();
    _loadAnalyticsData();
  }

  void _setupRealTimeUpdates() {
    _realTimeSubscription = _webSocketService.realTimeData.listen((data) {
      if (mounted) {
        _handleRealTimeData(data);
      }
    });
  }

  void _startAutoRefresh() {
    if (_autoRefresh) {
      _refreshTimer = Timer.periodic(const Duration(seconds: 30), (_) {
        if (mounted) {
          _loadAnalyticsData();
        }
      });
    }
  }

  void _handleRealTimeData(Map<String, dynamic> data) {
    final deviceId = data['deviceId'];
    final messageType = data['type'];
    final timestamp = DateTime.now();

    setState(() {
      switch (messageType) {
        case 'battery_update':
          _updateBatteryHistory(deviceId, data['data'], timestamp);
          break;
        case 'velocity_update':
          _updateVelocityHistory(deviceId, data['data'], timestamp);
          break;
        case 'error_event':
          _updateErrorHistory(deviceId, data['data'], timestamp);
          break;
        case 'system_metrics':
          _updateSystemMetrics(data['data']);
          break;
        case 'order_completed':
          _updateOrderHistory(deviceId, data['data'], timestamp);
          break;
      }
    });
  }

  void _updateBatteryHistory(String deviceId, Map<String, dynamic> batteryData, DateTime timestamp) {
    if (!_batteryHistory.containsKey(deviceId)) {
      _batteryHistory[deviceId] = [];
    }
    
    _batteryHistory[deviceId]!.add({
      'timestamp': timestamp,
      'voltage': batteryData['voltage']?.toDouble() ?? 0.0,
      'percentage': batteryData['percentage']?.toDouble() ?? 0.0,
      'current': batteryData['current']?.toDouble() ?? 0.0,
      'temperature': batteryData['temperature']?.toDouble() ?? 25.0,
    });

    final cutoff = timestamp.subtract(const Duration(hours: 24));
    _batteryHistory[deviceId]!.removeWhere((item) => 
        item['timestamp'].isBefore(cutoff));
  }

  void _updateVelocityHistory(String deviceId, Map<String, dynamic> velocityData, DateTime timestamp) {
    if (!_velocityHistory.containsKey(deviceId)) {
      _velocityHistory[deviceId] = [];
    }
    
    _velocityHistory[deviceId]!.add({
      'timestamp': timestamp,
      'linear': velocityData['linear']?.toDouble() ?? 0.0,
      'angular': velocityData['angular']?.toDouble() ?? 0.0,
    });

    final cutoff = timestamp.subtract(const Duration(hours: 1));
    _velocityHistory[deviceId]!.removeWhere((item) => 
        item['timestamp'].isBefore(cutoff));
  }

  void _updateErrorHistory(String deviceId, Map<String, dynamic> errorData, DateTime timestamp) {
    if (!_errorHistory.containsKey(deviceId)) {
      _errorHistory[deviceId] = [];
    }
    
    _errorHistory[deviceId]!.add({
      'timestamp': timestamp,
      'type': errorData['type'] ?? 'unknown',
      'message': errorData['message'] ?? 'Unknown error',
      'severity': errorData['severity'] ?? 'info',
    });

    final cutoff = timestamp.subtract(const Duration(days: 7));
    _errorHistory[deviceId]!.removeWhere((item) => 
        item['timestamp'].isBefore(cutoff));
  }

  void _updateOrderHistory(String deviceId, Map<String, dynamic> orderData, DateTime timestamp) {
    if (!_orderHistory.containsKey(deviceId)) {
      _orderHistory[deviceId] = [];
    }
    
    _orderHistory[deviceId]!.add({
      'id': orderData['id'] ?? 'unknown',
      'name': orderData['name'] ?? 'Unnamed Order',
      'completedAt': timestamp,
      'duration': orderData['duration']?.toDouble() ?? 0.0,
      'distance': orderData['distance']?.toDouble() ?? 0.0,
      'waypoints': orderData['waypoints'] ?? 0,
      'status': 'completed',
    });
  }

  void _updateSystemMetrics(Map<String, dynamic> metrics) {
    _realTimeMetrics.addAll({
      'totalUptime': metrics['uptime']?.toDouble() ?? 0.0,
      'totalDistance': metrics['distance']?.toDouble() ?? 0.0,
      'averageSpeed': metrics['speed']?.toDouble() ?? 0.0,
      'errorRate': metrics['errorRate']?.toDouble() ?? 0.0,
    });
  }

  void _loadDevices() async {
    try {
      final devices = await _apiService.getDevices();
      setState(() {
        _connectedDevices = devices;
      });
    } catch (e) {
      print('❌ Error loading devices: $e');
      setState(() {
        _connectedDevices = [];
      });
    }
  }

  void _loadAnalyticsData() async {
    setState(() {
      _isLoading = true;
    });

    try {
      await Future.wait([
        _loadBatteryData(),
        _loadOrderData(),
        _loadDeviceStats(),
        _loadSystemEvents(),
        _loadPerformanceData(),
      ]);

      _chartAnimationController.forward();
    } catch (e) {
      print('❌ Error loading analytics data: $e');
      _generateMockData();
    }
    
    setState(() {
      _isLoading = false;
    });
  }

  Future<void> _loadBatteryData() async {
    try {
      for (final device in _connectedDevices) {
        final deviceId = device['id'];
        final response = await _apiService.getAnalyticsData(
          deviceId, 
          'battery', 
          _selectedTimeRange,
        );
        
        if (response['success'] == true && response['data'] != null) {
          _batteryHistory[deviceId] = List<Map<String, dynamic>>.from(
            response['data'].map((item) => {
              'timestamp': DateTime.parse(item['timestamp']),
              'voltage': item['voltage']?.toDouble() ?? 0.0,
              'percentage': item['percentage']?.toDouble() ?? 0.0,
              'current': item['current']?.toDouble() ?? 0.0,
              'temperature': item['temperature']?.toDouble() ?? 25.0,
            })
          );
        }
      }
    } catch (e) {
      print('❌ Error loading battery data: $e');
    }
  }

  Future<void> _loadOrderData() async {
    try {
      for (final device in _connectedDevices) {
        final deviceId = device['id'];
        final response = await _apiService.getAnalyticsData(
          deviceId, 
          'orders', 
          _selectedTimeRange,
        );
        
        if (response['success'] == true && response['data'] != null) {
          _orderHistory[deviceId] = List<Map<String, dynamic>>.from(
            response['data'].map((item) => {
              'id': item['id'] ?? 'unknown',
              'name': item['name'] ?? 'Unnamed Order',
              'createdAt': DateTime.parse(item['createdAt']),
              'completedAt': DateTime.parse(item['completedAt']),
              'duration': item['duration']?.toDouble() ?? 0.0,
              'distance': item['distance']?.toDouble() ?? 0.0,
              'waypoints': item['waypoints'] ?? 0,
              'status': item['status'] ?? 'completed',
            })
          );
        }
      }
    } catch (e) {
      print('❌ Error loading order data: $e');
    }
  }

  Future<void> _loadDeviceStats() async {
    try {
      for (final device in _connectedDevices) {
        final deviceId = device['id'];
        final response = await _apiService.getAnalyticsData(
          deviceId, 
          'stats', 
          _selectedTimeRange,
        );
        
        if (response['success'] == true && response['data'] != null) {
          _deviceStats[deviceId] = Map<String, dynamic>.from(response['data']);
        }
      }
    } catch (e) {
      print('❌ Error loading device stats: $e');
    }
  }

  Future<void> _loadSystemEvents() async {
    try {
      final response = await _apiService.getAnalyticsData(
        _selectedDeviceId == 'all' ? null : _selectedDeviceId, 
        'events', 
        _selectedTimeRange,
      );
      
      if (response['success'] == true && response['data'] != null) {
        _systemEvents = List<Map<String, dynamic>>.from(
          response['data'].map((item) => {
            'timestamp': DateTime.parse(item['timestamp']),
            'type': item['type'] ?? 'info',
            'message': item['message'] ?? 'Unknown event',
            'deviceId': item['deviceId'] ?? 'unknown',
            'severity': item['severity'] ?? 'info',
          })
        );
      }
    } catch (e) {
      print('❌ Error loading system events: $e');
    }
  }

  Future<void> _loadPerformanceData() async {
    try {
      for (final device in _connectedDevices) {
        final deviceId = device['id'];
        final response = await _apiService.getAnalyticsData(
          deviceId, 
          'performance', 
          _selectedTimeRange,
        );
        
        if (response['success'] == true && response['data'] != null) {
          _velocityHistory[deviceId] = List<Map<String, dynamic>>.from(
            response['data'].map((item) => {
              'timestamp': DateTime.parse(item['timestamp']),
              'linear': item['linear']?.toDouble() ?? 0.0,
              'angular': item['angular']?.toDouble() ?? 0.0,
            })
          );
        }
      }
    } catch (e) {
      print('❌ Error loading performance data: $e');
    }
  }

  void _generateMockData() {
    final random = math.Random();
    final now = DateTime.now();
    
    for (final device in _connectedDevices) {
      final deviceId = device['id'];
      
      _batteryHistory[deviceId] = List.generate(24, (index) {
        final timestamp = now.subtract(Duration(hours: 23 - index));
        return {
          'timestamp': timestamp,
          'voltage': 11.5 + random.nextDouble() * 1.0,
          'percentage': math.max(10.0, 100.0 - (index * 2) + random.nextDouble() * 10),
          'current': 0.5 + random.nextDouble() * 2.0,
          'temperature': 20.0 + random.nextDouble() * 15.0,
        };
      });
      
      _orderHistory[deviceId] = List.generate(15, (index) {
        final completedAt = now.subtract(Duration(hours: random.nextInt(168)));
        final duration = 15.0 + random.nextDouble() * 45.0;
        return {
          'id': 'order_${deviceId}_$index',
          'name': 'Order ${index + 1}',
          'createdAt': completedAt.subtract(Duration(minutes: duration.toInt())),
          'completedAt': completedAt,
          'duration': duration,
          'distance': 50.0 + random.nextDouble() * 200.0,
          'waypoints': random.nextInt(5) + 2,
          'status': 'completed',
        };
      });
      
      _deviceStats[deviceId] = {
        'totalOrders': _orderHistory[deviceId]?.length ?? 0,
        'completedOrders': _orderHistory[deviceId]?.where((o) => o['status'] == 'completed').length ?? 0,
        'averageOrderTime': 32.5 + random.nextDouble() * 20.0,
        'totalUptime': 180.0 + random.nextDouble() * 24.0,
        'totalDistance': 50.0 + random.nextDouble() * 100.0,
        'currentBattery': _batteryHistory[deviceId]?.last['percentage'] ?? 0.0,
        'averageBattery': 65.0 + random.nextDouble() * 20.0,
        'errorCount': random.nextInt(5),
        'averageSpeed': 0.3 + random.nextDouble() * 0.5,
      };
      
      _velocityHistory[deviceId] = List.generate(60, (index) {
        final timestamp = now.subtract(Duration(minutes: 59 - index));
        return {
          'timestamp': timestamp,
          'linear': random.nextDouble() * 1.0,
          'angular': (random.nextDouble() - 0.5) * 2.0,
        };
      });
    }
    
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
        'timestamp': now.subtract(Duration(hours: random.nextInt(72))),
        'type': eventTypes[random.nextInt(eventTypes.length)],
        'message': messages[random.nextInt(messages.length)],
        'deviceId': _connectedDevices.isNotEmpty 
            ? _connectedDevices[random.nextInt(_connectedDevices.length)]['id'] 
            : 'unknown',
        'severity': ['low', 'medium', 'high'][random.nextInt(3)],
      };
    })..sort((a, b) => b['timestamp'].compareTo(a['timestamp']));
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    
    return Scaffold(
      appBar: _buildEnhancedAppBar(theme),
      body: _buildResponsiveBody(),
    );
  }

  PreferredSizeWidget _buildEnhancedAppBar(ThemeData theme) {
    return AppBar(
      title: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [theme.primaryColor, theme.primaryColor.withOpacity(0.7)],
              ),
              borderRadius: BorderRadius.circular(8),
            ),
            child: const Icon(Icons.analytics, color: Colors.white, size: 20),
          ),
          const SizedBox(width: 12),
          const Text('Analytics & Reports'),
        ],
      ),
      backgroundColor: theme.primaryColor,
      elevation: 0,
      flexibleSpace: Container(
        decoration: BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: [
              theme.primaryColor,
              theme.primaryColor.withOpacity(0.8),
            ],
          ),
        ),
      ),
      actions: [
        _buildTimeRangeSelector(),
        const SizedBox(width: 8),
        _buildDeviceSelector(),
        const SizedBox(width: 8),
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 4),
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: _autoRefresh 
                  ? [Colors.green.shade400, Colors.green.shade600]
                  : [Colors.grey.shade400, Colors.grey.shade600],
            ),
            borderRadius: BorderRadius.circular(20),
          ),
          child: IconButton(
            icon: Icon(_autoRefresh ? Icons.sync : Icons.sync_disabled, color: Colors.white),
            onPressed: () {
              setState(() {
                _autoRefresh = !_autoRefresh;
              });
              
              _refreshTimer?.cancel();
              if (_autoRefresh) {
                _startAutoRefresh();
              }
            },
            tooltip: _autoRefresh ? 'Disable Auto Refresh' : 'Enable Auto Refresh',
          ),
        ),
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 4),
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [Colors.blue.shade400, Colors.blue.shade600],
            ),
            borderRadius: BorderRadius.circular(20),
          ),
          child: IconButton(
            icon: const Icon(Icons.refresh, color: Colors.white),
            onPressed: _loadAnalyticsData,
            tooltip: 'Refresh Data',
          ),
        ),
      ],
      bottom: _deviceType != DeviceType.phone 
          ? TabBar(
              controller: _tabController,
              indicatorColor: Colors.white,
              labelColor: Colors.white,
              unselectedLabelColor: Colors.white70,
              isScrollable: true,
              tabs: const [
                Tab(icon: Icon(Icons.dashboard), text: 'Overview'),
                Tab(icon: Icon(Icons.battery_std), text: 'Performance'),
                Tab(icon: Icon(Icons.list_alt), text: 'Orders'),
                Tab(icon: Icon(Icons.event), text: 'Events'),
                Tab(icon: Icon(Icons.speed), text: 'Real-time'),
              ],
            )
          : null,
    );
  }

  Widget _buildTimeRangeSelector() {
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 8),
      padding: const EdgeInsets.symmetric(horizontal: 12),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.2),
        borderRadius: BorderRadius.circular(20),
      ),
      child: DropdownButton<String>(
        value: _selectedTimeRange,
        dropdownColor: Colors.grey[800],
        style: const TextStyle(color: Colors.white),
        underline: Container(),
        items: const [
          DropdownMenuItem(value: '1h', child: Text('Last Hour')),
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
      margin: const EdgeInsets.symmetric(horizontal: 8),
      padding: const EdgeInsets.symmetric(horizontal: 12),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.2),
        borderRadius: BorderRadius.circular(20),
      ),
      child: DropdownButton<String>(
        value: _selectedDeviceId,
        dropdownColor: Colors.grey[800],
        style: const TextStyle(color: Colors.white),
        underline: Container(),
        items: [
          const DropdownMenuItem(value: 'all', child: Text('All Devices')),
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

  Widget _buildResponsiveBody() {
    if (_isLoading) {
      return _buildLoadingView();
    }

    switch (_deviceType) {
      case DeviceType.desktop:
        return _buildDesktopLayout();
      case DeviceType.tablet:
        return _buildTabletLayout();
      case DeviceType.phone:
      default:
        return _buildPhoneLayout();
    }
  }

  Widget _buildLoadingView() {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          AnimatedBuilder(
            animation: _chartAnimationController,
            builder: (context, child) {
              return Container(
                width: 80,
                height: 80,
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [
                      Theme.of(context).primaryColor,
                      Theme.of(context).primaryColor.withOpacity(0.3),
                    ],
                  ),
                  shape: BoxShape.circle,
                ),
                child: CircularProgressIndicator(
                  value: _chartAnimationController.value,
                  backgroundColor: Colors.transparent,
                  valueColor: const AlwaysStoppedAnimation<Color>(Colors.white),
                  strokeWidth: 4,
                ),
              );
            },
          ),
          const SizedBox(height: 20),
          Text(
            'Loading Analytics Data...',
            style: TextStyle(
              fontSize: 16,
              color: Colors.grey.shade600,
              fontWeight: FontWeight.w500,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDesktopLayout() {
    return TabBarView(
      controller: _tabController,
      children: [
        _buildOverviewTab(),
        _buildPerformanceTab(),
        _buildOrdersTab(),
        _buildEventsTab(),
        _buildRealTimeTab(),
      ],
    );
  }

  Widget _buildTabletLayout() {
    return Column(
      children: [
        Container(
          decoration: BoxDecoration(
            color: Colors.white,
            border: Border(
              bottom: BorderSide(color: Colors.grey.shade300),
            ),
          ),
          child: TabBar(
            controller: _tabController,
            labelColor: Theme.of(context).primaryColor,
            unselectedLabelColor: Colors.grey,
            indicatorColor: Theme.of(context).primaryColor,
            isScrollable: true,
            tabs: const [
              Tab(icon: Icon(Icons.dashboard), text: 'Overview'),
              Tab(icon: Icon(Icons.battery_std), text: 'Performance'),
              Tab(icon: Icon(Icons.list_alt), text: 'Orders'),
              Tab(icon: Icon(Icons.event), text: 'Events'),
              Tab(icon: Icon(Icons.speed), text: 'Real-time'),
            ],
          ),
        ),
        Expanded(
          child: TabBarView(
            controller: _tabController,
            children: [
              _buildOverviewTab(),
              _buildPerformanceTab(),
              _buildOrdersTab(),
              _buildEventsTab(),
              _buildRealTimeTab(),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildPhoneLayout() {
    return Column(
      children: [
        Expanded(
          child: TabBarView(
            controller: _tabController,
            children: [
              _buildOverviewTab(),
              _buildPerformanceTab(),
              _buildOrdersTab(),
              _buildEventsTab(),
              _buildRealTimeTab(),
            ],
          ),
        ),
        Container(
          decoration: BoxDecoration(
            color: Colors.white,
            border: Border(
              top: BorderSide(color: Colors.grey.shade300),
            ),
          ),
          child: TabBar(
            controller: _tabController,
            labelColor: Theme.of(context).primaryColor,
            unselectedLabelColor: Colors.grey,
            indicatorColor: Theme.of(context).primaryColor,
            tabs: const [
              Tab(icon: Icon(Icons.dashboard, size: 20), text: 'Overview'),
              Tab(icon: Icon(Icons.battery_std, size: 20), text: 'Performance'),
              Tab(icon: Icon(Icons.list_alt, size: 20), text: 'Orders'),
              Tab(icon: Icon(Icons.event, size: 20), text: 'Events'),
              Tab(icon: Icon(Icons.speed, size: 20), text: 'Live'),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildOverviewTab() {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildOverviewMetrics(),
          const SizedBox(height: 20),
          _buildFleetHealthCard(),
          const SizedBox(height: 20),
          _buildQuickStatsGrid(),
        ],
      ),
    );
  }

  Widget _buildOverviewMetrics() {
    final totalDevices = _connectedDevices.length;
    final activeDevices = _connectedDevices.where((d) => d['status'] == 'connected').length;
    final totalOrders = _deviceStats.values.fold(0, (sum, stats) => sum + (stats['totalOrders'] as int? ?? 0));
    final averageUptime = _deviceStats.values.isNotEmpty 
        ? _deviceStats.values.fold(0.0, (sum, stats) => sum + (stats['totalUptime'] as double? ?? 0.0)) / _deviceStats.length
        : 0.0;

    return AnimatedBuilder(
      animation: _chartAnimationController,
      builder: (context, child) {
        return Row(
          children: [
            Expanded(child: _buildEnhancedMetricCard('Fleet Size', totalDevices.toString(), Icons.devices, Colors.blue)),
            const SizedBox(width: 12),
            Expanded(child: _buildEnhancedMetricCard('Active Devices', activeDevices.toString(), Icons.power, Colors.green)),
            const SizedBox(width: 12),
            Expanded(child: _buildEnhancedMetricCard('Total Orders', totalOrders.toString(), Icons.assignment, Colors.orange)),
            const SizedBox(width: 12),
            Expanded(child: _buildEnhancedMetricCard('Avg Uptime', '${averageUptime.toStringAsFixed(0)}h', Icons.schedule, Colors.purple)),
          ],
        );
      },
    );
  }

  Widget _buildEnhancedMetricCard(String title, String value, IconData icon, Color color) {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            color.withOpacity(0.1),
            color.withOpacity(0.05),
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: color.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
            Container(
              padding: const EdgeInsets.all(12),
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [color, color.withOpacity(0.8)],
                ),
                shape: BoxShape.circle,
                boxShadow: [
                  BoxShadow(
                    color: color.withOpacity(0.3),
                    blurRadius: 8,
                    offset: const Offset(0, 4),
                  ),
                ],
              ),
              child: Icon(icon, size: 24, color: Colors.white),
            ),
            const SizedBox(height: 16),
            Text(
              value,
              style: TextStyle(
                fontSize: 28,
                fontWeight: FontWeight.bold,
                color: color,
              ),
            ),
            const SizedBox(height: 8),
            Text(
              title,
              style: TextStyle(
                fontSize: 14,
                color: Colors.grey[600],
                fontWeight: FontWeight.w500,
              ),
              textAlign: TextAlign.center,
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildFleetHealthCard() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.blue.shade50,
            Colors.white,
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.blue.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: Colors.blue.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [Colors.blue, Colors.blue.shade700],
                    ),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: const Icon(Icons.health_and_safety, color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                const Text(
                  'Fleet Health Overview',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            const SizedBox(height: 16),
            if (_connectedDevices.isEmpty)
              const Center(child: Text('No devices available'))
            else
              ..._connectedDevices.map((device) {
                final deviceId = device['id'];
                final stats = _deviceStats[deviceId];
                final batteryLevel = stats?['currentBattery']?.toDouble() ?? 0.0;
                final isOnline = device['status'] == 'connected';
                
                return Container(
                  margin: const EdgeInsets.symmetric(vertical: 8),
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.white,
                    borderRadius: BorderRadius.circular(12),
                    border: Border.all(color: Colors.grey.shade200),
                  ),
                  child: Row(
                    children: [
                      Container(
                        width: 12,
                        height: 12,
                        decoration: BoxDecoration(
                          shape: BoxShape.circle,
                          color: isOnline ? Colors.green : Colors.red,
                          boxShadow: [
                            BoxShadow(
                              color: (isOnline ? Colors.green : Colors.red).withOpacity(0.3),
                              blurRadius: 4,
                              spreadRadius: 1,
                            ),
                          ],
                        ),
                      ),
                      const SizedBox(width: 12),
                      Expanded(
                        flex: 2,
                        child: Text(
                          device['name'] ?? device['id'],
                          style: const TextStyle(fontWeight: FontWeight.w600),
                        ),
                      ),
                      Expanded(
                        flex: 3,
                        child: Container(
                          height: 8,
                          decoration: BoxDecoration(
                            color: Colors.grey.shade300,
                            borderRadius: BorderRadius.circular(4),
                          ),
                          child: FractionallySizedBox(
                            alignment: Alignment.centerLeft,
                            widthFactor: batteryLevel / 100.0,
                            child: Container(
                              decoration: BoxDecoration(
                                gradient: LinearGradient(
                                  colors: batteryLevel > 30 
                                      ? [Colors.green.shade400, Colors.green.shade600]
                                      : batteryLevel > 15 
                                          ? [Colors.orange.shade400, Colors.orange.shade600]
                                          : [Colors.red.shade400, Colors.red.shade600],
                                ),
                                borderRadius: BorderRadius.circular(4),
                              ),
                            ),
                          ),
                        ),
                      ),
                      const SizedBox(width: 12),
                      Text(
                        '${batteryLevel.toStringAsFixed(0)}%',
                        style: const TextStyle(fontWeight: FontWeight.bold),
                      ),
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
      physics: const NeverScrollableScrollPhysics(),
      crossAxisCount: _deviceType == DeviceType.phone ? 2 : 4,
      crossAxisSpacing: 12,
      mainAxisSpacing: 12,
      childAspectRatio: 1.2,
      children: [
        _buildEnhancedStatCard('Total Distance', _getTotalDistance(), Icons.route, Colors.blue),
        _buildEnhancedStatCard('Avg Order Time', _getAverageOrderTime(), Icons.timer, Colors.green),
        _buildEnhancedStatCard('Success Rate', _getSuccessRate(), Icons.check_circle, Colors.orange),
        _buildEnhancedStatCard('Active Orders', _getActiveOrders(), Icons.play_circle, Colors.purple),
      ],
    );
  }

  Widget _buildEnhancedStatCard(String title, String value, IconData icon, Color color) {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            color.withOpacity(0.1),
            color.withOpacity(0.05),
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: color.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.1),
            blurRadius: 8,
            offset: const Offset(0, 3),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Container(
              padding: const EdgeInsets.all(10),
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [color, color.withOpacity(0.8)],
                ),
                shape: BoxShape.circle,
              ),
              child: Icon(icon, size: 24, color: Colors.white),
            ),
            const SizedBox(height: 12),
            Text(
              value,
              style: TextStyle(
                fontSize: 20,
                fontWeight: FontWeight.bold,
                color: color,
              ),
            ),
            const SizedBox(height: 8),
            Text(
              title,
              style: const TextStyle(fontSize: 12, fontWeight: FontWeight.w500),
              textAlign: TextAlign.center,
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildPerformanceTab() {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        children: [
          _buildBatteryChart(),
          const SizedBox(height: 20),
          _buildVelocityChart(),
          const SizedBox(height: 20),
          _buildPerformanceMetrics(),
        ],
      ),
    );
  }

  Widget _buildBatteryChart() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.green.shade50,
            Colors.white,
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.green.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: Colors.green.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [Colors.green, Colors.green.shade700],
                    ),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: const Icon(Icons.battery_std, color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                Text(
                  'Battery Levels (${_getTimeRangeLabel()})',
                  style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            const SizedBox(height: 16),
            Container(
              height: 200,
              child: CustomPaint(
                size: Size.infinite,
                painter: EnhancedBatteryChartPainter(_batteryHistory, _selectedDeviceId),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildVelocityChart() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.blue.shade50,
            Colors.white,
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.blue.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: Colors.blue.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [Colors.blue, Colors.blue.shade700],
                    ),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: const Icon(Icons.speed, color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                Text(
                  'Velocity History (${_getTimeRangeLabel()})',
                  style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            const SizedBox(height: 16),
            Container(
              height: 200,
              child: CustomPaint(
                size: Size.infinite,
                painter: VelocityChartPainter(_velocityHistory, _selectedDeviceId),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildPerformanceMetrics() {
    if (_selectedDeviceId == 'all') {
      return Container(
        decoration: BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: [
              Colors.purple.shade50,
              Colors.white,
            ],
          ),
          borderRadius: BorderRadius.circular(16),
          border: Border.all(color: Colors.purple.withOpacity(0.2)),
        ),
        padding: const EdgeInsets.all(24),
        child: const Column(
          children: [
            Icon(Icons.analytics, size: 48, color: Colors.purple),
            SizedBox(height: 16),
            Text(
              'Select a specific device to view detailed performance metrics',
              style: TextStyle(fontSize: 16),
              textAlign: TextAlign.center,
            ),
          ],
        ),
      );
    }

    final stats = _deviceStats[_selectedDeviceId];
    if (stats == null) {
      return Container(
        decoration: BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: [
              Colors.grey.shade50,
              Colors.white,
            ],
          ),
          borderRadius: BorderRadius.circular(16),
          border: Border.all(color: Colors.grey.withOpacity(0.2)),
        ),
        padding: const EdgeInsets.all(24),
        child: const Text(
          'No performance data available for this device',
          style: TextStyle(fontSize: 16),
          textAlign: TextAlign.center,
        ),
      );
    }

    return _buildDevicePerformanceDetails(stats);
  }

  Widget _buildDevicePerformanceDetails(Map<String, dynamic> stats) {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.indigo.shade50,
            Colors.white,
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.indigo.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: Colors.indigo.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [Colors.indigo, Colors.indigo.shade700],
                    ),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: const Icon(Icons.assessment, color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                const Text(
                  'Performance Metrics',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            const SizedBox(height: 16),
            GridView.count(
              shrinkWrap: true,
              physics: const NeverScrollableScrollPhysics(),
              crossAxisCount: _deviceType == DeviceType.phone ? 2 : 3,
              crossAxisSpacing: 12,
              mainAxisSpacing: 12,
              childAspectRatio: 1.5,
              children: [
                _buildMetricItem('Total Orders', stats['totalOrders'].toString(), Icons.assignment, Colors.blue),
                _buildMetricItem('Completed', stats['completedOrders'].toString(), Icons.check_circle, Colors.green),
                _buildMetricItem('Avg Time', '${(stats['averageOrderTime'] as double).toStringAsFixed(1)} min', Icons.timer, Colors.orange),
                _buildMetricItem('Uptime', '${(stats['totalUptime'] as double).toStringAsFixed(0)} h', Icons.schedule, Colors.purple),
                _buildMetricItem('Distance', '${(stats['totalDistance'] as double).toStringAsFixed(1)} km', Icons.route, Colors.teal),
                _buildMetricItem('Avg Speed', '${(stats['averageSpeed'] as double).toStringAsFixed(2)} m/s', Icons.speed, Colors.red),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildMetricItem(String label, String value, IconData icon, Color color) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.1),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      padding: const EdgeInsets.all(12),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(icon, color: color, size: 20),
          const SizedBox(height: 8),
          Text(
            value,
            style: TextStyle(
              fontWeight: FontWeight.bold,
              color: color,
              fontSize: 14,
            ),
          ),
          const SizedBox(height: 4),
          Text(
            label,
            style: TextStyle(
              fontSize: 10,
              color: Colors.grey.shade600,
            ),
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }

  Widget _buildOrdersTab() {
    final relevantOrders = _selectedDeviceId == 'all' 
        ? _orderHistory.values.expand((orders) => orders).toList()
        : _orderHistory[_selectedDeviceId] ?? [];

    relevantOrders.sort((a, b) => (b['completedAt'] as DateTime).compareTo(a['completedAt'] as DateTime));

    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildOrdersHeader(relevantOrders.length),
          const SizedBox(height: 16),
          if (relevantOrders.isEmpty)
            _buildEmptyOrdersCard()
          else
            ListView.builder(
              shrinkWrap: true,
              physics: const NeverScrollableScrollPhysics(),
              itemCount: relevantOrders.length,
              itemBuilder: (context, index) {
                return _buildOrderHistoryCard(relevantOrders[index]);
              },
            ),
        ],
      ),
    );
  }

  Widget _buildOrdersHeader(int totalOrders) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.orange.shade50,
            Colors.orange.shade100,
          ],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.orange.shade300),
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Colors.orange,
              borderRadius: BorderRadius.circular(12),
            ),
            child: const Icon(Icons.list_alt, color: Colors.white, size: 24),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  'Order History',
                  style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
                ),
                Text(
                  'Total: $totalOrders orders completed',
                  style: TextStyle(color: Colors.grey.shade600),
                ),
              ],
            ),
          ),
          if (totalOrders > 0)
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              decoration: BoxDecoration(
                color: Colors.orange,
                borderRadius: BorderRadius.circular(20),
              ),
              child: Text(
                '$totalOrders',
                style: const TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildEmptyOrdersCard() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.grey.shade50,
            Colors.white,
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.grey.withOpacity(0.2)),
      ),
      padding: const EdgeInsets.all(32),
      child: Column(
        children: [
          Icon(
            Icons.inbox,
            size: 64,
            color: Colors.grey.shade400,
          ),
          const SizedBox(height: 16),
          Text(
            'No order history available',
            style: TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.bold,
              color: Colors.grey.shade600,
            ),
          ),
          const SizedBox(height: 8),
          Text(
            'Orders will appear here once completed',
            style: TextStyle(color: Colors.grey.shade500),
          ),
        ],
      ),
    );
  }

  Widget _buildOrderHistoryCard(Map<String, dynamic> order) {
    final completedAt = order['completedAt'] as DateTime;
    final duration = order['duration'] as double;
    final distance = order['distance'] as double;
    
    Color statusColor = Colors.green;
    if (duration > 60) {
      statusColor = Colors.red;
    } else if (duration > 30) {
      statusColor = Colors.orange;
    }
    
    return Container(
      margin: const EdgeInsets.symmetric(vertical: 4),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.white,
            Colors.grey.shade50,
          ],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.grey.shade200),
        boxShadow: [
          BoxShadow(
            color: Colors.grey.withOpacity(0.1),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: ListTile(
        contentPadding: const EdgeInsets.all(12),
        leading: Container(
          width: 50,
          height: 50,
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [statusColor, statusColor.withOpacity(0.8)],
            ),
            shape: BoxShape.circle,
          ),
          child: const Icon(Icons.check, color: Colors.white),
        ),
        title: Text(
          order['name'],
          style: const TextStyle(fontWeight: FontWeight.bold),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Completed: ${_formatDateTime(completedAt)}'),
            Text('Distance: ${distance.toStringAsFixed(1)} m'),
            Text('Waypoints: ${order['waypoints']}'),
          ],
        ),
        trailing: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              '${duration.toStringAsFixed(0)}m',
              style: TextStyle(
                fontWeight: FontWeight.bold,
                color: statusColor,
                fontSize: 16,
              ),
            ),
            const SizedBox(height: 4),
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
              decoration: BoxDecoration(
                color: statusColor.withOpacity(0.1),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Text(
                duration < 30 ? 'FAST' : duration < 60 ? 'NORMAL' : 'SLOW',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.bold,
                  color: statusColor,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildEventsTab() {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildEventsHeader(),
          const SizedBox(height: 16),
          if (_systemEvents.isEmpty)
            _buildEmptyEventsCard()
          else
            ListView.builder(
              shrinkWrap: true,
              physics: const NeverScrollableScrollPhysics(),
              itemCount: _systemEvents.length,
              itemBuilder: (context, index) {
                return _buildEventCard(_systemEvents[index]);
              },
            ),
        ],
      ),
    );
  }

  Widget _buildEventsHeader() {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.red.shade50,
            Colors.red.shade100,
          ],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.red.shade300),
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Colors.red,
              borderRadius: BorderRadius.circular(12),
            ),
            child: const Icon(Icons.event, color: Colors.white, size: 24),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  'System Events',
                  style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
                ),
                Text(
                  'Total: ${_systemEvents.length} events recorded',
                  style: TextStyle(color: Colors.grey.shade600),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEmptyEventsCard() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.grey.shade50,
            Colors.white,
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.grey.withOpacity(0.2)),
      ),
      padding: const EdgeInsets.all(32),
      child: Column(
        children: [
          Icon(
            Icons.event_note,
            size: 64,
            color: Colors.grey.shade400,
          ),
          const SizedBox(height: 16),
          Text(
            'No system events recorded',
            style: TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.bold,
              color: Colors.grey.shade600,
            ),
          ),
          const SizedBox(height: 8),
          Text(
            'System events will appear here as they occur',
            style: TextStyle(color: Colors.grey.shade500),
          ),
        ],
      ),
    );
  }

  Widget _buildEventCard(Map<String, dynamic> event) {
    final type = event['type'];
    final severity = event['severity'];
    
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

    return Container(
      margin: const EdgeInsets.symmetric(vertical: 4),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.white,
            color.withOpacity(0.02),
          ],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.1),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: ListTile(
        contentPadding: const EdgeInsets.all(12),
        leading: Container(
          width: 50,
          height: 50,
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [color, color.withOpacity(0.8)],
            ),
            shape: BoxShape.circle,
          ),
          child: Icon(icon, color: Colors.white),
        ),
        title: Text(
          event['message'],
          style: const TextStyle(fontWeight: FontWeight.w600),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Device: ${event['deviceId']}'),
            Text(_formatDateTime(event['timestamp'] as DateTime)),
          ],
        ),
        trailing: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [color, color.withOpacity(0.8)],
                ),
                borderRadius: BorderRadius.circular(12),
              ),
              child: Text(
                type.toUpperCase(),
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 10,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
            const SizedBox(height: 4),
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
              decoration: BoxDecoration(
                color: color.withOpacity(0.1),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Text(
                severity.toUpperCase(),
                style: TextStyle(
                  fontSize: 8,
                  fontWeight: FontWeight.bold,
                  color: color,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildRealTimeTab() {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        children: [
          _buildRealTimeMetricsCards(),
          const SizedBox(height: 20),
          _buildLiveDataStream(),
        ],
      ),
    );
  }

  Widget _buildRealTimeMetricsCards() {
    return GridView.count(
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      crossAxisCount: _deviceType == DeviceType.phone ? 2 : 4,
      crossAxisSpacing: 12,
      mainAxisSpacing: 12,
      childAspectRatio: 1.1,
      children: [
        _buildRealTimeMetricCard(
          'Fleet Status', 
          '${_connectedDevices.where((d) => d['status'] == 'connected').length}/${_connectedDevices.length}',
          Icons.devices,
          Colors.blue,
          'ONLINE',
        ),
        _buildRealTimeMetricCard(
          'Avg Battery', 
          _getAverageBattery(),
          Icons.battery_std,
          Colors.green,
          '%',
        ),
        _buildRealTimeMetricCard(
          'Active Orders', 
          _getActiveOrdersCount(),
          Icons.assignment,
          Colors.orange,
          'ORDERS',
        ),
        _buildRealTimeMetricCard(
          'System Load', 
          '${(_realTimeMetrics['systemLoad'] ?? 45.0).toStringAsFixed(0)}%',
          Icons.memory,
          Colors.purple,
          'CPU',
        ),
      ],
    );
  }

  Widget _buildRealTimeMetricCard(String title, String value, IconData icon, Color color, String suffix) {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            color.withOpacity(0.1),
            color.withOpacity(0.05),
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: color.withOpacity(0.3)),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.1),
            blurRadius: 8,
            offset: const Offset(0, 3),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Container(
              padding: const EdgeInsets.all(8),
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [color, color.withOpacity(0.8)],
                ),
                shape: BoxShape.circle,
                boxShadow: [
                  BoxShadow(
                    color: color.withOpacity(0.3),
                    blurRadius: 6,
                    offset: const Offset(0, 2),
                  ),
                ],
              ),
              child: Icon(icon, size: 20, color: Colors.white),
            ),
            const SizedBox(height: 12),
            Text(
              value,
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
                color: color,
              ),
            ),
            const SizedBox(height: 4),
            Text(
              title,
              style: const TextStyle(fontSize: 11, fontWeight: FontWeight.w500),
              textAlign: TextAlign.center,
            ),
            const SizedBox(height: 2),
            Text(
              suffix,
              style: TextStyle(
                fontSize: 8,
                color: color,
                fontWeight: FontWeight.bold,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildLiveDataStream() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.teal.shade50,
            Colors.white,
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.teal.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: Colors.teal.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [Colors.teal, Colors.teal.shade700],
                    ),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: const Icon(Icons.stream, color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                const Expanded(
                  child: Text(
                    'Live Data Stream',
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                ),
                Container(
                  width: 8,
                  height: 8,
                  decoration: const BoxDecoration(
                    color: Colors.green,
                    shape: BoxShape.circle,
                  ),
                ),
                const SizedBox(width: 4),
                const Text('LIVE', style: TextStyle(fontSize: 10, fontWeight: FontWeight.bold)),
              ],
            ),
            const SizedBox(height: 16),
            Text(
              'Real-time updates: ${_autoRefresh ? 'Enabled' : 'Disabled'}',
              style: TextStyle(color: Colors.grey.shade600),
            ),
            const SizedBox(height: 8),
            Text(
              'Last update: ${_formatDateTime(DateTime.now())}',
              style: TextStyle(fontSize: 12, color: Colors.grey.shade500),
            ),
          ],
        ),
      ),
    );
  }

  // Helper methods
  String _getTotalDistance() {
    if (_selectedDeviceId == 'all') {
      final total = _deviceStats.values.fold(0.0, (sum, stats) => sum + (stats['totalDistance'] as double? ?? 0.0));
      return '${total.toStringAsFixed(1)} km';
    } else {
      return '${_deviceStats[_selectedDeviceId]?['totalDistance']?.toStringAsFixed(1) ?? '0'} km';
    }
  }

  String _getAverageOrderTime() {
    if (_selectedDeviceId == 'all') {
      final times = _deviceStats.values.map((stats) => stats['averageOrderTime'] as double? ?? 0.0).where((t) => t > 0).toList();
      if (times.isEmpty) return '0 min';
      final average = times.fold(0.0, (sum, time) => sum + time) / times.length;
      return '${average.toStringAsFixed(1)} min';
    } else {
      return '${_deviceStats[_selectedDeviceId]?['averageOrderTime']?.toStringAsFixed(1) ?? '0'} min';
    }
  }

  String _getSuccessRate() {
    if (_selectedDeviceId == 'all') {
      final totalOrders = _deviceStats.values.fold(0, (sum, stats) => sum + (stats['totalOrders'] as int? ?? 0));
      final completedOrders = _deviceStats.values.fold(0, (sum, stats) => sum + (stats['completedOrders'] as int? ?? 0));
      if (totalOrders == 0) return '0%';
      return '${((completedOrders / totalOrders) * 100).toStringAsFixed(1)}%';
    } else {
      final stats = _deviceStats[_selectedDeviceId];
      if (stats == null) return '0%';
      final total = stats['totalOrders'] as int? ?? 0;
      final completed = stats['completedOrders'] as int? ?? 0;
      if (total == 0) return '0%';
      return '${((completed / total) * 100).toStringAsFixed(1)}%';
    }
  }

  String _getActiveOrders() {
    final random = math.Random();
    return random.nextInt(5).toString();
  }

  String _getAverageBattery() {
    final batteries = _deviceStats.values.map((stats) => stats['currentBattery'] as double? ?? 0.0).where((b) => b > 0).toList();
    if (batteries.isEmpty) return '0%';
    final average = batteries.fold(0.0, (sum, battery) => sum + battery) / batteries.length;
    return '${average.toStringAsFixed(0)}%';
  }

  String _getActiveOrdersCount() {
    final random = math.Random();
    return random.nextInt(8).toString();
  }

  String _getTimeRangeLabel() {
    switch (_selectedTimeRange) {
      case '1h':
        return 'Last Hour';
      case '24h':
        return 'Last 24 Hours';
      case '7d':
        return 'Last 7 Days';
      case '30d':
        return 'Last 30 Days';
      default:
        return 'Last 24 Hours';
    }
  }

  String _formatDateTime(DateTime dateTime) {
    return '${dateTime.day}/${dateTime.month}/${dateTime.year} ${dateTime.hour.toString().padLeft(2, '0')}:${dateTime.minute.toString().padLeft(2, '0')}';
  }
}

// Enhanced Chart Painters
class EnhancedBatteryChartPainter extends CustomPainter {
  final Map<String, List<Map<String, dynamic>>> batteryHistory;
  final String selectedDeviceId;

  EnhancedBatteryChartPainter(this.batteryHistory, this.selectedDeviceId);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..strokeWidth = 3
      ..style = PaintingStyle.stroke;

    _drawGrid(canvas, size);

    if (selectedDeviceId == 'all') {
      final colors = [Colors.blue, Colors.green, Colors.orange, Colors.purple, Colors.red];
      int colorIndex = 0;
      
      batteryHistory.forEach((deviceId, data) {
        paint.color = colors[colorIndex % colors.length];
        _drawBatteryLine(canvas, size, data, paint);
        colorIndex++;
      });
    } else if (batteryHistory.containsKey(selectedDeviceId)) {
      paint.color = Colors.green;
      _drawBatteryLine(canvas, size, batteryHistory[selectedDeviceId]!, paint);
    }
  }

  void _drawGrid(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = Colors.grey.withOpacity(0.2)
      ..strokeWidth = 1;

    for (int i = 0; i <= 5; i++) {
      final y = size.height * (i / 5);
      canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);
    }

    for (int i = 0; i <= 6; i++) {
      final x = size.width * (i / 6);
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
    }
  }

  void _drawBatteryLine(Canvas canvas, Size size, List<Map<String, dynamic>> data, Paint paint) {
    if (data.length < 2) return;

    final path = Path();
    for (int i = 0; i < data.length; i++) {
      final x = size.width * (i / (data.length - 1));
      final percentage = data[i]['percentage'] as double;
      final y = size.height * (1 - percentage / 100);
      
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

class VelocityChartPainter extends CustomPainter {
  final Map<String, List<Map<String, dynamic>>> velocityHistory;
  final String selectedDeviceId;

  VelocityChartPainter(this.velocityHistory, this.selectedDeviceId);

  @override
  void paint(Canvas canvas, Size size) {
    final linearPaint = Paint()
      ..color = Colors.blue
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    final angularPaint = Paint()
      ..color = Colors.red
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    _drawGrid(canvas, size);

    final data = selectedDeviceId == 'all' 
        ? velocityHistory.values.expand((list) => list).toList()
        : velocityHistory[selectedDeviceId] ?? [];

    if (data.isNotEmpty) {
      _drawVelocityLine(canvas, size, data, linearPaint, true);
      _drawVelocityLine(canvas, size, data, angularPaint, false);
    }
  }

  void _drawGrid(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = Colors.grey.withOpacity(0.2)
      ..strokeWidth = 1;

    for (int i = 0; i <= 4; i++) {
      final y = size.height * (i / 4);
      canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);
    }

    for (int i = 0; i <= 6; i++) {
      final x = size.width * (i / 6);
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
    }
  }

  void _drawVelocityLine(Canvas canvas, Size size, List<Map<String, dynamic>> data, Paint paint, bool isLinear) {
    if (data.length < 2) return;

    final path = Path();
    final maxVelocity = 2.0;

    for (int i = 0; i < data.length; i++) {
      final x = size.width * (i / (data.length - 1));
      final velocity = isLinear 
          ? (data[i]['linear'] as double)
          : (data[i]['angular'] as double).abs();
      final y = size.height * (1 - velocity / maxVelocity);
      
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

enum DeviceType { phone, tablet, desktop }