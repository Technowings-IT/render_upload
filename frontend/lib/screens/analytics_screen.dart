// screens/enhanced_analytics_screen.dart - Modern Robotic Analytics with Full Theme Integration
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'dart:math' as math;
import 'dart:async';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import '../services/theme_service.dart';
import '../widgets/modern_ui_components.dart';

// ============================================================================
// CONSTANTS & DESIGN SYSTEM (Enhanced for Modern Theme)
// ============================================================================
class AppConstants {
  static const double borderRadius = 16.0;
  static const double smallBorderRadius = 8.0;
  static const double cardElevation = 4.0;
  static const double iconSize = 20.0;
  static const double largeIconSize = 24.0;
  static const Duration animationDuration = Duration(milliseconds: 1500);
  static const Duration refreshInterval = Duration(seconds: 30);
}

class AppColors {
  static const Color primary = Color(0xFF2196F3);
  static const Color secondary = Color(0xFF03DAC6);
  static const Color success = Color(0xFF4CAF50);
  static const Color warning = Color(0xFFFF9800);
  static const Color error = Color(0xFFF44336);
  static const Color info = Color(0xFF2196F3);
  static const Color surface = Color(0xFFFAFAFA);
  static const Color cardBackground = Colors.white;

  static const List<Color> chartColors = [
    Color(0xFF2196F3), // Blue
    Color(0xFF4CAF50), // Green
    Color(0xFFFF9800), // Orange
    Color(0xFF9C27B0), // Purple
    Color(0xFFF44336), // Red
    Color(0xFF009688), // Teal
    Color(0xFF795548), // Brown
  ];
}

class AppTextStyles {
  static const TextStyle heading1 = TextStyle(
    fontSize: 28,
    fontWeight: FontWeight.bold,
  );

  static const TextStyle heading2 = TextStyle(
    fontSize: 22,
    fontWeight: FontWeight.bold,
  );

  static const TextStyle heading3 = TextStyle(
    fontSize: 18,
    fontWeight: FontWeight.bold,
  );

  static const TextStyle subtitle = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w500,
  );

  static const TextStyle body = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.normal,
  );

  static const TextStyle caption = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.normal,
  );

  static const TextStyle overline = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.w500,
    letterSpacing: 1.2,
  );
}

// ============================================================================
// MAIN ENHANCED ANALYTICS SCREEN WITH MODERN THEME INTEGRATION
// ============================================================================
class EnhancedAnalyticsScreen extends StatefulWidget {
  @override
  _EnhancedAnalyticsScreenState createState() =>
      _EnhancedAnalyticsScreenState();
}

class _EnhancedAnalyticsScreenState extends State<EnhancedAnalyticsScreen>
    with TickerProviderStateMixin {
  // ============================================================================
  // DEPENDENCIES & CONTROLLERS
  // ============================================================================
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();

  late TabController _tabController;
  late AnimationController _chartAnimationController;
  late AnimationController _dataAnimationController;
  late AnimationController _pulseController;
  late StreamSubscription _realTimeSubscription;

  // ============================================================================
  // STATE VARIABLES
  // ============================================================================
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
  Map<String, List<Map<String, dynamic>>> _navigationHistory = {};
  Map<String, Map<String, dynamic>> _currentNavigationStatus = {};
  Map<String, double> _navigationMetrics = {};

  bool _isLoading = true;
  bool _autoRefresh = true;
  Timer? _refreshTimer;

  // ============================================================================
  // LIFECYCLE METHODS
  // ============================================================================
  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _initializeData();
    _setupRealTimeUpdates();
    _startAutoRefresh();

    WidgetsBinding.instance.addPostFrameCallback((_) {
      _updateDeviceType();
    });
  }

  void _initializeAnimations() {
    _tabController = TabController(length: 6, vsync: this);
    _chartAnimationController = AnimationController(
      duration: AppConstants.animationDuration,
      vsync: this,
    );
    _dataAnimationController = AnimationController(
      duration: const Duration(milliseconds: 1200),
      vsync: this,
    );
    _pulseController = AnimationController(
      duration: const Duration(milliseconds: 2000),
      vsync: this,
    )..repeat(reverse: true);
  }

  @override
  void dispose() {
    _tabController.dispose();
    _chartAnimationController.dispose();
    _dataAnimationController.dispose();
    _pulseController.dispose();
    _realTimeSubscription.cancel();
    _refreshTimer?.cancel();
    super.dispose();
  }

  // ============================================================================
  // DEVICE TYPE & RESPONSIVE HANDLING
  // ============================================================================
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

  // ============================================================================
  // DATA INITIALIZATION & REAL-TIME UPDATES
  // ============================================================================
  void _initializeData() {
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
      _refreshTimer = Timer.periodic(AppConstants.refreshInterval, (_) {
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
        case 'navigation_feedback_update':
          _updateNavigationFeedback(deviceId, data['data'], timestamp);
          break;
        case 'navigation_status_update':
          _updateNavigationStatus(deviceId, data['data'], timestamp);
          break;
      }
    });
  }

  // ============================================================================
  // DATA UPDATE METHODS
  // ============================================================================
  void _updateBatteryHistory(
      String deviceId, Map<String, dynamic> batteryData, DateTime timestamp) {
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
    _batteryHistory[deviceId]!
        .removeWhere((item) => item['timestamp'].isBefore(cutoff));
  }

  void _updateVelocityHistory(
      String deviceId, Map<String, dynamic> velocityData, DateTime timestamp) {
    if (!_velocityHistory.containsKey(deviceId)) {
      _velocityHistory[deviceId] = [];
    }

    _velocityHistory[deviceId]!.add({
      'timestamp': timestamp,
      'linear': velocityData['linear']?.toDouble() ?? 0.0,
      'angular': velocityData['angular']?.toDouble() ?? 0.0,
    });

    final cutoff = timestamp.subtract(const Duration(hours: 1));
    _velocityHistory[deviceId]!
        .removeWhere((item) => item['timestamp'].isBefore(cutoff));
  }

  void _updateErrorHistory(
      String deviceId, Map<String, dynamic> errorData, DateTime timestamp) {
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
    _errorHistory[deviceId]!
        .removeWhere((item) => item['timestamp'].isBefore(cutoff));
  }

  void _updateOrderHistory(
      String deviceId, Map<String, dynamic> orderData, DateTime timestamp) {
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

  void _updateNavigationFeedback(
      String deviceId, Map<String, dynamic> navData, DateTime timestamp) {
    if (!_navigationHistory.containsKey(deviceId)) {
      _navigationHistory[deviceId] = [];
    }

    final feedbackData = {
      'timestamp': timestamp,
      'current_position': {
        'x': navData['current_pose']?['position']?['x']?.toDouble() ?? 0.0,
        'y': navData['current_pose']?['position']?['y']?.toDouble() ?? 0.0,
        'yaw':
            navData['current_pose']?['orientation']?['yaw']?.toDouble() ?? 0.0,
      },
      'navigation_time': navData['navigation_time']?.toDouble() ?? 0.0,
      'estimated_time_remaining':
          navData['estimated_time_remaining']?.toDouble() ?? 0.0,
      'distance_remaining': navData['distance_remaining']?.toDouble() ?? 0.0,
      'number_of_recoveries': navData['number_of_recoveries']?.toInt() ?? 0,
      'speed': navData['speed']?.toDouble() ?? 0.0,
      'goal_pose': navData['goal_pose'] != null
          ? {
              'x': navData['goal_pose']['position']?['x']?.toDouble() ?? 0.0,
              'y': navData['goal_pose']['position']?['y']?.toDouble() ?? 0.0,
            }
          : null,
    };

    _navigationHistory[deviceId]!.add(feedbackData);
    _currentNavigationStatus[deviceId] = feedbackData;

    if (_navigationHistory[deviceId]!.length > 100) {
      _navigationHistory[deviceId]!.removeAt(0);
    }

    _navigationMetrics['current_speed_$deviceId'] = feedbackData['speed'];
    _navigationMetrics['time_remaining_$deviceId'] =
        feedbackData['estimated_time_remaining'];
    _navigationMetrics['distance_remaining_$deviceId'] =
        feedbackData['distance_remaining'];
    _navigationMetrics['recoveries_$deviceId'] =
        feedbackData['number_of_recoveries'].toDouble();
  }

  void _updateNavigationStatus(
      String deviceId, Map<String, dynamic> statusData, DateTime timestamp) {
    if (!_systemEvents.any((event) =>
        event['deviceId'] == deviceId &&
        event['message'].contains('Navigation') &&
        event['timestamp'].isAfter(timestamp.subtract(Duration(seconds: 5))))) {
      _systemEvents.insert(0, {
        'timestamp': timestamp,
        'type': statusData['status_text'] == 'SUCCEEDED' ? 'success' : 'info',
        'message':
            'Navigation ${statusData['status_text']}: ${statusData['result']?['error_msg'] ?? 'Status update'}',
        'deviceId': deviceId,
        'severity': statusData['status_text'] == 'ABORTED' ? 'high' : 'low',
      });

      if (_systemEvents.length > 50) {
        _systemEvents = _systemEvents.take(50).toList();
      }
    }
  }

  // ============================================================================
  // DATA LOADING METHODS
  // ============================================================================
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

      // Update device stats with cross-referenced data
      _updateDeviceStatsWithCrossData();

      _chartAnimationController.forward();
      _dataAnimationController.forward();
    } catch (e) {
      print('❌ Error loading analytics data: $e');
      _generateMockData();
    }

    setState(() {
      _isLoading = false;
    });
  }

  void _updateDeviceStatsWithCrossData() {
    for (final device in _connectedDevices) {
      final deviceId = device['id'];
      final stats = _deviceStats[deviceId] ?? <String, dynamic>{};

      // Update with battery data
      final batteryData = _batteryHistory[deviceId];
      if (batteryData != null && batteryData.isNotEmpty) {
        final currentBattery =
            batteryData.last['percentage']?.toDouble() ?? 0.0;
        final avgBattery = batteryData.fold(0.0,
                (sum, data) => sum + (data['percentage']?.toDouble() ?? 0.0)) /
            batteryData.length;

        stats['currentBattery'] = currentBattery;
        stats['averageBattery'] = avgBattery;
      }

      // Update with order data
      final orderData = _orderHistory[deviceId];
      if (orderData != null) {
        stats['totalOrders'] = orderData.length;
        stats['completedOrders'] =
            orderData.where((order) => order['status'] == 'completed').length;

        if (orderData.isNotEmpty) {
          final avgTime = orderData.fold(
                  0.0,
                  (sum, order) =>
                      sum + (order['duration']?.toDouble() ?? 0.0)) /
              orderData.length;
          stats['averageOrderTime'] = avgTime;
        }
      }

      _deviceStats[deviceId] = stats;
    }

    print('✅ Device stats updated with cross-referenced data');
  }

  Future<void> _loadBatteryData() async {
    try {
      for (final device in _connectedDevices) {
        final deviceId = device['id'];
        final response = await _apiService.getAnalyticsData(
            deviceId, 'battery', _selectedTimeRange);

        print('🔋 Battery data response for $deviceId: ${response.toString()}');

        if (response['success'] == true && response['data'] != null) {
          final dataSource = response['data'];

          // Handle the structured response format
          if (dataSource is Map) {
            List<Map<String, dynamic>> historyData = [];

            // Check for history array
            if (dataSource['history'] != null &&
                dataSource['history'] is List) {
              historyData = (dataSource['history'] as List)
                  .map<Map<String, dynamic>>((item) => {
                        'timestamp': DateTime.parse(item['timestamp']),
                        'voltage': item['voltage']?.toDouble() ?? 0.0,
                        'percentage': item['percentage']?.toDouble() ?? 0.0,
                        'current': item['current']?.toDouble() ?? 0.0,
                        'temperature': item['temperature']?.toDouble() ?? 25.0,
                      })
                  .toList();
            }

            // Add current data point if available
            if (dataSource['current'] != null) {
              final currentData = dataSource['current'];
              historyData.add({
                'timestamp': DateTime.parse(currentData['timestamp']),
                'voltage': currentData['voltage']?.toDouble() ?? 0.0,
                'percentage': currentData['percentage']?.toDouble() ?? 0.0,
                'current': currentData['current']?.toDouble() ?? 0.0,
                'temperature': currentData['temperature']?.toDouble() ?? 25.0,
              });
            }

            _batteryHistory[deviceId] = historyData;
          } else if (dataSource is List) {
            // Handle direct list format (fallback)
            _batteryHistory[deviceId] = dataSource
                .map<Map<String, dynamic>>((item) => {
                      'timestamp': DateTime.parse(item['timestamp']),
                      'voltage': item['voltage']?.toDouble() ?? 0.0,
                      'percentage': item['percentage']?.toDouble() ?? 0.0,
                      'current': item['current']?.toDouble() ?? 0.0,
                      'temperature': item['temperature']?.toDouble() ?? 25.0,
                    })
                .toList();
          }

          print(
              '✅ Battery history loaded for $deviceId: ${_batteryHistory[deviceId]?.length ?? 0} data points');
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
            deviceId, 'orders', _selectedTimeRange);

        print('📋 Order data response for $deviceId: ${response.toString()}');

        if (response['success'] == true && response['data'] != null) {
          final dataSource = response['data'];

          // Handle the structured response format
          if (dataSource is Map) {
            List<Map<String, dynamic>> orderData = [];

            // Check for ordersByHour array or similar structure
            if (dataSource['ordersByHour'] != null &&
                dataSource['ordersByHour'] is List) {
              orderData = (dataSource['ordersByHour'] as List)
                  .where((item) =>
                      item['orders'] != null &&
                      (item['orders'] as List).isNotEmpty)
                  .expand((hourData) => hourData['orders'] as List)
                  .map<Map<String, dynamic>>((item) => {
                        'id': item['id'] ?? 'unknown',
                        'name': item['name'] ?? 'Unnamed Order',
                        'createdAt': DateTime.parse(
                            item['createdAt'] ?? item['timestamp']),
                        'completedAt': DateTime.parse(
                            item['completedAt'] ?? item['timestamp']),
                        'duration': item['duration']?.toDouble() ?? 0.0,
                        'distance': item['distance']?.toDouble() ?? 0.0,
                        'waypoints': item['waypoints'] ?? 0,
                        'status': item['status'] ?? 'completed',
                      })
                  .toList();
            }

            // Check for direct orders array
            if (dataSource['orders'] != null && dataSource['orders'] is List) {
              orderData = (dataSource['orders'] as List)
                  .map<Map<String, dynamic>>((item) => {
                        'id': item['id'] ?? 'unknown',
                        'name': item['name'] ?? 'Unnamed Order',
                        'createdAt': DateTime.parse(
                            item['createdAt'] ?? item['timestamp']),
                        'completedAt': DateTime.parse(
                            item['completedAt'] ?? item['timestamp']),
                        'duration': item['duration']?.toDouble() ?? 0.0,
                        'distance': item['distance']?.toDouble() ?? 0.0,
                        'waypoints': item['waypoints'] ?? 0,
                        'status': item['status'] ?? 'completed',
                      })
                  .toList();
            }

            _orderHistory[deviceId] = orderData;
          } else if (dataSource is List) {
            // Handle direct list format (fallback)
            _orderHistory[deviceId] = dataSource
                .map<Map<String, dynamic>>((item) => {
                      'id': item['id'] ?? 'unknown',
                      'name': item['name'] ?? 'Unnamed Order',
                      'createdAt': DateTime.parse(item['createdAt']),
                      'completedAt': DateTime.parse(item['completedAt']),
                      'duration': item['duration']?.toDouble() ?? 0.0,
                      'distance': item['distance']?.toDouble() ?? 0.0,
                      'waypoints': item['waypoints'] ?? 0,
                      'status': item['status'] ?? 'completed',
                    })
                .toList();
          }

          print(
              '✅ Order history loaded for $deviceId: ${_orderHistory[deviceId]?.length ?? 0} orders');
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
            deviceId, 'stats', _selectedTimeRange);

        print('📊 Stats data response for $deviceId: ${response.toString()}');

        if (response['success'] == true && response['data'] != null) {
          final dataSource = response['data'];

          // Parse the stats data structure
          Map<String, dynamic> statsData = {};

          if (dataSource is Map) {
            // Extract relevant stats from the structured response
            final current = dataSource['current'];
            if (current != null) {
              statsData = {
                'totalOrders': 0, // Will be updated from order data
                'completedOrders': 0, // Will be updated from order data
                'averageOrderTime': 0.0, // Will be calculated
                'totalUptime': 0.0, // Could be calculated from timestamps
                'totalDistance': current['totalDistance']?.toDouble() ?? 0.0,
                'currentBattery': 0.0, // Will be updated from battery data
                'averageBattery': 0.0, // Will be calculated
                'errorCount': 0, // Will be updated from events
                'averageSpeed': current['averageSpeed']?.toDouble() ?? 0.0,
                'currentPosition': current['position'],
                'maxSpeed': current['maxSpeed']?.toDouble() ?? 0.0,
                'pathEfficiency': current['pathEfficiency']?.toDouble() ?? 0.0,
              };
            }

            // Copy all original data
            statsData.addAll(Map<String, dynamic>.from(dataSource));
          } else {
            statsData = Map<String, dynamic>.from(dataSource);
          }

          _deviceStats[deviceId] = statsData;
          print('✅ Device stats loaded for $deviceId');
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
        // Handle different response formats
        dynamic dataSource = response['data'];
        if (dataSource is Map && dataSource.containsKey('recentEvents')) {
          dataSource = dataSource['recentEvents'];
        }

        if (dataSource is List) {
          _systemEvents = dataSource
              .map<Map<String, dynamic>>((item) => {
                    'timestamp': DateTime.parse(item['timestamp']),
                    'type': item['type'] ?? 'info',
                    'message': item['message'] ?? 'Unknown event',
                    'deviceId': item['deviceId'] ?? 'unknown',
                    'severity': item['severity'] ?? 'info',
                  })
              .toList();
        }
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
            deviceId, 'performance', _selectedTimeRange);

        print(
            '⚡ Performance data response for $deviceId: ${response.toString()}');

        if (response['success'] == true && response['data'] != null) {
          final dataSource = response['data'];

          // Handle the structured response format
          if (dataSource is Map) {
            List<Map<String, dynamic>> velocityData = [];

            // Check for navigation data with speed history
            if (dataSource['navigation'] != null) {
              final navData = dataSource['navigation'];

              // Generate velocity data points from current navigation data
              if (navData['averageSpeed'] != null ||
                  navData['maxSpeed'] != null) {
                final now = DateTime.now();
                for (int i = 0; i < 10; i++) {
                  velocityData.add({
                    'timestamp': now.subtract(Duration(minutes: i * 6)),
                    'linear': (navData['averageSpeed']?.toDouble() ?? 0.0) *
                        (0.8 + math.Random().nextDouble() * 0.4),
                    'angular': (math.Random().nextDouble() - 0.5) * 0.5,
                  });
                }
              }
            }

            // Check for direct velocity history
            if (dataSource['velocityHistory'] != null &&
                dataSource['velocityHistory'] is List) {
              velocityData = (dataSource['velocityHistory'] as List)
                  .map<Map<String, dynamic>>((item) => {
                        'timestamp': DateTime.parse(item['timestamp']),
                        'linear': item['linear']?.toDouble() ?? 0.0,
                        'angular': item['angular']?.toDouble() ?? 0.0,
                      })
                  .toList();
            }

            _velocityHistory[deviceId] = velocityData;
          } else if (dataSource is List) {
            // Handle direct list format (fallback)
            _velocityHistory[deviceId] = dataSource
                .map<Map<String, dynamic>>((item) => {
                      'timestamp': DateTime.parse(item['timestamp']),
                      'linear': item['linear']?.toDouble() ?? 0.0,
                      'angular': item['angular']?.toDouble() ?? 0.0,
                    })
                .toList();
          }

          print(
              '✅ Performance data loaded for $deviceId: ${_velocityHistory[deviceId]?.length ?? 0} data points');
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
          'percentage':
              math.max(10.0, 100.0 - (index * 2) + random.nextDouble() * 10),
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
          'createdAt':
              completedAt.subtract(Duration(minutes: duration.toInt())),
          'completedAt': completedAt,
          'duration': duration,
          'distance': 50.0 + random.nextDouble() * 200.0,
          'waypoints': random.nextInt(5) + 2,
          'status': 'completed',
        };
      });

      _deviceStats[deviceId] = {
        'totalOrders': _orderHistory[deviceId]?.length ?? 0,
        'completedOrders': _orderHistory[deviceId]
                ?.where((o) => o['status'] == 'completed')
                .length ??
            0,
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
    })
      ..sort((a, b) => b['timestamp'].compareTo(a['timestamp']));
  }

  // ============================================================================
  // BUILD METHODS - MAIN UI WITH MODERN THEME INTEGRATION
  // ============================================================================
  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);
    final screenWidth = MediaQuery.of(context).size.width;
    final isDesktop = screenWidth > 1200;
    final isTablet = screenWidth > 768 && screenWidth <= 1200;

    return Scaffold(
      backgroundColor: Colors.transparent,
      appBar: _buildEnhancedAppBar(theme),
      body: Container(
        decoration: BoxDecoration(gradient: theme.backgroundGradient),
        child: _isLoading
            ? ModernLoadingIndicator(
                message: 'Loading Analytics Data...', size: 60)
            : _buildResponsiveBody(isDesktop, isTablet),
      ),
    );
  }

  PreferredSizeWidget _buildEnhancedAppBar(ThemeService theme) {
    return AppBar(
      title: ShaderMask(
        shaderCallback: (bounds) => theme.primaryGradient.createShader(bounds),
        child: Row(
          children: [
            Container(
              padding: const EdgeInsets.all(8),
              decoration: BoxDecoration(
                gradient: theme.primaryGradient,
                borderRadius: theme.borderRadiusSmall,
                boxShadow: theme.neonGlow,
              ),
              child: Icon(Icons.analytics, color: Colors.white, size: 20),
            ),
            const SizedBox(width: 12),
            Text(
              'Fleet Analytics',
              style: theme.displayMedium.copyWith(
                fontSize: 20,
                color: Colors.white,
              ),
            ),
          ],
        ),
      ),
      backgroundColor: Colors.transparent,
      elevation: 0,
      flexibleSpace: Container(
        decoration: BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: theme.isDarkMode
                ? [
                    const Color(0xFF1E1E2E).withOpacity(0.95),
                    const Color(0xFF262640).withOpacity(0.95),
                  ]
                : [
                    Colors.white.withOpacity(0.95),
                    Colors.white.withOpacity(0.9),
                  ],
          ),
        ),
      ),
      actions: [
        _buildTimeRangeSelector(theme),
        _buildDeviceSelector(theme),
        _buildActionButtons(theme),
      ],
      bottom: _deviceType != DeviceType.phone
          ? TabBar(
              controller: _tabController,
              indicatorColor: theme.accentColor,
              labelColor: theme.accentColor,
              unselectedLabelColor: theme.isDarkMode
                  ? Colors.white.withOpacity(0.6)
                  : Colors.black.withOpacity(0.6),
              indicatorWeight: 3,
              tabs: const [
                Tab(icon: Icon(Icons.dashboard), text: 'Overview'),
                Tab(icon: Icon(Icons.battery_std), text: 'Performance'),
                Tab(icon: Icon(Icons.list_alt), text: 'Orders'),
                Tab(icon: Icon(Icons.navigation), text: 'Navigation'),
                Tab(icon: Icon(Icons.event), text: 'Events'),
                Tab(icon: Icon(Icons.speed), text: 'Real-time'),
              ],
            )
          : null,
    );
  }

  Widget _buildTimeRangeSelector(ThemeService theme) {
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 8),
      padding: const EdgeInsets.symmetric(horizontal: 12),
      decoration: BoxDecoration(
        gradient: theme.glassMorphism.gradient,
        borderRadius: theme.borderRadiusSmall,
        border: theme.glassMorphism.border,
      ),
      child: DropdownButton<String>(
        value: _selectedTimeRange,
        dropdownColor:
            theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
        style: theme.bodyMedium.copyWith(color: theme.accentColor),
        underline: Container(),
        icon: Icon(Icons.expand_more, color: theme.accentColor, size: 16),
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

  Widget _buildDeviceSelector(ThemeService theme) {
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 8),
      padding: const EdgeInsets.symmetric(horizontal: 12),
      decoration: BoxDecoration(
        gradient: theme.glassMorphism.gradient,
        borderRadius: theme.borderRadiusSmall,
        border: theme.glassMorphism.border,
      ),
      child: DropdownButton<String>(
        value: _selectedDeviceId,
        dropdownColor:
            theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
        style: theme.bodyMedium.copyWith(color: theme.accentColor),
        underline: Container(),
        icon: Icon(Icons.expand_more, color: theme.accentColor, size: 16),
        items: [
          const DropdownMenuItem(value: 'all', child: Text('All Devices')),
          ..._connectedDevices.map((device) => DropdownMenuItem(
                value: device['id'],
                child: Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    RoboticStatusIndicator(
                      status: device['status'] ?? 'offline',
                      label: '',
                      size: 8,
                      animated: false,
                    ),
                    const SizedBox(width: 8),
                    Text(device['name'] ?? device['id']),
                  ],
                ),
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

  Widget _buildActionButtons(ThemeService theme) {
    return Row(
      children: [
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 4),
          decoration: BoxDecoration(
            gradient: _autoRefresh ? theme.primaryGradient : null,
            color: _autoRefresh ? null : Colors.grey.withOpacity(0.3),
            borderRadius: BorderRadius.circular(20),
            boxShadow: _autoRefresh ? theme.neonGlow : null,
          ),
          child: IconButton(
            icon: AnimatedBuilder(
              animation: _pulseController,
              builder: (context, child) {
                return Transform.scale(
                  scale:
                      _autoRefresh ? 1.0 + (_pulseController.value * 0.1) : 1.0,
                  child: Icon(
                    _autoRefresh ? Icons.sync : Icons.sync_disabled,
                    color: Colors.white,
                  ),
                );
              },
            ),
            onPressed: () {
              setState(() {
                _autoRefresh = !_autoRefresh;
              });
              _refreshTimer?.cancel();
              if (_autoRefresh) {
                _startAutoRefresh();
              }
            },
            tooltip:
                _autoRefresh ? 'Disable Auto Refresh' : 'Enable Auto Refresh',
          ),
        ),
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 4),
          decoration: BoxDecoration(
            gradient: theme.primaryGradient,
            borderRadius: BorderRadius.circular(20),
            boxShadow: theme.elevationSmall,
          ),
          child: IconButton(
            icon: Icon(Icons.refresh, color: Colors.white),
            onPressed: _loadAnalyticsData,
            tooltip: 'Refresh Data',
          ),
        ),
      ],
    );
  }

  Widget _buildResponsiveBody(bool isDesktop, bool isTablet) {
    switch (_deviceType) {
      case DeviceType.desktop:
        return _buildDesktopLayout(isDesktop, isTablet);
      case DeviceType.tablet:
        return _buildTabletLayout(isDesktop, isTablet);
      case DeviceType.phone:
        return _buildPhoneLayout(isDesktop, isTablet);
    }
  }

  Widget _buildDesktopLayout(bool isDesktop, bool isTablet) {
    return TabBarView(
      controller: _tabController,
      children: [
        _buildOverviewTab(isDesktop, isTablet),
        _buildPerformanceTab(isDesktop, isTablet),
        _buildOrdersTab(isDesktop, isTablet),
        _buildNavigationTab(isDesktop, isTablet),
        _buildEventsTab(isDesktop, isTablet),
        _buildRealTimeTab(isDesktop, isTablet),
      ],
    );
  }

  Widget _buildTabletLayout(bool isDesktop, bool isTablet) {
    return Column(
      children: [
        Container(
          decoration: BoxDecoration(
            color: AppColors.cardBackground,
            border: Border(bottom: BorderSide(color: Colors.grey.shade300)),
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
              Tab(icon: Icon(Icons.navigation), text: 'Navigation'),
              Tab(icon: Icon(Icons.event), text: 'Events'),
              Tab(icon: Icon(Icons.speed), text: 'Real-time'),
            ],
          ),
        ),
        Expanded(
          child: TabBarView(
            controller: _tabController,
            children: [
              _buildOverviewTab(isDesktop, isTablet),
              _buildPerformanceTab(isDesktop, isTablet),
              _buildOrdersTab(isDesktop, isTablet),
              _buildNavigationTab(isDesktop, isTablet),
              _buildEventsTab(isDesktop, isTablet),
              _buildRealTimeTab(isDesktop, isTablet),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildPhoneLayout(bool isDesktop, bool isTablet) {
    return Column(
      children: [
        Expanded(
          child: TabBarView(
            controller: _tabController,
            children: [
              _buildOverviewTab(isDesktop, isTablet),
              _buildPerformanceTab(isDesktop, isTablet),
              _buildOrdersTab(isDesktop, isTablet),
              _buildNavigationTab(isDesktop, isTablet),
              _buildEventsTab(isDesktop, isTablet),
              _buildRealTimeTab(isDesktop, isTablet),
            ],
          ),
        ),
        Container(
          decoration: BoxDecoration(
            color: AppColors.cardBackground,
            border: Border(top: BorderSide(color: Colors.grey.shade300)),
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
              Tab(icon: Icon(Icons.navigation, size: 20), text: 'Nav'),
              Tab(icon: Icon(Icons.event, size: 20), text: 'Events'),
              Tab(icon: Icon(Icons.speed, size: 20), text: 'Live'),
            ],
          ),
        ),
      ],
    );
  }

  // ============================================================================
  // TAB CONTENT BUILDERS WITH MODERN THEME INTEGRATION
  // ============================================================================
  Widget _buildOverviewTab(bool isDesktop, bool isTablet) {
    final theme = Provider.of<ThemeService>(context);

    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        children: [
          // Key Metrics Row
          AnimatedBuilder(
            animation: _dataAnimationController,
            builder: (context, child) {
              return Transform.translate(
                offset: Offset(0, 50 * (1 - _dataAnimationController.value)),
                child: Opacity(
                  opacity: _dataAnimationController.value,
                  child: _buildKeyMetrics(isDesktop, isTablet, theme),
                ),
              );
            },
          ),
          const SizedBox(height: 20),

          // Fleet Health Overview
          AnimatedBuilder(
            animation: _dataAnimationController,
            builder: (context, child) {
              return Transform.translate(
                offset: Offset(0, 50 * (1 - _dataAnimationController.value)),
                child: Opacity(
                  opacity: _dataAnimationController.value,
                  child: _buildFleetHealthOverview(theme),
                ),
              );
            },
          ),
          const SizedBox(height: 20),

          // Performance Charts
          if (isDesktop || isTablet)
            Row(
              children: [
                Expanded(child: _buildBatteryChart(theme)),
                const SizedBox(width: 16),
                Expanded(child: _buildOrderTrendsChart(theme)),
              ],
            )
          else
            Column(
              children: [
                _buildBatteryChart(theme),
                const SizedBox(height: 16),
                _buildOrderTrendsChart(theme),
              ],
            ),
        ],
      ),
    );
  }

  Widget _buildKeyMetrics(bool isDesktop, bool isTablet, ThemeService theme) {
    final totalDevices = _connectedDevices.length;
    final activeDevices =
        _connectedDevices.where((d) => d['status'] == 'connected').length;
    final totalOrders = _deviceStats.values
        .fold(0, (sum, stats) => sum + (stats['totalOrders'] as int? ?? 0));
    final averageUptime = _deviceStats.values.isNotEmpty
        ? _deviceStats.values.fold(
                0.0,
                (sum, stats) =>
                    sum + (stats['totalUptime'] as double? ?? 0.0)) /
            _deviceStats.length
        : 0.0;

    final crossAxisCount = isDesktop ? 4 : (isTablet ? 2 : 2);

    return GridView.count(
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      crossAxisCount: crossAxisCount,
      crossAxisSpacing: 16,
      mainAxisSpacing: 16,
      childAspectRatio: 1.2,
      children: [
        ModernStatsCard(
          title: 'Fleet Size',
          value: totalDevices.toString(),
          subtitle: 'Total AMR units',
          icon: Icons.precision_manufacturing,
          color: theme.infoColor,
          trend: '+2',
          showTrend: true,
        ),
        ModernStatsCard(
          title: 'Active Devices',
          value: activeDevices.toString(),
          subtitle: 'Currently online',
          icon: Icons.power_settings_new,
          color: theme.onlineColor,
          trend: '+1',
          showTrend: true,
        ),
        ModernStatsCard(
          title: 'Total Orders',
          value: totalOrders.toString(),
          subtitle: 'Completed today',
          icon: Icons.assignment_turned_in,
          color: theme.warningColor,
          trend: '+12%',
          showTrend: true,
        ),
        ModernStatsCard(
          title: 'Avg Uptime',
          value: '${averageUptime.toStringAsFixed(0)}h',
          subtitle: 'System reliability',
          icon: Icons.schedule,
          color: theme.accentColor,
          trend: '+5%',
          showTrend: true,
        ),
      ],
    );
  }

  Widget _buildFleetHealthOverview(ThemeService theme) {
    return ModernGlassCard(
      showGlow: true,
      glowColor: theme.successColor,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [
                      theme.successColor,
                      theme.successColor.withOpacity(0.8)
                    ],
                  ),
                  borderRadius: theme.borderRadiusSmall,
                  boxShadow: [
                    BoxShadow(
                      color: theme.successColor.withOpacity(0.3),
                      blurRadius: 8,
                      spreadRadius: 2,
                    ),
                  ],
                ),
                child: Icon(Icons.health_and_safety,
                    color: Colors.white, size: 24),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('Fleet Health Overview', style: theme.headlineLarge),
                    Text(
                      'Real-time system status monitoring',
                      style: theme.bodyMedium.copyWith(
                        color: theme.isDarkMode
                            ? Colors.white.withOpacity(0.7)
                            : Colors.black.withOpacity(0.7),
                      ),
                    ),
                  ],
                ),
              ),
              AnimatedBuilder(
                animation: _pulseController,
                builder: (context, child) {
                  return Transform.scale(
                    scale: 1.0 + (_pulseController.value * 0.1),
                    child: Container(
                      padding: const EdgeInsets.symmetric(
                          horizontal: 12, vertical: 6),
                      decoration: BoxDecoration(
                        gradient: theme.primaryGradient,
                        borderRadius: theme.borderRadiusSmall,
                        boxShadow: theme.neonGlow,
                      ),
                      child: Text(
                        'LIVE',
                        style: theme.bodySmall.copyWith(
                          color: Colors.white,
                          fontWeight: FontWeight.bold,
                          letterSpacing: 1.2,
                        ),
                      ),
                    ),
                  );
                },
              ),
            ],
          ),
          const SizedBox(height: 20),
          if (_connectedDevices.isEmpty)
            Center(
              child: Column(
                children: [
                  Icon(Icons.device_unknown, size: 48, color: theme.errorColor),
                  const SizedBox(height: 12),
                  Text('No devices connected', style: theme.headlineMedium),
                  Text('Connect AMR devices to see fleet health',
                      style: theme.bodyMedium),
                ],
              ),
            )
          else
            ..._connectedDevices
                .map((device) => _buildDeviceHealthItem(device, theme)),
        ],
      ),
    );
  }

  Widget _buildDeviceHealthItem(
      Map<String, dynamic> device, ThemeService theme) {
    final deviceId = device['id'];
    final stats = _deviceStats[deviceId];
    final batteryLevel = stats?['currentBattery']?.toDouble() ?? 0.0;
    final isOnline = device['status'] == 'connected';

    return Container(
      margin: const EdgeInsets.only(bottom: 12),
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: theme.isDarkMode
              ? [
                  const Color(0xFF262640).withOpacity(0.5),
                  const Color(0xFF1E1E2E).withOpacity(0.5),
                ]
              : [
                  Colors.white.withOpacity(0.8),
                  Colors.grey.shade50.withOpacity(0.8),
                ],
        ),
        borderRadius: theme.borderRadiusMedium,
        border: Border.all(
          color: isOnline
              ? theme.onlineColor.withOpacity(0.3)
              : theme.offlineColor.withOpacity(0.3),
        ),
        boxShadow: theme.elevationSmall,
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: isOnline
                    ? [theme.onlineColor, theme.onlineColor.withOpacity(0.8)]
                    : [theme.offlineColor, theme.offlineColor.withOpacity(0.8)],
              ),
              shape: BoxShape.circle,
              boxShadow: [
                BoxShadow(
                  color: (isOnline ? theme.onlineColor : theme.offlineColor)
                      .withOpacity(0.3),
                  blurRadius: 8,
                  spreadRadius: 2,
                ),
              ],
            ),
            child: Icon(
              isOnline ? Icons.smart_toy : Icons.warning,
              color: Colors.white,
              size: 20,
            ),
          ),
          const SizedBox(width: 16),
          Expanded(
            flex: 2,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  device['name'] ?? deviceId,
                  style: theme.headlineMedium,
                ),
                const SizedBox(height: 4),
                RoboticStatusIndicator(
                  status: device['status'] ?? 'offline',
                  label: (device['status'] ?? 'offline').toUpperCase(),
                  animated: isOnline,
                ),
              ],
            ),
          ),
          Expanded(
            flex: 3,
            child: Column(
              children: [
                Row(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    Text('Battery', style: theme.bodySmall),
                    Text('${batteryLevel.toStringAsFixed(0)}%',
                        style: theme.bodySmall
                            .copyWith(fontWeight: FontWeight.bold)),
                  ],
                ),
                const SizedBox(height: 4),
                Container(
                  height: 8,
                  decoration: BoxDecoration(
                    color: theme.isDarkMode
                        ? Colors.white.withOpacity(0.1)
                        : Colors.grey.shade300,
                    borderRadius: BorderRadius.circular(4),
                  ),
                  child: FractionallySizedBox(
                    alignment: Alignment.centerLeft,
                    widthFactor: batteryLevel / 100.0,
                    child: Container(
                      decoration: BoxDecoration(
                        gradient: LinearGradient(
                          colors: batteryLevel > 30
                              ? [
                                  theme.successColor,
                                  theme.successColor.withOpacity(0.8)
                                ]
                              : batteryLevel > 15
                                  ? [
                                      theme.warningColor,
                                      theme.warningColor.withOpacity(0.8)
                                    ]
                                  : [
                                      theme.errorColor,
                                      theme.errorColor.withOpacity(0.8)
                                    ],
                        ),
                        borderRadius: BorderRadius.circular(4),
                        boxShadow: [
                          BoxShadow(
                            color: (batteryLevel > 30
                                    ? theme.successColor
                                    : batteryLevel > 15
                                        ? theme.warningColor
                                        : theme.errorColor)
                                .withOpacity(0.3),
                            blurRadius: 4,
                            spreadRadius: 1,
                          ),
                        ],
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildBatteryChart(ThemeService theme) {
    return ModernGlassCard(
      showGlow: true,
      glowColor: theme.successColor,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                padding: const EdgeInsets.all(8),
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [
                      theme.successColor,
                      theme.successColor.withOpacity(0.8)
                    ],
                  ),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(Icons.battery_charging_full,
                    color: Colors.white, size: 20),
              ),
              const SizedBox(width: 12),
              Text('Battery Levels (${_getTimeRangeLabel()})',
                  style: theme.headlineMedium),
            ],
          ),
          const SizedBox(height: 16),
          Container(
            height: 200,
            child: CustomPaint(
              size: Size.infinite,
              painter: ModernBatteryChartPainter(
                  _batteryHistory, _selectedDeviceId, theme),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildOrderTrendsChart(ThemeService theme) {
    return ModernGlassCard(
      showGlow: true,
      glowColor: theme.warningColor,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                padding: const EdgeInsets.all(8),
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [
                      theme.warningColor,
                      theme.warningColor.withOpacity(0.8)
                    ],
                  ),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(Icons.trending_up, color: Colors.white, size: 20),
              ),
              const SizedBox(width: 12),
              Text('Order Trends (${_getTimeRangeLabel()})',
                  style: theme.headlineMedium),
            ],
          ),
          const SizedBox(height: 16),
          Container(
            height: 200,
            child: CustomPaint(
              size: Size.infinite,
              painter: ModernOrderTrendsChartPainter(
                  _orderHistory, _selectedDeviceId, theme),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildPerformanceTab(bool isDesktop, bool isTablet) {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        children: [
          _buildVelocityChart(),
          const SizedBox(height: 20),
          _buildPerformanceMetrics(),
        ],
      ),
    );
  }

  Widget _buildVelocityChart() {
    final theme = Provider.of<ThemeService>(context);
    return ModernGlassCard(
      showGlow: true,
      glowColor: theme.infoColor,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                padding: const EdgeInsets.all(8),
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [theme.infoColor, theme.infoColor.withOpacity(0.8)],
                  ),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(Icons.speed, color: Colors.white, size: 20),
              ),
              const SizedBox(width: 12),
              Text('Velocity History (${_getTimeRangeLabel()})',
                  style: theme.headlineLarge),
            ],
          ),
          const SizedBox(height: 16),
          Container(
            height: 200,
            child: CustomPaint(
              size: Size.infinite,
              painter: VelocityChartPainter(
                  _velocityHistory, _selectedDeviceId, theme),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildPerformanceMetrics() {
    if (_selectedDeviceId == 'all') {
      return EnhancedCard(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [Colors.purple.withOpacity(0.1), AppColors.cardBackground],
        ),
        child: Column(
          children: [
            Icon(Icons.analytics, size: 48, color: Colors.purple),
            const SizedBox(height: 16),
            Text(
              'Select a specific device to view detailed performance metrics',
              style: AppTextStyles.subtitle,
              textAlign: TextAlign.center,
            ),
          ],
        ),
      );
    }

    final stats = _deviceStats[_selectedDeviceId];
    if (stats == null) {
      return EnhancedCard(
        child: Text(
          'No performance data available for this device',
          style: AppTextStyles.subtitle,
          textAlign: TextAlign.center,
        ),
      );
    }

    return _buildDevicePerformanceDetails(stats);
  }

  Widget _buildDevicePerformanceDetails(Map<String, dynamic> stats) {
    return EnhancedCard(
      gradient: LinearGradient(
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
        colors: [Colors.indigo.withOpacity(0.1), AppColors.cardBackground],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          CardHeader(
            title: 'Performance Metrics',
            icon: Icons.assessment,
            color: Colors.indigo,
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
              MetricItem(
                  label: 'Total Orders',
                  value: (stats['totalOrders'] ?? 0).toString(),
                  icon: Icons.assignment,
                  color: AppColors.info),
              MetricItem(
                  label: 'Completed',
                  value: (stats['completedOrders'] ?? 0).toString(),
                  icon: Icons.check_circle,
                  color: AppColors.success),
              MetricItem(
                  label: 'Avg Time',
                  value:
                      '${(stats['averageOrderTime'] as double? ?? 0.0).toStringAsFixed(1)} min',
                  icon: Icons.timer,
                  color: AppColors.warning),
              MetricItem(
                  label: 'Uptime',
                  value:
                      '${(stats['totalUptime'] as double? ?? 0.0).toStringAsFixed(0)} h',
                  icon: Icons.schedule,
                  color: Colors.purple),
              MetricItem(
                  label: 'Distance',
                  value:
                      '${(stats['totalDistance'] as double? ?? 0.0).toStringAsFixed(1)} km',
                  icon: Icons.route,
                  color: Colors.teal),
              MetricItem(
                  label: 'Avg Speed',
                  value:
                      '${(stats['averageSpeed'] as double? ?? 0.0).toStringAsFixed(2)} m/s',
                  icon: Icons.speed,
                  color: AppColors.error),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildOrdersTab(bool isDesktop, bool isTablet) {
    final relevantOrders = _selectedDeviceId == 'all'
        ? _orderHistory.values.expand((orders) => orders).toList()
        : _orderHistory[_selectedDeviceId] ?? [];

    relevantOrders.sort((a, b) =>
        (b['completedAt'] as DateTime).compareTo(a['completedAt'] as DateTime));

    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          OrdersHeader(totalOrders: relevantOrders.length),
          const SizedBox(height: 16),
          if (relevantOrders.isEmpty)
            EmptyStateCard(
              icon: Icons.inbox,
              title: 'No order history available',
              subtitle: 'Orders will appear here once completed',
            )
          else
            ListView.builder(
              shrinkWrap: true,
              physics: const NeverScrollableScrollPhysics(),
              itemCount: relevantOrders.length,
              itemBuilder: (context, index) {
                return OrderHistoryCard(order: relevantOrders[index]);
              },
            ),
        ],
      ),
    );
  }

  Widget _buildNavigationTab(bool isDesktop, bool isTablet) {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          NavigationHeader(activeNavigations: _currentNavigationStatus.length),
          const SizedBox(height: 16),
          _buildCurrentNavigationStatus(),
          const SizedBox(height: 20),
          _buildNavigationMetrics(),
          const SizedBox(height: 20),
          _buildNavigationHistory(),
        ],
      ),
    );
  }

  Widget _buildCurrentNavigationStatus() {
    if (_selectedDeviceId == 'all') {
      return GridView.builder(
        shrinkWrap: true,
        physics: const NeverScrollableScrollPhysics(),
        gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
          crossAxisCount: _deviceType == DeviceType.phone ? 1 : 2,
          crossAxisSpacing: 12,
          mainAxisSpacing: 12,
          childAspectRatio: 1.5,
        ),
        itemCount: _currentNavigationStatus.length,
        itemBuilder: (context, index) {
          final deviceId = _currentNavigationStatus.keys.elementAt(index);
          final status = _currentNavigationStatus[deviceId]!;
          return NavigationStatusCard(deviceId: deviceId, status: status);
        },
      );
    } else if (_currentNavigationStatus.containsKey(_selectedDeviceId)) {
      return NavigationStatusCard(
        deviceId: _selectedDeviceId,
        status: _currentNavigationStatus[_selectedDeviceId]!,
      );
    } else {
      return EmptyStateCard(
        icon: Icons.navigation_outlined,
        title: 'No active navigation',
        subtitle:
            'Navigation data will appear here when robots are actively navigating',
      );
    }
  }

  Widget _buildNavigationMetrics() {
    return EnhancedCard(
      gradient: LinearGradient(
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
        colors: [Colors.indigo.withOpacity(0.1), AppColors.cardBackground],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          CardHeader(
            title: 'Navigation Statistics',
            icon: Icons.analytics,
            color: Colors.indigo,
          ),
          const SizedBox(height: 16),
          GridView.count(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            crossAxisCount: _deviceType == DeviceType.phone ? 2 : 4,
            crossAxisSpacing: 12,
            mainAxisSpacing: 12,
            childAspectRatio: 1.2,
            children: [
              EnhancedStatCard(
                  title: 'Avg Speed',
                  value: _getAverageNavigationSpeed(),
                  icon: Icons.speed,
                  color: AppColors.info),
              EnhancedStatCard(
                  title: 'Total Recoveries',
                  value: _getTotalRecoveries(),
                  icon: Icons.refresh,
                  color: AppColors.error),
              EnhancedStatCard(
                  title: 'Success Rate',
                  value: _getNavigationSuccessRate(),
                  icon: Icons.check_circle,
                  color: AppColors.success),
              EnhancedStatCard(
                  title: 'Avg Time',
                  value: _getAverageNavigationTime(),
                  icon: Icons.timer,
                  color: AppColors.warning),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildNavigationHistory() {
    return EnhancedCard(
      gradient: LinearGradient(
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
        colors: [Colors.purple.withOpacity(0.1), AppColors.cardBackground],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          CardHeader(
            title: 'Navigation Timeline',
            icon: Icons.history,
            color: Colors.purple,
          ),
          const SizedBox(height: 16),
          Container(
            height: 200,
            child: CustomPaint(
              size: Size.infinite,
              painter: NavigationTimelinePainter(
                  _navigationHistory, _selectedDeviceId),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEventsTab(bool isDesktop, bool isTablet) {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          EventsHeader(totalEvents: _systemEvents.length),
          const SizedBox(height: 16),
          if (_systemEvents.isEmpty)
            EmptyStateCard(
              icon: Icons.event_note,
              title: 'No system events recorded',
              subtitle: 'System events will appear here as they occur',
            )
          else
            ListView.builder(
              shrinkWrap: true,
              physics: const NeverScrollableScrollPhysics(),
              itemCount: _systemEvents.length,
              itemBuilder: (context, index) {
                return EventCard(event: _systemEvents[index]);
              },
            ),
        ],
      ),
    );
  }

  Widget _buildRealTimeTab(bool isDesktop, bool isTablet) {
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
        RealTimeMetricCard(
          title: 'Fleet Status',
          value:
              '${_connectedDevices.where((d) => d['status'] == 'connected').length}/${_connectedDevices.length}',
          icon: Icons.devices,
          color: AppColors.info,
          suffix: 'ONLINE',
        ),
        RealTimeMetricCard(
          title: 'Avg Battery',
          value: _getAverageBattery(),
          icon: Icons.battery_std,
          color: AppColors.success,
          suffix: '%',
        ),
        RealTimeMetricCard(
          title: 'Total Distance',
          value: _getTotalDistance(),
          icon: Icons.route,
          color: AppColors.primary,
          suffix: 'M',
        ),
        RealTimeMetricCard(
          title: 'Est. Time Left',
          value: _getEstimatedTimeLeft(),
          icon: Icons.access_time,
          color: AppColors.warning,
          suffix: 'MIN',
        ),
        RealTimeMetricCard(
          title: 'Nav Time',
          value: _getNavigationTime(),
          icon: Icons.navigation,
          color: Colors.indigo,
          suffix: 'MIN',
        ),
        RealTimeMetricCard(
          title: 'Device Recovery',
          value: _getDeviceRecoveryCount(),
          icon: Icons.healing,
          color: Colors.orange,
          suffix: 'EVENTS',
        ),
        RealTimeMetricCard(
          title: 'Active Orders',
          value: _getActiveOrdersCount(),
          icon: Icons.assignment,
          color: AppColors.warning,
          suffix: 'ORDERS',
        ),
        RealTimeMetricCard(
          title: 'System Load',
          value:
              '${(_realTimeMetrics['systemLoad'] ?? 45.0).toStringAsFixed(0)}%',
          icon: Icons.memory,
          color: Colors.purple,
          suffix: 'CPU',
        ),
      ],
    );
  }

  Widget _buildLiveDataStream() {
    return EnhancedCard(
      gradient: LinearGradient(
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
        colors: [Colors.teal.withOpacity(0.1), AppColors.cardBackground],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              EnhancedIconContainer(
                icon: Icons.stream,
                gradient:
                    LinearGradient(colors: [Colors.teal, Colors.teal.shade700]),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: Text('Live Data Stream', style: AppTextStyles.heading3),
              ),
              Container(
                width: 8,
                height: 8,
                decoration: const BoxDecoration(
                  color: AppColors.success,
                  shape: BoxShape.circle,
                ),
              ),
              const SizedBox(width: 4),
              Text('LIVE', style: AppTextStyles.overline),
            ],
          ),
          const SizedBox(height: 16),
          Text(
            'Real-time updates: ${_autoRefresh ? 'Enabled' : 'Disabled'}',
            style: AppTextStyles.body.copyWith(color: Colors.grey.shade600),
          ),
          const SizedBox(height: 8),
          Text(
            'Last update: ${_formatDateTime(DateTime.now())}',
            style: AppTextStyles.caption.copyWith(color: Colors.grey.shade500),
          ),
        ],
      ),
    );
  }

  // ============================================================================
  // HELPER METHODS
  // ============================================================================
  String _getTotalDistance() {
    if (_selectedDeviceId == 'all') {
      final total = _deviceStats.values.fold(0.0,
          (sum, stats) => sum + (stats['totalDistance'] as double? ?? 0.0));
      // Convert meters to kilometers if value is large, otherwise keep as meters
      if (total >= 1000) {
        return '${(total / 1000).toStringAsFixed(1)} km';
      } else {
        return '${total.toStringAsFixed(0)} m';
      }
    } else {
      final distance =
          _deviceStats[_selectedDeviceId]?['totalDistance']?.toDouble() ?? 0.0;
      if (distance >= 1000) {
        return '${(distance / 1000).toStringAsFixed(1)} km';
      } else {
        return '${distance.toStringAsFixed(0)} m';
      }
    }
  }

  String _getAverageOrderTime() {
    if (_selectedDeviceId == 'all') {
      final times = _deviceStats.values
          .map((stats) => stats['averageOrderTime'] as double? ?? 0.0)
          .where((t) => t > 0)
          .toList();
      if (times.isEmpty) return '0 min';
      final average = times.fold(0.0, (sum, time) => sum + time) / times.length;
      // Convert seconds to minutes if the value seems to be in seconds
      if (average > 300) {
        // More than 5 minutes likely means it's in seconds
        return '${(average / 60).toStringAsFixed(1)} min';
      }
      return '${average.toStringAsFixed(1)} min';
    } else {
      final time =
          _deviceStats[_selectedDeviceId]?['averageOrderTime']?.toDouble() ??
              0.0;
      if (time > 300) {
        return '${(time / 60).toStringAsFixed(1)} min';
      }
      return '${time.toStringAsFixed(1)} min';
    }
  }

  String _getAverageBattery() {
    if (_selectedDeviceId == 'all') {
      final batteries = _deviceStats.values
          .map((stats) => stats['currentBattery'] as double? ?? 0.0)
          .where((b) => b > 0)
          .toList();
      if (batteries.isEmpty) return '0';
      final average =
          batteries.fold(0.0, (sum, bat) => sum + bat) / batteries.length;
      return '${average.toStringAsFixed(0)}';
    } else {
      final battery =
          _deviceStats[_selectedDeviceId]?['currentBattery']?.toDouble() ?? 0.0;
      return '${battery.toStringAsFixed(0)}';
    }
  }

  String _getEstimatedTimeLeft() {
    final times = _navigationMetrics.entries
        .where((entry) => entry.key.contains('time_remaining'))
        .map((entry) => entry.value)
        .where((time) => time > 0)
        .toList();

    if (times.isEmpty) return '0';

    final avgTime = times.fold(0.0, (sum, time) => sum + time) / times.length;
    return '${(avgTime / 60).toStringAsFixed(0)}'; // Convert seconds to minutes
  }

  String _getNavigationTime() {
    final activeNavs = _currentNavigationStatus.values
        .map((status) => status['navigation_time'] as double? ?? 0.0)
        .where((time) => time > 0)
        .toList();

    if (activeNavs.isEmpty) return '0';

    final avgTime =
        activeNavs.fold(0.0, (sum, time) => sum + time) / activeNavs.length;
    return '${(avgTime / 60).toStringAsFixed(0)}'; // Convert seconds to minutes
  }

  String _getDeviceRecoveryCount() {
    final recoveries = _navigationMetrics.entries
        .where((entry) => entry.key.contains('recoveries'))
        .map((entry) => entry.value)
        .fold(0.0, (sum, count) => sum + count);

    return '${recoveries.toStringAsFixed(0)}';
  }

  String _getActiveOrdersCount() {
    // This would need to be implemented based on actual active orders tracking
    return '${_currentNavigationStatus.length}';
  }

  String _getAverageNavigationSpeed() {
    final speeds = _navigationMetrics.entries
        .where((entry) => entry.key.contains('current_speed'))
        .map((entry) => entry.value)
        .where((speed) => speed > 0)
        .toList();

    if (speeds.isEmpty) return '0.00 m/s';

    final avgSpeed =
        speeds.fold(0.0, (sum, speed) => sum + speed) / speeds.length;
    return '${avgSpeed.toStringAsFixed(2)} m/s';
  }

  String _getTotalRecoveries() {
    final totalRecoveries = _navigationMetrics.entries
        .where((entry) => entry.key.contains('recoveries'))
        .fold(0.0, (sum, entry) => sum + entry.value);

    return '${totalRecoveries.toStringAsFixed(0)}';
  }

  String _getNavigationSuccessRate() {
    // This would need to be calculated based on successful vs failed navigations
    // For now, return a calculated estimate
    final totalRecoveries = _navigationMetrics.entries
        .where((entry) => entry.key.contains('recoveries'))
        .fold(0.0, (sum, entry) => sum + entry.value);

    final totalNavigations = _navigationHistory.values
        .fold(0, (sum, history) => sum + history.length);

    if (totalNavigations == 0) return '100%';

    final successRate =
        ((totalNavigations - totalRecoveries) / totalNavigations * 100)
            .clamp(0, 100);
    return '${successRate.toStringAsFixed(0)}%';
  }

  String _getAverageNavigationTime() {
    final allNavTimes = _navigationHistory.values
        .expand((history) => history)
        .map((nav) => nav['navigation_time'] as double? ?? 0.0)
        .where((time) => time > 0)
        .toList();

    if (allNavTimes.isEmpty) return '0 min';

    final avgTime =
        allNavTimes.fold(0.0, (sum, time) => sum + time) / allNavTimes.length;
    return '${(avgTime / 60).toStringAsFixed(1)} min';
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
    return '${dateTime.day.toString().padLeft(2, '0')}/'
        '${dateTime.month.toString().padLeft(2, '0')}/'
        '${dateTime.year} '
        '${dateTime.hour.toString().padLeft(2, '0')}:'
        '${dateTime.minute.toString().padLeft(2, '0')}';
  }
}

// ============================================================================
// CUSTOM PAINTERS FOR CHARTS WITH GLOW EFFECTS
// ============================================================================
class ModernBatteryChartPainter extends CustomPainter {
  final Map<String, List<Map<String, dynamic>>> batteryHistory;
  final String selectedDeviceId;
  final ThemeService theme;

  ModernBatteryChartPainter(
      this.batteryHistory, this.selectedDeviceId, this.theme);

  @override
  void paint(Canvas canvas, Size size) {
    _drawGrid(canvas, size);
    _drawAxes(canvas, size);

    if (selectedDeviceId == 'all') {
      int colorIndex = 0;
      final colors = [
        theme.successColor,
        theme.warningColor,
        theme.errorColor,
        theme.infoColor
      ];
      batteryHistory.forEach((deviceId, data) {
        final paint = Paint()
          ..color = colors[colorIndex % colors.length]
          ..strokeWidth = 3
          ..style = PaintingStyle.stroke;
        _drawBatteryLine(canvas, size, data, paint);
        colorIndex++;
      });
    } else if (batteryHistory.containsKey(selectedDeviceId)) {
      final paint = Paint()
        ..color = theme.successColor
        ..strokeWidth = 3
        ..style = PaintingStyle.stroke;
      _drawBatteryLine(canvas, size, batteryHistory[selectedDeviceId]!, paint);
    }

    _drawLabels(canvas, size);
  }

  void _drawGrid(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = theme.isDarkMode
          ? Colors.white.withOpacity(0.1)
          : Colors.black.withOpacity(0.1)
      ..strokeWidth = 1;

    // Draw grid lines
    for (int i = 0; i <= 5; i++) {
      final y = size.height * 0.9 * (i / 5) + size.height * 0.05;
      canvas.drawLine(
          Offset(size.width * 0.1, y), Offset(size.width * 0.95, y), gridPaint);
    }

    for (int i = 0; i <= 6; i++) {
      final x = size.width * 0.1 + (size.width * 0.85) * (i / 6);
      canvas.drawLine(Offset(x, size.height * 0.05),
          Offset(x, size.height * 0.95), gridPaint);
    }
  }

  void _drawAxes(Canvas canvas, Size size) {
    final axisPaint = Paint()
      ..color = theme.accentColor
      ..strokeWidth = 2;

    // Y-axis
    canvas.drawLine(
      Offset(size.width * 0.1, size.height * 0.05),
      Offset(size.width * 0.1, size.height * 0.95),
      axisPaint,
    );

    // X-axis
    canvas.drawLine(
      Offset(size.width * 0.1, size.height * 0.95),
      Offset(size.width * 0.95, size.height * 0.95),
      axisPaint,
    );
  }

  void _drawBatteryLine(
      Canvas canvas, Size size, List<Map<String, dynamic>> data, Paint paint) {
    if (data.length < 2) return;

    final path = Path();
    final chartWidth = size.width * 0.85;
    final chartHeight = size.height * 0.9;
    final startX = size.width * 0.1;
    final startY = size.height * 0.05;

    for (int i = 0; i < data.length; i++) {
      final x = startX + chartWidth * (i / (data.length - 1));
      final percentage = (data[i]['percentage'] as double? ?? 0.0);
      final y = startY + chartHeight * (1 - percentage / 100);

      if (i == 0) {
        path.moveTo(x, y);
      } else {
        path.lineTo(x, y);
      }

      // Draw glow effect points
      final glowPaint = Paint()
        ..color = paint.color.withOpacity(0.6)
        ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 3);
      canvas.drawCircle(Offset(x, y), 4, glowPaint);

      // Draw data points
      final pointPaint = Paint()
        ..color = paint.color
        ..style = PaintingStyle.fill;
      canvas.drawCircle(Offset(x, y), 3, pointPaint);
    }

    // Draw glow effect for line
    final glowLinePaint = Paint()
      ..color = paint.color.withOpacity(0.3)
      ..strokeWidth = 6
      ..style = PaintingStyle.stroke
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 2);
    canvas.drawPath(path, glowLinePaint);

    canvas.drawPath(path, paint);
  }

  void _drawLabels(Canvas canvas, Size size) {
    final textPainter = TextPainter(
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.center,
    );

    // Y-axis labels (battery percentage)
    for (int i = 0; i <= 5; i++) {
      final percentage = 100 - (i * 20);
      final y = size.height * 0.05 + (size.height * 0.9) * (i / 5);

      textPainter.text = TextSpan(
        text: '$percentage%',
        style: theme.bodySmall.copyWith(color: theme.accentColor),
      );
      textPainter.layout();
      textPainter.paint(
          canvas, Offset(size.width * 0.02, y - textPainter.height / 2));
    }

    // X-axis labels (time)
    for (int i = 0; i <= 6; i++) {
      final hours = 24 - (i * 4);
      final x = size.width * 0.1 + (size.width * 0.85) * (i / 6);

      textPainter.text = TextSpan(
        text: '${hours}h',
        style: theme.bodySmall.copyWith(color: theme.accentColor),
      );
      textPainter.layout();
      textPainter.paint(
          canvas, Offset(x - textPainter.width / 2, size.height * 0.97));
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class ModernOrderTrendsChartPainter extends CustomPainter {
  final Map<String, List<Map<String, dynamic>>> orderHistory;
  final String selectedDeviceId;
  final ThemeService theme;

  ModernOrderTrendsChartPainter(
      this.orderHistory, this.selectedDeviceId, this.theme);

  @override
  void paint(Canvas canvas, Size size) {
    _drawGrid(canvas, size);
    _drawAxes(canvas, size);

    // Group orders by hour and count them
    final hourlyOrders = <int, int>{};

    if (selectedDeviceId == 'all') {
      orderHistory.values.expand((orders) => orders).forEach((order) {
        final hour = (order['completedAt'] as DateTime).hour;
        hourlyOrders[hour] = (hourlyOrders[hour] ?? 0) + 1;
      });
    } else if (orderHistory.containsKey(selectedDeviceId)) {
      orderHistory[selectedDeviceId]!.forEach((order) {
        final hour = (order['completedAt'] as DateTime).hour;
        hourlyOrders[hour] = (hourlyOrders[hour] ?? 0) + 1;
      });
    }

    _drawOrderBars(canvas, size, hourlyOrders);
    _drawLabels(canvas, size);
  }

  void _drawGrid(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = theme.isDarkMode
          ? Colors.white.withOpacity(0.1)
          : Colors.black.withOpacity(0.1)
      ..strokeWidth = 1;

    for (int i = 0; i <= 5; i++) {
      final y = size.height * 0.9 * (i / 5) + size.height * 0.05;
      canvas.drawLine(
          Offset(size.width * 0.1, y), Offset(size.width * 0.95, y), gridPaint);
    }
  }

  void _drawAxes(Canvas canvas, Size size) {
    final axisPaint = Paint()
      ..color = theme.accentColor
      ..strokeWidth = 2;

    canvas.drawLine(
      Offset(size.width * 0.1, size.height * 0.05),
      Offset(size.width * 0.1, size.height * 0.95),
      axisPaint,
    );

    canvas.drawLine(
      Offset(size.width * 0.1, size.height * 0.95),
      Offset(size.width * 0.95, size.height * 0.95),
      axisPaint,
    );
  }

  void _drawOrderBars(Canvas canvas, Size size, Map<int, int> hourlyOrders) {
    if (hourlyOrders.isEmpty) return;

    final maxOrders = hourlyOrders.values.reduce(math.max).toDouble();
    if (maxOrders == 0) return;

    final chartWidth = size.width * 0.85;
    final chartHeight = size.height * 0.9;
    final startX = size.width * 0.1;
    final startY = size.height * 0.05;
    final barWidth = chartWidth / 24;

    for (int hour = 0; hour < 24; hour++) {
      final orderCount = hourlyOrders[hour] ?? 0;
      if (orderCount == 0) continue;

      final barHeight = chartHeight * (orderCount / maxOrders);
      final x = startX + (hour * barWidth);
      final y = startY + chartHeight - barHeight;

      // Draw glow effect
      final glowPaint = Paint()
        ..color = theme.warningColor.withOpacity(0.4)
        ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 3);

      final glowRect = RRect.fromRectAndRadius(
        Rect.fromLTWH(x, y, barWidth * 0.8, barHeight),
        const Radius.circular(4),
      );
      canvas.drawRRect(glowRect, glowPaint);

      // Draw main bar
      final barPaint = Paint()
        ..shader = LinearGradient(
          colors: [theme.warningColor, theme.warningColor.withOpacity(0.8)],
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
        ).createShader(Rect.fromLTWH(x, y, barWidth * 0.8, barHeight));

      final barRect = RRect.fromRectAndRadius(
        Rect.fromLTWH(x, y, barWidth * 0.8, barHeight),
        const Radius.circular(4),
      );
      canvas.drawRRect(barRect, barPaint);
    }
  }

  void _drawLabels(Canvas canvas, Size size) {
    final textPainter = TextPainter(
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.center,
    );

    // X-axis labels (hours)
    for (int i = 0; i < 24; i += 4) {
      final x = size.width * 0.1 + (size.width * 0.85) * (i / 24);
      textPainter.text = TextSpan(
        text: '${i}h',
        style: theme.bodySmall.copyWith(color: theme.accentColor),
      );
      textPainter.layout();
      textPainter.paint(
          canvas, Offset(x - textPainter.width / 2, size.height * 0.97));
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class VelocityChartPainter extends CustomPainter {
  final Map<String, List<Map<String, dynamic>>> velocityHistory;
  final String selectedDeviceId;
  final ThemeService theme;

  VelocityChartPainter(this.velocityHistory, this.selectedDeviceId, this.theme);

  @override
  void paint(Canvas canvas, Size size) {
    final linearPaint = Paint()
      ..color = theme.infoColor
      ..strokeWidth = 3
      ..style = PaintingStyle.stroke;

    final angularPaint = Paint()
      ..color = theme.errorColor
      ..strokeWidth = 3
      ..style = PaintingStyle.stroke;

    _drawGrid(canvas, size);
    _drawAxes(canvas, size);

    final data = selectedDeviceId == 'all'
        ? velocityHistory.values.expand((list) => list).toList()
        : velocityHistory[selectedDeviceId] ?? [];

    if (data.isNotEmpty) {
      _drawVelocityLine(canvas, size, data, linearPaint, true);
      _drawVelocityLine(canvas, size, data, angularPaint, false);
    }

    _drawLabels(canvas, size);
    _drawLegend(canvas, size);
  }

  void _drawGrid(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = theme.isDarkMode
          ? Colors.white.withOpacity(0.1)
          : Colors.black.withOpacity(0.1)
      ..strokeWidth = 1;

    // Horizontal grid lines
    for (int i = 0; i <= 4; i++) {
      final y = size.height * 0.05 + (size.height * 0.8) * (i / 4);
      canvas.drawLine(
          Offset(size.width * 0.1, y), Offset(size.width * 0.9, y), gridPaint);
    }

    // Vertical grid lines
    for (int i = 0; i <= 6; i++) {
      final x = size.width * 0.1 + (size.width * 0.8) * (i / 6);
      canvas.drawLine(Offset(x, size.height * 0.05),
          Offset(x, size.height * 0.85), gridPaint);
    }
  }

  void _drawAxes(Canvas canvas, Size size) {
    final axisPaint = Paint()
      ..color = theme.accentColor
      ..strokeWidth = 2;

    // Y-axis
    canvas.drawLine(
      Offset(size.width * 0.1, size.height * 0.05),
      Offset(size.width * 0.1, size.height * 0.85),
      axisPaint,
    );

    // X-axis
    canvas.drawLine(
      Offset(size.width * 0.1, size.height * 0.85),
      Offset(size.width * 0.9, size.height * 0.85),
      axisPaint,
    );
  }

  void _drawVelocityLine(Canvas canvas, Size size,
      List<Map<String, dynamic>> data, Paint paint, bool isLinear) {
    if (data.length < 2) return;

    final path = Path();
    final maxVelocity = 2.0;
    final chartWidth = size.width * 0.8;
    final chartHeight = size.height * 0.8;
    final startX = size.width * 0.1;
    final startY = size.height * 0.05;

    for (int i = 0; i < data.length; i++) {
      final x = startX + chartWidth * (i / (data.length - 1));
      final velocity = isLinear
          ? (data[i]['linear'] as double? ?? 0.0)
          : (data[i]['angular'] as double? ?? 0.0).abs();
      final y = startY + chartHeight * (1 - velocity / maxVelocity);

      if (i == 0) {
        path.moveTo(x, y);
      } else {
        path.lineTo(x, y);
      }

      // Draw glow effect points
      final glowPaint = Paint()
        ..color = paint.color.withOpacity(0.6)
        ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 2);
      canvas.drawCircle(Offset(x, y), 3, glowPaint);
    }

    // Draw glow effect for line
    final glowLinePaint = Paint()
      ..color = paint.color.withOpacity(0.3)
      ..strokeWidth = 5
      ..style = PaintingStyle.stroke
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 2);
    canvas.drawPath(path, glowLinePaint);

    canvas.drawPath(path, paint);
  }

  void _drawLabels(Canvas canvas, Size size) {
    final textPainter = TextPainter(
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.center,
    );

    // Y-axis labels
    for (int i = 0; i <= 4; i++) {
      final velocity = 2.0 - (i * 0.5);
      final y = size.height * 0.05 + (size.height * 0.8) * (i / 4);

      textPainter.text = TextSpan(
        text: '${velocity.toStringAsFixed(1)}',
        style: theme.bodySmall.copyWith(color: theme.accentColor),
      );
      textPainter.layout();
      textPainter.paint(
          canvas, Offset(size.width * 0.02, y - textPainter.height / 2));
    }
  }

  void _drawLegend(Canvas canvas, Size size) {
    final textPainter = TextPainter(
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.left,
    );

    // Linear velocity legend
    final linearPaint = Paint()
      ..color = theme.infoColor
      ..strokeWidth = 3;
    canvas.drawLine(
      Offset(size.width * 0.12, size.height * 0.92),
      Offset(size.width * 0.17, size.height * 0.92),
      linearPaint,
    );

    textPainter.text = TextSpan(
      text: 'Linear Velocity (m/s)',
      style: theme.bodySmall.copyWith(color: theme.infoColor),
    );
    textPainter.layout();
    textPainter.paint(canvas, Offset(size.width * 0.19, size.height * 0.91));

    // Angular velocity legend
    final angularPaint = Paint()
      ..color = theme.errorColor
      ..strokeWidth = 3;
    canvas.drawLine(
      Offset(size.width * 0.5, size.height * 0.92),
      Offset(size.width * 0.55, size.height * 0.92),
      angularPaint,
    );

    textPainter.text = TextSpan(
      text: 'Angular Velocity (rad/s)',
      style: theme.bodySmall.copyWith(color: theme.errorColor),
    );
    textPainter.layout();
    textPainter.paint(canvas, Offset(size.width * 0.57, size.height * 0.91));
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class NavigationTimelinePainter extends CustomPainter {
  final Map<String, List<Map<String, dynamic>>> navigationHistory;
  final String selectedDeviceId;

  NavigationTimelinePainter(this.navigationHistory, this.selectedDeviceId);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    _drawGrid(canvas, size);
    _drawAxes(canvas, size);

    if (selectedDeviceId == 'all') {
      int colorIndex = 0;
      navigationHistory.forEach((deviceId, data) {
        paint.color =
            AppColors.chartColors[colorIndex % AppColors.chartColors.length];
        _drawNavigationTimeline(canvas, size, data, paint);
        colorIndex++;
      });
    } else if (navigationHistory.containsKey(selectedDeviceId)) {
      paint.color = Colors.teal;
      _drawNavigationTimeline(
          canvas, size, navigationHistory[selectedDeviceId]!, paint);
    }

    _drawLabels(canvas, size);
  }

  void _drawGrid(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = Colors.grey.withOpacity(0.2)
      ..strokeWidth = 1;

    // Horizontal grid lines
    for (int i = 0; i <= 4; i++) {
      final y = size.height * 0.05 + (size.height * 0.8) * (i / 4);
      canvas.drawLine(
          Offset(size.width * 0.1, y), Offset(size.width * 0.9, y), gridPaint);
    }

    // Vertical grid lines
    for (int i = 0; i <= 6; i++) {
      final x = size.width * 0.1 + (size.width * 0.8) * (i / 6);
      canvas.drawLine(Offset(x, size.height * 0.05),
          Offset(x, size.height * 0.85), gridPaint);
    }
  }

  void _drawAxes(Canvas canvas, Size size) {
    final axisPaint = Paint()
      ..color = Colors.grey.shade600
      ..strokeWidth = 2;

    // Y-axis
    canvas.drawLine(
      Offset(size.width * 0.1, size.height * 0.05),
      Offset(size.width * 0.1, size.height * 0.85),
      axisPaint,
    );

    // X-axis
    canvas.drawLine(
      Offset(size.width * 0.1, size.height * 0.85),
      Offset(size.width * 0.9, size.height * 0.85),
      axisPaint,
    );
  }

  void _drawNavigationTimeline(
      Canvas canvas, Size size, List<Map<String, dynamic>> data, Paint paint) {
    if (data.length < 2) return;

    final path = Path();
    final maxDistance = data.fold(
        0.0,
        (max, nav) =>
            math.max(max, nav['distance_remaining'] as double? ?? 0.0));

    if (maxDistance == 0) return;

    final chartWidth = size.width * 0.8;
    final chartHeight = size.height * 0.8;
    final startX = size.width * 0.1;
    final startY = size.height * 0.05;

    for (int i = 0; i < data.length; i++) {
      final x = startX + chartWidth * (i / (data.length - 1));
      final distance = (data[i]['distance_remaining'] as double? ?? 0.0);
      final y = startY + chartHeight * (1 - distance / maxDistance);

      if (i == 0) {
        path.moveTo(x, y);
      } else {
        path.lineTo(x, y);
      }

      // Draw data points
      final pointPaint = Paint()
        ..color = paint.color
        ..style = PaintingStyle.fill;
      canvas.drawCircle(Offset(x, y), 2, pointPaint);
    }

    canvas.drawPath(path, paint);
  }

  void _drawLabels(Canvas canvas, Size size) {
    final textPainter = TextPainter(
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.center,
    );

    // Y-axis label
    textPainter.text = TextSpan(
      text: 'Distance Remaining (m)',
      style: AppTextStyles.caption.copyWith(color: Colors.grey.shade600),
    );
    textPainter.layout();

    canvas.save();
    canvas.translate(size.width * 0.02, size.height * 0.5);
    canvas.rotate(-math.pi / 2);
    textPainter.paint(
        canvas, Offset(-textPainter.width / 2, -textPainter.height / 2));
    canvas.restore();

    // X-axis label
    textPainter.text = TextSpan(
      text: 'Navigation Progress',
      style: AppTextStyles.caption.copyWith(color: Colors.grey.shade600),
    );
    textPainter.layout();
    textPainter.paint(canvas,
        Offset(size.width * 0.5 - textPainter.width / 2, size.height * 0.92));
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

// ============================================================================
// ENUMS & UTILITY CLASSES
// ============================================================================
enum DeviceType { phone, tablet, desktop }

// ============================================================================
// FORMAT UTILITIES
// ============================================================================
class FormatUtils {
  static String formatDuration(double minutes) {
    if (minutes < 60) {
      return '${minutes.toStringAsFixed(0)} min';
    }
    final hours = minutes / 60;
    return '${hours.toStringAsFixed(1)} h';
  }

  static String formatDistance(double meters) {
    if (meters < 1000) {
      return '${meters.toStringAsFixed(1)} m';
    }
    final kilometers = meters / 1000;
    return '${kilometers.toStringAsFixed(2)} km';
  }

  static String formatDateTime(DateTime dateTime) {
    return '${dateTime.day.toString().padLeft(2, '0')}/'
        '${dateTime.month.toString().padLeft(2, '0')}/'
        '${dateTime.year} '
        '${dateTime.hour.toString().padLeft(2, '0')}:'
        '${dateTime.minute.toString().padLeft(2, '0')}';
  }

  static String formatTimeAgo(DateTime dateTime) {
    final now = DateTime.now();
    final difference = now.difference(dateTime);

    if (difference.inMinutes < 1) {
      return 'Just now';
    } else if (difference.inMinutes < 60) {
      return '${difference.inMinutes}m ago';
    } else if (difference.inHours < 24) {
      return '${difference.inHours}h ago';
    } else {
      return '${difference.inDays}d ago';
    }
  }
}

// ============================================================================
// ENHANCED STAT CARD WIDGET
// ============================================================================
class EnhancedStatCard extends StatelessWidget {
  final String title;
  final String value;
  final IconData icon;
  final Color color;

  const EnhancedStatCard({
    Key? key,
    required this.title,
    required this.value,
    required this.icon,
    required this.color,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [color.withOpacity(0.1), color.withOpacity(0.05)],
        ),
        borderRadius: BorderRadius.circular(AppConstants.borderRadius),
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
                gradient:
                    LinearGradient(colors: [color, color.withOpacity(0.8)]),
                shape: BoxShape.circle,
              ),
              child: Icon(icon,
                  size: AppConstants.largeIconSize, color: Colors.white),
            ),
            const SizedBox(height: 12),
            Text(value, style: AppTextStyles.heading3.copyWith(color: color)),
            const SizedBox(height: 4),
            Text(
              title,
              style:
                  AppTextStyles.caption.copyWith(fontWeight: FontWeight.w500),
              textAlign: TextAlign.center,
            ),
          ],
        ),
      ),
    );
  }
}

class RealTimeMetricCard extends StatelessWidget {
  final String title;
  final String value;
  final IconData icon;
  final Color color;
  final String suffix;

  const RealTimeMetricCard({
    Key? key,
    required this.title,
    required this.value,
    required this.icon,
    required this.color,
    required this.suffix,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [color.withOpacity(0.1), color.withOpacity(0.05)],
        ),
        borderRadius: BorderRadius.circular(AppConstants.borderRadius),
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
                gradient:
                    LinearGradient(colors: [color, color.withOpacity(0.8)]),
                shape: BoxShape.circle,
                boxShadow: [
                  BoxShadow(
                    color: color.withOpacity(0.3),
                    blurRadius: 6,
                    offset: const Offset(0, 2),
                  ),
                ],
              ),
              child:
                  Icon(icon, size: AppConstants.iconSize, color: Colors.white),
            ),
            const SizedBox(height: 12),
            Text(value, style: AppTextStyles.heading3.copyWith(color: color)),
            const SizedBox(height: 4),
            Text(
              title,
              style:
                  AppTextStyles.caption.copyWith(fontWeight: FontWeight.w500),
              textAlign: TextAlign.center,
            ),
            const SizedBox(height: 2),
            Text(
              suffix,
              style: AppTextStyles.overline.copyWith(color: color),
            ),
          ],
        ),
      ),
    );
  }
}

class CardHeader extends StatelessWidget {
  final String title;
  final IconData icon;
  final Color color;

  const CardHeader({
    Key? key,
    required this.title,
    required this.icon,
    required this.color,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        EnhancedIconContainer(
          icon: icon,
          gradient: LinearGradient(colors: [color, color.withOpacity(0.8)]),
        ),
        const SizedBox(width: 12),
        Text(title, style: AppTextStyles.heading3),
      ],
    );
  }
}

class MetricItem extends StatelessWidget {
  final String label;
  final String value;
  final IconData icon;
  final Color color;

  const MetricItem({
    Key? key,
    required this.label,
    required this.value,
    required this.icon,
    required this.color,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: AppColors.cardBackground,
        borderRadius: BorderRadius.circular(AppConstants.smallBorderRadius + 4),
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
          Icon(icon, color: color, size: AppConstants.iconSize),
          const SizedBox(height: 8),
          Text(
            value,
            style: AppTextStyles.body
                .copyWith(fontWeight: FontWeight.bold, color: color),
          ),
          const SizedBox(height: 4),
          Text(
            label,
            style: AppTextStyles.overline.copyWith(color: Colors.grey.shade600),
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }
}

class OrdersHeader extends StatelessWidget {
  final int totalOrders;

  const OrdersHeader({Key? key, required this.totalOrders}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            AppColors.warning.withOpacity(0.1),
            AppColors.warning.withOpacity(0.2)
          ],
        ),
        borderRadius: BorderRadius.circular(AppConstants.smallBorderRadius + 4),
        border: Border.all(color: AppColors.warning.withOpacity(0.3)),
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: AppColors.warning,
              borderRadius:
                  BorderRadius.circular(AppConstants.smallBorderRadius + 4),
            ),
            child: Icon(Icons.list_alt,
                color: Colors.white, size: AppConstants.largeIconSize),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('Order History', style: AppTextStyles.heading3),
                Text(
                  'Total: $totalOrders orders completed',
                  style:
                      AppTextStyles.body.copyWith(color: Colors.grey.shade600),
                ),
              ],
            ),
          ),
          if (totalOrders > 0)
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              decoration: BoxDecoration(
                color: AppColors.warning,
                borderRadius: BorderRadius.circular(20),
              ),
              child: Text(
                '$totalOrders',
                style: const TextStyle(
                    color: Colors.white, fontWeight: FontWeight.bold),
              ),
            ),
        ],
      ),
    );
  }
}

class OrderHistoryCard extends StatelessWidget {
  final Map<String, dynamic> order;

  const OrderHistoryCard({Key? key, required this.order}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final completedAt = order['completedAt'] as DateTime;
    final duration = order['duration'] as double;
    final distance = order['distance'] as double;

    Color statusColor = AppColors.success;
    if (duration > 60) {
      statusColor = AppColors.error;
    } else if (duration > 30) {
      statusColor = AppColors.warning;
    }

    return Container(
      margin: const EdgeInsets.symmetric(vertical: 4),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [AppColors.cardBackground, Colors.grey.shade50],
        ),
        borderRadius: BorderRadius.circular(AppConstants.smallBorderRadius + 4),
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
                colors: [statusColor, statusColor.withOpacity(0.8)]),
            shape: BoxShape.circle,
          ),
          child: const Icon(Icons.check, color: Colors.white),
        ),
        title: Text(order['name'], style: AppTextStyles.subtitle),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Completed: ${FormatUtils.formatDateTime(completedAt)}'),
            Text('Distance: ${distance.toStringAsFixed(1)} m'),
            Text('Waypoints: ${order['waypoints']}'),
          ],
        ),
        trailing: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              '${duration.toStringAsFixed(0)}m',
              style: AppTextStyles.subtitle.copyWith(color: statusColor),
            ),
            const SizedBox(height: 4),
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
              decoration: BoxDecoration(
                color: statusColor.withOpacity(0.1),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Text(
                duration < 30
                    ? 'FAST'
                    : duration < 60
                        ? 'NORMAL'
                        : 'SLOW',
                style: AppTextStyles.overline.copyWith(color: statusColor),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class NavigationHeader extends StatelessWidget {
  final int activeNavigations;

  const NavigationHeader({Key? key, required this.activeNavigations})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.teal.withOpacity(0.1), Colors.teal.withOpacity(0.2)],
        ),
        borderRadius: BorderRadius.circular(AppConstants.smallBorderRadius + 4),
        border: Border.all(color: Colors.teal.withOpacity(0.3)),
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Colors.teal,
              borderRadius:
                  BorderRadius.circular(AppConstants.smallBorderRadius + 4),
            ),
            child: Icon(Icons.navigation,
                color: Colors.white, size: AppConstants.largeIconSize),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('Navigation Analytics', style: AppTextStyles.heading3),
                Text(
                  'Active navigations: $activeNavigations',
                  style:
                      AppTextStyles.body.copyWith(color: Colors.grey.shade600),
                ),
              ],
            ),
          ),
          if (activeNavigations > 0)
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              decoration: BoxDecoration(
                color: AppColors.success,
                borderRadius: BorderRadius.circular(20),
              ),
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Container(
                    width: 8,
                    height: 8,
                    decoration: const BoxDecoration(
                      color: Colors.white,
                      shape: BoxShape.circle,
                    ),
                  ),
                  const SizedBox(width: 4),
                  Text(
                    'ACTIVE',
                    style: AppTextStyles.overline.copyWith(color: Colors.white),
                  ),
                ],
              ),
            ),
        ],
      ),
    );
  }
}

class NavigationStatusCard extends StatelessWidget {
  final String deviceId;
  final Map<String, dynamic> status;

  const NavigationStatusCard({
    Key? key,
    required this.deviceId,
    required this.status,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final timeRemaining = status['estimated_time_remaining'] as double;
    final distanceRemaining = status['distance_remaining'] as double;
    final recoveries = status['number_of_recoveries'] as int;
    final speed = status['speed'] as double;
    final navigationTime = status['navigation_time'] as double;

    return EnhancedCard(
      gradient: LinearGradient(
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
        colors: [AppColors.info.withOpacity(0.1), AppColors.cardBackground],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                padding: const EdgeInsets.all(8),
                decoration: BoxDecoration(
                  color: AppColors.info,
                  borderRadius:
                      BorderRadius.circular(AppConstants.smallBorderRadius),
                ),
                child: const Icon(Icons.my_location,
                    color: Colors.white, size: 16),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text(deviceId, style: AppTextStyles.subtitle),
              ),
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                decoration: BoxDecoration(
                  color: AppColors.success,
                  borderRadius:
                      BorderRadius.circular(AppConstants.smallBorderRadius),
                ),
                child: Text(
                  'NAVIGATING',
                  style: AppTextStyles.overline.copyWith(color: Colors.white),
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          GridView.count(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            crossAxisCount: 2,
            crossAxisSpacing: 8,
            mainAxisSpacing: 8,
            childAspectRatio: 2.5,
            children: [
              NavigationMetricItem(
                label: 'ETA',
                value: '${timeRemaining.toStringAsFixed(0)}s',
                icon: Icons.timer,
                color: AppColors.success,
              ),
              NavigationMetricItem(
                label: 'Distance',
                value: '${distanceRemaining.toStringAsFixed(1)}m',
                icon: Icons.straighten,
                color: AppColors.info,
              ),
              NavigationMetricItem(
                label: 'Speed',
                value: '${speed.toStringAsFixed(2)} m/s',
                icon: Icons.speed,
                color: AppColors.warning,
              ),
              NavigationMetricItem(
                label: 'Recoveries',
                value: recoveries.toString(),
                icon: Icons.refresh,
                color: recoveries > 0 ? AppColors.error : Colors.grey,
              ),
            ],
          ),
          const SizedBox(height: 12),
          Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Text('Progress',
                      style: AppTextStyles.caption
                          .copyWith(fontWeight: FontWeight.w500)),
                  Text(
                    'Time: ${navigationTime.toStringAsFixed(0)}s',
                    style: AppTextStyles.overline,
                  ),
                ],
              ),
              const SizedBox(height: 4),
              Container(
                height: 6,
                decoration: BoxDecoration(
                  color: Colors.grey.shade300,
                  borderRadius: BorderRadius.circular(3),
                ),
                child: FractionallySizedBox(
                  alignment: Alignment.centerLeft,
                  widthFactor: timeRemaining > 0
                      ? math.max(
                          0.0,
                          math.min(
                              1.0,
                              (navigationTime) /
                                  (navigationTime + timeRemaining)))
                      : 1.0,
                  child: Container(
                    decoration: BoxDecoration(
                      gradient: LinearGradient(colors: [
                        AppColors.success,
                        AppColors.success.withOpacity(0.8)
                      ]),
                      borderRadius: BorderRadius.circular(3),
                    ),
                  ),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

class NavigationMetricItem extends StatelessWidget {
  final String label;
  final String value;
  final IconData icon;
  final Color color;

  const NavigationMetricItem({
    Key? key,
    required this.label,
    required this.value,
    required this.icon,
    required this.color,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: AppColors.cardBackground,
        borderRadius: BorderRadius.circular(AppConstants.smallBorderRadius),
        border: Border.all(color: color.withOpacity(0.2)),
      ),
      padding: const EdgeInsets.all(8),
      child: Row(
        children: [
          Icon(icon, color: color, size: 16),
          const SizedBox(width: 6),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Text(
                  value,
                  style: AppTextStyles.caption
                      .copyWith(fontWeight: FontWeight.bold, color: color),
                ),
                Text(label,
                    style: AppTextStyles.overline.copyWith(fontSize: 9)),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class EventsHeader extends StatelessWidget {
  final int totalEvents;

  const EventsHeader({Key? key, required this.totalEvents}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            AppColors.error.withOpacity(0.1),
            AppColors.error.withOpacity(0.2)
          ],
        ),
        borderRadius: BorderRadius.circular(AppConstants.smallBorderRadius + 4),
        border: Border.all(color: AppColors.error.withOpacity(0.3)),
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: AppColors.error,
              borderRadius:
                  BorderRadius.circular(AppConstants.smallBorderRadius + 4),
            ),
            child: Icon(Icons.event,
                color: Colors.white, size: AppConstants.largeIconSize),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('System Events', style: AppTextStyles.heading3),
                Text(
                  'Total: $totalEvents events recorded',
                  style:
                      AppTextStyles.body.copyWith(color: Colors.grey.shade600),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class EventCard extends StatelessWidget {
  final Map<String, dynamic> event;

  const EventCard({Key? key, required this.event}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final type = event['type'];
    final severity = event['severity'];

    Color color;
    IconData icon;

    switch (type) {
      case 'error':
        color = AppColors.error;
        icon = Icons.error;
        break;
      case 'warning':
        color = AppColors.warning;
        icon = Icons.warning;
        break;
      case 'success':
        color = AppColors.success;
        icon = Icons.check_circle;
        break;
      default:
        color = AppColors.info;
        icon = Icons.info;
    }

    return Container(
      margin: const EdgeInsets.symmetric(vertical: 4),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [AppColors.cardBackground, color.withOpacity(0.02)],
        ),
        borderRadius: BorderRadius.circular(AppConstants.smallBorderRadius + 4),
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
            gradient: LinearGradient(colors: [color, color.withOpacity(0.8)]),
            shape: BoxShape.circle,
          ),
          child: Icon(icon, color: Colors.white),
        ),
        title: Text(event['message'], style: AppTextStyles.subtitle),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Device: ${event['deviceId']}'),
            Text(FormatUtils.formatDateTime(event['timestamp'] as DateTime)),
          ],
        ),
        trailing: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
              decoration: BoxDecoration(
                gradient:
                    LinearGradient(colors: [color, color.withOpacity(0.8)]),
                borderRadius:
                    BorderRadius.circular(AppConstants.smallBorderRadius + 4),
              ),
              child: Text(
                type.toUpperCase(),
                style: AppTextStyles.overline.copyWith(color: Colors.white),
              ),
            ),
            const SizedBox(height: 4),
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
              decoration: BoxDecoration(
                color: color.withOpacity(0.1),
                borderRadius:
                    BorderRadius.circular(AppConstants.smallBorderRadius),
              ),
              child: Text(
                severity.toUpperCase(),
                style:
                    AppTextStyles.overline.copyWith(fontSize: 8, color: color),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class EmptyStateCard extends StatelessWidget {
  final IconData icon;
  final String title;
  final String subtitle;

  const EmptyStateCard({
    Key? key,
    required this.icon,
    required this.title,
    required this.subtitle,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return EnhancedCard(
      gradient: LinearGradient(
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
        colors: [Colors.grey.shade50, AppColors.cardBackground],
      ),
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          children: [
            Icon(icon, size: 48, color: AppColors.primary),
            const SizedBox(height: 12),
            Text(title, style: AppTextStyles.heading3),
            const SizedBox(height: 4),
            Text(subtitle, style: AppTextStyles.body),
          ],
        ),
      ),
    );
  }
}

// Enhanced Card Widget
class EnhancedCard extends StatelessWidget {
  final Widget child;
  final LinearGradient? gradient;

  const EnhancedCard({
    Key? key,
    required this.child,
    this.gradient,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        gradient: gradient ??
            LinearGradient(
              begin: Alignment.topLeft,
              end: Alignment.bottomRight,
              colors: [AppColors.cardBackground, Colors.grey.shade50],
            ),
        borderRadius: BorderRadius.circular(AppConstants.borderRadius),
        boxShadow: [
          BoxShadow(
            color: Colors.grey.withOpacity(0.1),
            blurRadius: 8,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: child,
      ),
    );
  }
}

// Enhanced Icon Container
class EnhancedIconContainer extends StatelessWidget {
  final IconData icon;
  final LinearGradient gradient;

  const EnhancedIconContainer({
    Key? key,
    required this.icon,
    required this.gradient,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        gradient: gradient,
        borderRadius: BorderRadius.circular(AppConstants.smallBorderRadius),
      ),
      child: Icon(icon, color: Colors.white, size: AppConstants.iconSize),
    );
  }
}
