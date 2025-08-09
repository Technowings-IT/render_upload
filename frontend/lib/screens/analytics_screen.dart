// screens/enhanced_analytics_screen.dart - Responsive Modern Robotic Analytics
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'dart:math' as math;
import 'dart:async';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import '../services/theme_service.dart';
import '../widgets/modern_ui_components.dart';

// ============================================================================
// RESPONSIVE DESIGN SYSTEM (Enhanced for Multiple Device Sizes)
// ============================================================================
class ResponsiveConfig {
  // Improved breakpoints for specific device sizes
  static const double mobileMaxWidth = 428.0; // 6.7" mobile (iPhone 14 Pro Max)
  static const double tabletMaxWidth = 1024.0; // 8" tablet (iPad Mini)
  static const double desktopMinWidth = 1025.0; // Laptop and above

  // Dynamic sizing based on screen width
  static double getScaleFactor(double screenWidth) {
    if (screenWidth <= mobileMaxWidth) return 0.85;
    if (screenWidth <= tabletMaxWidth) return 0.95;
    return 1.0;
  }

  static double getCardPadding(double screenWidth) {
    if (screenWidth <= mobileMaxWidth) return 12.0;
    if (screenWidth <= tabletMaxWidth) return 16.0;
    return 20.0;
  }

  static double getGridSpacing(double screenWidth) {
    if (screenWidth <= mobileMaxWidth) return 8.0;
    if (screenWidth <= tabletMaxWidth) return 12.0;
    return 16.0;
  }

  static int getGridCrossAxisCount(double screenWidth, int defaultMobile,
      int defaultTablet, int defaultDesktop) {
    if (screenWidth <= mobileMaxWidth) return defaultMobile;
    if (screenWidth <= tabletMaxWidth) return defaultTablet;
    return defaultDesktop;
  }

  static double getChartHeight(double screenWidth) {
    if (screenWidth <= mobileMaxWidth) return 180.0;
    if (screenWidth <= tabletMaxWidth) return 220.0;
    return 250.0;
  }

  static double getFontScale(double screenWidth) {
    if (screenWidth <= mobileMaxWidth) return 0.9;
    if (screenWidth <= tabletMaxWidth) return 0.95;
    return 1.0;
  }
}

// ============================================================================
// CONSTANTS & DESIGN SYSTEM (Enhanced for Responsive Design)
// ============================================================================
class AppConstants {
  static double getBorderRadius(double screenWidth) {
    return screenWidth <= ResponsiveConfig.mobileMaxWidth ? 12.0 : 16.0;
  }

  static double getSmallBorderRadius(double screenWidth) {
    return screenWidth <= ResponsiveConfig.mobileMaxWidth ? 6.0 : 8.0;
  }

  static double getCardElevation(double screenWidth) {
    return screenWidth <= ResponsiveConfig.mobileMaxWidth ? 2.0 : 4.0;
  }

  static double getIconSize(double screenWidth) {
    return screenWidth <= ResponsiveConfig.mobileMaxWidth ? 18.0 : 20.0;
  }

  static double getLargeIconSize(double screenWidth) {
    return screenWidth <= ResponsiveConfig.mobileMaxWidth ? 20.0 : 24.0;
  }

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
    Color(0xFF2196F3),
    Color(0xFF4CAF50),
    Color(0xFFFF9800),
    Color(0xFF9C27B0),
    Color(0xFFF44336),
    Color(0xFF009688),
    Color(0xFF795548),
  ];
}

class AppTextStyles {
  static TextStyle getHeading1(double scaleFactor) => TextStyle(
        fontSize: 28 * scaleFactor,
        fontWeight: FontWeight.bold,
      );

  static TextStyle getHeading2(double scaleFactor) => TextStyle(
        fontSize: 22 * scaleFactor,
        fontWeight: FontWeight.bold,
      );

  static TextStyle getHeading3(double scaleFactor) => TextStyle(
        fontSize: 18 * scaleFactor,
        fontWeight: FontWeight.bold,
      );

  static TextStyle getSubtitle(double scaleFactor) => TextStyle(
        fontSize: 16 * scaleFactor,
        fontWeight: FontWeight.w500,
      );

  static TextStyle getBody(double scaleFactor) => TextStyle(
        fontSize: 14 * scaleFactor,
        fontWeight: FontWeight.normal,
      );

  static TextStyle getCaption(double scaleFactor) => TextStyle(
        fontSize: 12 * scaleFactor,
        fontWeight: FontWeight.normal,
      );

  static TextStyle getOverline(double scaleFactor) => TextStyle(
        fontSize: 10 * scaleFactor,
        fontWeight: FontWeight.w500,
        letterSpacing: 1.2,
      );
}

// ============================================================================
// MAIN ENHANCED ANALYTICS SCREEN WITH RESPONSIVE DESIGN
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

  // ✅ NEW: Enhanced Alert System
  List<Map<String, dynamic>> _activeAlerts = [];
  String _selectedAlertType = 'all';
  String _selectedAlertSeverity = 'all';
  Map<String, dynamic> _alertStatistics = {
    'total': 0,
    'critical': 0,
    'warning': 0,
    'info': 0,
    'connection': 0,
    'battery': 0,
    'order': 0,
    'system': 0,
  };

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
  // RESPONSIVE DEVICE TYPE & HANDLING
  // ============================================================================
  void _updateDeviceType() {
    final screenWidth = MediaQuery.of(context).size.width;
    setState(() {
      if (screenWidth > ResponsiveConfig.desktopMinWidth) {
        _deviceType = DeviceType.desktop;
      } else if (screenWidth > ResponsiveConfig.mobileMaxWidth) {
        _deviceType = DeviceType.tablet;
      } else {
        _deviceType = DeviceType.phone;
      }
    });
  }

  // ============================================================================
  // DATA METHODS (Keeping existing implementation)
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

  // [Keep all existing data update methods - they don't need changes for responsive design]
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

  // [Keep existing data loading methods...]
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

      _updateDeviceStatsWithCrossData();

      // ✅ NEW: Generate active alerts after loading data
      _generateActiveAlerts();

      _chartAnimationController.forward();
      _dataAnimationController.forward();
    } catch (e) {
      print('❌ Error loading analytics data: $e');
      _generateMockData();

      // ✅ NEW: Generate alerts even with mock data
      _generateActiveAlerts();
    }

    setState(() {
      _isLoading = false;
    });
  }

  void _updateDeviceStatsWithCrossData() {
    for (final device in _connectedDevices) {
      final deviceId = device['id'];
      final stats = _deviceStats[deviceId] ?? <String, dynamic>{};

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
  }

  // [Keep all existing data loading methods - no changes needed for responsive design]
  Future<void> _loadBatteryData() async {
    // Implementation stays the same...
  }

  Future<void> _loadOrderData() async {
    // Implementation stays the same...
  }

  Future<void> _loadDeviceStats() async {
    // Implementation stays the same...
  }

  Future<void> _loadSystemEvents() async {
    // Implementation stays the same...
  }

  Future<void> _loadPerformanceData() async {
    // Implementation stays the same...
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
  // RESPONSIVE BUILD METHODS
  // ============================================================================
  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);
    final screenWidth = MediaQuery.of(context).size.width;
    final screenHeight = MediaQuery.of(context).size.height;
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);

    return Scaffold(
      backgroundColor: Colors.transparent,
      appBar: _buildResponsiveAppBar(theme, screenWidth, scaleFactor),
      body: Container(
        decoration: BoxDecoration(gradient: theme.backgroundGradient),
        child: _isLoading
            ? ModernLoadingIndicator(
                message: 'Loading Analytics Data...',
                size: ResponsiveConfig.getChartHeight(screenWidth) * 0.3)
            : _buildResponsiveBody(screenWidth, screenHeight),
      ),
    );
  }

  PreferredSizeWidget _buildResponsiveAppBar(
      ThemeService theme, double screenWidth, double scaleFactor) {
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;
    final iconSize = AppConstants.getIconSize(screenWidth);

    return AppBar(
      title: ShaderMask(
        shaderCallback: (bounds) => theme.primaryGradient.createShader(bounds),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Container(
              padding: EdgeInsets.all(isCompact ? 6 : 8),
              decoration: BoxDecoration(
                gradient: theme.primaryGradient,
                borderRadius: BorderRadius.circular(12),
                boxShadow: theme.neonGlow,
              ),
              child: Icon(Icons.analytics, color: Colors.white, size: iconSize),
            ),
            SizedBox(width: isCompact ? 8 : 12),
            Flexible(
              child: Text(
                isCompact ? 'Analytics' : 'Fleet Analytics',
                style: theme.displayMedium.copyWith(
                  fontSize: (isCompact ? 18 : 20) * scaleFactor,
                  color: Colors.white,
                ),
                overflow: TextOverflow.ellipsis,
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
      actions: _buildResponsiveActions(theme, screenWidth, scaleFactor),
      bottom: _deviceType != DeviceType.phone
          ? TabBar(
              controller: _tabController,
              indicatorColor: theme.accentColor,
              labelColor: theme.accentColor,
              unselectedLabelColor: theme.isDarkMode
                  ? Colors.white.withOpacity(0.6)
                  : Colors.black.withOpacity(0.6),
              indicatorWeight: 3,
              isScrollable: _deviceType == DeviceType.tablet,
              labelStyle: TextStyle(fontSize: 12 * scaleFactor),
              tabs: _buildTabList(isCompact),
            )
          : null,
    );
  }

  List<Widget> _buildTabList(bool isCompact) {
    if (isCompact) {
      return [
        Tab(icon: Icon(Icons.dashboard, size: 18), text: 'Overview'),
        Tab(icon: Icon(Icons.battery_std, size: 18), text: 'Battery'),
        Tab(icon: Icon(Icons.list_alt, size: 18), text: 'Orders'),
        Tab(icon: Icon(Icons.navigation, size: 18), text: 'Nav'),
        Tab(icon: Icon(Icons.event, size: 18), text: 'Events'),
        Tab(icon: Icon(Icons.speed, size: 18), text: 'Live'),
      ];
    } else {
      return [
        Tab(icon: Icon(Icons.dashboard), text: 'Overview'),
        Tab(icon: Icon(Icons.battery_std), text: 'Performance'),
        Tab(icon: Icon(Icons.list_alt), text: 'Orders'),
        Tab(icon: Icon(Icons.navigation), text: 'Navigation'),
        Tab(icon: Icon(Icons.event), text: 'Events'),
        Tab(icon: Icon(Icons.speed), text: 'Real-time'),
      ];
    }
  }

  List<Widget> _buildResponsiveActions(
      ThemeService theme, double screenWidth, double scaleFactor) {
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;

    if (isCompact) {
      // Compact layout for mobile
      return [
        _buildCompactTimeRangeSelector(theme, scaleFactor),
        _buildCompactActionButtons(theme),
      ];
    } else {
      // Full layout for tablet and desktop
      return [
        _buildTimeRangeSelector(theme, scaleFactor),
        _buildDeviceSelector(theme, scaleFactor),
        _buildActionButtons(theme),
      ];
    }
  }

  Widget _buildCompactTimeRangeSelector(
      ThemeService theme, double scaleFactor) {
    return PopupMenuButton<String>(
      initialValue: _selectedTimeRange,
      icon: Container(
        padding: const EdgeInsets.all(8),
        decoration: BoxDecoration(
          gradient: theme.glassMorphism.gradient,
          borderRadius: BorderRadius.circular(12),
          border: theme.glassMorphism.border,
        ),
        child: Icon(Icons.schedule, color: theme.accentColor, size: 16),
      ),
      onSelected: (value) {
        setState(() {
          _selectedTimeRange = value;
        });
        _loadAnalyticsData();
      },
      itemBuilder: (context) => [
        PopupMenuItem(
            value: '1h',
            child: Text('Last Hour',
                style: TextStyle(fontSize: 12 * scaleFactor))),
        PopupMenuItem(
            value: '24h',
            child:
                Text('Last 24h', style: TextStyle(fontSize: 12 * scaleFactor))),
        PopupMenuItem(
            value: '7d',
            child: Text('Last 7 days',
                style: TextStyle(fontSize: 12 * scaleFactor))),
        PopupMenuItem(
            value: '30d',
            child: Text('Last 30 days',
                style: TextStyle(fontSize: 12 * scaleFactor))),
      ],
    );
  }

  Widget _buildTimeRangeSelector(ThemeService theme, double scaleFactor) {
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 8),
      padding: const EdgeInsets.symmetric(horizontal: 12),
      decoration: BoxDecoration(
        gradient: theme.glassMorphism.gradient,
        borderRadius: BorderRadius.circular(12),
        border: theme.glassMorphism.border,
      ),
      child: DropdownButton<String>(
        value: _selectedTimeRange,
        dropdownColor:
            theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
        style: theme.bodyMedium
            .copyWith(color: theme.accentColor, fontSize: 12 * scaleFactor),
        underline: Container(),
        icon: Icon(Icons.expand_more, color: theme.accentColor, size: 16),
        items: [
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

  Widget _buildDeviceSelector(ThemeService theme, double scaleFactor) {
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 8),
      padding: const EdgeInsets.symmetric(horizontal: 12),
      decoration: BoxDecoration(
        gradient: theme.glassMorphism.gradient,
        borderRadius: BorderRadius.circular(12),
        border: theme.glassMorphism.border,
      ),
      child: DropdownButton<String>(
        value: _selectedDeviceId,
        dropdownColor:
            theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
        style: theme.bodyMedium
            .copyWith(color: theme.accentColor, fontSize: 12 * scaleFactor),
        underline: Container(),
        icon: Icon(Icons.expand_more, color: theme.accentColor, size: 16),
        items: [
          DropdownMenuItem(value: 'all', child: Text('All Devices')),
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

  Widget _buildCompactActionButtons(ThemeService theme) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 2),
          decoration: BoxDecoration(
            gradient: _autoRefresh ? theme.primaryGradient : null,
            color: _autoRefresh ? null : Colors.grey.withOpacity(0.3),
            borderRadius: BorderRadius.circular(16),
            boxShadow: _autoRefresh ? theme.neonGlow : null,
          ),
          child: IconButton(
            icon: Icon(
              _autoRefresh ? Icons.sync : Icons.sync_disabled,
              color: Colors.white,
              size: 18,
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
          ),
        ),
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 2),
          decoration: BoxDecoration(
            gradient: theme.primaryGradient,
            borderRadius: BorderRadius.circular(16),
            boxShadow: theme.elevationSmall,
          ),
          child: IconButton(
            icon: Icon(Icons.refresh, color: Colors.white, size: 18),
            onPressed: _loadAnalyticsData,
          ),
        ),
      ],
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

  Widget _buildResponsiveBody(double screenWidth, double screenHeight) {
    switch (_deviceType) {
      case DeviceType.desktop:
        return _buildDesktopLayout(screenWidth, screenHeight);
      case DeviceType.tablet:
        return _buildTabletLayout(screenWidth, screenHeight);
      case DeviceType.phone:
        return _buildPhoneLayout(screenWidth, screenHeight);
    }
  }

  Widget _buildDesktopLayout(double screenWidth, double screenHeight) {
    return TabBarView(
      controller: _tabController,
      children: [
        _buildOverviewTab(screenWidth, screenHeight),
        _buildPerformanceTab(screenWidth, screenHeight),
        _buildOrdersTab(screenWidth, screenHeight),
        _buildNavigationTab(screenWidth, screenHeight),
        _buildEventsTab(screenWidth, screenHeight),
        _buildRealTimeTab(screenWidth, screenHeight),
      ],
    );
  }

  Widget _buildTabletLayout(double screenWidth, double screenHeight) {
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
            labelStyle: TextStyle(
                fontSize: 12 * ResponsiveConfig.getFontScale(screenWidth)),
            tabs: _buildTabList(false),
          ),
        ),
        Expanded(
          child: TabBarView(
            controller: _tabController,
            children: [
              _buildOverviewTab(screenWidth, screenHeight),
              _buildPerformanceTab(screenWidth, screenHeight),
              _buildOrdersTab(screenWidth, screenHeight),
              _buildNavigationTab(screenWidth, screenHeight),
              _buildEventsTab(screenWidth, screenHeight),
              _buildRealTimeTab(screenWidth, screenHeight),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildPhoneLayout(double screenWidth, double screenHeight) {
    return Column(
      children: [
        Expanded(
          child: TabBarView(
            controller: _tabController,
            children: [
              _buildOverviewTab(screenWidth, screenHeight),
              _buildPerformanceTab(screenWidth, screenHeight),
              _buildOrdersTab(screenWidth, screenHeight),
              _buildNavigationTab(screenWidth, screenHeight),
              _buildEventsTab(screenWidth, screenHeight),
              _buildRealTimeTab(screenWidth, screenHeight),
            ],
          ),
        ),
        Container(
          decoration: BoxDecoration(
            color: AppColors.cardBackground,
            border: Border(top: BorderSide(color: Colors.grey.shade300)),
            boxShadow: [
              BoxShadow(
                color: Colors.black.withOpacity(0.1),
                blurRadius: 4,
                offset: Offset(0, -2),
              ),
            ],
          ),
          child: SafeArea(
            child: TabBar(
              controller: _tabController,
              labelColor: Theme.of(context).primaryColor,
              unselectedLabelColor: Colors.grey,
              indicatorColor: Theme.of(context).primaryColor,
              labelStyle: TextStyle(fontSize: 9),
              tabs: _buildTabList(true),
            ),
          ),
        ),
      ],
    );
  }

  // ============================================================================
  // RESPONSIVE TAB CONTENT BUILDERS
  // ============================================================================
  Widget _buildOverviewTab(double screenWidth, double screenHeight) {
    final theme = Provider.of<ThemeService>(context);
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);
    final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);

    return SingleChildScrollView(
      padding: EdgeInsets.all(cardPadding),
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
                  child: _buildResponsiveKeyMetrics(
                      screenWidth, theme, scaleFactor),
                ),
              );
            },
          ),
          SizedBox(height: gridSpacing),

          // Fleet Health Overview
          AnimatedBuilder(
            animation: _dataAnimationController,
            builder: (context, child) {
              return Transform.translate(
                offset: Offset(0, 50 * (1 - _dataAnimationController.value)),
                child: Opacity(
                  opacity: _dataAnimationController.value,
                  child: _buildResponsiveFleetHealthOverview(
                      screenWidth, theme, scaleFactor),
                ),
              );
            },
          ),
          SizedBox(height: gridSpacing),

          // Performance Charts
          _buildResponsiveCharts(screenWidth, theme, gridSpacing),
        ],
      ),
    );
  }

  Widget _buildResponsiveKeyMetrics(
      double screenWidth, ThemeService theme, double scaleFactor) {
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

    final crossAxisCount =
        ResponsiveConfig.getGridCrossAxisCount(screenWidth, 2, 2, 4);
    final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);

    return GridView.count(
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      crossAxisCount: crossAxisCount,
      crossAxisSpacing: gridSpacing,
      mainAxisSpacing: gridSpacing,
      childAspectRatio:
          screenWidth <= ResponsiveConfig.mobileMaxWidth ? 1.1 : 1.2,
      children: [
        ResponsiveStatsCard(
          title: 'Fleet Size',
          value: totalDevices.toString(),
          subtitle: 'Total AMR units',
          icon: Icons.precision_manufacturing,
          color: theme.infoColor,
          trend: '+2',
          showTrend: true,
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
        ResponsiveStatsCard(
          title: 'Active Devices',
          value: activeDevices.toString(),
          subtitle: 'Currently online',
          icon: Icons.power_settings_new,
          color: theme.onlineColor,
          trend: '+1',
          showTrend: true,
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
        ResponsiveStatsCard(
          title: 'Total Orders',
          value: totalOrders.toString(),
          subtitle: 'Completed today',
          icon: Icons.assignment_turned_in,
          color: theme.warningColor,
          trend: '+12%',
          showTrend: true,
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
        ResponsiveStatsCard(
          title: 'Avg Uptime',
          value: '${averageUptime.toStringAsFixed(0)}h',
          subtitle: 'System reliability',
          icon: Icons.schedule,
          color: theme.accentColor,
          trend: '+5%',
          showTrend: true,
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
      ],
    );
  }

  Widget _buildResponsiveFleetHealthOverview(
      double screenWidth, ThemeService theme, double scaleFactor) {
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);

    return ModernGlassCard(
      showGlow: true,
      glowColor: theme.successColor,
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(
                      screenWidth <= ResponsiveConfig.mobileMaxWidth ? 8 : 12),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [
                        theme.successColor,
                        theme.successColor.withOpacity(0.8)
                      ],
                    ),
                    borderRadius: BorderRadius.circular(
                        AppConstants.getSmallBorderRadius(screenWidth)),
                    boxShadow: [
                      BoxShadow(
                        color: theme.successColor.withOpacity(0.3),
                        blurRadius: 8,
                        spreadRadius: 2,
                      ),
                    ],
                  ),
                  child: Icon(
                    Icons.health_and_safety,
                    color: Colors.white,
                    size: AppConstants.getLargeIconSize(screenWidth),
                  ),
                ),
                SizedBox(
                    width: screenWidth <= ResponsiveConfig.mobileMaxWidth
                        ? 12
                        : 16),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Fleet Health Overview',
                        style: AppTextStyles.getHeading3(scaleFactor).copyWith(
                            color: theme.isDarkMode
                                ? Colors.white
                                : Colors.black87),
                      ),
                      Text(
                        'Real-time system status monitoring',
                        style: AppTextStyles.getBody(scaleFactor).copyWith(
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
                        padding: EdgeInsets.symmetric(
                          horizontal:
                              screenWidth <= ResponsiveConfig.mobileMaxWidth
                                  ? 8
                                  : 12,
                          vertical:
                              screenWidth <= ResponsiveConfig.mobileMaxWidth
                                  ? 4
                                  : 6,
                        ),
                        decoration: BoxDecoration(
                          gradient: theme.primaryGradient,
                          borderRadius: BorderRadius.circular(
                              AppConstants.getSmallBorderRadius(screenWidth)),
                          boxShadow: theme.neonGlow,
                        ),
                        child: Text(
                          'LIVE',
                          style:
                              AppTextStyles.getOverline(scaleFactor).copyWith(
                            color: Colors.white,
                            fontWeight: FontWeight.bold,
                          ),
                        ),
                      ),
                    );
                  },
                ),
              ],
            ),
            SizedBox(
                height:
                    screenWidth <= ResponsiveConfig.mobileMaxWidth ? 16 : 20),
            if (_connectedDevices.isEmpty)
              Center(
                child: Column(
                  children: [
                    Icon(Icons.device_unknown,
                        size: 48 * scaleFactor, color: theme.errorColor),
                    SizedBox(height: 12),
                    Text(
                      'No devices connected',
                      style: AppTextStyles.getHeading3(scaleFactor).copyWith(
                          color:
                              theme.isDarkMode ? Colors.white : Colors.black87),
                    ),
                    Text(
                      'Connect AMR devices to see fleet health',
                      style: AppTextStyles.getBody(scaleFactor).copyWith(
                          color:
                              theme.isDarkMode ? Colors.white : Colors.black87),
                    ),
                  ],
                ),
              )
            else
              ..._connectedDevices.map((device) =>
                  _buildResponsiveDeviceHealthItem(
                      device, theme, screenWidth, scaleFactor)),
          ],
        ),
      ),
    );
  }

  Widget _buildResponsiveDeviceHealthItem(Map<String, dynamic> device,
      ThemeService theme, double screenWidth, double scaleFactor) {
    final deviceId = device['id'];
    final stats = _deviceStats[deviceId];
    final batteryLevel = stats?['currentBattery']?.toDouble() ?? 0.0;
    final isOnline = device['status'] == 'connected';
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;

    return Container(
      margin: EdgeInsets.only(bottom: isCompact ? 8 : 12),
      padding: EdgeInsets.all(isCompact ? 12 : 16),
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
        borderRadius: BorderRadius.circular(9),
        border: Border.all(
          color: isOnline
              ? theme.onlineColor.withOpacity(0.3)
              : theme.offlineColor.withOpacity(0.3),
        ),
        boxShadow: theme.elevationSmall,
      ),
      child: isCompact
          ? _buildCompactDeviceHealthItem(
              device, theme, batteryLevel, isOnline, scaleFactor)
          : _buildFullDeviceHealthItem(
              device, theme, batteryLevel, isOnline, scaleFactor),
    );
  }

  Widget _buildCompactDeviceHealthItem(
      Map<String, dynamic> device,
      ThemeService theme,
      double batteryLevel,
      bool isOnline,
      double scaleFactor) {
    final deviceId = device['id'];

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Container(
              padding: const EdgeInsets.all(6),
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: isOnline
                      ? [theme.onlineColor, theme.onlineColor.withOpacity(0.8)]
                      : [
                          theme.offlineColor,
                          theme.offlineColor.withOpacity(0.8)
                        ],
                ),
                shape: BoxShape.circle,
                boxShadow: [
                  BoxShadow(
                    color: (isOnline ? theme.onlineColor : theme.offlineColor)
                        .withOpacity(0.3),
                    blurRadius: 6,
                    spreadRadius: 1,
                  ),
                ],
              ),
              child: Icon(
                isOnline ? Icons.smart_toy : Icons.warning,
                color: Colors.white,
                size: 16,
              ),
            ),
            SizedBox(width: 12),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    device['name'] ?? deviceId,
                    style: AppTextStyles.getSubtitle(scaleFactor).copyWith(
                        color:
                            theme.isDarkMode ? Colors.white : Colors.black87),
                    overflow: TextOverflow.ellipsis,
                  ),
                  RoboticStatusIndicator(
                    status: device['status'] ?? 'offline',
                    label: (device['status'] ?? 'offline').toUpperCase(),
                    animated: isOnline,
                    size: 6,
                  ),
                ],
              ),
            ),
            Text(
              '${batteryLevel.toStringAsFixed(0)}%',
              style: AppTextStyles.getBody(scaleFactor).copyWith(
                fontWeight: FontWeight.bold,
                color: theme.isDarkMode ? Colors.white : Colors.black87,
              ),
            ),
          ],
        ),
        SizedBox(height: 8),
        Container(
          height: 6,
          decoration: BoxDecoration(
            color: theme.isDarkMode
                ? Colors.white.withOpacity(0.1)
                : Colors.grey.shade300,
            borderRadius: BorderRadius.circular(3),
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
                borderRadius: BorderRadius.circular(3),
                boxShadow: [
                  BoxShadow(
                    color: (batteryLevel > 30
                            ? theme.successColor
                            : batteryLevel > 15
                                ? theme.warningColor
                                : theme.errorColor)
                        .withOpacity(0.3),
                    blurRadius: 3,
                    spreadRadius: 1,
                  ),
                ],
              ),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildFullDeviceHealthItem(
      Map<String, dynamic> device,
      ThemeService theme,
      double batteryLevel,
      bool isOnline,
      double scaleFactor) {
    final deviceId = device['id'];

    return Row(
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
                style: AppTextStyles.getSubtitle(scaleFactor).copyWith(
                    color: theme.isDarkMode ? Colors.white : Colors.black87),
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
                  Text('Battery',
                      style: AppTextStyles.getCaption(scaleFactor).copyWith(
                          color: theme.isDarkMode
                              ? Colors.white
                              : Colors.black87)),
                  Text(
                    '${batteryLevel.toStringAsFixed(0)}%',
                    style: AppTextStyles.getCaption(scaleFactor).copyWith(
                      fontWeight: FontWeight.bold,
                      color: theme.isDarkMode ? Colors.white : Colors.black87,
                    ),
                  ),
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
    );
  }

  Widget _buildResponsiveCharts(
      double screenWidth, ThemeService theme, double gridSpacing) {
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;
    final chartHeight = ResponsiveConfig.getChartHeight(screenWidth);

    if (isCompact) {
      return Column(
        children: [
          _buildResponsiveBatteryChart(theme, chartHeight, screenWidth),
          SizedBox(height: gridSpacing),
          _buildResponsiveOrderTrendsChart(theme, chartHeight, screenWidth),
        ],
      );
    } else {
      return Row(
        children: [
          Expanded(
              child: _buildResponsiveBatteryChart(
                  theme, chartHeight, screenWidth)),
          SizedBox(width: gridSpacing),
          Expanded(
              child: _buildResponsiveOrderTrendsChart(
                  theme, chartHeight, screenWidth)),
        ],
      );
    }
  }

  Widget _buildResponsiveBatteryChart(
      ThemeService theme, double chartHeight, double screenWidth) {
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);

    return ModernGlassCard(
      showGlow: true,
      glowColor: theme.successColor,
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(
                      screenWidth <= ResponsiveConfig.mobileMaxWidth ? 6 : 8),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [
                        theme.successColor,
                        theme.successColor.withOpacity(0.8)
                      ],
                    ),
                    borderRadius: BorderRadius.circular(
                        AppConstants.getSmallBorderRadius(screenWidth)),
                  ),
                  child: Icon(
                    Icons.battery_charging_full,
                    color: Colors.white,
                    size: AppConstants.getIconSize(screenWidth),
                  ),
                ),
                SizedBox(
                    width: screenWidth <= ResponsiveConfig.mobileMaxWidth
                        ? 8
                        : 12),
                Expanded(
                  child: Text(
                    'Battery Levels (${_getTimeRangeLabel()})',
                    style: AppTextStyles.getHeading3(scaleFactor).copyWith(
                        color:
                            theme.isDarkMode ? Colors.white : Colors.black87),
                    overflow: TextOverflow.ellipsis,
                  ),
                ),
              ],
            ),
            SizedBox(height: cardPadding),
            Container(
              height: chartHeight,
              child: CustomPaint(
                size: Size.infinite,
                painter: ResponsiveBatteryChartPainter(
                  _batteryHistory,
                  _selectedDeviceId,
                  theme,
                  scaleFactor,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildResponsiveOrderTrendsChart(
      ThemeService theme, double chartHeight, double screenWidth) {
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);

    return ModernGlassCard(
      showGlow: true,
      glowColor: theme.warningColor,
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(
                      screenWidth <= ResponsiveConfig.mobileMaxWidth ? 6 : 8),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [
                        theme.warningColor,
                        theme.warningColor.withOpacity(0.8)
                      ],
                    ),
                    borderRadius: BorderRadius.circular(
                        AppConstants.getSmallBorderRadius(screenWidth)),
                  ),
                  child: Icon(
                    Icons.trending_up,
                    color: Colors.white,
                    size: AppConstants.getIconSize(screenWidth),
                  ),
                ),
                SizedBox(
                    width: screenWidth <= ResponsiveConfig.mobileMaxWidth
                        ? 8
                        : 12),
                Expanded(
                  child: Text(
                    'Order Trends (${_getTimeRangeLabel()})',
                    style: AppTextStyles.getHeading3(scaleFactor).copyWith(
                        color:
                            theme.isDarkMode ? Colors.white : Colors.black87),
                    overflow: TextOverflow.ellipsis,
                  ),
                ),
              ],
            ),
            SizedBox(height: cardPadding),
            Container(
              height: chartHeight,
              child: CustomPaint(
                size: Size.infinite,
                painter: ResponsiveOrderTrendsChartPainter(
                  _orderHistory,
                  _selectedDeviceId,
                  theme,
                  scaleFactor,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  // Continue with other tab builders using similar responsive patterns...
  Widget _buildPerformanceTab(double screenWidth, double screenHeight) {
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);
    final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);

    return SingleChildScrollView(
      padding: EdgeInsets.all(cardPadding),
      child: Column(
        children: [
          _buildResponsiveVelocityChart(screenWidth),
          SizedBox(height: gridSpacing),
          _buildResponsivePerformanceMetrics(screenWidth),
        ],
      ),
    );
  }

  Widget _buildResponsiveVelocityChart(double screenWidth) {
    final theme = Provider.of<ThemeService>(context);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final chartHeight = ResponsiveConfig.getChartHeight(screenWidth);
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);

    return ModernGlassCard(
      showGlow: true,
      glowColor: theme.infoColor,
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(
                      screenWidth <= ResponsiveConfig.mobileMaxWidth ? 6 : 8),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [
                        theme.infoColor,
                        theme.infoColor.withOpacity(0.8)
                      ],
                    ),
                    borderRadius: BorderRadius.circular(
                        AppConstants.getSmallBorderRadius(screenWidth)),
                  ),
                  child: Icon(
                    Icons.speed,
                    color: Colors.white,
                    size: AppConstants.getIconSize(screenWidth),
                  ),
                ),
                SizedBox(
                    width: screenWidth <= ResponsiveConfig.mobileMaxWidth
                        ? 8
                        : 12),
                Expanded(
                  child: Text(
                    'Velocity History (${_getTimeRangeLabel()})',
                    style: AppTextStyles.getHeading3(scaleFactor).copyWith(
                        color:
                            theme.isDarkMode ? Colors.white : Colors.black87),
                    overflow: TextOverflow.ellipsis,
                  ),
                ),
              ],
            ),
            SizedBox(height: cardPadding),
            Container(
              height: chartHeight,
              child: CustomPaint(
                size: Size.infinite,
                painter: ResponsiveVelocityChartPainter(
                  _velocityHistory,
                  _selectedDeviceId,
                  theme,
                  scaleFactor,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildResponsivePerformanceMetrics(double screenWidth) {
    if (_selectedDeviceId == 'all') {
      return ResponsiveEnhancedCard(
        screenWidth: screenWidth,
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [Colors.purple.withOpacity(0.1), AppColors.cardBackground],
        ),
        child: Column(
          children: [
            Icon(Icons.analytics,
                size: 48 * ResponsiveConfig.getScaleFactor(screenWidth),
                color: Colors.purple),
            SizedBox(height: 16),
            Text(
              'Select a specific device to view detailed performance metrics',
              style: AppTextStyles.getSubtitle(
                  ResponsiveConfig.getScaleFactor(screenWidth)),
              textAlign: TextAlign.center,
            ),
          ],
        ),
      );
    }

    final stats = _deviceStats[_selectedDeviceId];
    if (stats == null) {
      return ResponsiveEnhancedCard(
        screenWidth: screenWidth,
        child: Text(
          'No performance data available for this device',
          style: AppTextStyles.getSubtitle(
              ResponsiveConfig.getScaleFactor(screenWidth)),
          textAlign: TextAlign.center,
        ),
      );
    }

    return _buildResponsiveDevicePerformanceDetails(stats, screenWidth);
  }

  Widget _buildResponsiveDevicePerformanceDetails(
      Map<String, dynamic> stats, double screenWidth) {
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);
    final crossAxisCount =
        ResponsiveConfig.getGridCrossAxisCount(screenWidth, 2, 3, 3);

    return ResponsiveEnhancedCard(
      screenWidth: screenWidth,
      gradient: LinearGradient(
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
        colors: [Colors.indigo.withOpacity(0.1), AppColors.cardBackground],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          ResponsiveCardHeader(
            title: 'Performance Metrics',
            icon: Icons.assessment,
            color: Colors.indigo,
            screenWidth: screenWidth,
            scaleFactor: scaleFactor,
          ),
          SizedBox(height: gridSpacing),
          GridView.count(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            crossAxisCount: crossAxisCount,
            crossAxisSpacing: gridSpacing,
            mainAxisSpacing: gridSpacing,
            childAspectRatio:
                screenWidth <= ResponsiveConfig.mobileMaxWidth ? 1.2 : 1.5,
            children: [
              ResponsiveMetricItem(
                label: 'Total Orders',
                value: (stats['totalOrders'] ?? 0).toString(),
                icon: Icons.assignment,
                color: AppColors.info,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
              ResponsiveMetricItem(
                label: 'Completed',
                value: (stats['completedOrders'] ?? 0).toString(),
                icon: Icons.check_circle,
                color: AppColors.success,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
              ResponsiveMetricItem(
                label: 'Avg Time',
                value:
                    '${(stats['averageOrderTime'] as double? ?? 0.0).toStringAsFixed(1)} min',
                icon: Icons.timer,
                color: AppColors.warning,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
              ResponsiveMetricItem(
                label: 'Uptime',
                value:
                    '${(stats['totalUptime'] as double? ?? 0.0).toStringAsFixed(0)} h',
                icon: Icons.schedule,
                color: Colors.purple,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
              ResponsiveMetricItem(
                label: 'Distance',
                value:
                    '${(stats['totalDistance'] as double? ?? 0.0).toStringAsFixed(1)} km',
                icon: Icons.route,
                color: Colors.teal,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
              ResponsiveMetricItem(
                label: 'Avg Speed',
                value:
                    '${(stats['averageSpeed'] as double? ?? 0.0).toStringAsFixed(2)} m/s',
                icon: Icons.speed,
                color: AppColors.error,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildOrdersTab(double screenWidth, double screenHeight) {
    final relevantOrders = _selectedDeviceId == 'all'
        ? _orderHistory.values.expand((orders) => orders).toList()
        : _orderHistory[_selectedDeviceId] ?? [];

    relevantOrders.sort((a, b) =>
        (b['completedAt'] as DateTime).compareTo(a['completedAt'] as DateTime));

    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);
    final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);

    return SingleChildScrollView(
      padding: EdgeInsets.all(cardPadding),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          ResponsiveOrdersHeader(
              totalOrders: relevantOrders.length, screenWidth: screenWidth),
          SizedBox(height: gridSpacing),
          if (relevantOrders.isEmpty)
            ResponsiveEmptyStateCard(
              icon: Icons.inbox,
              title: 'No order history available',
              subtitle: 'Orders will appear here once completed',
              screenWidth: screenWidth,
            )
          else
            ListView.builder(
              shrinkWrap: true,
              physics: const NeverScrollableScrollPhysics(),
              itemCount: relevantOrders.length,
              itemBuilder: (context, index) {
                return ResponsiveOrderHistoryCard(
                  order: relevantOrders[index],
                  screenWidth: screenWidth,
                );
              },
            ),
        ],
      ),
    );
  }

  Widget _buildNavigationTab(double screenWidth, double screenHeight) {
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);
    final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);

    return SingleChildScrollView(
      padding: EdgeInsets.all(cardPadding),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          ResponsiveNavigationHeader(
            activeNavigations: _currentNavigationStatus.length,
            screenWidth: screenWidth,
          ),
          SizedBox(height: gridSpacing),
          _buildResponsiveCurrentNavigationStatus(screenWidth),
          SizedBox(height: gridSpacing),
          _buildResponsiveNavigationMetrics(screenWidth),
          SizedBox(height: gridSpacing),
          _buildResponsiveNavigationHistory(screenWidth),
        ],
      ),
    );
  }

  Widget _buildResponsiveCurrentNavigationStatus(double screenWidth) {
    if (_selectedDeviceId == 'all') {
      final crossAxisCount =
          ResponsiveConfig.getGridCrossAxisCount(screenWidth, 1, 2, 2);
      final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);

      return GridView.builder(
        shrinkWrap: true,
        physics: const NeverScrollableScrollPhysics(),
        gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
          crossAxisCount: crossAxisCount,
          crossAxisSpacing: gridSpacing,
          mainAxisSpacing: gridSpacing,
          childAspectRatio:
              screenWidth <= ResponsiveConfig.mobileMaxWidth ? 1.2 : 1.5,
        ),
        itemCount: _currentNavigationStatus.length,
        itemBuilder: (context, index) {
          final deviceId = _currentNavigationStatus.keys.elementAt(index);
          final status = _currentNavigationStatus[deviceId]!;
          return ResponsiveNavigationStatusCard(
            deviceId: deviceId,
            status: status,
            screenWidth: screenWidth,
          );
        },
      );
    } else if (_currentNavigationStatus.containsKey(_selectedDeviceId)) {
      return ResponsiveNavigationStatusCard(
        deviceId: _selectedDeviceId,
        status: _currentNavigationStatus[_selectedDeviceId]!,
        screenWidth: screenWidth,
      );
    } else {
      return ResponsiveEmptyStateCard(
        icon: Icons.navigation_outlined,
        title: 'No active navigation',
        subtitle:
            'Navigation data will appear here when robots are actively navigating',
        screenWidth: screenWidth,
      );
    }
  }

  Widget _buildResponsiveNavigationMetrics(double screenWidth) {
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);
    final crossAxisCount =
        ResponsiveConfig.getGridCrossAxisCount(screenWidth, 2, 4, 4);

    return ResponsiveEnhancedCard(
      screenWidth: screenWidth,
      gradient: LinearGradient(
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
        colors: [Colors.indigo.withOpacity(0.1), AppColors.cardBackground],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          ResponsiveCardHeader(
            title: 'Navigation Statistics',
            icon: Icons.analytics,
            color: Colors.indigo,
            screenWidth: screenWidth,
            scaleFactor: scaleFactor,
          ),
          SizedBox(height: gridSpacing),
          GridView.count(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            crossAxisCount: crossAxisCount,
            crossAxisSpacing: gridSpacing,
            mainAxisSpacing: gridSpacing,
            childAspectRatio:
                screenWidth <= ResponsiveConfig.mobileMaxWidth ? 1.0 : 1.2,
            children: [
              ResponsiveEnhancedStatCard(
                title: 'Avg Speed',
                value: _getAverageNavigationSpeed(),
                icon: Icons.speed,
                color: AppColors.info,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
              ResponsiveEnhancedStatCard(
                title: 'Total Recoveries',
                value: _getTotalRecoveries(),
                icon: Icons.refresh,
                color: AppColors.error,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
              ResponsiveEnhancedStatCard(
                title: 'Success Rate',
                value: _getNavigationSuccessRate(),
                icon: Icons.check_circle,
                color: AppColors.success,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
              ResponsiveEnhancedStatCard(
                title: 'Avg Time',
                value: _getAverageNavigationTime(),
                icon: Icons.timer,
                color: AppColors.warning,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildResponsiveNavigationHistory(double screenWidth) {
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final chartHeight = ResponsiveConfig.getChartHeight(screenWidth);

    return ResponsiveEnhancedCard(
      screenWidth: screenWidth,
      gradient: LinearGradient(
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
        colors: [Colors.purple.withOpacity(0.1), AppColors.cardBackground],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          ResponsiveCardHeader(
            title: 'Navigation Timeline',
            icon: Icons.history,
            color: Colors.purple,
            screenWidth: screenWidth,
            scaleFactor: scaleFactor,
          ),
          SizedBox(height: ResponsiveConfig.getGridSpacing(screenWidth)),
          Container(
            height: chartHeight,
            child: CustomPaint(
              size: Size.infinite,
              painter: ResponsiveNavigationTimelinePainter(
                _navigationHistory,
                _selectedDeviceId,
                scaleFactor,
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEventsTab(double screenWidth, double screenHeight) {
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);
    final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);

    return SingleChildScrollView(
      padding: EdgeInsets.all(cardPadding),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // ✅ Enhanced Events Header with Alert Statistics
          _buildEnhancedEventsHeader(screenWidth),
          SizedBox(height: gridSpacing),

          // ✅ Alert Filter Controls
          _buildAlertFilterControls(screenWidth),
          SizedBox(height: gridSpacing),

          // ✅ Active Alerts Section
          _buildActiveAlertsSection(screenWidth),
          SizedBox(height: gridSpacing),

          // ✅ System Events Section
          _buildSystemEventsSection(screenWidth),
        ],
      ),
    );
  }

  Widget _buildRealTimeTab(double screenWidth, double screenHeight) {
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);
    final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);

    return SingleChildScrollView(
      padding: EdgeInsets.all(cardPadding),
      child: Column(
        children: [
          _buildResponsiveRealTimeMetricsCards(screenWidth),
          SizedBox(height: gridSpacing),
          _buildResponsiveLiveDataStream(screenWidth),
        ],
      ),
    );
  }

  Widget _buildResponsiveRealTimeMetricsCards(double screenWidth) {
    final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);
    final crossAxisCount =
        ResponsiveConfig.getGridCrossAxisCount(screenWidth, 2, 3, 4);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);

    return GridView.count(
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      crossAxisCount: crossAxisCount,
      crossAxisSpacing: gridSpacing,
      mainAxisSpacing: gridSpacing,
      childAspectRatio:
          screenWidth <= ResponsiveConfig.mobileMaxWidth ? 0.9 : 1.1,
      children: [
        ResponsiveRealTimeMetricCard(
          title: 'Fleet Status',
          value:
              '${_connectedDevices.where((d) => d['status'] == 'connected').length}/${_connectedDevices.length}',
          icon: Icons.devices,
          color: AppColors.info,
          suffix: 'ONLINE',
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
        ResponsiveRealTimeMetricCard(
          title: 'Avg Battery',
          value: _getAverageBattery(),
          icon: Icons.battery_std,
          color: AppColors.success,
          suffix: '%',
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
        ResponsiveRealTimeMetricCard(
          title: 'Total Distance',
          value: _getTotalDistance(),
          icon: Icons.route,
          color: AppColors.primary,
          suffix: 'M',
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
        ResponsiveRealTimeMetricCard(
          title: 'Est. Time Left',
          value: _getEstimatedTimeLeft(),
          icon: Icons.access_time,
          color: AppColors.warning,
          suffix: 'MIN',
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
        ResponsiveRealTimeMetricCard(
          title: 'Nav Time',
          value: _getNavigationTime(),
          icon: Icons.navigation,
          color: Colors.indigo,
          suffix: 'MIN',
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
        ResponsiveRealTimeMetricCard(
          title: 'Device Recovery',
          value: _getDeviceRecoveryCount(),
          icon: Icons.healing,
          color: Colors.orange,
          suffix: 'EVENTS',
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
        ResponsiveRealTimeMetricCard(
          title: 'Active Orders',
          value: _getActiveOrdersCount(),
          icon: Icons.assignment,
          color: AppColors.warning,
          suffix: 'ORDERS',
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
        ResponsiveRealTimeMetricCard(
          title: 'System Load',
          value:
              '${(_realTimeMetrics['systemLoad'] ?? 45.0).toStringAsFixed(0)}%',
          icon: Icons.memory,
          color: Colors.purple,
          suffix: 'CPU',
          screenWidth: screenWidth,
          scaleFactor: scaleFactor,
        ),
      ],
    );
  }

  Widget _buildResponsiveLiveDataStream(double screenWidth) {
    final theme = Provider.of<ThemeService>(context);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);

    return ResponsiveEnhancedCard(
      screenWidth: screenWidth,
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
              ResponsiveEnhancedIconContainer(
                icon: Icons.stream,
                gradient:
                    LinearGradient(colors: [Colors.teal, Colors.teal.shade700]),
                screenWidth: screenWidth,
              ),
              SizedBox(
                  width:
                      screenWidth <= ResponsiveConfig.mobileMaxWidth ? 8 : 12),
              Expanded(
                child: Text(
                  'Live Data Stream',
                  style: AppTextStyles.getHeading3(scaleFactor).copyWith(
                      color: theme.isDarkMode ? Colors.white : Colors.black87),
                ),
              ),
              Container(
                width: 8 * scaleFactor,
                height: 8 * scaleFactor,
                decoration: const BoxDecoration(
                  color: AppColors.success,
                  shape: BoxShape.circle,
                ),
              ),
              SizedBox(width: 4),
              Text(
                'LIVE',
                style: AppTextStyles.getOverline(scaleFactor).copyWith(
                    color: theme.isDarkMode ? Colors.white : Colors.black87),
              ),
            ],
          ),
          SizedBox(height: ResponsiveConfig.getGridSpacing(screenWidth)),
          Text(
            'Real-time updates: ${_autoRefresh ? 'Enabled' : 'Disabled'}',
            style: AppTextStyles.getBody(scaleFactor).copyWith(
              color: theme.isDarkMode
                  ? Colors.white.withOpacity(0.6)
                  : Colors.grey.shade600,
            ),
          ),
          SizedBox(height: 8),
          Text(
            'Last update: ${_formatDateTime(DateTime.now())}',
            style: AppTextStyles.getCaption(scaleFactor).copyWith(
              color: theme.isDarkMode
                  ? Colors.white.withOpacity(0.5)
                  : Colors.grey.shade500,
            ),
          ),
        ],
      ),
    );
  }

  // ============================================================================
  // HELPER METHODS (Keep existing implementations)
  // ============================================================================
  String _getTotalDistance() {
    if (_selectedDeviceId == 'all') {
      final total = _deviceStats.values.fold(0.0,
          (sum, stats) => sum + (stats['totalDistance'] as double? ?? 0.0));
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

  // ============================================================================
  // ✅ ENHANCED ALERTS SYSTEM METHODS
  // ============================================================================

  Widget _buildEnhancedEventsHeader(double screenWidth) {
    final theme = Provider.of<ThemeService>(context);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);

    return ModernGlassCard(
      showGlow: true,
      glowColor: theme.warningColor,
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(
                      screenWidth <= ResponsiveConfig.mobileMaxWidth ? 8 : 12),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [
                        theme.warningColor,
                        theme.warningColor.withOpacity(0.8)
                      ],
                    ),
                    borderRadius: BorderRadius.circular(
                        AppConstants.getSmallBorderRadius(screenWidth)),
                    boxShadow: [
                      BoxShadow(
                        color: theme.warningColor.withOpacity(0.3),
                        blurRadius: 8,
                        spreadRadius: 2,
                      ),
                    ],
                  ),
                  child: Icon(
                    Icons.notification_important,
                    color: Colors.white,
                    size: AppConstants.getLargeIconSize(screenWidth),
                  ),
                ),
                SizedBox(
                    width: screenWidth <= ResponsiveConfig.mobileMaxWidth
                        ? 12
                        : 16),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'System Alerts & Events',
                        style: AppTextStyles.getHeading3(scaleFactor).copyWith(
                          color:
                              theme.isDarkMode ? Colors.white : Colors.black87,
                        ),
                      ),
                      Text(
                        'Real-time monitoring and alert management',
                        style: AppTextStyles.getBody(scaleFactor).copyWith(
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
                        padding: EdgeInsets.symmetric(
                          horizontal:
                              screenWidth <= ResponsiveConfig.mobileMaxWidth
                                  ? 8
                                  : 12,
                          vertical:
                              screenWidth <= ResponsiveConfig.mobileMaxWidth
                                  ? 4
                                  : 6,
                        ),
                        decoration: BoxDecoration(
                          gradient: theme.primaryGradient,
                          borderRadius: BorderRadius.circular(
                              AppConstants.getSmallBorderRadius(screenWidth)),
                        ),
                        child: Text(
                          'LIVE',
                          style:
                              AppTextStyles.getOverline(scaleFactor).copyWith(
                            color: Colors.white,
                            fontWeight: FontWeight.bold,
                          ),
                        ),
                      ),
                    );
                  },
                ),
              ],
            ),
            SizedBox(
                height:
                    screenWidth <= ResponsiveConfig.mobileMaxWidth ? 16 : 20),
            _buildAlertStatisticsRow(screenWidth, scaleFactor),
          ],
        ),
      ),
    );
  }

  Widget _buildAlertStatisticsRow(double screenWidth, double scaleFactor) {
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;

    if (isCompact) {
      return Column(
        children: [
          Row(
            children: [
              Expanded(
                  child: _buildAlertStatChip('Total', _alertStatistics['total'],
                      Colors.blue, scaleFactor)),
              SizedBox(width: 8),
              Expanded(
                  child: _buildAlertStatChip('Critical',
                      _alertStatistics['critical'], Colors.red, scaleFactor)),
            ],
          ),
          SizedBox(height: 8),
          Row(
            children: [
              Expanded(
                  child: _buildAlertStatChip('Warning',
                      _alertStatistics['warning'], Colors.orange, scaleFactor)),
              SizedBox(width: 8),
              Expanded(
                  child: _buildAlertStatChip('Info', _alertStatistics['info'],
                      Colors.green, scaleFactor)),
            ],
          ),
        ],
      );
    } else {
      return Row(
        children: [
          Expanded(
              child: _buildAlertStatChip('Total', _alertStatistics['total'],
                  Colors.blue, scaleFactor)),
          SizedBox(width: 12),
          Expanded(
              child: _buildAlertStatChip('Critical',
                  _alertStatistics['critical'], Colors.red, scaleFactor)),
          SizedBox(width: 12),
          Expanded(
              child: _buildAlertStatChip('Warning', _alertStatistics['warning'],
                  Colors.orange, scaleFactor)),
          SizedBox(width: 12),
          Expanded(
              child: _buildAlertStatChip(
                  'Info', _alertStatistics['info'], Colors.green, scaleFactor)),
        ],
      );
    }
  }

  Widget _buildAlertStatChip(
      String label, dynamic value, Color color, double scaleFactor) {
    return Container(
      padding: EdgeInsets.symmetric(vertical: 8, horizontal: 12),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: color.withOpacity(0.3)),
      ),
      child: Column(
        children: [
          Text(
            value.toString(),
            style: AppTextStyles.getHeading3(scaleFactor).copyWith(
              color: color,
              fontWeight: FontWeight.bold,
            ),
          ),
          Text(
            label,
            style: AppTextStyles.getCaption(scaleFactor).copyWith(
              color: color,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildAlertFilterControls(double screenWidth) {
    final theme = Provider.of<ThemeService>(context);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;

    return Container(
      padding: EdgeInsets.all(isCompact ? 12 : 16),
      decoration: BoxDecoration(
        color: theme.isDarkMode
            ? Colors.white.withOpacity(0.05)
            : Colors.black.withOpacity(0.05),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(
          color: theme.isDarkMode
              ? Colors.white.withOpacity(0.1)
              : Colors.black.withOpacity(0.1),
        ),
      ),
      child: isCompact
          ? _buildCompactFilters(theme, scaleFactor)
          : _buildFullFilters(theme, scaleFactor),
    );
  }

  Widget _buildCompactFilters(ThemeService theme, double scaleFactor) {
    return Column(
      children: [
        Row(
          children: [
            Icon(Icons.filter_list, size: 16, color: theme.accentColor),
            SizedBox(width: 8),
            Text(
              'Filters',
              style: AppTextStyles.getSubtitle(scaleFactor).copyWith(
                color: theme.isDarkMode ? Colors.white : Colors.black87,
              ),
            ),
          ],
        ),
        SizedBox(height: 12),
        Row(
          children: [
            Expanded(child: _buildAlertTypeDropdown(theme, scaleFactor)),
            SizedBox(width: 8),
            Expanded(child: _buildAlertSeverityDropdown(theme, scaleFactor)),
          ],
        ),
      ],
    );
  }

  Widget _buildFullFilters(ThemeService theme, double scaleFactor) {
    return Row(
      children: [
        Icon(Icons.filter_list, size: 20, color: theme.accentColor),
        SizedBox(width: 12),
        Text(
          'Filter Alerts:',
          style: AppTextStyles.getSubtitle(scaleFactor).copyWith(
            color: theme.isDarkMode ? Colors.white : Colors.black87,
          ),
        ),
        SizedBox(width: 16),
        Expanded(flex: 2, child: _buildAlertTypeDropdown(theme, scaleFactor)),
        SizedBox(width: 12),
        Expanded(
            flex: 2, child: _buildAlertSeverityDropdown(theme, scaleFactor)),
        Spacer(),
        ElevatedButton.icon(
          onPressed: () {
            setState(() {
              _selectedAlertType = 'all';
              _selectedAlertSeverity = 'all';
            });
            _generateActiveAlerts();
          },
          icon: Icon(Icons.clear, size: 16),
          label: Text('Clear'),
          style: ElevatedButton.styleFrom(
            backgroundColor: theme.errorColor.withOpacity(0.8),
            foregroundColor: Colors.white,
            padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
          ),
        ),
      ],
    );
  }

  Widget _buildAlertTypeDropdown(ThemeService theme, double scaleFactor) {
    return Container(
      padding: EdgeInsets.symmetric(horizontal: 8),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(6),
        border: Border.all(color: theme.accentColor.withOpacity(0.3)),
      ),
      child: DropdownButtonHideUnderline(
        child: DropdownButton<String>(
          value: _selectedAlertType,
          isExpanded: true,
          style: TextStyle(fontSize: 12 * scaleFactor, color: Colors.black),
          items: [
            DropdownMenuItem(value: 'all', child: Text('All Types')),
            DropdownMenuItem(value: 'connection', child: Text('Connection')),
            DropdownMenuItem(value: 'battery', child: Text('Battery')),
            DropdownMenuItem(value: 'order', child: Text('Orders')),
            DropdownMenuItem(value: 'system', child: Text('System')),
          ],
          onChanged: (value) {
            setState(() {
              _selectedAlertType = value!;
            });
            _generateActiveAlerts();
          },
        ),
      ),
    );
  }

  Widget _buildAlertSeverityDropdown(ThemeService theme, double scaleFactor) {
    return Container(
      padding: EdgeInsets.symmetric(horizontal: 8),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(6),
        border: Border.all(color: theme.accentColor.withOpacity(0.3)),
      ),
      child: DropdownButtonHideUnderline(
        child: DropdownButton<String>(
          value: _selectedAlertSeverity,
          isExpanded: true,
          style: TextStyle(fontSize: 12 * scaleFactor, color: Colors.black),
          items: [
            DropdownMenuItem(value: 'all', child: Text('All Levels')),
            DropdownMenuItem(value: 'critical', child: Text('Critical')),
            DropdownMenuItem(value: 'warning', child: Text('Warning')),
            DropdownMenuItem(value: 'info', child: Text('Info')),
          ],
          onChanged: (value) {
            setState(() {
              _selectedAlertSeverity = value!;
            });
            _generateActiveAlerts();
          },
        ),
      ),
    );
  }

  Widget _buildActiveAlertsSection(double screenWidth) {
    final theme = Provider.of<ThemeService>(context);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);

    var filteredAlerts = _activeAlerts.where((alert) {
      final typeMatch =
          _selectedAlertType == 'all' || alert['type'] == _selectedAlertType;
      final severityMatch = _selectedAlertSeverity == 'all' ||
          alert['severity'] == _selectedAlertSeverity;
      return typeMatch && severityMatch;
    }).toList();

    return ModernGlassCard(
      showGlow: true,
      glowColor: theme.errorColor,
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(
                  Icons.priority_high,
                  color: theme.errorColor,
                  size: AppConstants.getLargeIconSize(screenWidth),
                ),
                SizedBox(width: 12),
                Expanded(
                  child: Text(
                    'Active Alerts (${filteredAlerts.length})',
                    style: AppTextStyles.getHeading3(scaleFactor).copyWith(
                      color: theme.isDarkMode ? Colors.white : Colors.black87,
                    ),
                  ),
                ),
                if (filteredAlerts.isNotEmpty)
                  ElevatedButton.icon(
                    onPressed: _clearAllAlerts,
                    icon: Icon(Icons.clear_all, size: 16),
                    label: Text('Clear All'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: theme.errorColor,
                      foregroundColor: Colors.white,
                      padding:
                          EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                    ),
                  ),
              ],
            ),
            SizedBox(height: cardPadding),
            if (filteredAlerts.isEmpty)
              _buildNoAlertsWidget(screenWidth, scaleFactor)
            else
              Column(
                children: filteredAlerts
                    .take(10) // Limit to 10 most recent alerts
                    .map((alert) =>
                        _buildActiveAlertCard(alert, screenWidth, scaleFactor))
                    .toList(),
              ),
          ],
        ),
      ),
    );
  }

  Widget _buildNoAlertsWidget(double screenWidth, double scaleFactor) {
    return Container(
      padding: EdgeInsets.all(ResponsiveConfig.getCardPadding(screenWidth)),
      child: Column(
        children: [
          Icon(
            Icons.check_circle_outline,
            size: 48 * scaleFactor,
            color: AppColors.success,
          ),
          SizedBox(height: 12),
          Text(
            'All Systems Normal',
            style: AppTextStyles.getHeading3(scaleFactor).copyWith(
              color: AppColors.success,
            ),
          ),
          Text(
            'No active alerts at this time',
            style: AppTextStyles.getBody(scaleFactor).copyWith(
              color: Colors.grey,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildActiveAlertCard(
      Map<String, dynamic> alert, double screenWidth, double scaleFactor) {
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;
    final severity = alert['severity'] ?? 'info';
    final type = alert['type'] ?? 'system';
    final color = _getAlertColor(severity);

    return Container(
      margin: EdgeInsets.only(bottom: 8),
      padding: EdgeInsets.all(isCompact ? 12 : 16),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: color.withOpacity(0.3)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                padding: EdgeInsets.all(6),
                decoration: BoxDecoration(
                  color: color,
                  shape: BoxShape.circle,
                ),
                child: Icon(
                  _getAlertIcon(type),
                  color: Colors.white,
                  size: 16,
                ),
              ),
              SizedBox(width: 12),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      alert['title'] ?? 'Alert',
                      style: AppTextStyles.getSubtitle(scaleFactor).copyWith(
                        color: color,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                    Text(
                      alert['message'] ?? 'No details available',
                      style: AppTextStyles.getBody(scaleFactor),
                    ),
                  ],
                ),
              ),
              Container(
                padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                decoration: BoxDecoration(
                  color: color.withOpacity(0.2),
                  borderRadius: BorderRadius.circular(4),
                ),
                child: Text(
                  severity.toUpperCase(),
                  style: AppTextStyles.getOverline(scaleFactor).copyWith(
                    color: color,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ),
            ],
          ),
          SizedBox(height: 8),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(
                'Device: ${alert['deviceId'] ?? 'System'}',
                style: AppTextStyles.getCaption(scaleFactor).copyWith(
                  color: Colors.grey,
                ),
              ),
              Text(
                alert['timestamp'] ?? DateTime.now().toString(),
                style: AppTextStyles.getCaption(scaleFactor).copyWith(
                  color: Colors.grey,
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildSystemEventsSection(double screenWidth) {
    final theme = Provider.of<ThemeService>(context);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);

    return ModernGlassCard(
      showGlow: true,
      glowColor: theme.infoColor,
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(
                  Icons.history,
                  color: theme.infoColor,
                  size: AppConstants.getLargeIconSize(screenWidth),
                ),
                SizedBox(width: 12),
                Expanded(
                  child: Text(
                    'System Events (${_systemEvents.length})',
                    style: AppTextStyles.getHeading3(scaleFactor).copyWith(
                      color: theme.isDarkMode ? Colors.white : Colors.black87,
                    ),
                  ),
                ),
                ElevatedButton.icon(
                  onPressed: _loadSystemEvents,
                  icon: Icon(Icons.refresh, size: 16),
                  label: Text('Refresh'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: theme.infoColor,
                    foregroundColor: Colors.white,
                    padding: EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                  ),
                ),
              ],
            ),
            SizedBox(height: cardPadding),
            if (_systemEvents.isEmpty)
              _buildNoEventsWidget(screenWidth, scaleFactor)
            else
              Column(
                children: _systemEvents
                    .take(15) // Limit to 15 most recent events
                    .map((event) => ResponsiveEventCard(
                          event: event,
                          screenWidth: screenWidth,
                        ))
                    .toList(),
              ),
          ],
        ),
      ),
    );
  }

  Widget _buildNoEventsWidget(double screenWidth, double scaleFactor) {
    return Container(
      padding: EdgeInsets.all(ResponsiveConfig.getCardPadding(screenWidth)),
      child: Column(
        children: [
          Icon(
            Icons.event_note,
            size: 48 * scaleFactor,
            color: Colors.grey,
          ),
          SizedBox(height: 12),
          Text(
            'No Events Recorded',
            style: AppTextStyles.getHeading3(scaleFactor).copyWith(
              color: Colors.grey,
            ),
          ),
          Text(
            'System events will appear here as they occur',
            style: AppTextStyles.getBody(scaleFactor).copyWith(
              color: Colors.grey,
            ),
          ),
        ],
      ),
    );
  }

  Color _getAlertColor(String severity) {
    switch (severity.toLowerCase()) {
      case 'critical':
        return Colors.red;
      case 'warning':
        return Colors.orange;
      case 'info':
        return Colors.blue;
      default:
        return Colors.grey;
    }
  }

  IconData _getAlertIcon(String type) {
    switch (type.toLowerCase()) {
      case 'connection':
        return Icons.wifi_off;
      case 'battery':
        return Icons.battery_alert;
      case 'order':
        return Icons.assignment_late;
      case 'system':
        return Icons.error_outline;
      default:
        return Icons.warning;
    }
  }

  void _generateActiveAlerts() {
    _activeAlerts.clear();

    // Generate connection alerts
    for (final device in _connectedDevices) {
      if (device['status'] != 'connected') {
        _activeAlerts.add({
          'id': 'conn_${device['id']}',
          'type': 'connection',
          'severity': 'critical',
          'title': 'Device Disconnected',
          'message':
              'Device ${device['name'] ?? device['id']} is not connected',
          'deviceId': device['id'],
          'timestamp': DateTime.now().toString(),
        });
      }
    }

    // Generate battery alerts
    _deviceStats.forEach((deviceId, stats) {
      final batteryLevel = stats['currentBattery']?.toDouble() ?? 100.0;
      if (batteryLevel < 20) {
        _activeAlerts.add({
          'id': 'bat_$deviceId',
          'type': 'battery',
          'severity': batteryLevel < 10 ? 'critical' : 'warning',
          'title': 'Low Battery Alert',
          'message':
              'Device $deviceId battery at ${batteryLevel.toStringAsFixed(0)}%',
          'deviceId': deviceId,
          'timestamp': DateTime.now().toString(),
        });
      }
    });

    // Generate order alerts from system events
    for (final event in _systemEvents) {
      if (event['type'] == 'error' && event['message'].contains('order')) {
        _activeAlerts.add({
          'id': 'order_${event['timestamp']}',
          'type': 'order',
          'severity': 'warning',
          'title': 'Order Issue',
          'message': event['message'],
          'deviceId': event['deviceId'],
          'timestamp': event['timestamp'],
        });
      }
    }

    // Update statistics
    _updateAlertStatistics();

    setState(() {});
  }

  void _updateAlertStatistics() {
    _alertStatistics = {
      'total': _activeAlerts.length,
      'critical':
          _activeAlerts.where((a) => a['severity'] == 'critical').length,
      'warning': _activeAlerts.where((a) => a['severity'] == 'warning').length,
      'info': _activeAlerts.where((a) => a['severity'] == 'info').length,
      'connection':
          _activeAlerts.where((a) => a['type'] == 'connection').length,
      'battery': _activeAlerts.where((a) => a['type'] == 'battery').length,
      'order': _activeAlerts.where((a) => a['type'] == 'order').length,
      'system': _activeAlerts.where((a) => a['type'] == 'system').length,
    };
  }

  void _clearAllAlerts() {
    setState(() {
      _activeAlerts.clear();
      _updateAlertStatistics();
    });
  }
}

// ============================================================================
// RESPONSIVE CUSTOM PAINTERS FOR CHARTS
// ============================================================================
class ResponsiveBatteryChartPainter extends CustomPainter {
  final Map<String, List<Map<String, dynamic>>> batteryHistory;
  final String selectedDeviceId;
  final ThemeService theme;
  final double scaleFactor;

  ResponsiveBatteryChartPainter(
      this.batteryHistory, this.selectedDeviceId, this.theme, this.scaleFactor);

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
          ..strokeWidth = 3 * scaleFactor
          ..style = PaintingStyle.stroke;
        _drawBatteryLine(canvas, size, data, paint);
        colorIndex++;
      });
    } else if (batteryHistory.containsKey(selectedDeviceId)) {
      final paint = Paint()
        ..color = theme.successColor
        ..strokeWidth = 3 * scaleFactor
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
      ..strokeWidth = 1 * scaleFactor;

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
      ..strokeWidth = 2 * scaleFactor;

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

      final glowPaint = Paint()
        ..color = paint.color.withOpacity(0.6)
        ..maskFilter = MaskFilter.blur(BlurStyle.normal, 3 * scaleFactor);
      canvas.drawCircle(Offset(x, y), 4 * scaleFactor, glowPaint);

      final pointPaint = Paint()
        ..color = paint.color
        ..style = PaintingStyle.fill;
      canvas.drawCircle(Offset(x, y), 3 * scaleFactor, pointPaint);
    }

    final glowLinePaint = Paint()
      ..color = paint.color.withOpacity(0.3)
      ..strokeWidth = 6 * scaleFactor
      ..style = PaintingStyle.stroke
      ..maskFilter = MaskFilter.blur(BlurStyle.normal, 2 * scaleFactor);
    canvas.drawPath(path, glowLinePaint);

    canvas.drawPath(path, paint);
  }

  void _drawLabels(Canvas canvas, Size size) {
    final textPainter = TextPainter(
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.center,
    );

    for (int i = 0; i <= 5; i++) {
      final percentage = 100 - (i * 20);
      final y = size.height * 0.05 + (size.height * 0.9) * (i / 5);

      textPainter.text = TextSpan(
        text: '$percentage%',
        style: AppTextStyles.getCaption(scaleFactor)
            .copyWith(color: theme.accentColor),
      );
      textPainter.layout();
      textPainter.paint(
          canvas, Offset(size.width * 0.02, y - textPainter.height / 2));
    }

    for (int i = 0; i <= 6; i++) {
      final hours = 24 - (i * 4);
      final x = size.width * 0.1 + (size.width * 0.85) * (i / 6);

      textPainter.text = TextSpan(
        text: '${hours}h',
        style: AppTextStyles.getCaption(scaleFactor)
            .copyWith(color: theme.accentColor),
      );
      textPainter.layout();
      textPainter.paint(
          canvas, Offset(x - textPainter.width / 2, size.height * 0.97));
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class ResponsiveOrderTrendsChartPainter extends CustomPainter {
  final Map<String, List<Map<String, dynamic>>> orderHistory;
  final String selectedDeviceId;
  final ThemeService theme;
  final double scaleFactor;

  ResponsiveOrderTrendsChartPainter(
      this.orderHistory, this.selectedDeviceId, this.theme, this.scaleFactor);

  @override
  void paint(Canvas canvas, Size size) {
    _drawGrid(canvas, size);
    _drawAxes(canvas, size);

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
      ..strokeWidth = 1 * scaleFactor;

    for (int i = 0; i <= 5; i++) {
      final y = size.height * 0.9 * (i / 5) + size.height * 0.05;
      canvas.drawLine(
          Offset(size.width * 0.1, y), Offset(size.width * 0.95, y), gridPaint);
    }
  }

  void _drawAxes(Canvas canvas, Size size) {
    final axisPaint = Paint()
      ..color = theme.accentColor
      ..strokeWidth = 2 * scaleFactor;

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

      final glowPaint = Paint()
        ..color = theme.warningColor.withOpacity(0.4)
        ..maskFilter = MaskFilter.blur(BlurStyle.normal, 3 * scaleFactor);

      final glowRect = RRect.fromRectAndRadius(
        Rect.fromLTWH(x, y, barWidth * 0.8, barHeight),
        Radius.circular(4 * scaleFactor),
      );
      canvas.drawRRect(glowRect, glowPaint);

      final barPaint = Paint()
        ..shader = LinearGradient(
          colors: [theme.warningColor, theme.warningColor.withOpacity(0.8)],
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
        ).createShader(Rect.fromLTWH(x, y, barWidth * 0.8, barHeight));

      final barRect = RRect.fromRectAndRadius(
        Rect.fromLTWH(x, y, barWidth * 0.8, barHeight),
        Radius.circular(4 * scaleFactor),
      );
      canvas.drawRRect(barRect, barPaint);
    }
  }

  void _drawLabels(Canvas canvas, Size size) {
    final textPainter = TextPainter(
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.center,
    );

    for (int i = 0; i < 24; i += 4) {
      final x = size.width * 0.1 + (size.width * 0.85) * (i / 24);
      textPainter.text = TextSpan(
        text: '${i}h',
        style: AppTextStyles.getCaption(scaleFactor)
            .copyWith(color: theme.accentColor),
      );
      textPainter.layout();
      textPainter.paint(
          canvas, Offset(x - textPainter.width / 2, size.height * 0.97));
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class ResponsiveVelocityChartPainter extends CustomPainter {
  final Map<String, List<Map<String, dynamic>>> velocityHistory;
  final String selectedDeviceId;
  final ThemeService theme;
  final double scaleFactor;

  ResponsiveVelocityChartPainter(this.velocityHistory, this.selectedDeviceId,
      this.theme, this.scaleFactor);

  @override
  void paint(Canvas canvas, Size size) {
    final linearPaint = Paint()
      ..color = theme.infoColor
      ..strokeWidth = 3 * scaleFactor
      ..style = PaintingStyle.stroke;

    final angularPaint = Paint()
      ..color = theme.errorColor
      ..strokeWidth = 3 * scaleFactor
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
      ..strokeWidth = 1 * scaleFactor;

    for (int i = 0; i <= 4; i++) {
      final y = size.height * 0.05 + (size.height * 0.8) * (i / 4);
      canvas.drawLine(
          Offset(size.width * 0.1, y), Offset(size.width * 0.9, y), gridPaint);
    }

    for (int i = 0; i <= 6; i++) {
      final x = size.width * 0.1 + (size.width * 0.8) * (i / 6);
      canvas.drawLine(Offset(x, size.height * 0.05),
          Offset(x, size.height * 0.85), gridPaint);
    }
  }

  void _drawAxes(Canvas canvas, Size size) {
    final axisPaint = Paint()
      ..color = theme.accentColor
      ..strokeWidth = 2 * scaleFactor;

    canvas.drawLine(
      Offset(size.width * 0.1, size.height * 0.05),
      Offset(size.width * 0.1, size.height * 0.85),
      axisPaint,
    );

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

      final glowPaint = Paint()
        ..color = paint.color.withOpacity(0.6)
        ..maskFilter = MaskFilter.blur(BlurStyle.normal, 2 * scaleFactor);
      canvas.drawCircle(Offset(x, y), 3 * scaleFactor, glowPaint);
    }

    final glowLinePaint = Paint()
      ..color = paint.color.withOpacity(0.3)
      ..strokeWidth = 5 * scaleFactor
      ..style = PaintingStyle.stroke
      ..maskFilter = MaskFilter.blur(BlurStyle.normal, 2 * scaleFactor);
    canvas.drawPath(path, glowLinePaint);

    canvas.drawPath(path, paint);
  }

  void _drawLabels(Canvas canvas, Size size) {
    final textPainter = TextPainter(
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.center,
    );

    for (int i = 0; i <= 4; i++) {
      final velocity = 2.0 - (i * 0.5);
      final y = size.height * 0.05 + (size.height * 0.8) * (i / 4);

      textPainter.text = TextSpan(
        text: '${velocity.toStringAsFixed(1)}',
        style: AppTextStyles.getCaption(scaleFactor)
            .copyWith(color: theme.accentColor),
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

    final linearPaint = Paint()
      ..color = theme.infoColor
      ..strokeWidth = 3 * scaleFactor;
    canvas.drawLine(
      Offset(size.width * 0.12, size.height * 0.92),
      Offset(size.width * 0.17, size.height * 0.92),
      linearPaint,
    );

    textPainter.text = TextSpan(
      text: 'Linear Velocity (m/s)',
      style: AppTextStyles.getCaption(scaleFactor)
          .copyWith(color: theme.infoColor),
    );
    textPainter.layout();
    textPainter.paint(canvas, Offset(size.width * 0.19, size.height * 0.91));

    final angularPaint = Paint()
      ..color = theme.errorColor
      ..strokeWidth = 3 * scaleFactor;
    canvas.drawLine(
      Offset(size.width * 0.5, size.height * 0.92),
      Offset(size.width * 0.55, size.height * 0.92),
      angularPaint,
    );

    textPainter.text = TextSpan(
      text: 'Angular Velocity (rad/s)',
      style: AppTextStyles.getCaption(scaleFactor)
          .copyWith(color: theme.errorColor),
    );
    textPainter.layout();
    textPainter.paint(canvas, Offset(size.width * 0.57, size.height * 0.91));
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class ResponsiveNavigationTimelinePainter extends CustomPainter {
  final Map<String, List<Map<String, dynamic>>> navigationHistory;
  final String selectedDeviceId;
  final double scaleFactor;

  ResponsiveNavigationTimelinePainter(
      this.navigationHistory, this.selectedDeviceId, this.scaleFactor);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..strokeWidth = 2 * scaleFactor
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
      ..strokeWidth = 1 * scaleFactor;

    for (int i = 0; i <= 4; i++) {
      final y = size.height * 0.05 + (size.height * 0.8) * (i / 4);
      canvas.drawLine(
          Offset(size.width * 0.1, y), Offset(size.width * 0.9, y), gridPaint);
    }

    for (int i = 0; i <= 6; i++) {
      final x = size.width * 0.1 + (size.width * 0.8) * (i / 6);
      canvas.drawLine(Offset(x, size.height * 0.05),
          Offset(x, size.height * 0.85), gridPaint);
    }
  }

  void _drawAxes(Canvas canvas, Size size) {
    final axisPaint = Paint()
      ..color = Colors.grey.shade600
      ..strokeWidth = 2 * scaleFactor;

    canvas.drawLine(
      Offset(size.width * 0.1, size.height * 0.05),
      Offset(size.width * 0.1, size.height * 0.85),
      axisPaint,
    );

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

      final pointPaint = Paint()
        ..color = paint.color
        ..style = PaintingStyle.fill;
      canvas.drawCircle(Offset(x, y), 2 * scaleFactor, pointPaint);
    }

    canvas.drawPath(path, paint);
  }

  void _drawLabels(Canvas canvas, Size size) {
    final textPainter = TextPainter(
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.center,
    );

    textPainter.text = TextSpan(
      text: 'Distance Remaining (m)',
      style: AppTextStyles.getCaption(scaleFactor)
          .copyWith(color: Colors.grey.shade600),
    );
    textPainter.layout();

    canvas.save();
    canvas.translate(size.width * 0.02, size.height * 0.5);
    canvas.rotate(-math.pi / 2);
    textPainter.paint(
        canvas, Offset(-textPainter.width / 2, -textPainter.height / 2));
    canvas.restore();

    textPainter.text = TextSpan(
      text: 'Navigation Progress',
      style: AppTextStyles.getCaption(scaleFactor)
          .copyWith(color: Colors.grey.shade600),
    );
    textPainter.layout();
    textPainter.paint(canvas,
        Offset(size.width * 0.5 - textPainter.width / 2, size.height * 0.92));
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

// ============================================================================
// RESPONSIVE UI COMPONENTS
// ============================================================================
class ResponsiveStatsCard extends StatelessWidget {
  final String title;
  final String value;
  final String subtitle;
  final IconData icon;
  final Color color;
  final String trend;
  final bool showTrend;
  final double screenWidth;
  final double scaleFactor;

  const ResponsiveStatsCard({
    Key? key,
    required this.title,
    required this.value,
    required this.subtitle,
    required this.icon,
    required this.color,
    required this.trend,
    required this.showTrend,
    required this.screenWidth,
    required this.scaleFactor,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth) * 0.75;

    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [color.withOpacity(0.1), color.withOpacity(0.05)],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.1),
            blurRadius: 8 * scaleFactor,
            offset: Offset(0, 3 * scaleFactor),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Container(
              padding: EdgeInsets.all(
                  isCompact ? 8 * scaleFactor : 10 * scaleFactor),
              decoration: BoxDecoration(
                gradient:
                    LinearGradient(colors: [color, color.withOpacity(0.8)]),
                shape: BoxShape.circle,
              ),
              child: Icon(
                icon,
                size: AppConstants.getLargeIconSize(screenWidth),
                color: Colors.white,
              ),
            ),
            SizedBox(height: 12 * scaleFactor),
            Text(
              value,
              style:
                  AppTextStyles.getHeading3(scaleFactor).copyWith(color: color),
              overflow: TextOverflow.ellipsis,
            ),
            SizedBox(height: 4 * scaleFactor),
            Text(
              title,
              style: AppTextStyles.getCaption(scaleFactor).copyWith(
                fontWeight: FontWeight.w500,
              ),
              textAlign: TextAlign.center,
              overflow: TextOverflow.ellipsis,
            ),
            if (showTrend && !isCompact) ...[
              SizedBox(height: 4 * scaleFactor),
              Text(
                trend,
                style: AppTextStyles.getOverline(scaleFactor).copyWith(
                  color: color,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ],
          ],
        ),
      ),
    );
  }
}

class ResponsiveRealTimeMetricCard extends StatelessWidget {
  final String title;
  final String value;
  final IconData icon;
  final Color color;
  final String suffix;
  final double screenWidth;
  final double scaleFactor;

  const ResponsiveRealTimeMetricCard({
    Key? key,
    required this.title,
    required this.value,
    required this.icon,
    required this.color,
    required this.suffix,
    required this.screenWidth,
    required this.scaleFactor,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth) * 0.75;

    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [color.withOpacity(0.1), color.withOpacity(0.05)],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.3)),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.1),
            blurRadius: 8 * scaleFactor,
            offset: Offset(0, 3 * scaleFactor),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Container(
              padding:
                  EdgeInsets.all(isCompact ? 6 * scaleFactor : 8 * scaleFactor),
              decoration: BoxDecoration(
                gradient:
                    LinearGradient(colors: [color, color.withOpacity(0.8)]),
                shape: BoxShape.circle,
                boxShadow: [
                  BoxShadow(
                    color: color.withOpacity(0.3),
                    blurRadius: 6 * scaleFactor,
                    offset: Offset(0, 2 * scaleFactor),
                  ),
                ],
              ),
              child: Icon(
                icon,
                size: AppConstants.getIconSize(screenWidth),
                color: Colors.white,
              ),
            ),
            SizedBox(height: 12 * scaleFactor),
            FittedBox(
              fit: BoxFit.scaleDown,
              child: Text(
                value,
                style: AppTextStyles.getHeading3(scaleFactor)
                    .copyWith(color: color),
              ),
            ),
            SizedBox(height: 4 * scaleFactor),
            Text(
              title,
              style: AppTextStyles.getCaption(scaleFactor).copyWith(
                fontWeight: FontWeight.w500,
              ),
              textAlign: TextAlign.center,
              overflow: TextOverflow.ellipsis,
            ),
            SizedBox(height: 2 * scaleFactor),
            Text(
              suffix,
              style:
                  AppTextStyles.getOverline(scaleFactor).copyWith(color: color),
            ),
          ],
        ),
      ),
    );
  }
}

class ResponsiveEnhancedCard extends StatelessWidget {
  final Widget child;
  final LinearGradient? gradient;
  final double screenWidth;

  const ResponsiveEnhancedCard({
    Key? key,
    required this.child,
    required this.screenWidth,
    this.gradient,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);

    return Container(
      decoration: BoxDecoration(
        gradient: gradient ??
            LinearGradient(
              begin: Alignment.topLeft,
              end: Alignment.bottomRight,
              colors: [AppColors.cardBackground, Colors.grey.shade50],
            ),
        borderRadius: BorderRadius.circular(12),
        boxShadow: [
          BoxShadow(
            color: Colors.grey.withOpacity(0.1),
            blurRadius: 8 * scaleFactor,
            offset: Offset(0, 4 * scaleFactor),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: child,
      ),
    );
  }
}

class ResponsiveCardHeader extends StatelessWidget {
  final String title;
  final IconData icon;
  final Color color;
  final double screenWidth;
  final double scaleFactor;

  const ResponsiveCardHeader({
    Key? key,
    required this.title,
    required this.icon,
    required this.color,
    required this.screenWidth,
    required this.scaleFactor,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        ResponsiveEnhancedIconContainer(
          icon: icon,
          gradient: LinearGradient(colors: [color, color.withOpacity(0.8)]),
          screenWidth: screenWidth,
        ),
        SizedBox(
            width: screenWidth <= ResponsiveConfig.mobileMaxWidth ? 8 : 12),
        Text(
          title,
          style: AppTextStyles.getHeading3(scaleFactor),
        ),
      ],
    );
  }
}

class ResponsiveEnhancedIconContainer extends StatelessWidget {
  final IconData icon;
  final LinearGradient gradient;
  final double screenWidth;

  const ResponsiveEnhancedIconContainer({
    Key? key,
    required this.icon,
    required this.gradient,
    required this.screenWidth,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final iconSize = AppConstants.getIconSize(screenWidth);
    final padding = screenWidth <= ResponsiveConfig.mobileMaxWidth ? 6.0 : 8.0;

    return Container(
      padding: EdgeInsets.all(padding),
      decoration: BoxDecoration(
        gradient: gradient,
        borderRadius: BorderRadius.circular(12),
      ),
      child: Icon(icon, color: Colors.white, size: iconSize),
    );
  }
}

class ResponsiveMetricItem extends StatelessWidget {
  final String label;
  final String value;
  final IconData icon;
  final Color color;
  final double screenWidth;
  final double scaleFactor;

  const ResponsiveMetricItem({
    Key? key,
    required this.label,
    required this.value,
    required this.icon,
    required this.color,
    required this.screenWidth,
    required this.scaleFactor,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth) * 0.75;

    return Container(
      decoration: BoxDecoration(
        color: AppColors.cardBackground,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.1),
            blurRadius: 4 * scaleFactor,
            offset: Offset(0, 2 * scaleFactor),
          ),
        ],
      ),
      padding: EdgeInsets.all(cardPadding),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(icon, color: color, size: AppConstants.getIconSize(screenWidth)),
          SizedBox(height: 8 * scaleFactor),
          FittedBox(
            fit: BoxFit.scaleDown,
            child: Text(
              value,
              style: AppTextStyles.getBody(scaleFactor).copyWith(
                fontWeight: FontWeight.bold,
                color: color,
              ),
            ),
          ),
          SizedBox(height: 4 * scaleFactor),
          Text(
            label,
            style: AppTextStyles.getOverline(scaleFactor).copyWith(
              color: Colors.grey.shade600,
            ),
            textAlign: TextAlign.center,
            overflow: TextOverflow.ellipsis,
          ),
        ],
      ),
    );
  }
}

class ResponsiveEnhancedStatCard extends StatelessWidget {
  final String title;
  final String value;
  final IconData icon;
  final Color color;
  final double screenWidth;
  final double scaleFactor;

  const ResponsiveEnhancedStatCard({
    Key? key,
    required this.title,
    required this.value,
    required this.icon,
    required this.color,
    required this.screenWidth,
    required this.scaleFactor,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth) * 0.75;

    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [color.withOpacity(0.1), color.withOpacity(0.05)],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.1),
            blurRadius: 8 * scaleFactor,
            offset: Offset(0, 3 * scaleFactor),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Container(
              padding: EdgeInsets.all(8 * scaleFactor),
              decoration: BoxDecoration(
                gradient:
                    LinearGradient(colors: [color, color.withOpacity(0.8)]),
                shape: BoxShape.circle,
              ),
              child: Icon(
                icon,
                size: AppConstants.getLargeIconSize(screenWidth),
                color: Colors.white,
              ),
            ),
            SizedBox(height: 12 * scaleFactor),
            FittedBox(
              fit: BoxFit.scaleDown,
              child: Text(
                value,
                style: AppTextStyles.getHeading3(scaleFactor)
                    .copyWith(color: color),
              ),
            ),
            SizedBox(height: 4 * scaleFactor),
            Text(
              title,
              style: AppTextStyles.getCaption(scaleFactor).copyWith(
                fontWeight: FontWeight.w500,
              ),
              textAlign: TextAlign.center,
              overflow: TextOverflow.ellipsis,
            ),
          ],
        ),
      ),
    );
  }
}

class ResponsiveOrdersHeader extends StatelessWidget {
  final int totalOrders;
  final double screenWidth;

  const ResponsiveOrdersHeader({
    Key? key,
    required this.totalOrders,
    required this.screenWidth,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;

    return Container(
      padding: EdgeInsets.all(cardPadding),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            AppColors.warning.withOpacity(0.1),
            AppColors.warning.withOpacity(0.2)
          ],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: AppColors.warning.withOpacity(0.3)),
      ),
      child: Row(
        children: [
          Container(
            padding: EdgeInsets.all(isCompact ? 8 : 12),
            decoration: BoxDecoration(
              color: AppColors.warning,
              borderRadius: BorderRadius.circular(12),
            ),
            child: Icon(
              Icons.list_alt,
              color: Colors.white,
              size: AppConstants.getLargeIconSize(screenWidth),
            ),
          ),
          SizedBox(width: isCompact ? 12 : 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Order History',
                  style: AppTextStyles.getHeading3(scaleFactor),
                ),
                Text(
                  'Total: $totalOrders orders completed',
                  style: AppTextStyles.getBody(scaleFactor).copyWith(
                    color: Colors.grey.shade600,
                  ),
                ),
              ],
            ),
          ),
          if (totalOrders > 0)
            Container(
              padding: EdgeInsets.symmetric(
                horizontal: isCompact ? 8 : 12,
                vertical: isCompact ? 4 : 6,
              ),
              decoration: BoxDecoration(
                color: AppColors.warning,
                borderRadius: BorderRadius.circular(20),
              ),
              child: Text(
                '$totalOrders',
                style: TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  fontSize: 12 * scaleFactor,
                ),
              ),
            ),
        ],
      ),
    );
  }
}

class ResponsiveOrderHistoryCard extends StatelessWidget {
  final Map<String, dynamic> order;
  final double screenWidth;

  const ResponsiveOrderHistoryCard({
    Key? key,
    required this.order,
    required this.screenWidth,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final completedAt = order['completedAt'] as DateTime;
    final duration = order['duration'] as double;
    final distance = order['distance'] as double;
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;

    Color statusColor = AppColors.success;
    if (duration > 60) {
      statusColor = AppColors.error;
    } else if (duration > 30) {
      statusColor = AppColors.warning;
    }

    return Container(
      margin: EdgeInsets.symmetric(vertical: 4 * scaleFactor),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [AppColors.cardBackground, Colors.grey.shade50],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.grey.shade200),
        boxShadow: [
          BoxShadow(
            color: Colors.grey.withOpacity(0.1),
            blurRadius: 4 * scaleFactor,
            offset: Offset(0, 2 * scaleFactor),
          ),
        ],
      ),
      child: ListTile(
        contentPadding: EdgeInsets.all(isCompact ? 8 : 12),
        leading: Container(
          width: isCompact ? 40 : 50,
          height: isCompact ? 40 : 50,
          decoration: BoxDecoration(
            gradient: LinearGradient(
                colors: [statusColor, statusColor.withOpacity(0.8)]),
            shape: BoxShape.circle,
          ),
          child: Icon(
            Icons.check,
            color: Colors.white,
            size: isCompact ? 16 : 20,
          ),
        ),
        title: Text(
          order['name'],
          style: AppTextStyles.getSubtitle(scaleFactor),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Completed: ${FormatUtils.formatDateTime(completedAt)}',
              style: AppTextStyles.getCaption(scaleFactor),
            ),
            Text(
              'Distance: ${distance.toStringAsFixed(1)} m',
              style: AppTextStyles.getCaption(scaleFactor),
            ),
            Text(
              'Waypoints: ${order['waypoints']}',
              style: AppTextStyles.getCaption(scaleFactor),
            ),
          ],
        ),
        trailing: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              '${duration.toStringAsFixed(0)}m',
              style: AppTextStyles.getSubtitle(scaleFactor)
                  .copyWith(color: statusColor),
            ),
            SizedBox(height: 4 * scaleFactor),
            Container(
              padding: EdgeInsets.symmetric(horizontal: 6, vertical: 2),
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
                style: AppTextStyles.getOverline(scaleFactor)
                    .copyWith(color: statusColor),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class ResponsiveNavigationHeader extends StatelessWidget {
  final int activeNavigations;
  final double screenWidth;

  const ResponsiveNavigationHeader({
    Key? key,
    required this.activeNavigations,
    required this.screenWidth,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;

    return Container(
      padding: EdgeInsets.all(cardPadding),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.teal.withOpacity(0.1), Colors.teal.withOpacity(0.2)],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.teal.withOpacity(0.3)),
      ),
      child: Row(
        children: [
          Container(
            padding: EdgeInsets.all(isCompact ? 8 : 12),
            decoration: BoxDecoration(
              color: Colors.teal,
              borderRadius: BorderRadius.circular(12),
            ),
            child: Icon(
              Icons.navigation,
              color: Colors.white,
              size: AppConstants.getLargeIconSize(screenWidth),
            ),
          ),
          SizedBox(width: isCompact ? 12 : 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Navigation Analytics',
                  style: AppTextStyles.getHeading3(scaleFactor),
                ),
                Text(
                  'Active navigations: $activeNavigations',
                  style: AppTextStyles.getBody(scaleFactor).copyWith(
                    color: Colors.grey.shade600,
                  ),
                ),
              ],
            ),
          ),
          if (activeNavigations > 0)
            Container(
              padding: EdgeInsets.symmetric(
                horizontal: isCompact ? 8 : 12,
                vertical: isCompact ? 4 : 6,
              ),
              decoration: BoxDecoration(
                color: AppColors.success,
                borderRadius: BorderRadius.circular(20),
              ),
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Container(
                    width: 8 * scaleFactor,
                    height: 8 * scaleFactor,
                    decoration: const BoxDecoration(
                      color: Colors.white,
                      shape: BoxShape.circle,
                    ),
                  ),
                  SizedBox(width: 4),
                  Text(
                    'ACTIVE',
                    style: AppTextStyles.getOverline(scaleFactor).copyWith(
                      color: Colors.white,
                    ),
                  ),
                ],
              ),
            ),
        ],
      ),
    );
  }
}

class ResponsiveNavigationStatusCard extends StatelessWidget {
  final String deviceId;
  final Map<String, dynamic> status;
  final double screenWidth;

  const ResponsiveNavigationStatusCard({
    Key? key,
    required this.deviceId,
    required this.status,
    required this.screenWidth,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final timeRemaining = status['estimated_time_remaining'] as double;
    final distanceRemaining = status['distance_remaining'] as double;
    final recoveries = status['number_of_recoveries'] as int;
    final speed = status['speed'] as double;
    final navigationTime = status['navigation_time'] as double;
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final gridSpacing = ResponsiveConfig.getGridSpacing(screenWidth);

    return ResponsiveEnhancedCard(
      screenWidth: screenWidth,
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
                padding: EdgeInsets.all(6 * scaleFactor),
                decoration: BoxDecoration(
                  color: AppColors.info,
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Icon(
                  Icons.my_location,
                  color: Colors.white,
                  size: 16 * scaleFactor,
                ),
              ),
              SizedBox(width: 8 * scaleFactor),
              Expanded(
                child: Text(
                  deviceId,
                  style: AppTextStyles.getSubtitle(scaleFactor),
                ),
              ),
              Container(
                padding: EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                decoration: BoxDecoration(
                  color: AppColors.success,
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Text(
                  'NAVIGATING',
                  style: AppTextStyles.getOverline(scaleFactor).copyWith(
                    color: Colors.white,
                  ),
                ),
              ),
            ],
          ),
          SizedBox(height: gridSpacing),
          GridView.count(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            crossAxisCount: 2,
            crossAxisSpacing: gridSpacing * 0.5,
            mainAxisSpacing: gridSpacing * 0.5,
            childAspectRatio:
                screenWidth <= ResponsiveConfig.mobileMaxWidth ? 2.0 : 2.5,
            children: [
              ResponsiveNavigationMetricItem(
                label: 'ETA',
                value: '${timeRemaining.toStringAsFixed(0)}s',
                icon: Icons.timer,
                color: AppColors.success,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
              ResponsiveNavigationMetricItem(
                label: 'Distance',
                value: '${distanceRemaining.toStringAsFixed(1)}m',
                icon: Icons.straighten,
                color: AppColors.info,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
              ResponsiveNavigationMetricItem(
                label: 'Speed',
                value: '${speed.toStringAsFixed(2)} m/s',
                icon: Icons.speed,
                color: AppColors.warning,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
              ResponsiveNavigationMetricItem(
                label: 'Recoveries',
                value: recoveries.toString(),
                icon: Icons.refresh,
                color: recoveries > 0 ? AppColors.error : Colors.grey,
                screenWidth: screenWidth,
                scaleFactor: scaleFactor,
              ),
            ],
          ),
          SizedBox(height: gridSpacing * 0.75),
          Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Text(
                    'Progress',
                    style: AppTextStyles.getCaption(scaleFactor).copyWith(
                      fontWeight: FontWeight.w500,
                    ),
                  ),
                  Text(
                    'Time: ${navigationTime.toStringAsFixed(0)}s',
                    style: AppTextStyles.getOverline(scaleFactor),
                  ),
                ],
              ),
              SizedBox(height: 4 * scaleFactor),
              Container(
                height: 6 * scaleFactor,
                decoration: BoxDecoration(
                  color: Colors.grey.shade300,
                  borderRadius: BorderRadius.circular(3 * scaleFactor),
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
                      borderRadius: BorderRadius.circular(3 * scaleFactor),
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

class ResponsiveNavigationMetricItem extends StatelessWidget {
  final String label;
  final String value;
  final IconData icon;
  final Color color;
  final double screenWidth;
  final double scaleFactor;

  const ResponsiveNavigationMetricItem({
    Key? key,
    required this.label,
    required this.value,
    required this.icon,
    required this.color,
    required this.screenWidth,
    required this.scaleFactor,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;

    return Container(
      decoration: BoxDecoration(
        color: AppColors.cardBackground,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.2)),
      ),
      padding: EdgeInsets.all(isCompact ? 6 : 8),
      child: Row(
        children: [
          Icon(icon, color: color, size: isCompact ? 14 : 16),
          SizedBox(width: 6),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                FittedBox(
                  fit: BoxFit.scaleDown,
                  child: Text(
                    value,
                    style: AppTextStyles.getCaption(scaleFactor).copyWith(
                      fontWeight: FontWeight.bold,
                      color: color,
                    ),
                  ),
                ),
                Text(
                  label,
                  style: AppTextStyles.getOverline(scaleFactor).copyWith(
                    fontSize: isCompact ? 8 : 9,
                  ),
                  overflow: TextOverflow.ellipsis,
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class ResponsiveEventsHeader extends StatelessWidget {
  final int totalEvents;
  final double screenWidth;

  const ResponsiveEventsHeader({
    Key? key,
    required this.totalEvents,
    required this.screenWidth,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;

    return Container(
      padding: EdgeInsets.all(cardPadding),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            AppColors.error.withOpacity(0.1),
            AppColors.error.withOpacity(0.2)
          ],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: AppColors.error.withOpacity(0.3)),
      ),
      child: Row(
        children: [
          Container(
            padding: EdgeInsets.all(isCompact ? 8 : 12),
            decoration: BoxDecoration(
              color: AppColors.error,
              borderRadius: BorderRadius.circular(12),
            ),
            child: Icon(
              Icons.event,
              color: Colors.white,
              size: AppConstants.getLargeIconSize(screenWidth),
            ),
          ),
          SizedBox(width: isCompact ? 12 : 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'System Events',
                  style: AppTextStyles.getHeading3(scaleFactor),
                ),
                Text(
                  'Total: $totalEvents events recorded',
                  style: AppTextStyles.getBody(scaleFactor).copyWith(
                    color: Colors.grey.shade600,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class ResponsiveEventCard extends StatelessWidget {
  final Map<String, dynamic> event;
  final double screenWidth;

  const ResponsiveEventCard({
    Key? key,
    required this.event,
    required this.screenWidth,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final type = event['type'];
    final severity = event['severity'];
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final isCompact = screenWidth <= ResponsiveConfig.mobileMaxWidth;

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
      margin: EdgeInsets.symmetric(vertical: 4 * scaleFactor),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [AppColors.cardBackground, color.withOpacity(0.02)],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.2)),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.1),
            blurRadius: 4 * scaleFactor,
            offset: Offset(0, 2 * scaleFactor),
          ),
        ],
      ),
      child: ListTile(
        contentPadding: EdgeInsets.all(isCompact ? 8 : 12),
        leading: Container(
          width: isCompact ? 40 : 50,
          height: isCompact ? 40 : 50,
          decoration: BoxDecoration(
            gradient: LinearGradient(colors: [color, color.withOpacity(0.8)]),
            shape: BoxShape.circle,
          ),
          child: Icon(
            icon,
            color: Colors.white,
            size: isCompact ? 16 : 20,
          ),
        ),
        title: Text(
          event['message'],
          style: AppTextStyles.getSubtitle(scaleFactor),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Device: ${event['deviceId']}',
              style: AppTextStyles.getCaption(scaleFactor),
            ),
            Text(
              FormatUtils.formatDateTime(event['timestamp'] as DateTime),
              style: AppTextStyles.getCaption(scaleFactor),
            ),
          ],
        ),
        trailing: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Container(
              padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
              decoration: BoxDecoration(
                gradient:
                    LinearGradient(colors: [color, color.withOpacity(0.8)]),
                borderRadius: BorderRadius.circular(12),
              ),
              child: Text(
                type.toUpperCase(),
                style: AppTextStyles.getOverline(scaleFactor).copyWith(
                  color: Colors.white,
                ),
              ),
            ),
            SizedBox(height: 4 * scaleFactor),
            Container(
              padding: EdgeInsets.symmetric(horizontal: 6, vertical: 2),
              decoration: BoxDecoration(
                color: color.withOpacity(0.1),
                borderRadius: BorderRadius.circular(6),
              ),
              child: Text(
                severity.toUpperCase(),
                style: AppTextStyles.getOverline(scaleFactor).copyWith(
                  fontSize: 8 * scaleFactor,
                  color: color,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class ResponsiveEmptyStateCard extends StatelessWidget {
  final IconData icon;
  final String title;
  final String subtitle;
  final double screenWidth;

  const ResponsiveEmptyStateCard({
    Key? key,
    required this.icon,
    required this.title,
    required this.subtitle,
    required this.screenWidth,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final scaleFactor = ResponsiveConfig.getScaleFactor(screenWidth);
    final cardPadding = ResponsiveConfig.getCardPadding(screenWidth);

    return ResponsiveEnhancedCard(
      screenWidth: screenWidth,
      gradient: LinearGradient(
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
        colors: [Colors.grey.shade50, AppColors.cardBackground],
      ),
      child: Padding(
        padding: EdgeInsets.all(cardPadding),
        child: Column(
          children: [
            Icon(icon, size: 48 * scaleFactor, color: AppColors.primary),
            SizedBox(height: 12 * scaleFactor),
            Text(
              title,
              style: AppTextStyles.getHeading3(scaleFactor),
            ),
            SizedBox(height: 4 * scaleFactor),
            Text(
              subtitle,
              style: AppTextStyles.getBody(scaleFactor),
              textAlign: TextAlign.center,
            ),
          ],
        ),
      ),
    );
  }
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
