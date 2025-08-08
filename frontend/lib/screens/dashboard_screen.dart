// screens/dashboard_screen.dart - UPDATED with Simple Coordinate Order Creator
import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import '../models/odom.dart';
import '../models/map_data.dart';
import 'profile_screen.dart';
import 'map_page.dart'; // Your existing map page
import 'control_page.dart';
import '../widgets/simple_coordinates_order_creator.dart';

// ✅ ADD this extension for string capitalization
extension StringCapitalization on String {
  String capitalize() {
    if (isEmpty) return this;
    return "${this[0].toUpperCase()}${substring(1)}";
  }
}

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

  // UI State
  bool _isLoading = true;
  bool _isRefreshing = false;
  int _currentTabIndex = 0;
  Timer? _refreshTimer;
  String? _selectedDeviceFilter;

  // ✅ Enhanced map management variables
  Map<String, List<Map<String, dynamic>>> _savedMapsCache = {};
  bool _isLoadingMaps = false;

  // ✅ Order deletion tracking
  Set<String> _deletingOrders = {};

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
      // Load devices first
      await _loadDevices();

      // Then load saved maps cache
      await _loadAllSavedMaps();

      // Then try to load live maps (with fallback to saved maps)
      await _loadMapsForAllDevices();

      // Finally load orders
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
      case 'coordinate_order_failure': // ✅ NEW: Handle coordinate order failures
        _loadOrdersForAllDevices();
        // ✅ NEW: Show restart popup for coordinate order failures
        if (event['type'] == 'coordinate_order_failure' &&
            event['showRestartPopup'] == true) {
          _showCoordinateOrderRestartDialog(event);
        }
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

  // ... (Keep all your existing data loading methods unchanged until order creation methods)

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
    try {
      print('🔍 Loading maps for all devices...');
      print('🔗 API Service initialized: ${_apiService.toString()}');
      print('🔗 Connected devices count: ${_connectedDevices.length}');

      for (final device in _connectedDevices) {
        try {
          print(
              '🗺️ Attempting to load map for device: ${device['name']}(${device['id']})');

          // First try to get live map data
          final response = await _apiService.getMapData(device['id']);
          print('📥 API Response success: ${response['success']}');
          print('📥 API Response keys: ${response.keys.join(', ')}');

          if (response['success'] == true && response['mapData'] != null) {
            print('✅ Creating MapData object from response...');
            final mapData = MapData.fromJson(response['mapData']);
            setState(() {
              _availableMaps[device['id']] = mapData;
              // Also store by device name for compatibility
              if (device['name'] != null && device['name'] != device['id']) {
                _availableMaps[device['name']] = mapData;
              }
            });
            print(
                '✅ Live map loaded and stored for: ${device['id']} and ${device['name']}');
          } else {
            print(
                '⚠️ No live map data found for ${device['id']}, checking saved maps...');
            // If no live map, try to create from saved maps
            await _loadMapFromSavedMaps(device);
          }
        } catch (e, stackTrace) {
          print('❌ Error loading map for ${device['id']}: $e');
          print('📍 Stack trace: $stackTrace');
          // Try fallback to saved maps
          await _loadMapFromSavedMaps(device);
        }
      }
      print('🔍 Final available maps: ${_availableMaps.keys.join(', ')}');
    } catch (e, stackTrace) {
      print('❌ FATAL Error in _loadMapsForAllDevices: $e');
      print('📍 FATAL Stack trace: $stackTrace');
    }
  }

  /// Load map from saved maps data as fallback
  Future<void> _loadMapFromSavedMaps(Map<String, dynamic> device) async {
    try {
      final deviceId = device['id'];
      print('🔄 Attempting to load from saved maps for: $deviceId');

      // Try to get saved maps for this device
      final savedMaps = await _apiService.getSavedMaps(
        deviceId,
        includePreview: true,
        sortBy: 'savedAt',
        descending: true,
      );

      if (savedMaps.isNotEmpty) {
        print('📋 Found ${savedMaps.length} saved maps for $deviceId');

        // Use the most recent saved map to create a MapData object
        final mostRecentMap = savedMaps.first;

        // Create a basic MapData object from saved map metadata
        final mockMapData = {
          'deviceId': deviceId,
          'timestamp':
              mostRecentMap['savedAt'] ?? DateTime.now().toIso8601String(),
          'info': {
            'width': 1000, // Default values since we don't have actual map data
            'height': 1000,
            'resolution': 0.05,
            'origin': {
              'x': -25.0,
              'y': -25.0,
              'z': 0.0,
              'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
            },
          },
          'data': [], // Empty occupancy data
          'shapes': mostRecentMap['shapes'] ?? [],
          'version': 1,
        };

        final mapData = MapData.fromJson(mockMapData);
        setState(() {
          _availableMaps[deviceId] = mapData;
          // Also store by device name for compatibility
          if (device['name'] != null && device['name'] != deviceId) {
            _availableMaps[device['name']] = mapData;
          }
        });

        print(
            '✅ Saved map converted to MapData for: $deviceId and ${device['name']}');
        print(
            '📊 Map contains ${savedMaps.length} saved maps, using: ${mostRecentMap['name']}');
      } else {
        print('⚠️ No saved maps found for device: $deviceId');
        // Create minimal fallback map
        await _createFallbackMapData(device);
      }
    } catch (e) {
      print('❌ Error loading from saved maps for ${device['id']}: $e');
      await _createFallbackMapData(device);
    }
  }

  /// Create minimal fallback map data when no maps are available
  Future<void> _createFallbackMapData(Map<String, dynamic> device) async {
    try {
      final deviceId = device['id'];
      print('🔧 Creating fallback map data for: $deviceId');

      final fallbackMapData = {
        'deviceId': deviceId,
        'timestamp': DateTime.now().toIso8601String(),
        'info': {
          'width': 1000,
          'height': 1000,
          'resolution': 0.05,
          'origin': {
            'x': -25.0,
            'y': -25.0,
            'z': 0.0,
            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
          },
        },
        'data': [],
        'shapes': [],
        'version': 1,
      };

      final mapData = MapData.fromJson(fallbackMapData);
      setState(() {
        _availableMaps[deviceId] = mapData;
        if (device['name'] != null && device['name'] != deviceId) {
          _availableMaps[device['name']] = mapData;
        }
      });

      print('✅ Fallback map created for: $deviceId');
    } catch (e) {
      print('❌ Error creating fallback map for ${device['id']}: $e');
    }
  }

  Future<void> _loadOrdersForAllDevices() async {
    for (final device in _connectedDevices) {
      try {
        // Load regular orders
        final orders = await _apiService.getOrders(device['id']);

        // ✅ ENHANCED: Also load simple coordinate orders
        List<Map<String, dynamic>> allOrders = List.from(orders);

        try {
          final simpleOrdersResponse =
              await _apiService.getSimpleOrders(device['id']);
          if (simpleOrdersResponse['success'] == true &&
              simpleOrdersResponse['orders'] != null) {
            final simpleOrders = simpleOrdersResponse['orders'] as List;

            // Add simple orders to the list with proper formatting
            for (final simpleOrder in simpleOrders) {
              allOrders.add({
                'id': simpleOrder['id'],
                'name': simpleOrder['name'],
                'deviceId': device['id'],
                'deviceName': device['name'],
                'status': simpleOrder['status'] ?? 'pending',
                'coordinates': simpleOrder['coordinates'] ?? [],
                'currentCoordinate': simpleOrder['currentCoordinate'] ?? 0,
                'createdAt': simpleOrder['createdAt'],
                'completedAt': simpleOrder['completedAt'],
                'type': 'coordinate', // Mark as coordinate order
                'progress': {
                  'currentStep': simpleOrder['currentCoordinate'] ?? 0,
                  'percentage': simpleOrder['coordinates']?.isNotEmpty == true
                      ? ((simpleOrder['currentCoordinate'] ?? 0) /
                              simpleOrder['coordinates'].length *
                              100)
                          .round()
                      : 0,
                },
              });
            }

            print(
                '✅ Loaded ${simpleOrders.length} simple coordinate orders for ${device['id']}');
          }
        } catch (simpleOrderError) {
          print(
              '⚠️ Simple orders not available for ${device['id']}: $simpleOrderError');
          // Continue with regular orders only
        }

        setState(() {
          _deviceOrders[device['id']] = allOrders;
        });

        print('✅ Total orders loaded for ${device['id']}: ${allOrders.length}');
      } catch (e) {
        print('❌ Error loading orders for ${device['id']}: $e');
      }
    }
  }

  /// Load all saved maps with detailed information
  Future<void> _loadAllSavedMaps() async {
    if (_isLoadingMaps) return;

    setState(() {
      _isLoadingMaps = true;
    });

    try {
      _savedMapsCache.clear();

      for (final device in _connectedDevices) {
        try {
          // ✅ Use getSavedMaps if available, otherwise fallback to empty list
          List<Map<String, dynamic>> savedMaps = [];
          try {
            savedMaps = await _apiService.getSavedMaps(
              device['id'],
              includePreview: true,
              sortBy: 'savedAt',
              descending: true,
            );
          } catch (apiError) {
            print(
                '⚠️ getSavedMaps not available for ${device['id']}, using empty list');
            savedMaps = [];
          }

          setState(() {
            _savedMapsCache[device['id']] = savedMaps;
          });

          print('✅ Loaded ${savedMaps.length} saved maps for ${device['id']}');
        } catch (e) {
          print('❌ Error loading saved maps for ${device['id']}: $e');
          // Continue with other devices even if one fails
        }
      }
    } catch (e) {
      print('❌ Error loading all saved maps: $e');
    } finally {
      setState(() {
        _isLoadingMaps = false;
      });
    }
  }

  // ✅ FIXED: Proper navigation methods
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
        builder: (context) => EnhancedMapPage(), // Use your existing MapPage
        settings: RouteSettings(
          arguments: {
            'deviceId': device['id'],
          },
        ),
      ),
    );
  }

  // ✅ FIXED: Complete fleet management navigation
  // ✅ SIMPLIFIED: Simple coordinate order creation only
  void _showSimpleCoordinateOrderCreator() {
    if (_connectedDevices.isEmpty) {
      _showErrorSnackBar('No devices available. Connect a device first.');
      return;
    }

    print('🎯 Opening simple coordinate order creator...');
    print(
        '🔗 Available devices: ${_connectedDevices.map((d) => '${d['name']}(${d['id']})').join(', ')}');
    print('🗺️ Available maps: ${_availableMaps.keys.join(', ')}');

    // Find first device with map for pre-selection
    String? selectedDeviceId;
    for (final device in _connectedDevices) {
      if (_availableMaps.containsKey(device['id'])) {
        selectedDeviceId = device['id'];
        break;
      }
    }

    if (selectedDeviceId == null) {
      _showErrorSnackBar('No map data available. Create a map first.');
      return;
    }

    // ✅ SIMPLIFIED: Show only the simple coordinate order creator
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (context) => SimpleCoordinateOrderCreator(
          deviceId: selectedDeviceId!,
          mapData: _availableMaps[selectedDeviceId]!,
          onOrderCreated: (orderData) {
            print('✅ Simple coordinate order created: ${orderData['name']}');
            _handleSimpleOrderCreated(orderData);
          },
        ),
      ),
    );
  }

  // ✅ NEW: Handle simple coordinate order creation
  Future<void> _handleSimpleOrderCreated(Map<String, dynamic> orderData) async {
    try {
      print('📝 Processing simple coordinate order: ${orderData['name']}');

      // The order is already created by the SimpleCoordinateOrderCreator
      // Just refresh our data and show success message

      _showSuccessSnackBar(
          '✅ Coordinate order "${orderData['name']}" created successfully with ${(orderData['coordinates'] as List).length} coordinates!');

      // Refresh orders
      await _loadOrdersForAllDevices();
    } catch (e) {
      _showErrorSnackBar('❌ Error processing coordinate order: $e');
    }
  }

  void _showCreateOrderForDevice(String deviceId) {
    if (!_availableMaps.containsKey(deviceId)) {
      _showErrorSnackBar(
          'No map available for this device. Create a map first.');
      return;
    }

    final device = _connectedDevices.firstWhere(
      (d) => d['id'] == deviceId,
      orElse: () => {'id': deviceId, 'name': deviceId},
    );

    print(
        '📱 Creating coordinate order for device: ${device['name']} ($deviceId)');

    // ✅ SIMPLIFIED: Direct to coordinate order creator
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (context) => SimpleCoordinateOrderCreator(
          deviceId: deviceId,
          mapData: _availableMaps[deviceId]!,
          onOrderCreated: (orderData) {
            print(
                '✅ Coordinate order created for device $deviceId: ${orderData['name']}');
            _handleSimpleOrderCreated(orderData);
          },
        ),
      ),
    );
  }

  // ✅ NEW: Show restart dialog for coordinate order failures
  void _showCoordinateOrderRestartDialog(Map<String, dynamic> event) {
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (context) => AlertDialog(
        title: Text('❌ Order Execution Failed'),
        content: Text(
          'Navigation to coordinate ${event['failedCoordinate']} failed:\n\n${event['error']}\n\nWould you like to restart the order from the beginning?',
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () async {
              Navigator.of(context).pop();
              await _restartCoordinateOrder(event['orderId']);
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.orange),
            child: Text('Restart Order'),
          ),
        ],
      ),
    );
  }

  // ✅ NEW: Show delete order confirmation dialog
  void _showDeleteOrderDialog(Map<String, dynamic> order) {
    final orderName = order['name'] ?? 'Unnamed Order';
    final orderType = (order['coordinates'] as List?)?.isNotEmpty == true
        ? 'coordinate order'
        : 'order';

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Row(
          children: [
            Icon(Icons.warning, color: Colors.red),
            SizedBox(width: 8),
            Text('Delete Order'),
          ],
        ),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Are you sure you want to delete this $orderType?',
              style: TextStyle(fontSize: 16),
            ),
            SizedBox(height: 12),
            Container(
              padding: EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: Colors.red.shade50,
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.red.shade200),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    'Order: $orderName',
                    style: TextStyle(fontWeight: FontWeight.bold),
                  ),
                  Text('Device: ${order['deviceName'] ?? order['deviceId']}'),
                  Text(
                      'Status: ${(order['status'] ?? 'pending').toUpperCase()}'),
                ],
              ),
            ),
            SizedBox(height: 12),
            Text(
              '⚠️ This action cannot be undone!',
              style: TextStyle(
                color: Colors.red,
                fontWeight: FontWeight.bold,
              ),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _deleteOrder(order);
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.red,
              foregroundColor: Colors.white,
            ),
            child: Text('Delete'),
          ),
        ],
      ),
    );
  }

  // ✅ NEW: Delete order
  Future<void> _deleteOrder(Map<String, dynamic> order) async {
    final orderId = order['id']?.toString();
    if (orderId == null) {
      _showErrorSnackBar('Invalid order ID');
      return;
    }

    // ✅ Add to deletion tracking for loading state
    setState(() {
      _deletingOrders.add(orderId);
    });

    try {
      final deviceId = order['deviceId'];
      final coordinates = order['coordinates'] as List? ?? [];
      final isCoordinateOrder = coordinates.isNotEmpty;

      if (deviceId == null) {
        _showErrorSnackBar('Invalid device ID');
        return;
      }

      print(
          '🗑️ Deleting ${isCoordinateOrder ? 'coordinate' : 'regular'} order: $orderId from device: $deviceId');

      // ✅ ENHANCED: Store original order for potential rollback
      Map<String, dynamic>? originalOrder;
      if (_deviceOrders.containsKey(deviceId)) {
        originalOrder = _deviceOrders[deviceId]!.firstWhere(
          (o) => o['id'] == orderId,
          orElse: () => {},
        );
      }

      // ✅ ENHANCED: Remove from UI immediately for instant response
      setState(() {
        if (_deviceOrders.containsKey(deviceId)) {
          _deviceOrders[deviceId]!.removeWhere((o) => o['id'] == orderId);
          print(
              '🔄 Immediately removed order from UI: ${_deviceOrders[deviceId]!.length} orders remaining');
        }
      });

      // Show immediate feedback
      _showSuccessSnackBar('🗑️ Deleting order "${order['name']}"...');

      // Now attempt backend deletion
      Map<String, dynamic> response;

      if (isCoordinateOrder) {
        // Delete coordinate order from simple orders API
        response = await _apiService.deleteSimpleOrder(deviceId, orderId);
      } else {
        // Delete regular order from orders API
        response = await _apiService.deleteOrder(deviceId, orderId);
      }

      if (response['success'] == true) {
        print('✅ Order deleted successfully from backend');

        // Show success confirmation
        _showSuccessSnackBar(
            '✅ Order "${order['name']}" deleted successfully!');

        // ✅ ENHANCED: Refresh data from backend to ensure consistency (in background)
        _loadOrdersForAllDevices().catchError((refreshError) {
          print('⚠️ Error refreshing data after deletion: $refreshError');
          // UI is already updated, so this is not critical
        });
      } else {
        // ✅ ENHANCED: Rollback UI changes if backend deletion failed
        print('❌ Backend deletion failed, rolling back UI changes');
        if (originalOrder != null && originalOrder.isNotEmpty) {
          setState(() {
            if (_deviceOrders.containsKey(deviceId)) {
              _deviceOrders[deviceId]!.add(originalOrder!);
              // Re-sort to maintain order
              _deviceOrders[deviceId]!.sort((a, b) {
                final aTime =
                    DateTime.tryParse(a['createdAt'] ?? '') ?? DateTime.now();
                final bTime =
                    DateTime.tryParse(b['createdAt'] ?? '') ?? DateTime.now();
                return bTime.compareTo(aTime);
              });
            }
          });
        }
        _showErrorSnackBar('❌ Failed to delete order: ${response['error']}');
      }
    } catch (e) {
      print('❌ Error deleting order: $e');

      // ✅ ENHANCED: Refresh orders to restore correct state after error
      _loadOrdersForAllDevices().catchError((refreshError) {
        print(
            '⚠️ Error restoring orders after deletion failure: $refreshError');
      });

      _showErrorSnackBar('❌ Error deleting order: $e');
    } finally {
      // ✅ Remove from deletion tracking
      setState(() {
        _deletingOrders.remove(orderId);
      });
    }
  }

  // ✅ NEW: Restart coordinate order
  Future<void> _restartCoordinateOrder(String orderId) async {
    try {
      final response = await _apiService.restartOrder(orderId: orderId);

      if (response['success'] == true) {
        _showSuccessSnackBar('✅ Order restarted successfully!');
        await _loadOrdersForAllDevices();
      } else {
        _showErrorSnackBar('❌ Failed to restart order: ${response['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('❌ Error restarting order: $e');
    }
  }

  // ✅ NEW: Start coordinate order execution
  Future<void> _startCoordinateOrder(Map<String, dynamic> order) async {
    try {
      final orderId = order['id'];
      if (orderId == null) {
        _showErrorSnackBar('Invalid order ID');
        return;
      }

      print('🚀 Starting coordinate order: $orderId');

      final response = await _apiService.startOrderExecution(orderId: orderId);

      if (response['success'] == true) {
        _showSuccessSnackBar('✅ Coordinate order started successfully!');
        await _loadOrdersForAllDevices();
      } else {
        _showErrorSnackBar('❌ Failed to start order: ${response['error']}');
      }
    } catch (e) {
      print('❌ Error starting coordinate order: $e');
      _showErrorSnackBar('❌ Error starting order: $e');
    }
  }

  // ✅ NEW: Stop coordinate order execution
  Future<void> _stopCoordinateOrderExecution(Map<String, dynamic> order) async {
    try {
      print('🛑 Stopping coordinate order execution');

      final response = await _apiService.stopOrderExecution();

      if (response['success'] == true) {
        _showSuccessSnackBar('✅ Order execution stopped');
        await _loadOrdersForAllDevices();
      } else {
        _showErrorSnackBar('❌ Failed to stop execution: ${response['error']}');
      }
    } catch (e) {
      print('❌ Error stopping execution: $e');
      _showErrorSnackBar('❌ Error stopping execution: $e');
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
              // ✅ UPDATED: Show coordinates instead of waypoints
              if (order['coordinates'] != null) ...[
                Text('Coordinates: ${(order['coordinates'] as List).length}'),
                SizedBox(height: 16),
                Text('Coordinate Sequence:',
                    style: TextStyle(fontWeight: FontWeight.bold)),
                ...(order['coordinates'] as List<dynamic>)
                    .asMap()
                    .entries
                    .map((entry) {
                  final index = entry.key + 1;
                  final coord = entry.value;
                  return Text(
                      '$index. ${coord['name']} (${coord['x'].toStringAsFixed(2)}, ${coord['y'].toStringAsFixed(2)})');
                }).toList(),
              ] else if (order['waypoints'] != null) ...[
                Text('Sequence Steps: ${order['waypoints']?.length ?? 0}'),
                if (order['createdAt'] != null) ...[
                  SizedBox(height: 8),
                  Text('Created: ${order['createdAt']}'),
                ],
                SizedBox(height: 16),
                Text('Waypoints:',
                    style: TextStyle(fontWeight: FontWeight.bold)),
                ...(order['waypoints'] as List<dynamic>? ?? [])
                    .map((wp) => Text('• ${wp['name']} (${wp['type']})'))
                    .toList(),
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
        content: Text('Stop all active orders and AMR movement immediately?'),
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
        _loadAllSavedMaps(),
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
      await _loadOrdersForAllDevices();
    } catch (e) {
      print('❌ Background refresh failed: $e');
    }
  }

  // ✅ FIXED: Helper methods for colors and icons
  Color _getOrderStatusColor(String status) {
    switch (status) {
      case 'pending':
        return Colors.orange;
      case 'active':
      case 'executing': // ✅ NEW: Add executing status
        return Colors.blue;
      case 'completed':
        return Colors.green;
      case 'paused':
        return Colors.grey;
      case 'failed':
        return Colors.red;
      case 'cancelled':
        return Colors.red.shade300;
      default:
        return Colors.grey;
    }
  }

  IconData _getOrderStatusIcon(String status) {
    switch (status) {
      case 'pending':
        return Icons.pending;
      case 'active':
      case 'executing': // ✅ NEW: Add executing status
        return Icons.play_circle;
      case 'completed':
        return Icons.check_circle;
      case 'paused':
        return Icons.pause_circle;
      case 'failed':
        return Icons.error;
      case 'cancelled':
        return Icons.cancel;
      default:
        return Icons.help;
    }
  }

  Color _getStationTypeColor(String type) {
    switch (type) {
      case 'pickup':
        return Colors.green;
      case 'drop':
        return Colors.blue;
      case 'charging':
        return Colors.orange;
      case 'waypoint':
        return Colors.purple;
      default:
        return Colors.grey;
    }
  }

  IconData _getStationTypeIcon(String type) {
    switch (type) {
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
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('AMR Fleet Dashboard'),
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
      // case 2: // Orders tab
      //   return FloatingActionButton.extended(
      //     onPressed: () => _showSimpleCoordinateOrderCreator(), // ✅ SIMPLIFIED
      //     icon: Icon(Icons.add),
      //     label: Text('Create Order'),
      //     backgroundColor: Colors.green,
      //     heroTag: "create_order",
      //   );
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

  // ✅ SIMPLIFIED: Orders tab with coordinate order creation
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
                  Icon(Icons.assignment,
                      color: Colors.green.shade700, size: 28),
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
                        // Text(
                        //   'Coordinate orders are executed automatically in sequence',
                        //   style: TextStyle(
                        //     fontSize: 14,
                        //     color: Colors.green.shade600,
                        //   ),
                        // ),
                      ],
                    ),
                  ),
                  Row(
                    children: [
                      ElevatedButton.icon(
                        onPressed:
                            _showSimpleCoordinateOrderCreator, // ✅ SIMPLIFIED
                        icon: Icon(Icons.touch_app, size: 18),
                        label: Text('Create Coordinate Order'),
                        style: ElevatedButton.styleFrom(
                          backgroundColor: Colors.green,
                          foregroundColor: Colors.white,
                        ),
                      ),
                      SizedBox(width: 8),
                      // ElevatedButton.icon(
                      //   onPressed: _navigateToCompleteFleetManagement,
                      //   icon: Icon(Icons.dashboard, size: 18),
                      //   label: Text('Fleet Mgmt'),
                      //   style: ElevatedButton.styleFrom(
                      //     backgroundColor: Colors.purple,
                      //     foregroundColor: Colors.white,
                      //   ),
                      // ),
                    ],
                  ),
                ],
              ),
              SizedBox(height: 12),
              _buildOrderStatsRow(),
            ],
          ),
        ),

        // Orders table widget
        Expanded(
          child: _buildOrdersList(),
        ),
      ],
    );
  }

  Widget _buildMapsTab() {
    return Column(
      children: [
        // Enhanced maps header
        Container(
          padding: EdgeInsets.all(16),
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [Colors.purple.shade50, Colors.purple.shade100],
            ),
            border: Border(bottom: BorderSide(color: Colors.purple.shade200)),
          ),
          child: Column(
            children: [
              Row(
                children: [
                  Icon(Icons.map, color: Colors.purple.shade700, size: 28),
                  SizedBox(width: 12),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          'Map Management Hub',
                          style: TextStyle(
                            fontSize: 20,
                            fontWeight: FontWeight.bold,
                            color: Colors.purple.shade700,
                          ),
                        ),
                        Text(
                          'Manage maps for coordinate-based order creation',
                          style: TextStyle(
                            fontSize: 14,
                            color: Colors.purple.shade600,
                          ),
                        ),
                      ],
                    ),
                  ),
                  Row(
                    children: [
                      ElevatedButton.icon(
                        onPressed: _refreshDashboard,
                        icon: Icon(Icons.refresh, size: 18),
                        label: Text('Refresh'),
                        style: ElevatedButton.styleFrom(
                          backgroundColor: Colors.blue,
                          foregroundColor: Colors.white,
                        ),
                      ),
                      SizedBox(width: 8),
                      ElevatedButton.icon(
                        onPressed: () => Navigator.push(
                          context,
                          MaterialPageRoute(
                              builder: (context) => EnhancedMapPage()),
                        ),
                        icon: Icon(Icons.edit, size: 18),
                        label: Text('Edit Maps'),
                        style: ElevatedButton.styleFrom(
                          backgroundColor: Colors.purple,
                          foregroundColor: Colors.white,
                        ),
                      ),
                    ],
                  ),
                ],
              ),
              SizedBox(height: 12),
              _buildMapStatsRow(),
            ],
          ),
        ),

        // Maps content
        Expanded(
          child: _buildMapsList(),
        ),
      ],
    );
  }

  // Keep all your existing build methods - they remain the same
  Widget _buildOverviewCards() {
    // Device statistics
    final totalDevices = _connectedDevices.length;
    final onlineDevices =
        _connectedDevices.where((d) => d['status'] == 'connected').length;

    // ✅ UPDATED: Calculate order statistics from loaded orders in real-time
    final allOrders = <Map<String, dynamic>>[];
    _deviceOrders.forEach((deviceId, orders) {
      allOrders.addAll(orders);
    });

    final totalOrders = allOrders.length;
    final activeOrders = allOrders
        .where((o) => o['status'] == 'active' || o['status'] == 'executing')
        .length;

    return LayoutBuilder(
      builder: (context, constraints) {
        final isWideScreen = constraints.maxWidth > 600;

        if (isWideScreen) {
          return Row(
            children: [
              Expanded(
                  child: _buildOverviewCard('Total Devices',
                      totalDevices.toString(), Icons.devices, Colors.blue)),
              SizedBox(width: 12),
              Expanded(
                  child: _buildOverviewCard(
                      'Online',
                      '$onlineDevices/$totalDevices',
                      Icons.wifi,
                      onlineDevices == totalDevices
                          ? Colors.green
                          : Colors.orange)),
              SizedBox(width: 12),
              Expanded(
                  child: _buildOverviewCard('Total Orders',
                      totalOrders.toString(), Icons.list_alt, Colors.orange)),
              SizedBox(width: 12),
              Expanded(
                  child: _buildOverviewCard('Active', activeOrders.toString(),
                      Icons.play_circle, Colors.purple)),
            ],
          );
        } else {
          return Column(
            children: [
              Row(
                children: [
                  Expanded(
                      child: _buildOverviewCard('Devices',
                          totalDevices.toString(), Icons.devices, Colors.blue)),
                  SizedBox(width: 12),
                  Expanded(
                      child: _buildOverviewCard(
                          'Online',
                          '$onlineDevices/$totalDevices',
                          Icons.wifi,
                          onlineDevices == totalDevices
                              ? Colors.green
                              : Colors.orange)),
                ],
              ),
              SizedBox(height: 12),
              Row(
                children: [
                  Expanded(
                      child: _buildOverviewCard(
                          'Orders',
                          totalOrders.toString(),
                          Icons.list_alt,
                          Colors.orange)),
                  SizedBox(width: 12),
                  Expanded(
                      child: _buildOverviewCard(
                          'Active',
                          activeOrders.toString(),
                          Icons.play_circle,
                          Colors.purple)),
                ],
              ),
            ],
          );
        }
      },
    );
  }

  Widget _buildOverviewCard(
      String title, String value, IconData icon, Color color) {
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
                Expanded(
                    child: _buildHealthIndicator('Backend', true, 'Online')),
                SizedBox(width: 16),
                Expanded(
                    child: _buildHealthIndicator('ROS2', true, 'Connected')),
                SizedBox(width: 16),
                Expanded(
                    child: _buildHealthIndicator('WebSocket', true, 'Active')),
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

  // ✅ SIMPLIFIED: Quick actions for coordinate orders only
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
                  'Create Coordinate Order',
                  Icons.touch_app,
                  Colors.green,
                  _showSimpleCoordinateOrderCreator, // ✅ SIMPLIFIED
                ),
                // _buildQuickActionButton(
                //   'Fleet Management',
                //   Icons.dashboard,
                //   Colors.purple,
                //   _navigateToCompleteFleetManagement,
                // ),
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

  Widget _buildQuickActionButton(
      String label, IconData icon, Color color, VoidCallback onPressed) {
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
    // ✅ UPDATED: Calculate stats from actual loaded orders for real-time accuracy
    final allOrders = <Map<String, dynamic>>[];
    _deviceOrders.forEach((deviceId, orders) {
      allOrders.addAll(orders);
    });

    // Calculate counts by status
    final pendingCount =
        allOrders.where((o) => o['status'] == 'pending').length;
    final activeCount = allOrders
        .where((o) => o['status'] == 'active' || o['status'] == 'executing')
        .length;
    final completedCount =
        allOrders.where((o) => o['status'] == 'completed').length;
    final failedCount = allOrders
        .where((o) => o['status'] == 'failed' || o['status'] == 'cancelled')
        .length;

    return Container(
      padding: EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.8),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        children: [
          Expanded(
              child:
                  _buildOrderStatItem('Pending', pendingCount, Colors.orange)),
          _buildStatDivider(),
          Expanded(
              child: _buildOrderStatItem('Active', activeCount, Colors.blue)),
          _buildStatDivider(),
          Expanded(
              child: _buildOrderStatItem(
                  'Completed', completedCount, Colors.green)),
          _buildStatDivider(),
          Expanded(
              child: _buildOrderStatItem('Failed', failedCount, Colors.red)),
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

  Widget _buildMapStatsRow() {
    final totalMaps = _availableMaps.length;
    final mapsWithShapes =
        _availableMaps.values.where((map) => map.shapes.isNotEmpty).length;
    final totalShapes =
        _availableMaps.values.fold(0, (sum, map) => sum + map.shapes.length);

    return Container(
      padding: EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.8),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        children: [
          Expanded(
              child: _buildOrderStatItem('Maps', totalMaps, Colors.purple)),
          _buildStatDivider(),
          Expanded(
              child: _buildOrderStatItem(
                  'With Shapes', mapsWithShapes, Colors.blue)),
          _buildStatDivider(),
          Expanded(
              child: _buildOrderStatItem(
                  'Total Shapes', totalShapes, Colors.green)),
          _buildStatDivider(),
          Expanded(
              child: _buildOrderStatItem(
                  'Devices', _connectedDevices.length, Colors.orange)),
        ],
      ),
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
            DropdownMenuItem(value: 'with_maps', child: Text('With Maps')),
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
          filteredDevices = _connectedDevices
              .where((d) => d['status'] == 'connected')
              .toList();
          break;
        case 'with_orders':
          filteredDevices = _connectedDevices
              .where((d) => (_deviceOrders[d['id']]?.length ?? 0) > 0)
              .toList();
          break;
        case 'with_maps':
          filteredDevices = _connectedDevices
              .where((d) => _availableMaps.containsKey(d['id']))
              .toList();
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
              _connectedDevices.isEmpty
                  ? 'No Devices Connected'
                  : 'No devices match the filter',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            Text(_connectedDevices.isEmpty
                ? 'Connect your AMR devices to get started'
                : 'Try changing the filter'),
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
    final activeOrders = _deviceOrders[deviceId]
            ?.where(
                (o) => o['status'] == 'active' || o['status'] == 'executing')
            .length ??
        0;

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
                          style:
                              TextStyle(color: Colors.grey[600], fontSize: 12),
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
                      icon: Icon(Icons.touch_app, size: 16), // ✅ UPDATED icon
                      label: Text('Create Coordinate Order'), // ✅ UPDATED label
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
          'deviceName': _connectedDevices.firstWhere((d) => d['id'] == deviceId,
              orElse: () => {'name': deviceId})['name']
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
            Icon(Icons.touch_app,
                size: 64, color: Colors.grey), // ✅ UPDATED icon
            SizedBox(height: 16),
            Text(
              'No Coordinate Orders Created Yet',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            Text(
                'Create your first coordinate order by tapping on the map'), // ✅ UPDATED text
            SizedBox(height: 16),
            ElevatedButton.icon(
              onPressed: _showSimpleCoordinateOrderCreator, // ✅ SIMPLIFIED
              icon: Icon(Icons.touch_app),
              label: Text('Create Coordinate Order'),
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

    // ✅ UPDATED: Handle both coordinates and waypoints
    final coordinates = order['coordinates'] as List? ?? [];
    final waypoints = order['waypoints'] as List? ?? [];
    final totalPoints =
        coordinates.isNotEmpty ? coordinates.length : waypoints.length;

    final progress = order['progress'] as Map<String, dynamic>? ?? {};

    return Card(
      margin: EdgeInsets.symmetric(vertical: 4),
      child: ExpansionTile(
        leading: CircleAvatar(
          backgroundColor: statusColor,
          child: Icon(_getOrderStatusIcon(status), color: Colors.white),
        ),
        title: Row(
          children: [
            Expanded(
              child: Text(
                order['name'] ?? 'Order',
                style: TextStyle(fontWeight: FontWeight.bold),
              ),
            ),
            if (coordinates.isNotEmpty)
              Container(
                padding: EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                decoration: BoxDecoration(
                  color: Colors.blue.shade100,
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: Colors.blue.shade300),
                ),
                child: Text(
                  'COORDINATE ORDER',
                  style: TextStyle(
                    fontSize: 10,
                    fontWeight: FontWeight.bold,
                    color: Colors.blue.shade700,
                  ),
                ),
              ),
          ],
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Device: ${order['deviceName'] ?? order['deviceId']}'),
            Text(
                '${coordinates.isNotEmpty ? 'Coordinates' : 'Waypoints'}: $totalPoints • ${progress['percentage'] ?? 0}% complete'),
            SizedBox(height: 4),
            LinearProgressIndicator(
              value: (progress['percentage'] ?? 0) / 100.0,
              backgroundColor: Colors.grey.shade300,
              valueColor: AlwaysStoppedAnimation<Color>(statusColor),
            ),
          ],
        ),
        trailing: _buildOrderActionButton(order),
        children: [
          Container(
            padding: EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                if (coordinates.isNotEmpty) ...[
                  Text(
                    'Coordinate Sequence:',
                    style: TextStyle(fontWeight: FontWeight.bold),
                  ),
                  SizedBox(height: 8),
                  ...coordinates.asMap().entries.map((entry) {
                    final index = entry.key;
                    final coordinate = entry.value;
                    return _buildCoordinateSequenceItem(index + 1, coordinate,
                        index < (order['currentCoordinate'] ?? 0));
                  }),
                  SizedBox(height: 16),
                ] else if (waypoints.isNotEmpty) ...[
                  Text(
                    'Order Waypoints:',
                    style: TextStyle(fontWeight: FontWeight.bold),
                  ),
                  SizedBox(height: 8),
                  ...waypoints.asMap().entries.map((entry) {
                    final index = entry.key;
                    final waypoint = entry.value;
                    return _buildWaypointSequenceItem(index + 1, waypoint,
                        index < (order['currentWaypoint'] ?? 0));
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

  // ✅ NEW: Build coordinate sequence item
  Widget _buildCoordinateSequenceItem(
      int step, Map<String, dynamic> coordinate, bool isCompleted) {
    final name = coordinate['name'] ?? 'Coordinate $step';
    final x = coordinate['x']?.toStringAsFixed(2) ?? '0.00';
    final y = coordinate['y']?.toStringAsFixed(2) ?? '0.00';

    return Container(
      margin: EdgeInsets.symmetric(vertical: 2),
      padding: EdgeInsets.all(12),
      decoration: BoxDecoration(
        color:
            isCompleted ? Colors.green.withOpacity(0.2) : Colors.grey.shade50,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(
          color: isCompleted ? Colors.green : Colors.grey.shade300,
          width: isCompleted ? 2 : 1,
        ),
      ),
      child: Row(
        children: [
          Container(
            width: 28,
            height: 28,
            decoration: BoxDecoration(
              color: isCompleted ? Colors.green : Colors.grey.shade400,
              shape: BoxShape.circle,
            ),
            child: Center(
              child: isCompleted
                  ? Icon(Icons.check, color: Colors.white, size: 16)
                  : Text(
                      step.toString(),
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 12,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
            ),
          ),
          SizedBox(width: 12),
          Icon(Icons.place, color: Colors.blue, size: 20),
          SizedBox(width: 8),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  name,
                  style: TextStyle(
                    fontWeight: FontWeight.w600,
                    color: isCompleted ? Colors.green : Colors.black87,
                  ),
                ),
                Text(
                  'COORDINATE ($x, $y)',
                  style: TextStyle(
                    fontSize: 11,
                    color: Colors.blue,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildWaypointSequenceItem(
      int step, Map<String, dynamic> waypoint, bool isCompleted) {
    final type = waypoint['type'] ?? 'waypoint';
    final name = waypoint['name'] ?? 'Station $step';
    final color = _getStationTypeColor(type);
    final icon = _getStationTypeIcon(type);

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
                      step.toString(),
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
        ],
      ),
    );
  }

  Widget _buildOrderActionButtons(Map<String, dynamic> order) {
    final status = order['status'] ?? 'pending';
    final coordinates = order['coordinates'] as List? ?? [];
    final isCoordinateOrder = coordinates.isNotEmpty;
    final orderId = order['id']?.toString() ?? '';
    final isDeleting = _deletingOrders.contains(orderId);
    final canDelete = status == 'pending' ||
        status == 'completed' ||
        status == 'failed' ||
        status == 'cancelled';

    return Column(
      children: [
        // Primary action buttons row
        Row(
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
                  onPressed: isCoordinateOrder
                      ? () => _startCoordinateOrder(order)
                      : () => _executeOrder(order),
                  icon: Icon(Icons.play_arrow),
                  label: Text(isCoordinateOrder ? 'Start' : 'Execute'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.green,
                    foregroundColor: Colors.white,
                  ),
                ),
              ),
            if (status == 'active' || status == 'executing')
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: isCoordinateOrder
                      ? () => _stopCoordinateOrderExecution(order)
                      : () => _pauseOrder(order),
                  icon: Icon(isCoordinateOrder ? Icons.stop : Icons.pause),
                  label: Text(isCoordinateOrder ? 'Stop' : 'Pause'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor:
                        isCoordinateOrder ? Colors.red : Colors.orange,
                    foregroundColor: Colors.white,
                  ),
                ),
              ),
            if (status == 'paused')
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: isCoordinateOrder
                      ? () => _startCoordinateOrder(order)
                      : () => _executeOrder(order),
                  icon: Icon(Icons.play_arrow),
                  label: Text('Resume'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.blue,
                    foregroundColor: Colors.white,
                  ),
                ),
              ),
            if (status == 'failed' && isCoordinateOrder)
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: () => _restartCoordinateOrder(order['id']),
                  icon: Icon(Icons.refresh),
                  label: Text('Restart'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.orange,
                    foregroundColor: Colors.white,
                  ),
                ),
              ),
          ],
        ),

        // Delete button row (only for eligible orders)
        if (canDelete) ...[
          SizedBox(height: 8),
          SizedBox(
            width: double.infinity,
            child: OutlinedButton.icon(
              onPressed:
                  isDeleting ? null : () => _showDeleteOrderDialog(order),
              icon: isDeleting
                  ? SizedBox(
                      width: 16,
                      height: 16,
                      child: CircularProgressIndicator(
                        strokeWidth: 2,
                        valueColor: AlwaysStoppedAnimation<Color>(Colors.red),
                      ),
                    )
                  : Icon(Icons.delete, color: Colors.red),
              label: Text(
                isDeleting ? 'Deleting...' : 'Delete Order',
                style: TextStyle(color: isDeleting ? Colors.grey : Colors.red),
              ),
              style: OutlinedButton.styleFrom(
                side: BorderSide(color: isDeleting ? Colors.grey : Colors.red),
                backgroundColor:
                    isDeleting ? Colors.grey.shade50 : Colors.red.shade50,
              ),
            ),
          ),
        ],
      ],
    );
  }

  // ✅ NEW: Single action button for order card trailing
  Widget _buildOrderActionButton(Map<String, dynamic> order) {
    final status = order['status'] ?? 'pending';
    final statusColor = _getOrderStatusColor(status);
    final coordinates = order['coordinates'] as List? ?? [];
    final isCoordinateOrder = coordinates.isNotEmpty;
    final orderId = order['id']?.toString() ?? '';
    final isDeleting = _deletingOrders.contains(orderId);
    final canDelete = status == 'pending' ||
        status == 'completed' ||
        status == 'failed' ||
        status == 'cancelled';

    // Show status badge and quick action button
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
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
        SizedBox(width: 8),
        if (status == 'pending')
          ElevatedButton.icon(
            onPressed: isCoordinateOrder
                ? () => _startCoordinateOrder(order)
                : () => _executeOrder(order),
            icon: Icon(Icons.play_arrow, size: 16),
            label: Text('Start', style: TextStyle(fontSize: 12)),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.green,
              foregroundColor: Colors.white,
              padding: EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              minimumSize: Size(70, 32),
            ),
          ),
        if (status == 'executing')
          ElevatedButton.icon(
            onPressed: isCoordinateOrder
                ? () => _stopCoordinateOrderExecution(order)
                : () => _pauseOrder(order),
            icon: Icon(isCoordinateOrder ? Icons.stop : Icons.pause, size: 16),
            label: Text(isCoordinateOrder ? 'Stop' : 'Pause',
                style: TextStyle(fontSize: 12)),
            style: ElevatedButton.styleFrom(
              backgroundColor: isCoordinateOrder ? Colors.red : Colors.orange,
              foregroundColor: Colors.white,
              padding: EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              minimumSize: Size(70, 32),
            ),
          ),
        if (canDelete) ...[
          SizedBox(width: 4),
          IconButton(
            onPressed: isDeleting ? null : () => _showDeleteOrderDialog(order),
            icon: isDeleting
                ? SizedBox(
                    width: 16,
                    height: 16,
                    child: CircularProgressIndicator(
                      strokeWidth: 2,
                      valueColor: AlwaysStoppedAnimation<Color>(Colors.red),
                    ),
                  )
                : Icon(Icons.delete, color: Colors.red, size: 20),
            padding: EdgeInsets.all(4),
            constraints: BoxConstraints(minWidth: 32, minHeight: 32),
            tooltip: isDeleting ? 'Deleting...' : 'Delete Order',
            style: IconButton.styleFrom(
              backgroundColor:
                  isDeleting ? Colors.grey.shade50 : Colors.red.shade50,
              shape: CircleBorder(),
            ),
          ),
        ],
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
            Text(
                'Create maps for coordinate-based order creation'), // ✅ UPDATED text
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

    final shapeCounts = <String, int>{};
    for (final shape in mapData.shapes) {
      shapeCounts[shape.type] = (shapeCounts[shape.type] ?? 0) + 1;
    }

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
            Text('Available for coordinate order creation'), // ✅ UPDATED text
            if (shapeCounts.isNotEmpty)
              Wrap(
                spacing: 4,
                children: shapeCounts.entries.map((entry) {
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
              tooltip: 'Edit Map',
            ),
            IconButton(
              onPressed: () => _showCreateOrderForDevice(deviceId),
              icon: Icon(Icons.touch_app), // ✅ UPDATED icon
              tooltip: 'Create Coordinate Order', // ✅ UPDATED tooltip
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
        if (order['status'] == 'active' ||
            order['status'] == 'pending' ||
            order['status'] == 'executing') {
          activeOrders.add({
            ...order,
            'deviceId': deviceId,
            'deviceName': _connectedDevices.firstWhere(
                (d) => d['id'] == deviceId,
                orElse: () => {'name': deviceId})['name']
          });
        }
      }
    });

    if (activeOrders.isEmpty) {
      return Container(
        padding: EdgeInsets.all(16),
        decoration: BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: Colors.green.shade300),
        ),
        child: Row(
          children: [
            Icon(Icons.check_circle, color: Colors.green, size: 32),
            SizedBox(width: 12),
            Expanded(
              child: Text(
                'All systems operational. No active orders at the moment.',
                style: TextStyle(fontSize: 14, color: Colors.green.shade800),
              ),
            ),
          ],
        ),
      );
    }

    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Active Orders Summary',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 12),
            ...activeOrders.map((order) {
              final device = _connectedDevices.firstWhere(
                  (d) => d['id'] == order['deviceId'],
                  orElse: () =>
                      {'name': order['deviceId'], 'id': order['deviceId']});
              return Container(
                margin: EdgeInsets.symmetric(vertical: 4),
                padding: EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: Colors.blue.shade50,
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: Colors.blue.shade300),
                ),
                child: Row(
                  children: [
                    Container(
                      width: 40,
                      height: 40,
                      decoration: BoxDecoration(
                        color: Colors.blue.shade100,
                        shape: BoxShape.circle,
                      ),
                      child: Icon(Icons.assignment,
                          color: Colors.blue.shade700, size: 20),
                    ),
                    SizedBox(width: 12),
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            order['name'] ?? 'Order',
                            style: TextStyle(
                                fontWeight: FontWeight.bold, fontSize: 14),
                          ),
                          Text(
                            'Device: ${device['name'] ?? order['deviceId']}',
                            style: TextStyle(
                                color: Colors.grey[700], fontSize: 12),
                          ),
                          Text(
                            'Status: ${order['status'].toString().capitalize()}',
                            style: TextStyle(
                                color: Colors.blue.shade800,
                                fontWeight: FontWeight.w500,
                                fontSize: 12),
                          ),
                        ],
                      ),
                    ),
                    SizedBox(width: 8),
                    IconButton(
                      onPressed: () => _showOrderDetails(order),
                      icon:
                          Icon(Icons.info_outline, color: Colors.blue.shade700),
                      tooltip: 'View Details',
                    ),
                  ],
                ),
              );
            }).toList(),
          ],
        ),
      ),
    );
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
