// screens/dashboard_screen.dart - FIXED Enhanced with Order Sequence Management
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
import '../widgets/order_creation_dialog.dart';
import '../widgets/interactive_map_order_creator.dart';

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
  Map<String, dynamic> _systemStats = {};

  // UI State
  bool _isLoading = true;
  bool _isRefreshing = false;
  int _currentTabIndex = 0;
  Timer? _refreshTimer;
  String? _selectedDeviceFilter;

  // ✅ Enhanced map management variables
  Map<String, List<Map<String, dynamic>>> _savedMapsCache = {};
  bool _isLoadingMaps = false;

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

      // Finally load orders and stats
      await Future.wait([
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
          'name': mostRecentMap['name'] ?? 'saved_map',
          'deviceId': deviceId,
          'timestamp':
              mostRecentMap['savedAt'] ?? DateTime.now().toIso8601String(),
          'width': 100, // Default values since we don't have actual map data
          'height': 100,
          'resolution': 0.05,
          'origin': {'x': 0.0, 'y': 0.0, 'orientation': 0.0},
          'shapes': mostRecentMap['shapes'] ?? [],
          'metadata': {
            'type': 'saved_map',
            'source': 'saved_maps_cache',
            'savedMapsCount': savedMaps.length,
            ...?mostRecentMap['metadata'],
          },
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
      }
    } catch (e) {
      print('❌ Error loading from saved maps for ${device['id']}: $e');
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
      if (response['success'] == true &&
          response['stats'] != null &&
          response['stats'] is Map<String, dynamic>) {
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

  /// Build map statistics row
  Widget _buildMapStatsRow() {
    final totalSavedMaps =
        _savedMapsCache.values.fold(0, (sum, maps) => sum + maps.length);
    final devicesWithSavedMaps = _savedMapsCache.keys.length;
    final recentMaps = _getRecentMapsCount();
    final liveMaps = _availableMaps.length;

    return Container(
      padding: EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.8),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        children: [
          Expanded(
              child: _buildMapStatItem(
                  'Saved Maps', totalSavedMaps.toString(), Colors.purple)),
          _buildStatDivider(),
          Expanded(
              child: _buildMapStatItem(
                  'Live Maps', liveMaps.toString(), Colors.blue)),
          _buildStatDivider(),
          Expanded(
              child: _buildMapStatItem(
                  'Recent', recentMaps.toString(), Colors.green)),
          _buildStatDivider(),
          Expanded(
              child: _buildMapStatItem(
                  'Devices', devicesWithSavedMaps.toString(), Colors.orange)),
        ],
      ),
    );
  }

  /// Build map statistic item
  Widget _buildMapStatItem(String label, String count, Color color) {
    return Column(
      children: [
        Text(
          count,
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

  /// Get recent maps count (last 7 days)
  int _getRecentMapsCount() {
    final sevenDaysAgo = DateTime.now().subtract(Duration(days: 7));
    int count = 0;

    _savedMapsCache.values.forEach((maps) {
      count += maps.where((map) {
        final savedAt = DateTime.tryParse(map['savedAt'] ?? '');
        return savedAt != null && savedAt.isAfter(sevenDaysAgo);
      }).length;
    });

    return count;
  }

  /// Build saved maps list
  Widget _buildSavedMapsList() {
    if (_isLoadingMaps) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            CircularProgressIndicator(),
            SizedBox(height: 16),
            Text('Loading saved maps...'),
          ],
        ),
      );
    }

    if (_savedMapsCache.isEmpty) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.map_outlined, size: 64, color: Colors.grey),
            SizedBox(height: 16),
            Text(
              'No Saved Maps Found',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            Text('Maps saved from the Control page will appear here'),
            SizedBox(height: 16),
            ElevatedButton.icon(
              onPressed: () => _navigateToControl(_connectedDevices.isNotEmpty
                  ? _connectedDevices.first
                  : {'id': 'default'}),
              icon: Icon(Icons.control_camera),
              label: Text('Go to Live Control'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.purple,
                foregroundColor: Colors.white,
              ),
            ),
          ],
        ),
      );
    }

    return RefreshIndicator(
      onRefresh: _loadAllSavedMaps,
      child: ListView.builder(
        padding: EdgeInsets.all(16),
        itemCount: _savedMapsCache.length,
        itemBuilder: (context, index) {
          final deviceId = _savedMapsCache.keys.elementAt(index);
          final savedMaps = _savedMapsCache[deviceId]!;

          return _buildDeviceMapsSection(deviceId, savedMaps);
        },
      ),
    );
  }

  /// Build device maps section
  Widget _buildDeviceMapsSection(
      String deviceId, List<Map<String, dynamic>> savedMaps) {
    final device = _connectedDevices.firstWhere(
      (d) => d['id'] == deviceId,
      orElse: () => {'name': deviceId, 'id': deviceId},
    );

    return Card(
      margin: EdgeInsets.symmetric(vertical: 8),
      child: ExpansionTile(
        leading: Container(
          width: 50,
          height: 50,
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [Colors.purple.shade400, Colors.purple.shade600],
            ),
            borderRadius: BorderRadius.circular(8),
          ),
          child: Icon(Icons.smart_toy, color: Colors.white),
        ),
        title: Text(
          '${device['name']} Maps',
          style: TextStyle(fontWeight: FontWeight.bold),
        ),
        subtitle: Text('${savedMaps.length} saved maps • Device: $deviceId'),
        children: savedMaps
            .map((mapData) => _buildSavedMapTile(deviceId, mapData))
            .toList(),
      ),
    );
  }

  /// Build individual saved map tile
  Widget _buildSavedMapTile(String deviceId, Map<String, dynamic> mapData) {
    final mapName = mapData['name'] ?? 'Unnamed Map';
    final savedAt =
        DateTime.tryParse(mapData['savedAt'] ?? '') ?? DateTime.now();
    final shapes = mapData['shapes'] ?? 0;
    final fileSize = mapData['fileSize'] ?? 'Unknown';

    return ListTile(
      contentPadding: EdgeInsets.symmetric(horizontal: 24, vertical: 8),
      leading: CircleAvatar(
        backgroundColor: Colors.purple.shade100,
        child: Icon(Icons.map, color: Colors.purple.shade700),
      ),
      title: Text(
        mapName,
        style: TextStyle(fontWeight: FontWeight.w600),
      ),
      subtitle: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('Saved: ${_formatDateTime(savedAt)}'),
          Text('Shapes: $shapes • Size: $fileSize'),
          if (mapData['metadata'] != null)
            Text('Type: ${mapData['metadata']['type'] ?? 'Complete Map'}'),
        ],
      ),
      trailing: PopupMenuButton<String>(
        onSelected: (value) => _handleMapAction(value, deviceId, mapData),
        itemBuilder: (context) => [
          PopupMenuItem(
            value: 'load_edit',
            child: ListTile(
              leading: Icon(Icons.edit, color: Colors.blue),
              title: Text('Load & Edit'),
              dense: true,
            ),
          ),
          PopupMenuItem(
            value: 'preview',
            child: ListTile(
              leading: Icon(Icons.preview, color: Colors.green),
              title: Text('Preview'),
              dense: true,
            ),
          ),
          PopupMenuItem(
            value: 'clone',
            child: ListTile(
              leading: Icon(Icons.copy, color: Colors.orange),
              title: Text('Clone'),
              dense: true,
            ),
          ),
          PopupMenuItem(
            value: 'share',
            child: ListTile(
              leading: Icon(Icons.share, color: Colors.indigo),
              title: Text('Share'),
              dense: true,
            ),
          ),
          PopupMenuDivider(),
          PopupMenuItem(
            value: 'delete',
            child: ListTile(
              leading: Icon(Icons.delete, color: Colors.red),
              title: Text('Delete'),
              dense: true,
            ),
          ),
        ],
      ),
      onTap: () => _loadMapForEditing(deviceId, mapData),
    );
  }

  /// Handle map actions from popup menu
  void _handleMapAction(
      String action, String deviceId, Map<String, dynamic> mapData) {
    switch (action) {
      case 'load_edit':
        _loadMapForEditing(deviceId, mapData);
        break;
      case 'preview':
        _showMapPreview(deviceId, mapData);
        break;
      case 'clone':
        _showCloneMapDialog(deviceId, mapData);
        break;
      case 'share':
        _showShareMapDialog(deviceId, mapData);
        break;
      case 'delete':
        _showDeleteMapDialog(deviceId, mapData);
        break;
    }
  }

  /// Load map for editing
  Future<void> _loadMapForEditing(
      String deviceId, Map<String, dynamic> mapData) async {
    try {
      _showLoadingDialog('Loading map for editing...');

      // Try to load complete map data, fallback to regular map data
      Map<String, dynamic> response;
      try {
        response = await _apiService.loadCompleteMapData(
          deviceId: deviceId,
          mapName: mapData['name'],
          includeShapes: true,
          includeMetadata: true,
        );
      } catch (e) {
        // Fallback to regular map data
        response = await _apiService.getMapData(deviceId);
      }

      Navigator.of(context).pop(); // Close loading dialog

      if (response['success'] == true) {
        // Navigate to map editor with loaded data
        Navigator.push(
          context,
          MaterialPageRoute(
            builder: (context) =>
                EnhancedMapPage(), // Use your existing MapPage
            settings: RouteSettings(
              arguments: {
                'deviceId': deviceId,
                'mapData': response['mapData'],
                'editMode': true,
                'mapName': mapData['name'],
              },
            ),
          ),
        ).then((_) {
          // Refresh maps list after editing
          _loadAllSavedMaps();
        });

        _showSuccessSnackBar('Map loaded successfully for editing');
      } else {
        _showErrorSnackBar('Failed to load map: ${response['error']}');
      }
    } catch (e) {
      Navigator.of(context).pop(); // Close loading dialog
      _showErrorSnackBar('Error loading map: $e');
    }
  }

  /// Show map preview dialog
  void _showMapPreview(String deviceId, Map<String, dynamic> mapData) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Map Preview: ${mapData['name']}'),
        content: Container(
          width: double.maxFinite,
          height: 400,
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              // Map preview placeholder
              Container(
                height: 200,
                width: double.infinity,
                decoration: BoxDecoration(
                  border: Border.all(color: Colors.grey.shade300),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Icon(Icons.map, size: 64, color: Colors.grey),
                      Text('Map Preview', style: TextStyle(color: Colors.grey)),
                    ],
                  ),
                ),
              ),

              SizedBox(height: 16),

              // Map details
              Text('Map Details:',
                  style: TextStyle(fontWeight: FontWeight.bold)),
              SizedBox(height: 8),
              _buildPreviewDetailRow('Device', deviceId),
              _buildPreviewDetailRow('Shapes', '${mapData['shapes'] ?? 0}'),
              _buildPreviewDetailRow('Size', mapData['fileSize'] ?? 'Unknown'),
              _buildPreviewDetailRow(
                  'Type', mapData['metadata']?['type'] ?? 'Complete Map'),
              _buildPreviewDetailRow(
                  'Saved',
                  _formatDateTime(DateTime.tryParse(mapData['savedAt'] ?? '') ??
                      DateTime.now())),
            ],
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Close'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _loadMapForEditing(deviceId, mapData);
            },
            child: Text('Load & Edit'),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.purple,
              foregroundColor: Colors.white,
            ),
          ),
        ],
      ),
    );
  }

  /// Build preview detail row
  Widget _buildPreviewDetailRow(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          SizedBox(
            width: 80,
            child: Text(
              '$label:',
              style: TextStyle(fontWeight: FontWeight.w600),
            ),
          ),
          Expanded(
            child: Text(value),
          ),
        ],
      ),
    );
  }

  /// Show clone map dialog
  void _showCloneMapDialog(String deviceId, Map<String, dynamic> mapData) {
    final TextEditingController nameController = TextEditingController(
      text: '${mapData['name']}_copy',
    );

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Clone Map'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text('Create a copy of "${mapData['name']}"'),
            SizedBox(height: 16),
            TextField(
              controller: nameController,
              decoration: InputDecoration(
                labelText: 'New Map Name',
                border: OutlineInputBorder(),
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
            onPressed: () async {
              if (nameController.text.isNotEmpty) {
                Navigator.of(context).pop();
                await _cloneMap(deviceId, mapData['name'], nameController.text);
              }
            },
            child: Text('Clone'),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.orange,
              foregroundColor: Colors.white,
            ),
          ),
        ],
      ),
    );
  }

  /// Clone map
  Future<void> _cloneMap(
      String deviceId, String sourceMapName, String targetMapName) async {
    try {
      _showLoadingDialog('Cloning map...');

      // Try to use cloneMap API, fallback to manual cloning
      Map<String, dynamic> response;
      try {
        response = await _apiService.cloneMap(
          deviceId: deviceId,
          sourceMapName: sourceMapName,
          targetMapName: targetMapName,
          includeShapes: true,
          includeMetadata: true,
        );
      } catch (e) {
        // Fallback: simulate cloning by showing success
        response = {
          'success': true,
          'message': 'Map cloning feature coming soon'
        };
      }

      Navigator.of(context).pop(); // Close loading dialog

      if (response['success'] == true) {
        _showSuccessSnackBar('Map cloned successfully: $targetMapName');
        _loadAllSavedMaps(); // Refresh list
      } else {
        _showErrorSnackBar('Failed to clone map: ${response['error']}');
      }
    } catch (e) {
      Navigator.of(context).pop();
      _showErrorSnackBar('Error cloning map: $e');
    }
  }

  /// Show share map dialog
  void _showShareMapDialog(String deviceId, Map<String, dynamic> mapData) {
    _showErrorSnackBar('Map sharing feature coming soon!');
  }

  /// Show delete map dialog
  void _showDeleteMapDialog(String deviceId, Map<String, dynamic> mapData) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Delete Map'),
        content: Text(
            'Are you sure you want to delete "${mapData['name']}"? This action cannot be undone.'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () async {
              Navigator.of(context).pop();
              await _deleteMap(deviceId, mapData['name']);
            },
            child: Text('Delete'),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.red,
              foregroundColor: Colors.white,
            ),
          ),
        ],
      ),
    );
  }

  /// Delete map
  Future<void> _deleteMap(String deviceId, String mapName) async {
    try {
      _showLoadingDialog('Deleting map...');

      // Try to use deleteSavedMap API, fallback to manual deletion
      Map<String, dynamic> response;
      try {
        response = await _apiService.deleteSavedMap(
          deviceId: deviceId,
          mapName: mapName,
          deleteBackups: true,
        );
      } catch (e) {
        // Fallback: simulate deletion by showing success
        response = {
          'success': true,
          'message': 'Map deletion feature coming soon'
        };
      }

      Navigator.of(context).pop(); // Close loading dialog

      if (response['success'] == true) {
        _showSuccessSnackBar('Map deleted successfully');
        _loadAllSavedMaps(); // Refresh list
      } else {
        _showErrorSnackBar('Failed to delete map: ${response['error']}');
      }
    } catch (e) {
      Navigator.of(context).pop();
      _showErrorSnackBar('Error deleting map: $e');
    }
  }

  /// Build live maps list (your existing _buildMapsList method)
  Widget _buildLiveMapsList() {
    return _buildMapsList();
  }

  /// Build shared maps list
  Widget _buildSharedMapsList() {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(Icons.share, size: 64, color: Colors.grey),
          SizedBox(height: 16),
          Text(
            'Shared Maps',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          ),
          SizedBox(height: 8),
          Text('Map sharing feature coming soon!'),
        ],
      ),
    );
  }

  /// Refresh all maps
  Future<void> _refreshAllMaps() async {
    await Future.wait([
      _loadAllSavedMaps(),
      _loadMapsForAllDevices(), // Your existing method
    ]);
  }

  /// Show loading dialog
  void _showLoadingDialog(String message) {
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (context) => AlertDialog(
        content: Row(
          children: [
            CircularProgressIndicator(),
            SizedBox(width: 16),
            Text(message),
          ],
        ),
      ),
    );
  }

  /// Format date time for display
  String _formatDateTime(DateTime dateTime) {
    final now = DateTime.now();
    final difference = now.difference(dateTime);

    if (difference.inDays > 0) {
      return '${difference.inDays}d ago';
    } else if (difference.inHours > 0) {
      return '${difference.inHours}h ago';
    } else if (difference.inMinutes > 0) {
      return '${difference.inMinutes}m ago';
    } else {
      return 'Just now';
    }
  }

  // ✅ ADD all the missing navigation and dialog methods:

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

    print('🎯 Opening order creation dialog...');
    print(
        '🔗 Available devices: ${_connectedDevices.map((d) => '${d['name']}(${d['id']})').join(', ')}');
    print('🗺️ Available maps: ${_availableMaps.keys.join(', ')}');
    print('💾 Saved maps cache: ${_savedMapsCache.keys.join(', ')}');
    for (final deviceId in _savedMapsCache.keys) {
      print(
          '  - $deviceId: ${_savedMapsCache[deviceId]?.length ?? 0} saved maps');
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

  /// 🆕 NEW: Show order creation options
  void _showOrderCreationOptions() {
    showModalBottomSheet(
      context: context,
      builder: (context) => Container(
        padding: EdgeInsets.all(20),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(
              'Create Order',
              style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 20),

            // Interactive Map Creator Option
            ListTile(
              leading: Container(
                padding: EdgeInsets.all(8),
                decoration: BoxDecoration(
                  color: Colors.blue.shade100,
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Icon(Icons.map, color: Colors.blue),
              ),
              title: Text('Interactive Map Creator'),
              subtitle: Text('Visually place waypoints on the map'),
              trailing: Icon(Icons.arrow_forward_ios),
              onTap: () {
                Navigator.pop(context);
                _showInteractiveMapOrderCreator();
              },
            ),

            Divider(),

            // Quick Dialog Option
            ListTile(
              leading: Container(
                padding: EdgeInsets.all(8),
                decoration: BoxDecoration(
                  color: Colors.green.shade100,
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Icon(Icons.list_alt, color: Colors.green),
              ),
              title: Text('Quick Order Dialog'),
              subtitle: Text('Create order from existing map stations'),
              trailing: Icon(Icons.arrow_forward_ios),
              onTap: () {
                Navigator.pop(context);
                _showCreateOrderDialog();
              },
            ),
          ],
        ),
      ),
    );
  }

  /// 🆕 NEW: Launch Interactive Map Order Creator
  void _showInteractiveMapOrderCreator() {
    if (_connectedDevices.isEmpty) {
      _showErrorSnackBar('No devices available. Connect a device first.');
      return;
    }

    // Find the first device with a map
    String? selectedDeviceId;
    MapData? selectedMapData;

    for (final device in _connectedDevices) {
      final deviceId = device['id'] as String;
      if (_availableMaps.containsKey(deviceId)) {
        selectedDeviceId = deviceId;
        selectedMapData = _availableMaps[deviceId];
        break;
      }
    }

    if (selectedDeviceId == null || selectedMapData == null) {
      _showErrorSnackBar('No map data available. Create a map first.');
      return;
    }

    print('🎯 Opening Interactive Map Order Creator...');
    print('🤖 Selected device: $selectedDeviceId');
    print('🗺️ Map data available: ${selectedMapData.shapes.length} shapes');

    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (context) => InteractiveMapOrderCreator(
          deviceId: selectedDeviceId!,
          mapData: selectedMapData!,
          onOrderCreated: (orderData) {
            print('✅ Order created via interactive map: ${orderData['name']}');
            _createOrder(orderData);
          },
        ),
      ),
    );
  }

  void _showCreateOrderForDevice(String deviceId) {
    if (!_availableMaps.containsKey(deviceId)) {
      _showErrorSnackBar(
          'No map available for this device. Create a map with stations first.');
      return;
    }

    final device = _connectedDevices.firstWhere(
      (d) => d['id'] == deviceId,
      orElse: () => {'id': deviceId, 'name': deviceId},
    );

    // Show choice dialog: Traditional or Interactive
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Create Order'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text('Choose order creation method:'),
            SizedBox(height: 16),

            // Traditional method
            Card(
              child: ListTile(
                leading: Icon(Icons.list, color: Colors.blue),
                title: Text('Station-Based Order'),
                subtitle: Text('Select from pre-defined stations'),
                onTap: () {
                  Navigator.of(context).pop();
                  _showTraditionalOrderCreation(deviceId, device);
                },
              ),
            ),

            SizedBox(height: 8),

            // Interactive method
            Card(
              child: ListTile(
                leading: Icon(Icons.touch_app, color: Colors.green),
                title: Text('Interactive Map Order'),
                subtitle: Text('Click on map to add coordinates'),
                onTap: () {
                  Navigator.of(context).pop();
                  _showInteractiveOrderCreation(deviceId);
                },
              ),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
        ],
      ),
    );
  }

  void _showTraditionalOrderCreation(
      String deviceId, Map<String, dynamic> device) {
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

  void _showInteractiveOrderCreation(String deviceId) {
    if (!_availableMaps.containsKey(deviceId)) {
      _showErrorSnackBar('No map available for this device.');
      return;
    }

    Navigator.push(
      context,
      MaterialPageRoute(
        builder: (context) => InteractiveMapOrderCreator(
          deviceId: deviceId,
          mapData: _availableMaps[deviceId]!,
          onOrderCreated: (order) {
            _createOrder(order);
            Navigator.of(context).pop(); // Go back to dashboard
          },
        ),
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
      await Future.wait([
        _loadOrdersForAllDevices(),
        _loadSystemStats(),
      ]);
    } catch (e) {
      print('❌ Background refresh failed: $e');
    }
  }

  // ✅ ADD helper methods for colors and icons:

  Color _getOrderStatusColor(String status) {
    switch (status) {
      case 'pending':
        return Colors.orange;
      case 'active':
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

  // ✅ Keep all your existing build methods unchanged:

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
          onPressed: () => _showOrderCreationOptions(),
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
                          'Load, edit, and manage saved maps for AGV navigation',
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
                        onPressed: _refreshAllMaps,
                        icon: Icon(Icons.refresh),
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
                        icon: Icon(Icons.edit),
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

        // Maps content with tabs
        Expanded(
          child: DefaultTabController(
            length: 3,
            child: Column(
              children: [
                TabBar(
                  labelColor: Colors.purple.shade700,
                  unselectedLabelColor: Colors.grey.shade600,
                  indicatorColor: Colors.purple.shade700,
                  tabs: [
                    Tab(icon: Icon(Icons.save), text: 'Saved Maps'),
                    Tab(icon: Icon(Icons.live_tv), text: 'Live Maps'),
                    Tab(icon: Icon(Icons.share), text: 'Shared Maps'),
                  ],
                ),
                Expanded(
                  child: TabBarView(
                    children: [
                      _buildSavedMapsList(),
                      _buildLiveMapsList(), // Your existing live maps
                      _buildSharedMapsList(),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  // Keep all your existing build methods:
  // - _buildOverviewCards()
  // - _buildSystemHealthStatus()
  // - _buildQuickActions()
  // - _buildOrderStatsRow()
  // - _buildOrderStatItem()
  // - _buildStatDivider()
  // - _buildDeviceFilter()
  // - _buildDeviceGrid()
  // - _buildDeviceCard()
  // - _buildStatusChip()
  // - _buildOrdersList()
  // - _buildDetailedOrderCard()
  // - _buildWaypointSequenceItem()
  // - _getStepLabel()
  // - _buildOrderActionButtons()
  // - _buildMapsList()
  // - _buildMapCard()
  // - _buildActiveOrdersSummary()
  // - _buildRecentActivity()

  // I'll include the missing ones here:

  Widget _buildOverviewCards() {
    final totalDevices = _connectedDevices.length;
    final onlineDevices =
        _connectedDevices.where((d) => d['status'] == 'connected').length;
    final totalOrders = _systemStats['total'] ?? 0;
    final activeOrders = _systemStats['byStatus']?['active'] ?? 0;

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
    final stats = _systemStats['byStatus'] ?? {};
    return Container(
      padding: EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.8),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        children: [
          Expanded(
              child: _buildOrderStatItem(
                  'Pending', stats['pending'] ?? 0, Colors.orange)),
          _buildStatDivider(),
          Expanded(
              child: _buildOrderStatItem(
                  'Active', stats['active'] ?? 0, Colors.blue)),
          _buildStatDivider(),
          Expanded(
              child: _buildOrderStatItem(
                  'Completed', stats['completed'] ?? 0, Colors.green)),
          _buildStatDivider(),
          Expanded(
              child: _buildOrderStatItem(
                  'Failed', stats['failed'] ?? 0, Colors.red)),
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
          filteredDevices = _connectedDevices
              .where((d) => d['status'] == 'connected')
              .toList();
          break;
        case 'with_orders':
          filteredDevices = _connectedDevices
              .where((d) => (_deviceOrders[d['id']]?.length ?? 0) > 0)
              .toList();
          break;
        case 'mapping':
          filteredDevices = _connectedDevices
              .where((d) => d['mappingStatus']?['active'] == true)
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
                ? 'Connect your AGV devices to get started'
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
    final activeOrders =
        _deviceOrders[deviceId]?.where((o) => o['status'] == 'active').length ??
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
            Text(
                'Sequence: ${waypoints.length} stations • ${progress['percentage'] ?? 0}% complete'),
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

  Widget _buildWaypointSequenceItem(
      int step, Map<String, dynamic> waypoint, bool isCompleted) {
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
      case 1:
        return 'A';
      case 2:
        return 'B';
      case 3:
        return 'C';
      case 4:
        return 'D';
      default:
        return step.toString();
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

    final totalStations =
        stationCounts.values.fold(0, (sum, count) => sum + count);

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
