// screens/map_page.dart - Simplified for Integration
import 'package:flutter/material.dart';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import '../models/map_data.dart';
import '../models/odom.dart' as odom;
import '../widgets/map_canvas.dart';
import 'dart:async';

class MapPage extends StatefulWidget {
  final String? deviceId;

  const MapPage({Key? key, this.deviceId}) : super(key: key);

  @override
  _MapPageState createState() => _MapPageState();
}

class _MapPageState extends State<MapPage> with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();

  late TabController _tabController;
  late StreamSubscription _realTimeSubscription;
  late StreamSubscription _mapEventsSubscription;
  late StreamSubscription _deviceEventsSubscription;

  MapData? _currentMap;
  List<Map<String, dynamic>> _connectedDevices = <Map<String, dynamic>>[];
  String? _selectedDeviceId;
  bool _isLoading = false;
  bool _hasUnsavedChanges = false;
  bool _isWebSocketConnected = false;

  // Real-time data
  Map<String, dynamic>? _realTimeMapData;
  odom.OdometryData? _currentOdometry;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 2, vsync: this); // Reduced to 2 tabs
    _selectedDeviceId = widget.deviceId;

    _initializeConnections();
    _loadConnectedDevices();
    _subscribeToUpdates();

    if (_selectedDeviceId != null) {
      _loadMapForDevice(_selectedDeviceId!);
    }
  }

  @override
  void dispose() {
    _tabController.dispose();
    _realTimeSubscription.cancel();
    _mapEventsSubscription.cancel();
    _deviceEventsSubscription.cancel();
    super.dispose();
  }

  void _initializeConnections() async {
    // Print API configuration for debugging
    _apiService.printConnectionInfo();
    
    // Test API connectivity first
    final apiConnected = await _apiService.testConnection();
    if (!apiConnected) {
      _showErrorSnackBar('Failed to connect to API server. Check network settings.');
      return;
    }

    // Check WebSocket connection
    setState(() {
      _isWebSocketConnected = _webSocketService.isConnected;
    });

    // If not connected, try to connect
    if (!_isWebSocketConnected) {
      final wsUrl = _apiService.getWebSocketUrl();
      final connected = await _webSocketService.connect(wsUrl);
      setState(() {
        _isWebSocketConnected = connected;
      });
      
      if (!connected) {
        _showWarningSnackBar('WebSocket connection failed. Real-time features will be limited.');
      }
    }
  }

  void _subscribeToUpdates() {
    // Subscribe to real-time map data
    _realTimeSubscription = _webSocketService.realTimeData.listen((data) {
      if (data['deviceId'] == _selectedDeviceId) {
        _handleRealTimeData(data);
      }
    });

    // Subscribe to map events
    _mapEventsSubscription = _webSocketService.mapEvents.listen((data) {
      if (data['deviceId'] == _selectedDeviceId) {
        _handleMapEvent(data);
      }
    });

    // Subscribe to device events
    _deviceEventsSubscription = _webSocketService.deviceEvents.listen((event) {
      _loadConnectedDevices();
    });

    // Monitor WebSocket connection
    _webSocketService.connectionState.listen((connected) {
      setState(() {
        _isWebSocketConnected = connected;
      });
    });
  }

  void _handleRealTimeData(Map<String, dynamic> data) {
    setState(() {
      switch (data['type']) {
        case 'map_update':
          _realTimeMapData = data['data'];
          break;
        case 'odometry_update':
        case 'position_update':
          try {
            _currentOdometry = odom.OdometryData.fromJson(data['data']);
          } catch (e) {
            print('❌ Error parsing odometry data: $e');
          }
          break;
      }
    });
  }

  void _handleMapEvent(Map<String, dynamic> data) {
    switch (data['type']) {
      case 'map_edited':
        print('🗺️ Map edited by another client: ${data['editType']}');
        _showInfoSnackBar('Map updated by another client');
        break;
      case 'map_updated':
        _loadMapForDevice(_selectedDeviceId!);
        break;
    }
  }

  void _loadConnectedDevices() async {
    try {
      final devices = await _apiService.getDevices();
      setState(() {
        _connectedDevices = devices;
      });
    } catch (e) {
      print('❌ Error loading devices: $e');
      if (e is ApiException) {
        if (e.isNetworkError) {
          _showErrorSnackBar('Network error: Cannot reach server. Check IP address.');
        } else {
          _showErrorSnackBar('Failed to load devices: ${e.message}');
        }
      } else {
        _showErrorSnackBar('Failed to load devices: $e');
      }
      setState(() {
        _connectedDevices = <Map<String, dynamic>>[];
      });
    }
  }

  void _loadMapForDevice(String deviceId) async {
    setState(() {
      _isLoading = true;
    });

    try {
      final response = await _apiService.getMapData(deviceId);
      if (response['success'] == true && response['mapData'] != null) {
        try {
          setState(() {
            _currentMap = MapData.fromJson(response['mapData']);
            _hasUnsavedChanges = false;
          });
          print('✅ Map loaded successfully for device: $deviceId');
        } catch (parseError) {
          print('❌ Error parsing map data: $parseError');
          setState(() {
            _currentMap = _createEmptyMap(deviceId);
            _hasUnsavedChanges = false;
          });
          _showWarningSnackBar('Map data corrupted. Created new empty map.');
        }
      } else {
        setState(() {
          _currentMap = _createEmptyMap(deviceId);
          _hasUnsavedChanges = false;
        });
        _showInfoSnackBar('No existing map found. Created new empty map.');
      }
    } catch (e) {
      print('❌ Error loading map: $e');
      if (e is ApiException) {
        if (e.isNotFound) {
          setState(() {
            _currentMap = _createEmptyMap(deviceId);
            _hasUnsavedChanges = false;
          });
          _showInfoSnackBar('No map found for device. Created new empty map.');
        } else {
          _showErrorSnackBar('Failed to load map: ${e.message}');
          setState(() {
            _currentMap = _createEmptyMap(deviceId);
          });
        }
      } else {
        _showErrorSnackBar('Failed to load map: $e');
        setState(() {
          _currentMap = _createEmptyMap(deviceId);
        });
      }
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  MapData _createEmptyMap(String deviceId) {
    return MapData(
      deviceId: deviceId,
      timestamp: DateTime.now(),
      info: MapInfo(
        resolution: 0.05,
        width: 1000,
        height: 1000,
        origin: odom.Position(x: -25.0, y: -25.0, z: 0.0),
        originOrientation: odom.Orientation(x: 0, y: 0, z: 0, w: 1),
      ),
      occupancyData: List.filled(1000 * 1000, -1),
      shapes: [],
      version: 1,
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Map Editor'),
        actions: [
          // API status indicator
          Container(
            margin: EdgeInsets.only(right: 8),
            child: Center(
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Container(
                    width: 8,
                    height: 8,
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: Colors.green, // Assume connected if we got this far
                    ),
                  ),
                  SizedBox(width: 4),
                  Text('API', style: TextStyle(fontSize: 10)),
                ],
              ),
            ),
          ),
          // WebSocket status indicator
          Container(
            margin: EdgeInsets.only(right: 8),
            child: Center(
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Container(
                    width: 8,
                    height: 8,
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: _isWebSocketConnected ? Colors.green : Colors.red,
                    ),
                  ),
                  SizedBox(width: 4),
                  Text('WS', style: TextStyle(fontSize: 10)),
                ],
              ),
            ),
          ),
          _buildDeviceSelector(),
          IconButton(
            icon: Icon(Icons.save),
            onPressed: _hasUnsavedChanges ? _saveMap : null,
            tooltip: 'Save Map',
          ),
          IconButton(
            icon: Icon(Icons.refresh),
            onPressed: _selectedDeviceId != null
                ? () => _loadMapForDevice(_selectedDeviceId!)
                : null,
            tooltip: 'Refresh Map',
          ),
          PopupMenuButton(
            itemBuilder: (context) => [
              PopupMenuItem(
                child: ListTile(
                  leading: Icon(Icons.clear_all),
                  title: Text('Clear All'),
                  dense: true,
                ),
                onTap: _clearAllData,
              ),
              PopupMenuItem(
                child: ListTile(
                  leading: Icon(Icons.info),
                  title: Text('Connection Info'),
                  dense: true,
                ),
                onTap: _showConnectionInfo,
              ),
            ],
          ),
        ],
        bottom: TabBar(
          controller: _tabController,
          tabs: [
            Tab(icon: Icon(Icons.map), text: 'Edit Map'),
            Tab(icon: Icon(Icons.list), text: 'Locations'),
          ],
        ),
      ),
      body: _selectedDeviceId == null
          ? _buildSelectDeviceView()
          : _isLoading
              ? Center(child: CircularProgressIndicator())
              : TabBarView(
                  controller: _tabController,
                  children: [
                    _buildMapEditorTab(),
                    _buildLocationsTab(),
                  ],
                ),
    );
  }

  Widget _buildDeviceSelector() {
    return Container(
      margin: EdgeInsets.symmetric(horizontal: 8),
      child: DropdownButton<String>(
        value: _selectedDeviceId,
        hint: Text('Select Device', style: TextStyle(color: Colors.white)),
        dropdownColor: Colors.grey[800],
        style: TextStyle(color: Colors.white),
        underline: Container(),
        items: _connectedDevices.map((device) {
          final deviceId = device['id']?.toString() ?? '';
          final deviceName = device['name']?.toString() ?? deviceId;
          final deviceStatus = device['status']?.toString() ?? 'unknown';

          return DropdownMenuItem<String>(
            value: deviceId,
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                Container(
                  width: 8,
                  height: 8,
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    color: deviceStatus == 'connected'
                        ? Colors.green
                        : Colors.grey,
                  ),
                ),
                SizedBox(width: 8),
                Text(deviceName),
              ],
            ),
          );
        }).toList(),
        onChanged: (deviceId) {
          if (deviceId != null) {
            setState(() {
              _selectedDeviceId = deviceId;
              _currentMap = null;
              _hasUnsavedChanges = false;
              _realTimeMapData = null;
              _currentOdometry = null;
            });
            _loadMapForDevice(deviceId);

            // Subscribe to real-time updates for new device
            if (_isWebSocketConnected) {
              _webSocketService.subscribe('real_time_data', deviceId: deviceId);
              _webSocketService.subscribe('map_events', deviceId: deviceId);
            }
          }
        },
      ),
    );
  }

  Widget _buildSelectDeviceView() {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(Icons.map, size: 80, color: Colors.grey),
          SizedBox(height: 16),
          Text(
            'Select a Device',
            style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
          ),
          SizedBox(height: 8),
          Text('Choose a connected AGV device to edit its map'),
          SizedBox(height: 16),
          if (!_isWebSocketConnected) ...[
            Container(
              padding: EdgeInsets.all(16),
              margin: EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: Colors.orange[50],
                border: Border.all(color: Colors.orange),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Row(
                children: [
                  Icon(Icons.warning, color: Colors.orange),
                  SizedBox(width: 8),
                  Expanded(
                    child: Text(
                      'WebSocket disconnected. Real-time features unavailable.',
                      style: TextStyle(color: Colors.orange[700]),
                    ),
                  ),
                  TextButton(
                    onPressed: _reconnectWebSocket,
                    child: Text('Reconnect'),
                  ),
                ],
              ),
            ),
          ],
          SizedBox(height: 24),
          if (_connectedDevices.isEmpty) ...[
            Text('No connected devices found'),
            SizedBox(height: 16),
            ElevatedButton(
              onPressed: () => Navigator.pushNamed(context, '/connect'),
              child: Text('Connect Devices'),
            ),
            SizedBox(height: 8),
            TextButton(
              onPressed: _loadConnectedDevices,
              child: Text('Refresh Device List'),
            ),
          ] else
            Container(
              constraints: BoxConstraints(maxWidth: 400),
              child: ListView.builder(
                shrinkWrap: true,
                itemCount: _connectedDevices.length,
                itemBuilder: (context, index) {
                  final device = _connectedDevices[index];
                  final deviceId = device['id']?.toString() ?? '';
                  final deviceName = device['name']?.toString() ?? deviceId;
                  final deviceStatus = device['status']?.toString() ?? 'unknown';

                  return Card(
                    child: ListTile(
                      leading: Stack(
                        children: [
                          Icon(Icons.smart_toy),
                          Positioned(
                            right: 0,
                            bottom: 0,
                            child: Container(
                              width: 12,
                              height: 12,
                              decoration: BoxDecoration(
                                shape: BoxShape.circle,
                                color: deviceStatus == 'connected'
                                    ? Colors.green
                                    : Colors.grey,
                                border: Border.all(color: Colors.white, width: 2),
                              ),
                            ),
                          ),
                        ],
                      ),
                      title: Text(deviceName),
                      subtitle: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text('ID: $deviceId'),
                          Text('Status: $deviceStatus'),
                        ],
                      ),
                      trailing: deviceStatus == 'connected'
                          ? Icon(Icons.chevron_right)
                          : Icon(Icons.warning, color: Colors.orange),
                      onTap: () {
                        setState(() {
                          _selectedDeviceId = deviceId;
                        });
                        _loadMapForDevice(deviceId);
                      },
                    ),
                  );
                },
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildMapEditorTab() {
    if (_currentMap == null) {
      return Center(child: Text('No map data available'));
    }

    return Column(
      children: [
        if (_hasUnsavedChanges)
          Container(
            width: double.infinity,
            color: Colors.orange,
            padding: EdgeInsets.all(8),
            child: Row(
              children: [
                Icon(Icons.warning, color: Colors.white),
                SizedBox(width: 8),
                Text(
                  'You have unsaved changes',
                  style: TextStyle(
                      color: Colors.white, fontWeight: FontWeight.bold),
                ),
                Spacer(),
                TextButton(
                  onPressed: _saveMap,
                  child: Text('SAVE', style: TextStyle(color: Colors.white)),
                ),
              ],
            ),
          ),
        if (!_isWebSocketConnected)
          Container(
            width: double.infinity,
            color: Colors.red[100],
            padding: EdgeInsets.all(8),
            child: Row(
              children: [
                Icon(Icons.wifi_off, color: Colors.red),
                SizedBox(width: 8),
                Text(
                  'Real-time updates unavailable (WebSocket disconnected)',
                  style: TextStyle(color: Colors.red[700]),
                ),
                Spacer(),
                TextButton(
                  onPressed: _reconnectWebSocket,
                  child: Text('RECONNECT', style: TextStyle(color: Colors.red)),
                ),
              ],
            ),
          ),
        Expanded(
          child: MapCanvasWidget(
            mapData: _currentMap,
            onMapChanged: _onMapChanged,
            deviceId: _selectedDeviceId,
            enableRealTimeUpdates: _isWebSocketConnected,
          ),
        ),
      ],
    );
  }

  Widget _buildLocationsTab() {
    if (_currentMap == null) {
      return Center(child: Text('No map data available'));
    }

    final locations = _currentMap!.shapes;
    final groupedLocations = <String, List<MapShape>>{};

    for (final location in locations) {
      if (!groupedLocations.containsKey(location.type)) {
        groupedLocations[location.type] = [];
      }
      groupedLocations[location.type]!.add(location);
    }

    return SingleChildScrollView(
      padding: EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Text(
                'Defined Locations',
                style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
              ),
              Spacer(),
              Text('Total: ${locations.length}'),
            ],
          ),
          SizedBox(height: 16),
          if (locations.isEmpty)
            Card(
              child: Padding(
                padding: EdgeInsets.all(24),
                child: Column(
                  children: [
                    Icon(Icons.location_off, size: 48, color: Colors.grey),
                    SizedBox(height: 8),
                    Text('No locations defined'),
                    SizedBox(height: 4),
                    Text('Add locations using the map editor'),
                  ],
                ),
              ),
            )
          else
            ...groupedLocations.entries.map((entry) {
              return Card(
                child: Padding(
                  padding: EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        '${entry.key.toUpperCase()} LOCATIONS (${entry.value.length})',
                        style: TextStyle(
                            fontSize: 16, fontWeight: FontWeight.bold),
                      ),
                      SizedBox(height: 8),
                      ListView.builder(
                        shrinkWrap: true,
                        physics: NeverScrollableScrollPhysics(),
                        itemCount: entry.value.length,
                        itemBuilder: (context, index) {
                          final location = entry.value[index];
                          return _buildLocationListItem(location);
                        },
                      ),
                    ],
                  ),
                ),
              );
            }),
        ],
      ),
    );
  }

  Widget _buildLocationListItem(MapShape location) {
    Color color;
    try {
      final colorString = location.color.replaceAll('#', '');
      final colorValue = int.tryParse('0xFF$colorString') ?? 0xFF000000;
      color = Color(colorValue);
    } catch (e) {
      color = Colors.black;
    }

    return ListTile(
      leading: Container(
        width: 40,
        height: 40,
        decoration: BoxDecoration(
          color: color.withOpacity(0.3),
          border: Border.all(color: color),
          borderRadius: BorderRadius.circular(8),
        ),
        child: Icon(
          _getLocationIcon(location.type),
          color: color,
        ),
      ),
      title: Text(location.name),
      subtitle: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
              'Position: ${location.center.x.toStringAsFixed(2)}, ${location.center.y.toStringAsFixed(2)}'),
          if (location.sides.values.any((side) => side.isNotEmpty))
            Text(
                'Sides: ${location.sides.entries.where((e) => e.value.isNotEmpty).map((e) => '${e.key}: ${e.value}').join(', ')}'),
        ],
      ),
      trailing: IconButton(
        icon: Icon(Icons.navigation),
        onPressed: () => _navigateToLocation(location),
        tooltip: 'Navigate Here',
      ),
    );
  }

  IconData _getLocationIcon(String type) {
    switch (type) {
      case 'pickup':
        return Icons.outbox;
      case 'drop':
        return Icons.inbox;
      case 'charging':
        return Icons.battery_charging_full;
      case 'waypoint':
        return Icons.place;
      case 'obstacle':
        return Icons.warning;
      default:
        return Icons.location_on;
    }
  }

  // Event handlers
  void _onMapChanged(MapData newMapData) {
    setState(() {
      _currentMap = newMapData;
      _hasUnsavedChanges = true;
    });
  }

  void _saveMap() async {
    if (_currentMap == null || _selectedDeviceId == null) return;

    try {
      await _apiService.saveMapData(
        deviceId: _selectedDeviceId!,
        mapData: _currentMap!,
      );

      setState(() {
        _hasUnsavedChanges = false;
      });

      _showInfoSnackBar('Map saved successfully');
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to save map: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to save map: $e');
      }
    }
  }

  void _reconnectWebSocket() async {
    final wsUrl = _apiService.getWebSocketUrl();
    final connected = await _webSocketService.connect(wsUrl);
    setState(() {
      _isWebSocketConnected = connected;
    });

    if (connected) {
      _showInfoSnackBar('WebSocket reconnected');
      if (_selectedDeviceId != null) {
        _webSocketService.subscribe('real_time_data',
            deviceId: _selectedDeviceId);
        _webSocketService.subscribe('map_events', deviceId: _selectedDeviceId);
      }
    } else {
      _showErrorSnackBar('Failed to reconnect WebSocket');
    }
  }

  void _showConnectionInfo() async {
    try {
      final info = await _apiService.getConnectionInfo();
      
      showDialog(
        context: context,
        builder: (context) => AlertDialog(
          title: Text('Connection Information'),
          content: SingleChildScrollView(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisSize: MainAxisSize.min,
              children: [
                _buildInfoRow('Status', info['connected'] ? 'Connected' : 'Disconnected'),
                _buildInfoRow('Base URL', info['baseUrl']),
                _buildInfoRow('API URL', info['apiBaseUrl']),
                _buildInfoRow('WebSocket URL', info['websocketUrl']),
                if (info['serverInfo'] != null) ...[
                  SizedBox(height: 16),
                  Text('Server Info:', style: TextStyle(fontWeight: FontWeight.bold)),
                  _buildInfoRow('Server Status', info['serverInfo']['status']),
                ],
                if (info['error'] != null) ...[
                  SizedBox(height: 16),
                  Text('Error:', style: TextStyle(fontWeight: FontWeight.bold, color: Colors.red)),
                  Text(info['error'], style: TextStyle(color: Colors.red)),
                ],
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
    } catch (e) {
      _showErrorSnackBar('Failed to get connection info: $e');
    }
  }

  Widget _buildInfoRow(String label, String value, {Color? color}) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 4),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          SizedBox(
            width: 120,
            child: Text(
              '$label:',
              style: TextStyle(fontWeight: FontWeight.w500),
            ),
          ),
          Expanded(
            child: Text(
              value,
              style: TextStyle(color: color),
            ),
          ),
        ],
      ),
    );
  }

  void _navigateToLocation(MapShape location) async {
    if (_selectedDeviceId == null) return;

    try {
      // Use the movement API as a placeholder for navigation
      await _apiService.moveDevice(
        deviceId: _selectedDeviceId!,
        linear: 0.0,
        angular: 0.0,
      );

      _showInfoSnackBar('Navigation goal set to ${location.name}');
    } catch (e) {
      if (e is ApiException) {
        _showErrorSnackBar('Failed to set navigation goal: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to set navigation goal: $e');
      }
    }
  }

  void _clearAllData() async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Clear All Data'),
        content: Text(
            'This will clear all shapes and annotations. This action cannot be undone.'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(false),
            child: Text('Cancel'),
          ),
          TextButton(
            onPressed: () => Navigator.of(context).pop(true),
            child: Text('Clear All', style: TextStyle(color: Colors.red)),
          ),
        ],
      ),
    );

    if (confirmed == true && _currentMap != null) {
      final clearedMap = MapData(
        deviceId: _currentMap!.deviceId,
        timestamp: DateTime.now(),
        info: _currentMap!.info,
        occupancyData: _currentMap!.occupancyData,
        shapes: [],
        version: _currentMap!.version + 1,
      );

      _onMapChanged(clearedMap);
      _showInfoSnackBar('All data cleared');
    }
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

  void _showWarningSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.orange,
        duration: Duration(seconds: 3),
      ),
    );
  }

  void _showInfoSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.blue,
        duration: Duration(seconds: 2),
      ),
    );
  }
}