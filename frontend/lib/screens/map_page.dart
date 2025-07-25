// screens/enhanced_map_page.dart - Enhanced Map Page with Full Workflow
import 'package:flutter/material.dart';
import '../models/map_data.dart';
import '../services/api_service.dart';
import '../widgets/map_canvas.dart';
import '../widgets/pgm_map_editor.dart';
import '../services/web_socket_service.dart';
import '../widgets/ros2_saved_maps_screen.dart';
import 'dart:async';
import '../models/odom.dart' as odom;

class EnhancedMapPage extends StatefulWidget {
  final String? deviceId;

  const EnhancedMapPage({Key? key, this.deviceId}) : super(key: key);

  @override
  _EnhancedMapPageState createState() => _EnhancedMapPageState();
}

class _EnhancedMapPageState extends State<EnhancedMapPage>
    with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();

  AGVMode _currentMode = AGVMode.defaultMode;

  late TabController _tabController;
  late AnimationController _loadingAnimationController;
  late StreamSubscription _realTimeSubscription;
  late StreamSubscription _mapEventsSubscription;
  late StreamSubscription _deviceEventsSubscription;

  MapData? _currentMap;
  List<Map<String, dynamic>> _connectedDevices = <Map<String, dynamic>>[];
  String? _selectedDeviceId;
  bool _isLoading = false;
  bool _hasUnsavedChanges = false;
  bool _isWebSocketConnected = false;

  // Enhanced debugging and error handling
  String? _error;
  String? _loadSource; // Track where data was loaded from
  Map<String, dynamic>? _rawApiResponse;
  bool _showDebugInfo = false;

  // UI state for responsive design
  DeviceType _deviceType = DeviceType.phone;
  bool _showSidebar = true;
  bool _showPropertiesPanel = false;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 4, vsync: this);
    _loadingAnimationController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );
    _selectedDeviceId = widget.deviceId;

    _initializeConnections();
    _loadConnectedDevices();
    _subscribeToUpdates();

    if (_selectedDeviceId != null) {
      _loadMapForDevice(_selectedDeviceId!);
    }

    // Determine device type
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _updateDeviceType();

      // Check if we should load a ROS2 saved map
      final args =
          ModalRoute.of(context)?.settings.arguments as Map<String, dynamic>?;
      if (args != null && args['loadROS2Map'] == true) {
        final mapName = args['ros2MapName'] as String?;
        if (mapName != null && _selectedDeviceId != null) {
          _loadROS2SavedMap(mapName);
        }
      }
    });
  }

  void _updateDeviceType() {
    final screenWidth = MediaQuery.of(context).size.width;
    setState(() {
      if (screenWidth > 1200) {
        _deviceType = DeviceType.desktop;
        _showSidebar = true;
      } else if (screenWidth > 768) {
        _deviceType = DeviceType.tablet;
        _showSidebar = true;
      } else {
        _deviceType = DeviceType.phone;
        _showSidebar = false;
      }
    });
  }

  @override
  void dispose() {
    _tabController.dispose();
    _loadingAnimationController.dispose();
    _realTimeSubscription.cancel();
    _mapEventsSubscription.cancel();
    _deviceEventsSubscription.cancel();
    super.dispose();
  }

  void _initializeConnections() async {
    _apiService.printConnectionInfo();

    final apiConnected = await _apiService.testConnection();
    if (!apiConnected) {
      _showErrorSnackBar(
          'Failed to connect to API server. Check network settings.');
      return;
    }

    setState(() {
      _isWebSocketConnected = _webSocketService.isConnected;
    });

    if (!_isWebSocketConnected) {
      final wsUrl = _apiService.getWebSocketUrl();
      if (wsUrl == null) {
        _showErrorSnackBar('WebSocket URL is not available.');
        return;
      }
      final connected = await _webSocketService.connect(wsUrl);
      setState(() {
        _isWebSocketConnected = connected;
      });

      if (!connected) {
        _showWarningSnackBar(
            'WebSocket connection failed. Real-time features will be limited.');
      }
    }
  }

  void _subscribeToUpdates() {
    _realTimeSubscription = _webSocketService.realTimeData.listen((data) {
      if (data['deviceId'] == _selectedDeviceId) {
        _handleRealTimeData(data);
      }
    });

    _mapEventsSubscription = _webSocketService.mapEvents.listen((data) {
      if (data['deviceId'] == _selectedDeviceId) {
        _handleMapEvent(data);
      }
    });

    _deviceEventsSubscription = _webSocketService.deviceEvents.listen((event) {
      _loadConnectedDevices();
    });

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
          // Handle real-time map data
          break;
        case 'odometry_update':
        case 'position_update':
          try {
            // Handle odometry data
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

        // Validate that selected device still exists in the list
        if (_selectedDeviceId != null) {
          final deviceExists = devices
              .any((device) => device['id']?.toString() == _selectedDeviceId);
          if (!deviceExists) {
            _selectedDeviceId = null;
            _currentMap = null;
            _hasUnsavedChanges = false;
          }
        }
      });
    } catch (e) {
      print('❌ Error loading devices: $e');
      if (e is ApiException) {
        if (e.isNetworkError) {
          _showErrorSnackBar(
              'Network error: Cannot reach server. Check IP address.');
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

  void _loadMapWithConfirmation(String deviceId) async {
    if (_hasUnsavedChanges) {
      final confirmed = await showDialog<bool>(
        context: context,
        builder: (context) => AlertDialog(
          title: const Text('Unsaved Changes'),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text('You have unsaved changes that will be lost.'),
              const SizedBox(height: 8),
              Text(
                'Current map: ${_currentMap?.shapes.length ?? 0} locations',
                style: const TextStyle(fontWeight: FontWeight.bold),
              ),
            ],
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(false),
              child: const Text('Cancel'),
            ),
            ElevatedButton(
              onPressed: () => Navigator.of(context).pop(true),
              style: ElevatedButton.styleFrom(backgroundColor: Colors.orange),
              child: const Text('Load Anyway'),
            ),
          ],
        ),
      );

      if (confirmed == true) {
        _loadMapForDevice(deviceId);
      }
    } else {
      _loadMapForDevice(deviceId);
    }
  }

  void _loadMapForDevice(String deviceId) async {
    print('🔄 Loading map for device: $deviceId');

    setState(() {
      _isLoading = true;
      _error = null;
      _rawApiResponse = null;
    });
    _loadingAnimationController.repeat();

    try {
      final response = await _apiService.getMapData(deviceId);

      print('📦 API Response: ${response.keys.toList()}');
      // Store raw response for debugging
      _rawApiResponse = Map<String, dynamic>.from(response);

      if (response['success'] == true && response['mapData'] != null) {
        try {
          final mapData = MapData.fromJson(response['mapData']);
          setState(() {
            _currentMap = mapData;
            _hasUnsavedChanges = false;
            _loadSource = response['source'] ?? 'api';
            _error = null;
          });

          _showInfoSnackBar(
              'Map loaded successfully with ${mapData.shapes.length} locations from ${_loadSource}');
          print(
              '✅ Map loaded successfully for device: $deviceId from source: ${_loadSource}');
        } catch (parseError) {
          print('❌ Error parsing map data: $parseError');
          print(
              '📋 Raw map data structure: ${response['mapData']?.keys?.toList() ?? 'null'}');

          setState(() {
            _currentMap = _createEmptyMap(deviceId);
            _hasUnsavedChanges = false;
            _error = 'Failed to parse map data: $parseError';
            _loadSource = 'empty_fallback';
          });
          _showWarningSnackBar('Map data corrupted. Created new empty map.');
        }
      } else {
        setState(() {
          _currentMap = _createEmptyMap(deviceId);
          _hasUnsavedChanges = false;
          _error = response['error'] ?? 'No map data available';
          _loadSource = 'empty_created';
        });
        _showInfoSnackBar('No existing map found. Created new empty map.');
      }
    } catch (e) {
      print('❌ Error loading map: $e');
      setState(() {
        _error = 'Failed to load map data: $e';
      });

      if (e is ApiException) {
        if (e.isNotFound) {
          setState(() {
            _currentMap = _createEmptyMap(deviceId);
            _hasUnsavedChanges = false;
            _loadSource = 'empty_not_found';
          });
          _showInfoSnackBar('No map found for device. Created new empty map.');
        } else {
          _showErrorSnackBar('Failed to load map: ${e.message}');
          setState(() {
            _currentMap = _createEmptyMap(deviceId);
            _loadSource = 'empty_error';
          });
        }
      } else {
        _showErrorSnackBar('Failed to load map: $e');
        setState(() {
          _currentMap = _createEmptyMap(deviceId);
          _loadSource = 'empty_error';
        });
      }
    } finally {
      setState(() {
        _isLoading = false;
      });
      _loadingAnimationController.stop();
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
    final theme = Theme.of(context);

    return PopScope(
      canPop: false,
      onPopInvokedWithResult: (didPop, result) async {
        if (didPop) return;
        Navigator.pushReplacementNamed(context, '/dashboard');
      },
      child: Scaffold(
        appBar: _buildEnhancedAppBar(theme),
        body: _buildResponsiveBody(),
        drawer: _deviceType == DeviceType.phone ? _buildMobileDrawer() : null,
        floatingActionButton: _buildFloatingActionButtons(),
      ),
    );
  }

  PreferredSizeWidget _buildEnhancedAppBar(ThemeData theme) {
    return AppBar(
      leading: IconButton(
        icon: const Icon(Icons.arrow_back, color: Colors.white),
        onPressed: () {
          Navigator.pushReplacementNamed(context, '/dashboard');
        },
        tooltip: 'Back to Dashboard',
      ),
      title: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [
                  theme.primaryColor,
                  theme.primaryColor.withOpacity(0.7)
                ],
              ),
              borderRadius: BorderRadius.circular(8),
            ),
            child: const Icon(Icons.map, color: Colors.white, size: 20),
          ),
          const SizedBox(width: 12),
          Column(
            mainAxisAlignment: MainAxisAlignment.center,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text('Map Editor', style: TextStyle(fontSize: 18)),
              if (_currentMap != null)
                Text(
                  _hasUnsavedChanges
                      ? '● ${_currentMap!.shapes.length} locations (unsaved)'
                      : '${_currentMap!.shapes.length} locations (saved)',
                  style: TextStyle(
                    fontSize: 12,
                    color: _hasUnsavedChanges
                        ? Colors.orange.shade200
                        : Colors.white70,
                    fontWeight: _hasUnsavedChanges
                        ? FontWeight.bold
                        : FontWeight.normal,
                  ),
                ),
            ],
          ),
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
        // Data source indicator
        if (_loadSource != null)
          Container(
            margin: const EdgeInsets.only(right: 8),
            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
            decoration: BoxDecoration(
              color: _getSourceColor(_loadSource!),
              borderRadius: BorderRadius.circular(12),
            ),
            child: Text(
              _loadSource!.toUpperCase(),
              style: const TextStyle(fontSize: 10, color: Colors.white),
            ),
          ),

        // Debug toggle
        IconButton(
          onPressed: () {
            setState(() {
              _showDebugInfo = !_showDebugInfo;
            });
          },
          icon: Icon(Icons.bug_report,
              color: _showDebugInfo ? Colors.orange : Colors.white70),
          tooltip: 'Toggle Debug Info',
        ),

        // Enhanced status indicators
        _buildStatusIndicator('API', true, Colors.green),
        const SizedBox(width: 8),
        _buildStatusIndicator('WS', _isWebSocketConnected,
            _isWebSocketConnected ? Colors.green : Colors.red),
        const SizedBox(width: 16),

        // Device selector for larger screens
        if (_deviceType != DeviceType.phone) _buildDeviceSelector(),

        // Action buttons
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 4),
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: _hasUnsavedChanges
                  ? [Colors.orange.shade400, Colors.orange.shade600]
                  : [Colors.grey.shade400, Colors.grey.shade600],
            ),
            borderRadius: BorderRadius.circular(20),
          ),
          child: IconButton(
            icon: const Icon(Icons.save, color: Colors.white),
            onPressed: _hasUnsavedChanges ? _saveMap : null,
            tooltip: 'Save Map',
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
            onPressed: _selectedDeviceId != null
                ? () => _loadMapWithConfirmation(_selectedDeviceId!)
                : null,
            tooltip: 'Refresh Map',
          ),
        ),

        _buildMenuButton(),
      ],
      bottom: _deviceType != DeviceType.phone
          ? TabBar(
              controller: _tabController,
              indicatorColor: Colors.white,
              labelColor: Colors.white,
              unselectedLabelColor: Colors.white70,
              tabs: const [
                Tab(icon: Icon(Icons.edit), text: 'Edit Map'),
                Tab(icon: Icon(Icons.brush), text: 'PGM Editor'),
                Tab(icon: Icon(Icons.list), text: 'Locations'),
                Tab(icon: Icon(Icons.analytics), text: 'Analysis'),
              ],
            )
          : null,
    );
  }

  Widget _buildStatusIndicator(String label, bool connected, Color color) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        color: connected ? color : Colors.red,
        borderRadius: BorderRadius.circular(12),
        boxShadow: [
          BoxShadow(
            color: (connected ? color : Colors.red).withOpacity(0.3),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 6,
            height: 6,
            decoration: BoxDecoration(
              color: Colors.white,
              shape: BoxShape.circle,
            ),
          ),
          const SizedBox(width: 4),
          Text(
            label,
            style: const TextStyle(
              color: Colors.white,
              fontSize: 10,
              fontWeight: FontWeight.bold,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDeviceSelector() {
    // Get unique devices and check if selected device exists
    final uniqueDevices = _connectedDevices
        .fold<Map<String, Map<String, dynamic>>>(
          {},
          (map, device) {
            final deviceId = device['id']?.toString() ?? '';
            if (deviceId.isNotEmpty) {
              map[deviceId] = device;
            }
            return map;
          },
        )
        .values
        .toList();

    // Validate selected device exists in unique devices
    final selectedDeviceExists = _selectedDeviceId != null &&
        uniqueDevices
            .any((device) => device['id']?.toString() == _selectedDeviceId);

    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 8),
      padding: const EdgeInsets.symmetric(horizontal: 12),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.2),
        borderRadius: BorderRadius.circular(20),
      ),
      child: DropdownButton<String>(
        value: selectedDeviceExists ? _selectedDeviceId : null,
        hint:
            const Text('Select Device', style: TextStyle(color: Colors.white)),
        dropdownColor: Colors.grey[800],
        style: const TextStyle(color: Colors.white),
        underline: Container(),
        items: uniqueDevices.map((device) {
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
                const SizedBox(width: 8),
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
            });
            _loadMapForDevice(deviceId);

            if (_isWebSocketConnected) {
              _webSocketService.subscribe('real_time_data', deviceId: deviceId);
              _webSocketService.subscribe('map_events', deviceId: deviceId);
            }
          }
        },
      ),
    );
  }

  Widget _buildMenuButton() {
    return PopupMenuButton(
      icon: const Icon(Icons.more_vert, color: Colors.white),
      itemBuilder: (context) => [
        PopupMenuItem(
          child: ListTile(
            leading: const Icon(Icons.folder),
            title: const Text('Saved Maps'),
            dense: true,
          ),
          onTap: () => _showSavedMaps(),
        ),
        PopupMenuItem(
          child: ListTile(
            leading: const Icon(Icons.rocket_launch),
            title: const Text('ROS2 Saved Maps'),
            dense: true,
          ),
          onTap: () => _showROS2SavedMaps(),
        ),
        PopupMenuItem(
          child: ListTile(
            leading: const Icon(Icons.clear_all),
            title: const Text('Clear All'),
            dense: true,
          ),
          onTap: _clearAllData,
        ),
        PopupMenuItem(
          child: ListTile(
            leading: const Icon(Icons.send),
            title: const Text('Send to AGV'),
            dense: true,
          ),
          onTap: _sendMapToAGV,
        ),
        PopupMenuItem(
          child: ListTile(
            leading: const Icon(Icons.info),
            title: const Text('Connection Info'),
            dense: true,
          ),
          onTap: _showConnectionInfo,
        ),
      ],
    );
  }

  Widget _buildResponsiveBody() {
    return Column(
      children: [
        // Debug info panel
        if (_showDebugInfo) _buildDebugPanel(),

        // Error display
        if (_error != null) _buildErrorBanner(),

        // Main responsive content
        Expanded(
          child: _buildMainResponsiveContent(),
        ),
      ],
    );
  }

  Widget _buildMainResponsiveContent() {
    switch (_deviceType) {
      case DeviceType.desktop:
        return _buildDesktopLayout();
      case DeviceType.tablet:
        return _buildTabletLayout();
      case DeviceType.phone:
        return _buildPhoneLayout();
    }
  }

  Widget _buildDesktopLayout() {
    return Row(
      children: [
        // Sidebar
        if (_showSidebar)
          Container(
            width: 300,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topCenter,
                end: Alignment.bottomCenter,
                colors: [
                  Colors.grey.shade50,
                  Colors.grey.shade100,
                ],
              ),
              border: Border(
                right: BorderSide(color: Colors.grey.shade300),
              ),
            ),
            child: _buildSidebar(),
          ),

        // Main content
        Expanded(
          child: Column(
            children: [
              // Tab bar
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
                  tabs: const [
                    Tab(icon: Icon(Icons.edit), text: 'Edit Map'),
                    Tab(icon: Icon(Icons.brush), text: 'PGM Editor'),
                    Tab(icon: Icon(Icons.list), text: 'Locations'),
                    Tab(icon: Icon(Icons.analytics), text: 'Analysis'),
                  ],
                ),
              ),

              // Content
              Expanded(
                child: _buildMainContent(),
              ),
            ],
          ),
        ),

        // Properties panel
        if (_showPropertiesPanel)
          Container(
            width: 280,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topCenter,
                end: Alignment.bottomCenter,
                colors: [
                  Colors.blue.shade50,
                  Colors.blue.shade100,
                ],
              ),
              border: Border(
                left: BorderSide(color: Colors.blue.shade300),
              ),
            ),
            child: _buildPropertiesPanel(),
          ),
      ],
    );
  }

  Widget _buildTabletLayout() {
    return Column(
      children: [
        // Status bar
        _buildStatusBar(),

        // Tab bar
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
            tabs: const [
              Tab(icon: Icon(Icons.edit), text: 'Edit Map'),
              Tab(icon: Icon(Icons.brush), text: 'PGM Editor'),
              Tab(icon: Icon(Icons.list), text: 'Locations'),
              Tab(icon: Icon(Icons.analytics), text: 'Analysis'),
            ],
          ),
        ),

        // Content
        Expanded(
          child: _buildMainContent(),
        ),
      ],
    );
  }

// Update _buildPhoneLayout method:
  Widget _buildPhoneLayout() {
    return Column(
      children: [
        // Compact mode selector for mobile
        Container(
          padding: EdgeInsets.symmetric(horizontal: 16, vertical: 8),
          child: Row(
            children: [
              Text('Mode: ', style: TextStyle(fontWeight: FontWeight.bold)),
              Expanded(
                child: SingleChildScrollView(
                  scrollDirection: Axis.horizontal,
                  child: Row(
                    children: [
                      _buildCompactModeButton('Default', AGVMode.defaultMode),
                      SizedBox(width: 8),
                      _buildCompactModeButton('Auto', AGVMode.autonomous),
                      SizedBox(width: 8),
                      _buildCompactModeButton('Map', AGVMode.mapping),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),

        // Compact status bar
        if (_selectedDeviceId != null) _buildCompactStatusBar(),

        // Main content with better space usage
        Expanded(
          child: _selectedDeviceId == null
              ? _buildSelectDeviceView()
              : _isLoading
                  ? _buildLoadingView()
                  : _buildCompactMapEditor(),
        ),

        // Compact bottom controls
        _buildCompactBottomControls(),
      ],
    );
  }

  /// Compact map editor for mobile layout
  Widget _buildCompactMapEditor() {
    if (_currentMap == null) {
      return const Center(child: Text('No map data available'));
    }
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
      ),
      child: EnhancedMapCanvas(
        mapData: _currentMap,
        onMapChanged: _onMapChanged,
        deviceId: _selectedDeviceId,
        enableRealTimeUpdates: _isWebSocketConnected,
      ),
    );
  }

  /// Builds a compact mode button for mobile layout.
  Widget _buildCompactModeButton(String label, AGVMode mode) {
    final isSelected = _currentMode == mode;
    return Container(
      height: 32,
      child: ElevatedButton(
        onPressed: () => _setMode(mode),
        style: ElevatedButton.styleFrom(
          backgroundColor: isSelected
              ? Theme.of(context).primaryColor
              : Colors.grey.shade300,
          foregroundColor: isSelected ? Colors.white : Colors.black,
          padding: EdgeInsets.symmetric(horizontal: 12, vertical: 4),
          minimumSize: Size(60, 32),
        ),
        child: Text(label, style: TextStyle(fontSize: 12)),
      ),
    );
  }

  /// Sets the current AGV mode and updates the UI.
  void _setMode(AGVMode mode) {
    setState(() {
      _currentMode = mode;
    });
  }

  Widget _buildCompactStatusBar() {
    return Container(
      padding: EdgeInsets.symmetric(horizontal: 16, vertical: 4),
      decoration: BoxDecoration(
        color: Colors.grey.shade100,
        border: Border(bottom: BorderSide(color: Colors.grey.shade300)),
      ),
      child: Row(
        children: [
          Icon(Icons.device_hub,
              size: 16, color: Theme.of(context).primaryColor),
          SizedBox(width: 8),
          Expanded(
            child: Text(
              _selectedDeviceId!,
              style: TextStyle(fontSize: 12, fontWeight: FontWeight.bold),
              overflow: TextOverflow.ellipsis,
            ),
          ),
          if (_hasUnsavedChanges)
            Container(
              padding: EdgeInsets.symmetric(horizontal: 6, vertical: 2),
              decoration: BoxDecoration(
                color: Colors.orange,
                borderRadius: BorderRadius.circular(8),
              ),
              child: Text('UNSAVED',
                  style: TextStyle(color: Colors.white, fontSize: 8)),
            ),
        ],
      ),
    );
  }

  Widget _buildStatusBar() {
    if (_selectedDeviceId == null) return const SizedBox.shrink();

    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Theme.of(context).primaryColor.withOpacity(0.1),
            Theme.of(context).primaryColor.withOpacity(0.05),
          ],
        ),
        border: Border(
          bottom: BorderSide(color: Colors.grey.shade300),
        ),
      ),
      child: Column(
        children: [
          // Device info
          Row(
            children: [
              Container(
                padding: const EdgeInsets.all(8),
                decoration: BoxDecoration(
                  color: Theme.of(context).primaryColor,
                  borderRadius: BorderRadius.circular(8),
                ),
                child:
                    const Icon(Icons.device_hub, color: Colors.white, size: 16),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  'Device: $_selectedDeviceId',
                  style: const TextStyle(fontWeight: FontWeight.bold),
                ),
              ),
              if (_hasUnsavedChanges)
                Container(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                  decoration: BoxDecoration(
                    color: Colors.orange,
                    borderRadius: BorderRadius.circular(12),
                  ),
                  child: const Text(
                    'UNSAVED',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 10,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                ),
            ],
          ),

          const SizedBox(height: 8),

          // Quick stats
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              _buildQuickStat('Shapes',
                  _currentMap?.shapes.length.toString() ?? '0', Icons.category),
              _buildQuickStat('Version', _currentMap?.version.toString() ?? '0',
                  Icons.numbers),
              _buildQuickStat(
                  'Size',
                  _currentMap != null
                      ? '${_currentMap!.info.width}×${_currentMap!.info.height}'
                      : 'N/A',
                  Icons.aspect_ratio),
              _buildQuickStat(
                  'Resolution',
                  _currentMap?.info.resolution.toStringAsFixed(3) ?? 'N/A',
                  Icons.grid_4x4),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildQuickStat(String label, String value, IconData icon) {
    return Column(
      children: [
        Icon(icon, size: 16, color: Theme.of(context).primaryColor),
        const SizedBox(height: 4),
        Text(
          value,
          style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 12),
        ),
        Text(
          label,
          style: TextStyle(fontSize: 10, color: Colors.grey.shade600),
        ),
      ],
    );
  }

  Widget _buildMainContent() {
    if (_selectedDeviceId == null) {
      return _buildSelectDeviceView();
    }

    if (_isLoading) {
      return _buildLoadingView();
    }

    return TabBarView(
      controller: _tabController,
      children: [
        _buildMapEditor(),
        _buildPGMEditor(),
        _buildLocationsView(),
        _buildAnalysisView(),
      ],
    );
  }

  Widget _buildLoadingView() {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          AnimatedBuilder(
            animation: _loadingAnimationController,
            builder: (context, child) {
              return Transform.rotate(
                angle: _loadingAnimationController.value * 2 * 3.14159,
                child: Container(
                  width: 60,
                  height: 60,
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [
                        Theme.of(context).primaryColor,
                        Theme.of(context).primaryColor.withOpacity(0.3),
                      ],
                    ),
                    shape: BoxShape.circle,
                  ),
                  child: const Icon(Icons.map, color: Colors.white, size: 30),
                ),
              );
            },
          ),
          const SizedBox(height: 20),
          Text(
            'Loading Map Data...',
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

  Widget _buildMapEditor() {
    if (_currentMap == null) {
      return const Center(child: Text('No map data available'));
    }

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
      ),
      child: EnhancedMapCanvas(
        mapData: _currentMap,
        onMapChanged: _onMapChanged,
        deviceId: _selectedDeviceId,
        enableRealTimeUpdates: _isWebSocketConnected,
      ),
    );
  }

  Widget _buildPGMEditor() {
    if (_currentMap == null) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.map, size: 64, color: Colors.grey.shade400),
            const SizedBox(height: 16),
            Text(
              'No map data available for PGM editing',
              style: TextStyle(
                fontSize: 18,
                color: Colors.grey.shade600,
              ),
            ),
            const SizedBox(height: 8),
            Text(
              'Load a map first to enable GIMP-like editing',
              style: TextStyle(color: Colors.grey.shade500),
            ),
          ],
        ),
      );
    }

    return EnhancedPGMMapEditor(
      mapData: _currentMap,
      onMapChanged: _onMapChanged,
      deviceId: _selectedDeviceId,
    );
  }

  Widget _buildSelectDeviceView() {
    return Center(
      child: Container(
        constraints: const BoxConstraints(maxWidth: 400),
        padding: const EdgeInsets.all(24),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Container(
              width: 100,
              height: 100,
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [
                    Theme.of(context).primaryColor,
                    Theme.of(context).primaryColor.withOpacity(0.7),
                  ],
                ),
                shape: BoxShape.circle,
                boxShadow: [
                  BoxShadow(
                    color: Theme.of(context).primaryColor.withOpacity(0.3),
                    blurRadius: 20,
                    offset: const Offset(0, 8),
                  ),
                ],
              ),
              child: const Icon(Icons.map, size: 50, color: Colors.white),
            ),
            const SizedBox(height: 24),
            Text(
              'Select a Device',
              style: TextStyle(
                fontSize: 28,
                fontWeight: FontWeight.bold,
                color: Colors.grey.shade800,
              ),
            ),
            const SizedBox(height: 8),
            Text(
              'Choose a connected AGV device to edit its map',
              style: TextStyle(
                fontSize: 16,
                color: Colors.grey.shade600,
              ),
              textAlign: TextAlign.center,
            ),
            const SizedBox(height: 32),
            if (!_isWebSocketConnected) ...[
              Container(
                padding: const EdgeInsets.all(16),
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [
                      Colors.orange.shade50,
                      Colors.orange.shade100,
                    ],
                  ),
                  border: Border.all(color: Colors.orange.shade300),
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Row(
                  children: [
                    Icon(Icons.warning, color: Colors.orange.shade700),
                    const SizedBox(width: 12),
                    Expanded(
                      child: Text(
                        'WebSocket disconnected. Real-time features unavailable.',
                        style: TextStyle(color: Colors.orange.shade700),
                      ),
                    ),
                    TextButton(
                      onPressed: _reconnectWebSocket,
                      child: const Text('Reconnect'),
                    ),
                  ],
                ),
              ),
              const SizedBox(height: 24),
            ],
            if (_connectedDevices.isEmpty) ...[
              Text(
                'No connected devices found',
                style: TextStyle(color: Colors.grey.shade600),
              ),
              const SizedBox(height: 16),
              ElevatedButton.icon(
                onPressed: () => Navigator.pushNamed(context, '/connect'),
                icon: const Icon(Icons.add),
                label: const Text('Connect Devices'),
                style: ElevatedButton.styleFrom(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 24, vertical: 12),
                ),
              ),
              const SizedBox(height: 8),
              TextButton(
                onPressed: _loadConnectedDevices,
                child: const Text('Refresh Device List'),
              ),
            ] else
              _buildDeviceList(),
          ],
        ),
      ),
    );
  }

  Widget _buildDeviceList() {
    return Column(
      children: _connectedDevices.map((device) {
        final deviceId = device['id']?.toString() ?? '';
        final deviceName = device['name']?.toString() ?? deviceId;
        final deviceStatus = device['status']?.toString() ?? 'unknown';

        return Container(
          margin: const EdgeInsets.symmetric(vertical: 4),
          child: Card(
            elevation: 4,
            shape:
                RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
            child: InkWell(
              borderRadius: BorderRadius.circular(12),
              onTap: () {
                setState(() {
                  _selectedDeviceId = deviceId;
                });
                _loadMapForDevice(deviceId);
              },
              child: Padding(
                padding: const EdgeInsets.all(16),
                child: Row(
                  children: [
                    Container(
                      width: 50,
                      height: 50,
                      decoration: BoxDecoration(
                        gradient: LinearGradient(
                          colors: deviceStatus == 'connected'
                              ? [Colors.green.shade400, Colors.green.shade600]
                              : [Colors.grey.shade400, Colors.grey.shade600],
                        ),
                        borderRadius: BorderRadius.circular(25),
                      ),
                      child: Icon(
                        deviceStatus == 'connected'
                            ? Icons.smart_toy
                            : Icons.warning,
                        color: Colors.white,
                      ),
                    ),
                    const SizedBox(width: 16),
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            deviceName,
                            style: const TextStyle(
                              fontSize: 16,
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                          Text('ID: $deviceId'),
                          Container(
                            padding: const EdgeInsets.symmetric(
                                horizontal: 8, vertical: 2),
                            decoration: BoxDecoration(
                              color: deviceStatus == 'connected'
                                  ? Colors.green.shade100
                                  : Colors.orange.shade100,
                              borderRadius: BorderRadius.circular(8),
                            ),
                            child: Text(
                              deviceStatus.toUpperCase(),
                              style: TextStyle(
                                fontSize: 10,
                                fontWeight: FontWeight.bold,
                                color: deviceStatus == 'connected'
                                    ? Colors.green.shade700
                                    : Colors.orange.shade700,
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                    Icon(
                      Icons.chevron_right,
                      color: Colors.grey.shade400,
                    ),
                  ],
                ),
              ),
            ),
          ),
        );
      }).toList(),
    );
  }

  Widget _buildLocationsView() {
    if (_currentMap == null) {
      return const Center(child: Text('No map data available'));
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
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildLocationHeader(locations.length),
          const SizedBox(height: 16),
          if (locations.isEmpty)
            _buildEmptyLocationsCard()
          else
            ...groupedLocations.entries
                .map((entry) => _buildLocationTypeCard(entry)),
        ],
      ),
    );
  }

  Widget _buildLocationHeader(int totalLocations) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Theme.of(context).primaryColor.withOpacity(0.1),
            Theme.of(context).primaryColor.withOpacity(0.05),
          ],
        ),
        borderRadius: BorderRadius.circular(12),
        border:
            Border.all(color: Theme.of(context).primaryColor.withOpacity(0.2)),
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Theme.of(context).primaryColor,
              borderRadius: BorderRadius.circular(12),
            ),
            child: const Icon(Icons.location_on, color: Colors.white, size: 24),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  'Defined Locations',
                  style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
                ),
                Text(
                  'Total: $totalLocations locations',
                  style: TextStyle(color: Colors.grey.shade600),
                ),
              ],
            ),
          ),
          if (totalLocations > 0)
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              decoration: BoxDecoration(
                color: Theme.of(context).primaryColor,
                borderRadius: BorderRadius.circular(20),
              ),
              child: Text(
                '$totalLocations',
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

  Widget _buildEmptyLocationsCard() {
    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      child: Padding(
        padding: const EdgeInsets.all(32),
        child: Column(
          children: [
            Icon(
              Icons.location_off,
              size: 64,
              color: Colors.grey.shade400,
            ),
            const SizedBox(height: 16),
            Text(
              'No locations defined',
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
                color: Colors.grey.shade600,
              ),
            ),
            const SizedBox(height: 8),
            Text(
              'Add locations using the map editor',
              style: TextStyle(color: Colors.grey.shade500),
            ),
            const SizedBox(height: 16),
            ElevatedButton.icon(
              onPressed: () => _tabController.animateTo(0),
              icon: const Icon(Icons.edit),
              label: const Text('Go to Editor'),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildLocationTypeCard(MapEntry<String, List<MapShape>> entry) {
    final color = _getLocationTypeColor(entry.key);

    return Card(
      margin: const EdgeInsets.symmetric(vertical: 8),
      elevation: 3,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      child: Column(
        children: [
          Container(
            padding: const EdgeInsets.all(16),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [color.withOpacity(0.1), color.withOpacity(0.05)],
              ),
              borderRadius: const BorderRadius.only(
                topLeft: Radius.circular(12),
                topRight: Radius.circular(12),
              ),
            ),
            child: Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    color: color,
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Icon(
                    _getLocationTypeIcon(entry.key),
                    color: Colors.white,
                    size: 20,
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: Text(
                    '${entry.key.toUpperCase()} LOCATIONS',
                    style: const TextStyle(
                      fontSize: 16,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                ),
                Container(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                  decoration: BoxDecoration(
                    color: color,
                    borderRadius: BorderRadius.circular(20),
                  ),
                  child: Text(
                    '${entry.value.length}',
                    style: const TextStyle(
                      color: Colors.white,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                ),
              ],
            ),
          ),
          ListView.builder(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            itemCount: entry.value.length,
            itemBuilder: (context, index) {
              return _buildLocationListItem(entry.value[index], color);
            },
          ),
        ],
      ),
    );
  }

  Widget _buildLocationListItem(MapShape location, Color color) {
    return ListTile(
      leading: Container(
        width: 40,
        height: 40,
        decoration: BoxDecoration(
          gradient: LinearGradient(
            colors: [color.withOpacity(0.8), color],
          ),
          borderRadius: BorderRadius.circular(8),
        ),
        child: Icon(
          _getLocationTypeIcon(location.type),
          color: Colors.white,
          size: 20,
        ),
      ),
      title: Text(
        location.name,
        style: const TextStyle(fontWeight: FontWeight.bold),
      ),
      subtitle: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Position: ${location.center.x.toStringAsFixed(2)}, ${location.center.y.toStringAsFixed(2)}',
          ),
          if (location.sides.values.any((side) => side.isNotEmpty))
            Text(
              'Sides: ${location.sides.entries.where((e) => e.value.isNotEmpty).map((e) => '${e.key}: ${e.value}').join(', ')}',
              style: TextStyle(fontSize: 12, color: Colors.grey.shade600),
            ),
        ],
      ),
      trailing: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          IconButton(
            icon: const Icon(Icons.edit),
            onPressed: () => _editLocation(location),
            tooltip: 'Edit Location',
          ),
          IconButton(
            icon: const Icon(Icons.navigation),
            onPressed: () => _navigateToLocation(location),
            tooltip: 'Navigate Here',
          ),
        ],
      ),
    );
  }

  Color _getLocationTypeColor(String type) {
    switch (type) {
      case 'pickup':
        return Colors.orange;
      case 'drop':
        return Colors.green;
      case 'charging':
        return Colors.blue;
      case 'waypoint':
        return Colors.purple;
      case 'obstacle':
        return Colors.red;
      default:
        return Colors.grey;
    }
  }

  Color _getSourceColor(String source) {
    switch (source.toLowerCase()) {
      case 'normalized':
        return Colors.blue;
      case 'empty_fallback':
      case 'empty_created':
      case 'empty_not_found':
      case 'empty_error':
        return Colors.orange;
      case 'saved':
        return Colors.purple;
      case 'live':
      case 'api':
        return Colors.green;
      default:
        return Colors.grey;
    }
  }

  IconData _getLocationTypeIcon(String type) {
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

  Widget _buildAnalysisView() {
    if (_currentMap == null) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.analytics, size: 64, color: Colors.grey.shade400),
            const SizedBox(height: 16),
            Text(
              'No map data to analyze',
              style: TextStyle(
                fontSize: 18,
                color: Colors.grey.shade600,
              ),
            ),
          ],
        ),
      );
    }

    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildAnalysisHeader(),
          const SizedBox(height: 16),
          _buildAnalysisCards(),
        ],
      ),
    );
  }

  Widget _buildAnalysisHeader() {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.blue.shade50,
            Colors.blue.shade100,
          ],
        ),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.blue.shade300),
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Colors.blue,
              borderRadius: BorderRadius.circular(12),
            ),
            child: const Icon(Icons.analytics, color: Colors.white, size: 24),
          ),
          const SizedBox(width: 16),
          const Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Map Analysis',
                  style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
                ),
                Text('Detailed analysis of your map data'),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildAnalysisCards() {
    final mapData = _currentMap!;

    // Calculate analysis data
    final shapesByType = <String, int>{};
    for (final shape in mapData.shapes) {
      shapesByType[shape.type] = (shapesByType[shape.type] ?? 0) + 1;
    }

    final occupiedCells =
        mapData.occupancyData.where((cell) => cell == 100).length;
    final freeCells = mapData.occupancyData.where((cell) => cell == 0).length;
    final unknownCells =
        mapData.occupancyData.where((cell) => cell == -1).length;
    final totalCells = mapData.occupancyData.length;

    final mapAreaM2 = (mapData.info.width * mapData.info.resolution) *
        (mapData.info.height * mapData.info.resolution);

    return Column(
      children: [
        // Map coverage card
        _buildAnalysisCard(
          'Map Coverage',
          Icons.map,
          Colors.blue,
          [
            _buildAnalysisItem(
              'Total Coverage',
              '${((freeCells + occupiedCells) / totalCells * 100).toStringAsFixed(1)}%',
              Colors.blue,
            ),
            _buildAnalysisItem(
              'Free Space',
              '${(freeCells / totalCells * 100).toStringAsFixed(1)}%',
              Colors.green,
            ),
            _buildAnalysisItem(
              'Obstacles',
              '${(occupiedCells / totalCells * 100).toStringAsFixed(1)}%',
              Colors.red,
            ),
            _buildAnalysisItem(
              'Unknown',
              '${(unknownCells / totalCells * 100).toStringAsFixed(1)}%',
              Colors.grey,
            ),
          ],
        ),

        const SizedBox(height: 16),

        // Map properties card
        _buildAnalysisCard(
          'Map Properties',
          Icons.info,
          Colors.purple,
          [
            _buildAnalysisItem(
              'Total Area',
              '${mapAreaM2.toStringAsFixed(1)} m²',
              Colors.purple,
            ),
            _buildAnalysisItem(
              'Resolution',
              '${mapData.info.resolution.toStringAsFixed(3)} m/px',
              Colors.indigo,
            ),
            _buildAnalysisItem(
              'Dimensions',
              '${mapData.info.width} × ${mapData.info.height}',
              Colors.teal,
            ),
            _buildAnalysisItem(
              'Version',
              '${mapData.version}',
              Colors.amber,
            ),
          ],
        ),

        if (shapesByType.isNotEmpty) ...[
          const SizedBox(height: 16),
          _buildAnalysisCard(
            'Locations Summary',
            Icons.location_on,
            Colors.green,
            shapesByType.entries
                .map(
                  (entry) => _buildAnalysisItem(
                    entry.key.toUpperCase(),
                    '${entry.value}',
                    _getLocationTypeColor(entry.key),
                  ),
                )
                .toList(),
          ),
        ],

        const SizedBox(height: 16),

        // Timestamp info
        _buildAnalysisCard(
          'Timestamps',
          Icons.schedule,
          Colors.orange,
          [
            _buildAnalysisItem(
              'Last Updated',
              _formatTimestamp(mapData.timestamp),
              Colors.orange,
            ),
            _buildAnalysisItem(
              'Created',
              _formatTimestamp(mapData.timestamp),
              Colors.deepOrange,
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildAnalysisCard(
      String title, IconData icon, Color color, List<Widget> items) {
    return Card(
      elevation: 3,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      child: Column(
        children: [
          Container(
            padding: const EdgeInsets.all(16),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [color.withOpacity(0.1), color.withOpacity(0.05)],
              ),
              borderRadius: const BorderRadius.only(
                topLeft: Radius.circular(12),
                topRight: Radius.circular(12),
              ),
            ),
            child: Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    color: color,
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Icon(icon, color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                Text(
                  title,
                  style: const TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(16),
            child: Column(children: items),
          ),
        ],
      ),
    );
  }

  Widget _buildAnalysisItem(String label, String value, Color color) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          Container(
            width: 4,
            height: 20,
            decoration: BoxDecoration(
              color: color,
              borderRadius: BorderRadius.circular(2),
            ),
          ),
          const SizedBox(width: 12),
          Expanded(child: Text(label)),
          Text(
            value,
            style: const TextStyle(fontWeight: FontWeight.bold),
          ),
        ],
      ),
    );
  }

  String _formatTimestamp(DateTime timestamp) {
    final now = DateTime.now();
    final difference = now.difference(timestamp);

    if (difference.inMinutes < 1) {
      return 'Just now';
    } else if (difference.inHours < 1) {
      return '${difference.inMinutes}m ago';
    } else if (difference.inDays < 1) {
      return '${difference.inHours}h ago';
    } else {
      return '${difference.inDays}d ago';
    }
  }

  Widget _buildSidebar() {
    return Container(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Map Tools',
            style: TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.bold,
              color: Colors.grey.shade800,
            ),
          ),
          const SizedBox(height: 16),

          // Device selector
          _buildSidebarDeviceSelector(),

          const SizedBox(height: 20),

          // Quick actions
          _buildSidebarSection(
            'Quick Actions',
            [
              _buildSidebarButton(
                'Save Map',
                Icons.save,
                _hasUnsavedChanges ? _saveMap : null,
                _hasUnsavedChanges ? Colors.orange : Colors.grey,
              ),
              _buildSidebarButton(
                'Send to AGV',
                Icons.send,
                _currentMap != null ? _sendMapToAGV : null,
                Colors.blue,
              ),
              _buildSidebarButton(
                'Clear All',
                Icons.clear_all,
                _currentMap != null ? _clearAllData : null,
                Colors.red,
              ),
            ],
          ),

          const SizedBox(height: 20),

          // Map info
          if (_currentMap != null) _buildMapInfo(),
        ],
      ),
    );
  }

  Widget _buildSidebarDeviceSelector() {
    // Get unique devices and check if selected device exists
    final uniqueDevices = _connectedDevices
        .fold<Map<String, Map<String, dynamic>>>(
          {},
          (map, device) {
            final deviceId = device['id']?.toString() ?? '';
            if (deviceId.isNotEmpty) {
              map[deviceId] = device;
            }
            return map;
          },
        )
        .values
        .toList();

    // Validate selected device exists in unique devices
    final selectedDeviceExists = _selectedDeviceId != null &&
        uniqueDevices
            .any((device) => device['id']?.toString() == _selectedDeviceId);

    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.grey.shade300),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Text(
            'Selected Device',
            style: TextStyle(fontWeight: FontWeight.bold),
          ),
          const SizedBox(height: 8),
          DropdownButton<String>(
            value: selectedDeviceExists ? _selectedDeviceId : null,
            hint: const Text('Select Device'),
            isExpanded: true,
            items: uniqueDevices.map((device) {
              final deviceId = device['id']?.toString() ?? '';
              final deviceName = device['name']?.toString() ?? deviceId;
              final deviceStatus = device['status']?.toString() ?? 'unknown';

              return DropdownMenuItem<String>(
                value: deviceId,
                child: Row(
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
                    const SizedBox(width: 8),
                    Expanded(child: Text(deviceName)),
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
                });
                _loadMapForDevice(deviceId);
              }
            },
          ),
        ],
      ),
    );
  }

  Widget _buildSidebarSection(String title, List<Widget> children) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          title,
          style: const TextStyle(fontWeight: FontWeight.bold),
        ),
        const SizedBox(height: 8),
        ...children,
      ],
    );
  }

  Widget _buildSidebarButton(
      String text, IconData icon, VoidCallback? onPressed, Color color) {
    return Container(
      margin: const EdgeInsets.symmetric(vertical: 2),
      child: SizedBox(
        width: double.infinity,
        child: ElevatedButton.icon(
          onPressed: onPressed,
          icon: Icon(icon),
          label: Text(text),
          style: ElevatedButton.styleFrom(
            backgroundColor: onPressed != null ? color : Colors.grey,
            foregroundColor: Colors.white,
            alignment: Alignment.centerLeft,
          ),
        ),
      ),
    );
  }

  Widget _buildMapInfo() {
    final mapData = _currentMap!;
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.blue.shade50,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.blue.shade200),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Text(
            'Map Information',
            style: TextStyle(fontWeight: FontWeight.bold),
          ),
          const SizedBox(height: 8),
          _buildInfoRow('Version', mapData.version.toString()),
          _buildInfoRow('Shapes', mapData.shapes.length.toString()),
          _buildInfoRow('Size', '${mapData.info.width}×${mapData.info.height}'),
          _buildInfoRow(
              'Resolution', '${mapData.info.resolution.toStringAsFixed(3)}'),
        ],
      ),
    );
  }

  Widget _buildInfoRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label, style: const TextStyle(fontSize: 12)),
          Text(
            value,
            style: const TextStyle(fontSize: 12, fontWeight: FontWeight.bold),
          ),
        ],
      ),
    );
  }

  Widget _buildPropertiesPanel() {
    return Container(
      padding: const EdgeInsets.all(16),
      child: const Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Properties',
            style: TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.bold,
            ),
          ),
          SizedBox(height: 16),
          Text('Select a shape to edit its properties'),
        ],
      ),
    );
  }

  Widget _buildMobileDrawer() {
    return Drawer(
      child: Column(
        children: [
          DrawerHeader(
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [
                  Theme.of(context).primaryColor,
                  Theme.of(context).primaryColor.withOpacity(0.8),
                ],
              ),
            ),
            child: const Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Icon(Icons.map, color: Colors.white, size: 32),
                SizedBox(height: 8),
                Text(
                  'Map Editor',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 24,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                Text(
                  'AGV Navigation System',
                  style: TextStyle(color: Colors.white70),
                ),
              ],
            ),
          ),
          Expanded(
            child: _buildSidebar(),
          ),
        ],
      ),
    );
  }

  Widget _buildFloatingActionButtons() {
    if (_deviceType == DeviceType.phone) {
      return Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          // Load Map FAB
          if (_selectedDeviceId != null)
            FloatingActionButton(
              onPressed: () => _loadMapWithConfirmation(_selectedDeviceId!),
              backgroundColor: Colors.blue,
              heroTag: "load",
              child: const Icon(Icons.refresh, color: Colors.white),
              tooltip: 'Load/Refresh Map',
            ),

          if (_selectedDeviceId != null) const SizedBox(height: 16),

          // Save Map FAB
          FloatingActionButton(
            onPressed: _hasUnsavedChanges ? _saveMap : null,
            backgroundColor:
                _hasUnsavedChanges ? Colors.orange : Colors.grey.shade400,
            heroTag: "save",
            child: Icon(
              Icons.save,
              color: _hasUnsavedChanges ? Colors.white : Colors.grey.shade600,
            ),
            tooltip: _hasUnsavedChanges
                ? 'Save Map (${_currentMap?.shapes.length ?? 0} locations)'
                : 'No changes to save',
          ),
        ],
      );
    }
    return const SizedBox.shrink();
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

    // Show loading indicator
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (context) => const Center(
        child: CircularProgressIndicator(),
      ),
    );

    try {
      print('💾 Saving map for device: $_selectedDeviceId');

      await _apiService.saveMapData(
        deviceId: _selectedDeviceId!,
        mapData: _currentMap!,
      );

      setState(() {
        _hasUnsavedChanges = false;
      });

      // Close loading dialog
      Navigator.of(context).pop();

      _showInfoSnackBar(
          'Map saved successfully with ${_currentMap!.shapes.length} locations');

      print('✅ Map saved successfully for device: $_selectedDeviceId');
    } catch (e) {
      // Close loading dialog
      Navigator.of(context).pop();

      print('❌ Error saving map: $e');

      if (e is ApiException) {
        _showErrorSnackBar('Failed to save map: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to save map: $e');
      }
    }
  }

  void _sendMapToAGV() async {
    if (_currentMap == null || _selectedDeviceId == null) return;

    try {
      // Send map to AGV via WebSocket to /local_costmap/cost_map topic
      _webSocketService.sendMessage({
        'type': 'publish_costmap',
        'topic': '/local_costmap/cost_map',
        'deviceId': _selectedDeviceId,
        'data': {
          'header': {
            'stamp': DateTime.now().millisecondsSinceEpoch,
            'frame_id': 'map',
          },
          'info': {
            'map_load_time': DateTime.now().millisecondsSinceEpoch,
            'resolution': _currentMap!.info.resolution,
            'width': _currentMap!.info.width,
            'height': _currentMap!.info.height,
            'origin': {
              'position': {
                'x': _currentMap!.info.origin.x,
                'y': _currentMap!.info.origin.y,
                'z': _currentMap!.info.origin.z,
              },
              'orientation': {
                'x': _currentMap!.info.originOrientation.x,
                'y': _currentMap!.info.originOrientation.y,
                'z': _currentMap!.info.originOrientation.z,
                'w': _currentMap!.info.originOrientation.w,
              },
            },
          },
          'data': _currentMap!.occupancyData,
        },
      });

      _showInfoSnackBar('Map sent to AGV successfully');
    } catch (e) {
      _showErrorSnackBar('Failed to send map to AGV: $e');
    }
  }

  void _clearAllData() async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('Clear All Data'),
        content: const Text(
            'This will clear all shapes and annotations. This action cannot be undone.'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(false),
            child: const Text('Cancel'),
          ),
          TextButton(
            onPressed: () => Navigator.of(context).pop(true),
            child: const Text('Clear All', style: TextStyle(color: Colors.red)),
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

  void _editLocation(MapShape location) {
    setState(() {
      _showPropertiesPanel = true;
    });
    // TODO: Implement location editing
  }

  void _navigateToLocation(MapShape location) async {
    if (_selectedDeviceId == null) return;

    try {
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

  void _reconnectWebSocket() async {
    final wsUrl = _apiService.getWebSocketUrl();
    if (wsUrl == null) {
      _showErrorSnackBar('WebSocket URL is not available.');
      return;
    }
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
          title: const Text('Connection Information'),
          content: SingleChildScrollView(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisSize: MainAxisSize.min,
              children: [
                _buildConnectionInfoRow(
                    'Status', info['connected'] ? 'Connected' : 'Disconnected'),
                _buildConnectionInfoRow('Base URL', info['baseUrl']),
                _buildConnectionInfoRow('API URL', info['apiBaseUrl']),
                _buildConnectionInfoRow('WebSocket URL', info['websocketUrl']),
                if (info['serverInfo'] != null) ...[
                  const SizedBox(height: 16),
                  const Text('Server Info:',
                      style: TextStyle(fontWeight: FontWeight.bold)),
                  _buildConnectionInfoRow(
                      'Server Status', info['serverInfo']['status']),
                ],
                if (info['error'] != null) ...[
                  const SizedBox(height: 16),
                  const Text('Error:',
                      style: TextStyle(
                          fontWeight: FontWeight.bold, color: Colors.red)),
                  Text(info['error'],
                      style: const TextStyle(color: Colors.red)),
                ],
              ],
            ),
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: const Text('Close'),
            ),
          ],
        ),
      );
    } catch (e) {
      _showErrorSnackBar('Failed to get connection info: $e');
    }
  }

  void _showSavedMaps() {
    if (_selectedDeviceId == null) {
      _showErrorSnackBar('Please select a device first');
      return;
    }

    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (context) => SavedMapsScreen(
          deviceId: _selectedDeviceId!,
          onMapSelected: (mapData) {
            setState(() {
              _currentMap = mapData;
              _loadSource = 'saved_selected';
              _error = null;
              _hasUnsavedChanges = false;
            });
            _showInfoSnackBar('Map loaded from saved maps');
          },
        ),
      ),
    );
  }

  // NEW: Load ROS2 saved map
  Future<void> _loadROS2SavedMap(String mapName) async {
    setState(() {
      _isLoading = true;
      _error = null;
    });

    try {
      print('🔄 Loading ROS2 saved map: $mapName');

      final response = await _apiService.loadROS2SavedMap(
        deviceId: _selectedDeviceId!,
        mapName: mapName,
      );

      if (response['success'] == true && response['mapData'] != null) {
        final mapData = MapData.fromJson(response['mapData']);
        setState(() {
          _currentMap = mapData;
          _hasUnsavedChanges = false;
          _loadSource = 'ros2_saved';
          _error = null;
        });

        _showInfoSnackBar('ROS2 saved map loaded: $mapName');
        print('✅ ROS2 saved map loaded successfully: $mapName');
      } else {
        setState(() {
          _error = response['error'] ?? 'Failed to load ROS2 saved map';
        });
        _showErrorSnackBar('Failed to load ROS2 map: ${response['error']}');
      }
    } catch (e) {
      print('❌ Error loading ROS2 saved map: $e');
      setState(() {
        _error = 'Failed to load ROS2 saved map: $e';
      });
      _showErrorSnackBar('Failed to load ROS2 saved map: $e');
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  // NEW: Show ROS2 saved maps dialog
  void _showROS2SavedMaps() {
    if (_selectedDeviceId == null) {
      _showErrorSnackBar('Please select a device first');
      return;
    }

    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (context) => ROS2SavedMapsScreen(
          deviceId: _selectedDeviceId!,
          onMapSelected: (mapName) {
            _loadROS2SavedMap(mapName);
          },
        ),
      ),
    );
  }

  Widget _buildConnectionInfoRow(String label, String value, {Color? color}) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          SizedBox(
            width: 120,
            child: Text(
              '$label:',
              style: const TextStyle(fontWeight: FontWeight.w500),
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

  void _showErrorSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.red,
        duration: const Duration(seconds: 4),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      ),
    );
  }

  void _showWarningSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.orange,
        duration: const Duration(seconds: 3),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      ),
    );
  }

  void _showInfoSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.blue,
        duration: const Duration(seconds: 2),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      ),
    );
  }

  /// Bottom controls for compact/mobile layout
  Widget _buildCompactBottomControls() {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      color: Colors.white,
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: [
          IconButton(
            icon: const Icon(Icons.dashboard),
            color: Theme.of(context).primaryColor,
            tooltip: 'Back to Dashboard',
            onPressed: () {
              Navigator.pushReplacementNamed(context, '/dashboard');
            },
          ),
          IconButton(
            icon: const Icon(Icons.save),
            color: _hasUnsavedChanges ? Colors.orange : Colors.grey,
            tooltip: 'Save Map',
            onPressed: _hasUnsavedChanges ? _saveMap : null,
          ),
          IconButton(
            icon: const Icon(Icons.refresh),
            color: Colors.blue,
            tooltip: 'Refresh Map',
            onPressed: _selectedDeviceId != null
                ? () => _loadMapWithConfirmation(_selectedDeviceId!)
                : null,
          ),
          IconButton(
            icon: const Icon(Icons.send),
            color: Colors.blue,
            tooltip: 'Send to AGV',
            onPressed: _currentMap != null ? _sendMapToAGV : null,
          ),
          IconButton(
            icon: const Icon(Icons.clear_all),
            color: Colors.red,
            tooltip: 'Clear All',
            onPressed: _currentMap != null ? _clearAllData : null,
          ),
        ],
      ),
    );
  }

  /// Build debug information panel
  Widget _buildDebugPanel() {
    return Container(
      padding: const EdgeInsets.all(12),
      margin: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.orange.shade50,
        border: Border.all(color: Colors.orange.shade200),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              const Icon(Icons.bug_report, size: 16, color: Colors.orange),
              const SizedBox(width: 8),
              Text(
                'Debug Information',
                style: TextStyle(
                    fontWeight: FontWeight.bold, color: Colors.orange.shade800),
              ),
              const Spacer(),
              IconButton(
                onPressed: () => setState(() => _showDebugInfo = false),
                icon: const Icon(Icons.close, size: 16),
                padding: EdgeInsets.zero,
                constraints: const BoxConstraints(),
              ),
            ],
          ),
          const SizedBox(height: 8),
          _buildDebugRow('Device ID', _selectedDeviceId ?? 'None'),
          _buildDebugRow('Load Source', _loadSource ?? 'None'),
          _buildDebugRow('Map Data', _currentMap != null ? 'Loaded' : 'None'),
          if (_currentMap != null) ...[
            _buildDebugRow('Dimensions',
                '${_currentMap!.info.width}x${_currentMap!.info.height}'),
            _buildDebugRow('Shapes', '${_currentMap!.shapes.length}'),
            _buildDebugRow(
                'Occupancy Data', '${_currentMap!.occupancyData.length} cells'),
          ],
          _buildDebugRow(
              'API Initialized', _apiService.isInitialized.toString()),
          _buildDebugRow('API Base URL', _apiService.baseUrl ?? 'None'),
          _buildDebugRow(
              'WebSocket Connected', _isWebSocketConnected.toString()),
          _buildDebugRow('Has Unsaved Changes', _hasUnsavedChanges.toString()),
          if (_rawApiResponse != null) ...[
            const SizedBox(height: 8),
            const Text('Raw API Response:',
                style: TextStyle(fontWeight: FontWeight.bold)),
            Container(
              width: double.infinity,
              padding: const EdgeInsets.all(8),
              decoration: BoxDecoration(
                color: Colors.white,
                borderRadius: BorderRadius.circular(4),
              ),
              child: Text(
                _rawApiResponse.toString(),
                style: const TextStyle(fontSize: 10, fontFamily: 'monospace'),
                maxLines: 5,
                overflow: TextOverflow.ellipsis,
              ),
            ),
          ],
        ],
      ),
    );
  }

  Widget _buildDebugRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          SizedBox(
            width: 120,
            child: Text(
              '$label:',
              style: const TextStyle(fontSize: 12, fontWeight: FontWeight.w500),
            ),
          ),
          Expanded(
            child: Text(
              value,
              style: TextStyle(fontSize: 12, color: Colors.grey.shade700),
            ),
          ),
        ],
      ),
    );
  }

  /// Build error banner
  Widget _buildErrorBanner() {
    return Container(
      padding: const EdgeInsets.all(12),
      margin: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.red.shade50,
        border: Border.all(color: Colors.red.shade200),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        children: [
          Icon(Icons.error, color: Colors.red.shade700),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Map Loading Error',
                  style: TextStyle(
                    fontWeight: FontWeight.bold,
                    color: Colors.red.shade700,
                  ),
                ),
                Text(
                  _error!,
                  style: TextStyle(fontSize: 12, color: Colors.red.shade600),
                ),
              ],
            ),
          ),
          IconButton(
            onPressed: () => setState(() => _error = null),
            icon: const Icon(Icons.close, size: 16),
            padding: EdgeInsets.zero,
            constraints: const BoxConstraints(),
          ),
        ],
      ),
    );
  }
}

/// AGV operation modes for compact mode selector
enum AGVMode { defaultMode, autonomous, mapping }

enum DeviceType { phone, tablet, desktop }

// Enhanced SavedMapsScreen with map loading capability
class SavedMapsScreen extends StatefulWidget {
  final String deviceId;
  final Function(MapData)? onMapSelected;

  const SavedMapsScreen({
    Key? key,
    required this.deviceId,
    this.onMapSelected,
  }) : super(key: key);

  @override
  _SavedMapsScreenState createState() => _SavedMapsScreenState();
}

class _SavedMapsScreenState extends State<SavedMapsScreen> {
  final ApiService _apiService = ApiService();

  List<Map<String, dynamic>> _savedMaps = [];
  bool _isLoading = true;
  String? _error;

  @override
  void initState() {
    super.initState();
    _loadSavedMaps();
  }

  Future<void> _loadSavedMaps() async {
    setState(() {
      _isLoading = true;
      _error = null;
    });

    try {
      final maps = await _apiService.getSavedMaps(widget.deviceId);
      setState(() {
        _savedMaps = maps;
        _isLoading = false;
      });

      print('✅ Loaded ${maps.length} saved maps');
    } catch (e) {
      setState(() {
        _error = 'Failed to load saved maps: $e';
        _isLoading = false;
      });
      print('❌ Error loading saved maps: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Saved Maps (${_savedMaps.length})'),
        actions: [
          IconButton(
            onPressed: _loadSavedMaps,
            icon: const Icon(Icons.refresh),
          ),
        ],
      ),
      body: _buildBody(),
    );
  }

  Widget _buildBody() {
    if (_isLoading) {
      return const Center(child: CircularProgressIndicator());
    }

    if (_error != null) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Icon(Icons.error, size: 64, color: Colors.red),
            const SizedBox(height: 16),
            Text(_error!),
            const SizedBox(height: 16),
            ElevatedButton(
              onPressed: _loadSavedMaps,
              child: const Text('Retry'),
            ),
          ],
        ),
      );
    }

    if (_savedMaps.isEmpty) {
      return const Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.save, size: 64, color: Colors.grey),
            SizedBox(height: 16),
            Text('No saved maps found'),
            SizedBox(height: 8),
            Text(
              'Create and save maps using the map editor',
              style: TextStyle(color: Colors.grey),
            ),
          ],
        ),
      );
    }

    return ListView.builder(
      padding: const EdgeInsets.all(16),
      itemCount: _savedMaps.length,
      itemBuilder: (context, index) {
        final map = _savedMaps[index];
        return _buildMapCard(map);
      },
    );
  }

  Widget _buildMapCard(Map<String, dynamic> map) {
    final mapName = map['name'] ?? 'Unknown Map';
    final savedAt = map['savedAt'] != null
        ? DateTime.tryParse(map['savedAt'].toString())
        : null;
    final shapes = map['shapes'] ?? 0;
    final fileSize = map['fileSize'] ?? 'Unknown';

    return Card(
      margin: const EdgeInsets.only(bottom: 12),
      child: ListTile(
        contentPadding: const EdgeInsets.all(16),
        leading: Container(
          width: 48,
          height: 48,
          decoration: BoxDecoration(
            color: Colors.blue.shade100,
            borderRadius: BorderRadius.circular(8),
          ),
          child: const Icon(Icons.map, color: Colors.blue),
        ),
        title: Text(
          mapName,
          style: const TextStyle(fontWeight: FontWeight.bold),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const SizedBox(height: 4),
            Text('Locations: $shapes • Size: $fileSize'),
            if (savedAt != null)
              Text(
                'Saved: ${_formatDate(savedAt)}',
                style: const TextStyle(fontSize: 12, color: Colors.grey),
              ),
          ],
        ),
        trailing: ElevatedButton.icon(
          onPressed: () => _loadMap(mapName),
          icon: const Icon(Icons.open_in_new, size: 16),
          label: const Text('Load'),
          style: ElevatedButton.styleFrom(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
          ),
        ),
      ),
    );
  }

  Future<void> _loadMap(String mapName) async {
    try {
      showDialog(
        context: context,
        barrierDismissible: false,
        builder: (context) => const AlertDialog(
          content: Row(
            children: [
              CircularProgressIndicator(),
              SizedBox(width: 16),
              Text('Loading map...'),
            ],
          ),
        ),
      );

      final response = await _apiService.loadCompleteMapDataEnhanced(
        deviceId: widget.deviceId,
        mapName: mapName,
      );

      Navigator.of(context).pop(); // Close loading dialog

      if (response['success'] == true && response['mapData'] != null) {
        final mapData = MapData.fromJson(response['mapData']);

        if (widget.onMapSelected != null) {
          widget.onMapSelected!(mapData);
          Navigator.of(context).pop(); // Close saved maps screen
        }

        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('Map loaded: $mapName'),
            backgroundColor: Colors.green,
          ),
        );
      } else {
        throw Exception(response['error'] ?? 'Load failed');
      }
    } catch (e) {
      Navigator.of(context).pop(); // Close loading dialog

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Load failed: $e'),
          backgroundColor: Colors.red,
        ),
      );
    }
  }

  String _formatDate(DateTime date) {
    final now = DateTime.now();
    final difference = now.difference(date);

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
}
