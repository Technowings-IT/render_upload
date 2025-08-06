// screens/enhanced_map_page.dart - Modern Robotic Map Editor with Full Theme Integration
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'dart:async';
import 'dart:math' as math;
import '../models/map_data.dart';
import '../services/api_service.dart';
import '../services/theme_service.dart';
import '../widgets/pgm_map_editor.dart';
import '../services/web_socket_service.dart';
import '../widgets/ros2_saved_maps_screen.dart';
import '../widgets/modern_ui_components.dart';
import '../models/odom.dart' as odom;

enum DeviceType { phone, tablet, desktop }

enum AGVMode { defaultMode, autonomous, mapping }

class EnhancedMapPage extends StatefulWidget {
  final String? deviceId;

  const EnhancedMapPage({Key? key, this.deviceId}) : super(key: key);

  @override
  _EnhancedMapPageState createState() => _EnhancedMapPageState();
}

class _EnhancedMapPageState extends State<EnhancedMapPage>
    with TickerProviderStateMixin {
  // Services
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();

  // Controllers
  late TabController _tabController;
  late AnimationController _loadingAnimationController;
  late AnimationController _pulseController;
  late AnimationController _statusAnimationController;
  late StreamSubscription _realTimeSubscription;
  late StreamSubscription _mapEventsSubscription;
  late StreamSubscription _deviceEventsSubscription;
  late StreamSubscription _connectionStateSubscription;

  // State variables
  AGVMode _currentMode = AGVMode.defaultMode;
  MapData? _currentMap;
  List<Map<String, dynamic>> _connectedDevices = <Map<String, dynamic>>[];
  String? _selectedDeviceId;
  bool _isLoading = false;
  bool _hasUnsavedChanges = false;
  bool _isWebSocketConnected = false;

  // Enhanced debugging and error handling
  String? _error;
  String? _loadSource;
  Map<String, dynamic>? _rawApiResponse;
  bool _showDebugInfo = false;

  // UI state for responsive design
  DeviceType _deviceType = DeviceType.phone;
  bool _showSidebar = true;
  bool _showPropertiesPanel = false;

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _initializeConnections();
    _loadConnectedDevices();
    _subscribeToUpdates();
    _selectedDeviceId = widget.deviceId;

    if (_selectedDeviceId != null) {
      _loadMapForDevice(_selectedDeviceId!);
    }

    WidgetsBinding.instance.addPostFrameCallback((_) {
      _updateDeviceType();
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

  void _initializeAnimations() {
    _tabController = TabController(length: 3, vsync: this);
    _loadingAnimationController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );
    _pulseController = AnimationController(
      duration: const Duration(milliseconds: 2000),
      vsync: this,
    )..repeat(reverse: true);
    _statusAnimationController = AnimationController(
      duration: const Duration(milliseconds: 800),
      vsync: this,
    );
  }

  void _updateDeviceType() {
    final screenWidth = MediaQuery.of(context).size.width;
    if (mounted) {
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
  }

  void _initializeConnections() async {
    _apiService.printConnectionInfo();

    final apiConnected = await _apiService.testConnection();
    if (!apiConnected) {
      _showErrorSnackBar(
          'Failed to connect to API server. Check network settings.');
      return;
    }

    if (mounted) {
      setState(() {
        _isWebSocketConnected = _webSocketService.isConnected;
      });
    }

    if (!_isWebSocketConnected) {
      final wsUrl = _apiService.getWebSocketUrl();
      if (wsUrl == null) {
        _showErrorSnackBar('WebSocket URL is not available.');
        return;
      }
      final connected = await _webSocketService.connect(wsUrl);
      if (mounted) {
        setState(() {
          _isWebSocketConnected = connected;
        });
      }

      if (connected) {
        _statusAnimationController.forward();
      }
    }
  }

  void _subscribeToUpdates() {
    _realTimeSubscription = _webSocketService.realTimeData.listen((data) {
      if (mounted && data['deviceId'] == _selectedDeviceId) {
        _handleRealTimeData(data);
      }
    });

    _mapEventsSubscription = _webSocketService.mapEvents.listen((data) {
      if (mounted && data['deviceId'] == _selectedDeviceId) {
        _handleMapEvent(data);
      }
    });

    _deviceEventsSubscription = _webSocketService.deviceEvents.listen((event) {
      if (mounted) {
        _loadConnectedDevices();
      }
    });

    _connectionStateSubscription =
        _webSocketService.connectionState.listen((connected) {
      if (mounted) {
        setState(() {
          _isWebSocketConnected = connected;
        });
        if (connected) {
          _statusAnimationController.forward();
        } else {
          _statusAnimationController.reverse();
        }
      }
    });
  }

  void _handleRealTimeData(Map<String, dynamic> data) {
    if (mounted) {
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
      if (mounted) {
        setState(() {
          _connectedDevices = devices;

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
      }
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
      if (mounted) {
        setState(() {
          _connectedDevices = <Map<String, dynamic>>[];
        });
      }
    }
  }

  void _loadMapWithConfirmation(String deviceId) async {
    if (_hasUnsavedChanges) {
      final theme = Provider.of<ThemeService>(context, listen: false);
      final confirmed = await showDialog<bool>(
        context: context,
        builder: (context) => AlertDialog(
          backgroundColor:
              theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
          title: Text('Unsaved Changes', style: theme.headlineLarge),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text('You have unsaved changes that will be lost.',
                  style: theme.bodyMedium),
              const SizedBox(height: 8),
              Text(
                'Current map: ${_currentMap?.shapes.length ?? 0} locations',
                style: theme.bodyMedium.copyWith(fontWeight: FontWeight.bold),
              ),
            ],
          ),
          actions: [
            ModernActionButton(
              label: 'Cancel',
              icon: Icons.cancel,
              onPressed: () => Navigator.of(context).pop(false),
              isSecondary: true,
            ),
            ModernActionButton(
              label: 'Load Anyway',
              icon: Icons.warning,
              onPressed: () => Navigator.of(context).pop(true),
              backgroundColor: theme.warningColor,
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

    if (mounted) {
      setState(() {
        _isLoading = true;
        _error = null;
        _rawApiResponse = null;
      });
    }
    _loadingAnimationController.repeat();

    try {
      final response = await _apiService.getMapData(deviceId);
      _rawApiResponse = Map<String, dynamic>.from(response);

      if (response['success'] == true && response['mapData'] != null) {
        try {
          final mapData = MapData.fromJson(response['mapData']);
          if (mounted) {
            setState(() {
              _currentMap = mapData;
              _hasUnsavedChanges = false;
              _loadSource = response['source'] ?? 'api';
              _error = null;
            });
          }

          _showInfoSnackBar(
              'Map loaded successfully with ${mapData.shapes.length} locations from ${_loadSource}');
          print(
              '✅ Map loaded successfully for device: $deviceId from source: ${_loadSource}');
        } catch (parseError) {
          print('❌ Error parsing map data: $parseError');

          if (mounted) {
            setState(() {
              _currentMap = _createEmptyMap(deviceId);
              _hasUnsavedChanges = false;
              _error = 'Failed to parse map data: $parseError';
              _loadSource = 'empty_fallback';
            });
          }
          _showWarningSnackBar('Map data corrupted. Created new empty map.');
        }
      } else {
        if (mounted) {
          setState(() {
            _currentMap = _createEmptyMap(deviceId);
            _hasUnsavedChanges = false;
            _error = response['error'] ?? 'No map data available';
            _loadSource = 'empty_created';
          });
        }
        _showInfoSnackBar('No existing map found. Created new empty map.');
      }
    } catch (e) {
      print('❌ Error loading map: $e');
      if (mounted) {
        setState(() {
          _error = 'Failed to load map data: $e';
        });
      }

      if (e is ApiException) {
        if (e.isNotFound) {
          if (mounted) {
            setState(() {
              _currentMap = _createEmptyMap(deviceId);
              _hasUnsavedChanges = false;
              _loadSource = 'empty_not_found';
            });
          }
          _showInfoSnackBar('No map found for device. Created new empty map.');
        } else {
          _showErrorSnackBar('Failed to load map: ${e.message}');
          if (mounted) {
            setState(() {
              _currentMap = _createEmptyMap(deviceId);
              _loadSource = 'empty_error';
            });
          }
        }
      } else {
        _showErrorSnackBar('Failed to load map: $e');
        if (mounted) {
          setState(() {
            _currentMap = _createEmptyMap(deviceId);
            _loadSource = 'empty_error';
          });
        }
      }
    } finally {
      if (mounted) {
        setState(() {
          _isLoading = false;
        });
      }
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

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    return PopScope(
      canPop: false,
      onPopInvokedWithResult: (didPop, result) async {
        if (didPop) return;
        Navigator.pushReplacementNamed(context, '/dashboard');
      },
      child: Scaffold(
        appBar: _buildEnhancedAppBar(theme),
        body: Container(
          decoration: BoxDecoration(gradient: theme.backgroundGradient),
          child: _buildResponsiveBody(theme),
        ),
        drawer:
            _deviceType == DeviceType.phone ? _buildModernDrawer(theme) : null,
        floatingActionButton: _buildFloatingActionButtons(theme),
      ),
    );
  }

  PreferredSizeWidget _buildEnhancedAppBar(ThemeService theme) {
    return AppBar(
      leading: ModernActionButton(
        label: '',
        icon: Icons.arrow_back,
        onPressed: () => Navigator.pushReplacementNamed(context, '/dashboard'),
        isSecondary: true,
      ),
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
              child: Icon(Icons.map, color: Colors.white, size: 20),
            ),
            const SizedBox(width: 12),
            Column(
              mainAxisAlignment: MainAxisAlignment.center,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Map Editor',
                  style: theme.displayMedium.copyWith(
                    fontSize: 18,
                    color: Colors.white,
                  ),
                ),
                if (_currentMap != null)
                  AnimatedBuilder(
                    animation: _pulseController,
                    builder: (context, child) {
                      return Text(
                        _hasUnsavedChanges
                            ? '● ${_currentMap!.shapes.length} locations (unsaved)'
                            : '${_currentMap!.shapes.length} locations (saved)',
                        style: theme.bodySmall.copyWith(
                          color: _hasUnsavedChanges
                              ? Color.lerp(
                                  Colors.orange.shade200,
                                  Colors.orange.shade400,
                                  _pulseController.value)
                              : Colors.white70,
                          fontWeight: _hasUnsavedChanges
                              ? FontWeight.bold
                              : FontWeight.normal,
                        ),
                      );
                    },
                  ),
              ],
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
        // Data source indicator
        if (_loadSource != null)
          Container(
            margin: const EdgeInsets.only(right: 8),
            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [
                  _getSourceColor(_loadSource!, theme),
                  _getSourceColor(_loadSource!, theme).withOpacity(0.8)
                ],
              ),
              borderRadius: theme.borderRadiusSmall,
              boxShadow: [
                BoxShadow(
                  color: _getSourceColor(_loadSource!, theme).withOpacity(0.3),
                  blurRadius: 4,
                  spreadRadius: 1,
                ),
              ],
            ),
            child: Text(
              _loadSource!.toUpperCase(),
              style: theme.bodySmall.copyWith(
                color: Colors.white,
                fontWeight: FontWeight.bold,
                letterSpacing: 0.5,
              ),
            ),
          ),

        // Debug toggle
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 4),
          decoration: BoxDecoration(
            gradient: _showDebugInfo
                ? LinearGradient(
                    colors: [Colors.orange, Colors.orange.shade700])
                : null,
            color: _showDebugInfo ? null : Colors.grey.withOpacity(0.3),
            borderRadius: BorderRadius.circular(20),
          ),
          child: IconButton(
            onPressed: () {
              setState(() {
                _showDebugInfo = !_showDebugInfo;
              });
            },
            icon: Icon(
              Icons.bug_report,
              color: Colors.white,
            ),
            tooltip: 'Toggle Debug Info',
          ),
        ),

        // Enhanced status indicators
        _buildStatusIndicator('API', true, theme.onlineColor, theme),
        const SizedBox(width: 8),
        _buildStatusIndicator(
            'WS',
            _isWebSocketConnected,
            _isWebSocketConnected ? theme.onlineColor : theme.offlineColor,
            theme),
        const SizedBox(width: 16),

        // Device selector for larger screens
        if (_deviceType != DeviceType.phone) _buildDeviceSelector(theme),

        // Action buttons
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
              tabs: [
                Tab(icon: Icon(Icons.brush), text: 'PGM Editor'),
                Tab(icon: Icon(Icons.list), text: 'Locations'),
                Tab(icon: Icon(Icons.analytics), text: 'Analysis'),
              ],
            )
          : null,
    );
  }

  Widget _buildStatusIndicator(
      String label, bool connected, Color color, ThemeService theme) {
    return AnimatedBuilder(
      animation: _statusAnimationController,
      builder: (context, child) {
        return Container(
          padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [color, color.withOpacity(0.8)],
            ),
            borderRadius: theme.borderRadiusSmall,
            boxShadow: connected
                ? [
                    BoxShadow(
                      color: color.withOpacity(
                          0.4 + (_statusAnimationController.value * 0.2)),
                      blurRadius: 6 + (_statusAnimationController.value * 2),
                      spreadRadius: 1,
                    ),
                  ]
                : null,
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
                style: theme.bodySmall.copyWith(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  letterSpacing: 0.5,
                ),
              ),
            ],
          ),
        );
      },
    );
  }

  Widget _buildDeviceSelector(ThemeService theme) {
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

    final selectedDeviceExists = _selectedDeviceId != null &&
        uniqueDevices
            .any((device) => device['id']?.toString() == _selectedDeviceId);

    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 8),
      padding: const EdgeInsets.symmetric(horizontal: 12),
      decoration: BoxDecoration(
        gradient: theme.glassMorphism.gradient,
        borderRadius: theme.borderRadiusSmall,
        border: theme.glassMorphism.border,
      ),
      child: DropdownButton<String>(
        value: selectedDeviceExists ? _selectedDeviceId : null,
        hint: Text('Select Device',
            style: theme.bodyMedium.copyWith(color: theme.accentColor)),
        dropdownColor:
            theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
        style: theme.bodyMedium.copyWith(color: theme.accentColor),
        underline: Container(),
        icon: Icon(Icons.expand_more, color: theme.accentColor, size: 16),
        items: uniqueDevices.map((device) {
          final deviceId = device['id']?.toString() ?? '';
          final deviceName = device['name']?.toString() ?? deviceId;
          final deviceStatus = device['status']?.toString() ?? 'unknown';

          return DropdownMenuItem<String>(
            value: deviceId,
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                RoboticStatusIndicator(
                  status: deviceStatus,
                  label: '',
                  size: 8,
                  animated: deviceStatus == 'connected',
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

  Widget _buildActionButtons(ThemeService theme) {
    return Row(
      children: [
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 4),
          decoration: BoxDecoration(
            gradient: _hasUnsavedChanges
                ? LinearGradient(
                    colors: [Colors.orange.shade400, Colors.orange.shade600])
                : null,
            color: _hasUnsavedChanges ? null : Colors.grey.withOpacity(0.3),
            borderRadius: BorderRadius.circular(20),
            boxShadow: _hasUnsavedChanges ? theme.neonGlow : null,
          ),
          child: IconButton(
            icon: Icon(Icons.save, color: Colors.white),
            onPressed: _hasUnsavedChanges ? () => _saveMap() : null,
            tooltip: 'Save Map',
          ),
        ),
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 4),
          decoration: BoxDecoration(
            gradient: LinearGradient(
                colors: [Colors.blue.shade400, Colors.blue.shade600]),
            borderRadius: BorderRadius.circular(20),
            boxShadow: theme.elevationSmall,
          ),
          child: IconButton(
            icon: Icon(Icons.refresh, color: Colors.white),
            onPressed: _selectedDeviceId != null
                ? () => _loadMapWithConfirmation(_selectedDeviceId!)
                : null,
            tooltip: 'Refresh Map',
          ),
        ),
        _buildMenuButton(theme),
      ],
    );
  }

  Widget _buildMenuButton(ThemeService theme) {
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 4),
      decoration: BoxDecoration(
        gradient: theme.primaryGradient,
        borderRadius: BorderRadius.circular(20),
        boxShadow: theme.elevationSmall,
      ),
      child: PopupMenuButton(
        icon: Icon(Icons.more_vert, color: Colors.white),
        color: theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
        itemBuilder: (context) => [
          PopupMenuItem(
            child: ListTile(
              leading: Icon(Icons.folder, color: theme.accentColor),
              title: Text('Saved Maps', style: theme.bodyMedium),
              dense: true,
            ),
            onTap: () => _showSavedMaps(),
          ),
          PopupMenuItem(
            child: ListTile(
              leading: Icon(Icons.rocket_launch, color: theme.accentColor),
              title: Text('ROS2 Saved Maps', style: theme.bodyMedium),
              dense: true,
            ),
            onTap: () => _showROS2SavedMaps(),
          ),
          PopupMenuItem(
            child: ListTile(
              leading: Icon(Icons.clear_all, color: theme.errorColor),
              title: Text('Clear All', style: theme.bodyMedium),
              dense: true,
            ),
            onTap: _clearAllData,
          ),
          PopupMenuItem(
            child: ListTile(
              leading: Icon(Icons.send, color: theme.successColor),
              title: Text('Send to AGV', style: theme.bodyMedium),
              dense: true,
            ),
            onTap: _sendMapToAGV,
          ),
          PopupMenuItem(
            child: ListTile(
              leading: Icon(Icons.info, color: theme.infoColor),
              title: Text('Connection Info', style: theme.bodyMedium),
              dense: true,
            ),
            onTap: _showConnectionInfo,
          ),
        ],
      ),
    );
  }

  Widget _buildResponsiveBody(ThemeService theme) {
    return Column(
      children: [
        // Debug info panel
        if (_showDebugInfo) _buildDebugPanel(theme),

        // Error display
        if (_error != null) _buildErrorBanner(theme),

        // Main responsive content
        Expanded(
          child: _buildMainResponsiveContent(theme),
        ),
      ],
    );
  }

  Widget _buildMainResponsiveContent(ThemeService theme) {
    switch (_deviceType) {
      case DeviceType.desktop:
        return _buildDesktopLayout(theme);
      case DeviceType.tablet:
        return _buildTabletLayout(theme);
      case DeviceType.phone:
        return _buildPhoneLayout(theme);
    }
  }

  Widget _buildDesktopLayout(ThemeService theme) {
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
                colors: theme.isDarkMode
                    ? [
                        const Color(0xFF1E1E2E).withOpacity(0.95),
                        const Color(0xFF262640).withOpacity(0.95),
                      ]
                    : [
                        Colors.white.withOpacity(0.95),
                        Colors.grey.shade50.withOpacity(0.95),
                      ],
              ),
              border: Border(
                right: BorderSide(
                  color: theme.isDarkMode
                      ? Colors.white.withOpacity(0.1)
                      : Colors.black.withOpacity(0.1),
                ),
              ),
            ),
            child: _buildSidebar(theme),
          ),

        // Main content
        Expanded(
          child: Column(
            children: [
              // Tab bar
              Container(
                decoration: BoxDecoration(
                  gradient: theme.glassMorphism.gradient,
                  border: Border(
                    bottom: BorderSide(
                      color: theme.isDarkMode
                          ? Colors.white.withOpacity(0.1)
                          : Colors.black.withOpacity(0.1),
                    ),
                  ),
                ),
                child: TabBar(
                  controller: _tabController,
                  labelColor: theme.accentColor,
                  unselectedLabelColor: theme.isDarkMode
                      ? Colors.white.withOpacity(0.6)
                      : Colors.black.withOpacity(0.6),
                  indicatorColor: theme.accentColor,
                  indicatorWeight: 3,
                  tabs: [
                    Tab(icon: Icon(Icons.brush), text: 'PGM Editor'),
                    Tab(icon: Icon(Icons.list), text: 'Locations'),
                    Tab(icon: Icon(Icons.analytics), text: 'Analysis'),
                  ],
                ),
              ),

              // Content
              Expanded(
                child: _buildMainContent(theme),
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
                colors: theme.isDarkMode
                    ? [
                        const Color(0xFF262640).withOpacity(0.95),
                        const Color(0xFF1E1E2E).withOpacity(0.95),
                      ]
                    : [
                        Colors.blue.shade50.withOpacity(0.95),
                        Colors.blue.shade100.withOpacity(0.95),
                      ],
              ),
              border: Border(
                left: BorderSide(
                  color: theme.accentColor.withOpacity(0.3),
                ),
              ),
            ),
            child: _buildPropertiesPanel(theme),
          ),
      ],
    );
  }

  Widget _buildTabletLayout(ThemeService theme) {
    return Column(
      children: [
        // Status bar
        _buildStatusBar(theme),

        // Content
        Expanded(
          child: _buildMainContent(theme),
        ),
      ],
    );
  }

  Widget _buildPhoneLayout(ThemeService theme) {
    return Column(
      children: [
        // Compact mode selector for mobile
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
          decoration: BoxDecoration(
            gradient: theme.glassMorphism.gradient,
            border: Border(
              bottom: BorderSide(
                color: theme.isDarkMode
                    ? Colors.white.withOpacity(0.1)
                    : Colors.black.withOpacity(0.1),
              ),
            ),
          ),
          child: Row(
            children: [
              Text('Mode: ', style: theme.headlineMedium),
              Expanded(
                child: SingleChildScrollView(
                  scrollDirection: Axis.horizontal,
                  child: Row(
                    children: [
                      _buildCompactModeButton(
                          'Default', AGVMode.defaultMode, theme),
                      const SizedBox(width: 8),
                      _buildCompactModeButton(
                          'Auto', AGVMode.autonomous, theme),
                      const SizedBox(width: 8),
                      _buildCompactModeButton('Map', AGVMode.mapping, theme),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),

        // Compact status bar
        if (_selectedDeviceId != null) _buildCompactStatusBar(theme),

        // Main content with better space usage
        Expanded(
          child: _selectedDeviceId == null
              ? _buildSelectDeviceView(theme)
              : _isLoading
                  ? ModernLoadingIndicator(
                      message: 'Loading Map Data...', size: 60)
                  : _buildCompactMapEditor(theme),
        ),

        // Compact bottom controls
        _buildCompactBottomControls(theme),
      ],
    );
  }

  Widget _buildCompactModeButton(
      String label, AGVMode mode, ThemeService theme) {
    final isSelected = _currentMode == mode;
    return Container(
      height: 32,
      decoration: BoxDecoration(
        gradient: isSelected ? theme.primaryGradient : null,
        color: isSelected
            ? null
            : theme.isDarkMode
                ? Colors.white.withOpacity(0.1)
                : Colors.black.withOpacity(0.1),
        borderRadius: theme.borderRadiusSmall,
        boxShadow: isSelected ? theme.neonGlow : null,
      ),
      child: ElevatedButton(
        onPressed: () => _setMode(mode),
        style: ElevatedButton.styleFrom(
          backgroundColor: Colors.transparent,
          foregroundColor: isSelected ? Colors.white : theme.accentColor,
          elevation: 0,
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 4),
          minimumSize: const Size(60, 32),
          shape: RoundedRectangleBorder(borderRadius: theme.borderRadiusSmall),
        ),
        child: Text(label,
            style: theme.bodySmall.copyWith(fontWeight: FontWeight.bold)),
      ),
    );
  }

  void _setMode(AGVMode mode) {
    setState(() {
      _currentMode = mode;
    });
  }

  Widget _buildCompactStatusBar(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      decoration: BoxDecoration(
        gradient: theme.glassMorphism.gradient,
        border: Border(
          bottom: BorderSide(
            color: theme.isDarkMode
                ? Colors.white.withOpacity(0.1)
                : Colors.black.withOpacity(0.1),
          ),
        ),
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(4),
            decoration: BoxDecoration(
              gradient: theme.primaryGradient,
              shape: BoxShape.circle,
            ),
            child: Icon(Icons.device_hub, size: 16, color: Colors.white),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              _selectedDeviceId!,
              style: theme.headlineMedium.copyWith(fontSize: 14),
              overflow: TextOverflow.ellipsis,
            ),
          ),
          if (_hasUnsavedChanges)
            AnimatedBuilder(
              animation: _pulseController,
              builder: (context, child) {
                return Container(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                  decoration: BoxDecoration(
                    color: Color.lerp(Colors.orange, Colors.orange.shade700,
                        _pulseController.value),
                    borderRadius: theme.borderRadiusSmall,
                    boxShadow: [
                      BoxShadow(
                        color: Colors.orange
                            .withOpacity(0.3 + (_pulseController.value * 0.2)),
                        blurRadius: 4,
                        spreadRadius: 1,
                      ),
                    ],
                  ),
                  child: Text(
                    'UNSAVED',
                    style: theme.bodySmall.copyWith(
                      color: Colors.white,
                      fontWeight: FontWeight.bold,
                      letterSpacing: 0.5,
                    ),
                  ),
                );
              },
            ),
        ],
      ),
    );
  }

  Widget _buildCompactMapEditor(ThemeService theme) {
    if (_currentMap == null) {
      return Center(
        child: ModernGlassCard(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(Icons.map_outlined, size: 64, color: theme.errorColor),
              const SizedBox(height: 16),
              Text('No map data available', style: theme.headlineLarge),
              Text('Load a map to start editing', style: theme.bodyMedium),
            ],
          ),
        ),
      );
    }
    return Container(
      decoration: BoxDecoration(gradient: theme.backgroundGradient),
      child: EnhancedPGMMapEditor(
        mapData: _currentMap,
        onMapChanged: _onMapChanged,
        deviceId: _selectedDeviceId,
      ),
    );
  }

  Widget _buildCompactBottomControls(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        gradient: theme.glassMorphism.gradient,
        border: Border(
          top: BorderSide(
            color: theme.isDarkMode
                ? Colors.white.withOpacity(0.1)
                : Colors.black.withOpacity(0.1),
          ),
        ),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: [
          _buildCompactControlButton(
              Icons.dashboard,
              'Dashboard',
              theme.infoColor,
              () => Navigator.pushReplacementNamed(context, '/dashboard'),
              theme),
          _buildCompactControlButton(
              Icons.save,
              'Save',
              _hasUnsavedChanges ? theme.warningColor : Colors.grey,
              _hasUnsavedChanges ? _saveMap : null,
              theme),
          _buildCompactControlButton(
              Icons.refresh,
              'Refresh',
              theme.accentColor,
              _selectedDeviceId != null
                  ? () => _loadMapWithConfirmation(_selectedDeviceId!)
                  : null,
              theme),
          _buildCompactControlButton(Icons.send, 'Send', theme.successColor,
              _currentMap != null ? _sendMapToAGV : null, theme),
          _buildCompactControlButton(Icons.clear_all, 'Clear', theme.errorColor,
              _currentMap != null ? _clearAllData : null, theme),
        ],
      ),
    );
  }

  Widget _buildCompactControlButton(IconData icon, String tooltip, Color color,
      VoidCallback? onPressed, ThemeService theme) {
    return Container(
      decoration: BoxDecoration(
        gradient: onPressed != null
            ? LinearGradient(colors: [color, color.withOpacity(0.8)])
            : null,
        color: onPressed == null ? Colors.grey.withOpacity(0.3) : null,
        borderRadius: BorderRadius.circular(12),
        boxShadow: onPressed != null
            ? [
                BoxShadow(
                  color: color.withOpacity(0.3),
                  blurRadius: 4,
                  spreadRadius: 1,
                ),
              ]
            : null,
      ),
      child: IconButton(
        icon: Icon(icon, color: Colors.white),
        onPressed: onPressed,
        tooltip: tooltip,
      ),
    );
  }

  Widget _buildSelectDeviceView(ThemeService theme) {
    return Center(
      child: Container(
        constraints: const BoxConstraints(maxWidth: 400),
        padding: const EdgeInsets.all(24),
        child: ModernGlassCard(
          showGlow: true,
          glowColor: theme.accentColor,
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Container(
                width: 100,
                height: 100,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  gradient: RadialGradient(
                    colors: [
                      theme.accentColor.withOpacity(0.3),
                      theme.accentColor.withOpacity(0.1),
                      Colors.transparent,
                    ],
                  ),
                  boxShadow: theme.neonGlow,
                ),
                child: Icon(Icons.map, size: 50, color: theme.accentColor),
              ),
              const SizedBox(height: 24),
              Text('Select a Device',
                  style: theme.displayLarge.copyWith(fontSize: 28)),
              const SizedBox(height: 8),
              Text(
                'Choose a connected AGV device to edit its map',
                style: theme.bodyLarge,
                textAlign: TextAlign.center,
              ),
              const SizedBox(height: 32),
              if (!_isWebSocketConnected) ...[
                Container(
                  padding: const EdgeInsets.all(16),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [
                        Colors.orange.shade50.withOpacity(0.5),
                        Colors.orange.shade100.withOpacity(0.5),
                      ],
                    ),
                    border: Border.all(color: Colors.orange.withOpacity(0.3)),
                    borderRadius: theme.borderRadiusMedium,
                  ),
                  child: Row(
                    children: [
                      Icon(Icons.warning, color: Colors.orange.shade700),
                      const SizedBox(width: 12),
                      Expanded(
                        child: Text(
                          'WebSocket disconnected. Real-time features unavailable.',
                          style: theme.bodyMedium
                              .copyWith(color: Colors.orange.shade700),
                        ),
                      ),
                      ModernActionButton(
                        label: 'Reconnect',
                        icon: Icons.refresh,
                        onPressed: _reconnectWebSocket,
                        isSecondary: true,
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 24),
              ],
              if (_connectedDevices.isEmpty) ...[
                Text('No connected devices found', style: theme.bodyLarge),
                const SizedBox(height: 16),
                ModernActionButton(
                  label: 'Connect Devices',
                  icon: Icons.add,
                  onPressed: () => Navigator.pushNamed(context, '/connect'),
                ),
                const SizedBox(height: 8),
                ModernActionButton(
                  label: 'Refresh Device List',
                  icon: Icons.refresh,
                  onPressed: _loadConnectedDevices,
                  isSecondary: true,
                ),
              ] else
                _buildDeviceList(theme),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildDeviceList(ThemeService theme) {
    return Column(
      children: _connectedDevices.map((device) {
        final deviceId = device['id']?.toString() ?? '';
        final deviceName = device['name']?.toString() ?? deviceId;
        final deviceStatus = device['status']?.toString() ?? 'unknown';

        return Container(
          margin: const EdgeInsets.symmetric(vertical: 4),
          child: ModernGlassCard(
            onTap: () {
              setState(() {
                _selectedDeviceId = deviceId;
              });
              _loadMapForDevice(deviceId);
            },
            child: Row(
              children: [
                Container(
                  width: 50,
                  height: 50,
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: deviceStatus == 'connected'
                          ? [
                              theme.onlineColor,
                              theme.onlineColor.withOpacity(0.8)
                            ]
                          : [Colors.grey.shade400, Colors.grey.shade600],
                    ),
                    borderRadius: BorderRadius.circular(25),
                    boxShadow: deviceStatus == 'connected'
                        ? [
                            BoxShadow(
                              color: theme.onlineColor.withOpacity(0.3),
                              blurRadius: 8,
                              spreadRadius: 2,
                            ),
                          ]
                        : null,
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
                      Text(deviceName, style: theme.headlineMedium),
                      Text('ID: $deviceId', style: theme.bodyMedium),
                      RoboticStatusIndicator(
                        status: deviceStatus,
                        label: deviceStatus.toUpperCase(),
                        animated: deviceStatus == 'connected',
                      ),
                    ],
                  ),
                ),
                Icon(Icons.chevron_right, color: theme.accentColor),
              ],
            ),
          ),
        );
      }).toList(),
    );
  }

  Widget _buildStatusBar(ThemeService theme) {
    if (_selectedDeviceId == null) return const SizedBox.shrink();

    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            theme.accentColor.withOpacity(0.1),
            theme.accentColor.withOpacity(0.05),
          ],
        ),
        border: Border(
          bottom: BorderSide(
            color: theme.isDarkMode
                ? Colors.white.withOpacity(0.1)
                : Colors.black.withOpacity(0.1),
          ),
        ),
      ),
      child: Column(
        children: [
          Row(
            children: [
              Container(
                padding: const EdgeInsets.all(8),
                decoration: BoxDecoration(
                  gradient: theme.primaryGradient,
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(Icons.device_hub, color: Colors.white, size: 16),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text('Device: $_selectedDeviceId',
                    style: theme.headlineMedium),
              ),
              if (_hasUnsavedChanges)
                AnimatedBuilder(
                  animation: _pulseController,
                  builder: (context, child) {
                    return Container(
                      padding: const EdgeInsets.symmetric(
                          horizontal: 8, vertical: 4),
                      decoration: BoxDecoration(
                        color: Color.lerp(Colors.orange, Colors.orange.shade700,
                            _pulseController.value),
                        borderRadius: theme.borderRadiusSmall,
                        boxShadow: [
                          BoxShadow(
                            color: Colors.orange.withOpacity(
                                0.3 + (_pulseController.value * 0.2)),
                            blurRadius: 6,
                            spreadRadius: 2,
                          ),
                        ],
                      ),
                      child: Text(
                        'UNSAVED',
                        style: theme.bodySmall.copyWith(
                          color: Colors.white,
                          fontWeight: FontWeight.bold,
                          letterSpacing: 0.5,
                        ),
                      ),
                    );
                  },
                ),
            ],
          ),
          const SizedBox(height: 8),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              _buildQuickStat(
                  'Shapes',
                  _currentMap?.shapes.length.toString() ?? '0',
                  Icons.category,
                  theme),
              _buildQuickStat('Version', _currentMap?.version.toString() ?? '0',
                  Icons.numbers, theme),
              _buildQuickStat(
                  'Size',
                  _currentMap != null
                      ? '${_currentMap!.info.width}×${_currentMap!.info.height}'
                      : 'N/A',
                  Icons.aspect_ratio,
                  theme),
              _buildQuickStat(
                  'Resolution',
                  _currentMap?.info.resolution.toStringAsFixed(3) ?? 'N/A',
                  Icons.grid_4x4,
                  theme),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildQuickStat(
      String label, String value, IconData icon, ThemeService theme) {
    return Column(
      children: [
        Icon(icon, size: 16, color: theme.accentColor),
        const SizedBox(height: 4),
        Text(value, style: theme.headlineMedium.copyWith(fontSize: 12)),
        Text(label, style: theme.bodySmall),
      ],
    );
  }

  Widget _buildMainContent(ThemeService theme) {
    if (_selectedDeviceId == null) {
      return _buildSelectDeviceView(theme);
    }

    if (_isLoading) {
      return ModernLoadingIndicator(message: 'Loading Map Data...', size: 60);
    }

    return TabBarView(
      controller: _tabController,
      children: [
        _buildPGMEditor(theme),
        _buildLocationsView(theme),
        _buildAnalysisView(theme),
      ],
    );
  }

  Widget _buildPGMEditor(ThemeService theme) {
    if (_currentMap == null) {
      return Center(
        child: ModernGlassCard(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(Icons.map, size: 64, color: theme.errorColor),
              const SizedBox(height: 16),
              Text('No map data available for PGM editing',
                  style: theme.headlineLarge),
              const SizedBox(height: 8),
              Text('Load a map first to enable GIMP-like editing',
                  style: theme.bodyMedium),
            ],
          ),
        ),
      );
    }

    return EnhancedPGMMapEditor(
      mapData: _currentMap,
      onMapChanged: _onMapChanged,
      deviceId: _selectedDeviceId,
    );
  }

  Widget _buildLocationsView(ThemeService theme) {
    if (_currentMap == null) {
      return Center(
        child: ModernGlassCard(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(Icons.location_off, size: 64, color: theme.errorColor),
              const SizedBox(height: 16),
              Text('No map data available', style: theme.headlineLarge),
            ],
          ),
        ),
      );
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
          _buildLocationHeader(locations.length, theme),
          const SizedBox(height: 16),
          if (locations.isEmpty)
            _buildEmptyLocationsCard(theme)
          else
            ...groupedLocations.entries
                .map((entry) => _buildLocationTypeCard(entry, theme)),
        ],
      ),
    );
  }

  Widget _buildLocationHeader(int totalLocations, ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            theme.accentColor.withOpacity(0.1),
            theme.accentColor.withOpacity(0.05),
          ],
        ),
        borderRadius: theme.borderRadiusMedium,
        border: Border.all(color: theme.accentColor.withOpacity(0.2)),
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              gradient: theme.primaryGradient,
              borderRadius: theme.borderRadiusMedium,
            ),
            child: Icon(Icons.location_on, color: Colors.white, size: 24),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('Defined Locations', style: theme.displayMedium),
                Text('Total: $totalLocations locations',
                    style: theme.bodyMedium),
              ],
            ),
          ),
          if (totalLocations > 0)
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              decoration: BoxDecoration(
                gradient: theme.primaryGradient,
                borderRadius: BorderRadius.circular(20),
              ),
              child: Text(
                '$totalLocations',
                style: theme.bodyMedium.copyWith(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildEmptyLocationsCard(ThemeService theme) {
    return ModernGlassCard(
      child: Column(
        children: [
          Icon(Icons.location_off, size: 64, color: theme.warningColor),
          const SizedBox(height: 16),
          Text('No locations defined', style: theme.headlineLarge),
          const SizedBox(height: 8),
          Text('Add locations using the map editor', style: theme.bodyMedium),
          const SizedBox(height: 16),
          ModernActionButton(
            label: 'Go to Editor',
            icon: Icons.edit,
            onPressed: () => _tabController.animateTo(0),
          ),
        ],
      ),
    );
  }

  Widget _buildLocationTypeCard(
      MapEntry<String, List<MapShape>> entry, ThemeService theme) {
    final color = _getLocationTypeColor(entry.key);

    return Container(
      margin: const EdgeInsets.symmetric(vertical: 8),
      child: ModernGlassCard(
        child: Column(
          children: [
            Container(
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [color.withOpacity(0.1), color.withOpacity(0.05)],
                ),
                borderRadius: BorderRadius.only(
                  topLeft: theme.borderRadiusMedium.topLeft,
                  topRight: theme.borderRadiusMedium.topRight,
                ),
              ),
              child: Row(
                children: [
                  Container(
                    padding: const EdgeInsets.all(8),
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                          colors: [color, color.withOpacity(0.8)]),
                      borderRadius: theme.borderRadiusSmall,
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
                      style: theme.headlineMedium,
                    ),
                  ),
                  Container(
                    padding:
                        const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                          colors: [color, color.withOpacity(0.8)]),
                      borderRadius: BorderRadius.circular(20),
                    ),
                    child: Text(
                      '${entry.value.length}',
                      style: theme.bodyMedium.copyWith(
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
                return _buildLocationListItem(entry.value[index], color, theme);
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildLocationListItem(
      MapShape location, Color color, ThemeService theme) {
    return ListTile(
      leading: Container(
        width: 40,
        height: 40,
        decoration: BoxDecoration(
          gradient: LinearGradient(
            colors: [color.withOpacity(0.8), color],
          ),
          borderRadius: theme.borderRadiusSmall,
        ),
        child: Icon(
          _getLocationTypeIcon(location.type),
          color: Colors.white,
          size: 20,
        ),
      ),
      title: Text(location.name, style: theme.headlineMedium),
      subtitle: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Position: ${location.center.x.toStringAsFixed(2)}, ${location.center.y.toStringAsFixed(2)}',
            style: theme.bodyMedium,
          ),
          if (location.sides.values.any((side) => side.isNotEmpty))
            Text(
              'Sides: ${location.sides.entries.where((e) => e.value.isNotEmpty).map((e) => '${e.key}: ${e.value}').join(', ')}',
              style: theme.bodySmall,
            ),
        ],
      ),
      trailing: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            decoration: BoxDecoration(
              gradient: LinearGradient(
                  colors: [theme.infoColor, theme.infoColor.withOpacity(0.8)]),
              borderRadius: BorderRadius.circular(20),
            ),
            child: IconButton(
              icon: Icon(Icons.edit, color: Colors.white),
              onPressed: () => _editLocation(location),
              tooltip: 'Edit Location',
            ),
          ),
          const SizedBox(width: 8),
          Container(
            decoration: BoxDecoration(
              gradient: LinearGradient(colors: [
                theme.successColor,
                theme.successColor.withOpacity(0.8)
              ]),
              borderRadius: BorderRadius.circular(20),
            ),
            child: IconButton(
              icon: Icon(Icons.navigation, color: Colors.white),
              onPressed: () => _navigateToLocation(location),
              tooltip: 'Navigate Here',
            ),
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

  Widget _buildAnalysisView(ThemeService theme) {
    if (_currentMap == null) {
      return Center(
        child: ModernGlassCard(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(Icons.analytics, size: 64, color: theme.errorColor),
              const SizedBox(height: 16),
              Text('No map data to analyze', style: theme.headlineLarge),
            ],
          ),
        ),
      );
    }

    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildAnalysisHeader(theme),
          const SizedBox(height: 16),
          _buildAnalysisCards(theme),
        ],
      ),
    );
  }

  Widget _buildAnalysisHeader(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            theme.infoColor.withOpacity(0.1),
            theme.infoColor.withOpacity(0.05),
          ],
        ),
        borderRadius: theme.borderRadiusMedium,
        border: Border.all(color: theme.infoColor.withOpacity(0.3)),
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                  colors: [theme.infoColor, theme.infoColor.withOpacity(0.8)]),
              borderRadius: theme.borderRadiusMedium,
            ),
            child: Icon(Icons.analytics, color: Colors.white, size: 24),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('Map Analysis', style: theme.displayMedium),
                Text('Detailed analysis of your map data',
                    style: theme.bodyMedium),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildAnalysisCards(ThemeService theme) {
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
          theme.infoColor,
          [
            _buildAnalysisItem(
              'Total Coverage',
              '${((freeCells + occupiedCells) / totalCells * 100).toStringAsFixed(1)}%',
              theme.infoColor,
              theme,
            ),
            _buildAnalysisItem(
              'Free Space',
              '${(freeCells / totalCells * 100).toStringAsFixed(1)}%',
              theme.successColor,
              theme,
            ),
            _buildAnalysisItem(
              'Obstacles',
              '${(occupiedCells / totalCells * 100).toStringAsFixed(1)}%',
              theme.errorColor,
              theme,
            ),
            _buildAnalysisItem(
              'Unknown',
              '${(unknownCells / totalCells * 100).toStringAsFixed(1)}%',
              Colors.grey,
              theme,
            ),
          ],
          theme,
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
              theme,
            ),
            _buildAnalysisItem(
              'Resolution',
              '${mapData.info.resolution.toStringAsFixed(3)} m/px',
              Colors.indigo,
              theme,
            ),
            _buildAnalysisItem(
              'Dimensions',
              '${mapData.info.width} × ${mapData.info.height}',
              Colors.teal,
              theme,
            ),
            _buildAnalysisItem(
              'Version',
              '${mapData.version}',
              Colors.amber,
              theme,
            ),
          ],
          theme,
        ),

        if (shapesByType.isNotEmpty) ...[
          const SizedBox(height: 16),
          _buildAnalysisCard(
            'Locations Summary',
            Icons.location_on,
            theme.successColor,
            shapesByType.entries
                .map(
                  (entry) => _buildAnalysisItem(
                    entry.key.toUpperCase(),
                    '${entry.value}',
                    _getLocationTypeColor(entry.key),
                    theme,
                  ),
                )
                .toList(),
            theme,
          ),
        ],

        const SizedBox(height: 16),

        // Timestamp info
        _buildAnalysisCard(
          'Timestamps',
          Icons.schedule,
          theme.warningColor,
          [
            _buildAnalysisItem(
              'Last Updated',
              _formatTimestamp(mapData.timestamp),
              theme.warningColor,
              theme,
            ),
            _buildAnalysisItem(
              'Created',
              _formatTimestamp(mapData.timestamp),
              Colors.deepOrange,
              theme,
            ),
          ],
          theme,
        ),
      ],
    );
  }

  Widget _buildAnalysisCard(String title, IconData icon, Color color,
      List<Widget> items, ThemeService theme) {
    return ModernGlassCard(
      child: Column(
        children: [
          Container(
            padding: const EdgeInsets.all(16),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [color.withOpacity(0.1), color.withOpacity(0.05)],
              ),
              borderRadius: BorderRadius.only(
                topLeft: theme.borderRadiusMedium.topLeft,
                topRight: theme.borderRadiusMedium.topRight,
              ),
            ),
            child: Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    gradient:
                        LinearGradient(colors: [color, color.withOpacity(0.8)]),
                    borderRadius: theme.borderRadiusSmall,
                  ),
                  child: Icon(icon, color: Colors.white, size: 20),
                ),
                const SizedBox(width: 12),
                Text(title, style: theme.headlineMedium),
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

  Widget _buildAnalysisItem(
      String label, String value, Color color, ThemeService theme) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          Container(
            width: 4,
            height: 20,
            decoration: BoxDecoration(
              gradient: LinearGradient(colors: [color, color.withOpacity(0.8)]),
              borderRadius: BorderRadius.circular(2),
            ),
          ),
          const SizedBox(width: 12),
          Expanded(child: Text(label, style: theme.bodyMedium)),
          Text(value,
              style: theme.bodyMedium.copyWith(fontWeight: FontWeight.bold)),
        ],
      ),
    );
  }

  // Add these state variables at the top of the class
  final GlobalKey _mapEditorKey = GlobalKey();

  void _centerMap() {
    if (_currentMap != null && _mapEditorKey.currentState != null) {
      (_mapEditorKey.currentState as dynamic).centerMapView();
    }
  }

  // Removed duplicate _loadMapForDevice method

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

  Widget _buildSidebar(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('Map Tools', style: theme.displayMedium),
          const SizedBox(height: 16),
          _buildSidebarDeviceSelector(theme),
          const SizedBox(height: 20),
          ModernSettingsSection(
            title: 'Quick Actions',
            icon: Icons.flash_on,
            children: [
              ModernActionButton(
                label: 'Save Map',
                icon: Icons.save,
                onPressed: _hasUnsavedChanges ? _saveMap : () {},
                backgroundColor: _hasUnsavedChanges ? theme.warningColor : null,
                isLoading: false,
              ),
              const SizedBox(height: 8),
              ModernActionButton(
                label: 'Send to AGV',
                icon: Icons.send,
                onPressed: _currentMap != null ? _sendMapToAGV : () {},
                backgroundColor: theme.successColor,
              ),
              const SizedBox(height: 8),
              ModernActionButton(
                label: 'Clear All',
                icon: Icons.clear_all,
                onPressed: _currentMap != null ? _clearAllData : () {},
                backgroundColor: theme.errorColor,
              ),
            ],
          ),
          const SizedBox(height: 20),
          if (_currentMap != null) _buildMapInfo(theme),
        ],
      ),
    );
  }

  Widget _buildSidebarDeviceSelector(ThemeService theme) {
    return ModernGlassCard(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('Selected Device', style: theme.headlineMedium),
          const SizedBox(height: 8),
          // Device selector implementation would go here
        ],
      ),
    );
  }

  Widget _buildMapInfo(ThemeService theme) {
    final mapData = _currentMap!;
    return ModernGlassCard(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('Map Information', style: theme.headlineMedium),
          const SizedBox(height: 8),
          _buildInfoRow('Version', mapData.version.toString(), theme),
          _buildInfoRow('Shapes', mapData.shapes.length.toString(), theme),
          _buildInfoRow(
              'Size', '${mapData.info.width}×${mapData.info.height}', theme),
          _buildInfoRow('Resolution',
              '${mapData.info.resolution.toStringAsFixed(3)}', theme),
        ],
      ),
    );
  }

  Widget _buildInfoRow(String label, String value, ThemeService theme) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label, style: theme.bodyMedium),
          Text(value,
              style: theme.bodyMedium.copyWith(fontWeight: FontWeight.bold)),
        ],
      ),
    );
  }

  Widget _buildPropertiesPanel(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('Properties', style: theme.displayMedium),
          const SizedBox(height: 16),
          Text('Select a shape to edit its properties',
              style: theme.bodyMedium),
        ],
      ),
    );
  }

  Widget _buildModernDrawer(ThemeService theme) {
    return Drawer(
      backgroundColor: Colors.transparent,
      child: Container(
        decoration: BoxDecoration(gradient: theme.backgroundGradient),
        child: ListView(
          padding: EdgeInsets.zero,
          children: [
            DrawerHeader(
              decoration: BoxDecoration(gradient: theme.primaryGradient),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Container(
                    padding: const EdgeInsets.all(8),
                    decoration: BoxDecoration(
                      color: Colors.white.withOpacity(0.2),
                      borderRadius: theme.borderRadiusSmall,
                    ),
                    child: Icon(Icons.map, size: 40, color: Colors.white),
                  ),
                  const SizedBox(height: 12),
                  Text('Map Editor',
                      style: theme.headlineLarge.copyWith(color: Colors.white)),
                  Text('AGV Navigation System',
                      style: theme.bodySmall.copyWith(color: Colors.white70)),
                ],
              ),
            ),
            // Drawer items would go here
          ],
        ),
      ),
    );
  }

  Widget _buildFloatingActionButtons(ThemeService theme) {
    if (_deviceType == DeviceType.phone) {
      return Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          if (_selectedDeviceId != null)
            Container(
              decoration: BoxDecoration(
                gradient: theme.primaryGradient,
                shape: BoxShape.circle,
                boxShadow: theme.neonGlow,
              ),
              child: FloatingActionButton(
                onPressed: () => _loadMapWithConfirmation(_selectedDeviceId!),
                backgroundColor: Colors.transparent,
                elevation: 0,
                heroTag: "load",
                child: Icon(Icons.refresh, color: Colors.white),
                tooltip: 'Load/Refresh Map',
              ),
            ),
          if (_selectedDeviceId != null) const SizedBox(height: 16),
          Container(
            decoration: BoxDecoration(
              gradient: _hasUnsavedChanges
                  ? LinearGradient(
                      colors: [Colors.orange, Colors.orange.shade700])
                  : LinearGradient(
                      colors: [Colors.grey.shade400, Colors.grey.shade600]),
              shape: BoxShape.circle,
              boxShadow: _hasUnsavedChanges ? theme.neonGlow : null,
            ),
            child: FloatingActionButton(
              onPressed: _hasUnsavedChanges ? _saveMap : null,
              backgroundColor: Colors.transparent,
              elevation: 0,
              heroTag: "save",
              child: Icon(Icons.save, color: Colors.white),
              tooltip: _hasUnsavedChanges
                  ? 'Save Map (${_currentMap?.shapes.length ?? 0} locations)'
                  : 'No changes to save',
            ),
          ),
        ],
      );
    }
    return const SizedBox.shrink();
  }

  Widget _buildDebugPanel(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(12),
      margin: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.orange.shade50.withOpacity(0.8),
            Colors.orange.shade100.withOpacity(0.8),
          ],
        ),
        border: Border.all(color: Colors.orange.withOpacity(0.3)),
        borderRadius: theme.borderRadiusMedium,
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(Icons.bug_report, size: 16, color: Colors.orange.shade800),
              const SizedBox(width: 8),
              Text('Debug Information',
                  style: theme.headlineMedium
                      .copyWith(color: Colors.orange.shade800)),
              const Spacer(),
              IconButton(
                onPressed: () => setState(() => _showDebugInfo = false),
                icon:
                    Icon(Icons.close, size: 16, color: Colors.orange.shade800),
                padding: EdgeInsets.zero,
                constraints: const BoxConstraints(),
              ),
            ],
          ),
          const SizedBox(height: 8),
          _buildDebugRow('Device ID', _selectedDeviceId ?? 'None', theme),
          _buildDebugRow('Load Source', _loadSource ?? 'None', theme),
          _buildDebugRow(
              'Map Data', _currentMap != null ? 'Loaded' : 'None', theme),
          if (_currentMap != null) ...[
            _buildDebugRow(
                'Dimensions',
                '${_currentMap!.info.width}x${_currentMap!.info.height}',
                theme),
            _buildDebugRow('Shapes', '${_currentMap!.shapes.length}', theme),
            _buildDebugRow('Occupancy Data',
                '${_currentMap!.occupancyData.length} cells', theme),
          ],
          _buildDebugRow(
              'WebSocket Connected', _isWebSocketConnected.toString(), theme),
          _buildDebugRow(
              'Has Unsaved Changes', _hasUnsavedChanges.toString(), theme),
        ],
      ),
    );
  }

  Widget _buildDebugRow(String label, String value, ThemeService theme) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          SizedBox(
            width: 120,
            child: Text('$label:',
                style: theme.bodySmall.copyWith(fontWeight: FontWeight.w500)),
          ),
          Expanded(
            child: Text(value, style: theme.bodySmall),
          ),
        ],
      ),
    );
  }

  Widget _buildErrorBanner(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(12),
      margin: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.red.shade50.withOpacity(0.8),
            Colors.red.shade100.withOpacity(0.8),
          ],
        ),
        border: Border.all(color: Colors.red.withOpacity(0.3)),
        borderRadius: theme.borderRadiusMedium,
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
                  style:
                      theme.headlineMedium.copyWith(color: Colors.red.shade700),
                ),
                Text(_error!,
                    style:
                        theme.bodySmall.copyWith(color: Colors.red.shade600)),
              ],
            ),
          ),
          IconButton(
            onPressed: () => setState(() => _error = null),
            icon: Icon(Icons.close, size: 16, color: Colors.red.shade700),
            padding: EdgeInsets.zero,
            constraints: const BoxConstraints(),
          ),
        ],
      ),
    );
  }

  Color _getSourceColor(String source, ThemeService theme) {
    switch (source.toLowerCase()) {
      case 'normalized':
        return theme.infoColor;
      case 'empty_fallback':
      case 'empty_created':
      case 'empty_not_found':
      case 'empty_error':
        return theme.warningColor;
      case 'saved':
        return Colors.purple;
      case 'live':
      case 'api':
        return theme.successColor;
      case 'ros2_saved':
        return Colors.indigo;
      default:
        return Colors.grey;
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

    final theme = Provider.of<ThemeService>(context, listen: false);

    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (context) => AlertDialog(
        backgroundColor:
            theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
        content: Row(
          children: [
            ModernLoadingIndicator(message: '', size: 30),
            const SizedBox(width: 16),
            Text('Saving map...', style: theme.bodyMedium),
          ],
        ),
      ),
    );

    try {
      await _apiService.saveMapData(
        deviceId: _selectedDeviceId!,
        mapData: _currentMap!,
      );

      setState(() {
        _hasUnsavedChanges = false;
      });

      Navigator.of(context).pop(); // Close loading dialog

      _showInfoSnackBar(
          'Map saved successfully with ${_currentMap!.shapes.length} locations');
      print('✅ Map saved successfully for device: $_selectedDeviceId');
    } catch (e) {
      Navigator.of(context).pop(); // Close loading dialog
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
    final theme = Provider.of<ThemeService>(context, listen: false);
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (context) {
        return AlertDialog(
          backgroundColor:
              theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
          title: Text('Clear All Data', style: theme.headlineLarge),
          content: Text(
            'This will clear all shapes and annotations. This action cannot be undone.',
            style: theme.bodyMedium,
          ),
          actions: [
            ModernActionButton(
              label: 'Cancel',
              icon: Icons.cancel,
              onPressed: () => Navigator.of(context).pop(false),
              isSecondary: true,
            ),
            ModernActionButton(
              label: 'Clear All',
              icon: Icons.clear_all,
              onPressed: () => Navigator.of(context).pop(true),
              backgroundColor: theme.errorColor,
            ),
          ],
        );
      },
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

  // (Removed duplicate _loadROS2SavedMap method)

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
  // Removed duplicate and unused _buildCompactBottomControls method

  // (Removed duplicate and unused _buildDebugPanel method)

  // Widget _buildDebugRow(String label, String value) {
  //   return Padding(
  //     padding: const EdgeInsets.symmetric(vertical: 2),
  //     child: Row(
  //       children: [
  //         SizedBox(
  //           width: 120,
  //           child: Text(
  //             '$label:',
  //             style: const TextStyle(fontSize: 12, fontWeight: FontWeight.w500),
  //           ),
  //         ),
  //         Expanded(
  //           child: Text(
  //             value,
  //             style: TextStyle(fontSize: 12, color: Colors.grey.shade700),
  //           ),
  //         ),
  //       ],
  //     ),
  //   );
  // }

  // (Removed duplicate and unused _buildErrorBanner method)
}

/// AGV operation modes for compact mode selector
// Removed duplicate DeviceType enum declaration

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
