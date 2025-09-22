// screens/enhanced_map_page.dart - Fixed Map Positioning & Responsive Design
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'dart:async';
import '../models/map_data.dart';
import '../services/api_service.dart';
import '../services/theme_service.dart';
import '../widgets/pgm_map_editor.dart';
import '../services/web_socket_service.dart';
import '../widgets/ros2_saved_maps_screen.dart';
import '../widgets/modern_ui_components.dart';
import '../models/odom.dart' as odom;

enum DeviceType { phone, tablet, desktop }

enum AMRMode { defaultMode, autonomous, mapping }

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

  // State variables
  MapData? _currentMap;
  List<Map<String, dynamic>> _connectedDevices = <Map<String, dynamic>>[];
  String? _selectedDeviceId;
  bool _isLoading = false;
  bool _hasUnsavedChanges = false;
  bool _isWebSocketConnected = false;

  // Enhanced debugging and error handling
  String? _error;
  String? _loadSource;
  bool _showDebugInfo = false;

  // UI state for responsive design
  DeviceType _deviceType = DeviceType.phone;
  bool _showSidebar = true;
  bool _showPropertiesPanel = false;

  // Map editor key for proper rebuilding
  final GlobalKey _mapEditorKey = GlobalKey();

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
          _showSidebar = false; // Hide sidebar on tablet to give more map space
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
    _webSocketService.realTimeData.listen((data) {
      if (mounted && data['deviceId'] == _selectedDeviceId) {
        _handleRealTimeData(data);
      }
    });

    _webSocketService.mapEvents.listen((data) {
      if (mounted && data['deviceId'] == _selectedDeviceId) {
        _handleMapEvent(data);
      }
    });

    _webSocketService.deviceEvents.listen((event) {
      if (mounted) {
        _loadConnectedDevices();
      }
    });

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
              print(' Error parsing odometry data: $e');
            }
            break;
        }
      });
    }
  }

  void _handleMapEvent(Map<String, dynamic> data) {
    switch (data['type']) {
      case 'map_edited':
        print('️ Map edited by another client: ${data['editType']}');
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
      // print(' Error loading devices: $e');
      // if (e is ApiException) {
      //   if (e.isNetworkError) {
      //     _showErrorSnackBar(
      //         'Network error: Cannot reach server. Check IP address.');
      //   } else {
      //     _showErrorSnackBar('Failed to load devices: ${e.message}');
      //   }
      // } else {
      //   _showErrorSnackBar('Failed to load devices: $e');
      // }
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
    print(' Loading map for device: $deviceId');

    if (mounted) {
      setState(() {
        _isLoading = true;
        _error = null;
      });
    }
    _loadingAnimationController.repeat();

    try {
      final response = await _apiService.getMapData(deviceId);

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

            // Auto-center the map after loading
            WidgetsBinding.instance.addPostFrameCallback((_) {
              _centerMap();
            });
          }

          _showInfoSnackBar(
              'Map loaded successfully with ${mapData.shapes.length} locations from ${_loadSource}');
          print(
              ' Map loaded successfully for device: $deviceId from source: ${_loadSource}');
        } catch (parseError) {
          print(' Error parsing map data: $parseError');

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
      print(' Error loading map: $e');
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
      print(' Loading ROS2 saved map: $mapName');

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

        // Auto-center the map after loading
        WidgetsBinding.instance.addPostFrameCallback((_) {
          _centerMap();
        });

        _showInfoSnackBar('ROS2 saved map loaded: $mapName');
        print(' ROS2 saved map loaded successfully: $mapName');
      } else {
        setState(() {
          _error = response['error'] ?? 'Failed to load ROS2 saved map';
        });
        _showErrorSnackBar('Failed to load ROS2 map: ${response['error']}');
      }
    } catch (e) {
      print(' Error loading ROS2 saved map: $e');
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

  // Method to center the map properly
  void _centerMap() {
    if (_mapEditorKey.currentState != null) {
      final state = _mapEditorKey.currentState as dynamic;
      if (state != null && state.mounted) {
        try {
          state.centerMapView();
        } catch (e) {
          print('Error centering map: $e');
        }
      }
    }
  }

  // Method to fit map to screen
  void _fitMapToScreen() {
    if (_mapEditorKey.currentState != null) {
      final state = _mapEditorKey.currentState as dynamic;
      if (state != null && state.mounted) {
        try {
          state
              .centerMapView(); // Use centerMapView as fallback since _fitToScreen is private
        } catch (e) {
          print('Error fitting map to screen: $e');
        }
      }
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
      leading: _deviceType == DeviceType.phone
          ? null
          : ModernActionButton(
              label: '',
              icon: Icons.arrow_back,
              onPressed: () =>
                  Navigator.pushReplacementNamed(context, '/dashboard'),
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
            Expanded(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    'Map Editor',
                    style: theme.displayMedium.copyWith(
                      fontSize: _deviceType == DeviceType.phone ? 16 : 18,
                      color: Colors.white,
                    ),
                  ),
                  if (_currentMap != null && _selectedDeviceId != null)
                    AnimatedBuilder(
                      animation: _pulseController,
                      builder: (context, child) {
                        return Text(
                          _hasUnsavedChanges
                              ? '● ${_currentMap!.shapes.length} locations (unsaved)'
                              : '${_selectedDeviceId} - ${_currentMap!.shapes.length} locations',
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
                            fontSize: _deviceType == DeviceType.phone ? 11 : 12,
                          ),
                        );
                      },
                    ),
                ],
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
        // Map control buttons
        if (_currentMap != null) ...[
          Container(
            margin: const EdgeInsets.symmetric(horizontal: 2),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                  colors: [Colors.green.shade400, Colors.green.shade600]),
              borderRadius: BorderRadius.circular(20),
            ),
            child: IconButton(
              onPressed: _centerMap,
              icon: Icon(Icons.center_focus_strong,
                  color: Colors.white, size: 18),
              tooltip: 'Center Map',
            ),
          ),
          Container(
            margin: const EdgeInsets.symmetric(horizontal: 2),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                  colors: [Colors.blue.shade400, Colors.blue.shade600]),
              borderRadius: BorderRadius.circular(20),
            ),
            child: IconButton(
              onPressed: _fitMapToScreen,
              icon: Icon(Icons.fit_screen, color: Colors.white, size: 18),
              tooltip: 'Fit to Screen',
            ),
          ),
        ],

        // Data source indicator
        if (_loadSource != null && _deviceType != DeviceType.phone)
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
                fontSize: 10,
              ),
            ),
          ),

        // Enhanced status indicators
        if (_deviceType != DeviceType.phone) ...[
          _buildStatusIndicator('API', true, theme.onlineColor, theme),
          const SizedBox(width: 8),
          _buildStatusIndicator(
              'WS',
              _isWebSocketConnected,
              _isWebSocketConnected ? theme.onlineColor : theme.offlineColor,
              theme),
          const SizedBox(width: 16),
        ],

        // Device selector for larger screens
        if (_deviceType == DeviceType.desktop) _buildDeviceSelector(theme),

        // Action buttons
        _buildActionButtons(theme),
      ],
      bottom: _deviceType == DeviceType.desktop
          ? TabBar(
              controller: _tabController,
              indicatorColor: theme.accentColor,
              labelColor: theme.accentColor,
              unselectedLabelColor: theme.isDarkMode
                  ? Colors.white.withOpacity(0.6)
                  : Colors.black.withOpacity(0.6),
              indicatorWeight: 3,
              tabs: [
                Tab(icon: Icon(Icons.brush, size: 20), text: 'PGM Editor'),
                Tab(icon: Icon(Icons.list, size: 20), text: 'Locations'),
                Tab(icon: Icon(Icons.analytics, size: 20), text: 'Analysis'),
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
        if (_deviceType != DeviceType.phone) ...[
          Container(
            margin: const EdgeInsets.symmetric(horizontal: 2),
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
              icon: Icon(Icons.save, color: Colors.white, size: 18),
              onPressed: _hasUnsavedChanges ? () => _saveMap() : null,
              tooltip: 'Save Map',
            ),
          ),
          Container(
            margin: const EdgeInsets.symmetric(horizontal: 2),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                  colors: [Colors.blue.shade400, Colors.blue.shade600]),
              borderRadius: BorderRadius.circular(20),
              boxShadow: theme.elevationSmall,
            ),
            child: IconButton(
              icon: Icon(Icons.refresh, color: Colors.white, size: 18),
              onPressed: _selectedDeviceId != null
                  ? () => _loadMapWithConfirmation(_selectedDeviceId!)
                  : null,
              tooltip: 'Refresh Map',
            ),
          ),
        ],
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
        icon: Icon(Icons.more_vert, color: Colors.white, size: 18),
        color: theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
        itemBuilder: (context) => [
          PopupMenuItem(
            child: ListTile(
              leading: Icon(Icons.folder, color: theme.accentColor, size: 18),
              title: Text('Saved Maps',
                  style: theme.bodyMedium.copyWith(fontSize: 12)),
              dense: true,
            ),
            onTap: () => _showSavedMaps(),
          ),
          PopupMenuItem(
            child: ListTile(
              leading:
                  Icon(Icons.rocket_launch, color: theme.accentColor, size: 18),
              title: Text('ROS2 Saved Maps',
                  style: theme.bodyMedium.copyWith(fontSize: 12)),
              dense: true,
            ),
            onTap: () => _showROS2SavedMaps(),
          ),
          if (_currentMap != null) ...[
            PopupMenuItem(
              child: ListTile(
                leading: Icon(Icons.center_focus_strong,
                    color: theme.successColor, size: 18),
                title: Text('Center Map',
                    style: theme.bodyMedium.copyWith(fontSize: 12)),
                dense: true,
              ),
              onTap: () {
                Navigator.of(context).pop();
                _centerMap();
              },
            ),
            PopupMenuItem(
              child: ListTile(
                leading:
                    Icon(Icons.fit_screen, color: theme.infoColor, size: 18),
                title: Text('Fit to Screen',
                    style: theme.bodyMedium.copyWith(fontSize: 12)),
                dense: true,
              ),
              onTap: () {
                Navigator.of(context).pop();
                _fitMapToScreen();
              },
            ),
          ],
          PopupMenuItem(
            child: ListTile(
              leading: Icon(Icons.clear_all, color: theme.errorColor, size: 18),
              title: Text('Clear All',
                  style: theme.bodyMedium.copyWith(fontSize: 12)),
              dense: true,
            ),
            onTap: _clearAllData,
          ),
          PopupMenuItem(
            child: ListTile(
              leading: Icon(Icons.send, color: theme.successColor, size: 18),
              title: Text('Send to AMR',
                  style: theme.bodyMedium.copyWith(fontSize: 12)),
              dense: true,
            ),
            onTap: _sendMapToAMR,
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
          child: LayoutBuilder(
            builder: (context, constraints) {
              // Update device type based on actual constraints
              WidgetsBinding.instance.addPostFrameCallback((_) {
                final newDeviceType = constraints.maxWidth > 1200
                    ? DeviceType.desktop
                    : constraints.maxWidth > 768
                        ? DeviceType.tablet
                        : DeviceType.phone;

                if (newDeviceType != _deviceType) {
                  setState(() {
                    _deviceType = newDeviceType;
                    _showSidebar = newDeviceType == DeviceType.desktop;
                  });
                }
              });

              return _buildMainResponsiveContent(theme);
            },
          ),
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
            width: 280,
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
          child: TabBarView(
            controller: _tabController,
            children: [
              _buildPGMEditor(theme),
              _buildLocationsView(theme),
              _buildAnalysisView(theme),
            ],
          ),
        ),

        // Properties panel
        if (_showPropertiesPanel)
          Container(
            width: 260,
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
                left: BorderSide(color: theme.accentColor.withOpacity(0.3)),
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
        // Tab selector for tablet
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
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              _buildTabButton('Editor', Icons.brush, 0, theme),
              _buildTabButton('Locations', Icons.list, 1, theme),
              _buildTabButton('Analysis', Icons.analytics, 2, theme),
            ],
          ),
        ),

        // Status bar
        if (_selectedDeviceId != null) _buildStatusBar(theme),

        // Content
        Expanded(
          child: IndexedStack(
            index: _tabController.index,
            children: [
              _buildPGMEditor(theme),
              _buildLocationsView(theme),
              _buildAnalysisView(theme),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildTabButton(
      String label, IconData icon, int index, ThemeService theme) {
    final isSelected = _tabController.index == index;
    return Expanded(
      child: GestureDetector(
        onTap: () => _tabController.animateTo(index),
        child: Container(
          margin: const EdgeInsets.symmetric(horizontal: 4),
          padding: const EdgeInsets.symmetric(vertical: 8),
          decoration: BoxDecoration(
            gradient: isSelected ? theme.primaryGradient : null,
            color: isSelected ? null : Colors.transparent,
            borderRadius: theme.borderRadiusSmall,
          ),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Icon(
                icon,
                size: 16,
                color: isSelected ? Colors.white : theme.accentColor,
              ),
              const SizedBox(width: 4),
              Text(
                label,
                style: theme.bodySmall.copyWith(
                  color: isSelected ? Colors.white : theme.accentColor,
                  fontWeight: isSelected ? FontWeight.bold : FontWeight.normal,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildPhoneLayout(ThemeService theme) {
    return Column(
      children: [
        // Compact device selector for mobile
        if (_connectedDevices.isNotEmpty) _buildCompactDeviceSelector(theme),

        // Compact status bar
        if (_selectedDeviceId != null) _buildCompactStatusBar(theme),

        // Main content
        Expanded(
          child: _selectedDeviceId == null
              ? _buildSelectDeviceView(theme)
              : _isLoading
                  ? ModernLoadingIndicator(
                      message: 'Loading Map Data...', size: 60)
                  : _buildCompactMapEditor(theme),
        ),

        // Bottom navigation for phone
        if (_selectedDeviceId != null && _currentMap != null)
          _buildPhoneBottomNavigation(theme),
      ],
    );
  }

  Widget _buildCompactDeviceSelector(ThemeService theme) {
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
          Icon(Icons.device_hub, color: theme.accentColor, size: 16),
          const SizedBox(width: 8),
          Text('Device: ', style: theme.bodyMedium.copyWith(fontSize: 12)),
          Expanded(
            child: DropdownButton<String>(
              value: _selectedDeviceId,
              isDense: true,
              isExpanded: true,
              underline: Container(),
              style: theme.bodyMedium.copyWith(fontSize: 12),
              items: _connectedDevices.map((device) {
                final deviceId = device['id']?.toString() ?? '';
                final deviceName = device['name']?.toString() ?? deviceId;
                return DropdownMenuItem<String>(
                  value: deviceId,
                  child: Text(deviceName, style: TextStyle(fontSize: 12)),
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
          ),
        ],
      ),
    );
  }

  Widget _buildCompactStatusBar(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
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
            padding: const EdgeInsets.all(2),
            decoration: BoxDecoration(
              gradient: theme.primaryGradient,
              shape: BoxShape.circle,
            ),
            child: Icon(Icons.map, size: 12, color: Colors.white),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              '${_currentMap?.shapes.length ?? 0} locations',
              style: theme.bodySmall.copyWith(fontSize: 11),
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
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Text(
                    'UNSAVED',
                    style: theme.bodySmall.copyWith(
                      color: Colors.white,
                      fontWeight: FontWeight.bold,
                      fontSize: 9,
                    ),
                  ),
                );
              },
            ),
          const SizedBox(width: 8),
          Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              _buildMiniStatusIndicator('API', true, theme.onlineColor, theme),
              const SizedBox(width: 4),
              _buildMiniStatusIndicator(
                  'WS',
                  _isWebSocketConnected,
                  _isWebSocketConnected
                      ? theme.onlineColor
                      : theme.offlineColor,
                  theme),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildMiniStatusIndicator(
      String label, bool connected, Color color, ThemeService theme) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 4, vertical: 2),
      decoration: BoxDecoration(
        color: color,
        borderRadius: BorderRadius.circular(6),
      ),
      child: Text(
        label,
        style: TextStyle(
          color: Colors.white,
          fontSize: 8,
          fontWeight: FontWeight.bold,
        ),
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
              Icon(Icons.map_outlined, size: 48, color: theme.errorColor),
              const SizedBox(height: 12),
              Text('No map data available', style: theme.headlineMedium),
              Text('Load a map to start editing', style: theme.bodyMedium),
            ],
          ),
        ),
      );
    }

    return Container(
      decoration: BoxDecoration(gradient: theme.backgroundGradient),
      child: EnhancedPGMMapEditor(
        key: _mapEditorKey,
        mapData: _currentMap,
        onMapChanged: _onMapChanged,
        deviceId: _selectedDeviceId,
      ),
    );
  }

  Widget _buildPhoneBottomNavigation(ThemeService theme) {
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
          _buildCompactControlButton(Icons.center_focus_strong, 'Center',
              theme.successColor, _centerMap, theme),
          _buildCompactControlButton(
              Icons.fit_screen, 'Fit', theme.infoColor, _fitMapToScreen, theme),
          _buildCompactControlButton(
              Icons.save,
              'Save',
              _hasUnsavedChanges ? theme.warningColor : Colors.grey,
              _hasUnsavedChanges ? _saveMap : null,
              theme),
          _buildCompactControlButton(
              Icons.refresh,
              'Reload',
              theme.accentColor,
              _selectedDeviceId != null
                  ? () => _loadMapWithConfirmation(_selectedDeviceId!)
                  : null,
              theme),
          _buildCompactControlButton(Icons.more_vert, 'More', theme.accentColor,
              () => _showPhoneOptionsMenu(theme), theme),
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
        borderRadius: BorderRadius.circular(8),
      ),
      child: IconButton(
        icon: Icon(icon, color: Colors.white, size: 18),
        onPressed: onPressed,
        tooltip: tooltip,
      ),
    );
  }

  void _showPhoneOptionsMenu(ThemeService theme) {
    showModalBottomSheet(
      context: context,
      backgroundColor: Colors.transparent,
      builder: (context) => Container(
        decoration: BoxDecoration(
          gradient: theme.backgroundGradient,
          borderRadius: BorderRadius.only(
            topLeft: Radius.circular(16),
            topRight: Radius.circular(16),
          ),
        ),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Container(
              width: 40,
              height: 4,
              margin: const EdgeInsets.symmetric(vertical: 8),
              decoration: BoxDecoration(
                color: Colors.grey,
                borderRadius: BorderRadius.circular(2),
              ),
            ),
            ListTile(
              leading: Icon(Icons.folder, color: theme.accentColor),
              title: Text('Saved Maps'),
              onTap: () {
                Navigator.pop(context);
                _showSavedMaps();
              },
            ),
            ListTile(
              leading: Icon(Icons.rocket_launch, color: theme.accentColor),
              title: Text('ROS2 Saved Maps'),
              onTap: () {
                Navigator.pop(context);
                _showROS2SavedMaps();
              },
            ),
            ListTile(
              leading: Icon(Icons.send, color: theme.successColor),
              title: Text('Send to AMR'),
              onTap: () {
                Navigator.pop(context);
                _sendMapToAMR();
              },
            ),
            ListTile(
              leading: Icon(Icons.clear_all, color: theme.errorColor),
              title: Text('Clear All'),
              onTap: () {
                Navigator.pop(context);
                _clearAllData();
              },
            ),
            const SizedBox(height: 16),
          ],
        ),
      ),
    );
  }

  Widget _buildSelectDeviceView(ThemeService theme) {
    return Center(
      child: Container(
        constraints: BoxConstraints(
          maxWidth: _deviceType == DeviceType.phone ? double.infinity : 400,
        ),
        padding: EdgeInsets.all(_deviceType == DeviceType.phone ? 16 : 24),
        child: ModernGlassCard(
          showGlow: true,
          glowColor: theme.accentColor,
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Container(
                width: _deviceType == DeviceType.phone ? 80 : 100,
                height: _deviceType == DeviceType.phone ? 80 : 100,
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
                child: Icon(Icons.map,
                    size: _deviceType == DeviceType.phone ? 40 : 50,
                    color: theme.accentColor),
              ),
              SizedBox(height: _deviceType == DeviceType.phone ? 16 : 24),
              Text(
                'Select a Device',
                style: theme.displayLarge.copyWith(
                    fontSize: _deviceType == DeviceType.phone ? 24 : 28),
              ),
              const SizedBox(height: 8),
              Text(
                'Choose a connected AMR device to edit its map',
                style: theme.bodyLarge.copyWith(
                    fontSize: _deviceType == DeviceType.phone ? 14 : 16),
                textAlign: TextAlign.center,
              ),
              SizedBox(height: _deviceType == DeviceType.phone ? 24 : 32),
              if (!_isWebSocketConnected) ...[
                Container(
                  padding: const EdgeInsets.all(12),
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
                      Icon(Icons.warning,
                          color: Colors.orange.shade700, size: 16),
                      const SizedBox(width: 8),
                      Expanded(
                        child: Text(
                          'WebSocket disconnected. Real-time features unavailable.',
                          style: theme.bodySmall
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
                const SizedBox(height: 16),
              ],
              if (_connectedDevices.isEmpty) ...[
                Text('No connected devices found', style: theme.bodyMedium),
                const SizedBox(height: 12),
                ModernActionButton(
                  label: 'Refresh Device List',
                  icon: Icons.refresh,
                  onPressed: _loadConnectedDevices,
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
                  width: _deviceType == DeviceType.phone ? 40 : 50,
                  height: _deviceType == DeviceType.phone ? 40 : 50,
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: deviceStatus == 'connected'
                          ? [
                              theme.onlineColor,
                              theme.onlineColor.withOpacity(0.8)
                            ]
                          : [Colors.grey.shade400, Colors.grey.shade600],
                    ),
                    borderRadius: BorderRadius.circular(
                        _deviceType == DeviceType.phone ? 20 : 25),
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
                    size: _deviceType == DeviceType.phone ? 20 : 24,
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        deviceName,
                        style: theme.headlineMedium.copyWith(
                            fontSize:
                                _deviceType == DeviceType.phone ? 14 : 16),
                      ),
                      Text(
                        'ID: $deviceId',
                        style: theme.bodySmall.copyWith(
                            fontSize:
                                _deviceType == DeviceType.phone ? 11 : 12),
                      ),
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

    return Container(
      decoration: BoxDecoration(gradient: theme.backgroundGradient),
      child: EnhancedPGMMapEditor(
        key: _mapEditorKey,
        mapData: _currentMap,
        onMapChanged: _onMapChanged,
        deviceId: _selectedDeviceId,
      ),
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
                label: 'Center Map',
                icon: Icons.center_focus_strong,
                onPressed: _currentMap != null ? _centerMap : () {},
                backgroundColor: theme.successColor,
              ),
              const SizedBox(height: 8),
              ModernActionButton(
                label: 'Fit to Screen',
                icon: Icons.fit_screen,
                onPressed: _currentMap != null ? _fitMapToScreen : () {},
                backgroundColor: theme.infoColor,
              ),
              const SizedBox(height: 8),
              ModernActionButton(
                label: 'Save Map',
                icon: Icons.save,
                onPressed: _hasUnsavedChanges ? _saveMap : () {},
                backgroundColor: _hasUnsavedChanges ? theme.warningColor : null,
                isLoading: false,
              ),
              const SizedBox(height: 8),
              ModernActionButton(
                label: 'Send to AMR',
                icon: Icons.send,
                onPressed: _currentMap != null ? _sendMapToAMR : () {},
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
          Text(_selectedDeviceId ?? 'None selected', style: theme.bodyMedium),
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
                  Text('AMR Navigation System',
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
      print(' Map saved successfully for device: $_selectedDeviceId');
    } catch (e) {
      Navigator.of(context).pop(); // Close loading dialog
      print(' Error saving map: $e');

      if (e is ApiException) {
        _showErrorSnackBar('Failed to save map: ${e.message}');
      } else {
        _showErrorSnackBar('Failed to save map: $e');
      }
    }
  }

  void _sendMapToAMR() async {
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

      _showInfoSnackBar('Map sent to AMR successfully');
    } catch (e) {
      _showErrorSnackBar('Failed to send map to AMR: $e');
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

/// AMR operation modes for compact mode selector
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

      print(' Loaded ${maps.length} saved maps');
    } catch (e) {
      setState(() {
        _error = 'Failed to load saved maps: $e';
        _isLoading = false;
      });
      print(' Error loading saved maps: $e');
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
