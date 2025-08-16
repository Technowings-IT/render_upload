// screens/connect_screen.dart - Responsive Robotic Fleet Management Connection Interface
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:flutter/services.dart';
import 'dart:async';
import 'dart:io';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:math' as math;
import '../services/theme_service.dart';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import '../services/network_discovery_service.dart';
import '../widgets/modern_ui_components.dart';

class ConnectScreen extends StatefulWidget {
  @override
  _ConnectScreenState createState() => _ConnectScreenState();
}

class _ConnectScreenState extends State<ConnectScreen>
    with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();
  final NetworkDiscoveryService _discoveryService = NetworkDiscoveryService();

  // Controllers
  final _deviceIdController = TextEditingController(text: 'AMR_01');
  final _deviceNameController = TextEditingController(text: 'Primary AMR');
  final _manualBackendIpController = TextEditingController();
  final _manualBackendPortController = TextEditingController(text: '3000');
  final _manualDeviceIdController = TextEditingController();
  final _manualDeviceNameController = TextEditingController();
  final _manualDeviceIpController = TextEditingController();
  final _manualDevicePortController = TextEditingController(text: '3000');

  // Animation controllers
  late AnimationController _scanningController;
  late AnimationController _connectionController;
  late AnimationController _radarController;
  late AnimationController _discoveryController;
  late Animation<double> _scanningAnimation;
  late Animation<double> _connectionPulse;
  late Animation<double> _radarSweep;
  late Animation<double> _discoveryFade;

  // State
  List<Map<String, dynamic>> _connectedDevices = [];
  List<AMRDevice> _discoveredDevices = [];
  String? _detectedBackendIP;
  bool _isLoading = false;
  bool _isConnecting = false;
  bool _isDiscovering = false;
  bool _isWebSocketConnected = false;
  String _connectionStatus = 'Initializing...';
  int _currentTabIndex = 0;

  // Device deletion and retry tracking
  Set<String> _removingDevices = {};
  Set<String> _retryingDevices = {};
  Set<String> _connectingDevices = {};

  // Stream subscriptions
  late StreamSubscription _deviceEventsSubscription;
  late StreamSubscription _connectionStateSubscription;

  // Responsive breakpoints
  static const double mobileBreakpoint = 600;
  static const double tabletBreakpoint = 900;
  static const double desktopBreakpoint = 1200;

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _initializeConnections();
    _loadSavedConnections();
    _startInitialDiscovery();
  }

  // Helper methods for responsive design
  bool get isMobile => MediaQuery.of(context).size.width < mobileBreakpoint;
  bool get isTablet =>
      MediaQuery.of(context).size.width >= mobileBreakpoint &&
      MediaQuery.of(context).size.width < desktopBreakpoint;
  bool get isDesktop => MediaQuery.of(context).size.width >= desktopBreakpoint;

  double get screenWidth => MediaQuery.of(context).size.width;

  // Responsive values
  double get horizontalPadding {
    if (isMobile) return 16;
    if (isTablet) return 32;
    return 48;
  }

  double get verticalSpacing {
    if (isMobile) return 16;
    if (isTablet) return 20;
    return 24;
  }

  int get deviceGridCrossAxisCount {
    if (isMobile) return 1;
    if (isTablet) return 2;
    return 3;
  }

  EdgeInsets get responsivePadding => EdgeInsets.symmetric(
        horizontal: horizontalPadding,
        vertical: verticalSpacing,
      );

  // Helper methods to determine actual device status based on backend connectivity
  String _getActualDeviceStatus(String deviceStatus) {
    if (!_isWebSocketConnected) {
      return 'disconnected';
    }
    return deviceStatus;
  }

  String _getDeviceStatusLabel(String deviceStatus) {
    if (!_isWebSocketConnected) {
      return 'BACKEND DISCONNECTED';
    }

    switch (deviceStatus) {
      case 'connecting':
        return 'CONNECTING';
      case 'connected':
        return 'CONNECTED';
      case 'disconnected':
        return 'DISCONNECTED';
      case 'error':
        return 'ERROR';
      default:
        return deviceStatus.toUpperCase();
    }
  }

  @override
  void dispose() {
    // Stop all animations before disposing
    if (_scanningController.isAnimating) {
      _scanningController.stop();
    }
    if (_connectionController.isAnimating) {
      _connectionController.stop();
    }
    if (_radarController.isAnimating) {
      _radarController.stop();
    }
    if (_discoveryController.isAnimating) {
      _discoveryController.stop();
    }

    // Cancel stream subscriptions safely
    _deviceEventsSubscription.cancel();
    _connectionStateSubscription.cancel();

    // Dispose controllers and animations
    _disposeControllers();
    _disposeAnimations();

    super.dispose();
  }

  void _initializeAnimations() {
    _scanningController = AnimationController(
      duration: const Duration(seconds: 3),
      vsync: this,
    );
    _connectionController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );
    _radarController = AnimationController(
      duration: const Duration(seconds: 4),
      vsync: this,
    );
    _discoveryController = AnimationController(
      duration: const Duration(milliseconds: 2000),
      vsync: this,
    );

    _scanningAnimation = Tween<double>(begin: 0.0, end: 1.0).animate(
      CurvedAnimation(parent: _scanningController, curve: Curves.linear),
    );
    _connectionPulse = Tween<double>(begin: 0.8, end: 1.2).animate(
      CurvedAnimation(parent: _connectionController, curve: Curves.easeInOut),
    );
    _radarSweep = Tween<double>(begin: 0.0, end: 2 * math.pi).animate(
      CurvedAnimation(parent: _radarController, curve: Curves.linear),
    );
    _discoveryFade = Tween<double>(begin: 0.0, end: 1.0).animate(
      CurvedAnimation(parent: _discoveryController, curve: Curves.easeInOut),
    );

    _discoveryController.forward();
  }

  void _disposeControllers() {
    _deviceIdController.dispose();
    _deviceNameController.dispose();
    _manualBackendIpController.dispose();
    _manualBackendPortController.dispose();
    _manualDeviceIdController.dispose();
    _manualDeviceNameController.dispose();
    _manualDeviceIpController.dispose();
    _manualDevicePortController.dispose();
  }

  void _disposeAnimations() {
    _scanningController.dispose();
    _connectionController.dispose();
    _radarController.dispose();
    _discoveryController.dispose();
  }

  void _initializeConnections() {
    _connectionStateSubscription =
        _webSocketService.connectionState.listen((connected) {
      setState(() {
        _isWebSocketConnected = connected;
        _connectionStatus =
            connected ? 'Fleet Backend Connected' : 'Backend Disconnected';
      });

      if (connected) {
        _connectionController.repeat(reverse: true);
        _loadConnectedDevices();
      } else {
        _connectionController.stop();
        _connectionController.reset();
      }
    });

    _deviceEventsSubscription = _webSocketService.deviceEvents.listen((event) {
      switch (event['type']) {
        case 'device_connected':
        case 'device_disconnected':
        case 'initial_data':
          _loadConnectedDevices();
          break;
      }
    });
  }

  void _loadSavedConnections() {
    setState(() {
      _manualBackendIpController.text = '192.168.128.79';
      _manualDeviceIpController.text = '192.168.128.240';
    });
  }

  void _startInitialDiscovery() async {
    if (!mounted) return;

    setState(() {
      _connectionStatus = 'Scanning network for AMR systems...';
    });

    _radarController.repeat();

    try {
      final backendIP = await _detectBackendIP();

      if (!mounted) return;

      if (backendIP != null) {
        setState(() {
          _detectedBackendIP = backendIP;
          _connectionStatus = 'AMR Backend detected at $backendIP';
        });
        await _connectToDetectedBackend(backendIP);
      } else {
        if (mounted) {
          setState(() {
            _connectionStatus =
                'No AMR backend found - manual connection available';
          });
        }
      }

      if (mounted) {
        await _discoverAMRDevices();
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _connectionStatus = 'Discovery failed: ${e.toString()}';
        });
      }
      print('❌ Device discovery failed: $e');
    } finally {
      if (mounted && _radarController.isAnimating) {
        _radarController.stop();
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    return Scaffold(
      body: Container(
        decoration: BoxDecoration(gradient: theme.backgroundGradient),
        child: FadeTransition(
          opacity: _discoveryFade,
          child: isDesktop
              ? _buildDesktopLayout(theme)
              : _buildMobileLayout(theme),
        ),
      ),
      floatingActionButton: _buildFloatingActionButton(theme),
      floatingActionButtonLocation: FloatingActionButtonLocation.endFloat,
    );
  }

  Widget _buildMobileLayout(ThemeService theme) {
    return CustomScrollView(
      slivers: [
        _buildModernAppBar(theme),
        SliverPadding(
          padding: responsivePadding,
          sliver: SliverList(
            delegate: SliverChildListDelegate([
              _buildConnectionStatusPanel(theme),
              SizedBox(height: verticalSpacing),
              _buildConnectionTabs(theme),
              SizedBox(height: verticalSpacing),
              _buildTabContent(theme),
              SizedBox(height: verticalSpacing),
              _buildConnectedDevicesPanel(theme),
            ]),
          ),
        ),
      ],
    );
  }

  Widget _buildDesktopLayout(ThemeService theme) {
    return Row(
      children: [
        // Left sidebar for navigation and status
        Container(
          width: 350,
          decoration: BoxDecoration(
            gradient: LinearGradient(
              begin: Alignment.topCenter,
              end: Alignment.bottomCenter,
              colors: [
                theme.accentColor.withOpacity(0.1),
                Colors.transparent,
              ],
            ),
          ),
          child: Column(
            children: [
              _buildDesktopHeader(theme),
              Expanded(
                child: SingleChildScrollView(
                  padding: const EdgeInsets.all(24),
                  child: Column(
                    children: [
                      _buildConnectionStatusPanel(theme),
                      SizedBox(height: verticalSpacing),
                      _buildVerticalConnectionTabs(theme),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),

        // Vertical divider
        Container(
          width: 1,
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [
                Colors.transparent,
                theme.accentColor.withOpacity(0.3),
                Colors.transparent,
              ],
            ),
          ),
        ),

        // Main content area
        Expanded(
          child: Column(
            children: [
              _buildDesktopMainHeader(theme),
              Expanded(
                child: SingleChildScrollView(
                  padding: const EdgeInsets.all(32),
                  child: Column(
                    children: [
                      _buildTabContent(theme),
                      SizedBox(height: verticalSpacing * 2),
                      _buildConnectedDevicesPanel(theme),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildDesktopHeader(ThemeService theme) {
    return Container(
      height: 120,
      padding: const EdgeInsets.all(24),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            theme.accentColor.withOpacity(0.2),
            theme.accentColor.withOpacity(0.05),
          ],
        ),
      ),
      child: Row(
        children: [
          Container(
            padding: const EdgeInsets.all(16),
            decoration: BoxDecoration(
              gradient: theme.primaryGradient,
              borderRadius: theme.borderRadiusMedium,
              boxShadow: theme.elevationMedium,
            ),
            child: Icon(Icons.router, color: Colors.white, size: 32),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Text(
                  'Fleet Connection',
                  style: theme.headlineLarge.copyWith(
                    color: theme.accentColor,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                Text(
                  'Manage AMR Network',
                  style: theme.bodyMedium.copyWith(
                    color: theme.accentColor.withOpacity(0.7),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDesktopMainHeader(ThemeService theme) {
    return Container(
      height: 80,
      padding: const EdgeInsets.symmetric(horizontal: 32, vertical: 16),
      decoration: BoxDecoration(
        color: theme.isDarkMode
            ? Colors.white.withOpacity(0.05)
            : Colors.black.withOpacity(0.03),
        border: Border(
          bottom: BorderSide(
            color: theme.accentColor.withOpacity(0.1),
          ),
        ),
      ),
      child: Row(
        children: [
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Text(
                  'Fleet Management Dashboard',
                  style: theme.headlineMedium
                      .copyWith(fontWeight: FontWeight.w600),
                ),
                Text(
                  'Connect and manage your AMR devices',
                  style: theme.bodyMedium.copyWith(
                    color: theme.accentColor.withOpacity(0.7),
                  ),
                ),
              ],
            ),
          ),
          Row(
            children: [
              _buildDesktopActionButton(
                'Scan Network',
                Icons.radar,
                _isLoading ? null : () => _refreshConnections(),
                theme,
                isLoading: _isLoading,
              ),
              const SizedBox(width: 16),
              _buildDesktopActionButton(
                'Dashboard',
                Icons.dashboard,
                () => _goToDashboard(),
                theme,
                isPrimary: true,
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildDesktopActionButton(
    String label,
    IconData icon,
    VoidCallback? onPressed,
    ThemeService theme, {
    bool isPrimary = false,
    bool isLoading = false,
  }) {
    return Container(
      decoration: BoxDecoration(
        gradient: isPrimary ? theme.primaryGradient : null,
        color: isPrimary ? null : theme.accentColor.withOpacity(0.1),
        borderRadius: theme.borderRadiusMedium,
        border: isPrimary
            ? null
            : Border.all(
                color: theme.accentColor.withOpacity(0.3),
              ),
      ),
      child: Material(
        color: Colors.transparent,
        child: InkWell(
          onTap: onPressed,
          borderRadius: theme.borderRadiusMedium,
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 12),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                isLoading
                    ? SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          valueColor: AlwaysStoppedAnimation<Color>(
                            isPrimary ? Colors.white : theme.accentColor,
                          ),
                        ),
                      )
                    : Icon(
                        icon,
                        color: isPrimary ? Colors.white : theme.accentColor,
                        size: 18,
                      ),
                const SizedBox(width: 8),
                Text(
                  label,
                  style: theme.bodyMedium.copyWith(
                    color: isPrimary ? Colors.white : theme.accentColor,
                    fontWeight: FontWeight.w600,
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildVerticalConnectionTabs(ThemeService theme) {
    final tabs = [
      {'title': 'Auto Discovery', 'icon': Icons.radar},
      {'title': 'Manual Backend', 'icon': Icons.settings_ethernet},
      {'title': 'Manual Device', 'icon': Icons.precision_manufacturing},
    ];

    return Column(
      children: tabs.asMap().entries.map((entry) {
        final index = entry.key;
        final tab = entry.value;
        final isSelected = _currentTabIndex == index;

        return Container(
          margin: const EdgeInsets.only(bottom: 8),
          child: ModernGlassCard(
            onTap: () {
              setState(() => _currentTabIndex = index);
              HapticFeedback.lightImpact();
            },
            showGlow: isSelected,
            glowColor: theme.accentColor,
            child: Row(
              children: [
                Container(
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    gradient: isSelected ? theme.primaryGradient : null,
                    color:
                        isSelected ? null : theme.accentColor.withOpacity(0.1),
                    borderRadius: theme.borderRadiusSmall,
                  ),
                  child: Icon(
                    tab['icon'] as IconData,
                    color: isSelected ? Colors.white : theme.accentColor,
                    size: 20,
                  ),
                ),
                const SizedBox(width: 16),
                Expanded(
                  child: Text(
                    tab['title'] as String,
                    style: theme.bodyMedium.copyWith(
                      color: isSelected ? theme.accentColor : null,
                      fontWeight:
                          isSelected ? FontWeight.w600 : FontWeight.w500,
                    ),
                  ),
                ),
                if (isSelected)
                  Icon(
                    Icons.arrow_forward_ios,
                    color: theme.accentColor,
                    size: 16,
                  ),
              ],
            ),
          ),
        );
      }).toList(),
    );
  }

  Widget _buildFloatingActionButton(ThemeService theme) {
    if (isDesktop) return Container(); // Don't show FAB on desktop

    return Container(
      decoration: BoxDecoration(
        gradient: theme.primaryGradient,
        borderRadius: BorderRadius.circular(30),
        boxShadow: theme.elevationMedium,
      ),
      child: FloatingActionButton.extended(
        onPressed: () => _goToDashboard(),
        backgroundColor: Colors.transparent,
        elevation: 0,
        icon: Icon(Icons.dashboard, color: Colors.white),
        label: Text(
          'Dashboard',
          style: theme.bodyMedium.copyWith(
            color: Colors.white,
            fontWeight: FontWeight.w600,
          ),
        ),
      ),
    );
  }

  Widget _buildModernAppBar(ThemeService theme) {
    return SliverAppBar(
      expandedHeight: isTablet ? 160 : 140,
      floating: false,
      pinned: true,
      backgroundColor: Colors.transparent,
      elevation: 0,
      flexibleSpace: FlexibleSpaceBar(
        title: ShaderMask(
          shaderCallback: (bounds) => LinearGradient(
            colors: [theme.accentColor, theme.accentColor.withOpacity(0.7)],
          ).createShader(bounds),
          child: Text(
            'Fleet Connection',
            style: theme.displayMedium.copyWith(
              fontSize: isTablet ? 28 : 24,
              color: Colors.white,
            ),
          ),
        ),
        background: Container(
          decoration: BoxDecoration(
            gradient: LinearGradient(
              begin: Alignment.topLeft,
              end: Alignment.bottomRight,
              colors: [
                theme.accentColor.withOpacity(0.3),
                theme.accentColor.withOpacity(0.1),
                Colors.transparent,
              ],
            ),
          ),
          child: Stack(
            children: [
              // Animated scanning lines
              AnimatedBuilder(
                animation: _scanningAnimation,
                builder: (context, child) {
                  return Positioned(
                    left: screenWidth * _scanningAnimation.value - 100,
                    top: 0,
                    bottom: 0,
                    child: Container(
                      width: 2,
                      decoration: BoxDecoration(
                        gradient: LinearGradient(
                          colors: [
                            Colors.transparent,
                            theme.accentColor.withOpacity(0.5),
                            Colors.transparent,
                          ],
                        ),
                      ),
                    ),
                  );
                },
              ),
              // Network particles
              ...List.generate(isTablet ? 12 : 8,
                  (index) => _buildNetworkParticle(index, theme)),
            ],
          ),
        ),
      ),
      actions: isMobile
          ? [
              IconButton(
                onPressed: _isLoading ? null : () => _refreshConnections(),
                icon: AnimatedBuilder(
                  animation: _scanningController,
                  builder: (context, child) {
                    return Transform.rotate(
                      angle: _scanningAnimation.value * 2 * math.pi,
                      child: Icon(
                        Icons.radar,
                        color: theme.accentColor,
                        size: 28,
                      ),
                    );
                  },
                ),
                tooltip: 'Scan Network',
              ),
              IconButton(
                onPressed: () => _startAutoDiscovery(),
                icon: Icon(Icons.search, color: theme.accentColor),
                tooltip: 'Discover Devices',
              ),
              IconButton(
                onPressed: () => _goToDashboard(),
                icon: Icon(Icons.dashboard, color: theme.accentColor),
                tooltip: 'Go to Dashboard',
              ),
              const SizedBox(width: 8),
            ]
          : [],
    );
  }

  Widget _buildNetworkParticle(int index, ThemeService theme) {
    final left = (index * 47.3) % screenWidth;

    return TweenAnimationBuilder<double>(
      duration: Duration(milliseconds: 3000 + (index * 200)),
      tween: Tween(begin: 0.0, end: 1.0),
      builder: (context, value, child) {
        return Positioned(
          left: left,
          top: 30 + (index * 12.0),
          child: Transform.translate(
            offset: Offset(0, -30 * value),
            child: Opacity(
              opacity: 1.0 - value,
              child: Container(
                width: 3 + (index % 2),
                height: 3 + (index % 2),
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: theme.accentColor.withOpacity(0.4),
                  boxShadow: [
                    BoxShadow(
                      color: theme.accentColor.withOpacity(0.3),
                      blurRadius: 4,
                      spreadRadius: 1,
                    ),
                  ],
                ),
              ),
            ),
          ),
        );
      },
    );
  }

  Widget _buildConnectionStatusPanel(ThemeService theme) {
    return ModernGlassCard(
      showGlow: _isWebSocketConnected,
      glowColor: _isWebSocketConnected ? theme.successColor : theme.errorColor,
      child: Column(
        children: [
          Row(
            children: [
              AnimatedBuilder(
                animation: _connectionPulse,
                builder: (context, child) {
                  return Transform.scale(
                    scale: _isWebSocketConnected ? _connectionPulse.value : 1.0,
                    child: Container(
                      padding: const EdgeInsets.all(16),
                      decoration: BoxDecoration(
                        gradient: RadialGradient(
                          colors: _isWebSocketConnected
                              ? [
                                  theme.successColor,
                                  theme.successColor.withOpacity(0.7)
                                ]
                              : [
                                  theme.errorColor,
                                  theme.errorColor.withOpacity(0.7)
                                ],
                        ),
                        borderRadius: theme.borderRadiusMedium,
                        boxShadow:
                            _isWebSocketConnected ? theme.neonGlow : null,
                      ),
                      child: Icon(
                        _isWebSocketConnected ? Icons.wifi : Icons.wifi_off,
                        color: Colors.white,
                        size: isTablet ? 36 : 32,
                      ),
                    ),
                  );
                },
              ),
              SizedBox(width: isTablet ? 24 : 20),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      _isWebSocketConnected ? 'CONNECTED' : 'DISCONNECTED',
                      style: theme.headlineLarge.copyWith(
                        color: _isWebSocketConnected
                            ? theme.successColor
                            : theme.errorColor,
                        fontWeight: FontWeight.bold,
                        letterSpacing: 1.2,
                        fontSize: isTablet ? 20 : 16,
                      ),
                    ),
                    SizedBox(height: isTablet ? 12 : 8),
                    Text(
                      _connectionStatus,
                      style: theme.bodyLarge.copyWith(
                        fontSize: isTablet ? 16 : 14,
                      ),
                    ),
                    // Show device connectivity notice
                    if (!_isWebSocketConnected &&
                        _connectedDevices.isNotEmpty) ...[
                      SizedBox(height: isTablet ? 12 : 8),
                      Container(
                        padding: EdgeInsets.symmetric(
                            horizontal: isTablet ? 16 : 12,
                            vertical: isTablet ? 10 : 8),
                        decoration: BoxDecoration(
                          color: theme.warningColor.withOpacity(0.1),
                          borderRadius: theme.borderRadiusSmall,
                          border: Border.all(
                              color: theme.warningColor.withOpacity(0.3)),
                        ),
                        child: Row(
                          children: [
                            Icon(Icons.warning,
                                color: theme.warningColor,
                                size: isTablet ? 18 : 16),
                            SizedBox(width: isTablet ? 10 : 8),
                            Expanded(
                              child: Text(
                                '${_connectedDevices.length} device(s) offline due to backend disconnection',
                                style: theme.bodySmall.copyWith(
                                  color: theme.warningColor,
                                  fontWeight: FontWeight.w500,
                                  fontSize: isTablet ? 14 : 12,
                                ),
                              ),
                            ),
                          ],
                        ),
                      ),
                    ],
                    if (_detectedBackendIP != null) ...[
                      SizedBox(height: isTablet ? 16 : 12),
                      Container(
                        padding: EdgeInsets.symmetric(
                            horizontal: isTablet ? 16 : 12,
                            vertical: isTablet ? 8 : 6),
                        decoration: BoxDecoration(
                          color: theme.accentColor.withOpacity(0.1),
                          borderRadius: theme.borderRadiusSmall,
                          border: Border.all(
                              color: theme.accentColor.withOpacity(0.3)),
                        ),
                        child: Text(
                          'Backend: $_detectedBackendIP:3000',
                          style: theme.monospace.copyWith(
                            color: theme.accentColor,
                            fontWeight: FontWeight.w600,
                            fontSize: isTablet ? 14 : 12,
                          ),
                        ),
                      ),
                    ],
                  ],
                ),
              ),
              // Dashboard quick access button
              if (!isDesktop)
                Column(
                  children: [
                    RoboticStatusIndicator(
                      status: _isWebSocketConnected ? 'online' : 'offline',
                      label: _isWebSocketConnected ? 'ONLINE' : 'OFFLINE',
                      animated: _isWebSocketConnected,
                    ),
                    SizedBox(height: isTablet ? 16 : 12),
                    ModernActionButton(
                      label: 'Dashboard',
                      icon: Icons.dashboard,
                      onPressed: () => _goToDashboard(),
                      isSecondary: true,
                    ),
                  ],
                ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildConnectionTabs(ThemeService theme) {
    if (isDesktop) return Container(); // Handled in sidebar for desktop

    return Container(
      decoration: BoxDecoration(
        color: theme.isDarkMode
            ? Colors.white.withOpacity(0.05)
            : Colors.black.withOpacity(0.05),
        borderRadius: theme.borderRadiusMedium,
        border: Border.all(
          color: theme.isDarkMode
              ? Colors.white.withOpacity(0.1)
              : Colors.black.withOpacity(0.1),
        ),
      ),
      child: isTablet ? _buildTabletTabs(theme) : _buildMobileTabs(theme),
    );
  }

  Widget _buildMobileTabs(ThemeService theme) {
    return Row(
      children: [
        _buildTabButton('Auto Discovery', 0, Icons.radar, theme),
        _buildTabButton('Manual Backend', 1, Icons.settings_ethernet, theme),
        _buildTabButton(
            'Manual Device', 2, Icons.precision_manufacturing, theme),
      ],
    );
  }

  Widget _buildTabletTabs(ThemeService theme) {
    final tabs = [
      {
        'title': 'Auto Discovery',
        'subtitle': 'Network scan',
        'icon': Icons.radar
      },
      {
        'title': 'Manual Backend',
        'subtitle': 'Direct connection',
        'icon': Icons.settings_ethernet
      },
      {
        'title': 'Manual Device',
        'subtitle': 'Add specific AMR',
        'icon': Icons.precision_manufacturing
      },
    ];

    return Row(
      children: tabs.asMap().entries.map((entry) {
        final index = entry.key;
        final tab = entry.value;
        final isSelected = _currentTabIndex == index;

        return Expanded(
          child: GestureDetector(
            onTap: () {
              setState(() => _currentTabIndex = index);
              HapticFeedback.lightImpact();
            },
            child: AnimatedContainer(
              duration: const Duration(milliseconds: 200),
              padding: const EdgeInsets.symmetric(vertical: 20, horizontal: 16),
              decoration: BoxDecoration(
                gradient: isSelected ? theme.primaryGradient : null,
                borderRadius: theme.borderRadiusSmall,
              ),
              child: Column(
                children: [
                  Icon(
                    tab['icon'] as IconData,
                    color: isSelected ? Colors.white : theme.accentColor,
                    size: 28,
                  ),
                  const SizedBox(height: 12),
                  Text(
                    tab['title'] as String,
                    style: theme.bodyMedium.copyWith(
                      color: isSelected ? Colors.white : theme.accentColor,
                      fontWeight: FontWeight.w600,
                    ),
                    textAlign: TextAlign.center,
                  ),
                  const SizedBox(height: 4),
                  Text(
                    tab['subtitle'] as String,
                    style: theme.bodySmall.copyWith(
                      color: isSelected
                          ? Colors.white.withOpacity(0.8)
                          : theme.accentColor.withOpacity(0.6),
                    ),
                    textAlign: TextAlign.center,
                  ),
                ],
              ),
            ),
          ),
        );
      }).toList(),
    );
  }

  Widget _buildTabButton(
      String title, int index, IconData icon, ThemeService theme) {
    final isSelected = _currentTabIndex == index;

    return Expanded(
      child: GestureDetector(
        onTap: () {
          setState(() => _currentTabIndex = index);
          HapticFeedback.lightImpact();
        },
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          padding: const EdgeInsets.symmetric(vertical: 16, horizontal: 12),
          decoration: BoxDecoration(
            gradient: isSelected ? theme.primaryGradient : null,
            borderRadius: theme.borderRadiusSmall,
          ),
          child: Column(
            children: [
              Icon(
                icon,
                color: isSelected ? Colors.white : theme.accentColor,
                size: 24,
              ),
              const SizedBox(height: 8),
              Text(
                title,
                style: theme.bodySmall.copyWith(
                  color: isSelected ? Colors.white : theme.accentColor,
                  fontWeight: FontWeight.w600,
                ),
                textAlign: TextAlign.center,
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildTabContent(ThemeService theme) {
    switch (_currentTabIndex) {
      case 0:
        return _buildAutoDiscoveryPanel(theme);
      case 1:
        return _buildManualBackendPanel(theme);
      case 2:
        return _buildManualDevicePanel(theme);
      default:
        return Container();
    }
  }

  Widget _buildAutoDiscoveryPanel(ThemeService theme) {
    return ModernGlassCard(
      child: Column(
        children: [
          Row(
            children: [
              Container(
                padding: EdgeInsets.all(isTablet ? 16 : 12),
                decoration: BoxDecoration(
                  color: theme.infoColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: AnimatedBuilder(
                  animation: _radarController,
                  builder: (context, child) {
                    return Transform.rotate(
                      angle: _radarSweep.value,
                      child: Icon(
                        Icons.radar,
                        color: theme.infoColor,
                        size: isTablet ? 32 : 28,
                      ),
                    );
                  },
                ),
              ),
              SizedBox(width: isTablet ? 20 : 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Network Discovery',
                      style: theme.headlineMedium.copyWith(
                        fontSize: isTablet ? 20 : 18,
                      ),
                    ),
                    Text(
                      'Automatically find AMR systems on your network',
                      style: theme.bodyMedium.copyWith(
                        fontSize: isTablet ? 16 : 14,
                      ),
                    ),
                  ],
                ),
              ),
            ],
          ),
          SizedBox(height: verticalSpacing),

          // Discovery results
          if (_discoveredDevices.isNotEmpty) ...[
            _buildDiscoveryResults(theme),
            SizedBox(height: verticalSpacing),
          ],

          // Action buttons
          isTablet || isDesktop
              ? _buildTabletActionButtons(theme)
              : _buildMobileActionButtons(theme),
        ],
      ),
    );
  }

  Widget _buildMobileActionButtons(ThemeService theme) {
    return Row(
      children: [
        Expanded(
          child: ModernActionButton(
            label: _isDiscovering ? 'Scanning...' : 'Scan Network',
            icon: Icons.search,
            onPressed: _isDiscovering ? () {} : () => _startAutoDiscovery(),
            isLoading: _isDiscovering,
            isSecondary: true,
          ),
        ),
        const SizedBox(width: 16),
        Expanded(
          child: ModernActionButton(
            label: 'Quick Connect',
            icon: Icons.flash_on,
            onPressed: () => _autoConnectAMR(),
          ),
        ),
      ],
    );
  }

  Widget _buildTabletActionButtons(ThemeService theme) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        SizedBox(
          width: 200,
          child: ModernActionButton(
            label: _isDiscovering ? 'Scanning...' : 'Scan Network',
            icon: Icons.search,
            onPressed: _isDiscovering ? () {} : () => _startAutoDiscovery(),
            isLoading: _isDiscovering,
            isSecondary: true,
          ),
        ),
        const SizedBox(width: 24),
        SizedBox(
          width: 200,
          child: ModernActionButton(
            label: 'Quick Connect',
            icon: Icons.flash_on,
            onPressed: () => _autoConnectAMR(),
          ),
        ),
      ],
    );
  }

  Widget _buildDiscoveryResults(ThemeService theme) {
    return Container(
      padding: EdgeInsets.all(isTablet ? 20 : 16),
      decoration: BoxDecoration(
        color: theme.successColor.withOpacity(0.1),
        borderRadius: theme.borderRadiusMedium,
        border: Border.all(color: theme.successColor.withOpacity(0.3)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(Icons.devices, color: theme.successColor),
              SizedBox(width: isTablet ? 12 : 8),
              Text(
                'Discovered Devices (${_discoveredDevices.length})',
                style: theme.bodyLarge.copyWith(
                  fontWeight: FontWeight.w600,
                  color: theme.successColor,
                  fontSize: isTablet ? 18 : 16,
                ),
              ),
            ],
          ),
          SizedBox(height: isTablet ? 16 : 12),
          isTablet || isDesktop
              ? _buildDiscoveredDevicesGrid(theme)
              : _buildDiscoveredDevicesList(theme),
        ],
      ),
    );
  }

  Widget _buildDiscoveredDevicesList(ThemeService theme) {
    return ListView.separated(
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      itemCount: _discoveredDevices.length,
      separatorBuilder: (context, index) => SizedBox(height: isTablet ? 12 : 8),
      itemBuilder: (context, index) {
        final device = _discoveredDevices[index];
        return _buildDiscoveredDeviceCard(device, theme);
      },
    );
  }

  Widget _buildDiscoveredDevicesGrid(ThemeService theme) {
    return GridView.builder(
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
        crossAxisCount: isDesktop ? 2 : 1,
        childAspectRatio: isDesktop ? 3.5 : 4.5,
        crossAxisSpacing: 16,
        mainAxisSpacing: 12,
      ),
      itemCount: _discoveredDevices.length,
      itemBuilder: (context, index) {
        final device = _discoveredDevices[index];
        return _buildDiscoveredDeviceCard(device, theme);
      },
    );
  }

  Widget _buildDiscoveredDeviceCard(AMRDevice device, ThemeService theme) {
    return Container(
      padding: EdgeInsets.all(isTablet ? 16 : 12),
      decoration: BoxDecoration(
        color: theme.isDarkMode
            ? Colors.white.withOpacity(0.05)
            : Colors.black.withOpacity(0.05),
        borderRadius: theme.borderRadiusSmall,
        border: Border.all(
          color: theme.isDarkMode
              ? Colors.white.withOpacity(0.1)
              : Colors.black.withOpacity(0.1),
        ),
      ),
      child: Row(
        children: [
          Container(
            padding: EdgeInsets.all(isTablet ? 10 : 8),
            decoration: BoxDecoration(
              color: theme.accentColor.withOpacity(0.1),
              borderRadius: theme.borderRadiusSmall,
            ),
            child: Icon(
              Icons.router,
              color: theme.accentColor,
              size: isTablet ? 24 : 20,
            ),
          ),
          SizedBox(width: isTablet ? 16 : 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  device.name,
                  style: theme.bodyLarge.copyWith(
                    fontWeight: FontWeight.w600,
                    fontSize: isTablet ? 16 : 14,
                  ),
                ),
                Text(
                  '${device.ipAddress}:${device.port}',
                  style: theme.monospace.copyWith(
                    fontSize: isTablet ? 14 : 12,
                  ),
                ),
                Text(
                  'Method: ${device.discoveryMethod}',
                  style: theme.bodySmall.copyWith(
                    fontSize: isTablet ? 13 : 11,
                  ),
                ),
              ],
            ),
          ),
          ModernActionButton(
            label: 'Connect',
            icon: Icons.link,
            onPressed: () => _connectToDetectedBackend(device.ipAddress),
            isSecondary: true,
          ),
        ],
      ),
    );
  }

  Widget _buildManualBackendPanel(ThemeService theme) {
    return ModernGlassCard(
      child: Column(
        children: [
          Row(
            children: [
              Container(
                padding: EdgeInsets.all(isTablet ? 16 : 12),
                decoration: BoxDecoration(
                  color: theme.warningColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(
                  Icons.settings_ethernet,
                  color: theme.warningColor,
                  size: isTablet ? 32 : 28,
                ),
              ),
              SizedBox(width: isTablet ? 20 : 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Manual Backend Connection',
                      style: theme.headlineMedium.copyWith(
                        fontSize: isTablet ? 20 : 18,
                      ),
                    ),
                    Text(
                      'Connect to a specific AMR Fleet Backend',
                      style: theme.bodyMedium.copyWith(
                        fontSize: isTablet ? 16 : 14,
                      ),
                    ),
                  ],
                ),
              ),
            ],
          ),
          SizedBox(height: verticalSpacing),

          // Input fields
          isTablet || isDesktop
              ? _buildTabletBackendInputs(theme)
              : _buildMobileBackendInputs(theme),

          SizedBox(height: verticalSpacing),

          // Connect button
          SizedBox(
            width: isTablet ? 300 : double.infinity,
            child: ModernActionButton(
              label: _isConnecting ? 'Connecting...' : 'Connect to Backend',
              icon: Icons.wifi,
              onPressed:
                  _isConnecting ? () {} : () => _connectManualWebSocket(),
              isLoading: _isConnecting,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMobileBackendInputs(ThemeService theme) {
    return Column(
      children: [
        _buildModernTextField(
          controller: _manualBackendIpController,
          label: 'Backend IP Address',
          hint: '192.168.128.79',
          icon: Icons.computer,
          theme: theme,
        ),
        const SizedBox(height: 16),
        _buildModernTextField(
          controller: _manualBackendPortController,
          label: 'Port',
          hint: '3000',
          icon: Icons.settings,
          theme: theme,
          keyboardType: TextInputType.number,
        ),
      ],
    );
  }

  Widget _buildTabletBackendInputs(ThemeService theme) {
    return Row(
      children: [
        Expanded(
          flex: 3,
          child: _buildModernTextField(
            controller: _manualBackendIpController,
            label: 'Backend IP Address',
            hint: '192.168.128.79',
            icon: Icons.computer,
            theme: theme,
          ),
        ),
        const SizedBox(width: 24),
        Expanded(
          child: _buildModernTextField(
            controller: _manualBackendPortController,
            label: 'Port',
            hint: '3000',
            icon: Icons.settings,
            theme: theme,
            keyboardType: TextInputType.number,
          ),
        ),
      ],
    );
  }

  Widget _buildManualDevicePanel(ThemeService theme) {
    return ModernGlassCard(
      child: Column(
        children: [
          Row(
            children: [
              Container(
                padding: EdgeInsets.all(isTablet ? 16 : 12),
                decoration: BoxDecoration(
                  color: theme.accentColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(
                  Icons.precision_manufacturing,
                  color: theme.accentColor,
                  size: isTablet ? 32 : 28,
                ),
              ),
              SizedBox(width: isTablet ? 20 : 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Manual AMR Device',
                      style: theme.headlineMedium.copyWith(
                        fontSize: isTablet ? 20 : 18,
                      ),
                    ),
                    Text(
                      'Add a specific AMR device to your fleet',
                      style: theme.bodyMedium.copyWith(
                        fontSize: isTablet ? 16 : 14,
                      ),
                    ),
                  ],
                ),
              ),
            ],
          ),
          SizedBox(height: verticalSpacing),

          // Device information
          isTablet || isDesktop
              ? _buildTabletDeviceInputs(theme)
              : _buildMobileDeviceInputs(theme),

          SizedBox(height: verticalSpacing),

          // Add device button
          SizedBox(
            width: isTablet ? 300 : double.infinity,
            child: ModernActionButton(
              label: _isConnecting ? 'Adding Device...' : 'Add AMR Device',
              icon: Icons.add_circle,
              onPressed: _isConnecting ? () {} : () => _connectManualAMR(),
              isLoading: _isConnecting,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMobileDeviceInputs(ThemeService theme) {
    return Column(
      children: [
        _buildModernTextField(
          controller: _manualDeviceIdController,
          label: 'Device ID',
          hint: 'AMR_01',
          icon: Icons.badge,
          theme: theme,
        ),
        const SizedBox(height: 16),
        _buildModernTextField(
          controller: _manualDeviceNameController,
          label: 'Device Name (Optional)',
          hint: 'Primary AMR',
          icon: Icons.label,
          theme: theme,
        ),
        const SizedBox(height: 16),
        _buildModernTextField(
          controller: _manualDeviceIpController,
          label: 'AMR IP Address',
          hint: '192.168.128.240',
          icon: Icons.smart_toy,
          theme: theme,
          keyboardType: TextInputType.numberWithOptions(decimal: true),
        ),
        const SizedBox(height: 16),
        _buildModernTextField(
          controller: _manualDevicePortController,
          label: 'Port',
          hint: '3000',
          icon: Icons.router,
          theme: theme,
          keyboardType: TextInputType.number,
        ),
      ],
    );
  }

  Widget _buildTabletDeviceInputs(ThemeService theme) {
    return Column(
      children: [
        Row(
          children: [
            Expanded(
              child: _buildModernTextField(
                controller: _manualDeviceIdController,
                label: 'Device ID',
                hint: 'AMR_01',
                icon: Icons.badge,
                theme: theme,
              ),
            ),
            const SizedBox(width: 24),
            Expanded(
              child: _buildModernTextField(
                controller: _manualDeviceNameController,
                label: 'Device Name (Optional)',
                hint: 'Primary AMR',
                icon: Icons.label,
                theme: theme,
              ),
            ),
          ],
        ),
        const SizedBox(height: 20),
        Row(
          children: [
            Expanded(
              flex: 3,
              child: _buildModernTextField(
                controller: _manualDeviceIpController,
                label: 'AMR IP Address',
                hint: '192.168.128.240',
                icon: Icons.smart_toy,
                theme: theme,
                keyboardType: TextInputType.numberWithOptions(decimal: true),
              ),
            ),
            const SizedBox(width: 24),
            Expanded(
              child: _buildModernTextField(
                controller: _manualDevicePortController,
                label: 'Port',
                hint: '3000',
                icon: Icons.router,
                theme: theme,
                keyboardType: TextInputType.number,
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildModernTextField({
    required TextEditingController controller,
    required String label,
    required String hint,
    required IconData icon,
    required ThemeService theme,
    TextInputType? keyboardType,
  }) {
    return Container(
      decoration: theme.glassMorphism,
      child: TextField(
        controller: controller,
        keyboardType: keyboardType,
        style: theme.bodyLarge.copyWith(
          fontSize: isTablet ? 16 : 14,
        ),
        decoration: InputDecoration(
          labelText: label,
          hintText: hint,
          prefixIcon: Container(
            margin: EdgeInsets.all(isTablet ? 10 : 8),
            padding: EdgeInsets.all(isTablet ? 10 : 8),
            decoration: BoxDecoration(
              color: theme.accentColor.withOpacity(0.1),
              borderRadius: theme.borderRadiusSmall,
            ),
            child: Icon(
              icon,
              color: theme.accentColor,
              size: isTablet ? 22 : 20,
            ),
          ),
          border: OutlineInputBorder(
            borderRadius: theme.borderRadiusMedium,
            borderSide: BorderSide.none,
          ),
          filled: true,
          fillColor: Colors.transparent,
          labelStyle: theme.bodyMedium.copyWith(
            color: theme.accentColor,
            fontSize: isTablet ? 16 : 14,
          ),
          hintStyle: theme.bodyMedium.copyWith(
            color: theme.isDarkMode
                ? Colors.white.withOpacity(0.5)
                : Colors.black.withOpacity(0.5),
            fontSize: isTablet ? 16 : 14,
          ),
          contentPadding: EdgeInsets.symmetric(
            horizontal: isTablet ? 20 : 16,
            vertical: isTablet ? 20 : 16,
          ),
        ),
      ),
    );
  }

  Widget _buildConnectedDevicesPanel(ThemeService theme) {
    return ModernGlassCard(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                padding: EdgeInsets.all(isTablet ? 16 : 12),
                decoration: BoxDecoration(
                  color: theme.successColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(
                  Icons.devices_other,
                  color: theme.successColor,
                  size: isTablet ? 32 : 28,
                ),
              ),
              SizedBox(width: isTablet ? 20 : 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Connected Fleet',
                      style: theme.headlineMedium.copyWith(
                        fontSize: isTablet ? 20 : 18,
                      ),
                    ),
                    Text(
                      'Active AMR devices in your fleet',
                      style: theme.bodyMedium.copyWith(
                        fontSize: isTablet ? 16 : 14,
                      ),
                    ),
                  ],
                ),
              ),
              Container(
                padding: EdgeInsets.symmetric(
                  horizontal: isTablet ? 16 : 12,
                  vertical: isTablet ? 8 : 6,
                ),
                decoration: BoxDecoration(
                  color: theme.successColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                  border:
                      Border.all(color: theme.successColor.withOpacity(0.3)),
                ),
                child: Text(
                  '${_connectedDevices.length} UNITS',
                  style: theme.bodySmall.copyWith(
                    color: theme.successColor,
                    fontWeight: FontWeight.bold,
                    letterSpacing: 1.2,
                    fontSize: isTablet ? 14 : 12,
                  ),
                ),
              ),
            ],
          ),
          SizedBox(height: verticalSpacing),
          if (_connectedDevices.isEmpty)
            _buildEmptyFleetState(theme)
          else
            _buildConnectedDevicesList(theme),
        ],
      ),
    );
  }

  Widget _buildEmptyFleetState(ThemeService theme) {
    return Container(
      padding: EdgeInsets.all(isTablet ? 40 : 32),
      decoration: BoxDecoration(
        color: theme.isDarkMode
            ? Colors.white.withOpacity(0.03)
            : Colors.black.withOpacity(0.03),
        borderRadius: theme.borderRadiusMedium,
        border: Border.all(
          color: theme.isDarkMode
              ? Colors.white.withOpacity(0.1)
              : Colors.black.withOpacity(0.1),
        ),
      ),
      child: Column(
        children: [
          Container(
            padding: EdgeInsets.all(isTablet ? 24 : 20),
            decoration: BoxDecoration(
              gradient: RadialGradient(
                colors: [
                  theme.accentColor.withOpacity(0.2),
                  theme.accentColor.withOpacity(0.05),
                ],
              ),
              shape: BoxShape.circle,
            ),
            child: Icon(
              Icons.devices_other,
              size: isTablet ? 80 : 64,
              color: theme.accentColor.withOpacity(0.7),
            ),
          ),
          SizedBox(height: isTablet ? 24 : 20),
          Text(
            'No Devices Connected',
            style: theme.headlineLarge.copyWith(
              color: theme.accentColor,
              fontSize: isTablet ? 24 : 20,
            ),
          ),
          SizedBox(height: isTablet ? 16 : 12),
          Text(
            _isWebSocketConnected
                ? 'Fleet backend is connected. Use the tabs above to discover and add AMR devices to your fleet.'
                : 'Connect to the AMR backend first, then add devices using discovery or manual connection.',
            style: theme.bodyMedium.copyWith(
              fontSize: isTablet ? 16 : 14,
            ),
            textAlign: TextAlign.center,
          ),
          SizedBox(height: verticalSpacing),
          if (!_isWebSocketConnected)
            Container(
              padding: EdgeInsets.all(isTablet ? 20 : 16),
              decoration: BoxDecoration(
                color: theme.warningColor.withOpacity(0.1),
                borderRadius: theme.borderRadiusMedium,
                border: Border.all(color: theme.warningColor.withOpacity(0.3)),
              ),
              child: Row(
                children: [
                  Icon(
                    Icons.warning,
                    color: theme.warningColor,
                    size: isTablet ? 24 : 20,
                  ),
                  SizedBox(width: isTablet ? 16 : 12),
                  Expanded(
                    child: Text(
                      'Backend Connection Required',
                      style: theme.bodyMedium.copyWith(
                        color: theme.warningColor,
                        fontWeight: FontWeight.w600,
                        fontSize: isTablet ? 16 : 14,
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

  Widget _buildConnectedDevicesList(ThemeService theme) {
    if (isTablet || isDesktop) {
      return _buildConnectedDevicesGrid(theme);
    } else {
      return ListView.separated(
        shrinkWrap: true,
        physics: const NeverScrollableScrollPhysics(),
        itemCount: _connectedDevices.length,
        separatorBuilder: (context, index) =>
            SizedBox(height: isTablet ? 16 : 12),
        itemBuilder: (context, index) {
          final device = _connectedDevices[index];
          return _buildConnectedDeviceCard(device, theme);
        },
      );
    }
  }

  Widget _buildConnectedDevicesGrid(ThemeService theme) {
    return GridView.builder(
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
        crossAxisCount: deviceGridCrossAxisCount,
        childAspectRatio: isDesktop ? 2.8 : 2.5,
        crossAxisSpacing: isTablet ? 20 : 16,
        mainAxisSpacing: isTablet ? 20 : 16,
      ),
      itemCount: _connectedDevices.length,
      itemBuilder: (context, index) {
        final device = _connectedDevices[index];
        return _buildConnectedDeviceCard(device, theme);
      },
    );
  }

  Widget _buildConnectedDeviceCard(
      Map<String, dynamic> device, ThemeService theme) {
    final deviceId = device['id']?.toString() ?? '';
    final deviceName = device['name']?.toString() ?? deviceId;
    final deviceStatus = device['status']?.toString() ?? 'unknown';

    // Device is only truly online if both device is connected AND backend is connected
    final isDeviceConnected = deviceStatus == 'connected';
    final isOnline = isDeviceConnected && _isWebSocketConnected;

    final isConnecting =
        deviceStatus == 'connecting' || _connectingDevices.contains(deviceId);
    final isRemoving = _removingDevices.contains(deviceId);
    final isRetrying = _retryingDevices.contains(deviceId);

    // Don't show device if it's being removed
    if (isRemoving) {
      return Container();
    }

    return ModernGlassCard(
      showGlow: isOnline || isConnecting,
      glowColor: isOnline
          ? theme.successColor
          : isConnecting
              ? theme.warningColor
              : theme.errorColor,
      onTap: isOnline ? () => _navigateToControl(device) : null,
      child: Padding(
        padding: EdgeInsets.all(isTablet ? 20 : 16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                // Device icon and status
                Stack(
                  children: [
                    Container(
                      padding: EdgeInsets.all(isTablet ? 16 : 12),
                      decoration: BoxDecoration(
                        gradient: LinearGradient(
                          colors: isOnline
                              ? [
                                  theme.successColor,
                                  theme.successColor.withOpacity(0.7)
                                ]
                              : isConnecting
                                  ? [
                                      theme.warningColor,
                                      theme.warningColor.withOpacity(0.7)
                                    ]
                                  : [
                                      theme.errorColor,
                                      theme.errorColor.withOpacity(0.7)
                                    ],
                        ),
                        borderRadius: theme.borderRadiusMedium,
                      ),
                      child: isConnecting
                          ? SizedBox(
                              width: isTablet ? 28 : 24,
                              height: isTablet ? 28 : 24,
                              child: CircularProgressIndicator(
                                strokeWidth: 3,
                                valueColor:
                                    AlwaysStoppedAnimation<Color>(Colors.white),
                              ),
                            )
                          : Icon(
                              Icons.smart_toy,
                              color: Colors.white,
                              size: isTablet ? 28 : 24,
                            ),
                    ),
                    Positioned(
                      right: 0,
                      bottom: 0,
                      child: Container(
                        width: isTablet ? 12 : 10,
                        height: isTablet ? 12 : 10,
                        decoration: BoxDecoration(
                          shape: BoxShape.circle,
                          color: isOnline
                              ? theme.successColor
                              : isConnecting
                                  ? theme.warningColor
                                  : theme.errorColor,
                          border: Border.all(color: Colors.white, width: 2),
                        ),
                      ),
                    ),
                  ],
                ),
                SizedBox(width: isTablet ? 16 : 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        deviceName,
                        style: theme.bodyLarge.copyWith(
                          fontSize: isTablet ? 18 : 16,
                          fontWeight: FontWeight.w600,
                        ),
                        maxLines: 1,
                        overflow: TextOverflow.ellipsis,
                      ),
                      SizedBox(height: 4),
                      Text(
                        'ID: $deviceId',
                        style: theme.bodySmall.copyWith(
                          fontSize: isTablet ? 13 : 11,
                        ),
                        maxLines: 1,
                        overflow: TextOverflow.ellipsis,
                      ),
                    ],
                  ),
                ),
              ],
            ),
            SizedBox(height: isTablet ? 16 : 12),

            // Status indicator
            RoboticStatusIndicator(
              status: _getActualDeviceStatus(deviceStatus),
              label: _getDeviceStatusLabel(deviceStatus),
              animated: isOnline || isConnecting,
            ),

            if (device['ipAddress'] != null) ...[
              SizedBox(height: isTablet ? 12 : 8),
              Row(
                children: [
                  Expanded(
                    child: Text(
                      'IP: ${device['ipAddress']}${device['port'] != null ? ':${device['port']}' : ''}',
                      style: theme.monospace.copyWith(
                        fontSize: isTablet ? 13 : 11,
                        color: theme.accentColor,
                      ),
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                    ),
                  ),
                ],
              ),
            ],

            SizedBox(height: isTablet ? 16 : 12),

            // Action buttons
            Row(
              children: [
                // Primary action button
                Expanded(
                  child: _buildDeviceActionButton(device, theme),
                ),
                SizedBox(width: isTablet ? 12 : 8),
                // Options button
                if (!isConnecting)
                  Container(
                    decoration: BoxDecoration(
                      color: theme.accentColor.withOpacity(0.1),
                      borderRadius: theme.borderRadiusSmall,
                    ),
                    child: IconButton(
                      onPressed: () => _showDeviceOptions(device),
                      icon: Icon(
                        Icons.more_vert,
                        color: theme.accentColor,
                        size: isTablet ? 20 : 18,
                      ),
                    ),
                  ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildDeviceActionButton(
      Map<String, dynamic> device, ThemeService theme) {
    final deviceId = device['id']?.toString() ?? '';
    final deviceStatus = device['status']?.toString() ?? 'unknown';
    final isDeviceConnected = deviceStatus == 'connected';
    final isOnline = isDeviceConnected && _isWebSocketConnected;
    final isConnecting =
        deviceStatus == 'connecting' || _connectingDevices.contains(deviceId);
    final isRetrying = _retryingDevices.contains(deviceId);

    String label;
    IconData icon;
    VoidCallback? onPressed;
    bool isLoading = false;
    bool isPrimary = true;

    if (isOnline) {
      label = 'Control';
      icon = Icons.gamepad;
      onPressed = () => _navigateToControl(device);
      isPrimary = true;
    } else if (!_isWebSocketConnected) {
      label = 'Backend Off';
      icon = Icons.wifi_off;
      onPressed = null;
      isPrimary = false;
    } else if (isConnecting) {
      label = 'Connecting...';
      icon = Icons.wifi;
      onPressed = null;
      isLoading = true;
      isPrimary = false;
    } else {
      label = isRetrying ? 'Retrying...' : 'Retry';
      icon = Icons.refresh;
      onPressed = isRetrying ? null : () => _retryConnection(device);
      isLoading = isRetrying;
      isPrimary = false;
    }

    return ModernActionButton(
      label: label,
      icon: icon,
      onPressed: onPressed ?? () {},
      isLoading: isLoading,
      isSecondary: !isPrimary,
    );
  }

  // Keep all existing methods from the original implementation
  // (All the connection, discovery, and device management methods remain the same)

  Future<String?> _detectBackendIP() async {
    // Implementation remains the same as original
    final networkInfo = await _getNetworkInfo();
    if (networkInfo == null) return null;

    final subnet = networkInfo['subnet']!;
    print('🔍 Scanning subnet: $subnet for AMR backend...');

    final deviceIP = networkInfo['ip']!;
    final deviceIPLast = int.parse(deviceIP.split('.').last);

    final scanIPs = <String>[];
    scanIPs.add(deviceIP);

    for (int i = 70; i <= 90; i++) {
      if (i != deviceIPLast) {
        scanIPs.add('$subnet.$i');
      }
    }

    for (int offset = 1; offset <= 10; offset++) {
      final ip1 = deviceIPLast + offset;
      final ip2 = deviceIPLast - offset;

      if (ip1 <= 254) scanIPs.add('$subnet.$ip1');
      if (ip2 >= 1) scanIPs.add('$subnet.$ip2');
    }

    for (int i = 0; i < scanIPs.length; i += 5) {
      final batch = scanIPs.skip(i).take(5);
      final futures = batch.map((ip) => _testBackendAtIP(ip));
      final results = await Future.wait(futures);

      for (int j = 0; j < results.length; j++) {
        if (results[j] != null) {
          final foundIP = batch.elementAt(j);
          print('✅ Found AMR backend at: $foundIP');
          return foundIP;
        }
      }
    }

    return null;
  }

  Future<String?> _testBackendAtIP(String ip) async {
    try {
      final response = await http.get(
        Uri.parse('http://$ip:3000/health'),
        headers: {'Accept': 'application/json'},
      ).timeout(const Duration(seconds: 2));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        if (data['success'] == true && data['status'] == 'healthy') {
          return ip;
        }
      }
    } catch (e) {
      // Ignore connection failures
    }
    return null;
  }

  Future<Map<String, String>?> _getNetworkInfo() async {
    try {
      for (final interface in await NetworkInterface.list()) {
        for (final addr in interface.addresses) {
          if (addr.type == InternetAddressType.IPv4 &&
              !addr.isLoopback &&
              (addr.address.startsWith('192.168.') ||
                  addr.address.startsWith('10.') ||
                  addr.address.startsWith('172.'))) {
            final ip = addr.address;
            final subnet = ip.split('.').take(3).join('.');
            return {
              'interface': interface.name,
              'ip': ip,
              'subnet': subnet,
            };
          }
        }
      }
    } catch (e) {
      print('❌ Error getting network info: $e');
    }
    return null;
  }

  Future<void> _connectToDetectedBackend(String ip) async {
    if (!mounted) return;

    setState(() {
      _isConnecting = true;
      _connectionStatus = 'Connecting to backend at $ip...';
    });

    try {
      _apiService.setBaseUrl('http://$ip:3000');

      final apiWorking = await _apiService.testConnection();
      if (!apiWorking) {
        throw Exception('API connection failed');
      }

      final wsUrl = 'ws://$ip:3000';
      final wsConnected = await _webSocketService
          .connect(wsUrl, deviceId: 'flutter_app', deviceInfo: {
        'name': 'Flutter AMR Controller',
        'type': 'mobile_client',
        'platform': Platform.isAndroid ? 'android' : 'ios',
      });

      if (!mounted) return;

      if (wsConnected) {
        setState(() {
          _connectionStatus = 'Connected to AMR Fleet Backend';
          _isWebSocketConnected = true;
          _detectedBackendIP = ip;
        });

        _showSuccessSnackBar('Connected to AMR backend at $ip');
      } else {
        throw Exception('WebSocket connection failed');
      }
    } catch (e) {
      if (mounted) {
        _showErrorSnackBar('Connection failed: $e');
        setState(() {
          _connectionStatus = 'Connection failed: $e';
        });
      }
    } finally {
      if (mounted) {
        setState(() {
          _isConnecting = false;
        });
      }
    }
  }

  Future<void> _connectManualWebSocket() async {
    final ip = _manualBackendIpController.text.trim();

    if (ip.isEmpty || !_isValidIP(ip)) {
      _showErrorSnackBar('Please enter a valid IP address');
      return;
    }

    await _connectToDetectedBackend(ip);
  }

  Future<void> _connectManualAMR() async {
    final deviceId = _manualDeviceIdController.text.trim();
    final deviceName = _manualDeviceNameController.text.trim();
    final deviceIp = _manualDeviceIpController.text.trim();
    final devicePort = _manualDevicePortController.text.trim();

    if (deviceId.isEmpty || deviceIp.isEmpty) {
      _showErrorSnackBar('Device ID and IP Address are required');
      return;
    }

    if (!_isValidIP(deviceIp)) {
      _showErrorSnackBar('Invalid device IP address format');
      return;
    }

    if (!mounted) return;

    setState(() {
      _isConnecting = true;
      _connectingDevices.add(deviceId);
    });

    try {
      if (!_apiService.isInitialized) {
        throw Exception(
            'Backend not connected. Please connect to backend first.');
      }

      final newDevice = {
        'id': deviceId,
        'name': deviceName.isNotEmpty ? deviceName : 'AMR $deviceId',
        'ipAddress': deviceIp,
        'port': devicePort.isNotEmpty ? devicePort : '3000',
        'status': _isWebSocketConnected ? 'connecting' : 'disconnected',
        'type': 'differential_drive',
        'capabilities': ['mapping', 'navigation', 'remote_control'],
        'isTemporary': true,
      };

      final originalDevices =
          List<Map<String, dynamic>>.from(_connectedDevices);

      setState(() {
        _connectedDevices.add(newDevice);
      });

      _showSuccessSnackBar(_isWebSocketConnected
          ? '🔗 Connecting to ${newDevice['name']} at $deviceIp...'
          : '📝 ${newDevice['name']} added - will connect when backend is online');

      if (!_isWebSocketConnected) {
        _showWarningSnackBar(
            '⚠️ Connect to backend first to establish device connection');
        return;
      }

      final result = await _apiService.connectDevice(
        deviceId: deviceId,
        name: deviceName.isNotEmpty ? deviceName : 'AMR $deviceId',
        ipAddress: deviceIp,
        type: 'differential_drive',
        capabilities: ['mapping', 'navigation', 'remote_control'],
      );

      if (!mounted) return;

      if (result['success'] == true) {
        _showSuccessSnackBar('✅ AMR device connected successfully!');

        setState(() {
          final deviceIndex =
              _connectedDevices.indexWhere((d) => d['id'] == deviceId);
          if (deviceIndex != -1) {
            _connectedDevices[deviceIndex] = {
              ..._connectedDevices[deviceIndex],
              'status': 'connected',
              'isTemporary': false,
            };
          }
        });

        _loadConnectedDevices();
        _clearManualDeviceFields();
        _showNavigationDialog();
      } else {
        setState(() {
          _connectedDevices = originalDevices;
        });
        _showErrorSnackBar('❌ Failed to connect device: ${result['error']}');
      }
    } catch (e) {
      setState(() {
        _connectedDevices.removeWhere((device) =>
            device['id'] == deviceId && device['isTemporary'] == true);
      });

      if (mounted) {
        _showErrorSnackBar('❌ Failed to connect AMR device: $e');
      }
    } finally {
      if (mounted) {
        setState(() {
          _isConnecting = false;
          _connectingDevices.remove(deviceId);
        });
      }
    }
  }

  void _clearManualDeviceFields() {
    _manualDeviceIdController.clear();
    _manualDeviceNameController.clear();
    _manualDeviceIpController.clear();
  }

  bool _isValidIP(String ip) {
    final parts = ip.split('.');
    if (parts.length != 4) return false;

    for (final part in parts) {
      final num = int.tryParse(part);
      if (num == null || num < 0 || num > 255) return false;
    }

    return true;
  }

  Future<void> _startAutoDiscovery() async {
    if (!mounted) return;

    setState(() {
      _isDiscovering = true;
    });

    _scanningController.repeat();

    try {
      final devices = await _discoveryService.discoverDevices(
        timeout: const Duration(seconds: 15),
        useNetworkScan: true,
        useMDNS: false,
        useBroadcast: false,
      );

      if (!mounted) return;

      setState(() {
        _discoveredDevices = devices;
      });

      print('🔍 Discovered ${devices.length} AMR devices');
    } catch (e) {
      print('❌ Device discovery failed: $e');
    } finally {
      if (mounted) {
        setState(() {
          _isDiscovering = false;
        });
        if (_scanningController.isAnimating) {
          _scanningController.stop();
          _scanningController.reset();
        }
      }
    }
  }

  Future<void> _autoConnectAMR() async {
    if (!mounted) return;

    setState(() {
      _isConnecting = true;
    });

    try {
      final result = await _apiService.autoConnectAMR();

      if (!mounted) return;

      if (result['success'] == true) {
        _showSuccessSnackBar('AMR auto-connected successfully');
        _loadConnectedDevices();
        _showNavigationDialog();
      } else {
        _showErrorSnackBar('Auto-connect failed: ${result['error']}');
      }
    } catch (e) {
      if (mounted) {
        _showErrorSnackBar('Auto-connect failed: $e');
      }
    } finally {
      if (mounted) {
        setState(() {
          _isConnecting = false;
        });
      }
    }
  }

  Future<void> _loadConnectedDevices() async {
    if (!mounted) return;

    setState(() {
      _isLoading = true;
    });

    try {
      if (!_apiService.isInitialized) {
        if (mounted) {
          setState(() {
            _connectedDevices = [];
          });
        }
        return;
      }

      final devices = await _apiService.getDevices();

      if (!mounted) return;

      setState(() {
        _connectedDevices = devices;
      });

      print('✅ Loaded ${devices.length} connected devices');
    } catch (e) {
      print('❌ Error loading devices: $e');

      if (e.toString().contains('Network error') ||
          e.toString().contains('Connection failed')) {
        print('🔄 Network error detected, testing backend connectivity...');

        try {
          final isHealthy =
              await _apiService.testConnectionWithRetry(maxRetries: 3);
          if (!isHealthy) {
            if (mounted) {
              _showErrorSnackBar(
                  '❌ Cannot connect to backend server. Please check if the server is running.');
            }
          } else {
            if (mounted) {
              _showWarningSnackBar(
                  '⚠️ Temporary connection issue. Retrying device loading...');
              Future.delayed(Duration(seconds: 2), () {
                if (mounted) _loadConnectedDevices();
              });
            }
          }
        } catch (connectivityError) {
          if (mounted) {
            _showErrorSnackBar(
                '❌ Backend server is unreachable. Please verify server status.');
          }
        }
      } else {
        if (mounted) {
          _showErrorSnackBar('❌ Failed to load devices: ${e.toString()}');
        }
      }

      if (mounted) {
        setState(() {
          _connectedDevices = [];
        });
      }
    } finally {
      if (mounted) {
        setState(() {
          _isLoading = false;
        });
      }
    }
  }

  Future<void> _refreshConnections() async {
    _scanningController.repeat();
    await Future.wait([
      _startAutoDiscovery(),
      _loadConnectedDevices(),
    ]);
    _scanningController.stop();
    _scanningController.reset();
  }

  Future<void> _navigateToControl(Map<String, dynamic> device) async {
    await Navigator.pushNamed(
      context,
      '/control',
      arguments: {
        'deviceId': device['id'],
        'deviceName': device['name'],
      },
    );
    Navigator.pushReplacementNamed(context, '/dashboard');
  }

  void _goToDashboard() {
    Navigator.pushReplacementNamed(context, '/dashboard');
  }

  void _showDeviceOptions(Map<String, dynamic> device) {
    final theme = Provider.of<ThemeService>(context, listen: false);
    final deviceId = device['id']?.toString() ?? '';
    final deviceStatus = device['status']?.toString() ?? 'unknown';
    final isOnline = deviceStatus == 'connected';
    final isConnecting =
        deviceStatus == 'connecting' || _connectingDevices.contains(deviceId);
    final isRemoving = _removingDevices.contains(deviceId);
    final isRetrying = _retryingDevices.contains(deviceId);

    if (isDesktop) {
      _showDesktopDeviceOptions(context, device, theme);
    } else {
      _showMobileDeviceOptions(context, device, theme);
    }
  }

  void _showMobileDeviceOptions(
      BuildContext context, Map<String, dynamic> device, ThemeService theme) {
    final deviceId = device['id']?.toString() ?? '';
    final deviceStatus = device['status']?.toString() ?? 'unknown';
    final isOnline = deviceStatus == 'connected';
    final isConnecting =
        deviceStatus == 'connecting' || _connectingDevices.contains(deviceId);
    final isRemoving = _removingDevices.contains(deviceId);
    final isRetrying = _retryingDevices.contains(deviceId);

    showModalBottomSheet(
      context: context,
      backgroundColor: Colors.transparent,
      builder: (context) => Container(
        decoration: BoxDecoration(
          gradient: theme.backgroundGradient,
          borderRadius: const BorderRadius.only(
            topLeft: Radius.circular(20),
            topRight: Radius.circular(20),
          ),
        ),
        child: Padding(
          padding: EdgeInsets.all(isTablet ? 24 : 20),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Text(
                'Device Options',
                style: theme.headlineLarge.copyWith(
                  fontSize: isTablet ? 22 : 18,
                ),
              ),
              SizedBox(height: isTablet ? 24 : 20),
              if (isConnecting) ...[
                Container(
                  padding: EdgeInsets.all(isTablet ? 20 : 16),
                  decoration: BoxDecoration(
                    color: theme.warningColor.withOpacity(0.1),
                    borderRadius: theme.borderRadiusMedium,
                    border:
                        Border.all(color: theme.warningColor.withOpacity(0.3)),
                  ),
                  child: Row(
                    children: [
                      SizedBox(
                        width: isTablet ? 24 : 20,
                        height: isTablet ? 24 : 20,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          valueColor:
                              AlwaysStoppedAnimation<Color>(theme.warningColor),
                        ),
                      ),
                      SizedBox(width: isTablet ? 16 : 12),
                      Expanded(
                        child: Text(
                          'Device is connecting to the backend...',
                          style: theme.bodyMedium.copyWith(
                            color: theme.warningColor,
                            fontWeight: FontWeight.w600,
                            fontSize: isTablet ? 16 : 14,
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
                SizedBox(height: isTablet ? 24 : 20),
              ],
              if (isOnline)
                _buildDeviceOption('Control Device', Icons.gamepad, () {
                  Navigator.pop(context);
                  _navigateToControl(device);
                }, theme),
              if (!isOnline && !isConnecting)
                _buildDeviceOption(
                  isRetrying ? 'Retrying Connection...' : 'Retry Connection',
                  isRetrying ? Icons.refresh : Icons.refresh,
                  isRetrying
                      ? () {}
                      : () {
                          Navigator.pop(context);
                          _retryConnection(device);
                        },
                  theme,
                  isDisabled: isRetrying,
                ),
              _buildDeviceOption(
                'Map Editor',
                Icons.map,
                isConnecting
                    ? () {}
                    : () {
                        Navigator.pop(context);
                        _navigateToMap(device);
                      },
                theme,
                isDisabled: isConnecting,
              ),
              _buildDeviceOption('Device Status', Icons.info, () {
                Navigator.pop(context);
                _showDeviceStatus(device);
              }, theme),
              _buildDeviceOption(
                isRemoving ? 'Removing Device...' : 'Remove Device',
                Icons.delete,
                (isRemoving || isConnecting)
                    ? () {}
                    : () {
                        Navigator.pop(context);
                        _confirmRemoveDevice(device);
                      },
                theme,
                isDestructive: true,
                isDisabled: isRemoving || isConnecting,
              ),
            ],
          ),
        ),
      ),
    );
  }

  void _showDesktopDeviceOptions(
      BuildContext context, Map<String, dynamic> device, ThemeService theme) {
    showDialog(
      context: context,
      builder: (context) => Dialog(
        backgroundColor: Colors.transparent,
        child: Container(
          width: 400,
          decoration: BoxDecoration(
            gradient: theme.backgroundGradient,
            borderRadius: theme.borderRadiusMedium,
            border: Border.all(
              color: theme.accentColor.withOpacity(0.3),
            ),
          ),
          child: Padding(
            padding: const EdgeInsets.all(32),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Text(
                  'Device Options',
                  style: theme.headlineLarge,
                ),
                const SizedBox(height: 24),
                _buildDeviceOption('Control Device', Icons.gamepad, () {
                  Navigator.pop(context);
                  _navigateToControl(device);
                }, theme),
                _buildDeviceOption('Map Editor', Icons.map, () {
                  Navigator.pop(context);
                  _navigateToMap(device);
                }, theme),
                _buildDeviceOption('Device Status', Icons.info, () {
                  Navigator.pop(context);
                  _showDeviceStatus(device);
                }, theme),
                _buildDeviceOption(
                  'Remove Device',
                  Icons.delete,
                  () {
                    Navigator.pop(context);
                    _confirmRemoveDevice(device);
                  },
                  theme,
                  isDestructive: true,
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildDeviceOption(
    String title,
    IconData icon,
    VoidCallback onTap,
    ThemeService theme, {
    bool isDestructive = false,
    bool isDisabled = false,
  }) {
    final color = isDestructive ? theme.errorColor : theme.accentColor;
    final displayColor = isDisabled ? color.withOpacity(0.5) : color;

    return Container(
      margin: EdgeInsets.only(bottom: isTablet ? 16 : 12),
      child: ModernGlassCard(
        onTap: isDisabled ? () {} : onTap,
        child: Row(
          children: [
            Container(
              padding: EdgeInsets.all(isTablet ? 16 : 12),
              decoration: BoxDecoration(
                color: displayColor.withOpacity(0.1),
                borderRadius: theme.borderRadiusSmall,
              ),
              child: isDisabled && title.contains('...')
                  ? SizedBox(
                      width: isTablet ? 20 : 16,
                      height: isTablet ? 20 : 16,
                      child: CircularProgressIndicator(
                        strokeWidth: 2,
                        valueColor: AlwaysStoppedAnimation<Color>(displayColor),
                      ),
                    )
                  : Icon(
                      icon,
                      color: displayColor,
                      size: isTablet ? 22 : 18,
                    ),
            ),
            SizedBox(width: isTablet ? 20 : 16),
            Expanded(
              child: Text(
                title,
                style: theme.bodyLarge.copyWith(
                  color: isDestructive
                      ? (isDisabled
                          ? theme.errorColor.withOpacity(0.5)
                          : theme.errorColor)
                      : (isDisabled
                          ? theme.accentColor.withOpacity(0.5)
                          : null),
                  fontWeight: FontWeight.w600,
                  fontSize: isTablet ? 16 : 14,
                ),
              ),
            ),
            Icon(
              Icons.arrow_forward_ios,
              color: displayColor,
              size: isTablet ? 18 : 16,
            ),
          ],
        ),
      ),
    );
  }

  void _navigateToMap(Map<String, dynamic> device) {
    Navigator.pushNamed(
      context,
      '/map',
      arguments: {
        'deviceId': device['id'],
      },
    );
  }

  void _showDeviceStatus(Map<String, dynamic> device) {
    final theme = Provider.of<ThemeService>(context, listen: false);

    showDialog(
      context: context,
      builder: (context) => Dialog(
        backgroundColor: Colors.transparent,
        child: Container(
          width: isDesktop ? 500 : null,
          decoration: BoxDecoration(
            gradient: theme.backgroundGradient,
            borderRadius: theme.borderRadiusMedium,
            border: Border.all(
              color: theme.accentColor.withOpacity(0.3),
            ),
          ),
          child: Padding(
            padding: EdgeInsets.all(isTablet ? 32 : 24),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  children: [
                    Icon(
                      Icons.info,
                      color: theme.infoColor,
                      size: isTablet ? 28 : 24,
                    ),
                    SizedBox(width: isTablet ? 16 : 12),
                    Text(
                      'Device Status',
                      style: theme.headlineLarge.copyWith(
                        fontSize: isTablet ? 22 : 18,
                      ),
                    ),
                  ],
                ),
                SizedBox(height: isTablet ? 24 : 20),
                _buildStatusRow(
                    'Device ID', device['id']?.toString() ?? '', theme),
                _buildStatusRow(
                    'Name', device['name']?.toString() ?? '', theme),
                _buildStatusRow(
                    'Type', device['type']?.toString() ?? '', theme),
                _buildStatusRow(
                    'Status', device['status']?.toString() ?? '', theme),
                if (device['ipAddress'] != null)
                  _buildStatusRow(
                      'IP Address', device['ipAddress'].toString(), theme),
                SizedBox(height: isTablet ? 32 : 24),
                Row(
                  mainAxisAlignment: MainAxisAlignment.end,
                  children: [
                    TextButton(
                      onPressed: () => Navigator.of(context).pop(),
                      child: Text(
                        'Close',
                        style: TextStyle(fontSize: isTablet ? 16 : 14),
                      ),
                    ),
                    SizedBox(width: isTablet ? 16 : 12),
                    ElevatedButton(
                      onPressed: () {
                        Navigator.of(context).pop();
                        _navigateToControl(device);
                      },
                      style: ElevatedButton.styleFrom(
                        backgroundColor: theme.accentColor,
                        foregroundColor: Colors.white,
                        padding: EdgeInsets.symmetric(
                          horizontal: isTablet ? 24 : 20,
                          vertical: isTablet ? 16 : 12,
                        ),
                      ),
                      child: Text(
                        'Control',
                        style: TextStyle(fontSize: isTablet ? 16 : 14),
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildStatusRow(String label, String value, ThemeService theme) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: isTablet ? 8 : 6),
      child: Row(
        children: [
          SizedBox(
            width: isTablet ? 120 : 100,
            child: Text(
              '$label:',
              style: theme.bodyMedium.copyWith(
                fontWeight: FontWeight.w600,
                fontSize: isTablet ? 16 : 14,
              ),
            ),
          ),
          Expanded(
            child: Text(
              value,
              style: theme.monospace.copyWith(
                color: theme.accentColor,
                fontSize: isTablet ? 15 : 13,
              ),
            ),
          ),
        ],
      ),
    );
  }

  void _confirmRemoveDevice(Map<String, dynamic> device) {
    final theme = Provider.of<ThemeService>(context, listen: false);

    showDialog(
      context: context,
      builder: (context) => Dialog(
        backgroundColor: Colors.transparent,
        child: Container(
          width: isDesktop ? 450 : null,
          decoration: BoxDecoration(
            gradient: theme.backgroundGradient,
            borderRadius: theme.borderRadiusMedium,
            border: Border.all(
              color: theme.errorColor.withOpacity(0.3),
            ),
          ),
          child: Padding(
            padding: EdgeInsets.all(isTablet ? 32 : 24),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Row(
                  children: [
                    Icon(
                      Icons.warning,
                      color: theme.errorColor,
                      size: isTablet ? 28 : 24,
                    ),
                    SizedBox(width: isTablet ? 16 : 12),
                    Text(
                      'Remove Device',
                      style: theme.headlineLarge.copyWith(
                        fontSize: isTablet ? 22 : 18,
                      ),
                    ),
                  ],
                ),
                SizedBox(height: isTablet ? 24 : 20),
                Text(
                  'Are you sure you want to remove "${device['name'] ?? device['id']}" from your fleet?',
                  style: theme.bodyMedium.copyWith(
                    fontSize: isTablet ? 16 : 14,
                  ),
                ),
                SizedBox(height: isTablet ? 32 : 24),
                Row(
                  mainAxisAlignment: MainAxisAlignment.end,
                  children: [
                    TextButton(
                      onPressed: () => Navigator.of(context).pop(),
                      child: Text(
                        'Cancel',
                        style: TextStyle(fontSize: isTablet ? 16 : 14),
                      ),
                    ),
                    SizedBox(width: isTablet ? 16 : 12),
                    ElevatedButton(
                      onPressed: () {
                        Navigator.of(context).pop();
                        _removeDevice(device['id']);
                      },
                      style: ElevatedButton.styleFrom(
                        backgroundColor: theme.errorColor,
                        foregroundColor: Colors.white,
                        padding: EdgeInsets.symmetric(
                          horizontal: isTablet ? 24 : 20,
                          vertical: isTablet ? 16 : 12,
                        ),
                      ),
                      child: Text(
                        'Remove',
                        style: TextStyle(fontSize: isTablet ? 16 : 14),
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  Future<void> _removeDevice(String deviceId) async {
    if (!mounted) return;

    setState(() {
      _removingDevices.add(deviceId);
    });

    try {
      final originalDevices =
          List<Map<String, dynamic>>.from(_connectedDevices);

      setState(() {
        _connectedDevices
            .removeWhere((device) => device['id'].toString() == deviceId);
      });

      if (!_apiService.isInitialized) {
        throw Exception('Backend not connected');
      }

      final result = await _apiService.disconnectDevice(deviceId: deviceId);

      if (!mounted) return;

      if (result['success'] == true) {
        _showSuccessSnackBar('Device removed successfully');
        _loadConnectedDevices();
      } else {
        setState(() {
          _connectedDevices = originalDevices;
        });
        throw Exception('Backend removal failed: ${result['error']}');
      }
    } catch (e) {
      _loadConnectedDevices();

      if (mounted) {
        _showErrorSnackBar('Failed to remove device: $e');
      }
    } finally {
      if (mounted) {
        setState(() {
          _removingDevices.remove(deviceId);
        });
      }
    }
  }

  Future<void> _retryConnection(Map<String, dynamic> device) async {
    final deviceId = device['id']?.toString() ?? '';
    if (!mounted || deviceId.isEmpty) return;

    if (!_isWebSocketConnected) {
      _showErrorSnackBar(
          'Connect to backend first before retrying device connection');
      return;
    }

    setState(() {
      _retryingDevices.add(deviceId);
    });

    try {
      if (!_apiService.isInitialized) {
        throw Exception('Backend not connected');
      }

      _showSuccessSnackBar('Reconnecting to ${device['name'] ?? deviceId}...');

      final deviceIp = device['ipAddress']?.toString() ?? '';
      if (deviceIp.isNotEmpty) {
        List<String> capabilities;
        try {
          final rawCapabilities = device['capabilities'];
          if (rawCapabilities is List) {
            capabilities = rawCapabilities.map((e) => e.toString()).toList();
          } else {
            capabilities = ['mapping', 'navigation', 'remote_control'];
          }
        } catch (e) {
          capabilities = ['mapping', 'navigation', 'remote_control'];
        }

        final result = await _apiService.connectDevice(
          deviceId: deviceId,
          name: device['name']?.toString() ?? 'AMR $deviceId',
          ipAddress: deviceIp,
          type: device['type']?.toString() ?? 'differential_drive',
          capabilities: capabilities,
        );

        if (!mounted) return;

        if (result['success'] == true) {
          _showSuccessSnackBar('✅ Device reconnected successfully!');
          _loadConnectedDevices();
        } else {
          _showErrorSnackBar('❌ Failed to reconnect: ${result['error']}');
        }
      } else {
        throw Exception('Device IP address not available');
      }
    } catch (e) {
      if (mounted) {
        String errorMessage = 'Connection retry failed';
        if (e.toString().contains('List<dynamic>')) {
          errorMessage =
              'Device configuration error - please remove and re-add device';
        } else if (e.toString().contains('IP address')) {
          errorMessage = 'Device IP address not available';
        } else if (e.toString().contains('Backend not connected')) {
          errorMessage = 'Backend connection lost - please reconnect first';
        } else if (e.toString().contains('timeout')) {
          errorMessage = 'Connection timeout - check device network';
        } else {
          errorMessage =
              'Connection retry failed: ${e.toString().replaceAll('Exception: ', '')}';
        }
        _showErrorSnackBar('❌ $errorMessage');
      }
    } finally {
      if (mounted) {
        setState(() {
          _retryingDevices.remove(deviceId);
        });
      }
    }
  }

  void _showNavigationDialog() {
    final theme = Provider.of<ThemeService>(context, listen: false);

    showDialog(
      context: context,
      builder: (context) => Dialog(
        backgroundColor: Colors.transparent,
        child: Container(
          width: isDesktop ? 500 : null,
          decoration: BoxDecoration(
            gradient: theme.backgroundGradient,
            borderRadius: theme.borderRadiusMedium,
            border: Border.all(
              color: theme.successColor.withOpacity(0.3),
            ),
          ),
          child: Padding(
            padding: EdgeInsets.all(isTablet ? 32 : 24),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Row(
                  children: [
                    Icon(
                      Icons.check_circle,
                      color: theme.successColor,
                      size: isTablet ? 28 : 24,
                    ),
                    SizedBox(width: isTablet ? 16 : 12),
                    Text(
                      'Device Connected',
                      style: theme.headlineLarge.copyWith(
                        fontSize: isTablet ? 22 : 18,
                      ),
                    ),
                  ],
                ),
                SizedBox(height: isTablet ? 24 : 20),
                Text(
                  'Your AMR device has been successfully connected to the fleet. Would you like to go to the Dashboard to manage your fleet?',
                  style: theme.bodyMedium.copyWith(
                    fontSize: isTablet ? 16 : 14,
                  ),
                ),
                SizedBox(height: isTablet ? 32 : 24),
                Row(
                  mainAxisAlignment: MainAxisAlignment.end,
                  children: [
                    TextButton(
                      onPressed: () => Navigator.of(context).pop(),
                      child: Text(
                        'Stay Here',
                        style: TextStyle(fontSize: isTablet ? 16 : 14),
                      ),
                    ),
                    SizedBox(width: isTablet ? 16 : 12),
                    ElevatedButton(
                      onPressed: () {
                        Navigator.of(context).pop();
                        Navigator.pushNamed(context, '/dashboard');
                      },
                      style: ElevatedButton.styleFrom(
                        backgroundColor: theme.successColor,
                        foregroundColor: Colors.white,
                        padding: EdgeInsets.symmetric(
                          horizontal: isTablet ? 24 : 20,
                          vertical: isTablet ? 16 : 12,
                        ),
                      ),
                      child: Text(
                        'Go to Dashboard',
                        style: TextStyle(fontSize: isTablet ? 16 : 14),
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  void _showSuccessSnackBar(String message) {
    final theme = Provider.of<ThemeService>(context, listen: false);

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Row(
          children: [
            Icon(Icons.check_circle, color: Colors.white),
            SizedBox(width: isTablet ? 16 : 12),
            Expanded(
              child: Text(
                message,
                style: TextStyle(fontSize: isTablet ? 16 : 14),
              ),
            ),
          ],
        ),
        backgroundColor: theme.successColor,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: theme.borderRadiusMedium),
        margin: EdgeInsets.all(isTablet ? 20 : 16),
      ),
    );
  }

  void _showErrorSnackBar(String message) {
    final theme = Provider.of<ThemeService>(context, listen: false);

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Row(
          children: [
            Icon(Icons.error, color: Colors.white),
            SizedBox(width: isTablet ? 16 : 12),
            Expanded(
              child: Text(
                message,
                style: TextStyle(fontSize: isTablet ? 16 : 14),
              ),
            ),
          ],
        ),
        backgroundColor: theme.errorColor,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: theme.borderRadiusMedium),
        duration: const Duration(seconds: 4),
        margin: EdgeInsets.all(isTablet ? 20 : 16),
      ),
    );
  }

  void _showWarningSnackBar(String message) {
    final theme = Provider.of<ThemeService>(context, listen: false);

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Row(
          children: [
            Icon(Icons.warning, color: Colors.white),
            SizedBox(width: isTablet ? 16 : 12),
            Expanded(
              child: Text(
                message,
                style: TextStyle(fontSize: isTablet ? 16 : 14),
              ),
            ),
          ],
        ),
        backgroundColor: theme.warningColor,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: theme.borderRadiusMedium),
        duration: const Duration(seconds: 3),
        margin: EdgeInsets.all(isTablet ? 20 : 16),
      ),
    );
  }

  Future<void> _discoverAMRDevices() async {
    if (!mounted) return;

    setState(() {
      _isDiscovering = true;
    });

    try {
      final devices = await _discoveryService.discoverDevices(
        timeout: const Duration(seconds: 15),
        useNetworkScan: true,
        useMDNS: false,
        useBroadcast: false,
      );

      if (!mounted) return;

      setState(() {
        _discoveredDevices = devices;
      });

      print('🔍 Discovered ${devices.length} AMR devices');
    } catch (e) {
      print('❌ Device discovery failed: $e');
    } finally {
      if (mounted) {
        setState(() {
          _isDiscovering = false;
        });
      }
    }
  }
}
