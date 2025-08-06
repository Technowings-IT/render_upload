// screens/connect_screen.dart - Modern Robotic Fleet Management Connection Interface
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
  final _deviceIdController = TextEditingController(text: 'agv_01');
  final _deviceNameController = TextEditingController(text: 'Primary AGV');
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
  List<AGVDevice> _discoveredDevices = [];
  String? _detectedBackendIP;
  bool _isLoading = false;
  bool _isConnecting = false;
  bool _isDiscovering = false;
  bool _isWebSocketConnected = false;
  String _connectionStatus = 'Initializing...';
  int _currentTabIndex = 0;

  // Stream subscriptions
  late StreamSubscription _deviceEventsSubscription;
  late StreamSubscription _connectionStateSubscription;

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _initializeConnections();
    _loadSavedConnections();
    _startInitialDiscovery();
  }

  @override
  void dispose() {
    _disposeControllers();
    _disposeAnimations();
    _deviceEventsSubscription.cancel();
    _connectionStateSubscription.cancel();
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
      _manualBackendIpController.text = '192.168.0.63';
      _manualDeviceIpController.text = '192.168.0.63';
    });
  }

  void _startInitialDiscovery() async {
    setState(() {
      _connectionStatus = 'Scanning network for AGV systems...';
    });

    _radarController.repeat();

    try {
      final backendIP = await _detectBackendIP();

      if (backendIP != null) {
        setState(() {
          _detectedBackendIP = backendIP;
          _connectionStatus = 'AGV Backend detected at $backendIP';
        });
        await _connectToDetectedBackend(backendIP);
      } else {
        setState(() {
          _connectionStatus =
              'No AGV backend found - manual connection available';
        });
      }

      await _discoverAGVDevices();
    } catch (e) {
      setState(() {
        _connectionStatus = 'Discovery failed: ${e.toString()}';
      });
    } finally {
      _radarController.stop();
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
          child: CustomScrollView(
            slivers: [
              _buildModernAppBar(theme),
              SliverPadding(
                padding: const EdgeInsets.all(20),
                sliver: SliverList(
                  delegate: SliverChildListDelegate([
                    _buildConnectionStatusPanel(theme),
                    const SizedBox(height: 24),
                    _buildConnectionTabs(theme),
                    const SizedBox(height: 24),
                    _buildTabContent(theme),
                    const SizedBox(height: 24),
                    _buildConnectedDevicesPanel(theme),
                  ]),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildModernAppBar(ThemeService theme) {
    return SliverAppBar(
      expandedHeight: 140,
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
              fontSize: 24,
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
                    left: MediaQuery.of(context).size.width *
                            _scanningAnimation.value -
                        100,
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
              ...List.generate(
                  8, (index) => _buildNetworkParticle(index, theme)),
            ],
          ),
        ),
      ),
      actions: [
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
        const SizedBox(width: 8),
      ],
    );
  }

  Widget _buildNetworkParticle(int index, ThemeService theme) {
    final screenWidth = MediaQuery.of(context).size.width;
    final left = (index * 47.3) % screenWidth;
    final animationDelay = index * 0.3;

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
                        size: 32,
                      ),
                    ),
                  );
                },
              ),
              const SizedBox(width: 20),
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
                      ),
                    ),
                    const SizedBox(height: 8),
                    Text(
                      _connectionStatus,
                      style: theme.bodyLarge,
                    ),
                    if (_detectedBackendIP != null) ...[
                      const SizedBox(height: 12),
                      Container(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 12, vertical: 6),
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
                          ),
                        ),
                      ),
                    ],
                  ],
                ),
              ),
              RoboticStatusIndicator(
                status: _isWebSocketConnected ? 'online' : 'offline',
                label: _isWebSocketConnected ? 'ONLINE' : 'OFFLINE',
                animated: _isWebSocketConnected,
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildConnectionTabs(ThemeService theme) {
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
      child: Row(
        children: [
          _buildTabButton('Auto Discovery', 0, Icons.radar, theme),
          _buildTabButton('Manual Backend', 1, Icons.settings_ethernet, theme),
          _buildTabButton(
              'Manual Device', 2, Icons.precision_manufacturing, theme),
        ],
      ),
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
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: theme.infoColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: AnimatedBuilder(
                  animation: _radarController,
                  builder: (context, child) {
                    return Transform.rotate(
                      angle: _radarSweep.value,
                      child:
                          Icon(Icons.radar, color: theme.infoColor, size: 28),
                    );
                  },
                ),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('Network Discovery', style: theme.headlineMedium),
                    Text('Automatically find AGV systems on your network',
                        style: theme.bodyMedium),
                  ],
                ),
              ),
            ],
          ),
          const SizedBox(height: 24),

          // Discovery results
          if (_discoveredDevices.isNotEmpty) ...[
            _buildDiscoveryResults(theme),
            const SizedBox(height: 24),
          ],

          // Action buttons
          Row(
            children: [
              Expanded(
                child: ModernActionButton(
                  label: _isDiscovering ? 'Scanning...' : 'Scan Network',
                  icon: Icons.search,
                  onPressed:
                      _isDiscovering ? () {} : () => _startAutoDiscovery(),
                  isLoading: _isDiscovering,
                  isSecondary: true,
                ),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: ModernActionButton(
                  label: 'Quick Connect',
                  icon: Icons.flash_on,
                  onPressed: () => _autoConnectAGV(),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildDiscoveryResults(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(16),
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
              const SizedBox(width: 8),
              Text(
                'Discovered Devices (${_discoveredDevices.length})',
                style: theme.bodyLarge.copyWith(
                  fontWeight: FontWeight.w600,
                  color: theme.successColor,
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          ListView.separated(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            itemCount: _discoveredDevices.length,
            separatorBuilder: (context, index) => const SizedBox(height: 8),
            itemBuilder: (context, index) {
              final device = _discoveredDevices[index];
              return _buildDiscoveredDeviceCard(device, theme);
            },
          ),
        ],
      ),
    );
  }

  Widget _buildDiscoveredDeviceCard(AGVDevice device, ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(12),
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
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              color: theme.accentColor.withOpacity(0.1),
              borderRadius: theme.borderRadiusSmall,
            ),
            child: Icon(Icons.router, color: theme.accentColor, size: 20),
          ),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(device.name,
                    style:
                        theme.bodyLarge.copyWith(fontWeight: FontWeight.w600)),
                Text(
                  '${device.ipAddress}:${device.port}',
                  style: theme.monospace.copyWith(fontSize: 12),
                ),
                Text(
                  'Method: ${device.discoveryMethod}',
                  style: theme.bodySmall,
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
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: theme.warningColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(Icons.settings_ethernet,
                    color: theme.warningColor, size: 28),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('Manual Backend Connection',
                        style: theme.headlineMedium),
                    Text('Connect to a specific AGV Fleet Backend',
                        style: theme.bodyMedium),
                  ],
                ),
              ),
            ],
          ),
          const SizedBox(height: 24),

          // Input fields
          Row(
            children: [
              Expanded(
                flex: 3,
                child: _buildModernTextField(
                  controller: _manualBackendIpController,
                  label: 'Backend IP Address',
                  hint: '192.168.0.63',
                  icon: Icons.computer,
                  theme: theme,
                ),
              ),
              const SizedBox(width: 16),
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
          ),

          const SizedBox(height: 24),

          // Connect button
          SizedBox(
            width: double.infinity,
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

  Widget _buildManualDevicePanel(ThemeService theme) {
    return ModernGlassCard(
      child: Column(
        children: [
          Row(
            children: [
              Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: theme.accentColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(Icons.precision_manufacturing,
                    color: theme.accentColor, size: 28),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('Manual AGV Device', style: theme.headlineMedium),
                    Text('Add a specific AGV device to your fleet',
                        style: theme.bodyMedium),
                  ],
                ),
              ),
            ],
          ),
          const SizedBox(height: 24),

          // Device information
          _buildModernTextField(
            controller: _manualDeviceIdController,
            label: 'Device ID',
            hint: 'agv_01',
            icon: Icons.badge,
            theme: theme,
          ),
          const SizedBox(height: 16),
          _buildModernTextField(
            controller: _manualDeviceNameController,
            label: 'Device Name (Optional)',
            hint: 'Primary AGV',
            icon: Icons.label,
            theme: theme,
          ),
          const SizedBox(height: 16),

          // Device network information
          Row(
            children: [
              Expanded(
                flex: 3,
                child: _buildModernTextField(
                  controller: _manualDeviceIpController,
                  label: 'AGV IP Address',
                  hint: '192.168.0.64',
                  icon: Icons.smart_toy,
                  theme: theme,
                  keyboardType: TextInputType.numberWithOptions(decimal: true),
                ),
              ),
              const SizedBox(width: 16),
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

          const SizedBox(height: 24),

          // Add device button
          SizedBox(
            width: double.infinity,
            child: ModernActionButton(
              label: _isConnecting ? 'Adding Device...' : 'Add AGV Device',
              icon: Icons.add_circle,
              onPressed: _isConnecting ? () {} : () => _connectManualAGV(),
              isLoading: _isConnecting,
            ),
          ),
        ],
      ),
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
        style: theme.bodyLarge,
        decoration: InputDecoration(
          labelText: label,
          hintText: hint,
          prefixIcon: Container(
            margin: const EdgeInsets.all(8),
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              color: theme.accentColor.withOpacity(0.1),
              borderRadius: theme.borderRadiusSmall,
            ),
            child: Icon(icon, color: theme.accentColor, size: 20),
          ),
          border: OutlineInputBorder(
            borderRadius: theme.borderRadiusMedium,
            borderSide: BorderSide.none,
          ),
          filled: true,
          fillColor: Colors.transparent,
          labelStyle: theme.bodyMedium.copyWith(color: theme.accentColor),
          hintStyle: theme.bodyMedium.copyWith(
            color: theme.isDarkMode
                ? Colors.white.withOpacity(0.5)
                : Colors.black.withOpacity(0.5),
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
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: theme.successColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(Icons.devices_other,
                    color: theme.successColor, size: 28),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('Connected Fleet', style: theme.headlineMedium),
                    Text('Active AGV devices in your fleet',
                        style: theme.bodyMedium),
                  ],
                ),
              ),
              Container(
                padding:
                    const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
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
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 24),
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
      padding: const EdgeInsets.all(32),
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
            padding: const EdgeInsets.all(20),
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
              size: 64,
              color: theme.accentColor.withOpacity(0.7),
            ),
          ),
          const SizedBox(height: 20),
          Text(
            'No Devices Connected',
            style: theme.headlineLarge.copyWith(color: theme.accentColor),
          ),
          const SizedBox(height: 12),
          Text(
            _isWebSocketConnected
                ? 'Fleet backend is connected. Use the tabs above to discover and add AGV devices to your fleet.'
                : 'Connect to the AGV backend first, then add devices using discovery or manual connection.',
            style: theme.bodyMedium,
            textAlign: TextAlign.center,
          ),
          const SizedBox(height: 24),
          if (!_isWebSocketConnected)
            Container(
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: theme.warningColor.withOpacity(0.1),
                borderRadius: theme.borderRadiusMedium,
                border: Border.all(color: theme.warningColor.withOpacity(0.3)),
              ),
              child: Row(
                children: [
                  Icon(Icons.warning, color: theme.warningColor),
                  const SizedBox(width: 12),
                  Expanded(
                    child: Text(
                      'Backend Connection Required',
                      style: theme.bodyMedium.copyWith(
                        color: theme.warningColor,
                        fontWeight: FontWeight.w600,
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
    return ListView.separated(
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      itemCount: _connectedDevices.length,
      separatorBuilder: (context, index) => const SizedBox(height: 12),
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
    final isOnline = deviceStatus == 'connected';

    return ModernGlassCard(
      showGlow: isOnline,
      glowColor: isOnline ? theme.successColor : theme.errorColor,
      onTap: () => _navigateToControl(device),
      child: Row(
        children: [
          // Device icon and status
          Stack(
            children: [
              Container(
                padding: const EdgeInsets.all(16),
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: isOnline
                        ? [
                            theme.successColor,
                            theme.successColor.withOpacity(0.7)
                          ]
                        : [theme.errorColor, theme.errorColor.withOpacity(0.7)],
                  ),
                  borderRadius: theme.borderRadiusMedium,
                ),
                child: Icon(Icons.smart_toy, color: Colors.white, size: 32),
              ),
              Positioned(
                right: 0,
                bottom: 0,
                child: Container(
                  width: 16,
                  height: 16,
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    color: isOnline ? theme.successColor : theme.errorColor,
                    border: Border.all(color: Colors.white, width: 2),
                    boxShadow: theme.elevationSmall,
                  ),
                ),
              ),
            ],
          ),

          const SizedBox(width: 16),

          // Device information
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(deviceName, style: theme.headlineMedium),
                const SizedBox(height: 4),
                Text('ID: $deviceId', style: theme.bodySmall),
                const SizedBox(height: 8),
                RoboticStatusIndicator(
                  status: deviceStatus,
                  label: deviceStatus.toUpperCase(),
                  animated: isOnline,
                ),
                if (device['ipAddress'] != null) ...[
                  const SizedBox(height: 8),
                  Text(
                    'IP: ${device['ipAddress']}',
                    style: theme.monospace.copyWith(fontSize: 12),
                  ),
                ],
              ],
            ),
          ),

          // Action buttons
          Column(
            children: [
              ModernActionButton(
                label: 'Control',
                icon: Icons.gamepad,
                onPressed: () => _navigateToControl(device),
                isSecondary: true,
              ),
              const SizedBox(height: 8),
              IconButton(
                onPressed: () => _showDeviceOptions(device),
                icon: Icon(Icons.more_vert, color: theme.accentColor),
              ),
            ],
          ),
        ],
      ),
    );
  }

  // Keep all your existing methods but add modern UI touches
  Future<String?> _detectBackendIP() async {
    // Your existing implementation
    final networkInfo = await _getNetworkInfo();
    if (networkInfo == null) return null;

    final subnet = networkInfo['subnet']!;
    print('🔍 Scanning subnet: $subnet for AGV backend...');

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
          print('✅ Found AGV backend at: $foundIP');
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
        'name': 'Flutter AGV Controller',
        'type': 'mobile_client',
        'platform': Platform.isAndroid ? 'android' : 'ios',
      });

      if (wsConnected) {
        setState(() {
          _connectionStatus = 'Connected to AGV Fleet Backend';
          _isWebSocketConnected = true;
          _detectedBackendIP = ip;
        });

        _showSuccessSnackBar('Connected to AGV backend at $ip');
      } else {
        throw Exception('WebSocket connection failed');
      }
    } catch (e) {
      _showErrorSnackBar('Connection failed: $e');
      setState(() {
        _connectionStatus = 'Connection failed: $e';
      });
    } finally {
      setState(() {
        _isConnecting = false;
      });
    }
  }

  Future<void> _connectManualWebSocket() async {
    final ip = _manualBackendIpController.text.trim();
    final port = _manualBackendPortController.text.trim();

    if (ip.isEmpty || !_isValidIP(ip)) {
      _showErrorSnackBar('Please enter a valid IP address');
      return;
    }

    await _connectToDetectedBackend(ip);
  }

  Future<void> _connectManualAGV() async {
    final deviceId = _manualDeviceIdController.text.trim();
    final deviceName = _manualDeviceNameController.text.trim();
    final deviceIp = _manualDeviceIpController.text.trim();

    if (deviceId.isEmpty || deviceIp.isEmpty) {
      _showErrorSnackBar('Device ID and IP Address are required');
      return;
    }

    if (!_isValidIP(deviceIp)) {
      _showErrorSnackBar('Invalid device IP address format');
      return;
    }

    setState(() {
      _isConnecting = true;
    });

    try {
      if (!_apiService.isInitialized) {
        throw Exception(
            'Backend not connected. Please connect to backend first.');
      }

      final result = await _apiService.connectDevice(
        deviceId: deviceId,
        name: deviceName.isNotEmpty ? deviceName : 'AGV $deviceId',
        ipAddress: deviceIp,
        type: 'differential_drive',
        capabilities: ['mapping', 'navigation', 'remote_control'],
      );

      if (result['success'] == true) {
        _showSuccessSnackBar('AGV device connected successfully');
        _loadConnectedDevices();
        _clearManualDeviceFields();
        _showNavigationDialog();
      } else {
        _showErrorSnackBar('Failed to connect device: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Failed to connect AGV device: $e');
    } finally {
      setState(() {
        _isConnecting = false;
      });
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

      setState(() {
        _discoveredDevices = devices;
      });

      print('🔍 Discovered ${devices.length} AGV devices');
    } catch (e) {
      print('❌ Device discovery failed: $e');
    } finally {
      setState(() {
        _isDiscovering = false;
      });
      _scanningController.stop();
      _scanningController.reset();
    }
  }

  Future<void> _autoConnectAGV() async {
    setState(() {
      _isConnecting = true;
    });

    try {
      final result = await _apiService.autoConnectAGV();

      if (result['success'] == true) {
        _showSuccessSnackBar('AGV auto-connected successfully');
        _loadConnectedDevices();
        _showNavigationDialog();
      } else {
        _showErrorSnackBar('Auto-connect failed: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Auto-connect failed: $e');
    } finally {
      setState(() {
        _isConnecting = false;
      });
    }
  }

  Future<void> _loadConnectedDevices() async {
    setState(() {
      _isLoading = true;
    });

    try {
      if (!_apiService.isInitialized) {
        setState(() {
          _connectedDevices = [];
        });
        return;
      }

      final devices = await _apiService.getDevices();
      setState(() {
        _connectedDevices = devices;
      });
    } catch (e) {
      print('❌ Error loading devices: $e');
      setState(() {
        _connectedDevices = [];
      });
    } finally {
      setState(() {
        _isLoading = false;
      });
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

  void _showDeviceOptions(Map<String, dynamic> device) {
    final theme = Provider.of<ThemeService>(context, listen: false);

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
          padding: const EdgeInsets.all(20),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Text(
                'Device Options',
                style: theme.headlineLarge,
              ),
              const SizedBox(height: 20),
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
              _buildDeviceOption('Remove Device', Icons.delete, () {
                Navigator.pop(context);
                _confirmRemoveDevice(device);
              }, theme, isDestructive: true),
            ],
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
  }) {
    final color = isDestructive ? theme.errorColor : theme.accentColor;

    return Container(
      margin: const EdgeInsets.only(bottom: 12),
      child: ModernGlassCard(
        onTap: onTap,
        child: Row(
          children: [
            Container(
              padding: const EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: color.withOpacity(0.1),
                borderRadius: theme.borderRadiusSmall,
              ),
              child: Icon(icon, color: color),
            ),
            const SizedBox(width: 16),
            Expanded(
              child: Text(
                title,
                style: theme.bodyLarge.copyWith(
                  color: isDestructive ? theme.errorColor : null,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ),
            Icon(Icons.arrow_forward_ios, color: color, size: 16),
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
      builder: (context) => AlertDialog(
        backgroundColor:
            theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
        shape: RoundedRectangleBorder(
          borderRadius: theme.borderRadiusMedium,
        ),
        title: Row(
          children: [
            Icon(Icons.info, color: theme.infoColor),
            const SizedBox(width: 12),
            Text('Device Status'),
          ],
        ),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            _buildStatusRow('Device ID', device['id']?.toString() ?? '', theme),
            _buildStatusRow('Name', device['name']?.toString() ?? '', theme),
            _buildStatusRow('Type', device['type']?.toString() ?? '', theme),
            _buildStatusRow(
                'Status', device['status']?.toString() ?? '', theme),
            if (device['ipAddress'] != null)
              _buildStatusRow(
                  'IP Address', device['ipAddress'].toString(), theme),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Close'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _navigateToControl(device);
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: theme.accentColor,
              foregroundColor: Colors.white,
            ),
            child: Text('Control'),
          ),
        ],
      ),
    );
  }

  Widget _buildStatusRow(String label, String value, ThemeService theme) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Row(
        children: [
          SizedBox(
            width: 100,
            child: Text(
              '$label:',
              style: theme.bodyMedium.copyWith(fontWeight: FontWeight.w600),
            ),
          ),
          Expanded(
            child: Text(
              value,
              style: theme.monospace.copyWith(
                color: theme.accentColor,
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
      builder: (context) => AlertDialog(
        backgroundColor:
            theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
        shape: RoundedRectangleBorder(
          borderRadius: theme.borderRadiusMedium,
        ),
        title: Row(
          children: [
            Icon(Icons.warning, color: theme.errorColor),
            const SizedBox(width: 12),
            Text('Remove Device'),
          ],
        ),
        content: Text(
          'Are you sure you want to remove "${device['name'] ?? device['id']}" from your fleet?',
          style: theme.bodyMedium,
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _removeDevice(device['id']);
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: theme.errorColor,
              foregroundColor: Colors.white,
            ),
            child: Text('Remove'),
          ),
        ],
      ),
    );
  }

  Future<void> _removeDevice(String deviceId) async {
    setState(() {
      _isLoading = true;
    });

    try {
      if (!_apiService.isInitialized) {
        throw Exception('Backend not connected');
      }

      final result = await _apiService.disconnectDevice(deviceId: deviceId);
      if (result['success'] == true) {
        _showSuccessSnackBar('Device removed successfully');
        _loadConnectedDevices();
      } else {
        _showErrorSnackBar('Failed to remove device: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Failed to remove device: $e');
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  void _showNavigationDialog() {
    final theme = Provider.of<ThemeService>(context, listen: false);

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        backgroundColor:
            theme.isDarkMode ? const Color(0xFF262640) : Colors.white,
        shape: RoundedRectangleBorder(
          borderRadius: theme.borderRadiusMedium,
        ),
        title: Row(
          children: [
            Icon(Icons.check_circle, color: theme.successColor),
            const SizedBox(width: 12),
            Text('Device Connected'),
          ],
        ),
        content: Text(
          'Your AGV device has been successfully connected to the fleet. Would you like to go to the Dashboard to manage your fleet?',
          style: theme.bodyMedium,
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Stay Here'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              Navigator.pushNamed(context, '/dashboard');
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: theme.successColor,
              foregroundColor: Colors.white,
            ),
            child: Text('Go to Dashboard'),
          ),
        ],
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
            const SizedBox(width: 12),
            Expanded(child: Text(message)),
          ],
        ),
        backgroundColor: theme.successColor,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: theme.borderRadiusMedium),
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
            const SizedBox(width: 12),
            Expanded(child: Text(message)),
          ],
        ),
        backgroundColor: theme.errorColor,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: theme.borderRadiusMedium),
        duration: const Duration(seconds: 4),
      ),
    );
  }

  Future<void> _discoverAGVDevices() async {
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

      setState(() {
        _discoveredDevices = devices;
      });

      print('🔍 Discovered ${devices.length} AGV devices');
    } catch (e) {
      print('❌ Device discovery failed: $e');
    } finally {
      setState(() {
        _isDiscovering = false;
      });
    }
  }
}
