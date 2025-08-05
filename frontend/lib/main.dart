// main.dart - Enhanced with Modern Robotic Theme Integration
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'services/theme_service.dart';
import 'services/smart_connection_service.dart';
import 'screens/splash_screen.dart';
import 'screens/login_screen.dart';
import 'screens/dashboard_screen.dart';
import 'screens/connect_screen.dart';
import 'screens/control_page.dart';
import 'screens/map_page.dart';
import 'screens/analytics_screen.dart';
import 'screens/settings_screen.dart';
import 'screens/profile_screen.dart';
import 'services/web_socket_service.dart';
import 'services/api_service.dart';
import 'widgets/modern_ui_components.dart';

// Configuration constants
class AppConfig {
  static const String DEFAULT_SERVER_IP = '192.168.0.63';
  static const int DEFAULT_SERVER_PORT = 3000;
  static const int AGV_SSH_PORT = 22;

  static String get serverUrl =>
      'http://$DEFAULT_SERVER_IP:$DEFAULT_SERVER_PORT';
  static String get websocketUrl =>
      'ws://$DEFAULT_SERVER_IP:$DEFAULT_SERVER_PORT';

  static const Duration connectionTimeout = Duration(seconds: 15);
  static const Duration apiTimeout = Duration(seconds: 10);
  static const int maxRetryAttempts = 3;
}

void main() async {
  WidgetsFlutterBinding.ensureInitialized();

  // Set system UI overlay style for modern look
  SystemChrome.setSystemUIOverlayStyle(
    const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.light,
      systemNavigationBarColor: Color(0xFF0F0F23),
      systemNavigationBarIconBrightness: Brightness.light,
    ),
  );

  print('🚀 Starting AGV Fleet Management App...');
  print('📡 Server: ${AppConfig.serverUrl}');
  print('🔌 WebSocket: ${AppConfig.websocketUrl}');

  // Initialize theme service
  final themeService = ThemeService();
  await themeService.loadTheme();

  // Initialize API service
  print('🔧 Initializing API Service with URL: ${AppConfig.serverUrl}');
  ApiService().initialize(AppConfig.serverUrl);

  // Test backend connection
  try {
    final connectionTest = await ApiService().testConnection();
    print(
        '🔗 Backend connection test: ${connectionTest ? "✅ SUCCESS" : "❌ FAILED"}');
  } catch (e) {
    print('❌ Backend connection error: $e');
  }

  runApp(
    ChangeNotifierProvider.value(
      value: themeService,
      child: AGVFleetManagementApp(),
    ),
  );
}

class AGVFleetManagementApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Consumer<ThemeService>(
      builder: (context, themeService, child) {
        return MaterialApp(
          title: 'AGV Fleet Management',
          theme: themeService.lightTheme,
          darkTheme: themeService.darkTheme,
          themeMode: themeService.isDarkMode ? ThemeMode.dark : ThemeMode.light,
          debugShowCheckedModeBanner: false,
          initialRoute: '/splash',
          routes: {
            '/splash': (context) => ModernSplashScreen(),
            '/login': (context) => LoginScreen(),
            '/': (context) => ModernMainNavigationWrapper(),
            '/dashboard': (context) =>
                ModernMainNavigationWrapper(initialIndex: 0),
            '/connect': (context) =>
                ModernMainNavigationWrapper(initialIndex: 1),
            '/analytics': (context) =>
                ModernMainNavigationWrapper(initialIndex: 2),
            '/settings': (context) =>
                ModernMainNavigationWrapper(initialIndex: 3),
          },
          onGenerateRoute: (settings) {
            switch (settings.name) {
              case '/control':
                final args = settings.arguments as Map<String, dynamic>?;
                return _createModernPageRoute(
                  ControlPage(deviceId: args?['deviceId'] ?? 'agv_01'),
                );
              case '/map':
                final args = settings.arguments as Map<String, dynamic>?;
                return _createModernPageRoute(
                  EnhancedMapPage(deviceId: args?['deviceId']),
                );
              default:
                return _createModernPageRoute(ModernMainNavigationWrapper());
            }
          },
        );
      },
    );
  }

  PageRoute _createModernPageRoute(Widget page) {
    return PageRouteBuilder(
      pageBuilder: (context, animation, secondaryAnimation) => page,
      transitionsBuilder: (context, animation, secondaryAnimation, child) {
        const begin = Offset(1.0, 0.0);
        const end = Offset.zero;
        const curve = Curves.easeInOutCubic;

        var tween = Tween(begin: begin, end: end).chain(
          CurveTween(curve: curve),
        );

        return SlideTransition(
          position: animation.drive(tween),
          child: FadeTransition(
            opacity: animation,
            child: child,
          ),
        );
      },
      transitionDuration: const Duration(milliseconds: 600),
    );
  }
}

// 🚀 Modern Splash Screen
class ModernSplashScreen extends StatefulWidget {
  @override
  State<ModernSplashScreen> createState() => _ModernSplashScreenState();
}

class _ModernSplashScreenState extends State<ModernSplashScreen>
    with TickerProviderStateMixin {
  late AnimationController _logoController;
  late AnimationController _textController;
  late Animation<double> _logoScale;
  late Animation<double> _textFade;

  @override
  void initState() {
    super.initState();

    _logoController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );
    _textController = AnimationController(
      duration: const Duration(milliseconds: 1000),
      vsync: this,
    );

    _logoScale = Tween<double>(begin: 0.0, end: 1.0).animate(
      CurvedAnimation(parent: _logoController, curve: Curves.elasticOut),
    );
    _textFade = Tween<double>(begin: 0.0, end: 1.0).animate(
      CurvedAnimation(parent: _textController, curve: Curves.easeIn),
    );

    _startAnimations();
  }

  void _startAnimations() async {
    await _logoController.forward();
    await _textController.forward();
    await Future.delayed(const Duration(milliseconds: 1500));

    if (mounted) {
      Navigator.pushReplacementNamed(context, '/login');
    }
  }

  @override
  void dispose() {
    _logoController.dispose();
    _textController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    return Scaffold(
      body: Container(
        decoration: BoxDecoration(
          gradient: theme.backgroundGradient,
        ),
        child: Stack(
          children: [
            // Background particles
            ...List.generate(20, (index) => _buildParticle(index)),

            // Main content
            Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  AnimatedBuilder(
                    animation: _logoScale,
                    builder: (context, child) {
                      return Transform.scale(
                        scale: _logoScale.value,
                        child: Container(
                          padding: const EdgeInsets.all(30),
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
                          child: Icon(
                            Icons.smart_toy,
                            size: 80,
                            color: theme.accentColor,
                          ),
                        ),
                      );
                    },
                  ),
                  const SizedBox(height: 40),
                  AnimatedBuilder(
                    animation: _textFade,
                    builder: (context, child) {
                      return Opacity(
                        opacity: _textFade.value,
                        child: Column(
                          children: [
                            Text(
                              'AGV Fleet Manager',
                              style: theme.displayLarge.copyWith(
                                color: theme.accentColor,
                              ),
                            ),
                            const SizedBox(height: 16),
                            Text(
                              'Next-Generation Robotic Control',
                              style: theme.bodyLarge.copyWith(
                                color: theme.isDarkMode
                                    ? Colors.white.withOpacity(0.7)
                                    : Colors.black.withOpacity(0.7),
                              ),
                            ),
                          ],
                        ),
                      );
                    },
                  ),
                  const SizedBox(height: 60),
                  ModernLoadingIndicator(
                    message: 'Initializing systems...',
                    size: 40,
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildParticle(int index) {
    final screenSize = MediaQuery.of(context).size;
    final random = (index * 54321) % 1000;
    final left = (random % 100) / 100 * screenSize.width;
    final top = ((random ~/ 100) % 100) / 100 * screenSize.height;
    final size = 2.0 + (random % 6);

    return Positioned(
      left: left,
      top: top,
      child: Container(
        width: size,
        height: size,
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          color: Colors.cyan.withOpacity(0.1 + (random % 30) / 100),
        ),
      ),
    );
  }
}

// 🎯 Modern Main Navigation Wrapper
class ModernMainNavigationWrapper extends StatefulWidget {
  final int initialIndex;

  const ModernMainNavigationWrapper({Key? key, this.initialIndex = 0})
      : super(key: key);

  @override
  State<ModernMainNavigationWrapper> createState() =>
      _ModernMainNavigationWrapperState();
}

class _ModernMainNavigationWrapperState
    extends State<ModernMainNavigationWrapper> with TickerProviderStateMixin {
  late int _currentIndex;
  late AnimationController _fabController;
  final WebSocketService _webSocketService = WebSocketService();
  final ApiService _apiService = ApiService();

  bool _isConnectedToServer = false;
  bool _isWebSocketConnected = false;
  bool _isInitializing = true;
  String _connectionStatus = 'Connecting...';

  final List<Widget> _screens = [
    DashboardScreen(),
    ConnectScreen(),
    EnhancedAnalyticsScreen(),
    SettingsScreen(),
  ];

  @override
  void initState() {
    super.initState();
    _currentIndex = widget.initialIndex;
    _fabController = AnimationController(
      duration: const Duration(milliseconds: 300),
      vsync: this,
    );
    _initializeServices();
  }

  void _initializeServices() async {
    setState(() {
      _isInitializing = true;
      _connectionStatus = 'Connecting to AGV server...';
    });

    await _connectWithRetry();
    _fabController.forward();
  }

  Future<void> _connectWithRetry() async {
    for (int attempt = 1; attempt <= AppConfig.maxRetryAttempts; attempt++) {
      try {
        setState(() {
          _connectionStatus =
              'Attempting connection $attempt/${AppConfig.maxRetryAttempts}...';
        });

        final apiHealthy = await _apiService
            .testConnection()
            .timeout(AppConfig.connectionTimeout);

        if (apiHealthy) {
          setState(() {
            _isConnectedToServer = true;
            _connectionStatus = 'API connected, initializing WebSocket...';
          });

          final wsConnected = await _webSocketService
              .connect(_apiService.getWebSocketUrl() ?? '');

          setState(() {
            _isWebSocketConnected = wsConnected;
            _isInitializing = false;
            _connectionStatus = wsConnected ? 'Fully connected' : 'API only';
          });

          return;
        }
      } catch (e) {
        if (attempt < AppConfig.maxRetryAttempts) {
          setState(() {
            _connectionStatus = 'Retrying in 3 seconds...';
          });
          await Future.delayed(const Duration(seconds: 3));
        } else {
          setState(() {
            _isConnectedToServer = false;
            _isWebSocketConnected = false;
            _isInitializing = false;
            _connectionStatus = 'Connection failed';
          });
        }
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    if (_isInitializing) {
      return Scaffold(
        body: Container(
          decoration: BoxDecoration(gradient: theme.backgroundGradient),
          child: ModernLoadingIndicator(
            message: _connectionStatus,
            size: 60,
          ),
        ),
      );
    }

    return Scaffold(
      extendBody: true,
      appBar: _buildModernAppBar(theme),
      body: Container(
        decoration: BoxDecoration(gradient: theme.backgroundGradient),
        child: IndexedStack(
          index: _currentIndex,
          children: _screens,
        ),
      ),
      bottomNavigationBar: _buildModernBottomNav(theme),
      floatingActionButton: _buildModernFAB(theme),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerDocked,
      drawer: _buildModernDrawer(theme),
    );
  }

  PreferredSizeWidget _buildModernAppBar(ThemeService theme) {
    return AppBar(
      title: ShaderMask(
        shaderCallback: (bounds) => LinearGradient(
          colors: [theme.accentColor, theme.accentColor.withOpacity(0.7)],
        ).createShader(bounds),
        child: Text(
          'AGV Fleet Control',
          style: theme.displayMedium.copyWith(
            fontSize: 20,
            color: Colors.white,
          ),
        ),
      ),
      backgroundColor: Colors.transparent,
      elevation: 0,
      flexibleSpace: Container(
        decoration: BoxDecoration(
          gradient: LinearGradient(
            colors: theme.isDarkMode
                ? [
                    const Color(0xFF1E1E2E).withOpacity(0.95),
                    const Color(0xFF262640).withOpacity(0.95),
                  ]
                : [
                    Colors.white.withOpacity(0.95),
                    Colors.white.withOpacity(0.9),
                  ],
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
          ),
        ),
      ),
      actions: [
        // Connection status indicator
        Padding(
          padding: const EdgeInsets.only(right: 8),
          child: RoboticStatusIndicator(
            status: _isConnectedToServer ? 'online' : 'offline',
            label: _isConnectedToServer ? 'ONLINE' : 'OFFLINE',
            animated: _isConnectedToServer,
          ),
        ),
        // Theme toggle
        IconButton(
          onPressed: theme.toggleTheme,
          icon: AnimatedSwitcher(
            duration: const Duration(milliseconds: 300),
            child: Icon(
              theme.isDarkMode ? Icons.light_mode : Icons.dark_mode,
              key: ValueKey(theme.isDarkMode),
              color: theme.accentColor,
            ),
          ),
        ),
        // Notifications
        IconButton(
          onPressed: () => _showNotifications(),
          icon: Stack(
            children: [
              Icon(Icons.notifications_outlined, color: theme.accentColor),
              Positioned(
                right: 0,
                top: 0,
                child: Container(
                  width: 8,
                  height: 8,
                  decoration: BoxDecoration(
                    color: theme.errorColor,
                    shape: BoxShape.circle,
                  ),
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildModernBottomNav(ThemeService theme) {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
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
        borderRadius: const BorderRadius.only(
          topLeft: Radius.circular(20),
          topRight: Radius.circular(20),
        ),
        border: Border(
          top: BorderSide(
            color: theme.isDarkMode
                ? Colors.white.withOpacity(0.1)
                : Colors.black.withOpacity(0.1),
            width: 1,
          ),
        ),
      ),
      child: BottomNavigationBar(
        currentIndex: _currentIndex,
        onTap: (index) => setState(() => _currentIndex = index),
        type: BottomNavigationBarType.fixed,
        backgroundColor: Colors.transparent,
        elevation: 0,
        selectedItemColor: theme.accentColor,
        unselectedItemColor: theme.isDarkMode
            ? Colors.white.withOpacity(0.6)
            : Colors.black.withOpacity(0.6),
        selectedLabelStyle: TextStyle(fontWeight: FontWeight.w600),
        items: const [
          BottomNavigationBarItem(
            icon: Icon(Icons.dashboard_outlined),
            activeIcon: Icon(Icons.dashboard),
            label: 'Dashboard',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.devices_outlined),
            activeIcon: Icon(Icons.devices),
            label: 'Devices',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.analytics_outlined),
            activeIcon: Icon(Icons.analytics),
            label: 'Analytics',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.settings_outlined),
            activeIcon: Icon(Icons.settings),
            label: 'Settings',
          ),
        ],
      ),
    );
  }

  Widget _buildModernFAB(ThemeService theme) {
    return ScaleTransition(
      scale: _fabController,
      child: Container(
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          gradient: theme.primaryGradient,
          boxShadow: theme.neonGlow,
        ),
        child: FloatingActionButton(
          onPressed: () => _showQuickActions(),
          backgroundColor: Colors.transparent,
          elevation: 0,
          child: Icon(
            Icons.add,
            color: Colors.white,
            size: 28,
          ),
        ),
      ),
    );
  }

  Widget _buildModernDrawer(ThemeService theme) {
    return Drawer(
      backgroundColor: Colors.transparent,
      child: Container(
        decoration: BoxDecoration(
          gradient: theme.backgroundGradient,
        ),
        child: ListView(
          padding: EdgeInsets.zero,
          children: [
            DrawerHeader(
              decoration: BoxDecoration(
                gradient: theme.primaryGradient,
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Container(
                    padding: const EdgeInsets.all(8),
                    decoration: BoxDecoration(
                      color: Colors.white.withOpacity(0.2),
                      borderRadius: theme.borderRadiusSmall,
                    ),
                    child: Icon(Icons.smart_toy, size: 40, color: Colors.white),
                  ),
                  const SizedBox(height: 12),
                  Text(
                    'AGV Fleet Manager',
                    style: theme.headlineLarge.copyWith(color: Colors.white),
                  ),
                  Text(
                    _connectionStatus,
                    style: theme.bodySmall.copyWith(color: Colors.white70),
                  ),
                ],
              ),
            ),
            _buildDrawerItem(
              icon: Icons.dashboard,
              title: 'Dashboard',
              isSelected: _currentIndex == 0,
              onTap: () => _setIndex(0),
              theme: theme,
            ),
            _buildDrawerItem(
              icon: Icons.devices,
              title: 'Device Management',
              isSelected: _currentIndex == 1,
              onTap: () => _setIndex(1),
              theme: theme,
            ),
            _buildDrawerItem(
              icon: Icons.analytics,
              title: 'Analytics',
              isSelected: _currentIndex == 2,
              onTap: () => _setIndex(2),
              theme: theme,
            ),
            _buildDrawerItem(
              icon: Icons.settings,
              title: 'Settings',
              isSelected: _currentIndex == 3,
              onTap: () => _setIndex(3),
              theme: theme,
            ),
            const Divider(),
            _buildDrawerItem(
              icon: Icons.logout,
              title: 'Logout',
              onTap: () => _logout(),
              theme: theme,
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildDrawerItem({
    required IconData icon,
    required String title,
    required VoidCallback onTap,
    required ThemeService theme,
    bool isSelected = false,
  }) {
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 12, vertical: 4),
      decoration: BoxDecoration(
        borderRadius: theme.borderRadiusSmall,
        color: isSelected ? theme.accentColor.withOpacity(0.1) : null,
        border: isSelected
            ? Border.all(color: theme.accentColor.withOpacity(0.3))
            : null,
      ),
      child: ListTile(
        leading: Icon(
          icon,
          color: isSelected ? theme.accentColor : null,
        ),
        title: Text(
          title,
          style: TextStyle(
            color: isSelected ? theme.accentColor : null,
            fontWeight: isSelected ? FontWeight.w600 : null,
          ),
        ),
        onTap: () {
          Navigator.pop(context);
          onTap();
        },
      ),
    );
  }

  void _setIndex(int index) {
    setState(() => _currentIndex = index);
  }

  void _showQuickActions() {
    // Show quick action bottom sheet
    showModalBottomSheet(
      context: context,
      backgroundColor: Colors.transparent,
      builder: (context) => Container(
        decoration: BoxDecoration(
          gradient: Provider.of<ThemeService>(context).backgroundGradient,
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
                'Quick Actions',
                style: Provider.of<ThemeService>(context).headlineLarge,
              ),
              const SizedBox(height: 20),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: [
                  _buildQuickActionButton(
                    Icons.add_task,
                    'Create\nOrder',
                    () => Navigator.pop(context),
                  ),
                  _buildQuickActionButton(
                    Icons.map,
                    'Edit\nMap',
                    () => Navigator.pop(context),
                  ),
                  _buildQuickActionButton(
                    Icons.emergency,
                    'Emergency\nStop',
                    () => Navigator.pop(context),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildQuickActionButton(
      IconData icon, String label, VoidCallback onTap) {
    final theme = Provider.of<ThemeService>(context);
    return ModernGlassCard(
      onTap: onTap,
      child: Column(
        children: [
          Icon(icon, color: theme.accentColor, size: 32),
          const SizedBox(height: 8),
          Text(
            label,
            style: theme.bodySmall,
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }

  void _showNotifications() {
    // Implement notifications
  }

  void _logout() {
    Navigator.pushNamedAndRemoveUntil(context, '/login', (route) => false);
  }

  @override
  void dispose() {
    _fabController.dispose();
    _webSocketService.dispose();
    super.dispose();
  }
}
