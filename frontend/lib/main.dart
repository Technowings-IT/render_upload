// main.dart - Fixed with proper configuration and integration
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'services/theme_service.dart';
import '../services/smart_connection_service.dart';
import 'screens/splash_screen.dart';
import 'screens/login_screen.dart';
import 'screens/dashboard_screen.dart'; // Make sure this file exports DashboardScreen
import 'screens/connect_screen.dart';
import 'screens/control_page.dart';
import 'screens/map_page.dart';
import 'screens/analytics_screen.dart';
import 'screens/settings_screen.dart';
import 'screens/profile_screen.dart';
import 'services/web_socket_service.dart';
import 'services/api_service.dart';

// Configuration constants - Update these for your setup
class AppConfig {
  // Your AGV backend server configuration
  static const String DEFAULT_SERVER_IP = '192.168.0.156'; // Your AGV IP
  static const int DEFAULT_SERVER_PORT = 3000; // Backend port
  static const int AGV_SSH_PORT = 22; // AGV SSH port (mentioned by user)

  // Constructed URLs
  static String get serverUrl =>
      'http://$DEFAULT_SERVER_IP:$DEFAULT_SERVER_PORT';
  static String get websocketUrl =>
      'ws://$DEFAULT_SERVER_IP:$DEFAULT_SERVER_PORT';

  // Timeouts and retries
  static const Duration connectionTimeout = Duration(seconds: 15);
  static const Duration apiTimeout = Duration(seconds: 10);
  static const int maxRetryAttempts = 3;
}

void main() async {
  WidgetsFlutterBinding.ensureInitialized();

  print('🚀 Starting AGV Fleet Management App...');
  print('📡 Server: ${AppConfig.serverUrl}');
  print('🔌 WebSocket: ${AppConfig.websocketUrl}');
  print('🔧 AGV SSH Port: ${AppConfig.AGV_SSH_PORT}');

  // Initialize theme service
  final themeService = ThemeService();
  await themeService.loadTheme();

  // Initialize API service with configuration
  ApiService().initialize(AppConfig.serverUrl);

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
            '/splash': (context) => SplashScreen(),
            '/login': (context) => LoginScreen(),
            '/': (context) => MainNavigationWrapper(),
            '/dashboard': (context) => MainNavigationWrapper(initialIndex: 0),
            '/connect': (context) => MainNavigationWrapper(initialIndex: 1),
            '/analytics': (context) => MainNavigationWrapper(initialIndex: 2),
            '/settings': (context) => MainNavigationWrapper(initialIndex: 3),
          },
          onGenerateRoute: (settings) {
            switch (settings.name) {
              case '/control':
                final args = settings.arguments as Map<String, dynamic>?;
                return MaterialPageRoute(
                  builder: (context) => ControlPage(
                    deviceId: args?['deviceId'] ?? 'agv_01',
                  ),
                );
              case '/map':
                final args = settings.arguments as Map<String, dynamic>?;
                return MaterialPageRoute(
                  builder: (context) => EnhancedMapPage(
                    deviceId: args?['deviceId'],
                  ),
                );
              default:
                return MaterialPageRoute(
                  builder: (context) => MainNavigationWrapper(),
                );
            }
          },
        );
      },
    );
  }
}

class MainNavigationWrapper extends StatefulWidget {
  final int initialIndex;

  const MainNavigationWrapper({Key? key, this.initialIndex = 0})
      : super(key: key);

  @override
  _MainNavigationWrapperState createState() => _MainNavigationWrapperState();
}

class _MainNavigationWrapperState extends State<MainNavigationWrapper> {
  late int _currentIndex;
  final WebSocketService _webSocketService = WebSocketService();
  final ApiService _apiService = ApiService();
  final SmartConnectionService smartConnection = SmartConnectionService();

  // Connection states
  bool _isConnectedToServer = false;
  bool _isWebSocketConnected = false;
  bool _isInitializing = true;
  String _connectionStatus = 'Connecting...';
  int _retryAttempts = 0;

  final List<Widget> _screens = [
    DashboardScreen(),
    ConnectScreen(), // Note: Using standard ConnectScreen, not EnhancedConnectScreen
    EnhancedAnalyticsScreen(),
    SettingsScreen(),
  ];

  final List<BottomNavigationBarItem> _navItems = [
    BottomNavigationBarItem(
      icon: Icon(Icons.dashboard),
      label: 'Dashboard',
    ),
    BottomNavigationBarItem(
      icon: Icon(Icons.devices),
      label: 'Devices',
    ),
    BottomNavigationBarItem(
      icon: Icon(Icons.analytics),
      label: 'Analytics',
    ),
    BottomNavigationBarItem(
      icon: Icon(Icons.settings),
      label: 'Settings',
    ),
  ];

  @override
  void initState() {
    super.initState();
    _currentIndex = widget.initialIndex;
    _initializeServices();
  }

  void _initializeServices() async {
    setState(() {
      _isInitializing = true;
      _connectionStatus = 'Connecting to AGV server...';
    });

    await _connectWithRetry();
  }

  Future<void> _connectWithRetry() async {
    for (_retryAttempts = 1;
        _retryAttempts <= AppConfig.maxRetryAttempts;
        _retryAttempts++) {
      try {
        setState(() {
          _connectionStatus =
              'Attempting connection ${_retryAttempts}/${AppConfig.maxRetryAttempts}...';
        });

        print(
            '🔧 Connection attempt $_retryAttempts/${AppConfig.maxRetryAttempts}');

        // Test API connection first
        print('📡 Testing API connection to ${_apiService.baseUrl}...');
        final apiHealthy = await _apiService
            .testConnection()
            .timeout(AppConfig.connectionTimeout);

        if (apiHealthy) {
          print('✅ API connection successful');
          setState(() {
            _isConnectedToServer = true;
            _connectionStatus = 'API connected, initializing WebSocket...';
          });

          // Initialize WebSocket connection
          print(
              '🔌 Connecting WebSocket to ${_apiService.getWebSocketUrl()}...');
          final wsConnected = await _webSocketService
              .connect(_apiService.getWebSocketUrl() ?? '');

          setState(() {
            _isWebSocketConnected = wsConnected;
            _isInitializing = false;
            _connectionStatus =
                wsConnected ? 'Fully connected' : 'API only (limited features)';
          });

          if (wsConnected) {
            print('✅ WebSocket connected successfully');
            _subscribeToConnectionState();
            _tryAutoConnectAGV();
          } else {
            print('⚠️ WebSocket connection failed, but API is available');
            _showWebSocketWarning();
          }

          return; // Success, exit retry loop
        } else {
          throw Exception('API health check failed');
        }
      } catch (e) {
        print('❌ Connection attempt $_retryAttempts failed: $e');

        if (_retryAttempts < AppConfig.maxRetryAttempts) {
          setState(() {
            _connectionStatus = 'Retrying in 3 seconds...';
          });
          await Future.delayed(Duration(seconds: 3));
        } else {
          print('❌ All connection attempts failed');
          setState(() {
            _isConnectedToServer = false;
            _isWebSocketConnected = false;
            _isInitializing = false;
            _connectionStatus = 'Connection failed';
          });
          _showConnectionError();
        }
      }
    }
  }

  void _tryAutoConnectAGV() async {
    try {
      print('🤖 Attempting to auto-connect AGV...');
      final result = await _apiService.autoConnectAGV();
      if (result['success'] == true) {
        print('✅ AGV auto-connected: ${result['deviceId']}');
        _showSuccessSnackBar('AGV connected successfully');
      }
    } catch (e) {
      print('⚠️ AGV auto-connect failed: $e');
      // Don't show error - this is optional
    }
  }

  void _subscribeToConnectionState() {
    _webSocketService.connectionState.listen((connected) {
      setState(() {
        _isWebSocketConnected = connected;
        _connectionStatus =
            connected ? 'Fully connected' : 'WebSocket disconnected';
      });

      if (!connected) {
        _showConnectionLostSnackBar();
      } else {
        _showSuccessSnackBar('WebSocket reconnected');
      }
    });

    // Listen to WebSocket errors
    _webSocketService.errors.listen((error) {
      _showErrorSnackBar('WebSocket Error: $error');
    });
  }

  void _showConnectionError() {
    if (mounted) {
      showDialog(
        context: context,
        barrierDismissible: false,
        builder: (context) => AlertDialog(
          title: Row(
            children: [
              Icon(Icons.wifi_off, color: Colors.red),
              SizedBox(width: 8),
              Text('Connection Error'),
            ],
          ),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text('Unable to connect to the AGV fleet server.'),
              SizedBox(height: 16),
              Text('Configuration:',
                  style: TextStyle(fontWeight: FontWeight.bold)),
              Text('• Server: ${AppConfig.serverUrl}'),
              Text('• AGV IP: ${AppConfig.DEFAULT_SERVER_IP}'),
              Text('• AGV SSH Port: ${AppConfig.AGV_SSH_PORT}'),
              SizedBox(height: 16),
              Text('Please check:',
                  style: TextStyle(fontWeight: FontWeight.bold)),
              Text('• AGV backend server is running'),
              Text('• Network connectivity to AGV'),
              Text('• Firewall settings'),
              Text('• IP address configuration'),
            ],
          ),
          actions: [
            TextButton(
              onPressed: () {
                Navigator.of(context).pop();
                _connectWithRetry(); // Retry
              },
              child: Text('Retry'),
            ),
            ElevatedButton(
              onPressed: () {
                Navigator.of(context).pop();
                // Continue in offline mode
                setState(() {
                  _connectionStatus = 'Offline mode';
                });
              },
              child: Text('Continue Offline'),
            ),
          ],
        ),
      );
    }
  }

  void _showWebSocketWarning() {
    _showWarningSnackBar(
        'WebSocket failed. Real-time features will be limited.');
  }

  void _showConnectionLostSnackBar() {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Row(
            children: [
              SizedBox(
                width: 16,
                height: 16,
                child: CircularProgressIndicator(
                  strokeWidth: 2,
                  valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
                ),
              ),
              SizedBox(width: 12),
              Text('Connection lost. Attempting to reconnect...'),
            ],
          ),
          backgroundColor: Colors.orange,
          duration: Duration(seconds: 4),
        ),
      );
    }
  }

  void _showErrorSnackBar(String message) {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Row(
            children: [
              Icon(Icons.error, color: Colors.white, size: 20),
              SizedBox(width: 8),
              Expanded(child: Text(message)),
            ],
          ),
          backgroundColor: Colors.red,
          duration: Duration(seconds: 4),
        ),
      );
    }
  }

  void _showWarningSnackBar(String message) {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Row(
            children: [
              Icon(Icons.warning, color: Colors.white, size: 20),
              SizedBox(width: 8),
              Expanded(child: Text(message)),
            ],
          ),
          backgroundColor: Colors.orange,
          duration: Duration(seconds: 3),
        ),
      );
    }
  }

  void _showSuccessSnackBar(String message) {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Row(
            children: [
              Icon(Icons.check_circle, color: Colors.white, size: 20),
              SizedBox(width: 8),
              Text(message),
            ],
          ),
          backgroundColor: Colors.green,
          duration: Duration(seconds: 2),
        ),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    if (_isInitializing) {
      return Scaffold(
        body: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Container(
                width: 100,
                height: 100,
                child: Stack(
                  alignment: Alignment.center,
                  children: [
                    CircularProgressIndicator(strokeWidth: 3),
                    Icon(Icons.smart_toy, size: 40, color: Colors.blue),
                  ],
                ),
              ),
              SizedBox(height: 24),
              Text(
                'AGV Fleet Management',
                style: TextStyle(
                  fontSize: 24,
                  fontWeight: FontWeight.bold,
                ),
              ),
              SizedBox(height: 8),
              Text(
                _connectionStatus,
                style: TextStyle(
                  fontSize: 16,
                  color: Colors.grey[600],
                ),
              ),
              SizedBox(height: 16),
              if (_retryAttempts > 1)
                Text(
                  'Attempt $_retryAttempts/${AppConfig.maxRetryAttempts}',
                  style: TextStyle(
                    fontSize: 14,
                    color: Colors.grey[500],
                  ),
                ),
              SizedBox(height: 24),
              Text(
                'Connecting to: ${AppConfig.serverUrl}',
                style: TextStyle(
                  fontSize: 12,
                  fontFamily: 'monospace',
                  color: Colors.grey[500],
                ),
              ),
            ],
          ),
        ),
      );
    }

    return Scaffold(
      body: IndexedStack(
        index: _currentIndex,
        children: _screens,
      ),
      bottomNavigationBar: BottomNavigationBar(
        type: BottomNavigationBarType.fixed,
        currentIndex: _currentIndex,
        onTap: (index) {
          setState(() {
            _currentIndex = index;
          });
        },
        items: _navItems,
      ),
      drawer: _buildNavigationDrawer(),
    );
  }

  Widget _buildNavigationDrawer() {
    return Drawer(
      child: ListView(
        padding: EdgeInsets.zero,
        children: [
          DrawerHeader(
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
                colors: [Colors.blue[700]!, Colors.blue[500]!],
              ),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Icon(
                  Icons.smart_toy,
                  size: 48,
                  color: Colors.white,
                ),
                SizedBox(height: 8),
                Text(
                  'AGV Fleet Management',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                SizedBox(height: 4),
                Row(
                  children: [
                    // API Status
                    Container(
                      width: 8,
                      height: 8,
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        color: _isConnectedToServer ? Colors.green : Colors.red,
                      ),
                    ),
                    SizedBox(width: 4),
                    Text('API',
                        style: TextStyle(color: Colors.white70, fontSize: 12)),
                    SizedBox(width: 12),
                    // WebSocket Status
                    Container(
                      width: 8,
                      height: 8,
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        color:
                            _isWebSocketConnected ? Colors.green : Colors.red,
                      ),
                    ),
                    SizedBox(width: 4),
                    Text('WS',
                        style: TextStyle(color: Colors.white70, fontSize: 12)),
                  ],
                ),
                SizedBox(height: 4),
                Text(
                  _connectionStatus,
                  style: TextStyle(
                    color: Colors.white70,
                    fontSize: 12,
                  ),
                ),
              ],
            ),
          ),
          ListTile(
            leading: Icon(Icons.dashboard),
            title: Text('Dashboard'),
            selected: _currentIndex == 0,
            onTap: () {
              setState(() {
                _currentIndex = 0;
              });
              Navigator.pop(context);
            },
          ),
          ListTile(
            leading: Icon(Icons.devices),
            title: Text('Device Management'),
            selected: _currentIndex == 1,
            onTap: () {
              setState(() {
                _currentIndex = 1;
              });
              Navigator.pop(context);
            },
          ),
          ListTile(
            leading: Icon(Icons.map),
            title: Text('Map Editor'),
            onTap: () {
              Navigator.pop(context);
              Navigator.pushNamed(context, '/map');
            },
          ),
          ListTile(
            leading: Icon(Icons.analytics),
            title: Text('Analytics'),
            selected: _currentIndex == 2,
            onTap: () {
              setState(() {
                _currentIndex = 2;
              });
              Navigator.pop(context);
            },
          ),
          Divider(),
          ListTile(
            leading: Icon(Icons.settings),
            title: Text('Settings'),
            selected: _currentIndex == 3,
            onTap: () {
              setState(() {
                _currentIndex = 3;
              });
              Navigator.pop(context);
            },
          ),
          ListTile(
            leading: Icon(Icons.person),
            title: Text('Profile'),
            onTap: () {
              Navigator.pop(context);
              Navigator.push(
                context,
                MaterialPageRoute(builder: (context) => ProfileScreen()),
              );
            },
          ),
          ListTile(
            leading: Icon(Icons.wifi),
            title: Text('Connection Status'),
            subtitle: Text(_connectionStatus),
            trailing: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                if (_isConnectedToServer)
                  Icon(Icons.check_circle, color: Colors.green, size: 16),
                if (_isWebSocketConnected)
                  Icon(Icons.wifi, color: Colors.green, size: 16)
                else
                  Icon(Icons.wifi_off, color: Colors.orange, size: 16),
              ],
            ),
            onTap: () {
              Navigator.pop(context);
              _showConnectionDialog();
            },
          ),
          ListTile(
            leading: Icon(Icons.help),
            title: Text('Help & Support'),
            onTap: () {
              Navigator.pop(context);
              _showHelpDialog();
            },
          ),
          Divider(),
          ListTile(
            leading: Icon(Icons.logout),
            title: Text('Logout'),
            onTap: () {
              Navigator.pop(context);
              _showLogoutDialog();
            },
          ),
        ],
      ),
    );
  }

  void _showConnectionDialog() {
    final connectionInfo = _webSocketService.getConnectionInfo();

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Connection Details'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            _buildConnectionItem(
                'API Server', _apiService.baseUrl ?? 'Unknown'),
            _buildConnectionItem(
                'WebSocket', _apiService.getWebSocketUrl() ?? ''),
            _buildConnectionItem('AGV SSH Port', '${AppConfig.AGV_SSH_PORT}'),
            _buildConnectionItem(
                'Client ID', connectionInfo['clientId'] ?? 'Not assigned'),
            _buildConnectionItem('API Status',
                _isConnectedToServer ? 'Connected' : 'Disconnected'),
            _buildConnectionItem('WebSocket Status',
                _isWebSocketConnected ? 'Connected' : 'Disconnected'),
            if (!_isConnectedToServer)
              _buildConnectionItem('Last Error', 'Connection timeout'),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Close'),
          ),
          if (!_isConnectedToServer)
            ElevatedButton(
              onPressed: () {
                Navigator.of(context).pop();
                _connectWithRetry();
              },
              child: Text('Reconnect'),
            ),
        ],
      ),
    );
  }

  Widget _buildConnectionItem(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 4),
      child: Row(
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
              style: TextStyle(fontFamily: 'monospace', fontSize: 12),
            ),
          ),
        ],
      ),
    );
  }

  void _showHelpDialog() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Help & Support'),
        content: SingleChildScrollView(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text('AGV Fleet Management System',
                  style: TextStyle(fontWeight: FontWeight.bold)),
              SizedBox(height: 16),
              Text('Current Configuration:',
                  style: TextStyle(fontWeight: FontWeight.bold)),
              Text('• AGV IP: ${AppConfig.DEFAULT_SERVER_IP}'),
              Text('• Backend Port: ${AppConfig.DEFAULT_SERVER_PORT}'),
              Text('• SSH Port: ${AppConfig.AGV_SSH_PORT}'),
              SizedBox(height: 16),
              Text('Quick Start:',
                  style: TextStyle(fontWeight: FontWeight.bold)),
              Text('1. Ensure AGV backend is running'),
              Text('2. Connect your AGV devices'),
              Text('3. Use joystick control for manual operation'),
              Text('4. Create maps using SLAM mapping'),
              Text('5. Set up automated orders and routes'),
              SizedBox(height: 16),
              Text('Troubleshooting:',
                  style: TextStyle(fontWeight: FontWeight.bold)),
              Text('• Check AGV backend: curl ${AppConfig.serverUrl}/health'),
              Text('• Verify ROS2 topics: ros2 topic list'),
              Text('• Test SSH: ssh user@${AppConfig.DEFAULT_SERVER_IP}'),
              SizedBox(height: 16),
              Text('For technical support:',
                  style: TextStyle(fontWeight: FontWeight.bold)),
              Text('• Check ROS2 logs on AGV'),
              Text('• Verify network connectivity'),
              Text('• Ensure all required ROS2 nodes are running'),
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
  }

  void _showLogoutDialog() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Logout'),
        content: Text('Are you sure you want to logout?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              _webSocketService.disconnect();
              Navigator.of(context).pushNamedAndRemoveUntil(
                '/login',
                (route) => false,
              );
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child: Text('Logout'),
          ),
        ],
      ),
    );
  }

  @override
  void dispose() {
    _webSocketService.dispose();
    super.dispose();
  }
}
