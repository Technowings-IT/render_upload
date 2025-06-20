//main.dart - Updated with Proper Navigation and Integration
import 'package:flutter/material.dart';
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

void main() {
  runApp(AGVFleetManagementApp());
}

class AGVFleetManagementApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'AGV Fleet Management',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        visualDensity: VisualDensity.adaptivePlatformDensity,
        appBarTheme: AppBarTheme(
          backgroundColor: Colors.blue[700],
          foregroundColor: Colors.white,
          elevation: 4,
        ),
        cardTheme: CardThemeData(
          elevation: 2,
          margin: EdgeInsets.symmetric(vertical: 4),
        ),
        elevatedButtonTheme: ElevatedButtonThemeData(
          style: ElevatedButton.styleFrom(
            padding: EdgeInsets.symmetric(horizontal: 24, vertical: 12),
          ),
        ),
      ),
      darkTheme: ThemeData.dark().copyWith(
        primaryColor: Colors.blue[700],
        appBarTheme: AppBarTheme(
          backgroundColor: Colors.grey[900],
          foregroundColor: Colors.white,
        ),
      ),
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
        // Handle dynamic routes
        switch (settings.name) {
          case '/control':
            final args = settings.arguments as Map<String, dynamic>?;
            return MaterialPageRoute(
              builder: (context) => ControlPage(
                deviceId: args?['deviceId'] ?? '',
                deviceName: args?['deviceName'] ?? 'Unknown Device',
              ),
            );
          case '/map':
            final args = settings.arguments as Map<String, dynamic>?;
            return MaterialPageRoute(
              builder: (context) => MapPage(
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
  }
}

class MainNavigationWrapper extends StatefulWidget {
  final int initialIndex;

  const MainNavigationWrapper({Key? key, this.initialIndex = 0}) : super(key: key);

  @override
  _MainNavigationWrapperState createState() => _MainNavigationWrapperState();
}

class _MainNavigationWrapperState extends State<MainNavigationWrapper> {
  late int _currentIndex;
  final WebSocketService _webSocketService = WebSocketService();
  final ApiService _apiService = ApiService();
  bool _isConnectedToServer = false;
  bool _isInitializing = true;

  final List<Widget> _screens = [
    DashboardScreen(),
    ConnectScreen(),
    AnalyticsScreen(),
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
    try {
      // Test API connection first
      print('🔧 Testing API connection...');
      final apiHealthy = await _apiService.testConnection();
      
      if (apiHealthy) {
        print('✅ API connection successful');
        
        // Initialize WebSocket connection
        print('🔧 Initializing WebSocket...');
        final wsUrl = _apiService.getWebSocketUrl();
        final wsConnected = await _webSocketService.connect(wsUrl);
        
        setState(() {
          _isConnectedToServer = wsConnected;
          _isInitializing = false;
        });
        
        if (wsConnected) {
          print('✅ WebSocket connected successfully');
          _subscribeToConnectionState();
        } else {
          print('⚠️ WebSocket connection failed, but API is available');
        }
      } else {
        print('❌ API connection failed');
        setState(() {
          _isConnectedToServer = false;
          _isInitializing = false;
        });
        _showConnectionError();
      }
    } catch (e) {
      print('❌ Service initialization failed: $e');
      setState(() {
        _isConnectedToServer = false;
        _isInitializing = false;
      });
      _showConnectionError();
    }
  }

  void _subscribeToConnectionState() {
    _webSocketService.connectionState.listen((connected) {
      setState(() {
        _isConnectedToServer = connected;
      });
      
      if (!connected) {
        _showConnectionLostSnackBar();
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
          title: Text('Connection Error'),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(Icons.wifi_off, size: 64, color: Colors.red),
              SizedBox(height: 16),
              Text('Unable to connect to the AGV fleet server.'),
              SizedBox(height: 8),
              Text('Please check:'),
              Text('• Server is running on ${_apiService.baseUrl}'),
              Text('• Network connectivity'),
              Text('• Firewall settings'),
            ],
          ),
          actions: [
            TextButton(
              onPressed: () {
                Navigator.of(context).pop();
                _initializeServices(); // Retry
              },
              child: Text('Retry'),
            ),
            ElevatedButton(
              onPressed: () {
                Navigator.of(context).pop();
                // Continue in offline mode
              },
              child: Text('Continue Offline'),
            ),
          ],
        ),
      );
    }
  }

  void _showConnectionLostSnackBar() {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Row(
            children: [
              Icon(Icons.wifi_off, color: Colors.white),
              SizedBox(width: 8),
              Text('Connection lost. Attempting to reconnect...'),
            ],
          ),
          backgroundColor: Colors.orange,
          duration: Duration(seconds: 3),
        ),
      );
    }
  }

  void _showErrorSnackBar(String message) {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(message),
          backgroundColor: Colors.red,
          duration: Duration(seconds: 3),
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
              CircularProgressIndicator(),
              SizedBox(height: 16),
              Text('Initializing AGV Fleet Management...'),
              SizedBox(height: 8),
              Text('Connecting to server...'),
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
              color: Theme.of(context).primaryColor,
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
                    Container(
                      width: 8,
                      height: 8,
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        color: _isConnectedToServer ? Colors.green : Colors.red,
                      ),
                    ),
                    SizedBox(width: 8),
                    Text(
                      _isConnectedToServer ? 'Connected' : 'Disconnected',
                      style: TextStyle(
                        color: Colors.white70,
                        fontSize: 14,
                      ),
                    ),
                  ],
                ),
                if (!_isConnectedToServer) ...[
                  SizedBox(height: 4),
                  Text(
                    'Some features may be limited',
                    style: TextStyle(
                      color: Colors.orange[200],
                      fontSize: 12,
                    ),
                  ),
                ],
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
            subtitle: Text(_isConnectedToServer ? 'Connected' : 'Disconnected'),
            trailing: Icon(
              _isConnectedToServer ? Icons.check_circle : Icons.error,
              color: _isConnectedToServer ? Colors.green : Colors.red,
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
        title: Text('Connection Status'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            _buildConnectionItem('API Server', _apiService.baseUrl),
            _buildConnectionItem('WebSocket', connectionInfo['serverUrl'] ?? 'Not set'),
            _buildConnectionItem('Client ID', connectionInfo['clientId'] ?? 'Not assigned'),
            _buildConnectionItem('Status', _isConnectedToServer ? 'Connected' : 'Disconnected'),
            if (!_isConnectedToServer)
              _buildConnectionItem('Reconnect Attempts', '${connectionInfo['reconnectAttempts']}'),
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
                _initializeServices();
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
            width: 100,
            child: Text(
              '$label:',
              style: TextStyle(fontWeight: FontWeight.w500),
            ),
          ),
          Expanded(
            child: Text(
              value,
              style: TextStyle(fontFamily: 'monospace'),
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
              Text('AGV Fleet Management System'),
              SizedBox(height: 16),
              Text('Quick Start:', style: TextStyle(fontWeight: FontWeight.bold)),
              Text('1. Connect your AGV devices in the Devices tab'),
              Text('2. Use the Control interface to manually operate AGVs'),
              Text('3. Create maps using the Map Editor'),
              Text('4. Set up automated orders and routes'),
              Text('5. Monitor performance in the Analytics tab'),
              SizedBox(height: 16),
              Text('System Requirements:', style: TextStyle(fontWeight: FontWeight.bold)),
              Text('• ROS2 Jazzy installed on AGV'),
              Text('• Network connectivity (WiFi/Ethernet)'),
              Text('• Backend server running on port 3000'),
              SizedBox(height: 16),
              Text('Troubleshooting:', style: TextStyle(fontWeight: FontWeight.bold)),
              Text('• Check server IP: 192.168.253.79:3000'),
              Text('• Verify AGV IP: 192.168.253.79'),
              Text('• Ensure ROS2 topics are publishing'),
              SizedBox(height: 16),
              Text('For technical support, contact:'),
              Text('Email: support@agvfleet.com'),
              Text('Phone: +1-555-0123'),
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
              // In a real app, this would open a support ticket system
              ScaffoldMessenger.of(context).showSnackBar(
                SnackBar(
                  content: Text('Support system not implemented in demo'),
                  backgroundColor: Colors.orange,
                ),
              );
            },
            child: Text('Contact Support'),
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