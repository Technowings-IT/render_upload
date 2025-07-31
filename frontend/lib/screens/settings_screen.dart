// screens/settings_screen.dart
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import '../services/theme_service.dart';

class SettingsScreen extends StatefulWidget {
  @override
  _SettingsScreenState createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen>
    with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();

  // Server settings
  final _serverUrlController = TextEditingController();
  final _websocketUrlController = TextEditingController();

  // AGV settings
  double _maxLinearSpeed = 1.0;
  double _maxAngularSpeed = 2.0;
  double _batteryLowThreshold = 20.0;
  int _connectionTimeout = 30;
  bool _autoReconnect = true;
  bool _debugMode = false;

  // UI settings
  bool _showNotifications = true;
  bool _soundEnabled = true;
  String _selectedLanguage = 'English';
  double _joystickSize = 200.0;

  // Map settings
  double _mapRefreshRate = 1.0;
  bool _showGrid = true;
  bool _showRobotTrail = true;
  int _maxTrailLength = 100;

  bool _isLoading = false;

  // Animation controllers
  late AnimationController _slideController;
  late AnimationController _fadeController;
  late Animation<Offset> _slideAnimation;
  late Animation<double> _fadeAnimation;
  bool _animationsInitialized = false;

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _loadSettings();
  }

  @override
  void dispose() {
    _serverUrlController.dispose();
    _websocketUrlController.dispose();
    if (_animationsInitialized) {
      _slideController.dispose();
      _fadeController.dispose();
    }
    super.dispose();
  }

  void _initializeAnimations() {
    _slideController = AnimationController(
      duration: Duration(milliseconds: 800),
      vsync: this,
    );

    _fadeController = AnimationController(
      duration: Duration(milliseconds: 600),
      vsync: this,
    );

    _slideAnimation = Tween<Offset>(
      begin: Offset(0, 0.1),
      end: Offset.zero,
    ).animate(CurvedAnimation(parent: _slideController, curve: Curves.easeOut));

    _fadeAnimation = Tween<double>(begin: 0.0, end: 1.0).animate(
      CurvedAnimation(parent: _fadeController, curve: Curves.easeIn),
    );

    _animationsInitialized = true;

    // Start animations after a frame
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (mounted && _animationsInitialized) {
        _slideController.forward();
        _fadeController.forward();
      }
    });
  }

  void _loadSettings() {
    // Load current settings from API service
    setState(() {
      _serverUrlController.text = _apiService.baseUrl ?? '';
      _websocketUrlController.text = _apiService.getWebSocketUrl() ?? '';
      _isLoading = false;
    });
  }

  void _saveSettings() async {
    setState(() {
      _isLoading = true;
    });

    try {
      // Update API service configuration
      _apiService.setBaseUrl(_serverUrlController.text);

      // In a real app, save to SharedPreferences or local storage
      await Future.delayed(Duration(seconds: 1)); // Simulate save delay

      _showEnhancedSnackBar(
        'Settings saved successfully',
        Colors.green,
        Icons.check_circle,
      );
    } catch (e) {
      _showEnhancedSnackBar(
        'Failed to save settings: $e',
        Colors.red,
        Icons.error,
      );
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  void _resetSettings() {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
        title: Row(
          children: [
            Container(
              padding: EdgeInsets.all(8),
              decoration: BoxDecoration(
                color: Colors.orange,
                borderRadius: BorderRadius.circular(12),
              ),
              child: Icon(Icons.refresh, color: Colors.white, size: 24),
            ),
            SizedBox(width: 12),
            Text(
              'Reset Settings',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
          ],
        ),
        content: Text(
          'Are you sure you want to reset all settings to default values?',
          style: TextStyle(fontSize: 16),
        ),
        backgroundColor: isDark ? theme.cardColor : null,
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _performReset();
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.orange,
              foregroundColor: Colors.white,
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(12),
              ),
            ),
            child: Text('Reset'),
          ),
        ],
      ),
    );
  }

  void _performReset() {
    setState(() {
      _serverUrlController.text = 'http://192.168.0.63:3000';
      _websocketUrlController.text = 'ws://192.168.0.63:3000';
      _maxLinearSpeed = 1.0;
      _maxAngularSpeed = 2.0;
      _batteryLowThreshold = 20.0;
      _connectionTimeout = 30;
      _autoReconnect = true;
      _debugMode = false;
      _showNotifications = true;
      _soundEnabled = true;
      _selectedLanguage = 'English';
      _joystickSize = 200.0;
      _mapRefreshRate = 1.0;
      _showGrid = true;
      _showRobotTrail = true;
      _maxTrailLength = 100;
    });

    _showEnhancedSnackBar(
      'Settings reset to defaults',
      Colors.blue,
      Icons.restore,
    );
  }

  void _showEnhancedSnackBar(String message, Color color, IconData icon) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Row(
          children: [
            Icon(icon, color: Colors.white),
            SizedBox(width: 12),
            Expanded(child: Text(message)),
          ],
        ),
        backgroundColor: color,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
        duration: Duration(seconds: 3),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Consumer<ThemeService>(
      builder: (context, themeService, child) {
        return Scaffold(
          appBar: AppBar(
            title: Text(
              'Settings',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            backgroundColor: Colors.blue,
            foregroundColor: Colors.white,
            elevation: 0,
            flexibleSpace: Container(
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [Colors.blue.shade600, Colors.blue.shade400],
                  begin: Alignment.topLeft,
                  end: Alignment.bottomRight,
                ),
              ),
            ),
            actions: [
              IconButton(
                icon: Icon(Icons.refresh),
                onPressed: _resetSettings,
                tooltip: 'Reset to Defaults',
              ),
              IconButton(
                icon: Icon(Icons.save),
                onPressed: _isLoading ? null : _saveSettings,
                tooltip: 'Save Settings',
              ),
            ],
          ),
          body: _isLoading
              ? Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      CircularProgressIndicator(
                        valueColor: AlwaysStoppedAnimation<Color>(Colors.blue),
                      ),
                      SizedBox(height: 16),
                      Text(
                        'Saving settings...',
                        style: TextStyle(
                          fontSize: 16,
                          fontWeight: FontWeight.w500,
                          color: isDark ? Colors.grey[400] : Colors.grey[600],
                        ),
                      ),
                    ],
                  ),
                )
              : Container(
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      begin: Alignment.topCenter,
                      end: Alignment.bottomCenter,
                      colors: isDark
                          ? [theme.scaffoldBackgroundColor, theme.cardColor]
                          : [Colors.grey.shade50, Colors.white],
                    ),
                  ),
                  child: _animationsInitialized
                      ? AnimatedBuilder(
                          animation: _fadeController,
                          builder: (context, child) {
                            return FadeTransition(
                              opacity: _fadeAnimation,
                              child: SlideTransition(
                                position: _slideAnimation,
                                child: SingleChildScrollView(
                                  padding: EdgeInsets.all(16),
                                  child: Column(
                                    crossAxisAlignment:
                                        CrossAxisAlignment.start,
                                    children: [
                                      _buildEnhancedServerSettingsCard(),
                                      SizedBox(height: 20),
                                      _buildEnhancedAGVSettingsCard(),
                                      SizedBox(height: 20),
                                      _buildEnhancedUISettingsCard(
                                          themeService),
                                      SizedBox(height: 20),
                                      _buildEnhancedMapSettingsCard(),
                                      SizedBox(height: 20),
                                      _buildEnhancedSystemInfoCard(),
                                      SizedBox(height: 20),
                                      _buildEnhancedActionButtons(),
                                      SizedBox(height: 20),
                                    ],
                                  ),
                                ),
                              ),
                            );
                          },
                        )
                      : SingleChildScrollView(
                          padding: EdgeInsets.all(16),
                          child: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              _buildEnhancedServerSettingsCard(),
                              SizedBox(height: 20),
                              _buildEnhancedAGVSettingsCard(),
                              SizedBox(height: 20),
                              _buildEnhancedUISettingsCard(themeService),
                              SizedBox(height: 20),
                              _buildEnhancedMapSettingsCard(),
                              SizedBox(height: 20),
                              _buildEnhancedSystemInfoCard(),
                              SizedBox(height: 20),
                              _buildEnhancedActionButtons(),
                              SizedBox(height: 20),
                            ],
                          ),
                        ),
                ),
        );
      },
    );
  }

  Widget _buildEnhancedServerSettingsCard() {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: isDark
              ? [
                  Colors.blue.shade800.withOpacity(0.3),
                  Colors.blue.shade900.withOpacity(0.2)
                ]
              : [Colors.blue.shade50, Colors.blue.shade100],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(
          color: isDark
              ? Colors.blue.withOpacity(0.5)
              : Colors.blue.withOpacity(0.3),
        ),
        boxShadow: [
          BoxShadow(
            color: isDark
                ? Colors.black.withOpacity(0.3)
                : Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.blue,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.blue.withOpacity(0.3),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child: Icon(Icons.dns, color: Colors.white, size: 24),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Server Configuration',
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: isDark
                              ? Colors.blue.shade300
                              : Colors.blue.shade800,
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        'Configure backend connections',
                        style: TextStyle(
                          color: isDark ? Colors.grey[400] : Colors.grey[600],
                          fontSize: 14,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            SizedBox(height: 24),
            _buildEnhancedTextField(
              controller: _serverUrlController,
              label: 'API Server URL',
              icon: Icons.link,
              helperText: 'Base URL for the fleet management API',
            ),
            SizedBox(height: 20),
            _buildEnhancedTextField(
              controller: _websocketUrlController,
              label: 'WebSocket URL',
              icon: Icons.wifi,
              helperText: 'WebSocket URL for real-time communication',
            ),
            SizedBox(height: 20),
            Row(
              children: [
                Expanded(
                  child: Text(
                    'Connection Timeout',
                    style: TextStyle(
                      fontSize: 16,
                      fontWeight: FontWeight.w600,
                      color: isDark ? Colors.white : Colors.grey[800],
                    ),
                  ),
                ),
                SizedBox(width: 16),
                Container(
                  width: 100,
                  decoration: BoxDecoration(
                    color: isDark ? theme.cardColor : Colors.white,
                    borderRadius: BorderRadius.circular(12),
                    boxShadow: [
                      BoxShadow(
                        color: isDark
                            ? Colors.black.withOpacity(0.3)
                            : Colors.black.withOpacity(0.1),
                        blurRadius: 4,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child: TextFormField(
                    initialValue: _connectionTimeout.toString(),
                    style:
                        TextStyle(color: isDark ? Colors.white : Colors.black),
                    decoration: InputDecoration(
                      border: OutlineInputBorder(
                        borderRadius: BorderRadius.circular(12),
                        borderSide: BorderSide.none,
                      ),
                      filled: true,
                      fillColor: isDark ? theme.cardColor : Colors.white,
                      suffix: Text(
                        'sec',
                        style: TextStyle(
                          color: isDark ? Colors.grey[400] : Colors.grey[600],
                        ),
                      ),
                      contentPadding:
                          EdgeInsets.symmetric(horizontal: 12, vertical: 16),
                    ),
                    keyboardType: TextInputType.number,
                    onChanged: (value) {
                      _connectionTimeout = int.tryParse(value) ?? 30;
                    },
                  ),
                ),
              ],
            ),
            SizedBox(height: 20),
            _buildEnhancedSwitchTile(
              title: 'Auto Reconnect',
              subtitle: 'Automatically reconnect on connection loss',
              value: _autoReconnect,
              icon: Icons.autorenew,
              color: Colors.green,
              onChanged: (value) {
                setState(() {
                  _autoReconnect = value;
                });
              },
            ),
            SizedBox(height: 12),
            _buildEnhancedSwitchTile(
              title: 'Debug Mode',
              subtitle: 'Enable detailed logging for troubleshooting',
              value: _debugMode,
              icon: Icons.bug_report,
              color: Colors.orange,
              onChanged: (value) {
                setState(() {
                  _debugMode = value;
                });
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedAGVSettingsCard() {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: isDark
              ? [
                  Colors.green.shade800.withOpacity(0.3),
                  Colors.green.shade900.withOpacity(0.2)
                ]
              : [Colors.green.shade50, Colors.green.shade100],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(
          color: isDark
              ? Colors.green.withOpacity(0.5)
              : Colors.green.withOpacity(0.3),
        ),
        boxShadow: [
          BoxShadow(
            color: isDark
                ? Colors.black.withOpacity(0.3)
                : Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.green,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.green.withOpacity(0.3),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child: Icon(Icons.smart_toy, color: Colors.white, size: 24),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'AGV Control Settings',
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: isDark
                              ? Colors.green.shade300
                              : Colors.green.shade800,
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        'Configure robot movement parameters',
                        style: TextStyle(
                          color: isDark ? Colors.grey[400] : Colors.grey[600],
                          fontSize: 14,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            SizedBox(height: 24),
            _buildEnhancedSlider(
              title: 'Maximum Linear Speed',
              value: _maxLinearSpeed,
              min: 0.1,
              max: 3.0,
              divisions: 29,
              unit: 'm/s',
              color: Colors.blue,
              onChanged: (value) {
                setState(() {
                  _maxLinearSpeed = value;
                });
              },
            ),
            SizedBox(height: 20),
            _buildEnhancedSlider(
              title: 'Maximum Angular Speed',
              value: _maxAngularSpeed,
              min: 0.1,
              max: 5.0,
              divisions: 49,
              unit: 'rad/s',
              color: Colors.purple,
              onChanged: (value) {
                setState(() {
                  _maxAngularSpeed = value;
                });
              },
            ),
            SizedBox(height: 20),
            _buildEnhancedSlider(
              title: 'Battery Low Threshold',
              value: _batteryLowThreshold,
              min: 5.0,
              max: 50.0,
              divisions: 45,
              unit: '%',
              color: Colors.red,
              onChanged: (value) {
                setState(() {
                  _batteryLowThreshold = value;
                });
              },
            ),
            SizedBox(height: 20),
            _buildEnhancedSlider(
              title: 'Joystick Size',
              value: _joystickSize,
              min: 150.0,
              max: 300.0,
              divisions: 30,
              unit: 'px',
              color: Colors.orange,
              onChanged: (value) {
                setState(() {
                  _joystickSize = value;
                });
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedUISettingsCard(ThemeService themeService) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: isDark
              ? [
                  Colors.purple.shade800.withOpacity(0.3),
                  Colors.purple.shade900.withOpacity(0.2)
                ]
              : [Colors.purple.shade50, Colors.purple.shade100],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(
          color: isDark
              ? Colors.purple.withOpacity(0.5)
              : Colors.purple.withOpacity(0.3),
        ),
        boxShadow: [
          BoxShadow(
            color: isDark
                ? Colors.black.withOpacity(0.3)
                : Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.purple,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.purple.withOpacity(0.3),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child: Icon(Icons.palette, color: Colors.white, size: 24),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'User Interface',
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: isDark
                              ? Colors.purple.shade300
                              : Colors.purple.shade800,
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        'Customize your app experience',
                        style: TextStyle(
                          color: isDark ? Colors.grey[400] : Colors.grey[600],
                          fontSize: 14,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            SizedBox(height: 24),
            _buildEnhancedSwitchTile(
              title: 'Dark Mode',
              subtitle: 'Use dark theme throughout the app',
              value: themeService.isDarkMode,
              icon:
                  themeService.isDarkMode ? Icons.dark_mode : Icons.light_mode,
              color: themeService.isDarkMode ? Colors.indigo : Colors.amber,
              onChanged: (value) async {
                await themeService.setTheme(value);
                // Optionally, also save to backend if connected
                try {
                  await _apiService.updateTheme(value);
                } catch (e) {
                  // Handle API error silently
                }
              },
            ),
            SizedBox(height: 12),
            _buildEnhancedSwitchTile(
              title: 'Show Notifications',
              subtitle: 'Display system notifications and alerts',
              value: _showNotifications,
              icon: Icons.notifications,
              color: Colors.blue,
              onChanged: (value) {
                setState(() {
                  _showNotifications = value;
                });
              },
            ),
            SizedBox(height: 12),
            _buildEnhancedSwitchTile(
              title: 'Sound Effects',
              subtitle: 'Play sounds for alerts and actions',
              value: _soundEnabled,
              icon: _soundEnabled ? Icons.volume_up : Icons.volume_off,
              color: Colors.green,
              onChanged: (value) {
                setState(() {
                  _soundEnabled = value;
                });
              },
            ),
            SizedBox(height: 20),
            Container(
              padding: EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: isDark
                    ? theme.cardColor.withOpacity(0.8)
                    : Colors.white.withOpacity(0.8),
                borderRadius: BorderRadius.circular(12),
                border: Border.all(
                  color: isDark
                      ? Colors.purple.withOpacity(0.3)
                      : Colors.purple.withOpacity(0.2),
                ),
              ),
              child: Row(
                children: [
                  Icon(
                    Icons.language,
                    color: Colors.purple,
                    size: 24,
                  ),
                  SizedBox(width: 16),
                  Expanded(
                    child: Text(
                      'Language',
                      style: TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.w600,
                        color: isDark ? Colors.white : Colors.grey[800],
                      ),
                    ),
                  ),
                  Container(
                    padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                    decoration: BoxDecoration(
                      color: Colors.purple.withOpacity(0.1),
                      borderRadius: BorderRadius.circular(8),
                      border: Border.all(color: Colors.purple.withOpacity(0.3)),
                    ),
                    child: DropdownButton<String>(
                      value: _selectedLanguage,
                      underline: SizedBox(),
                      style: TextStyle(
                        color: isDark
                            ? Colors.purple.shade300
                            : Colors.purple.shade700,
                        fontWeight: FontWeight.w500,
                      ),
                      items: [
                        DropdownMenuItem(
                            value: 'English', child: Text('English')),
                        DropdownMenuItem(
                            value: 'Spanish', child: Text('Español')),
                        DropdownMenuItem(
                            value: 'French', child: Text('Français')),
                        DropdownMenuItem(
                            value: 'German', child: Text('Deutsch')),
                        DropdownMenuItem(value: 'Chinese', child: Text('中文')),
                      ],
                      onChanged: (value) {
                        if (value != null) {
                          setState(() {
                            _selectedLanguage = value;
                          });
                        }
                      },
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedMapSettingsCard() {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: isDark
              ? [
                  Colors.indigo.shade800.withOpacity(0.3),
                  Colors.indigo.shade900.withOpacity(0.2)
                ]
              : [Colors.indigo.shade50, Colors.indigo.shade100],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(
          color: isDark
              ? Colors.indigo.withOpacity(0.5)
              : Colors.indigo.withOpacity(0.3),
        ),
        boxShadow: [
          BoxShadow(
            color: isDark
                ? Colors.black.withOpacity(0.3)
                : Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.indigo,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.indigo.withOpacity(0.3),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child: Icon(Icons.map, color: Colors.white, size: 24),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Map Display Settings',
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: isDark
                              ? Colors.indigo.shade300
                              : Colors.indigo.shade800,
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        'Configure map visualization',
                        style: TextStyle(
                          color: isDark ? Colors.grey[400] : Colors.grey[600],
                          fontSize: 14,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            SizedBox(height: 24),
            _buildEnhancedSlider(
              title: 'Map Refresh Rate',
              value: _mapRefreshRate,
              min: 0.5,
              max: 5.0,
              divisions: 9,
              unit: 'Hz',
              color: Colors.teal,
              onChanged: (value) {
                setState(() {
                  _mapRefreshRate = value;
                });
              },
            ),
            SizedBox(height: 20),
            _buildEnhancedSwitchTile(
              title: 'Show Grid',
              subtitle: 'Display grid lines on the map',
              value: _showGrid,
              icon: Icons.grid_on,
              color: Colors.cyan,
              onChanged: (value) {
                setState(() {
                  _showGrid = value;
                });
              },
            ),
            SizedBox(height: 12),
            _buildEnhancedSwitchTile(
              title: 'Show Robot Trail',
              subtitle: 'Display the path history of robots',
              value: _showRobotTrail,
              icon: Icons.timeline,
              color: Colors.orange,
              onChanged: (value) {
                setState(() {
                  _showRobotTrail = value;
                });
              },
            ),
            if (_showRobotTrail) ...[
              SizedBox(height: 20),
              _buildEnhancedSlider(
                title: 'Maximum Trail Length',
                value: _maxTrailLength.toDouble(),
                min: 10.0,
                max: 500.0,
                divisions: 49,
                unit: 'points',
                color: Colors.deepOrange,
                onChanged: (value) {
                  setState(() {
                    _maxTrailLength = value.toInt();
                  });
                },
              ),
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedSystemInfoCard() {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: isDark
              ? [theme.cardColor, theme.cardColor.withOpacity(0.7)]
              : [Colors.white, Colors.grey.shade50],
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
        ),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(
          color: isDark
              ? Colors.grey.withOpacity(0.5)
              : Colors.grey.withOpacity(0.3),
        ),
        boxShadow: [
          BoxShadow(
            color: isDark
                ? Colors.black.withOpacity(0.3)
                : Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Container(
                  padding: EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.grey.shade700,
                    borderRadius: BorderRadius.circular(16),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.grey.withOpacity(0.3),
                        blurRadius: 8,
                        offset: Offset(0, 2),
                      ),
                    ],
                  ),
                  child: Icon(Icons.info, color: Colors.white, size: 24),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'System Information',
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: isDark ? Colors.white : Colors.grey[800],
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        'App and system details',
                        style: TextStyle(
                          color: isDark ? Colors.grey[400] : Colors.grey[600],
                          fontSize: 14,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            SizedBox(height: 24),
            Container(
              padding: EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: isDark
                    ? Colors.grey.shade800.withOpacity(0.3)
                    : Colors.grey.shade50,
                borderRadius: BorderRadius.circular(12),
                border: Border.all(
                  color: isDark
                      ? Colors.grey.withOpacity(0.3)
                      : Colors.grey.withOpacity(0.2),
                ),
              ),
              child: Column(
                children: [
                  _buildEnhancedInfoRow('App Version', '1.0.0', Icons.apps),
                  _buildEnhancedInfoRow('Build Number', '100', Icons.build),
                  _buildEnhancedInfoRow('API Version', '2.1.0', Icons.api),
                  _buildEnhancedInfoRow(
                      'Platform', 'Flutter', Icons.flutter_dash),
                  _buildEnhancedInfoRow(
                    'Connection Status',
                    _webSocketService.isConnected
                        ? 'Connected'
                        : 'Disconnected',
                    _webSocketService.isConnected ? Icons.wifi : Icons.wifi_off,
                    statusColor: _webSocketService.isConnected
                        ? Colors.green
                        : Colors.red,
                  ),
                  _buildEnhancedInfoRow(
                      'Server URL', _apiService.baseUrl ?? '', Icons.link),
                ],
              ),
            ),
            SizedBox(height: 20),
            Row(
              children: [
                Expanded(
                  child: Container(
                    height: 50,
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        colors: [Colors.blue.shade600, Colors.blue.shade400],
                      ),
                      borderRadius: BorderRadius.circular(25),
                      boxShadow: [
                        BoxShadow(
                          color: Colors.blue.withOpacity(0.3),
                          blurRadius: 8,
                          offset: Offset(0, 4),
                        ),
                      ],
                    ),
                    child: ElevatedButton.icon(
                      onPressed: _showAboutDialog,
                      icon: Icon(Icons.info, color: Colors.white),
                      label: Text(
                        'About',
                        style: TextStyle(
                            color: Colors.white, fontWeight: FontWeight.bold),
                      ),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.transparent,
                        shadowColor: Colors.transparent,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(25),
                        ),
                      ),
                    ),
                  ),
                ),
                SizedBox(width: 12),
                Expanded(
                  child: Container(
                    height: 50,
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        colors: [
                          Colors.purple.shade600,
                          Colors.purple.shade400
                        ],
                      ),
                      borderRadius: BorderRadius.circular(25),
                      boxShadow: [
                        BoxShadow(
                          color: Colors.purple.withOpacity(0.3),
                          blurRadius: 8,
                          offset: Offset(0, 4),
                        ),
                      ],
                    ),
                    child: ElevatedButton.icon(
                      onPressed: _showLicenses,
                      icon: Icon(Icons.article, color: Colors.white),
                      label: Text(
                        'Licenses',
                        style: TextStyle(
                            color: Colors.white, fontWeight: FontWeight.bold),
                      ),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.transparent,
                        shadowColor: Colors.transparent,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(25),
                        ),
                      ),
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

  Widget _buildEnhancedActionButtons() {
    return Column(
      children: [
        Container(
          width: double.infinity,
          height: 56,
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [Colors.green.shade600, Colors.green.shade400],
            ),
            borderRadius: BorderRadius.circular(28),
            boxShadow: [
              BoxShadow(
                color: Colors.green.withOpacity(0.3),
                blurRadius: 12,
                offset: Offset(0, 6),
              ),
            ],
          ),
          child: ElevatedButton.icon(
            onPressed: _isLoading ? null : _saveSettings,
            icon: _isLoading
                ? SizedBox(
                    width: 20,
                    height: 20,
                    child: CircularProgressIndicator(
                      strokeWidth: 2,
                      valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
                    ),
                  )
                : Icon(Icons.save, color: Colors.white),
            label: Text(
              _isLoading ? 'Saving...' : 'Save Settings',
              style: TextStyle(
                color: Colors.white,
                fontWeight: FontWeight.bold,
                fontSize: 16,
              ),
            ),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.transparent,
              shadowColor: Colors.transparent,
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(28),
              ),
            ),
          ),
        ),
        SizedBox(height: 16),
        Row(
          children: [
            Expanded(
              child: _buildActionButton(
                'Test Connection',
                Icons.wifi,
                Colors.blue,
                _testConnection,
              ),
            ),
            SizedBox(width: 12),
            Expanded(
              child: _buildActionButton(
                'Export',
                Icons.download,
                Colors.orange,
                _exportSettings,
              ),
            ),
            SizedBox(width: 12),
            Expanded(
              child: _buildActionButton(
                'Import',
                Icons.upload,
                Colors.purple,
                _importSettings,
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildActionButton(
      String label, IconData icon, Color color, VoidCallback onPressed) {
    return Container(
      height: 50,
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [color.withOpacity(0.9), color.withOpacity(0.7)],
        ),
        borderRadius: BorderRadius.circular(25),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.3),
            blurRadius: 8,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: ElevatedButton.icon(
        onPressed: onPressed,
        icon: Icon(icon, color: Colors.white, size: 20),
        label: Text(
          label,
          style: TextStyle(
            color: Colors.white,
            fontWeight: FontWeight.bold,
            fontSize: 13,
          ),
        ),
        style: ElevatedButton.styleFrom(
          backgroundColor: Colors.transparent,
          shadowColor: Colors.transparent,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(25),
          ),
        ),
      ),
    );
  }

  Widget _buildEnhancedTextField({
    required TextEditingController controller,
    required String label,
    required IconData icon,
    required String helperText,
  }) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Container(
      decoration: BoxDecoration(
        color: isDark ? theme.cardColor : Colors.white,
        borderRadius: BorderRadius.circular(12),
        boxShadow: [
          BoxShadow(
            color: isDark
                ? Colors.black.withOpacity(0.3)
                : Colors.black.withOpacity(0.1),
            blurRadius: 4,
            offset: Offset(0, 2),
          ),
        ],
      ),
      child: TextFormField(
        controller: controller,
        style: TextStyle(color: isDark ? Colors.white : Colors.black),
        decoration: InputDecoration(
          labelText: label,
          prefixIcon: Icon(icon, color: Colors.blue),
          border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(12),
            borderSide: BorderSide.none,
          ),
          filled: true,
          fillColor: isDark ? theme.cardColor : Colors.white,
          labelStyle: TextStyle(
              color: isDark ? Colors.blue.shade300 : Colors.blue.shade700),
          helperText: helperText,
          helperStyle: TextStyle(
            color: isDark ? Colors.grey[500] : Colors.grey[600],
            fontSize: 12,
          ),
        ),
      ),
    );
  }

  Widget _buildEnhancedSwitchTile({
    required String title,
    required String subtitle,
    required bool value,
    required IconData icon,
    required Color color,
    required Function(bool) onChanged,
  }) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark
            ? theme.cardColor.withOpacity(0.8)
            : Colors.white.withOpacity(0.8),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(
          color: value
              ? color.withOpacity(0.5)
              : isDark
                  ? Colors.grey.withOpacity(0.3)
                  : Colors.grey.withOpacity(0.2),
        ),
      ),
      child: Row(
        children: [
          Container(
            padding: EdgeInsets.all(8),
            decoration: BoxDecoration(
              color:
                  value ? color.withOpacity(0.2) : Colors.grey.withOpacity(0.2),
              borderRadius: BorderRadius.circular(8),
            ),
            child: Icon(
              icon,
              color: value ? color : Colors.grey,
              size: 20,
            ),
          ),
          SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.w600,
                    color: isDark ? Colors.white : Colors.grey[800],
                  ),
                ),
                SizedBox(height: 2),
                Text(
                  subtitle,
                  style: TextStyle(
                    fontSize: 13,
                    color: isDark ? Colors.grey[400] : Colors.grey[600],
                  ),
                ),
              ],
            ),
          ),
          Switch(
            value: value,
            onChanged: onChanged,
            activeColor: color,
            activeTrackColor: color.withOpacity(0.3),
          ),
        ],
      ),
    );
  }

  Widget _buildEnhancedSlider({
    required String title,
    required double value,
    required double min,
    required double max,
    required int divisions,
    required String unit,
    required Color color,
    required Function(double) onChanged,
  }) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark
            ? theme.cardColor.withOpacity(0.8)
            : Colors.white.withOpacity(0.8),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(
          color: color.withOpacity(0.3),
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(
                title,
                style: TextStyle(
                  fontSize: 16,
                  fontWeight: FontWeight.w600,
                  color: isDark ? Colors.white : Colors.grey[800],
                ),
              ),
              Container(
                padding: EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                decoration: BoxDecoration(
                  color: color.withOpacity(0.1),
                  borderRadius: BorderRadius.circular(20),
                  border: Border.all(color: color.withOpacity(0.3)),
                ),
                child: Text(
                  '${value.toStringAsFixed(value == value.toInt() ? 0 : 1)} $unit',
                  style: TextStyle(
                    color: color,
                    fontWeight: FontWeight.bold,
                    fontSize: 14,
                  ),
                ),
              ),
            ],
          ),
          SizedBox(height: 12),
          SliderTheme(
            data: SliderTheme.of(context).copyWith(
              trackHeight: 6,
              thumbShape: RoundSliderThumbShape(
                enabledThumbRadius: 12,
                pressedElevation: 8,
              ),
              overlayShape: RoundSliderOverlayShape(overlayRadius: 20),
              activeTrackColor: color,
              inactiveTrackColor: color.withOpacity(0.3),
              thumbColor: color,
              overlayColor: color.withOpacity(0.2),
            ),
            child: Slider(
              value: value,
              min: min,
              max: max,
              divisions: divisions,
              onChanged: onChanged,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEnhancedInfoRow(String label, String value, IconData icon,
      {Color? statusColor}) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Padding(
      padding: EdgeInsets.symmetric(vertical: 8),
      child: Row(
        children: [
          Icon(
            icon,
            size: 18,
            color:
                statusColor ?? (isDark ? Colors.grey[400] : Colors.grey[600]),
          ),
          SizedBox(width: 12),
          Expanded(
            child: Text(
              label,
              style: TextStyle(
                fontSize: 14,
                fontWeight: FontWeight.w500,
                color: isDark ? Colors.grey[300] : Colors.grey[700],
              ),
            ),
          ),
          Flexible(
            child: Text(
              value,
              style: TextStyle(
                fontSize: 14,
                fontWeight: FontWeight.w600,
                color:
                    statusColor ?? (isDark ? Colors.white : Colors.grey[800]),
              ),
              textAlign: TextAlign.right,
              overflow: TextOverflow.ellipsis,
            ),
          ),
        ],
      ),
    );
  }

  void _testConnection() async {
    try {
      final success = await _apiService.testConnection();

      _showEnhancedSnackBar(
        success ? 'Connection successful' : 'Connection failed',
        success ? Colors.green : Colors.red,
        success ? Icons.check_circle : Icons.error,
      );
    } catch (e) {
      _showEnhancedSnackBar(
        'Connection test failed: $e',
        Colors.red,
        Icons.error,
      );
    }
  }

  void _exportSettings() {
    _showEnhancedSnackBar(
      'Settings export not implemented',
      Colors.orange,
      Icons.info,
    );
  }

  void _importSettings() {
    _showEnhancedSnackBar(
      'Settings import not implemented',
      Colors.orange,
      Icons.info,
    );
  }

  void _showAboutDialog() {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    showAboutDialog(
      context: context,
      applicationName: 'AGV Fleet Management',
      applicationVersion: '1.0.0',
      applicationIcon: Container(
        padding: EdgeInsets.all(12),
        decoration: BoxDecoration(
          gradient: LinearGradient(
            colors: [Colors.blue.shade600, Colors.blue.shade400],
          ),
          borderRadius: BorderRadius.circular(16),
        ),
        child: Icon(Icons.smart_toy, size: 32, color: Colors.white),
      ),
      children: [
        Text(
          'A comprehensive fleet management system for Automated Guided Vehicles (AGVs).',
          style: TextStyle(
            fontSize: 16,
            color: isDark ? Colors.grey[300] : Colors.grey[700],
          ),
        ),
        SizedBox(height: 16),
        Text(
          'Features:',
          style: TextStyle(
            fontWeight: FontWeight.bold,
            fontSize: 16,
            color: isDark ? Colors.white : Colors.grey[800],
          ),
        ),
        SizedBox(height: 8),
        _buildFeatureItem('Real-time AGV control and monitoring'),
        _buildFeatureItem('Interactive map editing and navigation'),
        _buildFeatureItem('Order management and automation'),
        _buildFeatureItem('Analytics and performance tracking'),
        SizedBox(height: 16),
        Text(
          'Built with Flutter and ROS2 integration.',
          style: TextStyle(
            fontStyle: FontStyle.italic,
            color: isDark ? Colors.grey[400] : Colors.grey[600],
          ),
        ),
      ],
    );
  }

  Widget _buildFeatureItem(String feature) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Padding(
      padding: EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          Icon(
            Icons.check_circle,
            size: 16,
            color: Colors.green,
          ),
          SizedBox(width: 8),
          Expanded(
            child: Text(
              feature,
              style: TextStyle(
                color: isDark ? Colors.grey[300] : Colors.grey[700],
              ),
            ),
          ),
        ],
      ),
    );
  }

  void _showLicenses() {
    showLicensePage(
      context: context,
      applicationName: 'AGV Fleet Management',
      applicationVersion: '1.0.0',
      applicationIcon: Container(
        padding: EdgeInsets.all(12),
        decoration: BoxDecoration(
          gradient: LinearGradient(
            colors: [Colors.blue.shade600, Colors.blue.shade400],
          ),
          borderRadius: BorderRadius.circular(16),
        ),
        child: Icon(Icons.smart_toy, size: 32, color: Colors.white),
      ),
    );
  }
}
