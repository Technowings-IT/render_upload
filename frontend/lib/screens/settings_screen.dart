//settings_screen.dart
import 'package:flutter/material.dart';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';

class SettingsScreen extends StatefulWidget {
  @override
  _SettingsScreenState createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  final ApiService _apiService = ApiService();
  final WebSocketService _webSocketService = WebSocketService();
  
  // Server settings
  final _serverUrlController = TextEditingController(text: 'ws://192.168.253.79:3000/api');
  final _websocketUrlController = TextEditingController(text: 'ws://192.168.253.79.79:3000');
  
  // AGV settings
  double _maxLinearSpeed = 1.0;
  double _maxAngularSpeed = 2.0;
  double _batteryLowThreshold = 20.0;
  int _connectionTimeout = 30;
  bool _autoReconnect = true;
  bool _debugMode = false;
  
  // UI settings
  bool _darkMode = false;
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

  @override
  void initState() {
    super.initState();
    _loadSettings();
  }

  @override
  void dispose() {
    _serverUrlController.dispose();
    _websocketUrlController.dispose();
    super.dispose();
  }

  void _loadSettings() {
    // In a real app, load from SharedPreferences or local storage
    setState(() {
      _isLoading = false;
    });
  }

  void _saveSettings() async {
    setState(() {
      _isLoading = true;
    });

    try {
      // In a real app, save to SharedPreferences or local storage
      // and update API service configuration
      _apiService.setBaseUrl(_serverUrlController.text);
      
      await Future.delayed(Duration(seconds: 1)); // Simulate save delay
      
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Settings saved successfully'),
          backgroundColor: Colors.green,
        ),
      );
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Failed to save settings: $e'),
          backgroundColor: Colors.red,
        ),
      );
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  void _resetSettings() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Reset Settings'),
        content: Text('Are you sure you want to reset all settings to default values?'),
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
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child: Text('Reset'),
          ),
        ],
      ),
    );
  }

  void _performReset() {
    setState(() {
      _serverUrlController.text = 'http://localhost:3000/api';
      _websocketUrlController.text = 'ws://192.168.253.79:3000';
      _maxLinearSpeed = 1.0;
      _maxAngularSpeed = 2.0;
      _batteryLowThreshold = 20.0;
      _connectionTimeout = 30;
      _autoReconnect = true;
      _debugMode = false;
      _darkMode = false;
      _showNotifications = true;
      _soundEnabled = true;
      _selectedLanguage = 'English';
      _joystickSize = 200.0;
      _mapRefreshRate = 1.0;
      _showGrid = true;
      _showRobotTrail = true;
      _maxTrailLength = 100;
    });

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('Settings reset to defaults'),
        backgroundColor: Colors.blue,
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Settings'),
        actions: [
          IconButton(
            icon: Icon(Icons.refresh),
            onPressed: _resetSettings,
            tooltip: 'Reset to Defaults',
          ),
          IconButton(
            icon: Icon(Icons.save),
            onPressed: _isLoading ? null : _saveSettings,
          ),
        ],
      ),
      body: _isLoading
          ? Center(child: CircularProgressIndicator())
          : SingleChildScrollView(
              padding: EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  _buildServerSettingsCard(),
                  SizedBox(height: 16),
                  _buildAGVSettingsCard(),
                  SizedBox(height: 16),
                  _buildUISettingsCard(),
                  SizedBox(height: 16),
                  _buildMapSettingsCard(),
                  SizedBox(height: 16),
                  _buildSystemInfoCard(),
                  SizedBox(height: 20),
                  _buildActionButtons(),
                ],
              ),
            ),
    );
  }

  Widget _buildServerSettingsCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Server Configuration',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            TextFormField(
              controller: _serverUrlController,
              decoration: InputDecoration(
                labelText: 'API Server URL',
                border: OutlineInputBorder(),
                helperText: 'Base URL for the fleet management API',
              ),
            ),
            SizedBox(height: 16),
            TextFormField(
              controller: _websocketUrlController,
              decoration: InputDecoration(
                labelText: 'WebSocket URL',
                border: OutlineInputBorder(),
                helperText: 'WebSocket URL for real-time communication',
              ),
            ),
            SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: Text('Connection Timeout'),
                ),
                SizedBox(width: 16),
                SizedBox(
                  width: 80,
                  child: TextFormField(
                    initialValue: _connectionTimeout.toString(),
                    decoration: InputDecoration(
                      border: OutlineInputBorder(),
                      suffix: Text('sec'),
                    ),
                    keyboardType: TextInputType.number,
                    onChanged: (value) {
                      _connectionTimeout = int.tryParse(value) ?? 30;
                    },
                  ),
                ),
              ],
            ),
            SizedBox(height: 16),
            SwitchListTile(
              title: Text('Auto Reconnect'),
              subtitle: Text('Automatically reconnect on connection loss'),
              value: _autoReconnect,
              onChanged: (value) {
                setState(() {
                  _autoReconnect = value;
                });
              },
            ),
            SwitchListTile(
              title: Text('Debug Mode'),
              subtitle: Text('Enable detailed logging for troubleshooting'),
              value: _debugMode,
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

  Widget _buildAGVSettingsCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'AGV Control Settings',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            Text('Maximum Linear Speed'),
            Slider(
              value: _maxLinearSpeed,
              min: 0.1,
              max: 3.0,
              divisions: 29,
              label: '${_maxLinearSpeed.toStringAsFixed(1)} m/s',
              onChanged: (value) {
                setState(() {
                  _maxLinearSpeed = value;
                });
              },
            ),
            SizedBox(height: 16),
            Text('Maximum Angular Speed'),
            Slider(
              value: _maxAngularSpeed,
              min: 0.1,
              max: 5.0,
              divisions: 49,
              label: '${_maxAngularSpeed.toStringAsFixed(1)} rad/s',
              onChanged: (value) {
                setState(() {
                  _maxAngularSpeed = value;
                });
              },
            ),
            SizedBox(height: 16),
            Text('Battery Low Threshold'),
            Slider(
              value: _batteryLowThreshold,
              min: 5.0,
              max: 50.0,
              divisions: 45,
              label: '${_batteryLowThreshold.toStringAsFixed(0)}%',
              onChanged: (value) {
                setState(() {
                  _batteryLowThreshold = value;
                });
              },
            ),
            SizedBox(height: 16),
            Text('Joystick Size'),
            Slider(
              value: _joystickSize,
              min: 150.0,
              max: 300.0,
              divisions: 30,
              label: '${_joystickSize.toStringAsFixed(0)}px',
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

  Widget _buildUISettingsCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'User Interface',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            SwitchListTile(
              title: Text('Dark Mode'),
              subtitle: Text('Use dark theme throughout the app'),
              value: _darkMode,
              onChanged: (value) {
                setState(() {
                  _darkMode = value;
                });
              },
            ),
            SwitchListTile(
              title: Text('Show Notifications'),
              subtitle: Text('Display system notifications and alerts'),
              value: _showNotifications,
              onChanged: (value) {
                setState(() {
                  _showNotifications = value;
                });
              },
            ),
            SwitchListTile(
              title: Text('Sound Effects'),
              subtitle: Text('Play sounds for alerts and actions'),
              value: _soundEnabled,
              onChanged: (value) {
                setState(() {
                  _soundEnabled = value;
                });
              },
            ),
            SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: Text('Language'),
                ),
                DropdownButton<String>(
                  value: _selectedLanguage,
                  items: [
                    DropdownMenuItem(value: 'English', child: Text('English')),
                    DropdownMenuItem(value: 'Spanish', child: Text('Español')),
                    DropdownMenuItem(value: 'French', child: Text('Français')),
                    DropdownMenuItem(value: 'German', child: Text('Deutsch')),
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
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildMapSettingsCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Map Display Settings',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            Text('Map Refresh Rate'),
            Slider(
              value: _mapRefreshRate,
              min: 0.5,
              max: 5.0,
              divisions: 9,
              label: '${_mapRefreshRate.toStringAsFixed(1)} Hz',
              onChanged: (value) {
                setState(() {
                  _mapRefreshRate = value;
                });
              },
            ),
            SizedBox(height: 16),
            SwitchListTile(
              title: Text('Show Grid'),
              subtitle: Text('Display grid lines on the map'),
              value: _showGrid,
              onChanged: (value) {
                setState(() {
                  _showGrid = value;
                });
              },
            ),
            SwitchListTile(
              title: Text('Show Robot Trail'),
              subtitle: Text('Display the path history of robots'),
              value: _showRobotTrail,
              onChanged: (value) {
                setState(() {
                  _showRobotTrail = value;
                });
              },
            ),
            if (_showRobotTrail) ...[
              SizedBox(height: 16),
              Text('Maximum Trail Length'),
              Slider(
                value: _maxTrailLength.toDouble(),
                min: 10.0,
                max: 500.0,
                divisions: 49,
                label: '${_maxTrailLength} points',
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

  Widget _buildSystemInfoCard() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'System Information',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            _buildInfoRow('App Version', '1.0.0'),
            _buildInfoRow('Build Number', '100'),
            _buildInfoRow('API Version', '2.1.0'),
            _buildInfoRow('Platform', 'Flutter'),
            _buildInfoRow('Connection Status', 
                _webSocketService.isConnected ? 'Connected' : 'Disconnected'),
            _buildInfoRow('Connected Devices', '0'), // This would be dynamic
            SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: OutlinedButton.icon(
                    onPressed: _showAboutDialog,
                    icon: Icon(Icons.info),
                    label: Text('About'),
                  ),
                ),
                SizedBox(width: 8),
                Expanded(
                  child: OutlinedButton.icon(
                    onPressed: _showLicenses,
                    icon: Icon(Icons.article),
                    label: Text('Licenses'),
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildInfoRow(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 4),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label),
          Text(
            value,
            style: TextStyle(fontWeight: FontWeight.w500),
          ),
        ],
      ),
    );
  }

  Widget _buildActionButtons() {
    return Column(
      children: [
        SizedBox(
          width: double.infinity,
          child: ElevatedButton.icon(
            onPressed: _isLoading ? null : _saveSettings,
            icon: _isLoading 
                ? SizedBox(
                    width: 20,
                    height: 20,
                    child: CircularProgressIndicator(strokeWidth: 2),
                  )
                : Icon(Icons.save),
            label: Text(_isLoading ? 'Saving...' : 'Save Settings'),
            style: ElevatedButton.styleFrom(
              padding: EdgeInsets.symmetric(vertical: 16),
              backgroundColor: Colors.green,
            ),
          ),
        ),
        SizedBox(height: 12),
        Row(
          children: [
            Expanded(
              child: OutlinedButton.icon(
                onPressed: _testConnection,
                icon: Icon(Icons.wifi),
                label: Text('Test Connection'),
              ),
            ),
            SizedBox(width: 12),
            Expanded(
              child: OutlinedButton.icon(
                onPressed: _exportSettings,
                icon: Icon(Icons.download),
                label: Text('Export'),
              ),
            ),
            SizedBox(width: 12),
            Expanded(
              child: OutlinedButton.icon(
                onPressed: _importSettings,
                icon: Icon(Icons.upload),
                label: Text('Import'),
              ),
            ),
          ],
        ),
      ],
    );
  }

  void _testConnection() async {
    try {
      final success = await _apiService.testConnection();
      
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(success ? 'Connection successful' : 'Connection failed'),
          backgroundColor: success ? Colors.green : Colors.red,
        ),
      );
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Connection test failed: $e'),
          backgroundColor: Colors.red,
        ),
      );
    }
  }

  void _exportSettings() {
    // In a real app, this would export settings to a file
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('Settings export not implemented'),
        backgroundColor: Colors.orange,
      ),
    );
  }

  void _importSettings() {
    // In a real app, this would import settings from a file
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('Settings import not implemented'),
        backgroundColor: Colors.orange,
      ),
    );
  }

  void _showAboutDialog() {
    showAboutDialog(
      context: context,
      applicationName: 'AGV Fleet Management',
      applicationVersion: '1.0.0',
      applicationIcon: Icon(Icons.smart_toy, size: 48),
      children: [
        Text('A comprehensive fleet management system for Automated Guided Vehicles (AGVs).'),
        SizedBox(height: 16),
        Text('Features:'),
        Text('• Real-time AGV control and monitoring'),
        Text('• Interactive map editing and navigation'),
        Text('• Order management and automation'),
        Text('• Analytics and performance tracking'),
        SizedBox(height: 16),
        Text('Built with Flutter and ROS2 integration.'),
      ],
    );
  }

  void _showLicenses() {
    showLicensePage(
      context: context,
      applicationName: 'AGV Fleet Management',
      applicationVersion: '1.0.0',
      applicationIcon: Icon(Icons.smart_toy, size: 48),
    );
  }
}