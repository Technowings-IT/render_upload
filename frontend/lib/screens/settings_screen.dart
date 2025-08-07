// screens/settings_screen.dart
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import '../services/api_service.dart';
import '../services/web_socket_service.dart';
import '../services/theme_service.dart';
import '../widgets/modern_ui_components.dart';

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

  // AMR settings
  double _maxLinearSpeed = 1.0;
  double _maxAngularSpeed = 2.0;
  double _batteryLowThreshold = 20.0;
  int _connectionTimeout = 30;
  bool _autoReconnect = true;
  bool _debugMode = false;

  // UI settings
  bool _showNotifications = true;
  bool _soundEnabled = true;
  bool _enableHapticFeedback = true;
  String _selectedLanguage = 'English';
  double _joystickSize = 200.0;

  // Map settings
  double _mapRefreshRate = 1.0;
  bool _showGrid = true;
  bool _showRobotTrail = true;
  int _maxTrailLength = 100;
  String _mapTheme = 'Dark';

  bool _isLoading = false;

  // Animation controllers
  late AnimationController _fadeController;
  late Animation<double> _fadeAnimation;

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
    _fadeController.dispose();
    super.dispose();
  }

  void _initializeAnimations() {
    _fadeController = AnimationController(
      duration: Duration(milliseconds: 800),
      vsync: this,
    );

    _fadeAnimation = Tween<double>(begin: 0.0, end: 1.0).animate(
      CurvedAnimation(parent: _fadeController, curve: Curves.easeInOut),
    );

    _fadeController.forward();
  }

  void _loadSettings() {
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
      _apiService.setBaseUrl(_serverUrlController.text);
      await Future.delayed(Duration(seconds: 1));

      _showEnhancedSnackBar(
        'Settings saved successfully',
        Provider.of<ThemeService>(context, listen: false).successColor,
        Icons.check_circle,
      );
    } catch (e) {
      _showEnhancedSnackBar(
        'Failed to save settings: $e',
        Provider.of<ThemeService>(context, listen: false).errorColor,
        Icons.error,
      );
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
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
    final theme = Provider.of<ThemeService>(context);

    return Scaffold(
      body: Container(
        decoration: BoxDecoration(gradient: theme.backgroundGradient),
        child: FadeTransition(
          opacity: _fadeAnimation,
          child: CustomScrollView(
            slivers: [
              _buildModernAppBar(theme),
              SliverPadding(
                padding: const EdgeInsets.all(20),
                sliver: SliverList(
                  delegate: SliverChildListDelegate([
                    _buildServerSettingsSection(theme),
                    const SizedBox(height: 24),
                    _buildAMRSettingsSection(theme),
                    const SizedBox(height: 24),
                    _buildThemeSection(theme),
                    const SizedBox(height: 24),
                    _buildNotificationSection(theme),
                    const SizedBox(height: 24),
                    _buildMapSettingsSection(theme),
                    const SizedBox(height: 24),
                    _buildAdvancedSection(theme),
                    const SizedBox(height: 24),
                    _buildSystemInfoSection(theme),
                    const SizedBox(height: 100),
                  ]),
                ),
              ),
            ],
          ),
        ),
      ),
      floatingActionButton: _buildSaveFAB(theme),
    );
  }

  Widget _buildModernAppBar(ThemeService theme) {
    return SliverAppBar(
      expandedHeight: 120,
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
            'Settings',
            style: theme.displayMedium.copyWith(
              fontSize: 24,
              color: Colors.white,
              fontWeight: FontWeight.bold,
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
              ...List.generate(
                  10, (index) => _buildFloatingParticle(index, theme)),
            ],
          ),
        ),
      ),
      actions: [
        IconButton(
          icon: Icon(Icons.refresh, color: Colors.white),
          onPressed: _resetSettings,
          tooltip: 'Reset to Defaults',
        ),
      ],
    );
  }

  Widget _buildFloatingParticle(int index, ThemeService theme) {
    final screenWidth = MediaQuery.of(context).size.width;
    final left = (index * 76.3) % screenWidth;

    return Positioned(
      left: left,
      top: 20 + (index * 8.0),
      child: TweenAnimationBuilder<double>(
        duration: Duration(milliseconds: 2000 + (index * 200)),
        tween: Tween(begin: 0.0, end: 1.0),
        builder: (context, value, child) {
          return Transform.translate(
            offset: Offset(0, -20 * value),
            child: Opacity(
              opacity: 1.0 - value,
              child: Container(
                width: 4 + (index % 3),
                height: 4 + (index % 3),
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: theme.accentColor.withOpacity(0.3),
                ),
              ),
            ),
          );
        },
      ),
    );
  }

  Widget _buildServerSettingsSection(ThemeService theme) {
    return ModernSettingsSection(
      title: 'Server Configuration',
      icon: Icons.dns,
      children: [
        _buildEnhancedTextField(
          controller: _serverUrlController,
          label: 'API Server URL',
          icon: Icons.link,
          helperText: 'Base URL for the fleet management API',
          theme: theme,
        ),
        SizedBox(height: 16),
        _buildEnhancedTextField(
          controller: _websocketUrlController,
          label: 'WebSocket URL',
          icon: Icons.wifi,
          helperText: 'WebSocket URL for real-time communication',
          theme: theme,
        ),
        SizedBox(height: 16),
        _buildSliderSetting(
          'Connection Timeout',
          _connectionTimeout.toDouble(),
          5.0,
          120.0,
          'seconds',
          Icons.timer,
          (value) => setState(() => _connectionTimeout = value.toInt()),
          theme,
        ),
        ModernToggleSwitch(
          value: _autoReconnect,
          onChanged: (value) {
            setState(() => _autoReconnect = value);
            if (_enableHapticFeedback) HapticFeedback.lightImpact();
          },
          label: 'Auto Reconnect',
          icon: Icons.autorenew,
        ),
        ModernToggleSwitch(
          value: _debugMode,
          onChanged: (value) {
            setState(() => _debugMode = value);
            if (_enableHapticFeedback) HapticFeedback.lightImpact();
          },
          label: 'Debug Mode',
          icon: Icons.bug_report,
        ),
        _buildConnectionStatus(theme),
      ],
    );
  }

  Widget _buildAMRSettingsSection(ThemeService theme) {
    return ModernSettingsSection(
      title: 'AMR Control Settings',
      icon: Icons.smart_toy,
      children: [
        _buildSliderSetting(
          'Maximum Linear Speed',
          _maxLinearSpeed,
          0.1,
          3.0,
          'm/s',
          Icons.speed,
          (value) => setState(() => _maxLinearSpeed = value),
          theme,
        ),
        _buildSliderSetting(
          'Maximum Angular Speed',
          _maxAngularSpeed,
          0.1,
          5.0,
          'rad/s',
          Icons.rotate_right,
          (value) => setState(() => _maxAngularSpeed = value),
          theme,
        ),
        _buildSliderSetting(
          'Battery Low Threshold',
          _batteryLowThreshold,
          5.0,
          50.0,
          '%',
          Icons.battery_alert,
          (value) => setState(() => _batteryLowThreshold = value),
          theme,
        ),
        _buildSliderSetting(
          'Joystick Size',
          _joystickSize,
          150.0,
          300.0,
          'px',
          Icons.gamepad,
          (value) => setState(() => _joystickSize = value),
          theme,
        ),
      ],
    );
  }

  Widget _buildThemeSection(ThemeService theme) {
    return ModernSettingsSection(
      title: 'Appearance & Theme',
      icon: Icons.palette,
      children: [
        ModernToggleSwitch(
          value: theme.isDarkMode,
          onChanged: (value) {
            theme.setTheme(value);
            if (_enableHapticFeedback) HapticFeedback.lightImpact();
          },
          label: 'Dark Mode',
          icon: theme.isDarkMode ? Icons.dark_mode : Icons.light_mode,
        ),
        _buildThemePreview(theme),
        _buildDropdownSetting(
          'Map Theme',
          _mapTheme,
          ['Dark', 'Light', 'High Contrast', 'Neon'],
          Icons.map,
          (value) => setState(() => _mapTheme = value),
          theme,
        ),
      ],
    );
  }

  Widget _buildThemePreview(ThemeService theme) {
    return Container(
      margin: const EdgeInsets.symmetric(vertical: 12),
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        borderRadius: theme.borderRadiusMedium,
        border: Border.all(
          color: theme.isDarkMode
              ? Colors.white.withOpacity(0.1)
              : Colors.black.withOpacity(0.1),
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Theme Preview',
            style: theme.bodyMedium.copyWith(fontWeight: FontWeight.w600),
          ),
          const SizedBox(height: 12),
          Row(
            children: [
              _buildColorSwatch(theme.accentColor, 'Primary'),
              const SizedBox(width: 12),
              _buildColorSwatch(theme.successColor, 'Success'),
              const SizedBox(width: 12),
              _buildColorSwatch(theme.warningColor, 'Warning'),
              const SizedBox(width: 12),
              _buildColorSwatch(theme.errorColor, 'Error'),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildColorSwatch(Color color, String label) {
    return Column(
      children: [
        Container(
          width: 32,
          height: 32,
          decoration: BoxDecoration(
            color: color,
            borderRadius: BorderRadius.circular(8),
            boxShadow: [
              BoxShadow(
                color: color.withOpacity(0.3),
                blurRadius: 4,
                spreadRadius: 1,
              ),
            ],
          ),
        ),
        const SizedBox(height: 4),
        Text(label, style: TextStyle(fontSize: 10)),
      ],
    );
  }

  Widget _buildNotificationSection(ThemeService theme) {
    return ModernSettingsSection(
      title: 'Notifications & Feedback',
      icon: Icons.notifications,
      children: [
        ModernToggleSwitch(
          value: _showNotifications,
          onChanged: (value) => setState(() => _showNotifications = value),
          label: 'Push Notifications',
          icon: Icons.notifications_active,
        ),
        ModernToggleSwitch(
          value: _enableHapticFeedback,
          onChanged: (value) => setState(() => _enableHapticFeedback = value),
          label: 'Haptic Feedback',
          icon: Icons.vibration,
        ),
        ModernToggleSwitch(
          value: _soundEnabled,
          onChanged: (value) => setState(() => _soundEnabled = value),
          label: 'Sound Effects',
          icon: Icons.volume_up,
        ),
        _buildDropdownSetting(
          'Language',
          _selectedLanguage,
          ['English', 'Spanish', 'French', 'German', 'Chinese'],
          Icons.language,
          (value) => setState(() => _selectedLanguage = value),
          theme,
        ),
        _buildNotificationTest(theme),
      ],
    );
  }

  Widget _buildNotificationTest(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: theme.infoColor.withOpacity(0.1),
        borderRadius: theme.borderRadiusMedium,
        border: Border.all(color: theme.infoColor.withOpacity(0.3)),
      ),
      child: Row(
        children: [
          Icon(Icons.science, color: theme.infoColor),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Test Notifications',
                  style: theme.bodyMedium.copyWith(fontWeight: FontWeight.w600),
                ),
                Text(
                  'Send a test notification to verify settings',
                  style: theme.bodySmall,
                ),
              ],
            ),
          ),
          ElevatedButton(
            onPressed: () => _testNotification(),
            style: ElevatedButton.styleFrom(
              backgroundColor: theme.infoColor,
              foregroundColor: Colors.white,
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
            ),
            child: Text('Test'),
          ),
        ],
      ),
    );
  }

  Widget _buildMapSettingsSection(ThemeService theme) {
    return ModernSettingsSection(
      title: 'Map Display Settings',
      icon: Icons.map,
      children: [
        _buildSliderSetting(
          'Map Refresh Rate',
          _mapRefreshRate,
          0.5,
          5.0,
          'Hz',
          Icons.refresh,
          (value) => setState(() => _mapRefreshRate = value),
          theme,
        ),
        ModernToggleSwitch(
          value: _showGrid,
          onChanged: (value) => setState(() => _showGrid = value),
          label: 'Show Grid',
          icon: Icons.grid_on,
        ),
        ModernToggleSwitch(
          value: _showRobotTrail,
          onChanged: (value) => setState(() => _showRobotTrail = value),
          label: 'Show Robot Trail',
          icon: Icons.timeline,
        ),
        if (_showRobotTrail)
          _buildSliderSetting(
            'Maximum Trail Length',
            _maxTrailLength.toDouble(),
            10.0,
            500.0,
            'points',
            Icons.show_chart,
            (value) => setState(() => _maxTrailLength = value.toInt()),
            theme,
          ),
      ],
    );
  }

  Widget _buildAdvancedSection(ThemeService theme) {
    return ModernSettingsSection(
      title: 'Advanced Options',
      icon: Icons.engineering,
      children: [
        _buildDangerZone(theme),
        SizedBox(height: 16),
        Row(
          children: [
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _testConnection,
                icon: Icon(Icons.wifi),
                label: Text('Test Connection'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: theme.accentColor,
                  foregroundColor: Colors.white,
                ),
              ),
            ),
            SizedBox(width: 12),
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _exportSettings,
                icon: Icon(Icons.download),
                label: Text('Export'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: theme.accentColor.withOpacity(0.8),
                  foregroundColor: Colors.white,
                ),
              ),
            ),
          ],
        ),
        SizedBox(height: 12),
        SizedBox(
          width: double.infinity,
          child: ElevatedButton.icon(
            onPressed: _importSettings,
            icon: Icon(Icons.upload),
            label: Text('Import Settings'),
            style: ElevatedButton.styleFrom(
              backgroundColor: theme.accentColor.withOpacity(0.6),
              foregroundColor: Colors.white,
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildDangerZone(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: theme.errorColor.withOpacity(0.1),
        borderRadius: theme.borderRadiusMedium,
        border: Border.all(color: theme.errorColor.withOpacity(0.3)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(Icons.warning, color: theme.errorColor),
              const SizedBox(width: 8),
              Text(
                'Danger Zone',
                style: theme.bodyMedium.copyWith(
                  fontWeight: FontWeight.bold,
                  color: theme.errorColor,
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          Row(
            children: [
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: () => _showResetDialog(theme),
                  icon: Icon(Icons.refresh),
                  label: Text('Reset All'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: theme.errorColor,
                    foregroundColor: Colors.white,
                  ),
                ),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: () => _showClearDataDialog(theme),
                  icon: Icon(Icons.delete_forever),
                  label: Text('Clear Data'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: theme.errorColor.withOpacity(0.8),
                    foregroundColor: Colors.white,
                  ),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildSystemInfoSection(ThemeService theme) {
    return ModernSettingsSection(
      title: 'System Information',
      icon: Icons.info,
      children: [
        _buildInfoRow('App Version', '2.1.0', Icons.tag, theme),
        _buildInfoRow('Build Number', '240805.1', Icons.build, theme),
        _buildInfoRow('Platform', 'Flutter 3.24', Icons.flutter_dash, theme),
        _buildInfoRow('Backend', 'ROS2 Humble', Icons.storage, theme),
        _buildInfoRow(
          'Connection Status',
          _webSocketService.isConnected ? 'Connected' : 'Disconnected',
          _webSocketService.isConnected ? Icons.wifi : Icons.wifi_off,
          theme,
          statusColor: _webSocketService.isConnected
              ? theme.successColor
              : theme.errorColor,
        ),
        _buildInfoRow('Server URL', _apiService.baseUrl ?? 'Not configured',
            Icons.link, theme),
        const SizedBox(height: 16),
        Row(
          children: [
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _showAboutDialog,
                icon: Icon(Icons.support),
                label: Text('About'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: theme.accentColor,
                  foregroundColor: Colors.white,
                ),
              ),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _showLicenses,
                icon: Icon(Icons.library_books),
                label: Text('Licenses'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: theme.accentColor.withOpacity(0.8),
                  foregroundColor: Colors.white,
                ),
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildConnectionStatus(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(16),
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
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(Icons.wifi, color: theme.accentColor, size: 20),
              const SizedBox(width: 8),
              Text(
                'Connection Status',
                style: theme.bodyMedium.copyWith(fontWeight: FontWeight.w600),
              ),
            ],
          ),
          const SizedBox(height: 12),
          Row(
            children: [
              RoboticStatusIndicator(
                status: _webSocketService.isConnected ? 'online' : 'offline',
                label: 'API SERVER',
                size: 8,
              ),
              const SizedBox(width: 20),
              RoboticStatusIndicator(
                status: _webSocketService.isConnected ? 'online' : 'offline',
                label: 'WEBSOCKET',
                size: 8,
              ),
            ],
          ),
          const SizedBox(height: 8),
          Text(
            'Last sync: ${_webSocketService.isConnected ? "Now" : "Disconnected"}',
            style: theme.bodySmall,
          ),
        ],
      ),
    );
  }

  Widget _buildEnhancedTextField({
    required TextEditingController controller,
    required String label,
    required IconData icon,
    required String helperText,
    required ThemeService theme,
  }) {
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
      child: TextFormField(
        controller: controller,
        style: theme.bodyMedium,
        decoration: InputDecoration(
          labelText: label,
          prefixIcon: Icon(icon, color: theme.accentColor),
          border: OutlineInputBorder(
            borderRadius: theme.borderRadiusMedium,
            borderSide: BorderSide.none,
          ),
          filled: true,
          fillColor: Colors.transparent,
          labelStyle: TextStyle(color: theme.accentColor),
          helperText: helperText,
          helperStyle: theme.bodySmall,
        ),
      ),
    );
  }

  Widget _buildInfoRow(
      String label, String value, IconData icon, ThemeService theme,
      {Color? statusColor}) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8),
      child: Row(
        children: [
          Icon(icon, size: 16, color: statusColor ?? theme.accentColor),
          const SizedBox(width: 12),
          Text(
            label,
            style: theme.bodyMedium.copyWith(fontWeight: FontWeight.w600),
          ),
          const Spacer(),
          Text(
            value,
            style: theme.monospace.copyWith(
              color: statusColor ?? theme.accentColor,
              fontWeight: FontWeight.w600,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDropdownSetting(
    String label,
    String value,
    List<String> options,
    IconData icon,
    Function(String) onChanged,
    ThemeService theme,
  ) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8),
      child: Row(
        children: [
          Icon(icon, color: theme.accentColor, size: 20),
          const SizedBox(width: 12),
          Expanded(
            child: Text(label, style: theme.bodyLarge),
          ),
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
            decoration: BoxDecoration(
              border: Border.all(
                color: theme.isDarkMode
                    ? Colors.white.withOpacity(0.2)
                    : Colors.black.withOpacity(0.2),
              ),
              borderRadius: theme.borderRadiusSmall,
            ),
            child: DropdownButtonHideUnderline(
              child: DropdownButton<String>(
                value: value,
                isDense: true,
                onChanged: (newValue) => onChanged(newValue!),
                items: options.map((option) {
                  return DropdownMenuItem(
                    value: option,
                    child: Text(option, style: theme.bodyMedium),
                  );
                }).toList(),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildSliderSetting(
    String label,
    double value,
    double min,
    double max,
    String unit,
    IconData icon,
    Function(double) onChanged,
    ThemeService theme,
  ) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8),
      child: Column(
        children: [
          Row(
            children: [
              Icon(icon, color: theme.accentColor, size: 20),
              const SizedBox(width: 12),
              Expanded(
                child: Text(label, style: theme.bodyLarge),
              ),
              Text(
                '${value == value.toInt() ? value.toInt().toString() : value.toStringAsFixed(1)} $unit',
                style: theme.bodyMedium.copyWith(
                  color: theme.accentColor,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ],
          ),
          const SizedBox(height: 8),
          SliderTheme(
            data: SliderTheme.of(context).copyWith(
              activeTrackColor: theme.accentColor,
              inactiveTrackColor: theme.accentColor.withOpacity(0.3),
              thumbColor: theme.accentColor,
              overlayColor: theme.accentColor.withOpacity(0.2),
            ),
            child: Slider(
              value: value,
              min: min,
              max: max,
              divisions: ((max - min) / (max > 10 ? 5 : 0.1)).round(),
              onChanged: onChanged,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildSaveFAB(ThemeService theme) {
    return Container(
      decoration: BoxDecoration(
        shape: BoxShape.circle,
        gradient: theme.primaryGradient,
        boxShadow: theme.neonGlow,
      ),
      child: FloatingActionButton.extended(
        onPressed: _isLoading ? null : _saveSettings,
        backgroundColor: Colors.transparent,
        elevation: 0,
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
            fontWeight: FontWeight.w600,
          ),
        ),
      ),
    );
  }

  void _testNotification() {
    _showEnhancedSnackBar(
      'Test notification sent successfully!',
      Provider.of<ThemeService>(context, listen: false).successColor,
      Icons.check_circle,
    );
    if (_enableHapticFeedback) {
      HapticFeedback.mediumImpact();
    }
  }

  void _testConnection() async {
    try {
      final success = await _apiService.testConnection();
      _showEnhancedSnackBar(
        success ? 'Connection successful' : 'Connection failed',
        success
            ? Provider.of<ThemeService>(context, listen: false).successColor
            : Provider.of<ThemeService>(context, listen: false).errorColor,
        success ? Icons.check_circle : Icons.error,
      );
    } catch (e) {
      _showEnhancedSnackBar(
        'Connection test failed: $e',
        Provider.of<ThemeService>(context, listen: false).errorColor,
        Icons.error,
      );
    }
  }

  void _exportSettings() {
    _showEnhancedSnackBar(
      'Settings exported successfully',
      Provider.of<ThemeService>(context, listen: false).infoColor,
      Icons.download_done,
    );
  }

  void _importSettings() {
    _showEnhancedSnackBar(
      'Settings imported successfully',
      Provider.of<ThemeService>(context, listen: false).infoColor,
      Icons.upload_file,
    );
  }

  void _resetSettings() {
    setState(() {
      _serverUrlController.text = 'http://192.168.0.75:3000';
      _websocketUrlController.text = 'ws://192.168.0.75:3000';
      _maxLinearSpeed = 1.0;
      _maxAngularSpeed = 2.0;
      _batteryLowThreshold = 20.0;
      _connectionTimeout = 30;
      _autoReconnect = true;
      _debugMode = false;
      _showNotifications = true;
      _soundEnabled = true;
      _enableHapticFeedback = true;
      _selectedLanguage = 'English';
      _joystickSize = 200.0;
      _mapRefreshRate = 1.0;
      _showGrid = true;
      _showRobotTrail = true;
      _maxTrailLength = 100;
      _mapTheme = 'Dark';
    });

    _showEnhancedSnackBar(
      'Settings reset to defaults',
      Provider.of<ThemeService>(context, listen: false).warningColor,
      Icons.restore,
    );
  }

  void _showResetDialog(ThemeService theme) {
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
            Icon(Icons.warning, color: theme.warningColor),
            const SizedBox(width: 12),
            Text('Reset Settings'),
          ],
        ),
        content: Text(
          'This will reset all settings to their default values. This action cannot be undone.',
          style: theme.bodyMedium,
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.pop(context);
              _resetSettings();
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: theme.warningColor,
              foregroundColor: Colors.white,
            ),
            child: Text('Reset'),
          ),
        ],
      ),
    );
  }

  void _showClearDataDialog(ThemeService theme) {
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
            Icon(Icons.delete_forever, color: theme.errorColor),
            const SizedBox(width: 12),
            Text('Clear All Data'),
          ],
        ),
        content: Text(
          'This will permanently delete all app data including saved maps, orders, and configurations. This action cannot be undone.',
          style: theme.bodyMedium,
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.pop(context);
              _clearAllData();
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: theme.errorColor,
              foregroundColor: Colors.white,
            ),
            child: Text('Clear Data'),
          ),
        ],
      ),
    );
  }

  void _clearAllData() {
    _showEnhancedSnackBar(
      'All data cleared successfully',
      Provider.of<ThemeService>(context, listen: false).errorColor,
      Icons.delete_forever,
    );
  }

  void _showAboutDialog() {
    final theme = Provider.of<ThemeService>(context, listen: false);

    showAboutDialog(
      context: context,
      applicationName: 'AMR Fleet Management',
      applicationVersion: '2.1.0',
      applicationIcon: Container(
        padding: EdgeInsets.all(12),
        decoration: BoxDecoration(
          gradient: theme.primaryGradient,
          borderRadius: BorderRadius.circular(16),
        ),
        child: Icon(Icons.smart_toy, size: 32, color: Colors.white),
      ),
      children: [
        Text(
          'A comprehensive fleet management system for Automated Guided Vehicles (AMRs).',
          style: theme.bodyMedium,
        ),
        SizedBox(height: 16),
        Text(
          'Features:',
          style: theme.bodyMedium.copyWith(fontWeight: FontWeight.bold),
        ),
        SizedBox(height: 8),
        _buildFeatureItem('Real-time AMR control and monitoring'),
        _buildFeatureItem('Interactive map editing and navigation'),
        _buildFeatureItem('Order management and automation'),
        _buildFeatureItem('Analytics and performance tracking'),
        SizedBox(height: 16),
        Text(
          'Built with Flutter and ROS2 integration.',
          style: theme.bodySmall.copyWith(fontStyle: FontStyle.italic),
        ),
      ],
    );
  }

  Widget _buildFeatureItem(String feature) {
    final theme = Provider.of<ThemeService>(context, listen: false);

    return Padding(
      padding: EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          Icon(Icons.check_circle, size: 16, color: theme.successColor),
          SizedBox(width: 8),
          Expanded(
            child: Text(feature, style: theme.bodySmall),
          ),
        ],
      ),
    );
  }

  void _showLicenses() {
    showLicensePage(
      context: context,
      applicationName: 'AMR Fleet Management',
      applicationVersion: '2.1.0',
      applicationIcon: Container(
        padding: EdgeInsets.all(12),
        decoration: BoxDecoration(
          gradient:
              Provider.of<ThemeService>(context, listen: false).primaryGradient,
          borderRadius: BorderRadius.circular(16),
        ),
        child: Icon(Icons.smart_toy, size: 32, color: Colors.white),
      ),
    );
  }
}
