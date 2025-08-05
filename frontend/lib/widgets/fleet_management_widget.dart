// widgets/fleet_management_widgets.dart - Specialized AGV Fleet Management Components
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'dart:math' as math;
import '../services/theme_service.dart';
import 'modern_ui_components.dart';

// 🤖 AGV Status Dashboard Widget
class AGVStatusDashboard extends StatefulWidget {
  final Map<String, dynamic> agvData;
  final VoidCallback? onTap;

  const AGVStatusDashboard({
    Key? key,
    required this.agvData,
    this.onTap,
  }) : super(key: key);

  @override
  State<AGVStatusDashboard> createState() => _AGVStatusDashboardState();
}

class _AGVStatusDashboardState extends State<AGVStatusDashboard>
    with TickerProviderStateMixin {
  late AnimationController _pulseController;
  late AnimationController _batteryController;
  late Animation<double> _pulseAnimation;
  late Animation<double> _batteryAnimation;

  @override
  void initState() {
    super.initState();
    _pulseController = AnimationController(
      duration: const Duration(milliseconds: 2000),
      vsync: this,
    );
    _batteryController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );

    _pulseAnimation = Tween<double>(begin: 0.95, end: 1.05).animate(
      CurvedAnimation(parent: _pulseController, curve: Curves.easeInOut),
    );
    _batteryAnimation = Tween<double>(begin: 0.0, end: 1.0).animate(
      CurvedAnimation(parent: _batteryController, curve: Curves.easeOutCubic),
    );

    if (widget.agvData['status'] == 'active') {
      _pulseController.repeat(reverse: true);
    }
    _batteryController.forward();
  }

  @override
  void dispose() {
    _pulseController.dispose();
    _batteryController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);
    final isOnline = widget.agvData['status'] == 'online' ||
        widget.agvData['status'] == 'active';
    final batteryLevel = widget.agvData['battery'] ?? 85;
    final speed = widget.agvData['speed'] ?? 0.0;
    final position = widget.agvData['position'] ?? {'x': 0.0, 'y': 0.0};

    return ModernGlassCard(
      onTap: widget.onTap,
      showGlow: isOnline,
      glowColor: isOnline ? theme.successColor : theme.errorColor,
      child: Column(
        children: [
          // Header with AGV info
          Row(
            children: [
              AnimatedBuilder(
                animation: _pulseAnimation,
                builder: (context, child) {
                  return Transform.scale(
                    scale: isOnline ? _pulseAnimation.value : 1.0,
                    child: Container(
                      width: 60,
                      height: 60,
                      decoration: BoxDecoration(
                        gradient: RadialGradient(
                          colors: isOnline
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
                        boxShadow: isOnline ? theme.neonGlow : null,
                      ),
                      child: Icon(
                        Icons.smart_toy,
                        color: Colors.white,
                        size: 30,
                      ),
                    ),
                  );
                },
              ),
              const SizedBox(width: 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      widget.agvData['name'] ?? 'AGV Unit',
                      style: theme.headlineMedium,
                    ),
                    Text(
                      'ID: ${widget.agvData['id'] ?? 'unknown'}',
                      style: theme.bodySmall,
                    ),
                    RoboticStatusIndicator(
                      status: widget.agvData['status'] ?? 'offline',
                      label:
                          (widget.agvData['status'] ?? 'offline').toUpperCase(),
                      animated: isOnline,
                    ),
                  ],
                ),
              ),
              _buildBatteryIndicator(batteryLevel, theme),
            ],
          ),

          const SizedBox(height: 20),

          // Technical data grid
          _buildTechnicalDataGrid(speed, position, theme),

          const SizedBox(height: 16),

          // Action buttons
          Row(
            children: [
              Expanded(
                child: ModernActionButton(
                  label: 'Control',
                  icon: Icons.gamepad,
                  onPressed: () => _openControl(),
                  isSecondary: true,
                ),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: ModernActionButton(
                  label: 'Monitor',
                  icon: Icons.monitor,
                  onPressed: () => _openMonitor(),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildBatteryIndicator(int batteryLevel, ThemeService theme) {
    Color batteryColor;
    if (batteryLevel > 60)
      batteryColor = theme.successColor;
    else if (batteryLevel > 30)
      batteryColor = theme.warningColor;
    else
      batteryColor = theme.errorColor;

    return AnimatedBuilder(
      animation: _batteryAnimation,
      builder: (context, child) {
        return Column(
          children: [
            Stack(
              children: [
                Container(
                  width: 40,
                  height: 20,
                  decoration: BoxDecoration(
                    border: Border.all(color: batteryColor, width: 2),
                    borderRadius: BorderRadius.circular(4),
                  ),
                ),
                Container(
                  width: 38 * (batteryLevel / 100) * _batteryAnimation.value,
                  height: 16,
                  margin: const EdgeInsets.all(2),
                  decoration: BoxDecoration(
                    color: batteryColor,
                    borderRadius: BorderRadius.circular(2),
                  ),
                ),
                Positioned(
                  right: -4,
                  top: 6,
                  child: Container(
                    width: 4,
                    height: 8,
                    decoration: BoxDecoration(
                      color: batteryColor,
                      borderRadius: const BorderRadius.only(
                        topRight: Radius.circular(2),
                        bottomRight: Radius.circular(2),
                      ),
                    ),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 4),
            Text(
              '$batteryLevel%',
              style: theme.bodySmall.copyWith(
                color: batteryColor,
                fontWeight: FontWeight.bold,
              ),
            ),
          ],
        );
      },
    );
  }

  Widget _buildTechnicalDataGrid(
      double speed, Map<String, dynamic> position, ThemeService theme) {
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
          Expanded(
            child: _buildDataItem(
                'SPEED', '${speed.toStringAsFixed(1)} m/s', Icons.speed, theme),
          ),
          _buildVerticalDivider(theme),
          Expanded(
            child: _buildDataItem(
                'X POS',
                '${position['x']?.toStringAsFixed(2) ?? '0.00'}',
                Icons.straighten,
                theme),
          ),
          _buildVerticalDivider(theme),
          Expanded(
            child: _buildDataItem(
                'Y POS',
                '${position['y']?.toStringAsFixed(2) ?? '0.00'}',
                Icons.straighten,
                theme),
          ),
        ],
      ),
    );
  }

  Widget _buildDataItem(
      String label, String value, IconData icon, ThemeService theme) {
    return Column(
      children: [
        Icon(icon, color: theme.accentColor, size: 16),
        const SizedBox(height: 4),
        Text(
          value,
          style: theme.monospace.copyWith(
            color: theme.accentColor,
            fontWeight: FontWeight.bold,
          ),
        ),
        Text(
          label,
          style: theme.bodySmall.copyWith(fontSize: 10),
        ),
      ],
    );
  }

  Widget _buildVerticalDivider(ThemeService theme) {
    return Container(
      width: 1,
      height: 40,
      color: theme.isDarkMode
          ? Colors.white.withOpacity(0.2)
          : Colors.black.withOpacity(0.2),
      margin: const EdgeInsets.symmetric(horizontal: 8),
    );
  }

  void _openControl() {
    print('Opening AGV control for ${widget.agvData['id']}');
  }

  void _openMonitor() {
    print('Opening AGV monitor for ${widget.agvData['id']}');
  }
}

// 📊 Real-time Fleet Metrics Widget
class FleetMetricsWidget extends StatefulWidget {
  final Map<String, dynamic> metricsData;

  const FleetMetricsWidget({
    Key? key,
    required this.metricsData,
  }) : super(key: key);

  @override
  State<FleetMetricsWidget> createState() => _FleetMetricsWidgetState();
}

class _FleetMetricsWidgetState extends State<FleetMetricsWidget>
    with TickerProviderStateMixin {
  late AnimationController _chartController;
  late List<AnimationController> _barControllers;
  late List<Animation<double>> _barAnimations;

  @override
  void initState() {
    super.initState();
    _chartController = AnimationController(
      duration: const Duration(milliseconds: 2000),
      vsync: this,
    );

    _barControllers = List.generate(
        4,
        (index) => AnimationController(
              duration: Duration(milliseconds: 800 + (index * 200)),
              vsync: this,
            ));

    _barAnimations = _barControllers
        .map(
          (controller) => Tween<double>(begin: 0.0, end: 1.0).animate(
            CurvedAnimation(parent: controller, curve: Curves.easeOutCubic),
          ),
        )
        .toList();

    _startAnimations();
  }

  void _startAnimations() async {
    _chartController.forward();
    for (int i = 0; i < _barControllers.length; i++) {
      await Future.delayed(Duration(milliseconds: 100 * i));
      _barControllers[i].forward();
    }
  }

  @override
  void dispose() {
    _chartController.dispose();
    for (final controller in _barControllers) {
      controller.dispose();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    return ModernGlassCard(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                padding: const EdgeInsets.all(8),
                decoration: BoxDecoration(
                  color: theme.infoColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(Icons.analytics, color: theme.infoColor, size: 24),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('Fleet Metrics', style: theme.headlineMedium),
                    Text('Real-time performance data', style: theme.bodySmall),
                  ],
                ),
              ),
              _buildMetricBadge('99.2%', 'UPTIME', theme.successColor, theme),
            ],
          ),

          const SizedBox(height: 20),

          // Animated metrics chart
          _buildAnimatedMetricsChart(theme),

          const SizedBox(height: 16),

          // Key metrics row
          _buildKeyMetricsRow(theme),
        ],
      ),
    );
  }

  Widget _buildAnimatedMetricsChart(ThemeService theme) {
    final metrics = [
      {'label': 'Efficiency', 'value': 94, 'color': theme.successColor},
      {'label': 'Battery Avg', 'value': 78, 'color': theme.warningColor},
      {'label': 'Tasks/Hr', 'value': 86, 'color': theme.infoColor},
      {'label': 'Utilization', 'value': 92, 'color': theme.accentColor},
    ];

    return Container(
      height: 120,
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: theme.isDarkMode
            ? Colors.white.withOpacity(0.03)
            : Colors.black.withOpacity(0.03),
        borderRadius: theme.borderRadiusSmall,
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: metrics.asMap().entries.map((entry) {
          final index = entry.key;
          final metric = entry.value;

          return AnimatedBuilder(
            animation: _barAnimations[index],
            builder: (context, child) {
              return Column(
                mainAxisAlignment: MainAxisAlignment.end,
                children: [
                  Text(
                    '${((metric['value'] as int) * _barAnimations[index].value).toInt()}%',
                    style: theme.bodySmall.copyWith(
                      color: metric['color'] as Color,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 8),
                  Container(
                    width: 20,
                    height: 60 * _barAnimations[index].value,
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        begin: Alignment.bottomCenter,
                        end: Alignment.topCenter,
                        colors: [
                          metric['color'] as Color,
                          (metric['color'] as Color).withOpacity(0.5),
                        ],
                      ),
                      borderRadius: const BorderRadius.only(
                        topLeft: Radius.circular(4),
                        topRight: Radius.circular(4),
                      ),
                      boxShadow: [
                        BoxShadow(
                          color: (metric['color'] as Color).withOpacity(0.3),
                          blurRadius: 4,
                          spreadRadius: 1,
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(height: 8),
                  Text(
                    metric['label'] as String,
                    style: theme.bodySmall.copyWith(fontSize: 10),
                  ),
                ],
              );
            },
          );
        }).toList(),
      ),
    );
  }

  Widget _buildKeyMetricsRow(ThemeService theme) {
    return Row(
      children: [
        Expanded(
            child:
                _buildMetricBadge('24', 'ACTIVE AGVS', theme.infoColor, theme)),
        const SizedBox(width: 12),
        Expanded(
            child: _buildMetricBadge(
                '847', 'TASKS TODAY', theme.successColor, theme)),
        const SizedBox(width: 12),
        Expanded(
            child: _buildMetricBadge(
                '2.3s', 'AVG RESPONSE', theme.warningColor, theme)),
      ],
    );
  }

  Widget _buildMetricBadge(
      String value, String label, Color color, ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: theme.borderRadiusSmall,
        border: Border.all(color: color.withOpacity(0.3)),
      ),
      child: Column(
        children: [
          Text(
            value,
            style: theme.headlineMedium.copyWith(
              color: color,
              fontWeight: FontWeight.bold,
            ),
          ),
          const SizedBox(height: 4),
          Text(
            label,
            style: theme.bodySmall.copyWith(fontSize: 10),
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }
}

// 🗺️ Mission Control Panel Widget
class MissionControlPanel extends StatefulWidget {
  final List<Map<String, dynamic>> activeMissions;
  final VoidCallback? onCreateMission;

  const MissionControlPanel({
    Key? key,
    required this.activeMissions,
    this.onCreateMission,
  }) : super(key: key);

  @override
  State<MissionControlPanel> createState() => _MissionControlPanelState();
}

class _MissionControlPanelState extends State<MissionControlPanel>
    with TickerProviderStateMixin {
  late AnimationController _scanController;
  late Animation<double> _scanAnimation;

  @override
  void initState() {
    super.initState();
    _scanController = AnimationController(
      duration: const Duration(milliseconds: 3000),
      vsync: this,
    );
    _scanAnimation = Tween<double>(begin: 0.0, end: 1.0).animate(
      CurvedAnimation(parent: _scanController, curve: Curves.linear),
    );
    _scanController.repeat();
  }

  @override
  void dispose() {
    _scanController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    return ModernGlassCard(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Mission control header
          Row(
            children: [
              AnimatedBuilder(
                animation: _scanAnimation,
                builder: (context, child) {
                  return Container(
                    padding: const EdgeInsets.all(8),
                    decoration: BoxDecoration(
                      color: theme.successColor.withOpacity(0.1),
                      borderRadius: theme.borderRadiusSmall,
                      boxShadow: [
                        BoxShadow(
                          color: theme.successColor
                              .withOpacity(_scanAnimation.value * 0.3),
                          blurRadius: 8,
                          spreadRadius: 2,
                        ),
                      ],
                    ),
                    child: Icon(
                      Icons.radar,
                      color: theme.successColor,
                      size: 24,
                    ),
                  );
                },
              ),
              const SizedBox(width: 12),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('Mission Control', style: theme.headlineMedium),
                    Text('${widget.activeMissions.length} active missions',
                        style: theme.bodySmall),
                  ],
                ),
              ),
              ModernActionButton(
                label: 'New Mission',
                icon: Icons.add_task,
                onPressed: widget.onCreateMission ?? () {},
                isSecondary: true,
              ),
            ],
          ),

          const SizedBox(height: 20),

          // Active missions list
          if (widget.activeMissions.isEmpty)
            _buildEmptyMissionsState(theme)
          else
            ...widget.activeMissions.take(3).map(
                  (mission) => _buildMissionCard(mission, theme),
                ),
        ],
      ),
    );
  }

  Widget _buildEmptyMissionsState(ThemeService theme) {
    return Container(
      padding: const EdgeInsets.all(24),
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
          Icon(
            Icons.assignment_outlined,
            size: 48,
            color: theme.accentColor.withOpacity(0.5),
          ),
          const SizedBox(height: 12),
          Text(
            'No Active Missions',
            style: theme.headlineMedium,
          ),
          const SizedBox(height: 8),
          Text(
            'Fleet is standing by for new orders',
            style: theme.bodySmall,
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }

  Widget _buildMissionCard(Map<String, dynamic> mission, ThemeService theme) {
    final progress = mission['progress'] ?? 0;
    final status = mission['status'] ?? 'pending';
    final agvId = mission['agvId'] ?? 'unknown';

    Color statusColor;
    switch (status) {
      case 'active':
        statusColor = theme.infoColor;
        break;
      case 'completed':
        statusColor = theme.successColor;
        break;
      case 'failed':
        statusColor = theme.errorColor;
        break;
      default:
        statusColor = theme.warningColor;
    }

    return Container(
      margin: const EdgeInsets.only(bottom: 12),
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: statusColor.withOpacity(0.05),
        borderRadius: theme.borderRadiusMedium,
        border: Border.all(color: statusColor.withOpacity(0.2)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                width: 8,
                height: 8,
                decoration: BoxDecoration(
                  color: statusColor,
                  shape: BoxShape.circle,
                ),
              ),
              const SizedBox(width: 8),
              Text(
                mission['name'] ?? 'Mission',
                style: theme.bodyLarge.copyWith(fontWeight: FontWeight.w600),
              ),
              const Spacer(),
              Text(
                status.toUpperCase(),
                style: theme.bodySmall.copyWith(
                  color: statusColor,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ],
          ),
          const SizedBox(height: 8),
          Text(
            'AGV: $agvId',
            style: theme.bodySmall,
          ),
          const SizedBox(height: 12),
          LinearProgressIndicator(
            value: progress / 100.0,
            backgroundColor: theme.isDarkMode
                ? Colors.white.withOpacity(0.1)
                : Colors.black.withOpacity(0.1),
            valueColor: AlwaysStoppedAnimation<Color>(statusColor),
            borderRadius: BorderRadius.circular(4),
            minHeight: 4,
          ),
          const SizedBox(height: 8),
          Row(
            children: [
              Text(
                '$progress% Complete',
                style: theme.bodySmall.copyWith(
                  color: statusColor,
                  fontWeight: FontWeight.w600,
                ),
              ),
              const Spacer(),
              Text(
                mission['eta'] ?? 'Calculating...',
                style: theme.bodySmall,
              ),
            ],
          ),
        ],
      ),
    );
  }
}

// ⚡ Emergency Control Panel
class EmergencyControlPanel extends StatefulWidget {
  final VoidCallback? onEmergencyStop;
  final VoidCallback? onResetAll;

  const EmergencyControlPanel({
    Key? key,
    this.onEmergencyStop,
    this.onResetAll,
  }) : super(key: key);

  @override
  State<EmergencyControlPanel> createState() => _EmergencyControlPanelState();
}

class _EmergencyControlPanelState extends State<EmergencyControlPanel>
    with SingleTickerProviderStateMixin {
  late AnimationController _alertController;
  late Animation<double> _alertAnimation;

  @override
  void initState() {
    super.initState();
    _alertController = AnimationController(
      duration: const Duration(milliseconds: 1000),
      vsync: this,
    );
    _alertAnimation = Tween<double>(begin: 0.8, end: 1.0).animate(
      CurvedAnimation(parent: _alertController, curve: Curves.easeInOut),
    );
    _alertController.repeat(reverse: true);
  }

  @override
  void dispose() {
    _alertController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    return ModernGlassCard(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              AnimatedBuilder(
                animation: _alertAnimation,
                builder: (context, child) {
                  return Transform.scale(
                    scale: _alertAnimation.value,
                    child: Container(
                      padding: const EdgeInsets.all(8),
                      decoration: BoxDecoration(
                        color: theme.errorColor.withOpacity(0.2),
                        borderRadius: theme.borderRadiusSmall,
                        boxShadow: [
                          BoxShadow(
                            color: theme.errorColor.withOpacity(0.3),
                            blurRadius: 8,
                            spreadRadius: 2,
                          ),
                        ],
                      ),
                      child: Icon(
                        Icons.emergency,
                        color: theme.errorColor,
                        size: 24,
                      ),
                    ),
                  );
                },
              ),
              const SizedBox(width: 12),
              Text('Emergency Controls', style: theme.headlineMedium),
            ],
          ),

          const SizedBox(height: 20),

          // Emergency stop button
          Container(
            width: double.infinity,
            height: 60,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [theme.errorColor, theme.errorColor.withOpacity(0.8)],
              ),
              borderRadius: theme.borderRadiusMedium,
              boxShadow: [
                BoxShadow(
                  color: theme.errorColor.withOpacity(0.4),
                  blurRadius: 12,
                  spreadRadius: 2,
                ),
              ],
            ),
            child: ElevatedButton.icon(
              onPressed: () => _showEmergencyDialog(context, theme),
              icon: Icon(Icons.stop, color: Colors.white, size: 28),
              label: Text(
                'EMERGENCY STOP ALL',
                style: TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  fontSize: 18,
                  letterSpacing: 1.2,
                ),
              ),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.transparent,
                elevation: 0,
                shape: RoundedRectangleBorder(
                  borderRadius: theme.borderRadiusMedium,
                ),
              ),
            ),
          ),

          const SizedBox(height: 16),

          // Reset button
          SizedBox(
            width: double.infinity,
            child: OutlinedButton.icon(
              onPressed: widget.onResetAll,
              icon: Icon(Icons.refresh),
              label: Text('Reset All Systems'),
              style: OutlinedButton.styleFrom(
                foregroundColor: theme.warningColor,
                side: BorderSide(color: theme.warningColor),
                padding: const EdgeInsets.symmetric(vertical: 16),
              ),
            ),
          ),
        ],
      ),
    );
  }

  void _showEmergencyDialog(BuildContext context, ThemeService theme) {
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
            Icon(Icons.warning, color: theme.errorColor, size: 32),
            const SizedBox(width: 12),
            Text(
              'EMERGENCY STOP',
              style: TextStyle(
                color: theme.errorColor,
                fontWeight: FontWeight.bold,
              ),
            ),
          ],
        ),
        content: Text(
          'This will immediately stop all AGV operations and movement. Confirm emergency stop?',
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
              widget.onEmergencyStop?.call();
            },
            style: ElevatedButton.styleFrom(
              backgroundColor: theme.errorColor,
              foregroundColor: Colors.white,
            ),
            child: Text('EMERGENCY STOP'),
          ),
        ],
      ),
    );
  }
}
