// widgets/modern_ui_components.dart - Modern Fleet Management UI Components
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../services/theme_service.dart';
class ModernGlassCard extends StatelessWidget {
  final Widget child;
  final double? width;
  final double? height;
  final EdgeInsets? padding;
  final VoidCallback? onTap;
  final bool showGlow;
  final Color? glowColor;

  const ModernGlassCard({
    Key? key,
    required this.child,
    this.width,
    this.height,
    this.padding,
    this.onTap,
    this.showGlow = false,
    this.glowColor,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    return Container(
      width: width,
      height: height,
      decoration: BoxDecoration(
        borderRadius: theme.borderRadiusMedium,
        gradient: LinearGradient(
          colors: theme.isDarkMode
              ? [
                  Colors.white.withOpacity(0.1),
                  Colors.white.withOpacity(0.05),
                ]
              : [
                  Colors.white.withOpacity(0.9),
                  Colors.white.withOpacity(0.6),
                ],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        border: Border.all(
          color: theme.isDarkMode
              ? Colors.white.withOpacity(0.2)
              : Colors.black.withOpacity(0.1),
          width: 1,
        ),
        boxShadow: showGlow
            ? [
                BoxShadow(
                  color: (glowColor ?? theme.accentColor).withOpacity(0.3),
                  blurRadius: 12,
                  spreadRadius: 2,
                ),
                ...theme.elevationMedium,
              ]
            : theme.elevationMedium,
      ),
      child: Material(
        color: Colors.transparent,
        child: InkWell(
          onTap: onTap,
          borderRadius: theme.borderRadiusMedium,
          child: Padding(
            padding: padding ?? const EdgeInsets.all(20),
            child: child,
          ),
        ),
      ),
    );
  }
}

//  Robotic Status Indicator
class RoboticStatusIndicator extends StatefulWidget {
  final String status;
  final String label;
  final double size;
  final bool animated;

  const RoboticStatusIndicator({
    Key? key,
    required this.status,
    required this.label,
    this.size = 12,
    this.animated = true,
  }) : super(key: key);

  @override
  State<RoboticStatusIndicator> createState() => _RoboticStatusIndicatorState();
}

class _RoboticStatusIndicatorState extends State<RoboticStatusIndicator>
    with TickerProviderStateMixin {
  late AnimationController _pulseController;
  late Animation<double> _pulseAnimation;

  @override
  void initState() {
    super.initState();
    _pulseController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );
    _pulseAnimation = Tween<double>(begin: 0.8, end: 1.2).animate(
      CurvedAnimation(parent: _pulseController, curve: Curves.easeInOut),
    );

    if (widget.animated && widget.status.toLowerCase() == 'active') {
      _pulseController.repeat(reverse: true);
    }
  }

  @override
  void dispose() {
    _pulseController.dispose();
    super.dispose();
  }

  Color _getStatusColor(String status) {
    switch (status.toLowerCase()) {
      case 'online':
      case 'connected':
      case 'active':
        return const Color(0xFF10B981);
      case 'offline':
      case 'disconnected':
      case 'failed':
        return const Color(0xFFEF4444);
      case 'pending':
      case 'waiting':
        return const Color(0xFFF59E0B);
      case 'completed':
        return const Color(0xFF06B6D4);
      default:
        return const Color(0xFF64748B);
    }
  }

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);
    final color = _getStatusColor(widget.status);

    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        AnimatedBuilder(
          animation: _pulseAnimation,
          builder: (context, child) {
            return Transform.scale(
              scale: widget.animated ? _pulseAnimation.value : 1.0,
              child: Container(
                width: widget.size,
                height: widget.size,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: color,
                  boxShadow: [
                    BoxShadow(
                      color: color.withOpacity(0.5),
                      blurRadius: 4,
                      spreadRadius: 1,
                    ),
                  ],
                ),
              ),
            );
          },
        ),
        const SizedBox(width: 8),
        Text(
          widget.label,
          style: theme.bodySmall.copyWith(
            color: color,
            fontWeight: FontWeight.w600,
            letterSpacing: 0.5,
          ),
        ),
      ],
    );
  }
}

//  Modern Action Button with Hover Effect
class ModernActionButton extends StatefulWidget {
  final String label;
  final IconData icon;
  final VoidCallback onPressed;
  final Color? backgroundColor;
  final Color? foregroundColor;
  final bool isLoading;
  final bool isSecondary;

  const ModernActionButton({
    Key? key,
    required this.label,
    required this.icon,
    required this.onPressed,
    this.backgroundColor,
    this.foregroundColor,
    this.isLoading = false,
    this.isSecondary = false,
  }) : super(key: key);

  @override
  State<ModernActionButton> createState() => _ModernActionButtonState();
}

class _ModernActionButtonState extends State<ModernActionButton>
    with SingleTickerProviderStateMixin {
  late AnimationController _hoverController;
  late Animation<double> _scaleAnimation;
  bool _isHovered = false;

  @override
  void initState() {
    super.initState();
    _hoverController = AnimationController(
      duration: const Duration(milliseconds: 200),
      vsync: this,
    );
    _scaleAnimation = Tween<double>(begin: 1.0, end: 1.05).animate(
      CurvedAnimation(parent: _hoverController, curve: Curves.easeInOut),
    );
  }

  @override
  void dispose() {
    _hoverController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    return AnimatedBuilder(
      animation: _scaleAnimation,
      builder: (context, child) {
        return Transform.scale(
          scale: _scaleAnimation.value,
          child: MouseRegion(
            onEnter: (_) {
              setState(() => _isHovered = true);
              _hoverController.forward();
            },
            onExit: (_) {
              setState(() => _isHovered = false);
              _hoverController.reverse();
            },
            child: Container(
              decoration: BoxDecoration(
                borderRadius: theme.borderRadiusMedium,
                gradient: widget.isSecondary ? null : theme.primaryGradient,
                border: widget.isSecondary
                    ? Border.all(color: theme.accentColor, width: 2)
                    : null,
                boxShadow: _isHovered ? theme.neonGlow : theme.elevationSmall,
              ),
              child: ElevatedButton.icon(
                onPressed: widget.isLoading ? null : widget.onPressed,
                icon: widget.isLoading
                    ? SizedBox(
                        width: 20,
                        height: 20,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          valueColor: AlwaysStoppedAnimation(
                            widget.isSecondary
                                ? theme.accentColor
                                : Colors.white,
                          ),
                        ),
                      )
                    : Icon(widget.icon, size: 20),
                label: Text(
                  widget.label,
                  style: TextStyle(
                    fontWeight: FontWeight.w600,
                    letterSpacing: 0.5,
                  ),
                ),
                style: ElevatedButton.styleFrom(
                  backgroundColor: widget.isSecondary
                      ? Colors.transparent
                      : (widget.backgroundColor ?? Colors.transparent),
                  foregroundColor: widget.isSecondary
                      ? theme.accentColor
                      : (widget.foregroundColor ?? Colors.white),
                  elevation: 0,
                  padding:
                      const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
                  shape: RoundedRectangleBorder(
                    borderRadius: theme.borderRadiusMedium,
                  ),
                ),
              ),
            ),
          ),
        );
      },
    );
  }
}

//  Modern Data Grid
class ModernDataGrid extends StatelessWidget {
  final String title;
  final List<Map<String, dynamic>> data;
  final List<String> headers;
  final double? height;

  const ModernDataGrid({
    Key? key,
    required this.title,
    required this.data,
    required this.headers,
    this.height,
  }) : super(key: key);

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
                  color: theme.accentColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                ),
                child: Icon(
                  Icons.table_chart,
                  color: theme.accentColor,
                  size: 20,
                ),
              ),
              const SizedBox(width: 12),
              Text(title, style: theme.headlineMedium),
            ],
          ),
          const SizedBox(height: 16),
          Container(
            height: height ?? 300,
            decoration: BoxDecoration(
              borderRadius: theme.borderRadiusSmall,
              border: Border.all(
                color: theme.isDarkMode
                    ? Colors.white.withOpacity(0.1)
                    : Colors.black.withOpacity(0.1),
              ),
            ),
            child: SingleChildScrollView(
              child: DataTable(
                columnSpacing: 20,
                headingRowColor: MaterialStateProperty.all(
                  theme.accentColor.withOpacity(0.1),
                ),
                columns: headers
                    .map((header) => DataColumn(
                          label: Text(
                            header,
                            style: theme.bodyMedium.copyWith(
                              fontWeight: FontWeight.bold,
                              color: theme.accentColor,
                            ),
                          ),
                        ))
                    .toList(),
                rows: data.map((row) {
                  return DataRow(
                    cells: headers.map((header) {
                      final value = row[header]?.toString() ?? '-';
                      return DataCell(
                        Text(
                          value,
                          style: theme.bodyMedium,
                        ),
                      );
                    }).toList(),
                  );
                }).toList(),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

//  Modern Loading Indicator
class ModernLoadingIndicator extends StatefulWidget {
  final String message;
  final double size;

  const ModernLoadingIndicator({
    Key? key,
    this.message = 'Loading...',
    this.size = 50,
  }) : super(key: key);

  @override
  State<ModernLoadingIndicator> createState() => _ModernLoadingIndicatorState();
}

class _ModernLoadingIndicatorState extends State<ModernLoadingIndicator>
    with TickerProviderStateMixin {
  late AnimationController _rotationController;
  late AnimationController _pulseController;

  @override
  void initState() {
    super.initState();
    _rotationController = AnimationController(
      duration: const Duration(seconds: 2),
      vsync: this,
    )..repeat();

    _pulseController = AnimationController(
      duration: const Duration(milliseconds: 1000),
      vsync: this,
    )..repeat(reverse: true);
  }

  @override
  void dispose() {
    _rotationController.dispose();
    _pulseController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          AnimatedBuilder(
            animation: _rotationController,
            builder: (context, child) {
              return Transform.rotate(
                angle: _rotationController.value * 2 * 3.14159,
                child: Container(
                  width: widget.size,
                  height: widget.size,
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    gradient: LinearGradient(
                      colors: [
                        theme.accentColor,
                        theme.accentColor.withOpacity(0.3),
                        Colors.transparent,
                        Colors.transparent,
                      ],
                      stops: const [0.0, 0.3, 0.7, 1.0],
                    ),
                  ),
                  child: AnimatedBuilder(
                    animation: _pulseController,
                    builder: (context, child) {
                      return Center(
                        child: Container(
                          width: widget.size * 0.3,
                          height: widget.size * 0.3,
                          decoration: BoxDecoration(
                            shape: BoxShape.circle,
                            color: theme.accentColor,
                            boxShadow: [
                              BoxShadow(
                                color: theme.accentColor.withOpacity(
                                  _pulseController.value * 0.5,
                                ),
                                blurRadius: widget.size * 0.2,
                                spreadRadius: widget.size * 0.05,
                              ),
                            ],
                          ),
                        ),
                      );
                    },
                  ),
                ),
              );
            },
          ),
          const SizedBox(height: 24),
          Text(
            widget.message,
            style: theme.bodyLarge.copyWith(
              color: theme.accentColor,
              fontWeight: FontWeight.w500,
            ),
          ),
        ],
      ),
    );
  }
}

//  Modern Stats Card
class ModernStatsCard extends StatelessWidget {
  final String title;
  final String value;
  final String subtitle;
  final IconData icon;
  final Color? color;
  final String? trend;
  final bool showTrend;

  const ModernStatsCard({
    Key? key,
    required this.title,
    required this.value,
    required this.subtitle,
    required this.icon,
    this.color,
    this.trend,
    this.showTrend = false,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);
    final cardColor = color ?? theme.accentColor;

    return ModernGlassCard(
      showGlow: true,
      glowColor: cardColor,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: cardColor.withOpacity(0.1),
                  borderRadius: theme.borderRadiusSmall,
                  border: Border.all(
                    color: cardColor.withOpacity(0.3),
                    width: 1,
                  ),
                ),
                child: Icon(icon, color: cardColor, size: 24),
              ),
              const Spacer(),
              if (showTrend && trend != null)
                Container(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                  decoration: BoxDecoration(
                    color: trend!.startsWith('+')
                        ? theme.successColor.withOpacity(0.1)
                        : theme.errorColor.withOpacity(0.1),
                    borderRadius: theme.borderRadiusSmall,
                  ),
                  child: Text(
                    trend!,
                    style: theme.bodySmall.copyWith(
                      color: trend!.startsWith('+')
                          ? theme.successColor
                          : theme.errorColor,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                ),
            ],
          ),
          const SizedBox(height: 16),
          Text(
            value,
            style: theme.displayMedium.copyWith(
              color: cardColor,
              fontWeight: FontWeight.bold,
            ),
          ),
          const SizedBox(height: 4),
          Text(title, style: theme.headlineMedium),
          const SizedBox(height: 8),
          Text(
            subtitle,
            style: theme.bodySmall.copyWith(
              color: theme.isDarkMode
                  ? Colors.white.withOpacity(0.6)
                  : Colors.black.withOpacity(0.6),
            ),
          ),
        ],
      ),
    );
  }
}

// ️ Modern Toggle Switch
class ModernToggleSwitch extends StatelessWidget {
  final bool value;
  final ValueChanged<bool> onChanged;
  final String label;
  final IconData? icon;

  const ModernToggleSwitch({
    Key? key,
    required this.value,
    required this.onChanged,
    required this.label,
    this.icon,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    return Row(
      children: [
        if (icon != null) ...[
          Icon(icon, color: theme.accentColor, size: 20),
          const SizedBox(width: 12),
        ],
        Expanded(
          child: Text(label, style: theme.bodyLarge),
        ),
        GestureDetector(
          onTap: () => onChanged(!value),
          child: AnimatedContainer(
            duration: const Duration(milliseconds: 200),
            width: 50,
            height: 28,
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(14),
              color: value ? theme.accentColor : Colors.grey.shade400,
              boxShadow: value ? theme.neonGlow : null,
            ),
            child: AnimatedAlign(
              duration: const Duration(milliseconds: 200),
              alignment: value ? Alignment.centerRight : Alignment.centerLeft,
              child: Container(
                width: 24,
                height: 24,
                margin: const EdgeInsets.all(2),
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: Colors.white,
                  boxShadow: theme.elevationSmall,
                ),
              ),
            ),
          ),
        ),
      ],
    );
  }
}

//  Modern Settings Section
class ModernSettingsSection extends StatelessWidget {
  final String title;
  final List<Widget> children;
  final IconData? icon;

  const ModernSettingsSection({
    Key? key,
    required this.title,
    required this.children,
    this.icon,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final theme = Provider.of<ThemeService>(context);

    return ModernGlassCard(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              if (icon != null) ...[
                Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    color: theme.accentColor.withOpacity(0.1),
                    borderRadius: theme.borderRadiusSmall,
                  ),
                  child: Icon(icon, color: theme.accentColor, size: 20),
                ),
                const SizedBox(width: 12),
              ],
              Text(title, style: theme.headlineMedium),
            ],
          ),
          const SizedBox(height: 20),
          ...children.map((child) => Padding(
                padding: const EdgeInsets.only(bottom: 16),
                child: child,
              )),
        ],
      ),
    );
  }
}
