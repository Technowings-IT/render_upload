// services/theme_service.dart - Modern Robotic Fleet Management Theme
import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';

class ThemeService extends ChangeNotifier {
  static final ThemeService _instance = ThemeService._internal();
  factory ThemeService() => _instance;
  ThemeService._internal();

  bool _isDarkMode = true; // Default to dark for robotic feel
  bool get isDarkMode => _isDarkMode;

  // Modern Color Palettes
  static const _primaryBlue = Color(0xFF0EA5E9);
  static const _accentCyan = Color(0xFF06B6D4);
  static const _neonGreen = Color(0xFF10B981);
  static const _warningAmber = Color(0xFFF59E0B);
  static const _errorRed = Color(0xFFEF4444);
  static const _purpleAccent = Color(0xFF8B5CF6);

  // Dark Theme Colors
  static const _darkBackground = Color(0xFF0F0F23);
  static const _darkSurface = Color(0xFF1E1E2E);
  static const _darkCard = Color(0xFF262640);
  static const _darkBorder = Color(0xFF383851);

  // Light Theme Colors
  static const _lightBackground = Color(0xFFF8FAFC);
  static const _lightSurface = Color(0xFFFFFFFF);
  static const _lightCard = Color(0xFFF1F5F9);
  static const _lightBorder = Color(0xFFE2E8F0);

  // Modern Typography
  static const _primaryFont = 'Inter';
  static const _displayFont = 'Orbitron'; // Robotic font
  static const _monoFont = 'JetBrains Mono'; // Technical data

  ThemeData get lightTheme => ThemeData(
        useMaterial3: true,
        brightness: Brightness.light,
        fontFamily: _primaryFont,

        // Color Scheme
        colorScheme: ColorScheme.light(
          primary: _primaryBlue,
          primaryContainer: _primaryBlue.withOpacity(0.1),
          secondary: _accentCyan,
          secondaryContainer: _accentCyan.withOpacity(0.1),
          tertiary: _neonGreen,
          surface: _lightSurface,
          background: _lightBackground,
          error: _errorRed,
          onPrimary: Colors.white,
          onSecondary: Colors.white,
          onSurface: const Color(0xFF1E293B),
          onBackground: const Color(0xFF0F172A),
          outline: _lightBorder,
        ),

        // Card Theme
        cardTheme: CardThemeData(
          elevation: 4,
          shadowColor: Colors.black.withOpacity(0.1),
          shape:
              RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
          color: _lightCard,
          surfaceTintColor: Colors.transparent,
        ),

        // App Bar Theme
        appBarTheme: AppBarTheme(
          elevation: 0,
          backgroundColor: _lightSurface,
          foregroundColor: const Color(0xFF0F172A),
          surfaceTintColor: Colors.transparent,
          centerTitle: true,
          titleTextStyle: TextStyle(
            fontFamily: _displayFont,
            fontWeight: FontWeight.bold,
            fontSize: 20,
            color: const Color(0xFF0F172A),
            letterSpacing: 0.5,
          ),
        ),

        // Bottom Navigation
        bottomNavigationBarTheme: BottomNavigationBarThemeData(
          backgroundColor: _lightSurface,
          selectedItemColor: _primaryBlue,
          unselectedItemColor: const Color(0xFF64748B),
          type: BottomNavigationBarType.fixed,
          elevation: 8,
          selectedLabelStyle: const TextStyle(fontWeight: FontWeight.w600),
        ),

        // Elevated Button
        elevatedButtonTheme: ElevatedButtonThemeData(
          style: ElevatedButton.styleFrom(
            elevation: 2,
            backgroundColor: _primaryBlue,
            foregroundColor: Colors.white,
            shape:
                RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
            padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
            textStyle: const TextStyle(
              fontWeight: FontWeight.w600,
              fontSize: 16,
              letterSpacing: 0.5,
            ),
          ),
        ),

        // Text Button
        textButtonTheme: TextButtonThemeData(
          style: TextButton.styleFrom(
            foregroundColor: _primaryBlue,
            shape:
                RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
            textStyle: const TextStyle(fontWeight: FontWeight.w600),
          ),
        ),

        // Input Decoration
        inputDecorationTheme: InputDecorationTheme(
          filled: true,
          fillColor: _lightCard,
          border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(12),
            borderSide: BorderSide(color: _lightBorder),
          ),
          enabledBorder: OutlineInputBorder(
            borderRadius: BorderRadius.circular(12),
            borderSide: BorderSide(color: _lightBorder),
          ),
          focusedBorder: OutlineInputBorder(
            borderRadius: BorderRadius.circular(12),
            borderSide: BorderSide(color: _primaryBlue, width: 2),
          ),
          contentPadding: const EdgeInsets.all(16),
        ),

        // Tab Bar
        tabBarTheme: TabBarThemeData(
          labelColor: _primaryBlue,
          unselectedLabelColor: const Color(0xFF64748B),
          indicator: BoxDecoration(
            borderRadius: BorderRadius.circular(8),
            color: _primaryBlue.withOpacity(0.1),
          ),
          labelStyle: const TextStyle(fontWeight: FontWeight.w600),
        ),
      );

  ThemeData get darkTheme => ThemeData(
        useMaterial3: true,
        brightness: Brightness.dark,
        fontFamily: _primaryFont,

        // Color Scheme
        colorScheme: ColorScheme.dark(
          primary: _primaryBlue,
          primaryContainer: _primaryBlue.withOpacity(0.2),
          secondary: _accentCyan,
          secondaryContainer: _accentCyan.withOpacity(0.2),
          tertiary: _neonGreen,
          surface: _darkSurface,
          background: _darkBackground,
          error: _errorRed,
          onPrimary: Colors.black,
          onSecondary: Colors.black,
          onSurface: const Color(0xFFE2E8F0),
          onBackground: const Color(0xFFF1F5F9),
          outline: _darkBorder,
        ),

        // Card Theme
        cardTheme: CardThemeData(
          elevation: 8,
          shadowColor: Colors.black.withOpacity(0.3),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(16),
            side: BorderSide(color: _darkBorder, width: 1),
          ),
          color: _darkCard,
          surfaceTintColor: Colors.transparent,
        ),

        // App Bar Theme
        appBarTheme: AppBarTheme(
          elevation: 0,
          backgroundColor: _darkSurface,
          foregroundColor: const Color(0xFFF1F5F9),
          surfaceTintColor: Colors.transparent,
          centerTitle: true,
          titleTextStyle: TextStyle(
            fontFamily: _displayFont,
            fontWeight: FontWeight.bold,
            fontSize: 20,
            color: const Color(0xFFF1F5F9),
            letterSpacing: 0.5,
          ),
        ),

        // Bottom Navigation
        bottomNavigationBarTheme: BottomNavigationBarThemeData(
          backgroundColor: _darkSurface,
          selectedItemColor: _accentCyan,
          unselectedItemColor: const Color(0xFF64748B),
          type: BottomNavigationBarType.fixed,
          elevation: 8,
          selectedLabelStyle: const TextStyle(fontWeight: FontWeight.w600),
        ),

        // Elevated Button
        elevatedButtonTheme: ElevatedButtonThemeData(
          style: ElevatedButton.styleFrom(
            elevation: 4,
            backgroundColor: _primaryBlue,
            foregroundColor: Colors.white,
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(12),
              side: BorderSide(color: _primaryBlue.withOpacity(0.3)),
            ),
            padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
            textStyle: const TextStyle(
              fontWeight: FontWeight.w600,
              fontSize: 16,
              letterSpacing: 0.5,
            ),
          ),
        ),

        // Text Button
        textButtonTheme: TextButtonThemeData(
          style: TextButton.styleFrom(
            foregroundColor: _accentCyan,
            shape:
                RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
            textStyle: const TextStyle(fontWeight: FontWeight.w600),
          ),
        ),

        // Input Decoration
        inputDecorationTheme: InputDecorationTheme(
          filled: true,
          fillColor: _darkCard,
          border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(12),
            borderSide: BorderSide(color: _darkBorder),
          ),
          enabledBorder: OutlineInputBorder(
            borderRadius: BorderRadius.circular(12),
            borderSide: BorderSide(color: _darkBorder),
          ),
          focusedBorder: OutlineInputBorder(
            borderRadius: BorderRadius.circular(12),
            borderSide: BorderSide(color: _accentCyan, width: 2),
          ),
          contentPadding: const EdgeInsets.all(16),
        ),

        // Tab Bar
        tabBarTheme: TabBarThemeData(
          labelColor: _accentCyan,
          unselectedLabelColor: const Color(0xFF64748B),
          indicator: BoxDecoration(
            borderRadius: BorderRadius.circular(8),
            color: _accentCyan.withOpacity(0.2),
            border: Border.all(color: _accentCyan.withOpacity(0.3)),
          ),
          labelStyle: const TextStyle(fontWeight: FontWeight.w600),
        ),
      );

  // Custom Colors for Fleet Management
  Color get successColor => _neonGreen;
  Color get warningColor => _warningAmber;
  Color get errorColor => _errorRed;
  Color get infoColor => _primaryBlue;
  Color get accentColor => _isDarkMode ? _accentCyan : _primaryBlue;

  // Status Colors
  Color get onlineColor => _neonGreen;
  Color get offlineColor => _errorRed;
  Color get pendingColor => _warningAmber;
  Color get activeColor => _primaryBlue;
  Color get completedColor => _neonGreen;

  // Gradient Definitions
  LinearGradient get primaryGradient => LinearGradient(
        colors: _isDarkMode
            ? [_primaryBlue, _accentCyan]
            : [_primaryBlue, _primaryBlue.withOpacity(0.8)],
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
      );

  LinearGradient get cardGradient => LinearGradient(
        colors: _isDarkMode
            ? [_darkCard, _darkCard.withOpacity(0.8)]
            : [_lightCard, _lightSurface],
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
      );

  LinearGradient get backgroundGradient => LinearGradient(
        colors: _isDarkMode
            ? [_darkBackground, _darkSurface]
            : [_lightBackground, _lightSurface],
        begin: Alignment.topCenter,
        end: Alignment.bottomCenter,
      );

  // Text Styles
  TextStyle get displayLarge => TextStyle(
        fontFamily: _displayFont,
        fontWeight: FontWeight.bold,
        fontSize: 32,
        color: _isDarkMode ? const Color(0xFFF1F5F9) : const Color(0xFF0F172A),
        letterSpacing: 0.5,
      );

  TextStyle get displayMedium => TextStyle(
        fontFamily: _displayFont,
        fontWeight: FontWeight.w600,
        fontSize: 24,
        color: _isDarkMode ? const Color(0xFFF1F5F9) : const Color(0xFF0F172A),
        letterSpacing: 0.3,
      );

  TextStyle get headlineLarge => TextStyle(
        fontFamily: _primaryFont,
        fontWeight: FontWeight.bold,
        fontSize: 20,
        color: _isDarkMode ? const Color(0xFFE2E8F0) : const Color(0xFF1E293B),
      );

  TextStyle get headlineMedium => TextStyle(
        fontFamily: _primaryFont,
        fontWeight: FontWeight.w600,
        fontSize: 18,
        color: _isDarkMode ? const Color(0xFFE2E8F0) : const Color(0xFF1E293B),
      );

  TextStyle get bodyLarge => TextStyle(
        fontFamily: _primaryFont,
        fontWeight: FontWeight.normal,
        fontSize: 16,
        color: _isDarkMode ? const Color(0xFFCBD5E1) : const Color(0xFF475569),
      );

  TextStyle get bodyMedium => TextStyle(
        fontFamily: _primaryFont,
        fontWeight: FontWeight.normal,
        fontSize: 14,
        color: _isDarkMode ? const Color(0xFFCBD5E1) : const Color(0xFF475569),
      );

  TextStyle get bodySmall => TextStyle(
        fontFamily: _primaryFont,
        fontWeight: FontWeight.normal,
        fontSize: 12,
        color: _isDarkMode ? const Color(0xFF94A3B8) : const Color(0xFF64748B),
      );

  TextStyle get monospace => TextStyle(
        fontFamily: _monoFont,
        fontWeight: FontWeight.normal,
        fontSize: 14,
        color: _isDarkMode ? const Color(0xFFCBD5E1) : const Color(0xFF475569),
      );

  // Border Radius
  BorderRadius get borderRadiusSmall => BorderRadius.circular(8);
  BorderRadius get borderRadiusMedium => BorderRadius.circular(12);
  BorderRadius get borderRadiusLarge => BorderRadius.circular(16);
  BorderRadius get borderRadiusXLarge => BorderRadius.circular(24);

  // Shadows
  List<BoxShadow> get elevationSmall => [
        BoxShadow(
          color: Colors.black.withOpacity(_isDarkMode ? 0.3 : 0.1),
          blurRadius: 4,
          offset: const Offset(0, 2),
        ),
      ];

  List<BoxShadow> get elevationMedium => [
        BoxShadow(
          color: Colors.black.withOpacity(_isDarkMode ? 0.4 : 0.15),
          blurRadius: 8,
          offset: const Offset(0, 4),
        ),
      ];

  List<BoxShadow> get elevationLarge => [
        BoxShadow(
          color: Colors.black.withOpacity(_isDarkMode ? 0.5 : 0.2),
          blurRadius: 16,
          offset: const Offset(0, 8),
        ),
      ];

  // Glow Effects
  List<BoxShadow> get neonGlow => [
        BoxShadow(
          color: accentColor.withOpacity(0.3),
          blurRadius: 12,
          spreadRadius: 2,
        ),
      ];

  ThemeData get currentTheme => _isDarkMode ? darkTheme : lightTheme;

  Future<void> loadTheme() async {
    final prefs = await SharedPreferences.getInstance();
    _isDarkMode = prefs.getBool('isDarkMode') ?? true; // Default dark
    notifyListeners();
  }

  Future<void> toggleTheme() async {
    _isDarkMode = !_isDarkMode;
    final prefs = await SharedPreferences.getInstance();
    await prefs.setBool('isDarkMode', _isDarkMode);
    notifyListeners();
  }

  Future<void> setTheme(bool isDark) async {
    _isDarkMode = isDark;
    final prefs = await SharedPreferences.getInstance();
    await prefs.setBool('isDarkMode', _isDarkMode);
    notifyListeners();
  }

  // Glass Morphism Effect
  BoxDecoration get glassMorphism => BoxDecoration(
        borderRadius: borderRadiusMedium,
        gradient: LinearGradient(
          colors: _isDarkMode
              ? [
                  Colors.white.withOpacity(0.1),
                  Colors.white.withOpacity(0.05),
                ]
              : [
                  Colors.white.withOpacity(0.8),
                  Colors.white.withOpacity(0.4),
                ],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        border: Border.all(
          color: _isDarkMode
              ? Colors.white.withOpacity(0.2)
              : Colors.white.withOpacity(0.6),
          width: 1,
        ),
        boxShadow: elevationMedium,
      );

  // Fleet Status Decorations
  BoxDecoration statusDecoration(String status) {
    Color color;
    switch (status.toLowerCase()) {
      case 'online':
      case 'connected':
      case 'active':
        color = onlineColor;
        break;
      case 'offline':
      case 'disconnected':
      case 'failed':
        color = offlineColor;
        break;
      case 'pending':
      case 'waiting':
        color = pendingColor;
        break;
      case 'completed':
      case 'success':
        color = completedColor;
        break;
      default:
        color = _isDarkMode ? const Color(0xFF64748B) : const Color(0xFF94A3B8);
    }

    return BoxDecoration(
      color: color.withOpacity(0.1),
      borderRadius: borderRadiusSmall,
      border: Border.all(color: color.withOpacity(0.3)),
    );
  }
}
