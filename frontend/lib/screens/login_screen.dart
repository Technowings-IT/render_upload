// enhanced_login_screen.dart
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'connect_screen.dart';
import 'signup_screen.dart';
import 'package:shared_preferences/shared_preferences.dart';

class EnhancedLoginScreen extends StatefulWidget {
  const EnhancedLoginScreen({
    super.key,
  });

  @override
  State<EnhancedLoginScreen> createState() => _EnhancedLoginScreenState();
}

class _EnhancedLoginScreenState extends State<EnhancedLoginScreen>
    with TickerProviderStateMixin {
  bool _obscurePassword = true;
  final TextEditingController _usernameController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();

  late AnimationController _fadeController;
  late AnimationController _slideController;
  late AnimationController _pulseController;
  late AnimationController _floatController;
  late Animation<double> _fadeAnimation;
  late Animation<Offset> _slideAnimation;
  late Animation<double> _pulseAnimation;
  late Animation<double> _floatAnimation;

  bool _isLoading = false;
  bool _rememberMe = false;
  bool _isDarkMode = false; //  Follows system theme only

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _loadSavedCredentials();
    _detectSystemTheme();
  }

  void _detectSystemTheme() {
    //  Automatically follow system theme preference only
    final brightness =
        WidgetsBinding.instance.platformDispatcher.platformBrightness;
    setState(() {
      _isDarkMode = brightness == Brightness.dark;
    });

    //  Listen for system theme changes
    WidgetsBinding.instance.platformDispatcher.onPlatformBrightnessChanged =
        () {
      final newBrightness =
          WidgetsBinding.instance.platformDispatcher.platformBrightness;
      setState(() {
        _isDarkMode = newBrightness == Brightness.dark;
      });
    };
  }

  void _initializeAnimations() {
    _fadeController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );

    _slideController = AnimationController(
      duration: const Duration(milliseconds: 1200),
      vsync: this,
    );

    _pulseController = AnimationController(
      duration: const Duration(milliseconds: 2000),
      vsync: this,
    );

    _floatController = AnimationController(
      duration: const Duration(seconds: 3),
      vsync: this,
    );

    _fadeAnimation = Tween<double>(
      begin: 0.0,
      end: 1.0,
    ).animate(CurvedAnimation(
      parent: _fadeController,
      curve: Curves.easeInOut,
    ));

    _slideAnimation = Tween<Offset>(
      begin: const Offset(0, 0.3),
      end: Offset.zero,
    ).animate(CurvedAnimation(
      parent: _slideController,
      curve: Curves.easeOutCubic,
    ));

    _pulseAnimation = Tween<double>(
      begin: 1.0,
      end: 1.05,
    ).animate(CurvedAnimation(
      parent: _pulseController,
      curve: Curves.easeInOut,
    ));

    _floatAnimation = Tween<double>(
      begin: -10,
      end: 10,
    ).animate(CurvedAnimation(
      parent: _floatController,
      curve: Curves.easeInOut,
    ));

    _fadeController.forward();
    _slideController.forward();
    _pulseController.repeat(reverse: true);
    _floatController.repeat(reverse: true);
  }

  Future<void> _loadSavedCredentials() async {
    final prefs = await SharedPreferences.getInstance();
    final savedUsername = prefs.getString('remembered_username');
    final savedPassword = prefs.getString('remembered_password');
    final rememberMe = prefs.getBool('remember_me') ?? false;

    if (rememberMe && savedUsername != null && savedPassword != null) {
      setState(() {
        _usernameController.text = savedUsername;
        _passwordController.text = savedPassword;
        _rememberMe = rememberMe;
      });
    }
  }

  @override
  void dispose() {
    _fadeController.dispose();
    _slideController.dispose();
    _pulseController.dispose();
    _floatController.dispose();
    _usernameController.dispose();
    _passwordController.dispose();
    super.dispose();
  }

  ThemeColors _getThemeColors() {
    return _isDarkMode ? ThemeColors.dark() : ThemeColors.light();
  }

  DeviceType _getDeviceType(BuildContext context) {
    final screenWidth = MediaQuery.of(context).size.width;
    final screenHeight = MediaQuery.of(context).size.height;
    final shortestSide = MediaQuery.of(context).size.shortestSide;
    final aspectRatio = screenWidth / screenHeight;

    // Enhanced device detection with better breakpoints
    if (screenWidth >= 1200 && shortestSide >= 800) {
      return DeviceType.desktop;
    }
    if (screenWidth >= 768 && shortestSide >= 600) {
      return DeviceType.laptop;
    }
    if (shortestSide >= 600 || (screenWidth >= 768 && aspectRatio > 1.2)) {
      return DeviceType.tablet;
    }
    return DeviceType.mobile;
  }

  ResponsiveDimensions _getResponsiveDimensions(BuildContext context) {
    final deviceType = _getDeviceType(context);
    final screenHeight = MediaQuery.of(context).size.height;
    final screenWidth = MediaQuery.of(context).size.width;
    final shortestSide = MediaQuery.of(context).size.shortestSide;
    final isCompact = screenHeight < 700;
    final isExtraCompact = screenHeight < 600;
    final isUltraCompact = screenHeight < 500;
    final isLandscape = screenWidth > screenHeight;

    // Calculate responsive scale factor
    final baseWidth = 375.0; // iPhone 11 Pro width as base
    final scaleFactor = (screenWidth / baseWidth).clamp(0.8, 2.0);

    switch (deviceType) {
      case DeviceType.desktop:
        final isLargeDesktop = screenWidth >= 1600;
        return ResponsiveDimensions(
          horizontalPadding: isLargeDesktop ? 120.0 : 80.0,
          verticalPadding: isCompact ? 30.0 : 60.0,
          logoSize: (isCompact ? 260.0 : 300.0) * scaleFactor.clamp(0.9, 1.3),
          logoHeight: (isCompact ? 110.0 : 130.0) * scaleFactor.clamp(0.9, 1.3),
          titleSize: (isCompact ? 36.0 : 42.0) * scaleFactor.clamp(0.9, 1.2),
          subtitleSize: (isCompact ? 17.0 : 19.0) * scaleFactor.clamp(0.9, 1.1),
          cardPadding: 48.0,
          textFieldHeight: isCompact ? 68.0 : 74.0,
          buttonHeight: isCompact ? 72.0 : 78.0,
          buttonTextSize:
              (isCompact ? 17.0 : 19.0) * scaleFactor.clamp(0.9, 1.1),
          spacing: isCompact ? 40.0 : 55.0,
          maxWidth: isLargeDesktop ? 700.0 : 600.0,
          borderRadius: 22.0,
          iconSize: (isCompact ? 25.0 : 27.0) * scaleFactor.clamp(0.9, 1.1),
        );

      case DeviceType.laptop:
        return ResponsiveDimensions(
          horizontalPadding: 50.0,
          verticalPadding: isCompact ? 25.0 : 40.0,
          logoSize: (isCompact ? 240.0 : 280.0) * scaleFactor.clamp(0.9, 1.2),
          logoHeight: (isCompact ? 100.0 : 120.0) * scaleFactor.clamp(0.9, 1.2),
          titleSize: (isCompact ? 32.0 : 36.0) * scaleFactor.clamp(0.9, 1.1),
          subtitleSize: (isCompact ? 15.0 : 17.0) * scaleFactor.clamp(0.9, 1.1),
          cardPadding: 36.0,
          textFieldHeight: isCompact ? 62.0 : 68.0,
          buttonHeight: isCompact ? 64.0 : 70.0,
          buttonTextSize:
              (isCompact ? 16.0 : 18.0) * scaleFactor.clamp(0.9, 1.1),
          spacing: isCompact ? 35.0 : 45.0,
          maxWidth: 520.0,
          borderRadius: 18.0,
          iconSize: (isCompact ? 23.0 : 25.0) * scaleFactor.clamp(0.9, 1.1),
        );

      case DeviceType.tablet:
        final tabletScaleFactor = (shortestSide / 600.0).clamp(0.8, 1.2);
        return ResponsiveDimensions(
          horizontalPadding: isLandscape
              ? (screenWidth * 0.08).clamp(40.0, 80.0)
              : (screenWidth * 0.06).clamp(24.0, 40.0),
          verticalPadding: (screenHeight * 0.03).clamp(16.0, 32.0),
          logoSize: (isCompact ? 220.0 : 260.0) * tabletScaleFactor,
          logoHeight: (isCompact ? 90.0 : 110.0) * tabletScaleFactor,
          titleSize: (isCompact ? 28.0 : 32.0) * tabletScaleFactor,
          subtitleSize: (isCompact ? 14.0 : 16.0) * tabletScaleFactor,
          cardPadding: 28.0 * tabletScaleFactor,
          textFieldHeight: (isCompact ? 56.0 : 62.0) * tabletScaleFactor,
          buttonHeight: (isCompact ? 58.0 : 64.0) * tabletScaleFactor,
          buttonTextSize: (isCompact ? 15.0 : 17.0) * tabletScaleFactor,
          spacing: (isCompact ? 28.0 : 35.0) * tabletScaleFactor,
          maxWidth: isLandscape
              ? (screenWidth * 0.6).clamp(400.0, 600.0)
              : (screenWidth * 0.9).clamp(300.0, 500.0),
          borderRadius: 16.0,
          iconSize: (isCompact ? 21.0 : 23.0) * tabletScaleFactor,
        );

      case DeviceType.mobile:
        final mobileScaleFactor = (screenWidth / 375.0).clamp(0.75, 1.25);
        final isVerySmall = screenWidth < 340;
        final isTinyScreen = screenWidth < 320;

        return ResponsiveDimensions(
          horizontalPadding: isTinyScreen ? 12.0 : (isVerySmall ? 16.0 : 20.0),
          verticalPadding: isUltraCompact
              ? 8.0
              : (isExtraCompact ? 12.0 : (isCompact ? 16.0 : 20.0)),
          logoSize: (isUltraCompact
                  ? 160.0
                  : (isExtraCompact ? 180.0 : (isCompact ? 200.0 : 240.0))) *
              mobileScaleFactor,
          logoHeight: (isUltraCompact
                  ? 65.0
                  : (isExtraCompact ? 75.0 : (isCompact ? 85.0 : 100.0))) *
              mobileScaleFactor,
          titleSize: (isUltraCompact
                  ? 20.0
                  : (isExtraCompact ? 22.0 : (isCompact ? 24.0 : 28.0))) *
              mobileScaleFactor,
          subtitleSize: (isUltraCompact
                  ? 11.0
                  : (isExtraCompact ? 12.0 : (isCompact ? 13.0 : 15.0))) *
              mobileScaleFactor,
          cardPadding: (isTinyScreen ? 16.0 : (isVerySmall ? 18.0 : 22.0)) *
              mobileScaleFactor,
          textFieldHeight: (isUltraCompact
                  ? 44.0
                  : (isExtraCompact ? 48.0 : (isCompact ? 50.0 : 54.0))) *
              mobileScaleFactor,
          buttonHeight: (isUltraCompact
                  ? 44.0
                  : (isExtraCompact ? 48.0 : (isCompact ? 50.0 : 54.0))) *
              mobileScaleFactor,
          buttonTextSize: (isUltraCompact
                  ? 12.0
                  : (isExtraCompact ? 13.0 : (isCompact ? 14.0 : 15.0))) *
              mobileScaleFactor,
          spacing: (isUltraCompact
                  ? 16.0
                  : (isExtraCompact ? 18.0 : (isCompact ? 22.0 : 26.0))) *
              mobileScaleFactor,
          maxWidth: double.infinity,
          borderRadius: 12.0,
          iconSize: (isUltraCompact
                  ? 16.0
                  : (isExtraCompact ? 18.0 : (isCompact ? 19.0 : 21.0))) *
              mobileScaleFactor,
        );
    }
  }

  Future<void> _handleLogin() async {
    if (_usernameController.text.isEmpty || _passwordController.text.isEmpty) {
      _showSnackBar('Please fill all fields', false);
      return;
    }

    setState(() {
      _isLoading = true;
    });

    HapticFeedback.mediumImpact();
    await Future.delayed(const Duration(milliseconds: 800));

    try {
      final prefs = await SharedPreferences.getInstance();

      // Save credentials if remember me is checked
      if (_rememberMe) {
        await prefs.setString('remembered_username', _usernameController.text);
        await prefs.setString('remembered_password', _passwordController.text);
        await prefs.setBool('remember_me', true);
      } else {
        await prefs.remove('remembered_username');
        await prefs.remove('remembered_password');
        await prefs.setBool('remember_me', false);
      }

      final savedUsername = prefs.getString('saved_username');
      final savedPassword = prefs.getString('saved_password');

      if ((_usernameController.text == 'TWAIPL' &&
              _passwordController.text == '1234') ||
          (_usernameController.text == savedUsername &&
              _passwordController.text == savedPassword)) {
        HapticFeedback.lightImpact();

        Navigator.push(
          context,
          PageRouteBuilder(
            pageBuilder: (context, animation, secondaryAnimation) =>
                ConnectScreen(),
            transitionsBuilder:
                (context, animation, secondaryAnimation, child) {
              return SlideTransition(
                position: Tween<Offset>(
                  begin: const Offset(1.0, 0.0),
                  end: Offset.zero,
                ).animate(CurvedAnimation(
                  parent: animation,
                  curve: Curves.easeInOut,
                )),
                child: child,
              );
            },
            transitionDuration: const Duration(milliseconds: 600),
          ),
        );
      } else {
        HapticFeedback.heavyImpact();
        _showSnackBar('Invalid username or password', false);
      }
    } catch (e) {
      _showSnackBar('Login failed. Please try again.', false);
    } finally {
      if (mounted) {
        setState(() {
          _isLoading = false;
        });
      }
    }
  }

  void _showSnackBar(String message, bool isSuccess) {
    final dimensions = _getResponsiveDimensions(context);
    final colors = _getThemeColors();

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Row(
          children: [
            Icon(
              isSuccess ? Icons.check_circle_rounded : Icons.error_rounded,
              color: Colors.white,
              size: dimensions.iconSize,
            ),
            SizedBox(width: dimensions.spacing / 4),
            Expanded(
              child: Text(
                message,
                style: TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.w600,
                  fontSize: dimensions.subtitleSize - 2,
                ),
              ),
            ),
          ],
        ),
        backgroundColor: isSuccess ? colors.success : colors.error,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(dimensions.borderRadius / 2),
        ),
        duration: const Duration(seconds: 3),
        margin: EdgeInsets.all(dimensions.horizontalPadding / 2),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final dimensions = _getResponsiveDimensions(context);
    final deviceType = _getDeviceType(context);
    final colors = _getThemeColors();

    return Scaffold(
      backgroundColor: colors.background,
      body: Container(
        width: double.infinity,
        height: double.infinity,
        decoration: BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: colors.backgroundGradient,
            stops: const [0.0, 0.5, 1.0],
          ),
        ),
        child: Stack(
          children: [
            // Animated background pattern
            if (deviceType != DeviceType.mobile)
              ...List.generate(
                deviceType == DeviceType.desktop ? 10 : 6,
                (index) => _buildAnimatedBackground(index, dimensions, colors),
              ),

            // Glass morphism overlay for desktop/laptop
            if (deviceType == DeviceType.desktop ||
                deviceType == DeviceType.laptop)
              Positioned.fill(
                child: Container(
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      begin: Alignment.topCenter,
                      end: Alignment.bottomCenter,
                      colors: [
                        colors.overlay.withOpacity(0.02),
                        colors.overlay.withOpacity(0.05),
                      ],
                    ),
                  ),
                ),
              ),

            // Main content - NO THEME TOGGLE BUTTON
            SafeArea(
              child: LayoutBuilder(
                builder: (context, constraints) {
                  final availableHeight = constraints.maxHeight;
                  final availableWidth = constraints.maxWidth;

                  return Center(
                    child: ConstrainedBox(
                      constraints: BoxConstraints(
                        maxWidth: dimensions.maxWidth.clamp(
                          availableWidth * 0.9,
                          double.infinity,
                        ),
                        maxHeight: availableHeight,
                      ),
                      child: SingleChildScrollView(
                        physics: const BouncingScrollPhysics(),
                        padding: EdgeInsets.symmetric(
                          horizontal: dimensions.horizontalPadding.clamp(
                            8.0,
                            availableWidth * 0.1,
                          ),
                          vertical: dimensions.verticalPadding.clamp(
                            8.0,
                            availableHeight * 0.05,
                          ),
                        ),
                        child: AnimatedBuilder(
                          animation: _fadeAnimation,
                          builder: (context, child) {
                            return FadeTransition(
                              opacity: _fadeAnimation,
                              child: SlideTransition(
                                position: _slideAnimation,
                                child: ConstrainedBox(
                                  constraints: BoxConstraints(
                                    minHeight: availableHeight -
                                        (dimensions.verticalPadding * 2)
                                            .clamp(16.0, availableHeight * 0.1),
                                  ),
                                  child: IntrinsicHeight(
                                    child: Column(
                                      mainAxisAlignment:
                                          MainAxisAlignment.center,
                                      crossAxisAlignment:
                                          CrossAxisAlignment.stretch,
                                      children: [
                                        // Flexible spacer for top
                                        if (availableHeight > 600)
                                          const Spacer(flex: 1),

                                        _buildLogoSection(dimensions, colors),

                                        SizedBox(
                                            height: (dimensions.spacing * 1.2)
                                                .clamp(16.0, 40.0)),

                                        _buildLoginForm(dimensions, colors),

                                        SizedBox(
                                            height: (dimensions.spacing * 0.6)
                                                .clamp(8.0, 20.0)),

                                        _buildRememberForgot(
                                            dimensions, colors),

                                        SizedBox(
                                            height: (dimensions.spacing * 0.8)
                                                .clamp(12.0, 24.0)),

                                        _buildLoginButton(dimensions, colors),

                                        SizedBox(
                                            height: (dimensions.spacing * 0.8)
                                                .clamp(12.0, 24.0)),

                                        _buildDivider(dimensions, colors),

                                        SizedBox(
                                            height: (dimensions.spacing * 0.8)
                                                .clamp(12.0, 24.0)),

                                        _buildSocialLogin(
                                            dimensions, colors, deviceType),

                                        SizedBox(
                                            height: (dimensions.spacing)
                                                .clamp(16.0, 32.0)),

                                        _buildSignUpSection(dimensions, colors),

                                        // Flexible spacer for bottom
                                        if (availableHeight > 600)
                                          const Spacer(flex: 1),

                                        // Bottom padding for very small screens
                                        if (availableHeight <= 600)
                                          SizedBox(
                                              height: dimensions.spacing / 2),
                                      ],
                                    ),
                                  ),
                                ),
                              ),
                            );
                          },
                        ),
                      ),
                    ),
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildAnimatedBackground(
      int index, ResponsiveDimensions dimensions, ThemeColors colors) {
    final random = (index * 54321) % 1000;
    final screenSize = MediaQuery.of(context).size;
    final left = (random % 100) / 100 * screenSize.width;
    final top = ((random ~/ 100) % 100) / 100 * screenSize.height;
    final baseSize = dimensions.spacing * 1.5;
    final size = baseSize + (random % 40);

    return Positioned(
      left: left,
      top: top,
      child: AnimatedBuilder(
        animation: _floatController,
        builder: (context, child) {
          return Transform.translate(
            offset: Offset(
              _floatAnimation.value * (index.isEven ? 1 : -1),
              _floatAnimation.value * (index.isOdd ? 1 : -1),
            ),
            child: Container(
              width: size,
              height: size,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                gradient: RadialGradient(
                  colors: [
                    colors.primary.withOpacity(0.08),
                    colors.primary.withOpacity(0.04),
                    Colors.transparent,
                  ],
                ),
              ),
            ),
          );
        },
      ),
    );
  }

  Widget _buildLogoSection(
      ResponsiveDimensions dimensions, ThemeColors colors) {
    final screenWidth = MediaQuery.of(context).size.width;
    final screenHeight = MediaQuery.of(context).size.height;
    final isVerySmallScreen = screenWidth < 340 || screenHeight < 500;

    //  Select logo based on theme
    // Dark mode: uses 'assets/TW_WHITE.png' (white TW logo)
    // Light mode: uses 'assets/login_logo.png' (standard logo)
    final logoAsset =
        _isDarkMode ? 'assets/TW_WHITE.png' : 'assets/login_logo.png';

    return Column(
      children: [
        AnimatedBuilder(
          animation: _pulseAnimation,
          builder: (context, child) {
            return Transform.scale(
              scale: _pulseAnimation.value,
              child: Container(
                padding: EdgeInsets.all(
                    (dimensions.cardPadding / 2).clamp(12.0, 24.0)),
                decoration: BoxDecoration(
                  borderRadius: BorderRadius.circular(
                      20.0), //  Use rounded rectangle instead of circle
                  gradient: RadialGradient(
                    colors: [
                      colors.primary.withOpacity(0.12),
                      colors.primary.withOpacity(0.06),
                      Colors.transparent,
                    ],
                  ),
                  boxShadow: [
                    BoxShadow(
                      color: colors.primary.withOpacity(0.2),
                      blurRadius: dimensions.spacing.clamp(8.0, 32.0),
                      spreadRadius: (dimensions.spacing / 8).clamp(1.0, 4.0),
                    ),
                  ],
                ),
                child: Container(
                  padding: EdgeInsets.all(8.0), // Add some internal padding
                  child: Image.asset(
                    logoAsset, //  Use theme-appropriate logo
                    width: dimensions.logoSize.clamp(120.0, 400.0),
                    height: dimensions.logoHeight.clamp(50.0, 180.0),
                    fit: BoxFit
                        .contain, // This ensures the logo scales properly without cutting
                    errorBuilder: (context, error, stackTrace) {
                      //  Fallback logo with theme-appropriate styling
                      return Container(
                        width: dimensions.logoSize.clamp(120.0, 400.0),
                        height: dimensions.logoHeight.clamp(50.0, 180.0),
                        decoration: BoxDecoration(
                          borderRadius: BorderRadius.circular(12.0),
                          gradient: LinearGradient(
                            colors: [colors.primary, colors.secondary],
                          ),
                        ),
                        child: Icon(
                          Icons.android_rounded,
                          size: (dimensions.logoSize * 0.5).clamp(30.0, 120.0),
                          color: Colors.white,
                        ),
                      );
                    },
                  ),
                ),
              ),
            );
          },
        ),

        SizedBox(height: (dimensions.spacing * 0.6).clamp(8.0, 24.0)),

        // Title with responsive line height
        Padding(
          padding:
              EdgeInsets.symmetric(horizontal: isVerySmallScreen ? 16.0 : 8.0),
          child: ShaderMask(
            shaderCallback: (bounds) => LinearGradient(
              colors: [
                colors.textPrimary,
                colors.primary,
                colors.textPrimary,
              ],
            ).createShader(bounds),
            child: Text(
              'Welcome Back',
              style: TextStyle(
                color: Colors.white,
                fontSize: dimensions.titleSize.clamp(18.0, 50.0),
                fontWeight: FontWeight.bold,
                letterSpacing: isVerySmallScreen ? 0.8 : 1.2,
                height: 1.1,
              ),
              textAlign: TextAlign.center,
              maxLines: isVerySmallScreen ? 2 : 1,
              overflow: TextOverflow.ellipsis,
            ),
          ),
        ),

        SizedBox(height: (dimensions.spacing / 4).clamp(4.0, 12.0)),

        // Subtitle with responsive sizing
        Padding(
          padding:
              EdgeInsets.symmetric(horizontal: isVerySmallScreen ? 20.0 : 12.0),
          child: Text(
            'Access your AMR control panel',
            style: TextStyle(
              color: colors.textSecondary,
              fontSize: dimensions.subtitleSize.clamp(11.0, 22.0),
              letterSpacing: 0.3,
              height: 1.2,
            ),
            textAlign: TextAlign.center,
            maxLines: isVerySmallScreen ? 2 : 1,
            overflow: TextOverflow.ellipsis,
          ),
        ),
      ],
    );
  }

  Widget _buildLoginForm(ResponsiveDimensions dimensions, ThemeColors colors) {
    return Container(
      padding: EdgeInsets.all(dimensions.cardPadding),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(dimensions.borderRadius),
        color: colors.cardBackground,
        border: Border.all(
          color: colors.border,
          width: 1.5,
        ),
        boxShadow: [
          BoxShadow(
            color: colors.shadow,
            blurRadius: dimensions.spacing,
            offset: Offset(0, dimensions.spacing / 4),
            spreadRadius: 2,
          ),
        ],
      ),
      child: Column(
        children: [
          _buildTextField(
            controller: _usernameController,
            hint: 'Username or Email',
            icon: Icons.person_outline_rounded,
            obscureText: false,
            dimensions: dimensions,
            colors: colors,
          ),
          SizedBox(height: dimensions.spacing / 2),
          _buildTextField(
            controller: _passwordController,
            hint: 'Password',
            icon: Icons.lock_outline_rounded,
            obscureText: _obscurePassword,
            dimensions: dimensions,
            colors: colors,
            suffixIcon: IconButton(
              icon: Icon(
                _obscurePassword
                    ? Icons.visibility_off_rounded
                    : Icons.visibility_rounded,
                color: colors.textSecondary,
                size: dimensions.iconSize,
              ),
              onPressed: () {
                setState(() {
                  _obscurePassword = !_obscurePassword;
                });
                HapticFeedback.lightImpact();
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildTextField({
    required TextEditingController controller,
    required String hint,
    required IconData icon,
    required bool obscureText,
    required ResponsiveDimensions dimensions,
    required ThemeColors colors,
    Widget? suffixIcon,
  }) {
    final screenWidth = MediaQuery.of(context).size.width;
    final isVerySmallScreen = screenWidth < 340;

    return Container(
      height: dimensions.textFieldHeight.clamp(44.0, 80.0),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(dimensions.borderRadius / 1.5),
        color: colors.inputBackground,
        border: Border.all(
          color: colors.inputBorder,
          width: 2.0, //  Enhanced border width for better visibility
        ),
        boxShadow: [
          BoxShadow(
            color: colors.shadow.withOpacity(0.5),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: TextField(
        controller: controller,
        obscureText: obscureText,
        style: TextStyle(
          color:
              Colors.black87, //  Force dark black text for better readability
          fontSize: dimensions.subtitleSize.clamp(12.0, 20.0),
          fontWeight:
              FontWeight.w600, //  Enhanced font weight for better readability
        ),
        decoration: InputDecoration(
          hintText: hint,
          hintStyle: TextStyle(
            color: Colors
                .black54, //  Dark grey for hint text - visible but distinguishable
            fontSize: dimensions.subtitleSize.clamp(12.0, 20.0),
            fontWeight: FontWeight.w500, //  Enhanced hint font weight
          ),
          prefixIcon: Container(
            margin: EdgeInsets.all((dimensions.spacing / 4).clamp(4.0, 12.0)),
            decoration: BoxDecoration(
              color: colors.primary
                  .withOpacity(0.15), //  Enhanced icon background opacity
              borderRadius: BorderRadius.circular(dimensions.borderRadius / 2),
            ),
            child: Icon(
              icon,
              color: colors.primary,
              size: dimensions.iconSize.clamp(16.0, 28.0),
            ),
          ),
          suffixIcon: suffixIcon != null
              ? Padding(
                  padding:
                      EdgeInsets.only(right: isVerySmallScreen ? 8.0 : 12.0),
                  child: suffixIcon,
                )
              : null,
          border: InputBorder.none,
          contentPadding: EdgeInsets.symmetric(
            horizontal: (dimensions.cardPadding / 2).clamp(8.0, 24.0),
            vertical: (dimensions.spacing / 2).clamp(8.0, 16.0),
          ),
          isDense: isVerySmallScreen,
        ),
      ),
    );
  }

  Widget _buildRememberForgot(
      ResponsiveDimensions dimensions, ThemeColors colors) {
    final deviceType = _getDeviceType(context);
    final screenWidth = MediaQuery.of(context).size.width;
    final useColumn = deviceType == DeviceType.mobile && screenWidth < 380;

    if (useColumn) {
      return Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Transform.scale(
                scale: 1.1,
                child: Checkbox(
                  value: _rememberMe,
                  onChanged: (value) {
                    setState(() {
                      _rememberMe = value ?? false;
                    });
                    HapticFeedback.lightImpact();
                  },
                  activeColor: colors.primary,
                  checkColor: Colors.white,
                  side: BorderSide(
                    color: colors.border,
                    width: 2,
                  ),
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(4),
                  ),
                ),
              ),
              Flexible(
                child: Text(
                  'Remember me',
                  style: TextStyle(
                    color: colors.textSecondary,
                    fontSize: dimensions.subtitleSize - 2,
                    fontWeight: FontWeight.w500,
                  ),
                ),
              ),
            ],
          ),
          SizedBox(height: dimensions.spacing / 4),
          Center(
            child: TextButton(
              onPressed: () {
                HapticFeedback.lightImpact();
              },
              style: TextButton.styleFrom(
                padding: EdgeInsets.symmetric(
                  horizontal: dimensions.spacing / 2,
                  vertical: dimensions.spacing / 4,
                ),
              ),
              child: Text(
                'Forgot password?',
                style: TextStyle(
                  color: colors.primary,
                  fontSize: dimensions.subtitleSize - 2,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ),
          ),
        ],
      );
    }

    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceBetween,
      children: [
        Flexible(
          flex: 3,
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Transform.scale(
                scale: 1.1,
                child: Checkbox(
                  value: _rememberMe,
                  onChanged: (value) {
                    setState(() {
                      _rememberMe = value ?? false;
                    });
                    HapticFeedback.lightImpact();
                  },
                  activeColor: colors.primary,
                  checkColor: Colors.white,
                  side: BorderSide(
                    color: colors.border,
                    width: 2,
                  ),
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(4),
                  ),
                ),
              ),
              Flexible(
                child: Text(
                  'Remember me',
                  style: TextStyle(
                    color: colors.textSecondary,
                    fontSize: dimensions.subtitleSize - 2,
                    fontWeight: FontWeight.w500,
                  ),
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
          ),
        ),
        Flexible(
          flex: 2,
          child: TextButton(
            onPressed: () {
              HapticFeedback.lightImpact();
            },
            style: TextButton.styleFrom(
              padding: EdgeInsets.symmetric(
                horizontal: dimensions.spacing / 3,
                vertical: dimensions.spacing / 4,
              ),
            ),
            child: Text(
              'Forgot password?',
              style: TextStyle(
                color: colors.primary,
                fontSize: dimensions.subtitleSize - 2,
                fontWeight: FontWeight.w600,
              ),
              overflow: TextOverflow.ellipsis,
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildLoginButton(
      ResponsiveDimensions dimensions, ThemeColors colors) {
    final screenWidth = MediaQuery.of(context).size.width;
    final isVerySmallScreen = screenWidth < 340;

    return Container(
      width: double.infinity,
      height: dimensions.buttonHeight.clamp(44.0, 84.0),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(dimensions.borderRadius / 1.5),
        gradient: LinearGradient(
          colors: colors.buttonGradient,
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        boxShadow: [
          BoxShadow(
            color: colors.primary.withOpacity(0.4),
            blurRadius: (dimensions.spacing / 2).clamp(4.0, 16.0),
            offset: Offset(0, (dimensions.spacing / 4).clamp(2.0, 8.0)),
            spreadRadius: 1,
          ),
        ],
      ),
      child: ElevatedButton(
        onPressed: _isLoading ? null : _handleLogin,
        style: ElevatedButton.styleFrom(
          backgroundColor: Colors.transparent,
          shadowColor: Colors.transparent,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(dimensions.borderRadius / 1.5),
          ),
          elevation: 0,
          padding: EdgeInsets.symmetric(
            horizontal: isVerySmallScreen ? 16.0 : 24.0,
            vertical: isVerySmallScreen ? 8.0 : 12.0,
          ),
        ),
        child: _isLoading
            ? SizedBox(
                width: dimensions.iconSize.clamp(16.0, 28.0),
                height: dimensions.iconSize.clamp(16.0, 28.0),
                child: CircularProgressIndicator(
                  strokeWidth: 2.5,
                  valueColor: AlwaysStoppedAnimation<Color>(colors.buttonText),
                ),
              )
            : Row(
                mainAxisAlignment: MainAxisAlignment.center,
                mainAxisSize: MainAxisSize.min,
                children: [
                  Icon(
                    Icons.login_rounded,
                    color: colors.buttonText,
                    size: dimensions.iconSize.clamp(16.0, 28.0),
                  ),
                  SizedBox(width: (dimensions.spacing / 4).clamp(4.0, 8.0)),
                  Flexible(
                    child: Text(
                      'SIGN IN',
                      style: TextStyle(
                        color: colors.buttonText,
                        fontSize: dimensions.buttonTextSize.clamp(12.0, 22.0),
                        fontWeight: FontWeight.bold,
                        letterSpacing: isVerySmallScreen ? 1.0 : 1.2,
                      ),
                      overflow: TextOverflow.ellipsis,
                    ),
                  ),
                ],
              ),
      ),
    );
  }

  Widget _buildDivider(ResponsiveDimensions dimensions, ThemeColors colors) {
    return Row(
      children: [
        Expanded(
          child: Container(
            height: 1.5,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [
                  Colors.transparent,
                  colors.border,
                  colors.border,
                ],
              ),
            ),
          ),
        ),
        Padding(
          padding: EdgeInsets.symmetric(horizontal: dimensions.spacing / 2),
          child: Container(
            padding: EdgeInsets.symmetric(
              horizontal: dimensions.spacing / 2,
              vertical: dimensions.spacing / 4,
            ),
            decoration: BoxDecoration(
              color: colors.cardBackground,
              borderRadius: BorderRadius.circular(dimensions.borderRadius / 2),
              border: Border.all(color: colors.border, width: 1),
            ),
            child: Text(
              'OR',
              style: TextStyle(
                color: colors.textSecondary,
                fontSize: dimensions.subtitleSize - 3,
                fontWeight: FontWeight.w600,
                letterSpacing: 1,
              ),
            ),
          ),
        ),
        Expanded(
          child: Container(
            height: 1.5,
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [
                  colors.border,
                  colors.border,
                  Colors.transparent,
                ],
              ),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildSocialLogin(ResponsiveDimensions dimensions, ThemeColors colors,
      DeviceType deviceType) {
    final screenWidth = MediaQuery.of(context).size.width;
    final isVerySmallScreen = screenWidth < 340;
    final isSmallScreen = screenWidth < 400;
    final availableWidth = screenWidth - (dimensions.horizontalPadding * 2);

    // Calculate button width for small screens
    final buttonWidth = isVerySmallScreen
        ? (availableWidth / 3) - 8
        : isSmallScreen
            ? (availableWidth / 3) - 6
            : null;

    final buttons = [
      _buildSocialButton(
        icon: Icons.g_mobiledata_rounded,
        label: isVerySmallScreen ? '' : (isSmallScreen ? 'Google' : 'Google'),
        dimensions: dimensions,
        colors: colors,
        onTap: () {},
        fixedWidth: buttonWidth,
      ),
      _buildSocialButton(
        icon: Icons.apple_rounded,
        label: isVerySmallScreen ? '' : (isSmallScreen ? 'Apple' : 'Apple'),
        dimensions: dimensions,
        colors: colors,
        onTap: () {},
        fixedWidth: buttonWidth,
      ),
      _buildSocialButton(
        icon: Icons.fingerprint_rounded,
        label: isVerySmallScreen ? '' : (isSmallScreen ? 'Bio' : 'Biometric'),
        dimensions: dimensions,
        colors: colors,
        onTap: () {},
        fixedWidth: buttonWidth,
      ),
    ];

    if (isSmallScreen) {
      return Row(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: buttons
            .map((button) => isVerySmallScreen
                ? Expanded(
                    child: Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 2),
                      child: button,
                    ),
                  )
                : Flexible(
                    child: Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 3),
                      child: button,
                    ),
                  ))
            .toList(),
      );
    }

    return Row(
      children: buttons
          .map((button) => Expanded(
                child: Padding(
                  padding:
                      EdgeInsets.symmetric(horizontal: dimensions.spacing / 6),
                  child: button,
                ),
              ))
          .toList(),
    );
  }

  Widget _buildSocialButton({
    required IconData icon,
    required String label,
    required ResponsiveDimensions dimensions,
    required ThemeColors colors,
    required VoidCallback onTap,
    double? fixedWidth,
  }) {
    final isIconOnly = label.isEmpty;
    final screenWidth = MediaQuery.of(context).size.width;
    final isVerySmallScreen = screenWidth < 340;

    return Material(
      color: Colors.transparent,
      child: InkWell(
        onTap: () {
          HapticFeedback.lightImpact();
          onTap();
        },
        borderRadius: BorderRadius.circular(dimensions.borderRadius / 1.5),
        child: Container(
          width: fixedWidth ?? double.infinity,
          height: isVerySmallScreen ? dimensions.buttonHeight * 0.8 : null,
          padding: EdgeInsets.symmetric(
            horizontal: isIconOnly || isVerySmallScreen
                ? dimensions.spacing / 4
                : dimensions.spacing / 3,
            vertical: isVerySmallScreen
                ? dimensions.spacing / 3
                : dimensions.spacing / 2.5,
          ),
          decoration: BoxDecoration(
            color: colors.cardBackground,
            borderRadius: BorderRadius.circular(dimensions.borderRadius / 1.5),
            border: Border.all(
              color: colors.border,
              width: 1.5,
            ),
            boxShadow: [
              BoxShadow(
                color: colors.shadow.withOpacity(0.3),
                blurRadius: 4,
                offset: const Offset(0, 2),
              ),
            ],
          ),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Icon(
                icon,
                color: colors.textPrimary,
                size: isVerySmallScreen
                    ? dimensions.iconSize * 0.9
                    : dimensions.iconSize,
              ),
              if (!isIconOnly && !isVerySmallScreen) ...[
                SizedBox(height: dimensions.spacing / 8),
                Flexible(
                  child: Text(
                    label,
                    style: TextStyle(
                      color: colors.textSecondary,
                      fontSize: (dimensions.subtitleSize - 4).clamp(8.0, 12.0),
                      fontWeight: FontWeight.w500,
                    ),
                    overflow: TextOverflow.ellipsis,
                    textAlign: TextAlign.center,
                    maxLines: 1,
                  ),
                ),
              ],
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildSignUpSection(
      ResponsiveDimensions dimensions, ThemeColors colors) {
    return Container(
      width: double.infinity,
      padding: EdgeInsets.all(dimensions.cardPadding * 0.8),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(dimensions.borderRadius),
        color: colors.cardBackground.withOpacity(0.8),
        border: Border.all(
          color: colors.border,
          width: 1.5,
        ),
        boxShadow: [
          BoxShadow(
            color: colors.shadow.withOpacity(0.5),
            blurRadius: 8,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Wrap(
        alignment: WrapAlignment.center,
        crossAxisAlignment: WrapCrossAlignment.center,
        spacing: dimensions.spacing / 4,
        runSpacing: dimensions.spacing / 4,
        children: [
          Text(
            'New to the system?',
            style: TextStyle(
              color: colors.textSecondary,
              fontSize: dimensions.subtitleSize - 1,
              fontWeight: FontWeight.w500,
            ),
          ),
          Container(
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [
                  colors.primary.withOpacity(0.1),
                  colors.secondary.withOpacity(0.1),
                ],
              ),
              borderRadius: BorderRadius.circular(dimensions.borderRadius / 2),
              border: Border.all(
                color: colors.primary.withOpacity(0.3),
                width: 1,
              ),
            ),
            child: TextButton(
              onPressed: () {
                HapticFeedback.lightImpact();
                Navigator.push(
                  context,
                  PageRouteBuilder(
                    pageBuilder: (context, animation, secondaryAnimation) =>
                        EnhancedSignupScreen(),
                    transitionsBuilder:
                        (context, animation, secondaryAnimation, child) {
                      return SlideTransition(
                        position: Tween<Offset>(
                          begin: const Offset(1.0, 0.0),
                          end: Offset.zero,
                        ).animate(CurvedAnimation(
                          parent: animation,
                          curve: Curves.easeInOut,
                        )),
                        child: child,
                      );
                    },
                    transitionDuration: const Duration(milliseconds: 500),
                  ),
                );
              },
              style: TextButton.styleFrom(
                padding: EdgeInsets.symmetric(
                  horizontal: dimensions.spacing / 2,
                  vertical: dimensions.spacing / 3,
                ),
                backgroundColor: Colors.transparent,
                shape: RoundedRectangleBorder(
                  borderRadius:
                      BorderRadius.circular(dimensions.borderRadius / 2),
                ),
              ),
              child: Text(
                'CREATE ACCOUNT',
                style: TextStyle(
                  color: colors.primary,
                  fontWeight: FontWeight.bold,
                  fontSize: dimensions.subtitleSize - 3,
                  letterSpacing: 1,
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

// Helper classes
enum DeviceType { mobile, tablet, laptop, desktop }

class ResponsiveDimensions {
  final double horizontalPadding;
  final double verticalPadding;
  final double logoSize;
  final double logoHeight;
  final double titleSize;
  final double subtitleSize;
  final double cardPadding;
  final double textFieldHeight;
  final double buttonHeight;
  final double buttonTextSize;
  final double spacing;
  final double maxWidth;
  final double borderRadius;
  final double iconSize;

  ResponsiveDimensions({
    required this.horizontalPadding,
    required this.verticalPadding,
    required this.logoSize,
    required this.logoHeight,
    required this.titleSize,
    required this.subtitleSize,
    required this.cardPadding,
    required this.textFieldHeight,
    required this.buttonHeight,
    required this.buttonTextSize,
    required this.spacing,
    required this.maxWidth,
    required this.borderRadius,
    required this.iconSize,
  });
}

class ThemeColors {
  final Color primary;
  final Color secondary;
  final Color background;
  final List<Color> backgroundGradient;
  final Color cardBackground;
  final Color textPrimary;
  final Color textSecondary;
  final Color border;
  final Color inputBackground;
  final Color inputBorder;
  final List<Color> buttonGradient;
  final Color buttonText;
  final Color shadow;
  final Color overlay;
  final Color success;
  final Color error;

  ThemeColors({
    required this.primary,
    required this.secondary,
    required this.background,
    required this.backgroundGradient,
    required this.cardBackground,
    required this.textPrimary,
    required this.textSecondary,
    required this.border,
    required this.inputBackground,
    required this.inputBorder,
    required this.buttonGradient,
    required this.buttonText,
    required this.shadow,
    required this.overlay,
    required this.success,
    required this.error,
  });

  factory ThemeColors.light() {
    return ThemeColors(
      primary: const Color(0xFFFF4757),
      secondary: const Color(0xFF5F27CD),
      background: const Color(0xFFFAFAFA),
      backgroundGradient: const [
        Color(0xFFFFFFFF),
        Color(0xFFFAFAFA),
        Color(0xFFF5F5F5),
      ],
      cardBackground: const Color(0xFFFFFFFF),
      textPrimary: const Color(0xFF2C3E50),
      textSecondary: const Color(0xFF7F8C8D),
      border: const Color(0xFFE1E8ED),
      inputBackground: Colors
          .transparent, //  FIXED: Transparent background for better text visibility
      inputBorder: const Color(
          0xFFBDC3C7), //  Enhanced border color for better definition
      buttonGradient: const [
        Color(0xFFFF4757),
        Color(0xFFFF6B7A),
      ],
      buttonText: const Color(0xFFFFFFFF),
      shadow: const Color(0xFF2C3E50).withOpacity(0.08),
      overlay: const Color(0xFFFFFFFF),
      success: const Color(0xFF00D68F),
      error: const Color(0xFFFF3D71),
    );
  }

  factory ThemeColors.dark() {
    return ThemeColors(
      primary: const Color(0xFFFF4757),
      secondary: const Color(0xFF667EEA),
      background: const Color(0xFF1A1A1A),
      backgroundGradient: const [
        Color(0xFF2C2C2C),
        Color(0xFF1A1A1A),
        Color(0xFF0F0F0F),
      ],
      cardBackground: const Color(0xFF2C2C2C),
      textPrimary: const Color(0xFFE8E6E3),
      textSecondary: const Color(0xFFBBB8B5),
      border: const Color(0xFF444444),
      inputBackground: const Color(0xFF333333),
      inputBorder: const Color(0xFF555555),
      buttonGradient: const [
        Color(0xFFFF4757),
        Color(0xFFFF3742),
      ],
      buttonText: const Color(0xFFFFFFFF),
      shadow: const Color(0xFF000000).withOpacity(0.3),
      overlay: const Color(0xFF000000),
      success: const Color(0xFF00D68F),
      error: const Color(0xFFFF3D71),
    );
  }
}
