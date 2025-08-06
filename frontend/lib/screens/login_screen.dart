// enhanced_login_screen.dart
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'connect_screen.dart';
import 'signup_screen.dart';
import 'package:shared_preferences/shared_preferences.dart';

class EnhancedLoginScreen extends StatefulWidget {
  final bool isDarkMode;
  final Function(bool) onThemeToggle;

  const EnhancedLoginScreen({
    super.key,
    required this.isDarkMode,
    required this.onThemeToggle,
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

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _loadSavedCredentials();
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
    return widget.isDarkMode ? ThemeColors.dark() : ThemeColors.light();
  }

  DeviceType _getDeviceType(BuildContext context) {
    final screenWidth = MediaQuery.of(context).size.width;
    final screenHeight = MediaQuery.of(context).size.height;
    final aspectRatio = screenWidth / screenHeight;

    // More sophisticated device detection
    if (screenWidth > 1400 || (screenWidth > 1200 && aspectRatio > 1.6)) {
      return DeviceType.desktop;
    }
    if (screenWidth > 900 || (screenWidth > 768 && aspectRatio > 1.3)) {
      return DeviceType.laptop;
    }
    if (screenWidth > 600 || (screenWidth > 500 && aspectRatio > 0.7)) {
      return DeviceType.tablet;
    }
    return DeviceType.mobile;
  }

  ResponsiveDimensions _getResponsiveDimensions(BuildContext context) {
    final deviceType = _getDeviceType(context);
    final screenHeight = MediaQuery.of(context).size.height;
    final isCompact = screenHeight < 700;

    switch (deviceType) {
      case DeviceType.desktop:
        return ResponsiveDimensions(
          horizontalPadding: 80.0,
          verticalPadding: isCompact ? 30.0 : 50.0,
          logoSize: isCompact ? 320.0 : 380.0,
          logoHeight: isCompact ? 140.0 : 160.0,
          titleSize: 44.0,
          subtitleSize: 20.0,
          cardPadding: 52.0,
          textFieldHeight: 76.0,
          buttonHeight: 80.0,
          buttonTextSize: 20.0,
          spacing: isCompact ? 50.0 : 65.0,
          maxWidth: 650.0,
          borderRadius: 24.0,
          iconSize: 28.0,
        );
      case DeviceType.laptop:
        return ResponsiveDimensions(
          horizontalPadding: 72.0,
          verticalPadding: isCompact ? 30.0 : 45.0,
          logoSize: isCompact ? 300.0 : 350.0,
          logoHeight: isCompact ? 130.0 : 150.0,
          titleSize: 40.0,
          subtitleSize: 19.0,
          cardPadding: 46.0,
          textFieldHeight: 70.0,
          buttonHeight: 74.0,
          buttonTextSize: 19.0,
          spacing: isCompact ? 45.0 : 58.0,
          maxWidth: 580.0,
          borderRadius: 22.0,
          iconSize: 26.0,
        );
      case DeviceType.tablet:
        final isLandscape = MediaQuery.of(context).orientation == Orientation.landscape;
        return ResponsiveDimensions(
          horizontalPadding: isLandscape ? 80.0 : 50.0,
          verticalPadding: isCompact ? 24.0 : 36.0,
          logoSize: isLandscape ? 280.0 : 320.0,
          logoHeight: isLandscape ? 120.0 : 140.0,
          titleSize: 36.0,
          subtitleSize: 18.0,
          cardPadding: 38.0,
          textFieldHeight: 64.0,
          buttonHeight: 68.0,
          buttonTextSize: 18.0,
          spacing: isCompact ? 35.0 : 48.0,
          maxWidth: isLandscape ? 550.0 : 520.0,
          borderRadius: 20.0,
          iconSize: 24.0,
        );
      case DeviceType.mobile:
        return ResponsiveDimensions(
          horizontalPadding: 24.0,
          verticalPadding: isCompact ? 16.0 : 24.0,
          logoSize: isCompact ? 240.0 : 280.0,
          logoHeight: isCompact ? 100.0 : 120.0,
          titleSize: 30.0,
          subtitleSize: 16.0,
          cardPadding: 24.0,
          textFieldHeight: 56.0,
          buttonHeight: 56.0,
          buttonTextSize: 16.0,
          spacing: isCompact ? 24.0 : 36.0,
          maxWidth: double.infinity,
          borderRadius: 18.0,
          iconSize: 22.0,
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
                deviceType == DeviceType.desktop ? 15 : 10,
                (index) => _buildAnimatedBackground(index, dimensions, colors),
              ),

            // Glass morphism overlay for desktop/laptop
            if (deviceType == DeviceType.desktop || deviceType == DeviceType.laptop)
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

            // Theme toggle button
            Positioned(
              top: MediaQuery.of(context).padding.top + 20,
              right: 20,
              child: _buildThemeToggle(dimensions, colors),
            ),

            // Main content
            SafeArea(
              child: Center(
                child: ConstrainedBox(
                  constraints: BoxConstraints(
                    maxWidth: dimensions.maxWidth,
                  ),
                  child: SingleChildScrollView(
                    padding: EdgeInsets.symmetric(
                      horizontal: dimensions.horizontalPadding,
                      vertical: dimensions.verticalPadding,
                    ),
                    child: AnimatedBuilder(
                      animation: _fadeAnimation,
                      builder: (context, child) {
                        return FadeTransition(
                          opacity: _fadeAnimation,
                          child: SlideTransition(
                            position: _slideAnimation,
                            child: Column(
                              mainAxisAlignment: MainAxisAlignment.center,
                              children: [
                                _buildLogoSection(dimensions, colors),
                                SizedBox(height: dimensions.spacing * 1.5),
                                _buildLoginForm(dimensions, colors),
                                SizedBox(height: dimensions.spacing * 0.75),
                                _buildRememberForgot(dimensions, colors),
                                SizedBox(height: dimensions.spacing),
                                _buildLoginButton(dimensions, colors),
                                SizedBox(height: dimensions.spacing),
                                _buildDivider(dimensions, colors),
                                SizedBox(height: dimensions.spacing),
                                _buildSocialLogin(dimensions, colors, deviceType),
                                SizedBox(height: dimensions.spacing * 1.2),
                                _buildSignUpSection(dimensions, colors),
                              ],
                            ),
                          ),
                        );
                      },
                    ),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildThemeToggle(ResponsiveDimensions dimensions, ThemeColors colors) {
    return Container(
      decoration: BoxDecoration(
        color: colors.cardBackground,
        borderRadius: BorderRadius.circular(dimensions.borderRadius),
        boxShadow: [
          BoxShadow(
            color: colors.shadow,
            blurRadius: 10,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Material(
        color: Colors.transparent,
        child: InkWell(
          onTap: () {
            HapticFeedback.lightImpact();
            widget.onThemeToggle(!widget.isDarkMode);
          },
          borderRadius: BorderRadius.circular(dimensions.borderRadius),
          child: Padding(
            padding: EdgeInsets.all(dimensions.spacing / 3),
            child: AnimatedSwitcher(
              duration: const Duration(milliseconds: 300),
              child: Icon(
                widget.isDarkMode ? Icons.light_mode_rounded : Icons.dark_mode_rounded,
                key: ValueKey(widget.isDarkMode),
                color: colors.primary,
                size: dimensions.iconSize,
              ),
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildAnimatedBackground(int index, ResponsiveDimensions dimensions, ThemeColors colors) {
    final random = (index * 54321) % 1000;
    final screenSize = MediaQuery.of(context).size;
    final left = (random % 100) / 100 * screenSize.width;
    final top = ((random ~/ 100) % 100) / 100 * screenSize.height;
    final baseSize = dimensions.spacing * 2;
    final size = baseSize + (random % 60);

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
                    colors.primary.withOpacity(0.1),
                    colors.primary.withOpacity(0.05),
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

  Widget _buildLogoSection(ResponsiveDimensions dimensions, ThemeColors colors) {
    return Column(
      children: [
        AnimatedBuilder(
          animation: _pulseAnimation,
          builder: (context, child) {
            return Transform.scale(
              scale: _pulseAnimation.value,
              child: Container(
                padding: EdgeInsets.all(dimensions.cardPadding / 2),
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  gradient: RadialGradient(
                    colors: [
                      colors.primary.withOpacity(0.15),
                      colors.primary.withOpacity(0.05),
                      Colors.transparent,
                    ],
                  ),
                  boxShadow: [
                    BoxShadow(
                      color: colors.primary.withOpacity(0.3),
                      blurRadius: dimensions.spacing,
                      spreadRadius: dimensions.spacing / 8,
                    ),
                  ],
                ),
                child: Image.asset(
                  widget.isDarkMode ? 'assets/TW_WHITE.png' : 'assets/login_logo.png',
                  width: dimensions.logoSize,
                  height: dimensions.logoHeight,
                  fit: BoxFit.contain,
                ),
              ),
            );
          },
        ),
        SizedBox(height: dimensions.spacing * 0.75),
        ShaderMask(
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
              fontSize: dimensions.titleSize,
              fontWeight: FontWeight.bold,
              letterSpacing: 1.5,
            ),
          ),
        ),
        SizedBox(height: dimensions.spacing / 4),
        Text(
          'Access your AMR control panel',
          style: TextStyle(
            color: colors.textSecondary,
            fontSize: dimensions.subtitleSize,
            letterSpacing: 0.5,
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
          width: 1,
        ),
        boxShadow: [
          BoxShadow(
            color: colors.shadow,
            blurRadius: dimensions.spacing,
            offset: Offset(0, dimensions.spacing / 4),
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
    return Container(
      height: dimensions.textFieldHeight,
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(dimensions.borderRadius / 1.5),
        color: colors.inputBackground,
        border: Border.all(
          color: colors.inputBorder,
          width: 1,
        ),
      ),
      child: TextField(
        controller: controller,
        obscureText: obscureText,
        style: TextStyle(
          color: colors.textPrimary,
          fontSize: dimensions.subtitleSize,
          fontWeight: FontWeight.w500,
        ),
        decoration: InputDecoration(
          hintText: hint,
          hintStyle: TextStyle(
            color: colors.textSecondary.withOpacity(0.7),
            fontSize: dimensions.subtitleSize,
          ),
          prefixIcon: Container(
            margin: EdgeInsets.all(dimensions.spacing / 4),
            decoration: BoxDecoration(
              color: colors.primary.withOpacity(0.1),
              borderRadius: BorderRadius.circular(dimensions.borderRadius / 2),
            ),
            child: Icon(
              icon,
              color: colors.primary,
              size: dimensions.iconSize,
            ),
          ),
          suffixIcon: suffixIcon,
          border: InputBorder.none,
          contentPadding: EdgeInsets.symmetric(
            horizontal: dimensions.cardPadding / 2,
            vertical: dimensions.spacing / 2,
          ),
        ),
      ),
    );
  }

  Widget _buildRememberForgot(ResponsiveDimensions dimensions, ThemeColors colors) {
    final deviceType = _getDeviceType(context);
    final useColumn = deviceType == DeviceType.mobile && 
                      MediaQuery.of(context).size.width < 400;
    
    if (useColumn) {
      return Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Transform.scale(
                scale: 1.2,
                child: Checkbox(
                  value: _rememberMe,
                  onChanged: (value) {
                    setState(() {
                      _rememberMe = value ?? false;
                    });
                    HapticFeedback.lightImpact();
                  },
                  activeColor: colors.primary,
                  side: BorderSide(
                    color: colors.border,
                    width: 2,
                  ),
                ),
              ),
              Flexible(
                child: Text(
                  'Remember me',
                  style: TextStyle(
                    color: colors.textSecondary,
                    fontSize: dimensions.subtitleSize - 2,
                  ),
                ),
              ),
            ],
          ),
          Center(
            child: TextButton(
              onPressed: () {
                HapticFeedback.lightImpact();
              },
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
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Transform.scale(
                scale: 1.2,
                child: Checkbox(
                  value: _rememberMe,
                  onChanged: (value) {
                    setState(() {
                      _rememberMe = value ?? false;
                    });
                    HapticFeedback.lightImpact();
                  },
                  activeColor: colors.primary,
                  side: BorderSide(
                    color: colors.border,
                    width: 2,
                  ),
                ),
              ),
              Flexible(
                child: Text(
                  'Remember me',
                  style: TextStyle(
                    color: colors.textSecondary,
                    fontSize: dimensions.subtitleSize - 2,
                  ),
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
          ),
        ),
        TextButton(
          onPressed: () {
            HapticFeedback.lightImpact();
          },
          child: Text(
            'Forgot password?',
            style: TextStyle(
              color: colors.primary,
              fontSize: dimensions.subtitleSize - 2,
              fontWeight: FontWeight.w600,
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildLoginButton(ResponsiveDimensions dimensions, ThemeColors colors) {
    return Container(
      width: double.infinity,
      height: dimensions.buttonHeight,
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(dimensions.borderRadius / 1.5),
        gradient: LinearGradient(
          colors: colors.buttonGradient,
        ),
        boxShadow: [
          BoxShadow(
            color: colors.primary.withOpacity(0.3),
            blurRadius: dimensions.spacing / 2,
            offset: Offset(0, dimensions.spacing / 4),
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
        ),
        child: _isLoading
            ? SizedBox(
                width: dimensions.iconSize,
                height: dimensions.iconSize,
                child: CircularProgressIndicator(
                  strokeWidth: 2,
                  valueColor: AlwaysStoppedAnimation<Color>(colors.buttonText),
                ),
              )
            : Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(
                    Icons.login_rounded,
                    color: colors.buttonText,
                    size: dimensions.iconSize,
                  ),
                  SizedBox(width: dimensions.spacing / 4),
                  Text(
                    'SIGN IN',
                    style: TextStyle(
                      color: colors.buttonText,
                      fontSize: dimensions.buttonTextSize,
                      fontWeight: FontWeight.bold,
                      letterSpacing: 1.5,
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
            height: 1,
            color: colors.border,
          ),
        ),
        Padding(
          padding: EdgeInsets.symmetric(horizontal: dimensions.spacing / 2),
          child: Text(
            'OR',
            style: TextStyle(
              color: colors.textSecondary,
              fontSize: dimensions.subtitleSize - 2,
              fontWeight: FontWeight.w500,
            ),
          ),
        ),
        Expanded(
          child: Container(
            height: 1,
            color: colors.border,
          ),
        ),
      ],
    );
  }

  Widget _buildSocialLogin(ResponsiveDimensions dimensions, ThemeColors colors, DeviceType deviceType) {
    final isCompact = deviceType == DeviceType.mobile;
    final screenWidth = MediaQuery.of(context).size.width;
    final useWrap = screenWidth < 500;
    
    final buttons = [
      _buildSocialButton(
        icon: Icons.g_mobiledata_rounded,
        label: isCompact ? '' : 'Google',
        dimensions: dimensions,
        colors: colors,
        onTap: () {},
      ),
      _buildSocialButton(
        icon: Icons.apple_rounded,
        label: isCompact ? '' : 'Apple',
        dimensions: dimensions,
        colors: colors,
        onTap: () {},
      ),
      _buildSocialButton(
        icon: Icons.fingerprint_rounded,
        label: isCompact ? '' : 'Biometric',
        dimensions: dimensions,
        colors: colors,
        onTap: () {},
      ),
    ];
    
    if (useWrap) {
      return Wrap(
        alignment: WrapAlignment.center,
        spacing: dimensions.spacing / 2,
        runSpacing: dimensions.spacing / 3,
        children: buttons,
      );
    }
    
    return Row(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        for (int i = 0; i < buttons.length; i++) ...[
          Flexible(
            child: ConstrainedBox(
              constraints: const BoxConstraints(maxWidth: 150),
              child: buttons[i],
            ),
          ),
          if (i < buttons.length - 1) SizedBox(width: dimensions.spacing / 2),
        ],
      ],
    );
  }

  Widget _buildSocialButton({
    required IconData icon,
    required String label,
    required ResponsiveDimensions dimensions,
    required ThemeColors colors,
    required VoidCallback onTap,
  }) {
    final isIconOnly = label.isEmpty;
    
    return Material(
      color: Colors.transparent,
      child: InkWell(
        onTap: () {
          HapticFeedback.lightImpact();
          onTap();
        },
        borderRadius: BorderRadius.circular(dimensions.borderRadius / 1.5),
        child: Container(
          padding: EdgeInsets.symmetric(
            horizontal: isIconOnly ? dimensions.spacing / 2 : dimensions.spacing,
            vertical: dimensions.spacing / 2,
          ),
          decoration: BoxDecoration(
            color: colors.cardBackground,
            borderRadius: BorderRadius.circular(dimensions.borderRadius / 1.5),
            border: Border.all(
              color: colors.border,
              width: 1,
            ),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(
                icon,
                color: colors.textPrimary,
                size: dimensions.iconSize,
              ),
              if (!isIconOnly) ...[
                SizedBox(width: dimensions.spacing / 4),
                Text(
                  label,
                  style: TextStyle(
                    color: colors.textPrimary,
                    fontSize: dimensions.subtitleSize - 2,
                    fontWeight: FontWeight.w500,
                  ),
                ),
              ],
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildSignUpSection(ResponsiveDimensions dimensions, ThemeColors colors) {
    return Container(
      padding: EdgeInsets.all(dimensions.cardPadding / 1.5),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(dimensions.borderRadius / 1.5),
        color: colors.cardBackground.withOpacity(0.5),
        border: Border.all(
          color: colors.border,
          width: 1,
        ),
      ),
      child: Wrap(
        alignment: WrapAlignment.center,
        crossAxisAlignment: WrapCrossAlignment.center,
        spacing: dimensions.spacing / 6,
        runSpacing: dimensions.spacing / 4,
        children: [
          Text(
            'New to the system?',
            style: TextStyle(
              color: colors.textSecondary,
              fontSize: dimensions.subtitleSize - 1,
            ),
          ),
          TextButton(
            onPressed: () {
              HapticFeedback.lightImpact();
              Navigator.push(
                context,
                PageRouteBuilder(
                  pageBuilder: (context, animation, secondaryAnimation) =>
                      EnhancedSignupScreen(
                        isDarkMode: widget.isDarkMode,
                        onThemeToggle: widget.onThemeToggle,
                      ),
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
                vertical: dimensions.spacing / 4,
              ),
              backgroundColor: colors.primary.withOpacity(0.1),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(dimensions.borderRadius / 2),
              ),
            ),
            child: Text(
              'CREATE ACCOUNT',
              style: TextStyle(
                color: colors.primary,
                fontWeight: FontWeight.bold,
                fontSize: dimensions.subtitleSize - 4,
                letterSpacing: 1.2,
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
      background: const Color(0xFFF8F9FA),
      backgroundGradient: const [
        Color(0xFFF5F7FA),
        Color(0xFFE9ECEF),
        Color(0xFFDEE2E6),
      ],
      cardBackground: Colors.white,
      textPrimary: const Color(0xFF2C3E50),
      textSecondary: const Color(0xFF7F8C8D),
      border: const Color(0xFFE1E8ED),
      inputBackground: const Color(0xFFF7F9FC),
      inputBorder: const Color(0xFFDAE1E7),
      buttonGradient: const [
        Color(0xFFFF4757),
        Color(0xFFFF6B7A),
      ],
      buttonText: Colors.white,
      shadow: Colors.black.withOpacity(0.08),
      overlay: Colors.white,
      success: const Color(0xFF00D68F),
      error: const Color(0xFFFF3D71),
    );
  }

  factory ThemeColors.dark() {
    return ThemeColors(
      primary: const Color(0xFFFF4757),
      secondary: const Color(0xFF667EEA),
      background: const Color(0xFF0A0A0A),
      backgroundGradient: const [
        Color(0xFF0A0A0A),
        Color(0xFF1A1A1A),
        Color(0xFF0F0F0F),
      ],
      cardBackground: const Color(0xFF1A1A1A),
      textPrimary: Colors.white,
      textSecondary: Colors.white60,
      border: Colors.white.withOpacity(0.1),
      inputBackground: Colors.white.withOpacity(0.05),
      inputBorder: Colors.white.withOpacity(0.15),
      buttonGradient: const [
        Color(0xFFFF4757),
        Color(0xFFFF3742),
      ],
      buttonText: Colors.white,
      shadow: Colors.black.withOpacity(0.4),
      overlay: Colors.black,
      success: const Color(0xFF00D68F),
      error: const Color(0xFFFF3D71),
    );
  }
}