// login_screen.dart
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'connect_screen.dart';
import 'signup_screen.dart';
import 'package:shared_preferences/shared_preferences.dart';

class LoginScreen extends StatefulWidget {
  const LoginScreen({super.key});

  @override
  State<LoginScreen> createState() => _LoginScreenState();
}

class _LoginScreenState extends State<LoginScreen>
    with TickerProviderStateMixin {
  bool _obscurePassword = true;
  final TextEditingController _usernameController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();

  late AnimationController _fadeController;
  late AnimationController _slideController;
  late AnimationController _pulseController;
  late Animation<double> _fadeAnimation;
  late Animation<Offset> _slideAnimation;
  late Animation<double> _pulseAnimation;

  bool _isLoading = false;

  @override
  void initState() {
    super.initState();

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

    // Start animations
    _fadeController.forward();
    _slideController.forward();
    _pulseController.repeat(reverse: true);
  }

  @override
  void dispose() {
    _fadeController.dispose();
    _slideController.dispose();
    _pulseController.dispose();
    _usernameController.dispose();
    _passwordController.dispose();
    super.dispose();
  }

  // Device type detection
  DeviceType _getDeviceType(BuildContext context) {
    final screenWidth = MediaQuery.of(context).size.width;
    if (screenWidth > 1400) return DeviceType.desktop;
    if (screenWidth > 900) return DeviceType.laptop;
    if (screenWidth > 600) return DeviceType.tablet;
    return DeviceType.mobile;
  }

  // Get responsive dimensions
  ResponsiveDimensions _getResponsiveDimensions(BuildContext context) {
    final deviceType = _getDeviceType(context);

    switch (deviceType) {
      case DeviceType.desktop:
        return ResponsiveDimensions(
          horizontalPadding: 80.0,
          verticalPadding: 40.0,
          logoSize: 380.0,
          logoHeight: 160.0,
          titleSize: 42.0,
          subtitleSize: 20.0,
          cardPadding: 48.0,
          textFieldHeight: 72.0,
          buttonHeight: 76.0,
          buttonTextSize: 20.0,
          spacing: 60.0,
          maxWidth: 600.0,
        );
      case DeviceType.laptop:
        return ResponsiveDimensions(
          horizontalPadding: 72.0,
          verticalPadding: 40.0,
          logoSize: 340.0,
          logoHeight: 150.0,
          titleSize: 38.0,
          subtitleSize: 19.0,
          cardPadding: 44.0,
          textFieldHeight: 68.0,
          buttonHeight: 72.0,
          buttonTextSize: 19.0,
          spacing: 55.0,
          maxWidth: 560.0,
        );
      case DeviceType.tablet:
        return ResponsiveDimensions(
          horizontalPadding: 60.0,
          verticalPadding: 32.0,
          logoSize: 340.0,
          logoHeight: 140.0,
          titleSize: 38.0,
          subtitleSize: 18.0,
          cardPadding: 40.0,
          textFieldHeight: 64.0,
          buttonHeight: 68.0,
          buttonTextSize: 18.0,
          spacing: 50.0,
          maxWidth: 500.0,
        );
      case DeviceType.mobile:
        return ResponsiveDimensions(
          horizontalPadding: 32.0,
          verticalPadding: 24.0,
          logoSize: 280.0,
          logoHeight: 120.0,
          titleSize: 32.0,
          subtitleSize: 16.0,
          cardPadding: 28.0,
          textFieldHeight: 56.0,
          buttonHeight: 56.0,
          buttonTextSize: 16.0,
          spacing: 40.0,
          maxWidth: double.infinity,
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

    // Add haptic feedback
    HapticFeedback.mediumImpact();

    // Simulate network delay
    await Future.delayed(const Duration(milliseconds: 800));

    try {
      // Get saved credentials
      final prefs = await SharedPreferences.getInstance();
      final savedUsername = prefs.getString('saved_username');
      final savedPassword = prefs.getString('saved_password');

      // Check default or saved credentials
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

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Row(
          children: [
            Icon(
              isSuccess ? Icons.check_circle : Icons.error,
              color: Colors.white,
              size: dimensions.buttonTextSize + 4,
            ),
            SizedBox(width: dimensions.spacing / 4),
            Expanded(
              child: Text(
                message,
                style: TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.w500,
                  fontSize: dimensions.subtitleSize - 2,
                ),
              ),
            ),
          ],
        ),
        backgroundColor: isSuccess ? Colors.green : Colors.redAccent,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(12),
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

    return Scaffold(
      body: Container(
        width: double.infinity,
        height: double.infinity,
        decoration: const BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: [
              Color(0xFF0A0A0A),
              Color(0xFF1A1A1A),
              Color(0xFF0F0F0F),
            ],
            stops: [0.0, 0.5, 1.0],
          ),
        ),
        child: Stack(
          children: [
            // Dynamic background elements based on device type
            ...List.generate(
                deviceType == DeviceType.desktop
                    ? 20
                    : deviceType == DeviceType.laptop
                        ? 16
                        : deviceType == DeviceType.tablet
                            ? 12
                            : 8,
                (index) => _buildBackgroundElement(index, dimensions)),

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
                                _buildLogoSection(dimensions),
                                SizedBox(height: dimensions.spacing * 2),
                                _buildLoginForm(dimensions),
                                SizedBox(height: dimensions.spacing),
                                _buildLoginButton(dimensions),
                                SizedBox(height: dimensions.spacing / 1.5),
                                _buildForgotPassword(dimensions),
                                SizedBox(height: dimensions.spacing * 1.2),
                                _buildSignUpSection(dimensions),
                                SizedBox(height: dimensions.spacing),
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

  Widget _buildLogoSection(ResponsiveDimensions dimensions) {
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
                      Colors.redAccent.withOpacity(0.15),
                      Colors.redAccent.withOpacity(0.05),
                      Colors.transparent,
                    ],
                  ),
                  boxShadow: [
                    BoxShadow(
                      color: Colors.redAccent.withOpacity(0.3),
                      blurRadius: dimensions.spacing / 1.5,
                      spreadRadius: dimensions.spacing / 8,
                    ),
                  ],
                ),
                child: Image.asset(
                  'assets/TW_WHITE.png',
                  width: dimensions.logoSize,
                  height: dimensions.logoHeight,
                  fit: BoxFit.contain,
                ),
              ),
            );
          },
        ),
        SizedBox(height: dimensions.spacing),
        ShaderMask(
          shaderCallback: (bounds) => const LinearGradient(
            colors: [
              Colors.white,
              Colors.redAccent,
              Colors.white,
            ],
          ).createShader(bounds),
          child: Text(
            'Welcome back',
            style: TextStyle(
              color: Colors.white,
              fontSize: dimensions.titleSize,
              fontWeight: FontWeight.bold,
              letterSpacing: 2.0,
            ),
          ),
        ),
        SizedBox(height: dimensions.spacing / 4),
        Text(
          'Access your AMR control panel',
          style: TextStyle(
            color: Colors.white60,
            fontSize: dimensions.subtitleSize,
            letterSpacing: 0.5,
            height: 1.4,
          ),
        ),
      ],
    );
  }

  Widget _buildLoginForm(ResponsiveDimensions dimensions) {
    return Container(
      padding: EdgeInsets.all(dimensions.cardPadding),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(20),
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.white.withOpacity(0.08),
            Colors.white.withOpacity(0.04),
            Colors.white.withOpacity(0.02),
          ],
        ),
        border: Border.all(
          color: Colors.white.withOpacity(0.15),
          width: 1,
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.4),
            blurRadius: dimensions.spacing / 2,
            offset: Offset(0, dimensions.spacing / 4),
          ),
        ],
      ),
      child: Column(
        children: [
          _buildTextField(
            controller: _usernameController,
            hint: 'Username',
            icon: Icons.person_outline,
            obscureText: false,
            dimensions: dimensions,
          ),
          SizedBox(height: dimensions.spacing / 2),
          _buildTextField(
            controller: _passwordController,
            hint: 'Password',
            icon: Icons.lock_outline,
            obscureText: _obscurePassword,
            dimensions: dimensions,
            suffixIcon: IconButton(
              icon: Icon(
                _obscurePassword
                    ? Icons.visibility_off_outlined
                    : Icons.visibility_outlined,
                color: Colors.white54,
                size: dimensions.buttonTextSize + 4,
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
    Widget? suffixIcon,
  }) {
    return Container(
      height: dimensions.textFieldHeight,
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(15),
        gradient: LinearGradient(
          colors: [
            Colors.white.withOpacity(0.10),
            Colors.white.withOpacity(0.06),
          ],
        ),
        border: Border.all(
          color: Colors.white.withOpacity(0.2),
          width: 1,
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 8,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: TextField(
        controller: controller,
        obscureText: obscureText,
        style: TextStyle(
          color: Colors.white,
          fontSize: dimensions.subtitleSize,
          fontWeight: FontWeight.w500,
        ),
        decoration: InputDecoration(
          hintText: hint,
          hintStyle: TextStyle(
            color: Colors.white.withOpacity(0.5),
            fontSize: dimensions.subtitleSize,
          ),
          prefixIcon: Container(
            margin: EdgeInsets.all(dimensions.spacing / 5),
            padding: EdgeInsets.all(dimensions.spacing / 7),
            decoration: BoxDecoration(
              color: Colors.redAccent.withOpacity(0.25),
              borderRadius: BorderRadius.circular(10),
            ),
            child: Icon(
              icon,
              color: Colors.redAccent,
              size: dimensions.buttonTextSize + 4,
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

  Widget _buildLoginButton(ResponsiveDimensions dimensions) {
    return Container(
      width: double.infinity,
      height: dimensions.buttonHeight,
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(16),
        gradient: const LinearGradient(
          colors: [
            Color(0xFFFF4757),
            Color(0xFFFF3742),
          ],
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.redAccent.withOpacity(0.5),
            blurRadius: dimensions.spacing / 3,
            offset: Offset(0, dimensions.spacing / 6),
          ),
        ],
      ),
      child: ElevatedButton(
        onPressed: _isLoading ? null : _handleLogin,
        style: ElevatedButton.styleFrom(
          backgroundColor: Colors.transparent,
          shadowColor: Colors.transparent,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(16),
          ),
        ),
        child: _isLoading
            ? SizedBox(
                width: dimensions.buttonTextSize + 8,
                height: dimensions.buttonTextSize + 8,
                child: const CircularProgressIndicator(
                  strokeWidth: 2,
                  valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
                ),
              )
            : Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(
                    Icons.login,
                    color: Colors.white,
                    size: dimensions.buttonTextSize + 4,
                  ),
                  SizedBox(width: dimensions.spacing / 4),
                  Text(
                    'ACCESS SYSTEM',
                    style: TextStyle(
                      color: Colors.white,
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

  Widget _buildForgotPassword(ResponsiveDimensions dimensions) {
    return TextButton(
      onPressed: () {
        HapticFeedback.lightImpact();
      },
      style: TextButton.styleFrom(
        padding: EdgeInsets.symmetric(
          horizontal: dimensions.cardPadding / 2,
          vertical: dimensions.spacing / 4,
        ),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(
            Icons.help_outline,
            color: Colors.white.withOpacity(0.6),
            size: dimensions.subtitleSize,
          ),
          SizedBox(width: dimensions.spacing / 6),
          Text(
            'Forgot password?',
            style: TextStyle(
              color: Colors.white.withOpacity(0.6),
              fontSize: dimensions.subtitleSize - 2,
              decoration: TextDecoration.underline,
              decorationColor: Colors.white.withOpacity(0.6),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildSignUpSection(ResponsiveDimensions dimensions) {
    return Container(
      padding: EdgeInsets.all(dimensions.cardPadding / 1.5),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(15),
        gradient: LinearGradient(
          colors: [
            Colors.white.withOpacity(0.03),
            Colors.white.withOpacity(0.01),
          ],
        ),
        border: Border.all(
          color: Colors.white.withOpacity(0.1),
          width: 1,
        ),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Text(
            'New to the system?',
            style: TextStyle(
              color: Colors.white.withOpacity(0.7),
              fontSize: dimensions.subtitleSize - 1,
            ),
          ),
          SizedBox(width: dimensions.spacing / 6),
          TextButton(
            onPressed: () {
              HapticFeedback.lightImpact();
              Navigator.push(
                context,
                PageRouteBuilder(
                  pageBuilder: (context, animation, secondaryAnimation) =>
                      const SignupScreen(),
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
            child: Container(
              padding: EdgeInsets.symmetric(
                horizontal: dimensions.spacing / 3,
                vertical: dimensions.spacing / 6,
              ),
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [
                    Colors.redAccent.withOpacity(0.25),
                    Colors.redAccent.withOpacity(0.15),
                  ],
                ),
                borderRadius: BorderRadius.circular(20),
                border: Border.all(
                  color: Colors.redAccent.withOpacity(0.4),
                  width: 1,
                ),
              ),
              child: Text(
                'CREATE ACCOUNT',
                style: TextStyle(
                  color: Colors.redAccent,
                  fontWeight: FontWeight.bold,
                  fontSize: dimensions.subtitleSize - 4,
                  letterSpacing: 1.2,
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildBackgroundElement(int index, ResponsiveDimensions dimensions) {
    final random = (index * 54321) % 1000;
    final screenSize = MediaQuery.of(context).size;
    final left = (random % 100) / 100 * screenSize.width;
    final top = ((random ~/ 100) % 100) / 100 * screenSize.height;
    final baseSize = dimensions.spacing / 2;
    final size = baseSize + (random % 40);

    return Positioned(
      left: left,
      top: top,
      child: AnimatedBuilder(
        animation: _pulseController,
        builder: (context, child) {
          final opacity =
              0.02 + ((_pulseController.value + (index * 0.15)) % 1.0) * 0.03;
          return Opacity(
            opacity: opacity,
            child: Container(
              width: size,
              height: size,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                gradient: RadialGradient(
                  colors: [
                    Colors.redAccent.withOpacity(0.1),
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
}

// Helper classes for responsive design
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
  });
}
