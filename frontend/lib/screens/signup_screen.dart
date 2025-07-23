// signup_screen.dart - Enhanced Modern AMR Design
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'login_screen.dart';

class SignupScreen extends StatefulWidget {
  const SignupScreen({super.key});

  @override
  State<SignupScreen> createState() => _SignupScreenState();
}

class _SignupScreenState extends State<SignupScreen>
    with TickerProviderStateMixin {
  final TextEditingController _usernameController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();
  final TextEditingController _confirmPasswordController = TextEditingController();

  bool _obscurePassword = true;
  bool _obscureConfirmPassword = true;
  bool _isLoading = false;
  bool _acceptTerms = false;

  late AnimationController _fadeController;
  late AnimationController _slideController;
  late AnimationController _pulseController;
  late Animation<double> _fadeAnimation;
  late Animation<Offset> _slideAnimation;
  late Animation<double> _pulseAnimation;

  // Password strength indicators
  bool _hasMinLength = false;
  bool _hasUppercase = false;
  bool _hasLowercase = false;
  bool _hasNumber = false;

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

    // Listen to password changes
    _passwordController.addListener(_checkPasswordStrength);
  }

  @override
  void dispose() {
    _fadeController.dispose();
    _slideController.dispose();
    _pulseController.dispose();
    _usernameController.dispose();
    _passwordController.dispose();
    _confirmPasswordController.dispose();
    super.dispose();
  }

  // Device type detection
  DeviceType _getDeviceType(BuildContext context) {
    final screenWidth = MediaQuery.of(context).size.width;
    if (screenWidth > 1200) return DeviceType.desktop;
    if (screenWidth > 600) return DeviceType.tablet;
    return DeviceType.mobile;
  }

  // Get responsive dimensions
  SignupResponsiveDimensions _getResponsiveDimensions(BuildContext context) {
    final deviceType = _getDeviceType(context);
    
    switch (deviceType) {
      case DeviceType.desktop:
        return SignupResponsiveDimensions(
          horizontalPadding: 60.0,
          verticalPadding: 40.0,
          appBarPadding: 32.0,
          iconSize: 60.0,
          titleSize: 40.0,
          subtitleSize: 20.0,
          cardPadding: 48.0,
          textFieldHeight: 72.0,
          buttonHeight: 76.0,
          buttonTextSize: 20.0,
          spacing: 60.0,
          maxWidth: 700.0,
          particleCount: 15,
          particleBaseSize: 30.0,
        );
      case DeviceType.tablet:
        return SignupResponsiveDimensions(
          horizontalPadding: 50.0,
          verticalPadding: 32.0,
          appBarPadding: 28.0,
          iconSize: 52.0,
          titleSize: 36.0,
          subtitleSize: 18.0,
          cardPadding: 40.0,
          textFieldHeight: 64.0,
          buttonHeight: 68.0,
          buttonTextSize: 18.0,
          spacing: 50.0,
          maxWidth: 550.0,
          particleCount: 12,
          particleBaseSize: 25.0,
        );
      case DeviceType.mobile:
        return SignupResponsiveDimensions(
          horizontalPadding: 32.0,
          verticalPadding: 24.0,
          appBarPadding: 20.0,
          iconSize: 48.0,
          titleSize: 32.0,
          subtitleSize: 16.0,
          cardPadding: 28.0,
          textFieldHeight: 56.0,
          buttonHeight: 56.0,
          buttonTextSize: 16.0,
          spacing: 40.0,
          maxWidth: double.infinity,
          particleCount: 8,
          particleBaseSize: 20.0,
        );
    }
  }

  void _checkPasswordStrength() {
    final password = _passwordController.text;
    setState(() {
      _hasMinLength = password.length >= 8;
      _hasUppercase = password.contains(RegExp(r'[A-Z]'));
      _hasLowercase = password.contains(RegExp(r'[a-z]'));
      _hasNumber = password.contains(RegExp(r'[0-9]'));
    });
  }

  Future<void> _saveCredentialsAndGoToLogin() async {
    if (!_validateForm()) return;

    setState(() {
      _isLoading = true;
    });

    HapticFeedback.mediumImpact();

    try {
      await Future.delayed(const Duration(milliseconds: 1000));

      final prefs = await SharedPreferences.getInstance();
      await prefs.setString('saved_username', _usernameController.text);
      await prefs.setString('saved_password', _passwordController.text);

      HapticFeedback.lightImpact();

      _showSnackBar('Account created successfully! Please log in.', true);

      await Future.delayed(const Duration(milliseconds: 500));
      Navigator.pop(context);
    } catch (e) {
      _showSnackBar('Failed to create account. Please try again.', false);
    } finally {
      if (mounted) {
        setState(() {
          _isLoading = false;
        });
      }
    }
  }

  bool _validateForm() {
    if (_usernameController.text.isEmpty ||
        _passwordController.text.isEmpty ||
        _confirmPasswordController.text.isEmpty) {
      _showSnackBar('Please fill all fields', false);
      return false;
    }

    if (_usernameController.text.length < 3) {
      _showSnackBar('Username must be at least 3 characters', false);
      return false;
    }

    if (!_hasMinLength || !_hasUppercase || !_hasLowercase || !_hasNumber) {
      _showSnackBar('Password does not meet security requirements', false);
      return false;
    }

    if (_passwordController.text != _confirmPasswordController.text) {
      _showSnackBar('Passwords do not match', false);
      return false;
    }

    if (!_acceptTerms) {
      _showSnackBar('Please accept the terms and conditions', false);
      return false;
    }

    return true;
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
            SizedBox(width: dimensions.spacing / 5),
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
            // Dynamic background elements
            ...List.generate(
              dimensions.particleCount,
              (index) => _buildBackgroundElement(index, dimensions)
            ),

            // Main content
            SafeArea(
              child: Column(
                children: [
                  _buildCustomAppBar(dimensions),
                  
                  Expanded(
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
                                    crossAxisAlignment: CrossAxisAlignment.start,
                                    children: [
                                      _buildHeaderSection(dimensions),
                                      SizedBox(height: dimensions.spacing),
                                      _buildSignupForm(dimensions),
                                      SizedBox(height: dimensions.spacing / 1.5),
                                      _buildPasswordStrengthIndicator(dimensions),
                                      SizedBox(height: dimensions.spacing / 1.5),
                                      _buildTermsCheckbox(dimensions),
                                      SizedBox(height: dimensions.spacing),
                                      _buildSignupButton(dimensions),
                                      SizedBox(height: dimensions.spacing),
                                      _buildLoginSection(dimensions),
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
          ],
        ),
      ),
    );
  }

  Widget _buildCustomAppBar(SignupResponsiveDimensions dimensions) {
    return Container(
      padding: EdgeInsets.all(dimensions.appBarPadding),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.black.withOpacity(0.3),
            Colors.transparent,
          ],
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
        ),
      ),
      child: Row(
        children: [
          Container(
            decoration: BoxDecoration(
              color: Colors.redAccent.withOpacity(0.1),
              borderRadius: BorderRadius.circular(12),
              border: Border.all(
                color: Colors.redAccent.withOpacity(0.3),
                width: 1,
              ),
            ),
            child: IconButton(
              icon: Icon(
                Icons.arrow_back_ios_new,
                color: Colors.redAccent,
                size: dimensions.buttonTextSize + 4,
              ),
              onPressed: () {
                HapticFeedback.lightImpact();
                Navigator.pop(context);
              },
            ),
          ),
          SizedBox(width: dimensions.spacing / 4),
          Text(
            'Create Account',
            style: TextStyle(
              color: Colors.white,
              fontSize: dimensions.buttonTextSize + 4,
              fontWeight: FontWeight.bold,
              letterSpacing: 0.5,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildHeaderSection(SignupResponsiveDimensions dimensions) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        AnimatedBuilder(
          animation: _pulseAnimation,
          builder: (context, child) {
            return Transform.scale(
              scale: _pulseAnimation.value,
              child: Container(
                padding: EdgeInsets.all(dimensions.spacing / 3),
                decoration: BoxDecoration(
                  gradient: RadialGradient(
                    colors: [
                      Colors.redAccent.withOpacity(0.1),
                      Colors.transparent,
                    ],
                  ),
                  borderRadius: BorderRadius.circular(20),
                ),
                child: Icon(
                  Icons.person_add_alt_1,
                  size: dimensions.iconSize,
                  color: Colors.redAccent.withOpacity(0.8),
                ),
              ),
            );
          },
        ),
        SizedBox(height: dimensions.spacing / 2),
        ShaderMask(
          shaderCallback: (bounds) => const LinearGradient(
            colors: [
              Colors.white,
              Colors.redAccent,
              Colors.white,
            ],
          ).createShader(bounds),
          child: Text(
            'Join the System',
            style: TextStyle(
              color: Colors.white,
              fontSize: dimensions.titleSize,
              fontWeight: FontWeight.bold,
              letterSpacing: 1.2,
            ),
          ),
        ),
        SizedBox(height: dimensions.spacing / 4),
        Text(
          'Create your AMR control account to access advanced robotics features and fleet management tools.',
          style: TextStyle(
            color: Colors.white60,
            fontSize: dimensions.subtitleSize,
            height: 1.5,
            letterSpacing: 0.3,
          ),
        ),
      ],
    );
  }

  Widget _buildSignupForm(SignupResponsiveDimensions dimensions) {
    return Container(
      padding: EdgeInsets.all(dimensions.cardPadding),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(20),
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.white.withOpacity(0.05),
            Colors.white.withOpacity(0.02),
          ],
        ),
        border: Border.all(
          color: Colors.white.withOpacity(0.1),
          width: 1,
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.3),
            blurRadius: 20,
            offset: const Offset(0, 10),
          ),
        ],
      ),
      child: Column(
        children: [
          // Username field
          _buildTextField(
            controller: _usernameController,
            hint: 'Username',
            icon: Icons.person_outline,
            obscureText: false,
            dimensions: dimensions,
          ),

          SizedBox(height: dimensions.spacing / 2.5),

          // Password field
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

          SizedBox(height: dimensions.spacing / 2.5),

          // Confirm password field
          _buildTextField(
            controller: _confirmPasswordController,
            hint: 'Confirm Password',
            icon: Icons.lock_outline,
            obscureText: _obscureConfirmPassword,
            dimensions: dimensions,
            suffixIcon: IconButton(
              icon: Icon(
                _obscureConfirmPassword
                    ? Icons.visibility_off_outlined
                    : Icons.visibility_outlined,
                color: Colors.white54,
                size: dimensions.buttonTextSize + 4,
              ),
              onPressed: () {
                setState(() {
                  _obscureConfirmPassword = !_obscureConfirmPassword;
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
    required SignupResponsiveDimensions dimensions,
    Widget? suffixIcon,
  }) {
    return Container(
      height: dimensions.textFieldHeight,
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(15),
        gradient: LinearGradient(
          colors: [
            Colors.white.withOpacity(0.08),
            Colors.white.withOpacity(0.04),
          ],
        ),
        border: Border.all(
          color: Colors.white.withOpacity(0.15),
          width: 1,
        ),
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
            margin: EdgeInsets.all(dimensions.iconSize / 4),
            padding: EdgeInsets.all(dimensions.iconSize / 4),
            decoration: BoxDecoration(
              color: Colors.redAccent.withOpacity(0.2),
              borderRadius: BorderRadius.circular(8),
            ),
            child: Icon(
              icon,
              color: Colors.redAccent,
              size: dimensions.iconSize,
            ),
          ),
          suffixIcon: suffixIcon,
          border: InputBorder.none,
          contentPadding: EdgeInsets.symmetric(
            horizontal: dimensions.horizontalPadding,
            vertical: dimensions.verticalPadding,
          ),
        ),
      ),
    );
  }

  Widget _buildSignupButton(SignupResponsiveDimensions dimensions) {
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
            color: Colors.redAccent.withOpacity(0.4),
            blurRadius: 15,
            offset: const Offset(0, 8),
          ),
        ],
      ),
      child: ElevatedButton(
        onPressed: _isLoading ? null : _saveCredentialsAndGoToLogin,
        style: ElevatedButton.styleFrom(
          backgroundColor: Colors.transparent,
          shadowColor: Colors.transparent,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(16),
          ),
        ),
        child: _isLoading
            ? SizedBox(
                width: dimensions.buttonTextSize,
                height: dimensions.buttonTextSize,
                child: const CircularProgressIndicator(
                  strokeWidth: 2,
                  valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
                ),
              )
            : Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(
                    Icons.rocket_launch,
                    color: Colors.white,
                    size: dimensions.buttonTextSize + 4,
                  ),
                  SizedBox(width: dimensions.spacing / 2),
                  Text(
                    'CREATE ACCOUNT',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: dimensions.buttonTextSize,
                      fontWeight: FontWeight.bold,
                      letterSpacing: 1.2,
                    ),
                  ),
                ],
              ),
      ),
    );
  }

  Widget _buildPasswordStrengthIndicator(SignupResponsiveDimensions dimensions) {
    if (_passwordController.text.isEmpty) return const SizedBox.shrink();

    return Container(
      padding: EdgeInsets.all(dimensions.cardPadding),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(15),
        gradient: LinearGradient(
          colors: [
            Colors.blue.withOpacity(0.05),
            Colors.blue.withOpacity(0.02),
          ],
        ),
        border: Border.all(
          color: Colors.blue.withOpacity(0.2),
          width: 1,
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(
                Icons.security,
                color: Colors.blue.withOpacity(0.7),
                size: dimensions.iconSize - 10,
              ),
              SizedBox(width: dimensions.spacing / 4),
              Text(
                'Password Security',
                style: TextStyle(
                  color: Colors.white,
                  fontSize: dimensions.subtitleSize,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ],
          ),
          SizedBox(height: dimensions.spacing / 2),
          _buildStrengthItem('At least 8 characters', _hasMinLength, dimensions),
          _buildStrengthItem('Contains uppercase letter', _hasUppercase, dimensions),
          _buildStrengthItem('Contains lowercase letter', _hasLowercase, dimensions),
          _buildStrengthItem('Contains number', _hasNumber, dimensions),
        ],
      ),
    );
  }

  Widget _buildStrengthItem(String text, bool isValid, SignupResponsiveDimensions dimensions) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: dimensions.spacing / 4),
      child: Row(
        children: [
          Icon(
            isValid ? Icons.check_circle : Icons.radio_button_unchecked,
            color: isValid ? Colors.green : Colors.white.withOpacity(0.3),
            size: dimensions.iconSize - 10,
          ),
          SizedBox(width: dimensions.spacing / 4),
          Text(
            text,
            style: TextStyle(
              color: isValid ? Colors.green : Colors.white.withOpacity(0.6),
              fontSize: dimensions.subtitleSize - 2,
              fontWeight: isValid ? FontWeight.w500 : FontWeight.normal,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildTermsCheckbox(SignupResponsiveDimensions dimensions) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(12),
        border: Border.all(
          color: Colors.white.withOpacity(0.1),
          width: 1,
        ),
      ),
      child: Row(
        children: [
          Transform.scale(
            scale: 1.2,
            child: Checkbox(
              value: _acceptTerms,
              onChanged: (value) {
                setState(() {
                  _acceptTerms = value ?? false;
                });
                HapticFeedback.lightImpact();
              },
              fillColor: MaterialStateProperty.resolveWith((states) {
                if (states.contains(MaterialState.selected)) {
                  return Colors.redAccent;
                }
                return Colors.transparent;
              }),
              side: BorderSide(
                color: Colors.white.withOpacity(0.3),
                width: 2,
              ),
            ),
          ),
          const SizedBox(width: 12),
          Expanded(
            child: RichText(
              text: TextSpan(
                style: TextStyle(
                  color: Colors.white.withOpacity(0.7),
                  fontSize: dimensions.subtitleSize - 4,
                ),
                children: [
                  const TextSpan(text: 'I accept the '),
                  TextSpan(
                    text: 'Terms of Service',
                    style: TextStyle(
                      color: Colors.redAccent,
                      decoration: TextDecoration.underline,
                      decorationColor: Colors.redAccent,
                    ),
                  ),
                  const TextSpan(text: ' and '),
                  TextSpan(
                    text: 'Privacy Policy',
                    style: TextStyle(
                      color: Colors.redAccent,
                      decoration: TextDecoration.underline,
                      decorationColor: Colors.redAccent,
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildLoginSection(SignupResponsiveDimensions dimensions) {
    return Container(
      padding: const EdgeInsets.all(20),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(15),
        border: Border.all(
          color: Colors.white.withOpacity(0.1),
          width: 1,
        ),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Text(
            'Already have an account?',
            style: TextStyle(
              color: Colors.white.withOpacity(0.7),
              fontSize: dimensions.subtitleSize - 2,
            ),
          ),
          const SizedBox(width: 8),
          TextButton(
            onPressed: () {
              HapticFeedback.lightImpact();
              Navigator.pop(context);
            },
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [
                    Colors.blue.withOpacity(0.2),
                    Colors.blue.withOpacity(0.1),
                  ],
                ),
                borderRadius: BorderRadius.circular(20),
                border: Border.all(
                  color: Colors.blue.withOpacity(0.3),
                  width: 1,
                ),
              ),
              child: const Text(
                'SIGN IN',
                style: TextStyle(
                  color: Colors.blueAccent,
                  fontWeight: FontWeight.bold,
                  fontSize: 12,
                  letterSpacing: 1,
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildBackgroundElement(int index, SignupResponsiveDimensions dimensions) {
    final random = (index * 98765) % 1000;
    final screenSize = MediaQuery.of(context).size;
    final left = (random % 100) / 100 * screenSize.width;
    final top = ((random ~/ 100) % 100) / 100 * screenSize.height;
    // Adjust size for larger screens
    final baseSize = screenSize.width > 400 ? 20.0 : 15.0;
    final size = baseSize + (random % 30);

    return Positioned(
      left: left,
      top: top,
      child: AnimatedBuilder(
        animation: _pulseController,
        builder: (context, child) {
          final opacity =
              0.02 + ((_pulseController.value + (index * 0.2)) % 1.0) * 0.04;
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

// Responsive dimensions class
enum DeviceType { mobile, tablet, desktop }

class SignupResponsiveDimensions {
  final double horizontalPadding;
  final double verticalPadding;
  final double appBarPadding;
  final double iconSize;
  final double titleSize;
  final double subtitleSize;
  final double cardPadding;
  final double textFieldHeight;
  final double buttonHeight;
  final double buttonTextSize;
  final double spacing;
  final double maxWidth;
  final int particleCount;
  final double particleBaseSize;

  SignupResponsiveDimensions({
    required this.horizontalPadding,
    required this.verticalPadding,
    required this.appBarPadding,
    required this.iconSize,
    required this.titleSize,
    required this.subtitleSize,
    required this.cardPadding,
    required this.textFieldHeight,
    required this.buttonHeight,
    required this.buttonTextSize,
    required this.spacing,
    required this.maxWidth,
    required this.particleCount,
    required this.particleBaseSize,
  });
}
