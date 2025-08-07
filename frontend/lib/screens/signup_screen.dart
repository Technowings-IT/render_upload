// enhanced_signup_screen.dart
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart';

class EnhancedSignupScreen extends StatefulWidget {
  const EnhancedSignupScreen({
    super.key,
  });

  @override
  State<EnhancedSignupScreen> createState() => _EnhancedSignupScreenState();
}

class _EnhancedSignupScreenState extends State<EnhancedSignupScreen>
    with TickerProviderStateMixin {
  final TextEditingController _fullNameController = TextEditingController();
  final TextEditingController _emailController = TextEditingController();
  final TextEditingController _usernameController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();
  final TextEditingController _confirmPasswordController =
      TextEditingController();

  bool _obscurePassword = true;
  bool _obscureConfirmPassword = true;
  bool _isLoading = false;
  bool _acceptTerms = false;
  bool _subscribeNewsletter = false;

  late AnimationController _fadeController;
  late AnimationController _slideController;
  late AnimationController _pulseController;
  late AnimationController _progressController;
  late Animation<double> _fadeAnimation;
  late Animation<Offset> _slideAnimation;
  late Animation<double> _pulseAnimation;
  late Animation<double> _progressAnimation;

  // Password strength indicators
  bool _hasMinLength = false;
  bool _hasUppercase = false;
  bool _hasLowercase = false;
  bool _hasNumber = false;
  bool _hasSpecialChar = false;

  // Form validation
  final _formKey = GlobalKey<FormState>();
  int _currentStep = 0;

  @override
  void initState() {
    super.initState();
    _initializeAnimations();
    _passwordController.addListener(_checkPasswordStrength);
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

    _progressController = AnimationController(
      duration: const Duration(milliseconds: 500),
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

    _progressAnimation = Tween<double>(
      begin: 0.0,
      end: 1.0,
    ).animate(CurvedAnimation(
      parent: _progressController,
      curve: Curves.easeInOut,
    ));

    _fadeController.forward();
    _slideController.forward();
    _pulseController.repeat(reverse: true);
  }

  @override
  void dispose() {
    _fadeController.dispose();
    _slideController.dispose();
    _pulseController.dispose();
    _progressController.dispose();
    _fullNameController.dispose();
    _emailController.dispose();
    _usernameController.dispose();
    _passwordController.dispose();
    _confirmPasswordController.dispose();
    super.dispose();
  }

  ThemeColors _getThemeColors() {
    return ThemeColors.light();
  }

  DeviceType _getDeviceType(BuildContext context) {
    final screenWidth = MediaQuery.of(context).size.width;
    final screenHeight = MediaQuery.of(context).size.height;
    final aspectRatio = screenWidth / screenHeight;

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

  SignupResponsiveDimensions _getResponsiveDimensions(BuildContext context) {
    final deviceType = _getDeviceType(context);
    final screenHeight = MediaQuery.of(context).size.height;
    final isCompact = screenHeight < 700;

    switch (deviceType) {
      case DeviceType.desktop:
        return SignupResponsiveDimensions(
          horizontalPadding: 80.0,
          verticalPadding: isCompact ? 35.0 : 50.0,
          appBarPadding: 36.0,
          iconSize: 65.0,
          titleSize: 44.0,
          subtitleSize: 20.0,
          cardPadding: 52.0,
          textFieldHeight: 76.0,
          buttonHeight: 80.0,
          buttonTextSize: 20.0,
          spacing: isCompact ? 55.0 : 65.0,
          maxWidth: 750.0,
          particleCount: 18,
          particleBaseSize: 35.0,
          borderRadius: 24.0,
          stepIndicatorSize: 14.0,
        );
      case DeviceType.laptop:
        return SignupResponsiveDimensions(
          horizontalPadding: 70.0,
          verticalPadding: isCompact ? 32.0 : 45.0,
          appBarPadding: 32.0,
          iconSize: 58.0,
          titleSize: 40.0,
          subtitleSize: 19.0,
          cardPadding: 46.0,
          textFieldHeight: 70.0,
          buttonHeight: 74.0,
          buttonTextSize: 19.0,
          spacing: isCompact ? 48.0 : 58.0,
          maxWidth: 650.0,
          particleCount: 15,
          particleBaseSize: 30.0,
          borderRadius: 22.0,
          stepIndicatorSize: 13.0,
        );
      case DeviceType.tablet:
        final isLandscape =
            MediaQuery.of(context).orientation == Orientation.landscape;
        return SignupResponsiveDimensions(
          horizontalPadding: isLandscape ? 70.0 : 50.0,
          verticalPadding: isCompact ? 28.0 : 36.0,
          appBarPadding: 28.0,
          iconSize: 52.0,
          titleSize: 36.0,
          subtitleSize: 18.0,
          cardPadding: 38.0,
          textFieldHeight: 64.0,
          buttonHeight: 68.0,
          buttonTextSize: 18.0,
          spacing: isCompact ? 40.0 : 48.0,
          maxWidth: isLandscape ? 600.0 : 550.0,
          particleCount: 12,
          particleBaseSize: 25.0,
          borderRadius: 20.0,
          stepIndicatorSize: 12.0,
        );
      case DeviceType.mobile:
        return SignupResponsiveDimensions(
          horizontalPadding: 24.0,
          verticalPadding: isCompact ? 16.0 : 24.0,
          appBarPadding: 20.0,
          iconSize: 46.0,
          titleSize: 30.0,
          subtitleSize: 16.0,
          cardPadding: 24.0,
          textFieldHeight: 56.0,
          buttonHeight: 56.0,
          buttonTextSize: 16.0,
          spacing: isCompact ? 28.0 : 36.0,
          maxWidth: double.infinity,
          particleCount: 8,
          particleBaseSize: 20.0,
          borderRadius: 18.0,
          stepIndicatorSize: 10.0,
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
      _hasSpecialChar = password.contains(RegExp(r'[!@#$%^&*(),.?":{}|<>]'));
    });

    // Update progress animation
    final strength = _calculatePasswordStrength();
    _progressController.animateTo(strength);
  }

  double _calculatePasswordStrength() {
    int count = 0;
    if (_hasMinLength) count++;
    if (_hasUppercase) count++;
    if (_hasLowercase) count++;
    if (_hasNumber) count++;
    if (_hasSpecialChar) count++;
    return count / 5.0;
  }

  Color _getPasswordStrengthColor() {
    final strength = _calculatePasswordStrength();
    if (strength <= 0.2) return Colors.red;
    if (strength <= 0.4) return Colors.orange;
    if (strength <= 0.6) return Colors.amber;
    if (strength <= 0.8) return Colors.lightGreen;
    return Colors.green;
  }

  String _getPasswordStrengthText() {
    final strength = _calculatePasswordStrength();
    if (strength <= 0.2) return 'Very Weak';
    if (strength <= 0.4) return 'Weak';
    if (strength <= 0.6) return 'Fair';
    if (strength <= 0.8) return 'Good';
    return 'Strong';
  }

  Future<void> _handleSignup() async {
    if (!_validateForm()) return;

    setState(() {
      _isLoading = true;
    });

    HapticFeedback.mediumImpact();

    try {
      await Future.delayed(const Duration(milliseconds: 1500));

      final prefs = await SharedPreferences.getInstance();
      await prefs.setString('saved_username', _usernameController.text);
      await prefs.setString('saved_password', _passwordController.text);
      await prefs.setString('saved_email', _emailController.text);
      await prefs.setString('saved_fullname', _fullNameController.text);

      HapticFeedback.lightImpact();
      _showSnackBar(
          'Account created successfully! Redirecting to login...', true);

      await Future.delayed(const Duration(milliseconds: 1000));
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
    // Step 1: Personal Info
    if (_fullNameController.text.isEmpty) {
      _showSnackBar('Please enter your full name', false);
      return false;
    }

    if (_emailController.text.isEmpty || !_emailController.text.contains('@')) {
      _showSnackBar('Please enter a valid email address', false);
      return false;
    }

    // Step 2: Account Details
    if (_usernameController.text.isEmpty ||
        _usernameController.text.length < 3) {
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

    // Step 3: Terms
    if (!_acceptTerms) {
      _showSnackBar('Please accept the terms and conditions', false);
      return false;
    }

    return true;
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
              size: dimensions.iconSize / 2,
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
    final colors = _getThemeColors();
    final deviceType = _getDeviceType(context);

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
            // Animated background
            if (deviceType != DeviceType.mobile)
              ...List.generate(
                  dimensions.particleCount,
                  (index) =>
                      _buildBackgroundElement(index, dimensions, colors)),

            // Main content
            SafeArea(
              child: Column(
                children: [
                  _buildCustomAppBar(dimensions, colors),

                  // Step indicators
                  if (deviceType != DeviceType.mobile)
                    _buildStepIndicator(dimensions, colors),

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
                                  child: Form(
                                    key: _formKey,
                                    child: Column(
                                      crossAxisAlignment:
                                          CrossAxisAlignment.start,
                                      children: [
                                        _buildHeaderSection(dimensions, colors),
                                        SizedBox(height: dimensions.spacing),
                                        _buildFormSection(
                                            dimensions, colors, deviceType),
                                        SizedBox(height: dimensions.spacing),
                                        _buildSignupButton(dimensions, colors),
                                        SizedBox(height: dimensions.spacing),
                                        _buildLoginSection(dimensions, colors),
                                      ],
                                    ),
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

  Widget _buildCustomAppBar(
      SignupResponsiveDimensions dimensions, ThemeColors colors) {
    return Container(
      padding: EdgeInsets.all(dimensions.appBarPadding),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            colors.overlay.withOpacity(0.05),
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
              color: colors.primary.withOpacity(0.1),
              borderRadius:
                  BorderRadius.circular(dimensions.borderRadius / 1.5),
              border: Border.all(
                color: colors.primary.withOpacity(0.3),
                width: 1,
              ),
            ),
            child: IconButton(
              icon: Icon(
                Icons.arrow_back_ios_new_rounded,
                color: colors.primary,
                size: dimensions.iconSize / 2,
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
              color: colors.textPrimary,
              fontSize: dimensions.buttonTextSize + 4,
              fontWeight: FontWeight.bold,
              letterSpacing: 0.5,
            ),
          ),
          const Spacer(),
        ],
      ),
    );
  }

  Widget _buildStepIndicator(
      SignupResponsiveDimensions dimensions, ThemeColors colors) {
    return Container(
      padding: EdgeInsets.symmetric(
        horizontal: dimensions.horizontalPadding,
        vertical: dimensions.spacing / 2,
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: List.generate(3, (index) {
          final isActive = index == _currentStep;
          final isCompleted = index < _currentStep;

          return Flexible(
            child: Row(
              children: [
                AnimatedContainer(
                  duration: const Duration(milliseconds: 300),
                  width: dimensions.stepIndicatorSize * 3,
                  height: dimensions.stepIndicatorSize * 3,
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    color: isActive || isCompleted
                        ? colors.primary
                        : colors.border,
                    boxShadow: isActive
                        ? [
                            BoxShadow(
                              color: colors.primary.withOpacity(0.4),
                              blurRadius: 10,
                              spreadRadius: 2,
                            ),
                          ]
                        : [],
                  ),
                  child: Center(
                    child: isCompleted
                        ? Icon(
                            Icons.check_rounded,
                            color: colors.buttonText,
                            size: dimensions.stepIndicatorSize * 1.5,
                          )
                        : Text(
                            '${index + 1}',
                            style: TextStyle(
                              color: isActive
                                  ? colors.buttonText
                                  : colors.textSecondary,
                              fontWeight: FontWeight.bold,
                              fontSize: dimensions.stepIndicatorSize,
                            ),
                          ),
                  ),
                ),
                if (index < 2)
                  Expanded(
                    child: Container(
                      height: 2,
                      margin: EdgeInsets.symmetric(
                          horizontal: dimensions.spacing / 4),
                      decoration: BoxDecoration(
                        color: isCompleted ? colors.primary : colors.border,
                        borderRadius: BorderRadius.circular(1),
                      ),
                    ),
                  ),
              ],
            ),
          );
        }),
      ),
    );
  }

  Widget _buildHeaderSection(
      SignupResponsiveDimensions dimensions, ThemeColors colors) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        AnimatedBuilder(
          animation: _pulseAnimation,
          builder: (context, child) {
            return Transform.scale(
              scale: _pulseAnimation.value,
              child: Container(
                padding: EdgeInsets.all(dimensions.spacing / 2.5),
                decoration: BoxDecoration(
                  gradient: RadialGradient(
                    colors: [
                      colors.primary.withOpacity(0.1),
                      Colors.transparent,
                    ],
                  ),
                  borderRadius: BorderRadius.circular(dimensions.borderRadius),
                ),
                child: Icon(
                  Icons.rocket_launch_rounded,
                  size: dimensions.iconSize,
                  color: colors.primary,
                ),
              ),
            );
          },
        ),
        SizedBox(height: dimensions.spacing / 2),
        ShaderMask(
          shaderCallback: (bounds) => LinearGradient(
            colors: [
              colors.textPrimary,
              colors.primary,
              colors.textPrimary,
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
          'Create your AMR control account to access advanced robotics features',
          style: TextStyle(
            color: colors.textSecondary,
            fontSize: dimensions.subtitleSize,
            height: 1.5,
            letterSpacing: 0.3,
          ),
        ),
      ],
    );
  }

  Widget _buildFormSection(SignupResponsiveDimensions dimensions,
      ThemeColors colors, DeviceType deviceType) {
    final useColumns =
        deviceType == DeviceType.desktop || deviceType == DeviceType.laptop;

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
            blurRadius: 20,
            offset: const Offset(0, 10),
          ),
        ],
      ),
      child: Column(
        children: [
          // Personal Information
          if (useColumns) ...[
            Row(
              children: [
                Expanded(
                  child: _buildTextField(
                    controller: _fullNameController,
                    hint: 'Full Name',
                    icon: Icons.badge_rounded,
                    obscureText: false,
                    dimensions: dimensions,
                    colors: colors,
                  ),
                ),
                SizedBox(width: dimensions.spacing / 2),
                Expanded(
                  child: _buildTextField(
                    controller: _emailController,
                    hint: 'Email Address',
                    icon: Icons.email_rounded,
                    obscureText: false,
                    dimensions: dimensions,
                    colors: colors,
                    keyboardType: TextInputType.emailAddress,
                  ),
                ),
              ],
            ),
          ] else ...[
            _buildTextField(
              controller: _fullNameController,
              hint: 'Full Name',
              icon: Icons.badge_rounded,
              obscureText: false,
              dimensions: dimensions,
              colors: colors,
            ),
            SizedBox(height: dimensions.spacing / 2),
            _buildTextField(
              controller: _emailController,
              hint: 'Email Address',
              icon: Icons.email_rounded,
              obscureText: false,
              dimensions: dimensions,
              colors: colors,
              keyboardType: TextInputType.emailAddress,
            ),
          ],

          SizedBox(height: dimensions.spacing / 2),

          // Username
          _buildTextField(
            controller: _usernameController,
            hint: 'Username',
            icon: Icons.person_rounded,
            obscureText: false,
            dimensions: dimensions,
            colors: colors,
          ),

          SizedBox(height: dimensions.spacing / 2),

          // Password fields
          if (useColumns) ...[
            Row(
              children: [
                Expanded(
                  child: _buildTextField(
                    controller: _passwordController,
                    hint: 'Password',
                    icon: Icons.lock_rounded,
                    obscureText: _obscurePassword,
                    dimensions: dimensions,
                    colors: colors,
                    suffixIcon: IconButton(
                      icon: Icon(
                        _obscurePassword
                            ? Icons.visibility_off_rounded
                            : Icons.visibility_rounded,
                        color: colors.textSecondary,
                        size: dimensions.iconSize / 2,
                      ),
                      onPressed: () {
                        setState(() {
                          _obscurePassword = !_obscurePassword;
                        });
                        HapticFeedback.lightImpact();
                      },
                    ),
                  ),
                ),
                SizedBox(width: dimensions.spacing / 2),
                Expanded(
                  child: _buildTextField(
                    controller: _confirmPasswordController,
                    hint: 'Confirm Password',
                    icon: Icons.lock_rounded,
                    obscureText: _obscureConfirmPassword,
                    dimensions: dimensions,
                    colors: colors,
                    suffixIcon: IconButton(
                      icon: Icon(
                        _obscureConfirmPassword
                            ? Icons.visibility_off_rounded
                            : Icons.visibility_rounded,
                        color: colors.textSecondary,
                        size: dimensions.iconSize / 2,
                      ),
                      onPressed: () {
                        setState(() {
                          _obscureConfirmPassword = !_obscureConfirmPassword;
                        });
                        HapticFeedback.lightImpact();
                      },
                    ),
                  ),
                ),
              ],
            ),
          ] else ...[
            _buildTextField(
              controller: _passwordController,
              hint: 'Password',
              icon: Icons.lock_rounded,
              obscureText: _obscurePassword,
              dimensions: dimensions,
              colors: colors,
              suffixIcon: IconButton(
                icon: Icon(
                  _obscurePassword
                      ? Icons.visibility_off_rounded
                      : Icons.visibility_rounded,
                  color: colors.textSecondary,
                  size: dimensions.iconSize / 2,
                ),
                onPressed: () {
                  setState(() {
                    _obscurePassword = !_obscurePassword;
                  });
                  HapticFeedback.lightImpact();
                },
              ),
            ),
            SizedBox(height: dimensions.spacing / 2),
            _buildTextField(
              controller: _confirmPasswordController,
              hint: 'Confirm Password',
              icon: Icons.lock_rounded,
              obscureText: _obscureConfirmPassword,
              dimensions: dimensions,
              colors: colors,
              suffixIcon: IconButton(
                icon: Icon(
                  _obscureConfirmPassword
                      ? Icons.visibility_off_rounded
                      : Icons.visibility_rounded,
                  color: colors.textSecondary,
                  size: dimensions.iconSize / 2,
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

          // Password strength indicator
          if (_passwordController.text.isNotEmpty) ...[
            SizedBox(height: dimensions.spacing / 2),
            _buildPasswordStrengthIndicator(dimensions, colors),
          ],

          SizedBox(height: dimensions.spacing / 2),

          // Terms and newsletter
          _buildCheckboxes(dimensions, colors),
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
    required ThemeColors colors,
    Widget? suffixIcon,
    TextInputType? keyboardType,
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
        keyboardType: keyboardType,
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
              size: dimensions.iconSize / 2,
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

  Widget _buildPasswordStrengthIndicator(
      SignupResponsiveDimensions dimensions, ThemeColors colors) {
    return Container(
      padding: EdgeInsets.all(dimensions.spacing / 2),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(dimensions.borderRadius / 2),
        color: colors.overlay.withOpacity(0.03),
        border: Border.all(
          color: _getPasswordStrengthColor().withOpacity(0.3),
          width: 1,
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(
                Icons.security_rounded,
                color: _getPasswordStrengthColor(),
                size: dimensions.iconSize / 2.5,
              ),
              SizedBox(width: dimensions.spacing / 4),
              Text(
                'Password Strength: ${_getPasswordStrengthText()}',
                style: TextStyle(
                  color: _getPasswordStrengthColor(),
                  fontSize: dimensions.subtitleSize - 2,
                  fontWeight: FontWeight.w600,
                ),
              ),
              const Spacer(),
              AnimatedBuilder(
                animation: _progressAnimation,
                builder: (context, child) {
                  return SizedBox(
                    width: 100,
                    child: LinearProgressIndicator(
                      value: _progressAnimation.value,
                      backgroundColor: colors.border,
                      valueColor: AlwaysStoppedAnimation<Color>(
                        _getPasswordStrengthColor(),
                      ),
                      minHeight: 6,
                      borderRadius: BorderRadius.circular(3),
                    ),
                  );
                },
              ),
            ],
          ),
          SizedBox(height: dimensions.spacing / 3),
          Wrap(
            spacing: dimensions.spacing / 3,
            runSpacing: dimensions.spacing / 4,
            children: [
              _buildStrengthChip('8+ chars', _hasMinLength, dimensions, colors),
              _buildStrengthChip(
                  'Uppercase', _hasUppercase, dimensions, colors),
              _buildStrengthChip(
                  'Lowercase', _hasLowercase, dimensions, colors),
              _buildStrengthChip('Number', _hasNumber, dimensions, colors),
              _buildStrengthChip(
                  'Special', _hasSpecialChar, dimensions, colors),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildStrengthChip(String text, bool isValid,
      SignupResponsiveDimensions dimensions, ThemeColors colors) {
    return AnimatedContainer(
      duration: const Duration(milliseconds: 300),
      padding: EdgeInsets.symmetric(
        horizontal: dimensions.spacing / 2.5,
        vertical: dimensions.spacing / 5,
      ),
      decoration: BoxDecoration(
        color: isValid
            ? colors.success.withOpacity(0.1)
            : colors.border.withOpacity(0.2),
        borderRadius: BorderRadius.circular(dimensions.borderRadius / 3),
        border: Border.all(
          color: isValid ? colors.success : colors.border,
          width: 1,
        ),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(
            isValid ? Icons.check_circle_rounded : Icons.circle_outlined,
            color: isValid ? colors.success : colors.textSecondary,
            size: dimensions.iconSize / 3,
          ),
          SizedBox(width: dimensions.spacing / 6),
          Text(
            text,
            style: TextStyle(
              color: isValid ? colors.success : colors.textSecondary,
              fontSize: dimensions.subtitleSize - 4,
              fontWeight: isValid ? FontWeight.w600 : FontWeight.normal,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildCheckboxes(
      SignupResponsiveDimensions dimensions, ThemeColors colors) {
    return Column(
      children: [
        Row(
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
                activeColor: colors.primary,
                side: BorderSide(
                  color: colors.border,
                  width: 2,
                ),
              ),
            ),
            Expanded(
              child: RichText(
                text: TextSpan(
                  style: TextStyle(
                    color: colors.textSecondary,
                    fontSize: dimensions.subtitleSize - 3,
                  ),
                  children: [
                    const TextSpan(text: 'I accept the '),
                    TextSpan(
                      text: 'Terms of Service',
                      style: TextStyle(
                        color: colors.primary,
                        decoration: TextDecoration.underline,
                        decorationColor: colors.primary,
                        fontWeight: FontWeight.w600,
                      ),
                    ),
                    const TextSpan(text: ' and '),
                    TextSpan(
                      text: 'Privacy Policy',
                      style: TextStyle(
                        color: colors.primary,
                        decoration: TextDecoration.underline,
                        decorationColor: colors.primary,
                        fontWeight: FontWeight.w600,
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
        Row(
          children: [
            Transform.scale(
              scale: 1.2,
              child: Checkbox(
                value: _subscribeNewsletter,
                onChanged: (value) {
                  setState(() {
                    _subscribeNewsletter = value ?? false;
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
            Expanded(
              child: Text(
                'Subscribe to newsletter and updates',
                style: TextStyle(
                  color: colors.textSecondary,
                  fontSize: dimensions.subtitleSize - 3,
                ),
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildSignupButton(
      SignupResponsiveDimensions dimensions, ThemeColors colors) {
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
            blurRadius: 15,
            offset: const Offset(0, 8),
          ),
        ],
      ),
      child: ElevatedButton(
        onPressed: _isLoading ? null : _handleSignup,
        style: ElevatedButton.styleFrom(
          backgroundColor: Colors.transparent,
          shadowColor: Colors.transparent,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(dimensions.borderRadius / 1.5),
          ),
        ),
        child: _isLoading
            ? SizedBox(
                width: dimensions.iconSize / 2,
                height: dimensions.iconSize / 2,
                child: CircularProgressIndicator(
                  strokeWidth: 2,
                  valueColor: AlwaysStoppedAnimation<Color>(colors.buttonText),
                ),
              )
            : Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(
                    Icons.rocket_launch_rounded,
                    color: colors.buttonText,
                    size: dimensions.iconSize / 2,
                  ),
                  SizedBox(width: dimensions.spacing / 3),
                  Text(
                    'CREATE ACCOUNT',
                    style: TextStyle(
                      color: colors.buttonText,
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

  Widget _buildLoginSection(
      SignupResponsiveDimensions dimensions, ThemeColors colors) {
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
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Text(
            'Already have an account?',
            style: TextStyle(
              color: colors.textSecondary,
              fontSize: dimensions.subtitleSize - 2,
            ),
          ),
          SizedBox(width: dimensions.spacing / 4),
          TextButton(
            onPressed: () {
              HapticFeedback.lightImpact();
              Navigator.pop(context);
            },
            style: TextButton.styleFrom(
              padding: EdgeInsets.symmetric(
                horizontal: dimensions.spacing / 2,
                vertical: dimensions.spacing / 4,
              ),
              backgroundColor: colors.secondary.withOpacity(0.1),
              shape: RoundedRectangleBorder(
                borderRadius:
                    BorderRadius.circular(dimensions.borderRadius / 2),
              ),
            ),
            child: Text(
              'SIGN IN',
              style: TextStyle(
                color: colors.secondary,
                fontWeight: FontWeight.bold,
                fontSize: dimensions.subtitleSize - 4,
                letterSpacing: 1,
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildBackgroundElement(
      int index, SignupResponsiveDimensions dimensions, ThemeColors colors) {
    final random = (index * 98765) % 1000;
    final screenSize = MediaQuery.of(context).size;
    final left = (random % 100) / 100 * screenSize.width;
    final top = ((random ~/ 100) % 100) / 100 * screenSize.height;
    final size = dimensions.particleBaseSize + (random % 40);

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
                    colors.primary.withOpacity(0.15),
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
}

// Responsive dimensions class
enum DeviceType { mobile, tablet, laptop, desktop }

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
  final double borderRadius;
  final double stepIndicatorSize;

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
    required this.borderRadius,
    required this.stepIndicatorSize,
  });
}

// Theme colors class (shared with login)
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
