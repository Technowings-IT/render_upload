// splash_screen.dart - Enhanced Modern AMR Design
import 'package:flutter/material.dart';
import 'login_screen.dart';

class SplashScreen extends StatefulWidget {
  const SplashScreen({super.key});

  @override
  State<SplashScreen> createState() => _SplashScreenState();
}

class _SplashScreenState extends State<SplashScreen>
    with TickerProviderStateMixin {
  late AnimationController _logoController;
  late AnimationController _progressController;
  late AnimationController _pulseController;
  late Animation<double> _logoAnimation;
  late Animation<double> _progressAnimation;
  late Animation<double> _pulseAnimation;

  @override
  void initState() {
    super.initState();

    // Initialize animation controllers
    _logoController = AnimationController(
      duration: const Duration(milliseconds: 1200),
      vsync: this,
    );

    _progressController = AnimationController(
      duration: const Duration(milliseconds: 2000),
      vsync: this,
    );

    _pulseController = AnimationController(
      duration: const Duration(milliseconds: 1500),
      vsync: this,
    );

    // Initialize animations
    _logoAnimation = Tween<double>(
      begin: 0.0,
      end: 1.0,
    ).animate(CurvedAnimation(
      parent: _logoController,
      curve: Curves.elasticOut,
    ));

    _progressAnimation = Tween<double>(
      begin: 0.0,
      end: 1.0,
    ).animate(CurvedAnimation(
      parent: _progressController,
      curve: Curves.easeInOut,
    ));

    _pulseAnimation = Tween<double>(
      begin: 1.0,
      end: 1.1,
    ).animate(CurvedAnimation(
      parent: _pulseController,
      curve: Curves.easeInOut,
    ));

    // Start animations
    _startAnimations();

    // Navigate after delay
    Future.delayed(const Duration(seconds: 2), () {
      Navigator.of(context).pushReplacement(
        PageRouteBuilder(
          pageBuilder: (context, animation, secondaryAnimation) =>
              EnhancedLoginScreen(
                  // isDarkMode: false, // Set to your desired initial value
                  ),
          transitionsBuilder: (context, animation, secondaryAnimation, child) {
            return FadeTransition(
              opacity: animation,
              child: child,
            );
          },
          transitionDuration: const Duration(milliseconds: 700),
        ),
      );
    });
  }

  void _startAnimations() {
    Future.delayed(const Duration(milliseconds: 300), () {
      _logoController.forward();
    });

    Future.delayed(const Duration(milliseconds: 800), () {
      _progressController.forward();
    });

    _pulseController.repeat(reverse: true);
  }

  @override
  void dispose() {
    _logoController.dispose();
    _progressController.dispose();
    _pulseController.dispose();
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
  SplashResponsiveDimensions _getResponsiveDimensions(BuildContext context) {
    final deviceType = _getDeviceType(context);
    final screenSize = MediaQuery.of(context).size;

    switch (deviceType) {
      case DeviceType.desktop:
        return SplashResponsiveDimensions(
          logoSize: 160.0,
          logoPadding: 30.0,
          titleSize: 38.0,
          subtitleSize: 20.0,
          spacing: 60.0,
          progressWidth: screenSize.width * 0.4,
          progressHeight: 6.0,
          versionSize: 14.0,
          companySize: 12.0,
          particleCount: 25,
          particleBaseSize: 3.0,
          particleVariance: 4,
          glowBlur: 40.0,
          glowSpread: 15.0,
        );
      case DeviceType.tablet:
        return SplashResponsiveDimensions(
          logoSize: 140.0,
          logoPadding: 25.0,
          titleSize: 32.0,
          subtitleSize: 18.0,
          spacing: 50.0,
          progressWidth: screenSize.width * 0.5,
          progressHeight: 5.0,
          versionSize: 13.0,
          companySize: 11.0,
          particleCount: 20,
          particleBaseSize: 2.5,
          particleVariance: 3,
          glowBlur: 35.0,
          glowSpread: 12.0,
        );
      case DeviceType.mobile:
        return SplashResponsiveDimensions(
          logoSize: 120.0,
          logoPadding: 20.0,
          titleSize: 28.0,
          subtitleSize: 16.0,
          spacing: 40.0,
          progressWidth: screenSize.width * 0.7,
          progressHeight: 4.0,
          versionSize: 12.0,
          companySize: 10.0,
          particleCount: 15,
          particleBaseSize: 2.0,
          particleVariance: 2,
          glowBlur: 30.0,
          glowSpread: 10.0,
        );
    }
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
              Color(0xFF0A0A0A),
            ],
            stops: [0.0, 0.5, 1.0],
          ),
        ),
        child: Stack(
          children: [
            // Animated background particles
            ...List.generate(
              dimensions.particleCount,
              (index) => _buildBackgroundParticle(index, dimensions),
            ),

            // Main content
            Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  // Logo container with responsive glow effect
                  AnimatedBuilder(
                    animation: _logoAnimation,
                    builder: (context, child) {
                      return Transform.scale(
                        scale: _logoAnimation.value,
                        child: AnimatedBuilder(
                          animation: _pulseAnimation,
                          builder: (context, child) {
                            return Transform.scale(
                              scale: _pulseAnimation.value,
                              child: Container(
                                padding: EdgeInsets.all(dimensions.logoPadding),
                                decoration: BoxDecoration(
                                  shape: BoxShape.circle,
                                  boxShadow: [
                                    BoxShadow(
                                      color: Colors.redAccent.withOpacity(0.3),
                                      blurRadius: dimensions.glowBlur,
                                      spreadRadius: dimensions.glowSpread,
                                    ),
                                    BoxShadow(
                                      color: Colors.redAccent.withOpacity(0.1),
                                      blurRadius: dimensions.glowBlur * 2,
                                      spreadRadius: dimensions.glowSpread * 1.5,
                                    ),
                                  ],
                                ),
                                child: Container(
                                  padding: EdgeInsets.all(
                                      dimensions.logoPadding * 0.75),
                                  decoration: BoxDecoration(
                                    shape: BoxShape.circle,
                                    gradient: LinearGradient(
                                      begin: Alignment.topLeft,
                                      end: Alignment.bottomRight,
                                      colors: [
                                        Colors.redAccent.withOpacity(0.2),
                                        Colors.transparent,
                                      ],
                                    ),
                                    border: Border.all(
                                      color: Colors.redAccent.withOpacity(0.3),
                                      width: deviceType == DeviceType.desktop
                                          ? 3
                                          : 2,
                                    ),
                                  ),
                                  child: Image.asset(
                                    'assets/app_logo.png',
                                    width: dimensions.logoSize,
                                    height: dimensions.logoSize,
                                    fit: BoxFit.contain,
                                  ),
                                ),
                              ),
                            );
                          },
                        ),
                      );
                    },
                  ),

                  SizedBox(height: dimensions.spacing),

                  // App title with responsive styling
                  AnimatedBuilder(
                    animation: _logoAnimation,
                    builder: (context, child) {
                      return Opacity(
                        opacity: _logoAnimation.value,
                        child: Column(
                          children: [
                            ShaderMask(
                              shaderCallback: (bounds) => const LinearGradient(
                                colors: [
                                  Colors.white,
                                  Colors.redAccent,
                                  Colors.white,
                                ],
                              ).createShader(bounds),
                              child: Text(
                                'AMR CONTROL',
                                style: TextStyle(
                                  fontSize: dimensions.titleSize,
                                  fontWeight: FontWeight.bold,
                                  letterSpacing:
                                      deviceType == DeviceType.desktop ? 4 : 3,
                                  color: Colors.white,
                                ),
                              ),
                            ),
                            SizedBox(height: dimensions.spacing / 6),
                            Container(
                              height: deviceType == DeviceType.desktop ? 3 : 2,
                              width:
                                  deviceType == DeviceType.desktop ? 120 : 100,
                              decoration: BoxDecoration(
                                gradient: const LinearGradient(
                                  colors: [
                                    Colors.transparent,
                                    Colors.redAccent,
                                    Colors.transparent,
                                  ],
                                ),
                                borderRadius: BorderRadius.circular(1),
                              ),
                            ),
                            SizedBox(height: dimensions.spacing / 3),
                            Text(
                              'Autonomous Mobile Robot',
                              style: TextStyle(
                                color: Colors.white70,
                                fontSize: dimensions.subtitleSize,
                                letterSpacing:
                                    deviceType == DeviceType.desktop ? 2 : 1.5,
                                fontWeight: FontWeight.w300,
                              ),
                            ),
                          ],
                        ),
                      );
                    },
                  ),

                  SizedBox(height: dimensions.spacing * 2),

                  // Loading section with responsive progress bar
                  AnimatedBuilder(
                    animation: _progressAnimation,
                    builder: (context, child) {
                      return Opacity(
                        opacity: _progressAnimation.value,
                        child: Column(
                          children: [
                            // Loading text with responsive spacing
                            Row(
                              mainAxisAlignment: MainAxisAlignment.center,
                              children: [
                                Icon(
                                  Icons.memory,
                                  color: Colors.redAccent,
                                  size: dimensions.subtitleSize + 4,
                                ),
                                SizedBox(width: dimensions.spacing / 5),
                                Text(
                                  'INITIALIZING SYSTEM',
                                  style: TextStyle(
                                    color: Colors.white,
                                    fontSize: dimensions.subtitleSize,
                                    fontWeight: FontWeight.w500,
                                    letterSpacing:
                                        deviceType == DeviceType.desktop
                                            ? 1.5
                                            : 1.2,
                                  ),
                                ),
                                SizedBox(width: dimensions.spacing / 5),
                                _buildBlinkingDots(dimensions),
                              ],
                            ),

                            SizedBox(height: dimensions.spacing * 0.8),

                            // Enhanced responsive progress bar
                            Container(
                              width: dimensions.progressWidth,
                              height: dimensions.progressHeight,
                              decoration: BoxDecoration(
                                borderRadius: BorderRadius.circular(
                                    dimensions.progressHeight / 2),
                                color: Colors.white.withOpacity(0.1),
                              ),
                              child: Stack(
                                children: [
                                  // Background glow
                                  Container(
                                    decoration: BoxDecoration(
                                      borderRadius: BorderRadius.circular(
                                          dimensions.progressHeight / 2),
                                      boxShadow: [
                                        BoxShadow(
                                          color:
                                              Colors.redAccent.withOpacity(0.3),
                                          blurRadius: dimensions.glowBlur / 5,
                                          spreadRadius: 1,
                                        ),
                                      ],
                                    ),
                                  ),
                                  // Progress fill
                                  AnimatedBuilder(
                                    animation: _progressController,
                                    builder: (context, child) {
                                      return Container(
                                        width: dimensions.progressWidth *
                                            _progressController.value,
                                        decoration: BoxDecoration(
                                          borderRadius: BorderRadius.circular(
                                              dimensions.progressHeight / 2),
                                          gradient: const LinearGradient(
                                            colors: [
                                              Colors.redAccent,
                                              Color(0xFFFF6B6B),
                                              Colors.redAccent,
                                            ],
                                          ),
                                        ),
                                      );
                                    },
                                  ),
                                ],
                              ),
                            ),

                            SizedBox(height: dimensions.spacing / 3),

                            // Progress percentage
                            AnimatedBuilder(
                              animation: _progressController,
                              builder: (context, child) {
                                return Text(
                                  '${(_progressController.value * 100).toInt()}%',
                                  style: TextStyle(
                                    color: Colors.redAccent.withOpacity(0.8),
                                    fontSize: dimensions.subtitleSize - 2,
                                    fontWeight: FontWeight.w500,
                                    letterSpacing: 1,
                                  ),
                                );
                              },
                            ),
                          ],
                        ),
                      );
                    },
                  ),
                ],
              ),
            ),

            // Responsive version info
            Positioned(
              bottom: deviceType == DeviceType.desktop ? 40 : 30,
              left: 0,
              right: 0,
              child: AnimatedBuilder(
                animation: _logoAnimation,
                builder: (context, child) {
                  return Opacity(
                    opacity: _logoAnimation.value * 0.7,
                    child: Column(
                      children: [
                        Text(
                          'Version 1.0.0',
                          style: TextStyle(
                            color: Colors.white30,
                            fontSize: dimensions.versionSize,
                            letterSpacing: 1,
                          ),
                        ),
                        SizedBox(height: dimensions.spacing / 15),
                        Text(
                          'TWAIPL Robotics',
                          style: TextStyle(
                            color: Colors.white30,
                            fontSize: dimensions.companySize,
                            letterSpacing: 1,
                          ),
                        ),
                      ],
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

  Widget _buildBlinkingDots(SplashResponsiveDimensions dimensions) {
    return AnimatedBuilder(
      animation: _pulseController,
      builder: (context, child) {
        return Row(
          children: List.generate(3, (index) {
            final delay = index * 0.3;
            final opacity =
                ((_pulseController.value + delay) % 1.0) > 0.5 ? 1.0 : 0.3;
            return Container(
              margin: EdgeInsets.symmetric(horizontal: dimensions.spacing / 30),
              child: Opacity(
                opacity: opacity,
                child: Container(
                  width: dimensions.particleBaseSize + 1,
                  height: dimensions.particleBaseSize + 1,
                  decoration: const BoxDecoration(
                    color: Colors.redAccent,
                    shape: BoxShape.circle,
                  ),
                ),
              ),
            );
          }),
        );
      },
    );
  }

  Widget _buildBackgroundParticle(
      int index, SplashResponsiveDimensions dimensions) {
    final random = (index * 12345) % 1000;
    final screenSize = MediaQuery.of(context).size;
    final left = (random % 100) / 100 * screenSize.width;
    final top = ((random ~/ 100) % 100) / 100 * screenSize.height;
    final size =
        dimensions.particleBaseSize + (random % dimensions.particleVariance);

    return Positioned(
      left: left,
      top: top,
      child: AnimatedBuilder(
        animation: _pulseController,
        builder: (context, child) {
          final opacity =
              0.1 + ((_pulseController.value + (index * 0.1)) % 1.0) * 0.2;
          return Opacity(
            opacity: opacity,
            child: Container(
              width: size,
              height: size,
              decoration: BoxDecoration(
                color: Colors.redAccent.withOpacity(0.6),
                shape: BoxShape.circle,
                boxShadow: [
                  BoxShadow(
                    color: Colors.redAccent.withOpacity(0.3),
                    blurRadius: size,
                    spreadRadius: size * 0.25,
                  ),
                ],
              ),
            ),
          );
        },
      ),
    );
  }
}

// Helper classes for responsive design
enum DeviceType { mobile, tablet, desktop }

class SplashResponsiveDimensions {
  final double logoSize;
  final double logoPadding;
  final double titleSize;
  final double subtitleSize;
  final double spacing;
  final double progressWidth;
  final double progressHeight;
  final double versionSize;
  final double companySize;
  final int particleCount;
  final double particleBaseSize;
  final int particleVariance;
  final double glowBlur;
  final double glowSpread;

  SplashResponsiveDimensions({
    required this.logoSize,
    required this.logoPadding,
    required this.titleSize,
    required this.subtitleSize,
    required this.spacing,
    required this.progressWidth,
    required this.progressHeight,
    required this.versionSize,
    required this.companySize,
    required this.particleCount,
    required this.particleBaseSize,
    required this.particleVariance,
    required this.glowBlur,
    required this.glowSpread,
  });
}
