// widgets/joystick.dart - Enhanced with Deadman Switch and Real-time Control
import 'package:flutter/material.dart';
import 'dart:math' as math;
import 'dart:async';
import '../services/web_socket_service.dart';
import '../services/api_service.dart';

class JoystickWidget extends StatefulWidget {
  final double size;
  final Function(double, double, bool) onChanged;
  final double maxLinearSpeed;
  final double maxAngularSpeed;
  final bool enabled;
  final bool requireDeadman;

  const JoystickWidget({
    Key? key,
    this.size = 200.0,
    required this.onChanged,
    this.maxLinearSpeed = 1.0,
    this.maxAngularSpeed = 2.0,
    this.enabled = true,
    this.requireDeadman = true,
  }) : super(key: key);

  @override
  State<JoystickWidget> createState() => _JoystickWidgetState();
}

class _JoystickWidgetState extends State<JoystickWidget> {
  Offset _knobPosition = Offset.zero;
  bool _isDragging = false;
  bool _deadmanActive = false;

  @override
  Widget build(BuildContext context) {
    return Container(
      width: widget.size,
      height: widget.size,
      child: Stack(
        alignment: Alignment.center,
        children: [
          // Outer circle (base)
          _buildBaseCircle(),
          
          // Knob
          _buildKnob(),
          
          // Center dot
          _buildCenterDot(),
          
          // Deadman switch instructions
          if (widget.requireDeadman && !_deadmanActive)
            _buildDeadmanInstructions(),
        ],
      ),
    );
  }

  Widget _buildBaseCircle() {
    return Container(
      width: widget.size,
      height: widget.size,
      decoration: BoxDecoration(
        shape: BoxShape.circle,
        color: widget.enabled
            ? (_deadmanActive ? Colors.green.shade100 : Colors.grey.shade300)
            : Colors.grey.shade200,
        border: Border.all(
          color: widget.enabled
              ? (_deadmanActive ? Colors.green.shade400 : Colors.grey.shade400)
              : Colors.grey.shade300,
          width: 3,
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 8,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: Stack(
        children: [
          _buildDirectionIndicators(),
          _buildSpeedRings(),
          if (widget.requireDeadman) _buildDeadmanIndicator(),
        ],
      ),
    );
  }

  Widget _buildKnob() {
    return AnimatedContainer(
      duration: _isDragging ? Duration.zero : const Duration(milliseconds: 200),
      transform: Matrix4.translationValues(
        _knobPosition.dx,
        _knobPosition.dy,
        0,
      ),
      child: GestureDetector(
        onPanStart: widget.enabled ? _onPanStart : null,
        onPanUpdate: widget.enabled ? _onPanUpdate : null,
        onPanEnd: widget.enabled ? _onPanEnd : null,
        child: Container(
          width: widget.size * 0.3,
          height: widget.size * 0.3,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            color: widget.enabled
                ? (_deadmanActive
                    ? Colors.green.shade600
                    : (_isDragging ? Colors.blue.shade600 : Colors.blue.shade500))
                : Colors.grey.shade400,
            border: Border.all(
              color: widget.enabled
                  ? (_deadmanActive ? Colors.green.shade700 : Colors.blue.shade700)
                  : Colors.grey.shade500,
              width: 2,
            ),
            boxShadow: [
              BoxShadow(
                color: Colors.black.withOpacity(0.2),
                blurRadius: 6,
                offset: const Offset(0, 2),
              ),
            ],
          ),
          child: Icon(
            _deadmanActive ? Icons.lock_open : Icons.control_camera,
            color: Colors.white,
            size: widget.size * 0.08,
          ),
        ),
      ),
    );
  }

  Widget _buildCenterDot() {
    return Container(
      width: 4,
      height: 4,
      decoration: BoxDecoration(
        shape: BoxShape.circle,
        color: Colors.grey.shade600,
      ),
    );
  }

  Widget _buildDeadmanInstructions() {
    return Positioned(
      bottom: 10,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
        decoration: BoxDecoration(
          color: Colors.orange,
          borderRadius: BorderRadius.circular(12),
        ),
        child: const Text(
          'Hold to Enable',
          style: TextStyle(
            color: Colors.white,
            fontSize: 10,
            fontWeight: FontWeight.bold,
          ),
        ),
      ),
    );
  }

  Widget _buildDirectionIndicators() {
    final iconColor = _deadmanActive ? Colors.green.shade700 : Colors.grey.shade600;
    
    return Stack(
      children: [
        // Forward arrow
        Positioned(
          top: 10,
          left: 0,
          right: 0,
          child: Center(
            child: Icon(Icons.keyboard_arrow_up, color: iconColor, size: 24),
          ),
        ),
        // Backward arrow
        Positioned(
          bottom: 30,
          left: 0,
          right: 0,
          child: Center(
            child: Icon(Icons.keyboard_arrow_down, color: iconColor, size: 24),
          ),
        ),
        // Left arrow
        Positioned(
          left: 10,
          top: 0,
          bottom: 0,
          child: Center(
            child: Icon(Icons.keyboard_arrow_left, color: iconColor, size: 24),
          ),
        ),
        // Right arrow
        Positioned(
          right: 10,
          top: 0,
          bottom: 0,
          child: Center(
            child: Icon(Icons.keyboard_arrow_right, color: iconColor, size: 24),
          ),
        ),
      ],
    );
  }

  Widget _buildSpeedRings() {
    final ringColor = _deadmanActive ? Colors.green.shade300 : Colors.grey.shade300;
    
    return Stack(
      children: [
        // 25% speed ring
        Center(
          child: Container(
            width: widget.size * 0.5,
            height: widget.size * 0.5,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              border: Border.all(color: ringColor, width: 1),
            ),
          ),
        ),
        // 75% speed ring
        Center(
          child: Container(
            width: widget.size * 0.75,
            height: widget.size * 0.75,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              border: Border.all(color: ringColor, width: 1),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildDeadmanIndicator() {
    return Positioned(
      bottom: 8,
      left: 0,
      right: 0,
      child: Center(
        child: Container(
          width: 12,
          height: 12,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            color: _deadmanActive ? Colors.green : Colors.red,
            border: Border.all(color: Colors.white, width: 2),
          ),
        ),
      ),
    );
  }

  void _onPanStart(DragStartDetails details) {
    setState(() {
      _isDragging = true;
      if (widget.requireDeadman) {
        _deadmanActive = true;
      }
    });
  }

  void _onPanUpdate(DragUpdateDetails details) {
    final center = Offset(widget.size / 2, widget.size / 2);
    final localPosition = details.localPosition - center;
    final distance = localPosition.distance;
    final maxDistance = widget.size / 2 - widget.size * 0.15;

    // Limit the knob position to the circular boundary
    Offset newPosition;
    if (distance <= maxDistance) {
      newPosition = localPosition;
    } else {
      final angle = math.atan2(localPosition.dy, localPosition.dx);
      newPosition = Offset(
        math.cos(angle) * maxDistance,
        math.sin(angle) * maxDistance,
      );
    }

    setState(() {
      _knobPosition = newPosition;
    });

    // Calculate normalized values (-1.0 to 1.0)
    final normalizedX = (newPosition.dx / maxDistance).clamp(-1.0, 1.0);
    final normalizedY = (-newPosition.dy / maxDistance).clamp(-1.0, 1.0); // Invert Y for forward/backward

    // Convert to linear and angular velocities with strict limits
    final linear = (normalizedY * widget.maxLinearSpeed).clamp(-widget.maxLinearSpeed, widget.maxLinearSpeed);
    final angular = (normalizedX * widget.maxAngularSpeed).clamp(-widget.maxAngularSpeed, widget.maxAngularSpeed);

    // Only send commands if deadman is active (or not required)
    final deadmanOk = !widget.requireDeadman || _deadmanActive;
    widget.onChanged(linear, angular, deadmanOk);
  }

  void _onPanEnd(DragEndDetails details) {
    setState(() {
      _knobPosition = Offset.zero;
      _isDragging = false;
      _deadmanActive = false;
    });

    // Stop the robot
    widget.onChanged(0.0, 0.0, false);
  }
}

class JoystickControlPage extends StatefulWidget {
  final String deviceId;
  final Function(double, double, bool)? onVelocityChanged;

  const JoystickControlPage({
    Key? key,
    required this.deviceId,
    this.onVelocityChanged,
  }) : super(key: key);

  @override
  State<JoystickControlPage> createState() => _JoystickControlPageState();
}

class _JoystickControlPageState extends State<JoystickControlPage> {
  final WebSocketService _webSocketService = WebSocketService();
  final ApiService _apiService = ApiService();

  double _currentLinear = 0.0;
  double _currentAngular = 0.0;
  double _maxLinearSpeed = 1.0;
  double _maxAngularSpeed = 2.0;
  bool _controlEnabled = true;
  bool _deadmanRequired = true;
  bool _useWebSocket = true;
  bool _isConnected = false;

  Timer? _accelerationTimer;
  double _targetLinear = 0.0;
  double _targetAngular = 0.0;
  static const double _accelerationStep = 0.05;
  static const Duration _accelerationInterval = Duration(milliseconds: 30);

  StreamSubscription<bool>? _connectionSubscription;
  StreamSubscription<Map<String, dynamic>>? _mappingSubscription;
  StreamSubscription<Map<String, dynamic>>? _controlSubscription;
  StreamSubscription<String>? _errorSubscription;

  @override
  void initState() {
    super.initState();
    _initializeConnection();
  }

  void _initializeConnection() {
    setState(() {
      _isConnected = _webSocketService.isConnected;
    });

    // Listen to connection state changes
    _connectionSubscription = _webSocketService.connectionState.listen((connected) {
      if (mounted) {
        setState(() {
          _isConnected = connected;
        });
      }
    });

    // ✅ NEW: Listen for mapping results and joystick results
    _mappingSubscription = _webSocketService.mappingEvents.listen((event) {
      if (mounted && event['type'] == 'mapping_result') {
        final success = event['result']?['success'] ?? false;
        final command = event['command'] ?? 'unknown';
        
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('Mapping $command: ${success ? 'Success' : 'Failed'}'),
            backgroundColor: success ? Colors.green : Colors.red,
            duration: const Duration(seconds: 2),
          ),
        );
      }
    });

    // Listen for control command results
    _controlSubscription = _webSocketService.controlEvents.listen((event) {
      if (mounted && event['type'] == 'joystick_result') {
        final success = event['result']?['success'] ?? false;
        if (!success) {
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(
              content: Text('Joystick command failed'),
              backgroundColor: Colors.orange,
              duration: Duration(seconds: 1),
            ),
          );
        }
      }
    });

    // Listen for WebSocket errors
    _errorSubscription = _webSocketService.errors.listen((error) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('WebSocket Error: $error'),
            backgroundColor: Colors.red,
            duration: const Duration(seconds: 3),
          ),
        );
      }
    });
  }

  @override
  void dispose() {
    _accelerationTimer?.cancel();
    _connectionSubscription?.cancel();
    _mappingSubscription?.cancel();
    _controlSubscription?.cancel();
    _errorSubscription?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(16),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          _buildConnectionStatus(),
          const SizedBox(height: 16),
          _buildSpeedControls(),
          const SizedBox(height: 20),
          _buildJoystick(),
          const SizedBox(height: 20),
          _buildVelocityDisplay(),
          const SizedBox(height: 20),
          _buildControlSettings(),
          const SizedBox(height: 16),
          _buildControlButtons(),
        ],
      ),
    );
  }

  Widget _buildConnectionStatus() {
    return Card(
      color: _isConnected ? Colors.green.shade50 : Colors.red.shade50,
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Row(
          children: [
            Icon(
              _isConnected ? Icons.wifi : Icons.wifi_off,
              color: _isConnected ? Colors.green : Colors.red,
            ),
            const SizedBox(width: 8),
            Text(
              _isConnected ? 'Connected' : 'Disconnected',
              style: TextStyle(
                fontWeight: FontWeight.bold,
                color: _isConnected ? Colors.green.shade700 : Colors.red.shade700,
              ),
            ),
            const Spacer(),
            Text(
              _useWebSocket ? 'WebSocket' : 'HTTP API',
              style: const TextStyle(fontSize: 12),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSpeedControls() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Speed Settings',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 16),
            _buildLinearSpeedSlider(),
            _buildAngularSpeedSlider(),
            const SizedBox(height: 8),
            // Speed limit indicator
            Container(
              padding: const EdgeInsets.all(8),
              decoration: BoxDecoration(
                color: Colors.blue.shade50,
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.blue.shade200),
              ),
              child: Row(
                children: [
                  Icon(Icons.info, size: 16, color: Colors.blue.shade600),
                  const SizedBox(width: 8),
                  Expanded(
                    child: Text(
                      'Joystick will be limited to these maximum speeds',
                      style: TextStyle(
                        fontSize: 12,
                        color: Colors.blue.shade700,
                        fontWeight: FontWeight.w500,
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

  Widget _buildLinearSpeedSlider() {
    return Row(
      children: [
        const Icon(Icons.arrow_upward, size: 20),
        const SizedBox(width: 8),
        const Text('Linear Speed:'),
        Expanded(
          child: Slider(
            value: _maxLinearSpeed,
            min: 0.1,
            max: 2.0,
            divisions: 19,
            label: '${_maxLinearSpeed.toStringAsFixed(1)} m/s',
            onChanged: (value) {
              setState(() {
                _maxLinearSpeed = value;
              });
            },
          ),
        ),
        Text('${_maxLinearSpeed.toStringAsFixed(1)} m/s'),
      ],
    );
  }

  Widget _buildAngularSpeedSlider() {
    return Row(
      children: [
        const Icon(Icons.rotate_right, size: 20),
        const SizedBox(width: 8),
        const Text('Angular Speed:'),
        Expanded(
          child: Slider(
            value: _maxAngularSpeed,
            min: 0.1,
            max: 3.0,
            divisions: 29,
            label: '${_maxAngularSpeed.toStringAsFixed(1)} rad/s',
            onChanged: (value) {
              setState(() {
                _maxAngularSpeed = value;
              });
            },
          ),
        ),
        Text('${_maxAngularSpeed.toStringAsFixed(1)} rad/s'),
      ],
    );
  }

  Widget _buildJoystick() {
    return JoystickWidget(
      size: 250,
      maxLinearSpeed: _maxLinearSpeed,
      maxAngularSpeed: _maxAngularSpeed,
      enabled: _controlEnabled && _isConnected,
      requireDeadman: _deadmanRequired,
      onChanged: _onVelocityChanged,
    );
  }

  Widget _buildVelocityDisplay() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          children: [
            _buildVelocityItem(
              icon: Icons.arrow_upward,
              label: 'Linear',
              value: _currentLinear,
              maxValue: _maxLinearSpeed,
              unit: 'm/s',
            ),
            Container(
              width: 1,
              height: 50,
              color: Colors.grey.shade300,
            ),
            _buildVelocityItem(
              icon: Icons.rotate_right,
              label: 'Angular',
              value: _currentAngular,
              maxValue: _maxAngularSpeed,
              unit: 'rad/s',
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildVelocityItem({
    required IconData icon,
    required String label,
    required double value,
    required double maxValue,
    required String unit,
  }) {
    Color iconColor;
    if (value > 0) {
      iconColor = Colors.green;
    } else if (value < 0) {
      iconColor = Colors.red;
    } else {
      iconColor = Colors.grey;
    }

    // Calculate percentage of max speed
    final percentage = maxValue > 0 ? (value.abs() / maxValue * 100) : 0.0;

    return Column(
      children: [
        Icon(icon, color: iconColor),
        Text(label),
        Text(
          '${value.toStringAsFixed(2)} $unit',
          style: TextStyle(
            fontWeight: FontWeight.bold,
            color: value != 0 ? Colors.blue : Colors.grey,
          ),
        ),
        Text(
          '${percentage.toStringAsFixed(0)}% of max',
          style: TextStyle(
            fontSize: 10,
            color: Colors.grey.shade600,
          ),
        ),
      ],
    );
  }

  Widget _buildControlSettings() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Control Settings',
              style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 8),
            SwitchListTile(
              title: const Text('Require Deadman Switch'),
              subtitle: const Text('Must hold joystick to enable movement'),
              value: _deadmanRequired,
              onChanged: (value) {
                setState(() {
                  _deadmanRequired = value;
                });
              },
            ),
            SwitchListTile(
              title: const Text('Use WebSocket'),
              subtitle: const Text('Real-time control via WebSocket'),
              value: _useWebSocket,
              onChanged: (value) {
                setState(() {
                  _useWebSocket = value;
                });
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildControlButtons() {
    return Column(
      children: [
        // Main control buttons
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            ElevatedButton.icon(
              onPressed: _controlEnabled ? _toggleControl : null,
              icon: Icon(_controlEnabled ? Icons.pause : Icons.play_arrow),
              label: Text(_controlEnabled ? 'Disable' : 'Enable'),
              style: ElevatedButton.styleFrom(
                backgroundColor: _controlEnabled ? Colors.orange : Colors.green,
              ),
            ),
            ElevatedButton.icon(
              onPressed: _emergencyStop,
              icon: const Icon(Icons.stop),
              label: const Text('STOP'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.red,
                foregroundColor: Colors.white,
              ),
            ),
          ],
        ),
        
        const SizedBox(height: 12),
        
        // ✅ NEW: Mapping control buttons for testing
        if (_isConnected) ...[
          const Text(
            'Mapping Controls',
            style: TextStyle(fontSize: 14, fontWeight: FontWeight.bold),
          ),
          const SizedBox(height: 8),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              ElevatedButton.icon(
                onPressed: () => _sendMappingCommand('start'),
                icon: const Icon(Icons.play_arrow, size: 16),
                label: const Text('Start Map'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.green.shade600,
                  foregroundColor: Colors.white,
                  minimumSize: const Size(100, 36),
                ),
              ),
              ElevatedButton.icon(
                onPressed: () => _sendMappingCommand('stop'),
                icon: const Icon(Icons.stop, size: 16),
                label: const Text('Stop Map'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.red.shade600,
                  foregroundColor: Colors.white,
                  minimumSize: const Size(100, 36),
                ),
              ),
              ElevatedButton.icon(
                onPressed: () => _sendMappingCommand('save'),
                icon: const Icon(Icons.save, size: 16),
                label: const Text('Save Map'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.blue.shade600,
                  foregroundColor: Colors.white,
                  minimumSize: const Size(100, 36),
                ),
              ),
            ],
          ),
        ],
      ],
    );
  }

  void _onVelocityChanged(double linear, double angular, bool deadmanActive) {
    // Ensure target values don't exceed max speeds
    _targetLinear = linear.clamp(-_maxLinearSpeed, _maxLinearSpeed);
    _targetAngular = angular.clamp(-_maxAngularSpeed, _maxAngularSpeed);

    // Start smooth acceleration towards target
    _startAcceleration(deadmanActive);

    // Call external callback if provided
    widget.onVelocityChanged?.call(_currentLinear, _currentAngular, deadmanActive);
  }

  void _startAcceleration(bool deadmanActive) {
    _accelerationTimer?.cancel();
    
    _accelerationTimer = Timer.periodic(_accelerationInterval, (timer) {
      bool updated = false;

      // Smooth linear velocity transition with speed limit enforcement
      if ((_currentLinear - _targetLinear).abs() > _accelerationStep) {
        _currentLinear += _accelerationStep * (_targetLinear - _currentLinear).sign;
        // Ensure we don't exceed max speed during acceleration
        _currentLinear = _currentLinear.clamp(-_maxLinearSpeed, _maxLinearSpeed);
        updated = true;
      } else if (_currentLinear != _targetLinear) {
        _currentLinear = _targetLinear;
        updated = true;
      }

      // Smooth angular velocity transition with speed limit enforcement
      if ((_currentAngular - _targetAngular).abs() > _accelerationStep) {
        _currentAngular += _accelerationStep * (_targetAngular - _currentAngular).sign;
        // Ensure we don't exceed max speed during acceleration
        _currentAngular = _currentAngular.clamp(-_maxAngularSpeed, _maxAngularSpeed);
        updated = true;
      } else if (_currentAngular != _targetAngular) {
        _currentAngular = _targetAngular;
        updated = true;
      }

      if (updated) {
        setState(() {});
        _sendVelocityCommand(_currentLinear, _currentAngular, deadmanActive);
      }

      // Stop timer when both velocities reach their targets
      if (_currentLinear == _targetLinear && _currentAngular == _targetAngular) {
        timer.cancel();
      }
    });
  }

  Future<void> _sendVelocityCommand(double linear, double angular, bool deadmanActive) async {
    if (!_isConnected || !_controlEnabled) return;

    try {
      if (_useWebSocket) {
        // ✅ FIXED: Use the correct WebSocket method for joystick control with max speeds
        _webSocketService.sendJoystickControl(
          widget.deviceId,
          angular / _maxAngularSpeed, // Normalize for WebSocket
          linear / _maxLinearSpeed,   // Normalize for WebSocket
          deadmanActive,
          maxLinearSpeed: _maxLinearSpeed,   // ✅ NEW: Pass actual max speeds
          maxAngularSpeed: _maxAngularSpeed, // ✅ NEW: Pass actual max speeds
        );
      } else {
        // Send via HTTP API (less real-time but more reliable)
        await _apiService.joystickControl(
          deviceId: widget.deviceId,
          x: angular / _maxAngularSpeed, // Normalize for API
          y: linear / _maxLinearSpeed,   // Normalize for API
          deadman: deadmanActive,
        );
      }
    } catch (e) {
      debugPrint('❌ Error sending velocity command: $e');
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('Failed to send command: ${e.toString()}'),
            backgroundColor: Colors.red,
            duration: const Duration(seconds: 2),
          ),
        );
      }
    }
  }

  void _toggleControl() {
    setState(() {
      _controlEnabled = !_controlEnabled;
    });

    if (!_controlEnabled) {
      // Stop the robot when disabling control
      _sendVelocityCommand(0.0, 0.0, false);
      setState(() {
        _currentLinear = 0.0;
        _currentAngular = 0.0;
        _targetLinear = 0.0;
        _targetAngular = 0.0;
      });
    }
  }

  void _emergencyStop() {
    _targetLinear = 0.0;
    _targetAngular = 0.0;
    _currentLinear = 0.0;
    _currentAngular = 0.0;
    
    _accelerationTimer?.cancel();
    
    // Immediately send stop command
    _sendVelocityCommand(0.0, 0.0, false);
    
    setState(() {});

    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('Emergency stop activated'),
          backgroundColor: Colors.red,
          duration: Duration(seconds: 2),
        ),
      );
    }
  }

  // ✅ NEW: Method to send mapping commands for testing
  void _sendMappingCommand(String command) {
    if (!_isConnected) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('Not connected to WebSocket'),
          backgroundColor: Colors.orange,
        ),
      );
      return;
    }

    try {
      _webSocketService.sendMappingCommand(widget.deviceId, command);
      
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Mapping command sent: $command'),
          backgroundColor: Colors.blue,
          duration: const Duration(seconds: 1),
        ),
      );
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Failed to send mapping command: $e'),
          backgroundColor: Colors.red,
        ),
      );
    }
  }
}