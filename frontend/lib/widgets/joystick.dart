//widgets/joystick.dart - Enhanced with Deadman Switch and Real-time Control
import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../services/web_socket_service.dart';
import '../services/api_service.dart';

class JoystickWidget extends StatefulWidget {
  final double size;
  final Function(double, double, bool) onChanged; // Added deadman parameter
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
  _JoystickWidgetState createState() => _JoystickWidgetState();
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
          Container(
            width: widget.size,
            height: widget.size,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: widget.enabled 
                  ? (_deadmanActive ? Colors.green[100] : Colors.grey[300]) 
                  : Colors.grey[200],
              border: Border.all(
                color: widget.enabled 
                    ? (_deadmanActive ? Colors.green[400]! : Colors.grey[400]!)
                    : Colors.grey[300]!,
                width: 3,
              ),
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.1),
                  blurRadius: 8,
                  offset: Offset(0, 2),
                ),
              ],
            ),
            child: Stack(
              children: [
                // Direction indicators
                _buildDirectionIndicators(),
                // Speed rings
                _buildSpeedRings(),
                // Deadman switch indicator
                if (widget.requireDeadman) _buildDeadmanIndicator(),
              ],
            ),
          ),
          
          // Knob
          AnimatedContainer(
            duration: _isDragging ? Duration.zero : Duration(milliseconds: 200),
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
                          ? Colors.green[600] 
                          : (_isDragging ? Colors.blue[600] : Colors.blue[500]))
                      : Colors.grey[400],
                  border: Border.all(
                    color: widget.enabled 
                        ? (_deadmanActive 
                            ? Colors.green[700]! 
                            : Colors.blue[700]!)
                        : Colors.grey[500]!,
                    width: 2,
                  ),
                  boxShadow: [
                    BoxShadow(
                      color: Colors.black.withOpacity(0.2),
                      blurRadius: 6,
                      offset: Offset(0, 2),
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
          ),
          
          // Center dot
          Container(
            width: 4,
            height: 4,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: Colors.grey[600],
            ),
          ),
          
          // Deadman switch instructions
          if (widget.requireDeadman && !_deadmanActive)
            Positioned(
              bottom: 10,
              child: Container(
                padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                decoration: BoxDecoration(
                  color: Colors.orange,
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Text(
                  'Hold to Enable',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 10,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildDirectionIndicators() {
    return Stack(
      children: [
        // Forward arrow
        Positioned(
          top: 10,
          left: 0,
          right: 0,
          child: Center(
            child: Icon(
              Icons.keyboard_arrow_up,
              color: _deadmanActive ? Colors.green[700] : Colors.grey[600],
              size: 24,
            ),
          ),
        ),
        // Backward arrow
        Positioned(
          bottom: 30, // Adjusted for deadman indicator
          left: 0,
          right: 0,
          child: Center(
            child: Icon(
              Icons.keyboard_arrow_down,
              color: _deadmanActive ? Colors.green[700] : Colors.grey[600],
              size: 24,
            ),
          ),
        ),
        // Left arrow
        Positioned(
          left: 10,
          top: 0,
          bottom: 0,
          child: Center(
            child: Icon(
              Icons.keyboard_arrow_left,
              color: _deadmanActive ? Colors.green[700] : Colors.grey[600],
              size: 24,
            ),
          ),
        ),
        // Right arrow
        Positioned(
          right: 10,
          top: 0,
          bottom: 0,
          child: Center(
            child: Icon(
              Icons.keyboard_arrow_right,
              color: _deadmanActive ? Colors.green[700] : Colors.grey[600],
              size: 24,
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildSpeedRings() {
    return Stack(
      children: [
        // 25% speed ring
        Center(
          child: Container(
            width: widget.size * 0.5,
            height: widget.size * 0.5,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              border: Border.all(
                color: _deadmanActive ? Colors.green[300]! : Colors.grey[300]!,
                width: 1,
              ),
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
              border: Border.all(
                color: _deadmanActive ? Colors.green[300]! : Colors.grey[300]!,
                width: 1,
              ),
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
    
    // Calculate the distance from center
    final distance = localPosition.distance;
    final maxDistance = widget.size / 2 - widget.size * 0.15; // Account for knob size
    
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
    
    // Calculate velocities
    final normalizedX = newPosition.dx / maxDistance;
    final normalizedY = -newPosition.dy / maxDistance; // Invert Y for forward/backward
    
    // Convert to linear and angular velocities
    final linear = normalizedY * widget.maxLinearSpeed;
    final angular = -normalizedX * widget.maxAngularSpeed;
    
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
  final Function(double, double, bool)? onVelocityChanged; // Updated signature

  const JoystickControlPage({
    Key? key,
    required this.deviceId,
    this.onVelocityChanged,
  }) : super(key: key);

  @override
  _JoystickControlPageState createState() => _JoystickControlPageState();
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

  @override
  void initState() {
    super.initState();
    _checkConnection();
  }

  void _checkConnection() {
    setState(() {
      _isConnected = _webSocketService.isConnected;
    });
    
    // Listen to connection state changes
    _webSocketService.connectionState.listen((connected) {
      setState(() {
        _isConnected = connected;
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: EdgeInsets.all(16),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // Connection status
          _buildConnectionStatus(),
          
          SizedBox(height: 16),
          
          // Speed controls
          _buildSpeedControls(),
          
          SizedBox(height: 20),
          
          // Joystick
          JoystickWidget(
            size: 250,
            maxLinearSpeed: _maxLinearSpeed,
            maxAngularSpeed: _maxAngularSpeed,
            enabled: _controlEnabled && _isConnected,
            requireDeadman: _deadmanRequired,
            onChanged: _onVelocityChanged,
          ),
          
          SizedBox(height: 20),
          
          // Velocity display
          _buildVelocityDisplay(),
          
          SizedBox(height: 20),
          
          // Control settings
          _buildControlSettings(),
          
          SizedBox(height: 20),
          
          // Control buttons
          _buildControlButtons(),
        ],
      ),
    );
  }

  Widget _buildConnectionStatus() {
    return Card(
      color: _isConnected ? Colors.green[50] : Colors.red[50],
      child: Padding(
        padding: EdgeInsets.all(12),
        child: Row(
          children: [
            Icon(
              _isConnected ? Icons.wifi : Icons.wifi_off,
              color: _isConnected ? Colors.green : Colors.red,
            ),
            SizedBox(width: 8),
            Text(
              _isConnected ? 'Connected' : 'Disconnected',
              style: TextStyle(
                fontWeight: FontWeight.bold,
                color: _isConnected ? Colors.green[700] : Colors.red[700],
              ),
            ),
            Spacer(),
            Text(
              _useWebSocket ? 'WebSocket' : 'HTTP API',
              style: TextStyle(fontSize: 12),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSpeedControls() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Speed Settings',
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: 16),
            
            // Linear speed control
            Row(
              children: [
                Icon(Icons.arrow_upward, size: 20),
                SizedBox(width: 8),
                Text('Linear Speed:'),
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
            ),
            
            // Angular speed control
            Row(
              children: [
                Icon(Icons.rotate_right, size: 20),
                SizedBox(width: 8),
                Text('Angular Speed:'),
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
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildVelocityDisplay() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          children: [
            Column(
              children: [
                Icon(
                  Icons.arrow_upward,
                  color: _currentLinear > 0 
                      ? Colors.green 
                      : _currentLinear < 0 
                          ? Colors.red 
                          : Colors.grey,
                ),
                Text('Linear'),
                Text(
                  '${_currentLinear.toStringAsFixed(2)} m/s',
                  style: TextStyle(
                    fontWeight: FontWeight.bold,
                    color: _currentLinear != 0 ? Colors.blue : Colors.grey,
                  ),
                ),
              ],
            ),
            Container(
              width: 1,
              height: 50,
              color: Colors.grey[300],
            ),
            Column(
              children: [
                Icon(
                  Icons.rotate_right,
                  color: _currentAngular > 0 
                      ? Colors.green 
                      : _currentAngular < 0 
                          ? Colors.red 
                          : Colors.grey,
                ),
                Text('Angular'),
                Text(
                  '${_currentAngular.toStringAsFixed(2)} rad/s',
                  style: TextStyle(
                    fontWeight: FontWeight.bold,
                    color: _currentAngular != 0 ? Colors.blue : Colors.grey,
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildControlSettings() {
    return Card(
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Control Settings',
              style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            SwitchListTile(
              title: Text('Require Deadman Switch'),
              subtitle: Text('Must hold joystick to enable movement'),
              value: _deadmanRequired,
              onChanged: (value) {
                setState(() {
                  _deadmanRequired = value;
                });
              },
            ),
            SwitchListTile(
              title: Text('Use WebSocket'),
              subtitle: Text('Real-time control via WebSocket'),
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
    return Row(
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
          icon: Icon(Icons.stop),
          label: Text('STOP'),
          style: ElevatedButton.styleFrom(
            backgroundColor: Colors.red,
            foregroundColor: Colors.white,
          ),
        ),
      ],
    );
  }

  void _onVelocityChanged(double linear, double angular, bool deadmanActive) {
    // Only update if deadman is active or not required
    if (!_deadmanRequired || deadmanActive) {
      setState(() {
        _currentLinear = linear;
        _currentAngular = angular;
      });
      
      _sendVelocityCommand(linear, angular, deadmanActive);
      
      // Call external callback if provided
      if (widget.onVelocityChanged != null) {
        widget.onVelocityChanged!(linear, angular, deadmanActive);
      }
    } else {
      // Stop if deadman not active
      setState(() {
        _currentLinear = 0.0;
        _currentAngular = 0.0;
      });
      
      _sendVelocityCommand(0.0, 0.0, false);
      
      if (widget.onVelocityChanged != null) {
        widget.onVelocityChanged!(0.0, 0.0, false);
      }
    }
  }

  void _sendVelocityCommand(double linear, double angular, bool deadmanActive) {
    if (!_isConnected || !_controlEnabled) return;

    try {
      if (_useWebSocket) {
        // Send via WebSocket for real-time control
        _webSocketService.sendControlCommand(widget.deviceId, 'move', {
          'linear': linear,
          'angular': angular,
          'deadman': deadmanActive,
        });
      } else {
        // Send via HTTP API (less real-time but more reliable)
        _apiService.joystickControl(
          deviceId: widget.deviceId,
          x: angular / _maxAngularSpeed, // Normalize for API
          y: linear / _maxLinearSpeed,   // Normalize for API
          deadman: deadmanActive,
        );
      }
    } catch (e) {
      print('❌ Error sending velocity command: $e');
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
      });
    }
  }

  void _emergencyStop() {
    _sendVelocityCommand(0.0, 0.0, false);
    setState(() {
      _currentLinear = 0.0;
      _currentAngular = 0.0;
    });
    
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('Emergency stop activated'),
        backgroundColor: Colors.red,
        duration: Duration(seconds: 2),
      ),
    );
  }
}