//services/multi_device_websocket_service.dart - Enhanced WebSocket for Multiple Devices
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/io.dart';

class MultiDeviceWebSocketService {
  static final MultiDeviceWebSocketService _instance = MultiDeviceWebSocketService._internal();
  factory MultiDeviceWebSocketService() => _instance;
  MultiDeviceWebSocketService._internal();

  // Map of device connections
  final Map<String, DeviceConnection> _connections = {};
  
  // Global stream controllers for aggregated data
  StreamController<Map<String, dynamic>>? _globalRealTimeController;
  StreamController<Map<String, dynamic>>? _globalDeviceEventsController;
  StreamController<Map<String, dynamic>>? _globalControlEventsController;
  StreamController<Map<String, bool>>? _connectionStatesController;

  // Public streams for aggregated data from all devices
  Stream<Map<String, dynamic>> get globalRealTimeData {
    _globalRealTimeController ??= StreamController<Map<String, dynamic>>.broadcast();
    return _globalRealTimeController!.stream;
  }

  Stream<Map<String, dynamic>> get globalDeviceEvents {
    _globalDeviceEventsController ??= StreamController<Map<String, dynamic>>.broadcast();
    return _globalDeviceEventsController!.stream;
  }

  Stream<Map<String, dynamic>> get globalControlEvents {
    _globalControlEventsController ??= StreamController<Map<String, dynamic>>.broadcast();
    return _globalControlEventsController!.stream;
  }

  Stream<Map<String, bool>> get connectionStates {
    _connectionStatesController ??= StreamController<Map<String, bool>>.broadcast();
    return _connectionStatesController!.stream;
  }

  // Connect to a specific device
  Future<bool> connectToDevice(String deviceId, String ipAddress, {int port = 3000}) async {
    final wsUrl = 'ws://$ipAddress:$port';
    print('🔌 Connecting to device $deviceId at $wsUrl');

    try {
      // Disconnect existing connection if any
      await disconnectDevice(deviceId);

      // Create new connection
      final connection = DeviceConnection(
        deviceId: deviceId,
        wsUrl: wsUrl,
        ipAddress: ipAddress,
        port: port,
      );

      // Set up connection listeners
      connection.realTimeData.listen((data) => _handleDeviceData(deviceId, data));
      connection.deviceEvents.listen((data) => _handleDeviceEvent(deviceId, data));
      connection.controlEvents.listen((data) => _handleControlEvent(deviceId, data));
      connection.connectionState.listen((connected) => _updateConnectionState(deviceId, connected));

      // Attempt connection
      final success = await connection.connect();
      
      if (success) {
        _connections[deviceId] = connection;
        print('✅ Connected to device $deviceId');
        _broadcastConnectionStates();
        return true;
      } else {
        print('❌ Failed to connect to device $deviceId');
        return false;
      }
    } catch (e) {
      print('❌ Error connecting to device $deviceId: $e');
      return false;
    }
  }

  // Connect to multiple devices
  Future<Map<String, bool>> connectToMultipleDevices(List<AGVDevice> devices) async {
    final results = <String, bool>{};
    
    print('🔌 Connecting to ${devices.length} devices...');
    
    // Connect to devices in parallel (with some delay to avoid overwhelming network)
    for (int i = 0; i < devices.length; i++) {
      final device = devices[i];
      
      // Add small delay between connections
      if (i > 0) {
        await Future.delayed(Duration(milliseconds: 500));
      }
      
      final success = await connectToDevice(device.id, device.ipAddress, port: device.port);
      results[device.id] = success;
    }
    
    final successCount = results.values.where((success) => success).length;
    print('✅ Connected to $successCount/${devices.length} devices');
    
    return results;
  }

  // Disconnect from a specific device
  Future<void> disconnectDevice(String deviceId) async {
    final connection = _connections[deviceId];
    if (connection != null) {
      await connection.disconnect();
      _connections.remove(deviceId);
      _broadcastConnectionStates();
      print('🔌 Disconnected from device $deviceId');
    }
  }

  // Disconnect from all devices
  Future<void> disconnectAll() async {
    print('🔌 Disconnecting from all devices...');
    final futures = _connections.values.map((conn) => conn.disconnect());
    await Future.wait(futures);
    _connections.clear();
    _broadcastConnectionStates();
    print('✅ Disconnected from all devices');
  }

  // Get connection for specific device
  DeviceConnection? getDeviceConnection(String deviceId) {
    return _connections[deviceId];
  }

  // Check if device is connected
  bool isDeviceConnected(String deviceId) {
    return _connections[deviceId]?.isConnected ?? false;
  }

  // Get all connected device IDs
  List<String> getConnectedDeviceIds() {
    return _connections.entries
        .where((entry) => entry.value.isConnected)
        .map((entry) => entry.key)
        .toList();
  }

  // Send message to specific device
  void sendToDevice(String deviceId, Map<String, dynamic> message) {
    final connection = _connections[deviceId];
    if (connection != null && connection.isConnected) {
      connection.sendMessage(message);
    } else {
      print('⚠️ Device $deviceId not connected, cannot send message');
    }
  }

  // Send message to all connected devices
  void sendToAllDevices(Map<String, dynamic> message) {
    final connectedDevices = _connections.entries
        .where((entry) => entry.value.isConnected)
        .toList();
    
    for (final entry in connectedDevices) {
      entry.value.sendMessage(message);
    }
    
    print('📡 Sent message to ${connectedDevices.length} devices');
  }

  // Control commands for specific device
  void sendControlCommand(String deviceId, String command, Map<String, dynamic>? data) {
    sendToDevice(deviceId, {
      'type': 'control_command',
      'deviceId': deviceId,
      'command': command,
      'data': data ?? {},
    });
  }

  // Emergency stop all devices
  void emergencyStopAll() {
    print('🚨 EMERGENCY STOP ALL DEVICES');
    sendToAllDevices({
      'type': 'control_command',
      'command': 'emergency_stop',
      'data': {},
    });
  }

  // Event handlers
  void _handleDeviceData(String deviceId, Map<String, dynamic> data) {
    // Add device ID to data and forward to global stream
    final enrichedData = {
      ...data,
      'sourceDeviceId': deviceId,
      'timestamp': DateTime.now().toIso8601String(),
    };
    
    _globalRealTimeController?.add(enrichedData);
  }

  void _handleDeviceEvent(String deviceId, Map<String, dynamic> data) {
    final enrichedData = {
      ...data,
      'sourceDeviceId': deviceId,
      'timestamp': DateTime.now().toIso8601String(),
    };
    
    _globalDeviceEventsController?.add(enrichedData);
  }

  void _handleControlEvent(String deviceId, Map<String, dynamic> data) {
    final enrichedData = {
      ...data,
      'sourceDeviceId': deviceId,
      'timestamp': DateTime.now().toIso8601String(),
    };
    
    _globalControlEventsController?.add(enrichedData);
  }

  void _updateConnectionState(String deviceId, bool connected) {
    _broadcastConnectionStates();
    
    if (connected) {
      print('✅ Device $deviceId connected');
    } else {
      print('❌ Device $deviceId disconnected');
    }
  }

  void _broadcastConnectionStates() {
    final states = <String, bool>{};
    for (final entry in _connections.entries) {
      states[entry.key] = entry.value.isConnected;
    }
    _connectionStatesController?.add(states);
  }

  // Get connection statistics
  Map<String, dynamic> getConnectionStats() {
    final totalDevices = _connections.length;
    final connectedDevices = _connections.values.where((conn) => conn.isConnected).length;
    final disconnectedDevices = totalDevices - connectedDevices;
    
    return {
      'totalDevices': totalDevices,
      'connectedDevices': connectedDevices,
      'disconnectedDevices': disconnectedDevices,
      'connectionRate': totalDevices > 0 ? (connectedDevices / totalDevices) : 0.0,
      'deviceStates': _connections.map((key, conn) => MapEntry(key, {
        'deviceId': key,
        'connected': conn.isConnected,
        'url': conn.wsUrl,
        'lastHeartbeat': conn.lastHeartbeat?.toIso8601String(),
        'reconnectAttempts': conn.reconnectAttempts,
      })),
    };
  }

  void dispose() {
    disconnectAll();
    _globalRealTimeController?.close();
    _globalDeviceEventsController?.close();
    _globalControlEventsController?.close();
    _connectionStatesController?.close();
  }
}

// Individual device connection class
class DeviceConnection {
  final String deviceId;
  final String wsUrl;
  final String ipAddress;
  final int port;

  WebSocketChannel? _channel;
  bool _isConnected = false;
  Timer? _heartbeatTimer;
  Timer? _reconnectTimer;
  int _reconnectAttempts = 0;
  DateTime? _lastHeartbeat;
  
  static const int maxReconnectAttempts = 3;

  // Stream controllers for this device
  StreamController<Map<String, dynamic>>? _realTimeController;
  StreamController<Map<String, dynamic>>? _deviceEventsController;
  StreamController<Map<String, dynamic>>? _controlEventsController;
  StreamController<bool>? _connectionStateController;

  DeviceConnection({
    required this.deviceId,
    required this.wsUrl,
    required this.ipAddress,
    required this.port,
  }) {
    _initializeControllers();
  }

  void _initializeControllers() {
    _realTimeController = StreamController<Map<String, dynamic>>.broadcast();
    _deviceEventsController = StreamController<Map<String, dynamic>>.broadcast();
    _controlEventsController = StreamController<Map<String, dynamic>>.broadcast();
    _connectionStateController = StreamController<bool>.broadcast();
  }

  // Public streams
  Stream<Map<String, dynamic>> get realTimeData => _realTimeController!.stream;
  Stream<Map<String, dynamic>> get deviceEvents => _deviceEventsController!.stream;
  Stream<Map<String, dynamic>> get controlEvents => _controlEventsController!.stream;
  Stream<bool> get connectionState => _connectionStateController!.stream;

  bool get isConnected => _isConnected;
  DateTime? get lastHeartbeat => _lastHeartbeat;
  int get reconnectAttempts => _reconnectAttempts;

  Future<bool> connect() async {
    try {
      print('🔌 Connecting to $deviceId at $wsUrl');

      _channel = IOWebSocketChannel.connect(
        Uri.parse(wsUrl),
        protocols: ['websocket'],
        connectTimeout: Duration(seconds: 10),
      );

      // Set up message listener
      _channel!.stream.listen(
        _handleMessage,
        onError: _handleError,
        onDone: _handleDisconnection,
      );

      // Wait for connection confirmation
      final connected = await _waitForConnection();
      
      if (connected) {
        _isConnected = true;
        _reconnectAttempts = 0;
        _startHeartbeat();
        _connectionStateController!.add(true);
        return true;
      } else {
        await disconnect();
        return false;
      }
    } catch (e) {
      print('❌ Error connecting to $deviceId: $e');
      await disconnect();
      return false;
    }
  }

  Future<bool> _waitForConnection() async {
    final completer = Completer<bool>();
    late StreamSubscription subscription;
    
    subscription = _channel!.stream.listen((data) {
      try {
        final message = json.decode(data);
        if (message['type'] == 'connection') {
          subscription.cancel();
          completer.complete(true);
        }
      } catch (e) {
        // Ignore parsing errors during connection
      }
    }, onError: (error) {
      subscription.cancel();
      completer.complete(false);
    });

    // Timeout after 10 seconds
    return await Future.any([
      completer.future,
      Future.delayed(Duration(seconds: 10), () => false),
    ]);
  }

  void _handleMessage(dynamic data) {
    try {
      final message = json.decode(data);
      final messageType = message['type'];
      
      switch (messageType) {
        case 'connection':
          _isConnected = true;
          _connectionStateController!.add(true);
          print('✅ Device $deviceId connection established');
          break;
          
        case 'heartbeat':
          _lastHeartbeat = DateTime.now();
          _sendHeartbeatAck();
          break;
          
        case 'broadcast':
          _routeMessage(message['topic'], message['data'] ?? {});
          break;
          
        case 'real_time_data':
        case 'odometry_update':
        case 'battery_update':
        case 'laser_scan':
          _realTimeController!.add(message);
          break;
          
        case 'device_event':
        case 'status_update':
        case 'alert':
          _deviceEventsController!.add(message);
          break;
          
        case 'control_event':
        case 'movement_command':
        case 'stop_command':
          _controlEventsController!.add(message);
          break;
          
        default:
          print('❓ Unknown message type from $deviceId: $messageType');
      }
    } catch (e) {
      print('❌ Error handling message from $deviceId: $e');
    }
  }

  void _routeMessage(String topic, Map<String, dynamic> data) {
    switch (topic) {
      case 'real_time_data':
        _realTimeController!.add(data);
        break;
      case 'device_events':
        _deviceEventsController!.add(data);
        break;
      case 'control_events':
        _controlEventsController!.add(data);
        break;
    }
  }

  void _handleError(error) {
    print('❌ WebSocket error for $deviceId: $error');
    _handleDisconnection();
  }

  void _handleDisconnection() {
    _isConnected = false;
    _connectionStateController!.add(false);
    _stopHeartbeat();
    
    // Attempt reconnection if within limits
    if (_reconnectAttempts < maxReconnectAttempts) {
      _startReconnectTimer();
    } else {
      print('❌ Max reconnection attempts reached for $deviceId');
    }
  }

  void _startHeartbeat() {
    _heartbeatTimer = Timer.periodic(Duration(seconds: 30), (timer) {
      if (_isConnected) {
        sendMessage({
          'type': 'heartbeat',
          'deviceId': deviceId,
          'timestamp': DateTime.now().toIso8601String(),
        });
      }
    });
  }

  void _stopHeartbeat() {
    _heartbeatTimer?.cancel();
    _heartbeatTimer = null;
  }

  void _sendHeartbeatAck() {
    sendMessage({
      'type': 'heartbeat_ack',
      'deviceId': deviceId,
      'timestamp': DateTime.now().toIso8601String(),
    });
  }

  void _startReconnectTimer() {
    _reconnectTimer = Timer(Duration(seconds: 5 + _reconnectAttempts * 2), () async {
      _reconnectAttempts++;
      print('🔄 Reconnecting to $deviceId (attempt $_reconnectAttempts/$maxReconnectAttempts)');
      
      final success = await connect();
      if (!success) {
        _handleDisconnection();
      }
    });
  }

  void sendMessage(Map<String, dynamic> message) {
    if (_isConnected && _channel != null) {
      try {
        _channel!.sink.add(json.encode(message));
      } catch (e) {
        print('❌ Error sending message to $deviceId: $e');
      }
    } else {
      print('⚠️ Device $deviceId not connected, cannot send message');
    }
  }

  Future<void> disconnect() async {
    try {
      _stopHeartbeat();
      _reconnectTimer?.cancel();
      
      if (_channel != null) {
        await _channel!.sink.close();
        _channel = null;
      }
      
      _isConnected = false;
      _connectionStateController!.add(false);
      
    } catch (e) {
      print('❌ Error disconnecting from $deviceId: $e');
    }
  }

  void dispose() {
    disconnect();
    _realTimeController?.close();
    _deviceEventsController?.close();
    _controlEventsController?.close();
    _connectionStateController?.close();
  }
}

// For backward compatibility, keep the AGVDevice class definition
class AGVDevice {
  final String id;
  final String name;
  final String ipAddress;
  final int port;
  final String discoveryMethod;
  final List<String> services;
  final String? apiEndpoint;
  final Map<String, dynamic>? metadata;

  AGVDevice({
    required this.id,
    required this.name,
    required this.ipAddress,
    required this.port,
    required this.discoveryMethod,
    required this.services,
    this.apiEndpoint,
    this.metadata,
  });

  String get webSocketUrl => 'ws://$ipAddress:$port';
  String get httpBaseUrl => 'http://$ipAddress:$port';
}