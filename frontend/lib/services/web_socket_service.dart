//services/websocket_service.dart - Restructured for Better Organization
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/io.dart';

class WebSocketService {
  // ==========================================
  // SINGLETON SETUP & CONSTANTS
  // ==========================================
  static final WebSocketService _instance = WebSocketService._internal();
  factory WebSocketService() => _instance;
  WebSocketService._internal();

  static const int maxReconnectAttempts = 10;
  static const Duration pingInterval = Duration(seconds: 5);
  static const Duration connectionTimeout = Duration(seconds: 15);

  // ==========================================
  // FIELDS & PROPERTIES
  // ==========================================
  WebSocketChannel? _channel;
  String? _clientId;
  String? _serverUrl;
  
  // Connection state
  bool _isConnected = false;
  bool _isReconnecting = false;
  int _reconnectAttempts = 0;
  DateTime? _lastPongReceived;

  // Timers
  Timer? _heartbeatTimer;
  Timer? _reconnectTimer;
  Timer? _pingTimer;

  // Stream controllers for different message types (matching backend topics)
  StreamController<Map<String, dynamic>>? _realTimeDataController;
  StreamController<Map<String, dynamic>>? _controlEventsController;
  StreamController<Map<String, dynamic>>? _mappingEventsController;
  StreamController<Map<String, dynamic>>? _mapEventsController;
  StreamController<Map<String, dynamic>>? _orderEventsController;
  StreamController<Map<String, dynamic>>? _deviceEventsController;
  StreamController<bool>? _connectionStateController;
  StreamController<String>? _errorController;

  // ==========================================
  // GETTERS - PROPERTIES
  // ==========================================
  bool get isConnected => _isConnected;
  String? get clientId => _clientId;

  // ==========================================
  // GETTERS - STREAMS
  // ==========================================
  Stream<Map<String, dynamic>> get realTimeData {
    _initializeControllers();
    return _realTimeDataController!.stream;
  }

  Stream<Map<String, dynamic>> get controlEvents {
    _initializeControllers();
    return _controlEventsController!.stream;
  }

  Stream<Map<String, dynamic>> get mappingEvents {
    _initializeControllers();
    return _mappingEventsController!.stream;
  }

  Stream<Map<String, dynamic>> get mapEvents {
    _initializeControllers();
    return _mapEventsController!.stream;
  }

  Stream<Map<String, dynamic>> get orderEvents {
    _initializeControllers();
    return _orderEventsController!.stream;
  }

  Stream<Map<String, dynamic>> get deviceEvents {
    _initializeControllers();
    return _deviceEventsController!.stream;
  }

  Stream<bool> get connectionState {
    _initializeControllers();
    return _connectionStateController!.stream;
  }

  Stream<String> get errors {
    _initializeControllers();
    return _errorController!.stream;
  }

  // Legacy streams for backward compatibility
  Stream<Map<String, dynamic>> get odometry => realTimeData
      .where((data) => data['type'] == 'odometry_update')
      .map((data) => data['data'] ?? {});

  Stream<Map<String, dynamic>> get mapData => realTimeData
      .where((data) => data['type'] == 'map_update')
      .map((data) => data['data'] ?? {});

  Stream<Map<String, dynamic>> get batteryState => realTimeData
      .where((data) => data['type'] == 'battery_update')
      .map((data) => data['data'] ?? {});

  Stream<Map<String, dynamic>> get robotState => realTimeData
      .where((data) => data['type'] == 'robot_state')
      .map((data) => data['data'] ?? {});

  Stream<Map<String, dynamic>> get alerts => deviceEvents
      .where((data) => data['type'] == 'alert')
      .map((data) => data['data'] ?? {});

  // ==========================================
  // INITIALIZATION METHODS
  // ==========================================
  void _initializeControllers() {
    _realTimeDataController ??=
        StreamController<Map<String, dynamic>>.broadcast();
    _controlEventsController ??=
        StreamController<Map<String, dynamic>>.broadcast();
    _mappingEventsController ??=
        StreamController<Map<String, dynamic>>.broadcast();
    _mapEventsController ??= StreamController<Map<String, dynamic>>.broadcast();
    _orderEventsController ??=
        StreamController<Map<String, dynamic>>.broadcast();
    _deviceEventsController ??=
        StreamController<Map<String, dynamic>>.broadcast();
    _connectionStateController ??= StreamController<bool>.broadcast();
    _errorController ??= StreamController<String>.broadcast();
  }

  void _subscribeToTopics() {
    subscribe('real_time_data');
    subscribe('control_events');
    subscribe('mapping_events');
    subscribe('map_events');
    subscribe('order_events');
    subscribe('device_events');
  }

  // ==========================================
  // CONNECTION METHODS
  // ==========================================
  Future<bool> connect(
    String serverUrl, {
    String? deviceId,
    Map<String, dynamic>? deviceInfo,
  }) async {
    _serverUrl = serverUrl;
    _initializeControllers();
    return await _attemptConnection(
      deviceId: deviceId,
      deviceInfo: deviceInfo,
    );
  }

  Future<bool> _attemptConnection({
    String? deviceId,
    Map<String, dynamic>? deviceInfo,
  }) async {
    if (_isReconnecting) {
      print('🔄 Already attempting to reconnect, skipping...');
      return false;
    }

    _isReconnecting = true;

    try {
      // Close existing connection properly
      if (_channel != null) {
        try {
          await _channel!.sink.close(1000, 'Reconnecting');
        } catch (e) {
          print('⚠️ Error closing existing connection: $e');
        }
        _channel = null;
      }

      print('🔌 Connecting to WebSocket server: $_serverUrl');

      final uri = Uri.parse(_serverUrl!);

      // Create connection with TCP keep-alive
      _channel = IOWebSocketChannel.connect(
        uri,
        protocols: ['websocket'],
        connectTimeout: connectionTimeout,
      );

      // Test connection by sending ping
      bool connectionEstablished = false;
      final completer = Completer<bool>();

      late StreamSubscription subscription;
      Timer? timeoutTimer;

      timeoutTimer = Timer(connectionTimeout, () {
        if (!connectionEstablished) {
          print('⏰ Connection timeout');
          subscription.cancel();
          completer.complete(false);
        }
      });

      subscription = _channel!.stream.listen(
        (data) {
          if (!connectionEstablished) {
            connectionEstablished = true;
            timeoutTimer?.cancel();
            subscription.cancel();

            // Send device info after connection established
            if (deviceId != null || deviceInfo != null) {
              sendMessage({
                'type': 'register_device',
                if (deviceId != null) 'deviceId': deviceId,
                if (deviceInfo != null) 'deviceInfo': deviceInfo,
              });
            }

            // Set up permanent listener
            _setupPermanentListener();

            completer.complete(true);
          }
        },
        onError: (error) {
          print('❌ WebSocket stream error: $error');
          if (!connectionEstablished) {
            connectionEstablished = true;
            timeoutTimer?.cancel();
            subscription.cancel();
            completer.complete(false);
          }
        },
        onDone: () {
          print('🔌 WebSocket stream closed during connection');
          if (!connectionEstablished) {
            connectionEstablished = true;
            timeoutTimer?.cancel();
            subscription.cancel();
            completer.complete(false);
          }
        },
      );

      final result = await completer.future;

      if (result) {
        _reconnectAttempts = 0;
        _isConnected = true;
        _isReconnecting = false;

        _startApplicationPing();
        _startHeartbeat();
        _subscribeToTopics();

        print('✅ WebSocket connected successfully');

        if (_connectionStateController != null &&
            !_connectionStateController!.isClosed) {
          _connectionStateController!.add(true);
        }

        return true;
      } else {
        print('❌ WebSocket connection failed');
        _cleanup();
        return false;
      }
    } catch (e) {
      print('❌ Failed to connect to WebSocket: $e');
      _cleanup();
      return false;
    } finally {
      _isReconnecting = false;
    }
  }

  void _setupPermanentListener() {
    _channel!.stream.listen(
      _handleMessage,
      onError: (error) {
        print('❌ WebSocket error: $error');
        _handleDisconnection();
      },
      onDone: () {
        print('🔌 WebSocket connection closed');
        _handleDisconnection();
      },
    );
  }

  void disconnect() {
    _isConnected = false;
    _clientId = null;
    _stopApplicationPing();
    _stopHeartbeat();
    _stopReconnectTimer();
    if (_channel != null) {
      try {
        _channel!.sink.close();
      } catch (e) {
        // Ignore cleanup errors
      }
      _channel = null;
    }
    if (_connectionStateController != null &&
        !_connectionStateController!.isClosed) {
      _connectionStateController!.add(false);
    }
  }

  // ==========================================
  // PING/HEARTBEAT METHODS
  // ==========================================
  void _startApplicationPing() {
    _stopApplicationPing();

    _pingTimer = Timer.periodic(pingInterval, (timer) {
      if (_isConnected && _channel != null) {
        try {
          // Send application-level ping
          sendMessage({
            'type': 'ping',
            'timestamp': DateTime.now().toIso8601String(),
          });

          // Check if we received a recent pong
          if (_lastPongReceived != null) {
            final timeSincePong = DateTime.now().difference(_lastPongReceived!);
            if (timeSincePong > Duration(seconds: 30)) {
              print(
                  '💔 No pong received for ${timeSincePong.inSeconds}s, reconnecting...');
              _handleDisconnection();
              return;
            }
          }
        } catch (e) {
          print('❌ Error sending ping: $e');
          _handleDisconnection();
        }
      } else {
        timer.cancel();
      }
    });
  }

  void _stopApplicationPing() {
    _pingTimer?.cancel();
    _pingTimer = null;
  }

  void _startHeartbeat() {
    _heartbeatTimer?.cancel();
    _heartbeatTimer = Timer.periodic(Duration(seconds: 20), (timer) {
      if (_isConnected) {
        sendMessage({
          'type': 'heartbeat',
          'timestamp': DateTime.now().toIso8601String()
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
      'timestamp': DateTime.now().toIso8601String(),
    });
  }

  // ==========================================
  // MESSAGE HANDLING METHODS
  // ==========================================
  void _handleMessage(dynamic data) {
    try {
      final message = json.decode(data);

      switch (message['type']) {
        case 'connection':
          _isConnected = true;
          _clientId = message['clientId'];
          if (_connectionStateController != null &&
              !_connectionStateController!.isClosed) {
            _connectionStateController!.add(true);
          }
          print('✅ Connection established with client ID: $_clientId');
          break;

        case 'pong':
          _lastPongReceived = DateTime.now();
          print('🏓 Pong received from server');
          break;

        case 'ping':
          // Respond to server ping
          sendMessage({
            'type': 'pong',
            'timestamp': DateTime.now().toIso8601String(),
          });
          break;

        case 'initial_data':
          print(
              '📊 Received initial data: ${message['devices']?.length ?? 0} devices');
          if (_deviceEventsController != null &&
              !_deviceEventsController!.isClosed) {
            _deviceEventsController!.add({
              'type': 'initial_data',
              'devices': message['devices'] ?? [],
              'timestamp': DateTime.now().toIso8601String()
            });
          }
          break;

        case 'heartbeat':
          _sendHeartbeatAck();
          break;

        case 'heartbeat_ack':
          print('💓 Heartbeat acknowledged');
          break;

        case 'broadcast':
          _handleBroadcast(message['topic'], message['data'] ?? {});
          break;

        case 'subscription_ack':
          print('📡 Subscribed to topic: ${message['topic']}');
          if (message['deviceId'] != null) {
            print('   Device: ${message['deviceId']}');
          }
          break;

        case 'error':
          final errorMsg = message['message'] ?? 'Unknown server error';
          print('❌ Server error: $errorMsg');
          if (_errorController != null && !_errorController!.isClosed) {
            _errorController!.add(errorMsg);
          }
          break;

        case 'device_status':
          if (_deviceEventsController != null &&
              !_deviceEventsController!.isClosed) {
            _deviceEventsController!.add({
              'type': 'device_status_update',
              'deviceId': message['deviceId'],
              'status': message['status'],
              'data': message['data'] ?? {},
              'timestamp': DateTime.now().toIso8601String()
            });
          }
          break;

        case 'device_connected':
        case 'device_disconnected':
          print(
              '🔌 Device ${message['deviceId']} ${message['type'].replaceAll('device_', '')}');
          if (_deviceEventsController != null &&
              !_deviceEventsController!.isClosed) {
            _deviceEventsController!.add({
              'type': message['type'],
              'deviceId': message['deviceId'],
              'timestamp':
                  message['timestamp'] ?? DateTime.now().toIso8601String()
            });
          }
          break;

        case 'device_discovery_response':
          if (_deviceEventsController != null &&
              !_deviceEventsController!.isClosed) {
            _deviceEventsController!.add(message);
          }
          break;

        default:
          print('❓ Unknown message type: ${message['type']}');
          print('📨 Full message: $message');
      }
    } catch (e) {
      print('❌ Error handling message: $e');
      print('📨 Raw message data: $data');
    }
  }

  void _handleBroadcast(String topic, Map<String, dynamic> data) {
    try {
      switch (topic) {
        case 'real_time_data':
          if (_realTimeDataController != null &&
              !_realTimeDataController!.isClosed) {
            _realTimeDataController!.add(data);
          }
          break;
        case 'control_events':
          if (_controlEventsController != null &&
              !_controlEventsController!.isClosed) {
            _controlEventsController!.add(data);
          }
          break;
        case 'mapping_events':
          if (_mappingEventsController != null &&
              !_mappingEventsController!.isClosed) {
            _mappingEventsController!.add(data);
          }
          break;
        case 'map_events':
          if (_mapEventsController != null && !_mapEventsController!.isClosed) {
            _mapEventsController!.add(data);
          }
          break;
        case 'order_events':
          if (_orderEventsController != null &&
              !_orderEventsController!.isClosed) {
            _orderEventsController!.add(data);
          }
          break;
        case 'device_events':
          if (_deviceEventsController != null &&
              !_deviceEventsController!.isClosed) {
            _deviceEventsController!.add(data);
          }
          break;
        default:
          print('❓ Unknown broadcast topic: $topic');
      }
    } catch (e) {
      print('❌ Error handling broadcast for topic $topic: $e');
    }
  }

  void _handleDataResponse(Map<String, dynamic> message) {
    try {
      final requestType = message['requestType'];
      final data = message['data'] ?? {};

      switch (requestType) {
        case 'device_status':
          if (_deviceEventsController != null &&
              !_deviceEventsController!.isClosed) {
            _deviceEventsController!.add({
              'type': 'status_response',
              'data': data,
            });
          }
          break;
        case 'orders':
          if (_orderEventsController != null &&
              !_orderEventsController!.isClosed) {
            _orderEventsController!.add({
              'type': 'orders_response',
              'data': data,
            });
          }
          break;
      }
    } catch (e) {
      print('❌ Error handling data response: $e');
    }
  }

  void _handleError(error) {
    print('❌ WebSocket error: $error');
    if (_errorController != null && !_errorController!.isClosed) {
      _errorController!.add(error.toString());
    }
    _handleDisconnection();
  }

  // ==========================================
  // RECONNECTION METHODS
  // ==========================================
  void _handleDisconnection() {
    _isConnected = false;
    _clientId = null;

    if (_connectionStateController != null &&
        !_connectionStateController!.isClosed) {
      _connectionStateController!.add(false);
    }

    _stopApplicationPing();
    _stopHeartbeat();
    _stopReconnectTimer();

    if (_reconnectAttempts < maxReconnectAttempts &&
        _serverUrl != null &&
        !_isReconnecting) {
      _reconnectAttempts++;
      print(
          '🔄 WebSocket disconnected, attempting to reconnect... (attempt $_reconnectAttempts/$maxReconnectAttempts)');

      // Exponential backoff with jitter
      final baseDelay =
          Duration(seconds: math.min(2 << (_reconnectAttempts - 1), 30));
      final jitter = Duration(milliseconds: math.Random().nextInt(1000));
      final totalDelay = baseDelay + jitter;

      print('⏳ Waiting ${totalDelay.inSeconds}s before reconnect attempt...');

      _startReconnectTimer(totalDelay);
    } else {
      print('❌ WebSocket disconnected, max reconnection attempts reached');
      if (_errorController != null && !_errorController!.isClosed) {
        _errorController!
            .add('Connection lost after $_reconnectAttempts attempts');
      }
    }
  }

  void _startReconnectTimer([Duration? delay]) {
    _stopReconnectTimer();

    _reconnectTimer = Timer(delay ?? Duration(seconds: 5), () async {
      if (!_isConnected && _serverUrl != null && !_isReconnecting) {
        print(
            '🔄 Attempting to reconnect... (attempt $_reconnectAttempts/$maxReconnectAttempts)');
        final success = await _attemptConnection();
        if (!success) {
          _handleDisconnection();
        }
      }
    });
  }

  void _stopReconnectTimer() {
    _reconnectTimer?.cancel();
    _reconnectTimer = null;
  }

  // ==========================================
  // SUBSCRIPTION METHODS
  // ==========================================
  void subscribe(String topic, {String? deviceId}) {
    sendMessage({
      'type': 'subscribe',
      'topic': topic,
      'deviceId': deviceId,
    });
  }

  void unsubscribe(String topic, {String? deviceId}) {
    sendMessage({
      'type': 'unsubscribe',
      'topic': topic,
      'deviceId': deviceId,
    });
  }

  // ==========================================
  // CORE MESSAGING METHOD
  // ==========================================
  void sendMessage(Map<String, dynamic> message) {
    if (_isConnected && _channel != null) {
      try {
        _channel!.sink.add(json.encode(message));
      } catch (e) {
        print('❌ Error sending message: $e');
        if (_errorController != null && !_errorController!.isClosed) {
          _errorController!.add('Failed to send message: $e');
        }
      }
    } else {
      print(
          '⚠️ WebSocket not connected, cannot send message: ${message['type']}');
    }
  }

  // ==========================================
  // JOYSTICK & MOVEMENT CONTROL
  // ==========================================
  void sendJoystickControl(String deviceId, double x, double y, bool deadman) {
    sendMessage({
      'type': 'joystick_control',
      'deviceId': deviceId,
      'x': x,
      'y': y,
      'deadman': deadman,
      'timestamp': DateTime.now().toIso8601String(),
    });
  }

  void moveRobot(String deviceId, double linear, double angular) {
    sendControlCommand(deviceId, 'move', {
      'linear': linear,
      'angular': angular,
    });
  }

  void stopRobot(String deviceId) {
    sendControlCommand(deviceId, 'stop', {});
  }

  void setRobotGoal(String deviceId, double x, double y, double orientation) {
    sendControlCommand(deviceId, 'goal', {
      'x': x,
      'y': y,
      'orientation': orientation,
    });
  }

  // ==========================================
  // MAPPING CONTROL
  // ==========================================
  void sendMappingCommand(String deviceId, String command) {
    sendMessage({
      'type': 'mapping_command',
      'deviceId': deviceId,
      'command': command,
      'timestamp': DateTime.now().toIso8601String(),
    });
  }

  void startMapping(String deviceId) {
    sendMessage({
      'type': 'control_command',
      'deviceId': deviceId,
      'command': 'start_mapping',
      'data': {},
    });
  }

  void stopMapping(String deviceId) {
    sendMessage({
      'type': 'control_command',
      'deviceId': deviceId,
      'command': 'stop_mapping',
      'data': {},
    });
  }

  // ==========================================
  // MAP EDITING METHODS
  // ==========================================
  void sendMapEdit(
      String deviceId, String editType, Map<String, dynamic> editData) {
    sendMessage({
      'type': 'map_edit',
      'deviceId': deviceId,
      'editType': editType,
      'editData': editData,
    });
  }

  void addMapShape(String deviceId, Map<String, dynamic> shapeData) {
    sendMapEdit(deviceId, 'add_shape', shapeData);
  }

  void updateMapShape(
      String deviceId, String shapeId, Map<String, dynamic> updates) {
    sendMapEdit(deviceId, 'update_shape', {'id': shapeId, ...updates});
  }

  void deleteMapShape(String deviceId, String shapeId) {
    sendMapEdit(deviceId, 'delete_shape', {'id': shapeId});
  }

  void addMapAnnotation(String deviceId, Map<String, dynamic> annotationData) {
    sendMapEdit(deviceId, 'add_annotation', annotationData);
  }

  // ==========================================
  // ORDER MANAGEMENT METHODS
  // ==========================================
  void sendOrderCommand(
      String deviceId, String orderAction, Map<String, dynamic>? orderData) {
    sendMessage({
      'type': 'order_command',
      'deviceId': deviceId,
      'orderAction': orderAction,
      'orderData': orderData ?? {},
    });
  }

  void executeOrder(String deviceId, String orderId) {
    sendOrderCommand(deviceId, 'execute', {'orderId': orderId});
  }

  void pauseOrder(String deviceId, String orderId) {
    sendOrderCommand(deviceId, 'pause', {'orderId': orderId});
  }

  void cancelOrder(String deviceId, String orderId) {
    sendOrderCommand(deviceId, 'cancel', {'orderId': orderId});
  }

  // ==========================================
  // GENERAL CONTROL & DATA REQUESTS
  // ==========================================
  void sendControlCommand(
      String deviceId, String command, Map<String, dynamic>? data) {
    sendMessage({
      'type': 'control_command',
      'deviceId': deviceId,
      'command': command,
      'data': data ?? {},
    });
  }

  void requestData(String requestType, {String? deviceId}) {
    sendMessage({
      'type': 'request_data',
      'requestType': requestType,
      'deviceId': deviceId,
    });
  }

  void requestDeviceDiscovery() {
    sendMessage({'type': 'device_discovery_request'});
  }

  // ==========================================
  // UTILITY METHODS
  // ==========================================
  void ping() {
    sendMessage({
      'type': 'ping',
      'timestamp': DateTime.now().toIso8601String(),
    });
  }

  Map<String, dynamic> getConnectionInfo() {
    return {
      'isConnected': _isConnected,
      'clientId': _clientId,
      'serverUrl': _serverUrl,
      'reconnectAttempts': _reconnectAttempts,
    };
  }

  // ==========================================
  // CLEANUP METHODS
  // ==========================================
  void _cleanup() {
    _isReconnecting = false;
    _stopApplicationPing();
    _stopHeartbeat();
    if (_channel != null) {
      try {
        _channel!.sink.close();
      } catch (e) {
        // Ignore cleanup errors
      }
      _channel = null;
    }
  }

  void dispose() {
    // Close all stream controllers
    _realTimeDataController?.close();
    _controlEventsController?.close();
    _mappingEventsController?.close();
    _mapEventsController?.close();
    _orderEventsController?.close();
    _deviceEventsController?.close();
    _connectionStateController?.close();
    _errorController?.close();

    // Set them to null
    _realTimeDataController = null;
    _controlEventsController = null;
    _mappingEventsController = null;
    _mapEventsController = null;
    _orderEventsController = null;
    _deviceEventsController = null;
    _connectionStateController = null;
    _errorController = null;

    disconnect();
  }
}