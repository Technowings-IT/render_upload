// services/web_socket_service.dart - FIXED with Complete Implementation
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
  
  // ✅ FIXED: Added missing analytics fields
  final Map<String, int> _messageStats = {};
  int _totalMessagesReceived = 0;
  int _totalMessagesSent = 0;
  DateTime? _lastConnectionTime;

  // ✅ FIXED: Added missing message queue fields
  final List<Map<String, dynamic>> _messageQueue = [];
  static const int _maxQueueSize = 100;

  // Connection state
  bool _isConnected = false;
  bool _isReconnecting = false;
  int _reconnectAttempts = 0;
  DateTime? _lastPongReceived;
  DateTime? _lastMessageReceived;

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
  
  // ✅ FIXED: Added analytics controller
  final StreamController<Map<String, dynamic>> _analyticsDataController = 
      StreamController<Map<String, dynamic>>.broadcast();

  // ==========================================
  // GETTERS - PROPERTIES
  // ==========================================
  bool get isConnected => _isConnected;
  String? get clientId => _clientId;
  
  // ✅ FIXED: Analytics getters
  Map<String, int> get messageStats => Map.unmodifiable(_messageStats);
  int get totalMessagesReceived => _totalMessagesReceived;
  int get totalMessagesSent => _totalMessagesSent;
  Duration? get connectionDuration => _lastConnectionTime != null
      ? DateTime.now().difference(_lastConnectionTime!)
      : null;

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

  // ✅ FIXED: Analytics stream
  Stream<Map<String, dynamic>> get analyticsData => _analyticsDataController.stream;

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
    if (_isConnected) {
      subscribe('real_time_data');
      subscribe('control_events');
      subscribe('mapping_events');
      subscribe('map_events');
      subscribe('order_events');
      subscribe('device_events');

      // Add costmap subscriptions
      subscribe('global_costmap_update');
      subscribe('local_costmap_update');

      // Process any queued messages
      _processMessageQueue();

      print('📡 Subscribed to all topics, waiting for server data...');
    }
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
      await _cleanup();

      print('🔌 Connecting to WebSocket server: $_serverUrl');

      final uri = Uri.parse(_serverUrl!);

      // ✅ FIXED: Create connection with proper options
      _channel = IOWebSocketChannel.connect(
        uri,
        protocols: ['websocket'],
        connectTimeout: connectionTimeout,
        pingInterval: const Duration(seconds: 20),
      );

      // ✅ FIXED: Improved connection establishment
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
          _lastMessageReceived = DateTime.now();

          if (!connectionEstablished) {
            connectionEstablished = true;
            timeoutTimer?.cancel();
            _lastConnectionTime = DateTime.now();

            _isConnected = true;

            if (deviceId != null || deviceInfo != null) {
              sendMessage({
                'type': 'device_connect',
                if (deviceId != null) 'deviceId': deviceId,
                if (deviceInfo != null) 'deviceInfo': deviceInfo,
                'timestamp': DateTime.now().toIso8601String(),
              });
            }

            completer.complete(true);
          }

          _handleMessage(data);
        },
        onError: (error) {
          print('❌ WebSocket stream error: $error');
          _handleDisconnection();
          if (!connectionEstablished) {
            connectionEstablished = true;
            timeoutTimer?.cancel();
            completer.complete(false);
          }
        },
        onDone: () {
          print('🔌 WebSocket stream closed during connection');
          _handleDisconnection();
          if (!connectionEstablished) {
            connectionEstablished = true;
            timeoutTimer?.cancel();
            completer.complete(false);
          }
        },
      );

      final result = await completer.future;

      if (result) {
        _reconnectAttempts = 0;
        _isConnected = true;
        _isReconnecting = false;
        _lastMessageReceived = DateTime.now();

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
        await _cleanup();
        return false;
      }
    } catch (e) {
      print('❌ Failed to connect to WebSocket: $e');
      await _cleanup();
      return false;
    } finally {
      _isReconnecting = false;
    }
  }

  void disconnect() {
    print('📱 Disconnecting WebSocket...');
    _isConnected = false;
    _clientId = null;
    _stopApplicationPing();
    _stopHeartbeat();
    _stopReconnectTimer();

    if (_channel != null) {
      try {
        _channel!.sink.close(1000, 'Client disconnect');
      } catch (e) {
        print('⚠️ Error closing WebSocket: $e');
      }
      _channel = null;
    }

    if (_connectionStateController != null &&
        !_connectionStateController!.isClosed) {
      _connectionStateController!.add(false);
    }

    print('✅ WebSocket disconnected');
  }

  // ==========================================
  // ✅ FIXED: PING/HEARTBEAT METHODS
  // ==========================================
  void _startApplicationPing() {
    _stopApplicationPing();

    _pingTimer = Timer.periodic(const Duration(seconds: 20), (timer) {
      if (_isConnected && _channel != null) {
        try {
          sendMessage({
            'type': 'ping',
            'timestamp': DateTime.now().toIso8601String(),
          });

          if (_lastMessageReceived != null) {
            final timeSinceLastMessage =
                DateTime.now().difference(_lastMessageReceived!);
            if (timeSinceLastMessage > const Duration(minutes: 2)) {
              print(
                  '💔 No messages received for ${timeSinceLastMessage.inSeconds}s, reconnecting...');
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
    _heartbeatTimer = Timer.periodic(const Duration(seconds: 30), (timer) {
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

  // ✅ FIXED: MESSAGE QUEUE HANDLING
  void _queueMessage(Map<String, dynamic> message) {
    if (_messageQueue.length < _maxQueueSize) {
      _messageQueue.add(message);
      print('📥 Message queued: ${message['type']}');
    } else {
      print('🗑️ Message queue full, dropping message: ${message['type']}');
    }
  }

  void _processMessageQueue() {
    if (_messageQueue.isNotEmpty) {
      print('📤 Processing ${_messageQueue.length} queued messages');
      final messages = List<Map<String, dynamic>>.from(_messageQueue);
      _messageQueue.clear();
      for (final message in messages) {
        sendMessage(message);
      }
    }
  }

  // ==========================================
  // ✅ FIXED: MESSAGE HANDLING METHODS
  // ==========================================
  void _handleMessage(dynamic data) {
    try {
      _totalMessagesReceived++;
      final message = json.decode(data);
      final type = message['type'];
      if (type != null) {
        _messageStats[type] = (_messageStats[type] ?? 0) + 1;
      }

      // Handle analytics data routing
      if (type != null && ['battery_update', 'velocity_update', 'error_event', 
          'system_metrics', 'order_completed'].contains(type)) {
        if (!_analyticsDataController.isClosed) {
          _analyticsDataController.add(message);
        }
      }

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

        case 'ping':
          _sendPongResponse();
          break;

        case 'pong':
          _lastPongReceived = DateTime.now();
          print('🏓 Pong received from server');
          break;

        case 'heartbeat':
          _sendHeartbeatAck();
          break;

        case 'heartbeat_ack':
          print('💓 Heartbeat acknowledged');
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

        case 'broadcast':
          _handleBroadcast(message['topic'], message['data'] ?? {});
          break;

        case 'subscription_ack':
        case 'subscription_confirmed':
          print(
              '📡 Subscribed to topic: ${message['topic'] ?? message['topics']}');
          if (message['deviceId'] != null) {
            print('   Device: ${message['deviceId']}');
          }
          break;

        case 'device_connected':
          print('🔌 Device connection confirmed: ${message['deviceId']}');
          if (_deviceEventsController != null &&
              !_deviceEventsController!.isClosed) {
            _deviceEventsController!.add({
              'type': 'device_connected',
              'deviceId': message['deviceId'],
              'timestamp': DateTime.now().toIso8601String()
            });
          }
          break;

        case 'joystick_result':
          if (_controlEventsController != null &&
              !_controlEventsController!.isClosed) {
            _controlEventsController!.add({
              'type': 'joystick_result',
              'deviceId': message['deviceId'],
              'result': message['result'],
              'timestamp': DateTime.now().toIso8601String()
            });
          }
          break;

        case 'mapping_result':
          if (_mappingEventsController != null &&
              !_mappingEventsController!.isClosed) {
            _mappingEventsController!.add({
              'type': 'mapping_result',
              'deviceId': message['deviceId'],
              'command': message['command'],
              'result': message['result'],
              'timestamp': DateTime.now().toIso8601String()
            });
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

        case 'device_disconnected':
          print('🔌 Device ${message['deviceId']} disconnected');
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
      }
    } catch (e) {
      print('❌ Error handling message: $e');
      print('📨 Raw message data: $data');
    }
  }

  void _sendPongResponse() {
    try {
      sendMessage({
        'type': 'pong',
        'timestamp': DateTime.now().toIso8601String(),
      });
      print('🏓 Sent pong response to server');
    } catch (e) {
      print('❌ Error sending pong: $e');
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

  // ==========================================
  // ✅ FIXED: RECONNECTION METHODS
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

      final baseDelay = Duration(seconds: math.min(2 + _reconnectAttempts, 10));
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

    _reconnectTimer = Timer(delay ?? const Duration(seconds: 5), () async {
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
      'topics': [topic],
      'deviceId': deviceId,
      'timestamp': DateTime.now().toIso8601String(),
    });
  }

  void unsubscribe(String topic, {String? deviceId}) {
    sendMessage({
      'type': 'unsubscribe',
      'topics': [topic],
      'deviceId': deviceId,
      'timestamp': DateTime.now().toIso8601String(),
    });
  }

  // ==========================================
  // CORE MESSAGING METHOD
  // ==========================================
  bool sendMessage(Map<String, dynamic> message) {
    try {
      if (!_isConnected || _channel == null) {
        if (_messageQueue.length < _maxQueueSize) {
          _messageQueue.add({
            ...message,
            'queued_at': DateTime.now().toIso8601String(),
          });
          print('📮 Message queued (offline): ${message['type']}');
          return true;
        } else {
          print('❌ Message queue full, dropping message: ${message['type']}');
          return false;
        }
      }

      final jsonMessage = json.encode({
        ...message,
        'client_timestamp': DateTime.now().toIso8601String(),
      });

      _channel!.sink.add(jsonMessage);
      _totalMessagesSent++;
      _messageStats[message['type']] = (_messageStats[message['type']] ?? 0) + 1;

      print('📤 Sent: ${message['type']} to ${message['deviceId'] ?? 'server'}');
      return true;
    } catch (e) {
      print('❌ Error sending message: $e');
      _errorController?.add('Failed to send message: $e');
      return false;
    }
  }

  // ==========================================
  // JOYSTICK & MOVEMENT CONTROL
  // ==========================================
  void sendJoystickControl(String deviceId, double x, double y, bool deadman, 
      {double? maxLinearSpeed, double? maxAngularSpeed}) {
    sendMessage({
      'type': 'joystick_control',
      'deviceId': deviceId,
      'x': x,
      'y': y,
      'deadman': deadman,
      'maxLinearSpeed': maxLinearSpeed,
      'maxAngularSpeed': maxAngularSpeed,
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
    sendMappingCommand(deviceId, 'start');
  }

  void stopMapping(String deviceId) {
    sendMappingCommand(deviceId, 'stop');
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

  void requestDeviceDiscovery() {
    sendMessage({
      'type': 'device_discovery',
      'timestamp': DateTime.now().toIso8601String(),
    });
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
      'lastMessageReceived': _lastMessageReceived?.toIso8601String(),
      'lastPongReceived': _lastPongReceived?.toIso8601String(),
      'messageStats': messageStats,
      'totalMessagesReceived': totalMessagesReceived,
      'totalMessagesSent': totalMessagesSent,
      'connectionDuration': connectionDuration?.inSeconds,
    };
  }

  void clearMessageStats() {
    _messageStats.clear();
    _totalMessagesReceived = 0;
    _totalMessagesSent = 0;
  }

  void clearMessageQueue() {
    _messageQueue.clear();
    print('🗑️ Message queue cleared');
  }

  // ==========================================
  // CLEANUP METHODS
  // ==========================================
  Future<void> _cleanup() async {
    _isReconnecting = false;
    _stopApplicationPing();
    _stopHeartbeat();
    _stopReconnectTimer();

    if (_channel != null) {
      try {
        await _channel!.sink.close(1000, 'Cleanup');
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
    _analyticsDataController.close();
    
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