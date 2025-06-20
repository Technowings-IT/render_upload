//services/websocket_service.dart - Fixed Version
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/io.dart';

class WebSocketService {
  static final WebSocketService _instance = WebSocketService._internal();
  factory WebSocketService() => _instance;
  WebSocketService._internal();

  WebSocketChannel? _channel;
  String? _clientId;
  bool _isConnected = false;
  Timer? _heartbeatTimer;
  Timer? _reconnectTimer;
  String? _serverUrl;
  int _reconnectAttempts = 0;
  static const int maxReconnectAttempts = 5;
  
  // Stream controllers for different message types (matching backend topics)
  StreamController<Map<String, dynamic>>? _realTimeDataController;
  StreamController<Map<String, dynamic>>? _controlEventsController;
  StreamController<Map<String, dynamic>>? _mappingEventsController;
  StreamController<Map<String, dynamic>>? _mapEventsController;
  StreamController<Map<String, dynamic>>? _orderEventsController;
  StreamController<Map<String, dynamic>>? _deviceEventsController;
  StreamController<bool>? _connectionStateController;
  StreamController<String>? _errorController;

  // Initialize controllers
  void _initializeControllers() {
    _realTimeDataController ??= StreamController<Map<String, dynamic>>.broadcast();
    _controlEventsController ??= StreamController<Map<String, dynamic>>.broadcast();
    _mappingEventsController ??= StreamController<Map<String, dynamic>>.broadcast();
    _mapEventsController ??= StreamController<Map<String, dynamic>>.broadcast();
    _orderEventsController ??= StreamController<Map<String, dynamic>>.broadcast();
    _deviceEventsController ??= StreamController<Map<String, dynamic>>.broadcast();
    _connectionStateController ??= StreamController<bool>.broadcast();
    _errorController ??= StreamController<String>.broadcast();
  }

  // Public streams (matching backend topics)
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

  bool get isConnected => _isConnected;
  String? get clientId => _clientId;

  Future<bool> connect(String serverUrl) async {
    _serverUrl = serverUrl;
    _initializeControllers();
    return await _attemptConnection();
  }

  Future<bool> _attemptConnection() async {
    try {
      // Close existing connection but don't dispose controllers
      if (_channel != null) {
        await _channel!.sink.close();
        _channel = null;
      }

      print('🔌 Connecting to WebSocket server: $_serverUrl');
      
      // Add URL validation
      final uri = Uri.parse(_serverUrl!);
      print('📋 Parsed URI: scheme=${uri.scheme}, host=${uri.host}, port=${uri.port}');
      
      if (uri.scheme != 'ws' && uri.scheme != 'wss') {
        throw Exception('Invalid WebSocket scheme: ${uri.scheme}. Expected ws:// or wss://');
      }

      _channel = IOWebSocketChannel.connect(
        uri,
        protocols: ['websocket'],
        connectTimeout: Duration(seconds: 10),
      );

      // Wait for connection with timeout
      bool connectionEstablished = false;
      final completer = Completer<bool>();
      
      // Set up listeners before waiting
      late StreamSubscription subscription;
      subscription = _channel!.stream.listen(
        (data) {
          print('📨 Message received: $data');
          if (!connectionEstablished) {
            connectionEstablished = true;
            subscription.cancel();
            completer.complete(true);
          } else {
            _handleMessage(data);
          }
        },
        onError: (error) {
          print('❌ WebSocket stream error: $error');
          if (!connectionEstablished) {
            connectionEstablished = true;
            subscription.cancel();
            completer.complete(false);
          } else {
            _handleError(error);
          }
        },
        onDone: () {
          print('🔌 WebSocket stream closed');
          if (!connectionEstablished) {
            connectionEstablished = true;
            subscription.cancel();
            completer.complete(false);
          } else {
            _handleDisconnection();
          }
        },
      );

      // Wait for connection confirmation or timeout
      final result = await Future.any([
        completer.future,
        Future.delayed(Duration(seconds: 10), () => false),
      ]);

      if (result) {
        // Set up proper listener for ongoing messages
        _channel!.stream.listen(
          _handleMessage,
          onError: _handleError,
          onDone: _handleDisconnection,
        );
        
        _reconnectAttempts = 0;
        _startHeartbeat();
        _subscribeToTopics();
        print('✅ WebSocket connected successfully with client ID: $_clientId');
        return true;
      } else {
        print('❌ WebSocket connection timeout or failed');
        if (_channel != null) {
          await _channel!.sink.close();
          _channel = null;
        }
        return false;
      }
    } catch (e) {
      print('❌ Failed to connect to WebSocket: $e');
      print('📊 Connection details: serverUrl=$_serverUrl');
      _handleDisconnection();
      return false;
    }
  }

  Future<void> disconnect() async {
    try {
      _stopHeartbeat();
      _stopReconnectTimer();
      
      if (_channel != null) {
        await _channel!.sink.close();
        _channel = null;
      }
      
      _isConnected = false;
      _clientId = null;
      
      if (_connectionStateController != null && !_connectionStateController!.isClosed) {
        _connectionStateController!.add(false);
      }
      
      print('🔌 WebSocket disconnected');
    } catch (e) {
      print('❌ Error during disconnect: $e');
    }
  }

  void _handleMessage(dynamic data) {
    try {
      final message = json.decode(data);
      
      switch (message['type']) {
        case 'connection':
          _isConnected = true;
          _clientId = message['clientId'];
          if (_connectionStateController != null && !_connectionStateController!.isClosed) {
            _connectionStateController!.add(true);
          }
          print('✅ Connection established with client ID: $_clientId');
          
          // Send capabilities info
          final capabilities = message['capabilities'];
          if (capabilities != null) {
            print('🚀 Server capabilities: ${capabilities.keys.join(', ')}');
          }
          break;

        case 'heartbeat':
          _sendHeartbeatAck();
          break;

        case 'heartbeat_ack':
          // Heartbeat acknowledged
          break;

        case 'broadcast':
          _handleBroadcast(message['topic'], message['data'] ?? {});
          break;

        case 'subscription_ack':
          print('📡 Subscribed to topic: ${message['topic']}');
          break;

        case 'unsubscription_ack':
          print('📡 Unsubscribed from topic: ${message['topic']}');
          break;

        case 'command_ack':
          print('✅ Command acknowledged: ${message['command']} for device ${message['deviceId']}');
          break;

        case 'map_edit_ack':
          print('✅ Map edit acknowledged: ${message['editType']}');
          break;

        case 'order_ack':
          print('✅ Order command acknowledged: ${message['orderAction']}');
          break;

        case 'error':
          final errorMsg = message['message'] ?? 'Unknown server error';
          print('❌ Server error: $errorMsg');
          if (_errorController != null && !_errorController!.isClosed) {
            _errorController!.add(errorMsg);
          }
          break;

        case 'initial_data':
          print('📊 Received initial data: ${message['devices']?.length ?? 0} devices');
          if (_deviceEventsController != null && !_deviceEventsController!.isClosed) {
            _deviceEventsController!.add({
              'type': 'initial_data',
              'devices': message['devices'] ?? []
            });
          }
          break;

        case 'initial_topic_data':
          print('📊 Received initial topic data for: ${message['topic']}');
          _handleBroadcast(message['topic'], {
            'type': 'initial_data',
            'data': message['data'] ?? {}
          });
          break;

        case 'data_response':
          print('📊 Received data response for: ${message['requestType']}');
          _handleDataResponse(message);
          break;

        default:
          print('❓ Unknown message type: ${message['type']}');
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
          if (_realTimeDataController != null && !_realTimeDataController!.isClosed) {
            _realTimeDataController!.add(data);
          }
          break;
        case 'control_events':
          if (_controlEventsController != null && !_controlEventsController!.isClosed) {
            _controlEventsController!.add(data);
          }
          break;
        case 'mapping_events':
          if (_mappingEventsController != null && !_mappingEventsController!.isClosed) {
            _mappingEventsController!.add(data);
          }
          break;
        case 'map_events':
          if (_mapEventsController != null && !_mapEventsController!.isClosed) {
            _mapEventsController!.add(data);
          }
          break;
        case 'order_events':
          if (_orderEventsController != null && !_orderEventsController!.isClosed) {
            _orderEventsController!.add(data);
          }
          break;
        case 'device_events':
          if (_deviceEventsController != null && !_deviceEventsController!.isClosed) {
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
          if (_deviceEventsController != null && !_deviceEventsController!.isClosed) {
            _deviceEventsController!.add({
              'type': 'status_response',
              'data': data,
            });
          }
          break;
        case 'orders':
          if (_orderEventsController != null && !_orderEventsController!.isClosed) {
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

  void _handleDisconnection() {
    _isConnected = false;
    _clientId = null;
    
    if (_connectionStateController != null && !_connectionStateController!.isClosed) {
      _connectionStateController!.add(false);
    }
    
    _stopHeartbeat();
    
    if (_reconnectAttempts < maxReconnectAttempts && _serverUrl != null) {
      print('🔄 WebSocket disconnected, attempting to reconnect... (attempt ${_reconnectAttempts + 1}/$maxReconnectAttempts)');
      _startReconnectTimer();
    } else {
      print('❌ WebSocket disconnected, max reconnection attempts reached');
      if (_errorController != null && !_errorController!.isClosed) {
        _errorController!.add('Connection lost and unable to reconnect');
      }
    }
  }

  void _startHeartbeat() {
    _heartbeatTimer = Timer.periodic(Duration(seconds: 25), (timer) {
      if (_isConnected) {
        _sendHeartbeat();
      }
    });
  }

  void _stopHeartbeat() {
    _heartbeatTimer?.cancel();
    _heartbeatTimer = null;
  }

  void _startReconnectTimer() {
    _reconnectTimer = Timer(Duration(seconds: 5 + _reconnectAttempts * 2), () async {
      if (!_isConnected && _serverUrl != null) {
        _reconnectAttempts++;
        print('🔄 Attempting to reconnect... (attempt $_reconnectAttempts/$maxReconnectAttempts)');
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
    _reconnectAttempts = 0;
  }

  void _sendHeartbeat() {
    sendMessage({
      'type': 'heartbeat',
      'timestamp': DateTime.now().toIso8601String(),
    });
  }

  void _sendHeartbeatAck() {
    sendMessage({
      'type': 'heartbeat_ack',
      'timestamp': DateTime.now().toIso8601String(),
    });
  }

  void _subscribeToTopics() {
    final topics = [
      'real_time_data',
      'control_events',
      'mapping_events',
      'map_events',
      'order_events',
      'device_events',
    ];

    for (String topic in topics) {
      subscribe(topic);
    }
  }

  // Public subscription methods
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
      print('⚠️ WebSocket not connected, cannot send message: ${message['type']}');
    }
  }

  // Control commands (matching backend WebSocket handlers)
  void sendControlCommand(String deviceId, String command, Map<String, dynamic>? data) {
    sendMessage({
      'type': 'control_command',
      'deviceId': deviceId,
      'command': command,
      'data': data ?? {},
    });
  }

  void sendMapEdit(String deviceId, String editType, Map<String, dynamic> editData) {
    sendMessage({
      'type': 'map_edit',
      'deviceId': deviceId,
      'editType': editType,
      'editData': editData,
    });
  }

  void sendOrderCommand(String deviceId, String orderAction, Map<String, dynamic>? orderData) {
    sendMessage({
      'type': 'order_command',
      'deviceId': deviceId,
      'orderAction': orderAction,
      'orderData': orderData ?? {},
    });
  }

  void requestData(String requestType, {String? deviceId}) {
    sendMessage({
      'type': 'request_data',
      'requestType': requestType,
      'deviceId': deviceId,
    });
  }

  // Convenience methods for common commands
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

  // Map editing convenience methods
  void addMapShape(String deviceId, Map<String, dynamic> shapeData) {
    sendMapEdit(deviceId, 'add_shape', shapeData);
  }

  void updateMapShape(String deviceId, String shapeId, Map<String, dynamic> updates) {
    sendMapEdit(deviceId, 'update_shape', {'id': shapeId, ...updates});
  }

  void deleteMapShape(String deviceId, String shapeId) {
    sendMapEdit(deviceId, 'delete_shape', {'id': shapeId});
  }

  void addMapAnnotation(String deviceId, Map<String, dynamic> annotationData) {
    sendMapEdit(deviceId, 'add_annotation', annotationData);
  }

  // Order management convenience methods
  void executeOrder(String deviceId, String orderId) {
    sendOrderCommand(deviceId, 'execute', {'orderId': orderId});
  }

  void pauseOrder(String deviceId, String orderId) {
    sendOrderCommand(deviceId, 'pause', {'orderId': orderId});
  }

  void cancelOrder(String deviceId, String orderId) {
    sendOrderCommand(deviceId, 'cancel', {'orderId': orderId});
  }

  // Utility methods
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