// services/enhanced_ros2_service.dart - Enhanced ROS2 service with command-based control
import 'dart:convert';
import 'package:http/http.dart' as http;
import 'web_socket_service.dart';

class EnhancedROS2Service {
  final String baseUrl;
  final WebSocketService _webSocketService;

  // Command tracking
  final Map<String, CommandStatus> _activeCommands = {};
  final List<CommandResult> _commandHistory = [];

  EnhancedROS2Service({
    required this.baseUrl,
    required WebSocketService webSocketService,
  }) : _webSocketService = webSocketService {
    _setupResponseHandlers();
  }

  void _setupResponseHandlers() {
    // Listen for command responses
    _webSocketService.realTimeData.listen((data) {
      _handleCommandResponse(data);
    });
  }

  void _handleCommandResponse(Map<String, dynamic> data) {
    final commandId = data['commandId'];
    if (commandId != null && _activeCommands.containsKey(commandId)) {
      final command = _activeCommands[commandId]!;

      switch (data['type']) {
        case 'slam_command_response':
        case 'navigation_command_response':
        case 'stop_process_response':
        case 'emergency_stop_response':
          _completeCommand(commandId, data);
          break;
        case 'ros2_status_update':
          // Update status for all relevant commands
          _updateCommandStatus(data);
          break;
      }
    }
  }

  void _completeCommand(String commandId, Map<String, dynamic> response) {
    if (_activeCommands.containsKey(commandId)) {
      final command = _activeCommands[commandId]!;
      command.status =
          response['success'] ? CommandState.completed : CommandState.failed;
      command.response = response;
      command.completedAt = DateTime.now();

      // Add to history
      _commandHistory.add(CommandResult.fromCommand(command));

      // Remove from active commands
      _activeCommands.remove(commandId);

      print(' Command completed: $commandId - ${command.type}');
    }
  }

  void _updateCommandStatus(Map<String, dynamic> data) {
    // Update status for commands that might be affected
    final processUpdate = data['data'];
    if (processUpdate != null) {
      _activeCommands.values
          .where((cmd) =>
              cmd.type.contains(processUpdate['process']) &&
              cmd.status == CommandState.active)
          .forEach((cmd) {
        cmd.lastUpdate = DateTime.now();
        cmd.statusData = processUpdate;
      });
    }
  }

  // ==========================================
  // COMMAND-BASED ROS2 CONTROL METHODS
  // ==========================================

  /// Start SLAM with enhanced command tracking
  Future<CommandResult> startSLAM({
    String? mapName,
    bool useSimTime = false,
    Duration timeout = const Duration(seconds: 30),
  }) async {
    final commandId = _generateCommandId();
    final command = CommandStatus(
      id: commandId,
      type: 'start_slam',
      parameters: {
        'mapName': mapName,
        'useSimTime': useSimTime,
      },
      startedAt: DateTime.now(),
    );

    _activeCommands[commandId] = command;

    try {
      print('️ Starting SLAM - Command ID: $commandId');

      // Send command via WebSocket for real-time response
      _webSocketService.sendMessage({
        'type': 'start_slam',
        'commandId': commandId,
        'data': {
          'mapName': mapName,
          'useSimTime': useSimTime,
        }
      });

      // Wait for completion or timeout
      return await _waitForCommandCompletion(commandId, timeout);
    } catch (e) {
      _failCommand(commandId, e.toString());
      throw Exception('Failed to start SLAM: $e');
    }
  }

  /// Start Navigation with enhanced command tracking
  Future<CommandResult> startNavigation({
    String? mapPath,
    String? mapName,
    bool useSimTime = false,
    String? navParamsFile,
    Duration timeout = const Duration(seconds: 30),
  }) async {
    final commandId = _generateCommandId();
    final command = CommandStatus(
      id: commandId,
      type: 'start_navigation',
      parameters: {
        'mapPath': mapPath,
        'mapName': mapName,
        'useSimTime': useSimTime,
        'navParamsFile': navParamsFile,
      },
      startedAt: DateTime.now(),
    );

    _activeCommands[commandId] = command;

    try {
      if (mapPath == null && mapName == null) {
        throw Exception('Either mapPath or mapName must be provided');
      }

      print(' Starting Navigation - Command ID: $commandId');

      // Send command via WebSocket for real-time response
      _webSocketService.sendMessage({
        'type': 'start_navigation',
        'commandId': commandId,
        'data': {
          'mapPath': mapPath,
          'mapName': mapName,
          'useSimTime': useSimTime,
          'navParamsFile': navParamsFile,
        }
      });

      // Wait for completion or timeout
      return await _waitForCommandCompletion(commandId, timeout);
    } catch (e) {
      _failCommand(commandId, e.toString());
      throw Exception('Failed to start navigation: $e');
    }
  }

  /// Stop ROS2 process with enhanced command tracking
  Future<CommandResult> stopProcess(
    String processName, {
    Duration timeout = const Duration(seconds: 15),
  }) async {
    final commandId = _generateCommandId();
    final command = CommandStatus(
      id: commandId,
      type: 'stop_process',
      parameters: {'processName': processName},
      startedAt: DateTime.now(),
    );

    _activeCommands[commandId] = command;

    try {
      final validProcesses = ['slam', 'navigation', 'robot_control', 'all'];
      if (!validProcesses.contains(processName)) {
        throw Exception(
            'Invalid process name. Valid options: ${validProcesses.join(', ')}');
      }

      print(' Stopping process: $processName - Command ID: $commandId');

      // Send command via WebSocket for real-time response
      _webSocketService.sendMessage({
        'type': 'stop_ros2_process',
        'commandId': commandId,
        'data': {
          'processName': processName,
        }
      });

      // Wait for completion or timeout
      return await _waitForCommandCompletion(commandId, timeout);
    } catch (e) {
      _failCommand(commandId, e.toString());
      throw Exception('Failed to stop $processName: $e');
    }
  }

  /// Emergency stop with immediate response
  Future<CommandResult> emergencyStop({
    Duration timeout = const Duration(seconds: 10),
  }) async {
    final commandId = _generateCommandId();
    final command = CommandStatus(
      id: commandId,
      type: 'emergency_stop',
      parameters: {},
      startedAt: DateTime.now(),
    );

    _activeCommands[commandId] = command;

    try {
      print(' EMERGENCY STOP - Command ID: $commandId');

      // Send emergency stop command
      _webSocketService.sendMessage(
          {'type': 'emergency_stop', 'commandId': commandId, 'data': {}});

      // Wait for completion or timeout
      return await _waitForCommandCompletion(commandId, timeout);
    } catch (e) {
      _failCommand(commandId, e.toString());
      throw Exception('Emergency stop failed: $e');
    }
  }

  /// Restart ROS2 process
  Future<CommandResult> restartProcess(
    String processName, {
    Map<String, dynamic>? options,
    Duration timeout = const Duration(seconds: 45),
  }) async {
    final commandId = _generateCommandId();
    final command = CommandStatus(
      id: commandId,
      type: 'restart_process',
      parameters: {
        'processName': processName,
        'options': options ?? {},
      },
      startedAt: DateTime.now(),
    );

    _activeCommands[commandId] = command;

    try {
      print(' Restarting process: $processName - Command ID: $commandId');

      // Send command via WebSocket
      _webSocketService.sendMessage({
        'type': 'restart_ros2_process',
        'commandId': commandId,
        'data': {
          'processName': processName,
          'options': options ?? {},
        }
      });

      // Wait for completion or timeout
      return await _waitForCommandCompletion(commandId, timeout);
    } catch (e) {
      _failCommand(commandId, e.toString());
      throw Exception('Failed to restart $processName: $e');
    }
  }

  /// Get real-time ROS2 status
  Future<ROS2Status> getStatus() async {
    final commandId = _generateCommandId();

    try {
      // Send status request via WebSocket for real-time data
      _webSocketService.sendMessage({
        'type': 'get_ros2_status',
        'commandId': commandId,
      });

      // For status, we can also use HTTP as backup
      final response = await http.get(
        Uri.parse('$baseUrl/api/ros2/status'),
        headers: {'Content-Type': 'application/json'},
      ).timeout(const Duration(seconds: 10));

      final result = _handleResponse(response);
      return ROS2Status.fromJson(result['data']);
    } catch (e) {
      throw Exception('Failed to get ROS2 status: $e');
    }
  }

  // ==========================================
  // MAP MANAGEMENT WITH COMMAND TRACKING
  // ==========================================

  /// Upload and convert map with deployment option
  Future<CommandResult> uploadAndConvertMap({
    required Map<String, dynamic> mapData,
    required String mapName,
    bool deployToPi = false,
    Duration timeout = const Duration(seconds: 60),
  }) async {
    final commandId = _generateCommandId();
    final command = CommandStatus(
      id: commandId,
      type: 'map_upload',
      parameters: {
        'mapName': mapName,
        'deployToPi': deployToPi,
      },
      startedAt: DateTime.now(),
    );

    _activeCommands[commandId] = command;

    try {
      print(
          ' Uploading and converting map: $mapName - Command ID: $commandId');

      // Send via WebSocket for progress tracking
      _webSocketService.sendMessage({
        'type': 'upload_map',
        'commandId': commandId,
        'data': {
          'mapData': mapData,
          'mapName': mapName,
          'deployToPi': deployToPi,
        }
      });

      return await _waitForCommandCompletion(commandId, timeout);
    } catch (e) {
      _failCommand(commandId, e.toString());
      throw Exception('Failed to upload map: $e');
    }
  }

  /// Convert map from JSON to PGM+YAML
  Future<CommandResult> convertMap({
    required Map<String, dynamic> mapData,
    required String mapName,
    Duration timeout = const Duration(seconds: 30),
  }) async {
    final commandId = _generateCommandId();
    final command = CommandStatus(
      id: commandId,
      type: 'map_conversion',
      parameters: {'mapName': mapName},
      startedAt: DateTime.now(),
    );

    _activeCommands[commandId] = command;

    try {
      print(' Converting map: $mapName - Command ID: $commandId');

      _webSocketService.sendMessage({
        'type': 'convert_map',
        'commandId': commandId,
        'data': {
          'mapData': mapData,
          'mapName': mapName,
        }
      });

      return await _waitForCommandCompletion(commandId, timeout);
    } catch (e) {
      _failCommand(commandId, e.toString());
      throw Exception('Failed to convert map: $e');
    }
  }

  /// Deploy map to Raspberry Pi
  Future<CommandResult> deployMapToPi({
    required String mapName,
    String? piAddress,
    Duration timeout = const Duration(seconds: 45),
  }) async {
    final commandId = _generateCommandId();
    final command = CommandStatus(
      id: commandId,
      type: 'map_deployment',
      parameters: {
        'mapName': mapName,
        'piAddress': piAddress,
      },
      startedAt: DateTime.now(),
    );

    _activeCommands[commandId] = command;

    try {
      print(' Deploying map to Pi: $mapName - Command ID: $commandId');

      _webSocketService.sendMessage({
        'type': 'deploy_map',
        'commandId': commandId,
        'data': {
          'mapName': mapName,
          'piAddress': piAddress,
        }
      });

      return await _waitForCommandCompletion(commandId, timeout);
    } catch (e) {
      _failCommand(commandId, e.toString());
      throw Exception('Failed to deploy map: $e');
    }
  }

  // ==========================================
  // LEGACY HTTP API METHODS (for compatibility)
  // ==========================================

  /// Get available maps via HTTP
  Future<List<Map<String, dynamic>>> getAvailableMaps() async {
    try {
      final response = await http.get(
        Uri.parse('$baseUrl/api/ros2/maps'),
        headers: {'Content-Type': 'application/json'},
      ).timeout(const Duration(seconds: 15));

      final result = _handleResponse(response);
      return List<Map<String, dynamic>>.from(result['data'] ?? []);
    } catch (e) {
      throw Exception('Failed to get available maps: $e');
    }
  }

  /// Get map info via HTTP
  Future<Map<String, dynamic>> getMapInfo(String mapName) async {
    try {
      final response = await http.get(
        Uri.parse('$baseUrl/api/ros2/maps/$mapName'),
        headers: {'Content-Type': 'application/json'},
      ).timeout(const Duration(seconds: 10));

      return _handleResponse(response);
    } catch (e) {
      throw Exception('Failed to get map info: $e');
    }
  }

  /// Delete map via HTTP
  Future<Map<String, dynamic>> deleteMap(String mapName) async {
    try {
      final response = await http.delete(
        Uri.parse('$baseUrl/api/ros2/maps/$mapName'),
        headers: {'Content-Type': 'application/json'},
      ).timeout(const Duration(seconds: 10));

      return _handleResponse(response);
    } catch (e) {
      throw Exception('Failed to delete map: $e');
    }
  }

  // ==========================================
  // COMMAND MANAGEMENT UTILITIES
  // ==========================================

  Future<CommandResult> _waitForCommandCompletion(
      String commandId, Duration timeout) async {
    final startTime = DateTime.now();

    while (DateTime.now().difference(startTime) < timeout) {
      if (!_activeCommands.containsKey(commandId)) {
        // Command completed, check history
        final completedCommand =
            _commandHistory.where((cmd) => cmd.id == commandId).firstOrNull;

        if (completedCommand != null) {
          return completedCommand;
        }
      }

      // Wait a bit before checking again
      await Future.delayed(const Duration(milliseconds: 100));
    }

    // Timeout reached
    _failCommand(
        commandId, 'Command timeout after ${timeout.inSeconds} seconds');
    throw Exception('Command timeout: $commandId');
  }

  void _failCommand(String commandId, String error) {
    if (_activeCommands.containsKey(commandId)) {
      final command = _activeCommands[commandId]!;
      command.status = CommandState.failed;
      command.error = error;
      command.completedAt = DateTime.now();

      // Add to history
      _commandHistory.add(CommandResult.fromCommand(command));

      // Remove from active commands
      _activeCommands.remove(commandId);

      print(' Command failed: $commandId - $error');
    }
  }

  String _generateCommandId() {
    return 'cmd_${DateTime.now().millisecondsSinceEpoch}_${_generateRandomString(6)}';
  }

  String _generateRandomString(int length) {
    const chars = 'abcdefghijklmnopqrstuvwxyz0123456789';
    return List.generate(
        length,
        (index) =>
            chars[(DateTime.now().microsecond + index) % chars.length]).join();
  }

  // ==========================================
  // LEGACY WebSocket METHODS (for compatibility)
  // ==========================================

  void sendSLAMCommand({String? mapName}) {
    _webSocketService.sendMessage({
      'type': 'start_slam',
      'data': {
        'mapName': mapName,
      }
    });
  }

  void sendNavigationCommand({String? mapPath, String? mapName}) {
    _webSocketService.sendMessage({
      'type': 'start_navigation',
      'data': {
        'mapPath': mapPath,
        'mapName': mapName,
      }
    });
  }

  void sendStopCommand(String processName) {
    _webSocketService.sendMessage({
      'type': 'stop_ros2_process',
      'data': {
        'processName': processName,
      }
    });
  }

  void requestROS2Status() {
    _webSocketService.sendMessage({'type': 'get_ros2_status', 'data': {}});
  }

  void uploadMapViaWebSocket({
    required Map<String, dynamic> mapData,
    required String mapName,
  }) {
    _webSocketService.sendMessage({
      'type': 'upload_map',
      'data': {
        'mapData': mapData,
        'mapName': mapName,
      }
    });
  }

  // ==========================================
  // STATUS AND MONITORING
  // ==========================================

  /// Get current command status
  List<CommandStatus> getActiveCommands() {
    return _activeCommands.values.toList();
  }

  /// Get command history
  List<CommandResult> getCommandHistory({int? limit}) {
    final history = List<CommandResult>.from(_commandHistory);
    if (limit != null && history.length > limit) {
      return history.sublist(history.length - limit);
    }
    return history;
  }

  /// Clear command history
  void clearCommandHistory() {
    _commandHistory.clear();
  }

  /// Cancel active command (if possible)
  bool cancelCommand(String commandId) {
    if (_activeCommands.containsKey(commandId)) {
      final command = _activeCommands[commandId]!;
      command.status = CommandState.cancelled;
      command.completedAt = DateTime.now();

      // Add to history
      _commandHistory.add(CommandResult.fromCommand(command));

      // Remove from active commands
      _activeCommands.remove(commandId);

      print(' Command cancelled: $commandId');
      return true;
    }
    return false;
  }

  /// Get service statistics
  Map<String, dynamic> getStatistics() {
    final now = DateTime.now();
    final recentCommands = _commandHistory
        .where((cmd) =>
            now.difference(cmd.completedAt ?? cmd.startedAt).inHours < 24)
        .toList();

    return {
      'activeCommands': _activeCommands.length,
      'totalCommandsToday': recentCommands.length,
      'successfulCommands': recentCommands.where((cmd) => cmd.success).length,
      'failedCommands': recentCommands.where((cmd) => !cmd.success).length,
      'averageExecutionTime': _calculateAverageExecutionTime(recentCommands),
      'commandsByType': _groupCommandsByType(recentCommands),
    };
  }

  double _calculateAverageExecutionTime(List<CommandResult> commands) {
    if (commands.isEmpty) return 0.0;

    final completedCommands = commands.where((cmd) => cmd.completedAt != null);
    if (completedCommands.isEmpty) return 0.0;

    final totalTime = completedCommands
        .map((cmd) => cmd.completedAt!.difference(cmd.startedAt).inMilliseconds)
        .reduce((a, b) => a + b);

    return totalTime / completedCommands.length / 1000.0; // Convert to seconds
  }

  Map<String, int> _groupCommandsByType(List<CommandResult> commands) {
    final grouped = <String, int>{};
    for (final command in commands) {
      grouped[command.type] = (grouped[command.type] ?? 0) + 1;
    }
    return grouped;
  }

  // ==========================================
  // HTTP RESPONSE HANDLING
  // ==========================================

  Map<String, dynamic> _handleResponse(http.Response response) {
    if (response.statusCode >= 200 && response.statusCode < 300) {
      return jsonDecode(response.body) as Map<String, dynamic>;
    } else {
      final errorBody = jsonDecode(response.body) as Map<String, dynamic>;
      throw Exception(errorBody['error'] ?? 'Unknown error occurred');
    }
  }
}

// ==========================================
// DATA MODELS FOR COMMAND TRACKING
// ==========================================

enum CommandState {
  pending,
  active,
  completed,
  failed,
  cancelled,
  timeout,
}

class CommandStatus {
  final String id;
  final String type;
  final Map<String, dynamic> parameters;
  final DateTime startedAt;

  CommandState status;
  DateTime? completedAt;
  DateTime? lastUpdate;
  Map<String, dynamic>? response;
  Map<String, dynamic>? statusData;
  String? error;

  CommandStatus({
    required this.id,
    required this.type,
    required this.parameters,
    required this.startedAt,
    this.status = CommandState.pending,
  });

  Duration? get executionTime {
    final endTime = completedAt ?? DateTime.now();
    return endTime.difference(startedAt);
  }

  bool get isCompleted => status == CommandState.completed;
  bool get isFailed => status == CommandState.failed;
  bool get isActive => status == CommandState.active;

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'type': type,
      'parameters': parameters,
      'startedAt': startedAt.toIso8601String(),
      'status': status.name,
      'completedAt': completedAt?.toIso8601String(),
      'lastUpdate': lastUpdate?.toIso8601String(),
      'response': response,
      'statusData': statusData,
      'error': error,
      'executionTimeMs': executionTime?.inMilliseconds,
    };
  }
}

class CommandResult {
  final String id;
  final String type;
  final Map<String, dynamic> parameters;
  final DateTime startedAt;
  final DateTime? completedAt;
  final bool success;
  final Map<String, dynamic>? response;
  final String? error;

  CommandResult({
    required this.id,
    required this.type,
    required this.parameters,
    required this.startedAt,
    this.completedAt,
    required this.success,
    this.response,
    this.error,
  });

  factory CommandResult.fromCommand(CommandStatus command) {
    return CommandResult(
      id: command.id,
      type: command.type,
      parameters: command.parameters,
      startedAt: command.startedAt,
      completedAt: command.completedAt,
      success: command.status == CommandState.completed,
      response: command.response,
      error: command.error,
    );
  }

  Duration? get executionTime {
    if (completedAt != null) {
      return completedAt!.difference(startedAt);
    }
    return null;
  }

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'type': type,
      'parameters': parameters,
      'startedAt': startedAt.toIso8601String(),
      'completedAt': completedAt?.toIso8601String(),
      'success': success,
      'response': response,
      'error': error,
      'executionTimeMs': executionTime?.inMilliseconds,
    };
  }
}

// ==========================================
// ENHANCED ROS2 STATUS MODEL
// ==========================================

class ROS2Status {
  final Map<String, String> processes;
  final int activeCount;
  final List<String> activeProcesses;
  final DateTime timestamp;
  final Map<String, dynamic>? processDetails;

  ROS2Status({
    required this.processes,
    required this.activeCount,
    required this.activeProcesses,
    required this.timestamp,
    this.processDetails,
  });

  factory ROS2Status.fromJson(Map<String, dynamic> json) {
    return ROS2Status(
      processes: Map<String, String>.from(json['processes'] ?? {}),
      activeCount: json['activeCount'] ?? 0,
      activeProcesses: List<String>.from(json['activeProcesses'] ?? []),
      timestamp:
          DateTime.parse(json['timestamp'] ?? DateTime.now().toIso8601String()),
      processDetails: json['processDetails'],
    );
  }

  bool get isSLAMRunning => processes['slam'] == 'running';
  bool get isNavigationRunning => processes['navigation'] == 'running';
  bool get isRobotControlRunning => processes['robot_control'] == 'running';
  bool get isAnyProcessRunning => activeCount > 0;

  String get statusSummary {
    if (activeCount == 0) return 'All processes stopped';
    if (isSLAMRunning && isNavigationRunning) return 'SLAM + Navigation active';
    if (isSLAMRunning) return 'SLAM active';
    if (isNavigationRunning) return 'Navigation active';
    if (isRobotControlRunning) return 'Robot control active';
    return '$activeCount processes running';
  }

  Map<String, dynamic> toJson() {
    return {
      'processes': processes,
      'activeCount': activeCount,
      'activeProcesses': activeProcesses,
      'timestamp': timestamp.toIso8601String(),
      'processDetails': processDetails,
    };
  }
}

// ==========================================
// MAP INFO MODEL (Enhanced)
// ==========================================

class EnhancedMapInfo {
  final String name;
  final String image;
  final double resolution;
  final List<double> origin;
  final double occupiedThresh;
  final double freeThresh;
  final int negate;
  final bool valid;
  final String? error;
  final Map<String, dynamic>? validation;
  final Map<String, int>? fileSizes;

  EnhancedMapInfo({
    required this.name,
    required this.image,
    required this.resolution,
    required this.origin,
    required this.occupiedThresh,
    required this.freeThresh,
    required this.negate,
    required this.valid,
    this.error,
    this.validation,
    this.fileSizes,
  });

  factory EnhancedMapInfo.fromJson(String name, Map<String, dynamic> json) {
    final info = json['info'] ?? {};
    final validation = json['validation'] ?? {};

    return EnhancedMapInfo(
      name: name,
      image: info['image'] ?? '',
      resolution: (info['resolution'] ?? 0.05).toDouble(),
      origin: List<double>.from(info['origin'] ?? [-10.0, -10.0, 0.0]),
      occupiedThresh: (info['occupiedThresh'] ?? 0.65).toDouble(),
      freeThresh: (info['freeThresh'] ?? 0.196).toDouble(),
      negate: info['negate'] ?? 0,
      valid: validation['valid'] ?? false,
      error: validation['error'],
      validation: validation,
      fileSizes: Map<String, int>.from(validation['file_sizes'] ?? {}),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'name': name,
      'image': image,
      'resolution': resolution,
      'origin': origin,
      'occupiedThresh': occupiedThresh,
      'freeThresh': freeThresh,
      'negate': negate,
      'valid': valid,
      'error': error,
      'validation': validation,
      'fileSizes': fileSizes,
    };
  }
}
