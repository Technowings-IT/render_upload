// models/odom.dart - Enhanced odometry and position models
import 'dart:math';

class Position {
  final double x;
  final double y;
  final double z;

  Position({
    required this.x,
    required this.y,
    required this.z,
  });

  factory Position.fromJson(Map<String, dynamic> json) {
    return Position(
      x: _safeDouble(json['x']) ?? 0.0,
      y: _safeDouble(json['y']) ?? 0.0,
      z: _safeDouble(json['z']) ?? 0.0,
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'x': x,
      'y': y,
      'z': z,
    };
  }

  // Calculate distance to another position
  double distanceTo(Position other) {
    return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2) + pow(z - other.z, 2));
  }

  // Calculate 2D distance (ignoring z)
  double distance2DTo(Position other) {
    return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
  }

  Position operator +(Position other) {
    return Position(x: x + other.x, y: y + other.y, z: z + other.z);
  }

  Position operator -(Position other) {
    return Position(x: x - other.x, y: y - other.y, z: z - other.z);
  }

  Position operator *(double scalar) {
    return Position(x: x * scalar, y: y * scalar, z: z * scalar);
  }

  @override
  String toString() => 'Position(x: ${x.toStringAsFixed(3)}, y: ${y.toStringAsFixed(3)}, z: ${z.toStringAsFixed(3)})';

  @override
  bool operator ==(Object other) {
    return other is Position && 
           (x - other.x).abs() < 0.001 && 
           (y - other.y).abs() < 0.001 && 
           (z - other.z).abs() < 0.001;
  }

  @override
  int get hashCode => Object.hash(x, y, z);

  static double? _safeDouble(dynamic value) {
    if (value == null) return null;
    if (value is double) return value;
    if (value is int) return value.toDouble();
    if (value is num) return value.toDouble();
    if (value is String) return double.tryParse(value);
    return null;
  }
}

class Orientation {
  final double x;
  final double y;
  final double z;
  final double w;
  final double yaw; // Cached yaw angle in radians

  Orientation({
    required this.x,
    required this.y,
    required this.z,
    required this.w,
    double? yaw,
  }) : yaw = yaw ?? _calculateYaw(x, y, z, w);

  factory Orientation.fromJson(Map<String, dynamic> json) {
    final x = _safeDouble(json['x']) ?? 0.0;
    final y = _safeDouble(json['y']) ?? 0.0;
    final z = _safeDouble(json['z']) ?? 0.0;
    final w = _safeDouble(json['w']) ?? 1.0;
    
    // Check if yaw is provided directly
    final yaw = _safeDouble(json['yaw']);
    
    return Orientation(
      x: x,
      y: y,
      z: z,
      w: w,
      yaw: yaw,
    );
  }

  factory Orientation.fromYaw(double yawRadians) {
    return Orientation(
      x: 0.0,
      y: 0.0,
      z: sin(yawRadians / 2),
      w: cos(yawRadians / 2),
      yaw: yawRadians,
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'x': x,
      'y': y,
      'z': z,
      'w': w,
      'yaw': yaw,
    };
  }

  // Get yaw angle in degrees
  double get yawDegrees => yaw * 180.0 / pi;

  // Get normalized yaw (0 to 2π)
  double get yawNormalized {
    double normalizedYaw = yaw % (2 * pi);
    if (normalizedYaw < 0) normalizedYaw += 2 * pi;
    return normalizedYaw;
  }

  // Get yaw in degrees (0 to 360)
  double get yawDegreesNormalized {
    double normalizedDegrees = yawDegrees % 360;
    if (normalizedDegrees < 0) normalizedDegrees += 360;
    return normalizedDegrees;
  }

  // Convert to rotation matrix (for advanced calculations)
  List<List<double>> get rotationMatrix {
    final double xx = x * x;
    final double xy = x * y;
    final double xz = x * z;
    final double xw = x * w;
    final double yy = y * y;
    final double yz = y * z;
    final double yw = y * w;
    final double zz = z * z;
    final double zw = z * w;

    return [
      [1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw)],
      [2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw)],
      [2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy)],
    ];
  }

  // Get forward vector (direction robot is facing)
  Position get forwardVector {
    return Position(
      x: cos(yaw),
      y: sin(yaw),
      z: 0.0,
    );
  }

  // Get right vector (robot's right side)
  Position get rightVector {
    return Position(
      x: -sin(yaw),
      y: cos(yaw),
      z: 0.0,
    );
  }

  static double _calculateYaw(double x, double y, double z, double w) {
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  static double? _safeDouble(dynamic value) {
    if (value == null) return null;
    if (value is double) return value;
    if (value is int) return value.toDouble();
    if (value is num) return value.toDouble();
    if (value is String) return double.tryParse(value);
    return null;
  }

  @override
  String toString() => 'Orientation(yaw: ${yawDegrees.toStringAsFixed(1)}°)';
}

class Velocity {
  final Position linear;
  final Position angular;

  Velocity({
    required this.linear,
    required this.angular,
  });

  factory Velocity.fromJson(Map<String, dynamic> json) {
    return Velocity(
      linear: Position.fromJson(_safeMap(json['linear'])),
      angular: Position.fromJson(_safeMap(json['angular'])),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'linear': linear.toJson(),
      'angular': angular.toJson(),
    };
  }

  // Get speed (magnitude of linear velocity)
  double get speed {
    return sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z);
  }

  // Get 2D speed (ignoring z component)
  double get speed2D {
    return sqrt(linear.x * linear.x + linear.y * linear.y);
  }

  // Get angular speed (magnitude of angular velocity)
  double get angularSpeed {
    return sqrt(angular.x * angular.x + angular.y * angular.y + angular.z * angular.z);
  }

  // Check if robot is moving
  bool get isMoving {
    return speed2D > 0.01 || angularSpeed > 0.01;
  }

  // Check if robot is moving forward
  bool get isMovingForward => linear.x > 0.05;

  // Check if robot is moving backward
  bool get isMovingBackward => linear.x < -0.05;

  // Check if robot is rotating
  bool get isRotating => angular.z.abs() > 0.05;

  static Map<String, dynamic> _safeMap(dynamic value) {
    if (value == null) return {'x': 0.0, 'y': 0.0, 'z': 0.0};
    if (value is Map<String, dynamic>) return value;
    if (value is Map) return Map<String, dynamic>.from(value);
    return {'x': 0.0, 'y': 0.0, 'z': 0.0};
  }

  @override
  String toString() => 'Velocity(linear: ${speed2D.toStringAsFixed(2)} m/s, angular: ${angular.z.toStringAsFixed(2)} rad/s)';
}

class OdometryData {
  final Position position;
  final Orientation orientation;
  final Velocity? velocity;
  final String timestamp;
  final String frameId;
  final List<double>? poseCovariance;
  final List<double>? twistCovariance;

  OdometryData({
    required this.position,
    required this.orientation,
    this.velocity,
    required this.timestamp,
    this.frameId = 'odom',
    this.poseCovariance,
    this.twistCovariance,
  });

  factory OdometryData.fromJson(Map<String, dynamic> json) {
    try {
      return OdometryData(
        position: Position.fromJson(_safeMap(json['position'])),
        orientation: Orientation.fromJson(_safeMap(json['orientation'])),
        velocity: json['velocity'] != null ? Velocity.fromJson(_safeMap(json['velocity'])) : null,
        timestamp: json['timestamp']?.toString() ?? DateTime.now().toIso8601String(),
        frameId: json['frame_id']?.toString() ?? json['frameId']?.toString() ?? 'odom',
        poseCovariance: _safeDoubleList(json['poseCovariance'] ?? json['pose_covariance']),
        twistCovariance: _safeDoubleList(json['twistCovariance'] ?? json['twist_covariance']),
      );
    } catch (e) {
      print('❌ Error parsing OdometryData: $e');
      print('📋 JSON data: $json');
      
      // Return safe defaults
      return OdometryData(
        position: Position(x: 0, y: 0, z: 0),
        orientation: Orientation(x: 0, y: 0, z: 0, w: 1),
        timestamp: DateTime.now().toIso8601String(),
      );
    }
  }

  Map<String, dynamic> toJson() {
    return {
      'position': position.toJson(),
      'orientation': orientation.toJson(),
      'velocity': velocity?.toJson(),
      'timestamp': timestamp,
      'frame_id': frameId,
      'poseCovariance': poseCovariance,
      'twistCovariance': twistCovariance,
    };
  }

  // Get robot's current heading direction as a unit vector
  Position get headingVector => orientation.forwardVector;

  // Predict future position based on current velocity
  Position predictPosition(double deltaTime) {
    if (velocity == null) return position;
    
    return Position(
      x: position.x + velocity!.linear.x * deltaTime,
      y: position.y + velocity!.linear.y * deltaTime,
      z: position.z + velocity!.linear.z * deltaTime,
    );
  }

  // Predict future orientation based on current angular velocity
  Orientation predictOrientation(double deltaTime) {
    if (velocity == null) return orientation;
    
    final newYaw = orientation.yaw + velocity!.angular.z * deltaTime;
    return Orientation.fromYaw(newYaw);
  }

  // Check if odometry data is recent
  bool get isRecent {
    try {
      final dataTime = DateTime.parse(timestamp);
      final now = DateTime.now();
      return now.difference(dataTime).inSeconds < 5;
    } catch (e) {
      return false;
    }
  }

  // Get age of the data in seconds
  double get ageInSeconds {
    try {
      final dataTime = DateTime.parse(timestamp);
      final now = DateTime.now();
      return now.difference(dataTime).inMilliseconds / 1000.0;
    } catch (e) {
      return double.infinity;
    }
  }

  static Map<String, dynamic> _safeMap(dynamic value) {
    if (value == null) return <String, dynamic>{};
    if (value is Map<String, dynamic>) return value;
    if (value is Map) return Map<String, dynamic>.from(value);
    return <String, dynamic>{};
  }

  static List<double>? _safeDoubleList(dynamic value) {
    if (value == null) return null;
    if (value is List<double>) return value;
    if (value is List) {
      try {
        return value.map((e) => (e as num?)?.toDouble() ?? 0.0).toList();
      } catch (e) {
        return null;
      }
    }
    return null;
  }

  @override
  String toString() => 'OdometryData(pos: $position, orient: $orientation, vel: $velocity)';
}

// Robot state aggregation class
class RobotState {
  final OdometryData odometry;
  final bool mappingActive;
  final bool isConnected;
  final DateTime lastUpdate;
  final Map<String, dynamic> additionalData;

  RobotState({
    required this.odometry,
    this.mappingActive = false,
    this.isConnected = false,
    required this.lastUpdate,
    this.additionalData = const {},
  });

  factory RobotState.fromJson(Map<String, dynamic> json) {
    return RobotState(
      odometry: OdometryData.fromJson(_safeMap(json['odometry'])),
      mappingActive: json['mappingActive'] as bool? ?? false,
      isConnected: json['isConnected'] as bool? ?? false,
      lastUpdate: _parseDateTime(json['lastUpdate']),
      additionalData: _safeMap(json['additionalData']),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'odometry': odometry.toJson(),
      'mappingActive': mappingActive,
      'isConnected': isConnected,
      'lastUpdate': lastUpdate.toIso8601String(),
      'additionalData': additionalData,
    };
  }

  // Convenience getters
  Position get position => odometry.position;
  Orientation get orientation => odometry.orientation;
  Velocity? get velocity => odometry.velocity;
  double get speed => velocity?.speed2D ?? 0.0;
  bool get isMoving => velocity?.isMoving ?? false;
  
  // Status checks
  bool get isDataRecent => DateTime.now().difference(lastUpdate).inSeconds < 10;
  bool get isHealthy => isConnected && isDataRecent;

  static Map<String, dynamic> _safeMap(dynamic value) {
    if (value == null) return <String, dynamic>{};
    if (value is Map<String, dynamic>) return value;
    if (value is Map) return Map<String, dynamic>.from(value);
    return <String, dynamic>{};
  }

  static DateTime _parseDateTime(dynamic dateTime) {
    if (dateTime == null) return DateTime.now();
    
    if (dateTime is String) {
      try {
        return DateTime.parse(dateTime);
      } catch (e) {
        return DateTime.now();
      }
    }
    
    if (dateTime is DateTime) return dateTime;
    
    return DateTime.now();
  }

  @override
  String toString() => 'RobotState(pos: $position, connected: $isConnected, mapping: $mappingActive)';
}