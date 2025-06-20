// models/odom.dart
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
      x: (json['x'] as num).toDouble(),
      y: (json['y'] as num).toDouble(),
      z: (json['z'] as num).toDouble(),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'x': x,
      'y': y,
      'z': z,
    };
  }

  @override
  String toString() => 'Position(x: $x, y: $y, z: $z)';
}

class Orientation {
  final double x;
  final double y;
  final double z;
  final double w;

  Orientation({
    required this.x,
    required this.y,
    required this.z,
    required this.w,
  });

  factory Orientation.fromJson(Map<String, dynamic> json) {
    return Orientation(
      x: (json['x'] as num).toDouble(),
      y: (json['y'] as num).toDouble(),
      z: (json['z'] as num).toDouble(),
      w: (json['w'] as num).toDouble(),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'x': x,
      'y': y,
      'z': z,
      'w': w,
    };
  }

  // Convert quaternion to yaw angle (radians)
  double get yaw {
    return 2 * atan2(z, w);
  }

  // Convert quaternion to yaw angle (degrees)
  double get yawDegrees {
    return yaw * 180 / pi;
  }

  @override
  String toString() => 'Orientation(x: $x, y: $y, z: $z, w: $w)';
}

class Velocity {
  final double x;
  final double y;
  final double z;

  Velocity({
    required this.x,
    required this.y,
    required this.z,
  });

  factory Velocity.fromJson(Map<String, dynamic> json) {
    return Velocity(
      x: (json['x'] as num).toDouble(),
      y: (json['y'] as num).toDouble(),
      z: (json['z'] as num).toDouble(),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'x': x,
      'y': y,
      'z': z,
    };
  }

  @override
  String toString() => 'Velocity(x: $x, y: $y, z: $z)';
}

class OdometryData {
  final String deviceId;
  final DateTime timestamp;
  final Position position;
  final Orientation orientation;
  final Velocity linearVelocity;
  final Velocity angularVelocity;

  OdometryData({
    required this.deviceId,
    required this.timestamp,
    required this.position,
    required this.orientation,
    required this.linearVelocity,
    required this.angularVelocity,
  });

  factory OdometryData.fromJson(Map<String, dynamic> json) {
    return OdometryData(
      deviceId: json['deviceId'],
      timestamp: DateTime.parse(json['timestamp']),
      position: Position.fromJson(json['position']),
      orientation: Orientation.fromJson(json['orientation']),
      linearVelocity: Velocity.fromJson(json['linearVelocity']),
      angularVelocity: Velocity.fromJson(json['angularVelocity']),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'deviceId': deviceId,
      'timestamp': timestamp.toIso8601String(),
      'position': position.toJson(),
      'orientation': orientation.toJson(),
      'linearVelocity': linearVelocity.toJson(),
      'angularVelocity': angularVelocity.toJson(),
    };
  }

  @override
  String toString() => 'OdometryData(deviceId: $deviceId, pos: $position, vel: $linearVelocity)';
}
