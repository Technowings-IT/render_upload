//map_data.dart - Updated with Better Error Handling
import 'dart:math';
import 'odom.dart';

class MapInfo {
  final double resolution;
  final int width;
  final int height;
  final Position origin;
  final Orientation originOrientation;

  MapInfo({
    required this.resolution,
    required this.width,
    required this.height,
    required this.origin,
    required this.originOrientation,
  });

  factory MapInfo.fromJson(Map<String, dynamic> json) {
    try {
      // Handle different possible origin structures from backend
      Position origin;
      Orientation originOrientation;
      
      if (json['origin'] != null) {
        final originData = json['origin'];
        
        // Check if origin has nested position/orientation structure
        if (originData['position'] != null && originData['orientation'] != null) {
          origin = Position.fromJson(_safeMap(originData['position']));
          originOrientation = Orientation.fromJson(_safeMap(originData['orientation']));
        } else {
          // Direct position data in origin
          origin = Position.fromJson(_safeMap(originData));
          originOrientation = Orientation.fromJson(_safeMap(json['originOrientation']));
        }
      } else {
        // Fallback to default values
        origin = Position(x: -25.0, y: -25.0, z: 0.0);
        originOrientation = Orientation(x: 0, y: 0, z: 0, w: 1);
      }

      return MapInfo(
        resolution: _safeDouble(json['resolution']) ?? 0.05,
        width: _safeInt(json['width']) ?? 1000,
        height: _safeInt(json['height']) ?? 1000,
        origin: origin,
        originOrientation: originOrientation,
      );
    } catch (e) {
      print(' Error parsing MapInfo: $e');
      print(' JSON data: $json');
      
      // Return safe defaults
      return MapInfo(
        resolution: 0.05,
        width: 1000,
        height: 1000,
        origin: Position(x: -25.0, y: -25.0, z: 0.0),
        originOrientation: Orientation(x: 0, y: 0, z: 0, w: 1),
      );
    }
  }

  static Map<String, dynamic> _safeMap(dynamic value) {
    if (value == null) return <String, dynamic>{};
    if (value is Map<String, dynamic>) return value;
    if (value is Map) return Map<String, dynamic>.from(value);
    return <String, dynamic>{};
  }

  static double? _safeDouble(dynamic value) {
    if (value == null) return null;
    if (value is double) return value;
    if (value is int) return value.toDouble();
    if (value is num) return value.toDouble();
    if (value is String) return double.tryParse(value);
    return null;
  }

  static int? _safeInt(dynamic value) {
    if (value == null) return null;
    if (value is int) return value;
    if (value is double) return value.toInt();
    if (value is num) return value.toInt();
    if (value is String) return int.tryParse(value);
    return null;
  }

  Map<String, dynamic> toJson() {
    return {
      'resolution': resolution,
      'width': width,
      'height': height,
      'origin': {
        'position': origin.toJson(),
        'orientation': originOrientation.toJson(),
      },
      'originOrientation': originOrientation.toJson(), // Backup for compatibility
    };
  }
}

class MapShape {
  final String id;
  final String type; // 'pickup', 'drop', 'charging', 'obstacle', 'waypoint'
  final String name;
  final List<Position> points;
  final Map<String, String> sides; // 'left', 'right', 'front', 'back'
  final String color;
  final DateTime createdAt;

  MapShape({
    required this.id,
    required this.type,
    required this.name,
    required this.points,
    required this.sides,
    required this.color,
    required this.createdAt,
  });

  factory MapShape.fromJson(Map<String, dynamic> json) {
    try {
      return MapShape(
        id: _safeString(json['id']) ?? DateTime.now().millisecondsSinceEpoch.toString(),
        type: _safeString(json['type']) ?? 'waypoint',
        name: _safeString(json['name']) ?? 'Unnamed Shape',
        points: _parsePoints(json['points']),
        sides: _parseSides(json['sides']),
        color: _safeString(json['color']) ?? 'FF0000FF',
        createdAt: _parseDateTime(json['createdAt']),
      );
    } catch (e) {
      print(' Error parsing MapShape: $e');
      print(' JSON data: $json');
      rethrow;
    }
  }

  static String _safeString(dynamic value) {
    if (value == null) return '';
    return value.toString();
  }

  static List<Position> _parsePoints(dynamic points) {
    if (points == null) return [];
    if (points is! List) return [];
    
    final List<Position> result = [];
    for (final point in points) {
      try {
        if (point is Map<String, dynamic>) {
          result.add(Position.fromJson(point));
        } else if (point is Map) {
          result.add(Position.fromJson(Map<String, dynamic>.from(point)));
        }
      } catch (e) {
        print(' Error parsing point: $e');
        // Skip invalid points
      }
    }
    return result;
  }

  static Map<String, String> _parseSides(dynamic sides) {
    final defaultSides = {'left': '', 'right': '', 'front': '', 'back': ''};
    
    if (sides == null) return defaultSides;
    if (sides is! Map) return defaultSides;
    
    final result = Map<String, String>.from(defaultSides);
    
    sides.forEach((key, value) {
      if (key != null && value != null) {
        result[key.toString()] = value.toString();
      }
    });
    
    return result;
  }

  static DateTime _parseDateTime(dynamic dateTime) {
    if (dateTime == null) return DateTime.now();
    
    if (dateTime is String) {
      try {
        return DateTime.parse(dateTime);
      } catch (e) {
        print(' Error parsing datetime: $dateTime');
        return DateTime.now();
      }
    }
    
    if (dateTime is int) {
      return DateTime.fromMillisecondsSinceEpoch(dateTime);
    }
    
    return DateTime.now();
  }

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'type': type,
      'name': name,
      'points': points.map((p) => p.toJson()).toList(),
      'sides': sides,
      'color': color,
      'createdAt': createdAt.toIso8601String(),
    };
  }

  // Get center point of the shape
  Position get center {
    if (points.isEmpty) return Position(x: 0, y: 0, z: 0);
    
    double sumX = points.fold(0, (sum, p) => sum + p.x);
    double sumY = points.fold(0, (sum, p) => sum + p.y);
    
    return Position(
      x: sumX / points.length,
      y: sumY / points.length,
      z: 0,
    );
  }

  // Check if a point is inside this shape (improved point-in-polygon algorithm)
  bool containsPoint(Position point) {
    if (points.length < 3) {
      // For lines or single points, use distance check
      if (points.length == 2) {
        return _distanceToLineSegment(point, points[0], points[1]) < 0.5;
      } else if (points.length == 1) {
        return _distance(point, points[0]) < 0.5;
      }
      return false;
    }
    
    // Ray casting algorithm for polygon
    int crossings = 0;
    for (int i = 0; i < points.length; i++) {
      final p1 = points[i];
      final p2 = points[(i + 1) % points.length];

      if (((p1.y <= point.y) && (point.y < p2.y)) ||
          ((p2.y <= point.y) && (point.y < p1.y))) {
        if (point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x) {
          crossings++;
        }
      }
    }

    return crossings % 2 == 1;
  }

  double _distance(Position p1, Position p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
  }

  double _distanceToLineSegment(Position point, Position lineStart, Position lineEnd) {
    final A = point.x - lineStart.x;
    final B = point.y - lineStart.y;
    final C = lineEnd.x - lineStart.x;
    final D = lineEnd.y - lineStart.y;

    final dot = A * C + B * D;
    final lenSq = C * C + D * D;
    
    if (lenSq == 0) return _distance(point, lineStart);
    
    final param = dot / lenSq;

    double xx, yy;
    if (param < 0) {
      xx = lineStart.x;
      yy = lineStart.y;
    } else if (param > 1) {
      xx = lineEnd.x;
      yy = lineEnd.y;
    } else {
      xx = lineStart.x + param * C;
      yy = lineStart.y + param * D;
    }

    final dx = point.x - xx;
    final dy = point.y - yy;
    return sqrt(dx * dx + dy * dy);
  }
}

class MapData {
  final String deviceId;
  final DateTime timestamp;
  final MapInfo info;
  final List<int> occupancyData;
  final List<MapShape> shapes;
  final int version;

  MapData({
    required this.deviceId,
    required this.timestamp,
    required this.info,
    required this.occupancyData,
    required this.shapes,
    required this.version,
  });

  factory MapData.fromJson(Map<String, dynamic> json) {
    try {
      print('️ Parsing MapData from JSON...');
      
      return MapData(
        deviceId: _safeString(json['deviceId']) ?? 'unknown',
        timestamp: _parseDateTime(json['timestamp']),
        info: MapInfo.fromJson(_safeMap(json['info'])),
        occupancyData: _parseOccupancyData(json),
        shapes: _parseShapes(json['shapes']),
        version: _safeInt(json['version']) ?? 1,
      );
    } catch (e) {
      print(' Error parsing MapData: $e');
      print(' JSON keys: ${json.keys.toList()}');
      rethrow;
    }
  }

  static String _safeString(dynamic value) {
    if (value == null) return '';
    return value.toString();
  }

  static int _safeInt(dynamic value) {
    if (value == null) return 0;
    if (value is int) return value;
    if (value is double) return value.toInt();
    if (value is num) return value.toInt();
    if (value is String) return int.tryParse(value) ?? 0;
    return 0;
  }

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
        print(' Error parsing datetime: $dateTime');
        return DateTime.now();
      }
    }
    
    if (dateTime is int) {
      return DateTime.fromMillisecondsSinceEpoch(dateTime);
    }
    
    return DateTime.now();
  }

  static List<int> _parseOccupancyData(Map<String, dynamic> json) {
    // Try different possible field names for occupancy data
    dynamic data = json['occupancyData'] ?? json['data'] ?? json['gridData'];
    
    if (data == null) {
      print('️ No occupancy data found, creating empty grid');
      return [];
    }
    
    if (data is List<int>) return data;
    
    if (data is List) {
      try {
        return data.map((e) => (e as num?)?.toInt() ?? -1).toList();
      } catch (e) {
        print(' Error converting occupancy data: $e');
        return [];
      }
    }
    
    return [];
  }

  static List<MapShape> _parseShapes(dynamic shapes) {
    if (shapes == null) return [];
    if (shapes is! List) return [];
    
    final List<MapShape> result = [];
    for (final shape in shapes) {
      try {
        if (shape is Map<String, dynamic>) {
          result.add(MapShape.fromJson(shape));
        } else if (shape is Map) {
          result.add(MapShape.fromJson(Map<String, dynamic>.from(shape)));
        }
      } catch (e) {
        print(' Error parsing shape: $e');
        // Skip invalid shapes
      }
    }
    
    print(' Parsed ${result.length} shapes successfully');
    return result;
  }

  Map<String, dynamic> toJson() {
    return {
      'deviceId': deviceId,
      'timestamp': timestamp.toIso8601String(),
      'info': info.toJson(),
      'data': occupancyData, // Keep original field name
      'occupancyData': occupancyData, // Also include alternative name
      'shapes': shapes.map((s) => s.toJson()).toList(),
      'version': version,
    };
  }

  // Convert world coordinates to map pixel coordinates
  Position worldToMap(Position worldPos) {
    final mapX = (worldPos.x - info.origin.x) / info.resolution;
    final mapY = (worldPos.y - info.origin.y) / info.resolution;
    
    return Position(x: mapX, y: mapY, z: 0);
  }

  // Convert map pixel coordinates to world coordinates
  Position mapToWorld(Position mapPos) {
    final worldX = mapPos.x * info.resolution + info.origin.x;
    final worldY = mapPos.y * info.resolution + info.origin.y;
    
    return Position(x: worldX, y: worldY, z: 0);
  }

  // Get occupancy value at specific coordinates
  int getOccupancyAt(int x, int y) {
    if (x < 0 || x >= info.width || y < 0 || y >= info.height) {
      return -1; // Unknown
    }
    
    final index = y * info.width + x;
    if (index >= occupancyData.length) return -1;
    
    return occupancyData[index];
  }

  // Add a new shape to the map
  MapData addShape(MapShape shape) {
    final newShapes = List<MapShape>.from(shapes);
    newShapes.add(shape);
    
    return MapData(
      deviceId: deviceId,
      timestamp: DateTime.now(),
      info: info,
      occupancyData: occupancyData,
      shapes: newShapes,
      version: version + 1,
    );
  }

  // Remove a shape from the map
  MapData removeShape(String shapeId) {
    final newShapes = shapes.where((s) => s.id != shapeId).toList();
    
    return MapData(
      deviceId: deviceId,
      timestamp: DateTime.now(),
      info: info,
      occupancyData: occupancyData,
      shapes: newShapes,
      version: version + 1,
    );
  }

  // Update a shape in the map
  MapData updateShape(MapShape updatedShape) {
    final newShapes = shapes.map((shape) {
      return shape.id == updatedShape.id ? updatedShape : shape;
    }).toList();
    
    return MapData(
      deviceId: deviceId,
      timestamp: DateTime.now(),
      info: info,
      occupancyData: occupancyData,
      shapes: newShapes,
      version: version + 1,
    );
  }

  // Get shapes by type
  List<MapShape> getShapesByType(String type) {
    return shapes.where((s) => s.type == type).toList();
  }

  @override
  String toString() => 'MapData(deviceId: $deviceId, size: ${info.width}x${info.height}, shapes: ${shapes.length})';
}