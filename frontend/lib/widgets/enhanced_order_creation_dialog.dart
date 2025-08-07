// widgets/enhanced_order_creation_dialog.dart - Perfect Order Creation Dialog
import 'package:flutter/material.dart';
import 'dart:math';
import '../models/map_data.dart';
import '../services/api_service.dart';

class EnhancedOrderCreationDialog extends StatefulWidget {
  final String deviceId;
  final MapData mapData;
  final Function(Map<String, dynamic>) onOrderCreated;

  const EnhancedOrderCreationDialog({
    Key? key,
    required this.deviceId,
    required this.mapData,
    required this.onOrderCreated,
  }) : super(key: key);

  @override
  State<EnhancedOrderCreationDialog> createState() =>
      _EnhancedOrderCreationDialogState();
}

class _EnhancedOrderCreationDialogState
    extends State<EnhancedOrderCreationDialog> {
  final ApiService _apiService = ApiService();

  // Form controllers
  final TextEditingController _AMRSerialController =
      TextEditingController(text: '1');

  // Selection state
  MapShape? _selectedFromStation;
  MapShape? _selectedToStation;
  List<MapShape> _availableStations = [];
  bool _isCreating = false;

  // Map visualization
  double _mapScale = 1.0;
  Offset _mapOffset = Offset.zero;

  @override
  void initState() {
    super.initState();
    _loadAvailableStations();
  }

  void _loadAvailableStations() {
    // Get all stations from map shapes
    _availableStations = widget.mapData.shapes
        .where((shape) => [
              'pickup',
              'drop',
              'charging',
              'waypoint',
              'pick_up',
              'charge_1',
              'charge_2'
            ].contains(shape.type))
        .toList();

    // Sort by name for better UX
    _availableStations.sort((a, b) => a.name.compareTo(b.name));

    print('📍 Loaded ${_availableStations.length} stations');
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return Dialog(
      backgroundColor: Colors.transparent,
      insetPadding: EdgeInsets.all(8),
      child: Container(
        width: MediaQuery.of(context).size.width * 0.95,
        height: MediaQuery.of(context).size.height * 0.85,
        decoration: BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.circular(16),
          boxShadow: [
            BoxShadow(
              color: Colors.black.withOpacity(0.3),
              blurRadius: 20,
              spreadRadius: 5,
            ),
          ],
        ),
        child: Column(
          children: [
            _buildHeader(),
            Expanded(
              child: Row(
                children: [
                  // Map Section
                  Expanded(
                    flex: 2,
                    child: _buildMapSection(),
                  ),

                  // Control Panel Section
                  Container(
                    width: 350,
                    decoration: BoxDecoration(
                      color: Colors.grey.shade50,
                      border:
                          Border(left: BorderSide(color: Colors.grey.shade300)),
                    ),
                    child: _buildControlPanel(),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildHeader() {
    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.blue.shade600, Colors.blue.shade800],
        ),
        borderRadius: BorderRadius.vertical(top: Radius.circular(16)),
      ),
      child: Row(
        children: [
          // Back Button
          IconButton(
            onPressed: () => Navigator.of(context).pop(),
            icon: Icon(Icons.arrow_back, color: Colors.white, size: 24),
            tooltip: 'Back',
          ),
          SizedBox(width: 8),
          Icon(Icons.add_task, color: Colors.white, size: 28),
          SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Create Order',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 20,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                Text(
                  'Select stations and create AMR order',
                  style: TextStyle(
                    color: Colors.white.withOpacity(0.9),
                    fontSize: 14,
                  ),
                ),
              ],
            ),
          ),
          IconButton(
            onPressed: () => Navigator.of(context).pop(),
            icon: Icon(Icons.close, color: Colors.white, size: 28),
          ),
        ],
      ),
    );
  }

  Widget _buildMapSection() {
    return Container(
      padding: EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Map header with info
          Container(
            padding: EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Colors.blue.shade50,
              borderRadius: BorderRadius.circular(8),
              border: Border.all(color: Colors.blue.shade200),
            ),
            child: Row(
              children: [
                Icon(Icons.map, color: Colors.blue.shade700),
                SizedBox(width: 8),
                Text(
                  'Map: ${widget.mapData.info.width}×${widget.mapData.info.height} • ${_availableStations.length} Stations',
                  style: TextStyle(
                    color: Colors.blue.shade700,
                    fontWeight: FontWeight.w600,
                  ),
                ),
                Spacer(),
                Text(
                  'Device: ${widget.deviceId}',
                  style: TextStyle(
                    color: Colors.blue.shade600,
                    fontSize: 12,
                  ),
                ),
              ],
            ),
          ),

          SizedBox(height: 16),

          // Map canvas
          Expanded(
            child: Container(
              width: double.infinity,
              decoration: BoxDecoration(
                border: Border.all(color: Colors.grey.shade300),
                borderRadius: BorderRadius.circular(8),
              ),
              child: ClipRRect(
                borderRadius: BorderRadius.circular(8),
                child: _buildInteractiveMap(),
              ),
            ),
          ),

          // Map controls
          SizedBox(height: 8),
          _buildMapControls(),
        ],
      ),
    );
  }

  Widget _buildInteractiveMap() {
    return LayoutBuilder(
      builder: (context, constraints) {
        return GestureDetector(
          onScaleStart: (details) {},
          onScaleUpdate: (details) {
            setState(() {
              _mapScale = (_mapScale * details.scale).clamp(0.5, 3.0);
              _mapOffset += details.focalPointDelta;
            });
          },
          child: CustomPaint(
            size: Size(constraints.maxWidth, constraints.maxHeight),
            painter: OrderCreationMapPainter(
              mapData: widget.mapData,
              availableStations: _availableStations,
              selectedFromStation: _selectedFromStation,
              selectedToStation: _selectedToStation,
              mapScale: _mapScale,
              mapOffset: _mapOffset,
              canvasSize: Size(constraints.maxWidth, constraints.maxHeight),
            ),
          ),
        );
      },
    );
  }

  Widget _buildMapControls() {
    return Row(
      children: [
        Text('Zoom: ${(_mapScale * 100).toInt()}%',
            style: TextStyle(fontSize: 12)),
        Expanded(
          child: Slider(
            value: _mapScale,
            min: 0.5,
            max: 3.0,
            divisions: 25,
            onChanged: (value) => setState(() => _mapScale = value),
          ),
        ),
        IconButton(
          icon: Icon(Icons.center_focus_strong, size: 20),
          onPressed: () => setState(() {
            _mapScale = 1.0;
            _mapOffset = Offset.zero;
          }),
          tooltip: 'Reset View',
        ),
      ],
    );
  }

  Widget _buildControlPanel() {
    return Padding(
      padding: EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // AMR Serial Input
          _buildSectionHeader('AMR Configuration', Icons.smart_toy),
          SizedBox(height: 12),
          TextField(
            controller: _AMRSerialController,
            decoration: InputDecoration(
              labelText: 'AMR Serial',
              hintText: 'Enter AMR number',
              border: OutlineInputBorder(),
              prefixIcon: Icon(Icons.tag),
            ),
            keyboardType: TextInputType.number,
          ),

          SizedBox(height: 24),

          // From Station Selection
          _buildSectionHeader('From Station', Icons.start),
          SizedBox(height: 12),
          _buildStationDropdown(
            'Select pickup/start station',
            _selectedFromStation,
            (station) => setState(() => _selectedFromStation = station),
          ),

          SizedBox(height: 24),

          // To Station Selection
          _buildSectionHeader('To Station', Icons.flag),
          SizedBox(height: 12),
          _buildStationDropdown(
            'Select destination station',
            _selectedToStation,
            (station) => setState(() => _selectedToStation = station),
          ),

          SizedBox(height: 24),

          // Route Preview
          if (_selectedFromStation != null && _selectedToStation != null)
            _buildRoutePreview(),

          Spacer(),

          // Back and Create Order Buttons
          Row(
            children: [
              // Back Button
              Expanded(
                child: SizedBox(
                  height: 50,
                  child: OutlinedButton.icon(
                    onPressed: () => Navigator.of(context).pop(),
                    icon: Icon(Icons.arrow_back),
                    label: Text(
                      'Back',
                      style: TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                    style: OutlinedButton.styleFrom(
                      foregroundColor: Colors.grey.shade700,
                      side: BorderSide(color: Colors.grey.shade400),
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12),
                      ),
                    ),
                  ),
                ),
              ),
              SizedBox(width: 12),
              // Create Order Button
              Expanded(
                flex: 2,
                child: _buildCreateOrderButton(),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildSectionHeader(String title, IconData icon) {
    return Row(
      children: [
        Icon(icon, color: Colors.blue.shade700, size: 20),
        SizedBox(width: 8),
        Text(
          title,
          style: TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
            color: Colors.blue.shade700,
          ),
        ),
      ],
    );
  }

  Widget _buildStationDropdown(
    String hint,
    MapShape? selectedStation,
    Function(MapShape?) onChanged,
  ) {
    return DropdownButtonFormField<MapShape>(
      value: selectedStation,
      decoration: InputDecoration(
        border: OutlineInputBorder(),
        contentPadding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      ),
      hint: Text(hint),
      items: _availableStations.map((station) {
        return DropdownMenuItem<MapShape>(
          value: station,
          child: Row(
            children: [
              Container(
                width: 16,
                height: 16,
                decoration: BoxDecoration(
                  color: _getStationTypeColor(station.type),
                  shape: BoxShape.circle,
                ),
              ),
              SizedBox(width: 8),
              Expanded(
                child: Text(
                  '${station.name} (${station.type})',
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
          ),
        );
      }).toList(),
      onChanged: onChanged,
      isExpanded: true,
    );
  }

  Widget _buildRoutePreview() {
    final distance =
        _calculateDistance(_selectedFromStation!, _selectedToStation!);

    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.green.shade50,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.green.shade200),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(Icons.route, color: Colors.green.shade700),
              SizedBox(width: 8),
              Text(
                'Route Preview',
                style: TextStyle(
                  fontWeight: FontWeight.bold,
                  color: Colors.green.shade700,
                ),
              ),
            ],
          ),
          SizedBox(height: 12),
          _buildRouteStep(1, _selectedFromStation!, 'START'),
          SizedBox(height: 8),
          _buildRouteConnector(),
          SizedBox(height: 8),
          _buildRouteStep(2, _selectedToStation!, 'END'),
          SizedBox(height: 12),
          Container(
            padding: EdgeInsets.all(8),
            decoration: BoxDecoration(
              color: Colors.white,
              borderRadius: BorderRadius.circular(6),
            ),
            child: Row(
              children: [
                Icon(Icons.straighten, color: Colors.green.shade600, size: 16),
                SizedBox(width: 6),
                Text(
                  'Distance: ${distance.toStringAsFixed(2)}m',
                  style: TextStyle(
                    color: Colors.green.shade700,
                    fontWeight: FontWeight.w600,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildRouteStep(int step, MapShape station, String label) {
    return Row(
      children: [
        Container(
          width: 24,
          height: 24,
          decoration: BoxDecoration(
            color: _getStationTypeColor(station.type),
            shape: BoxShape.circle,
          ),
          child: Center(
            child: Text(
              step.toString(),
              style: TextStyle(
                color: Colors.white,
                fontSize: 12,
                fontWeight: FontWeight.bold,
              ),
            ),
          ),
        ),
        SizedBox(width: 12),
        Expanded(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                station.name,
                style: TextStyle(fontWeight: FontWeight.w600),
              ),
              Text(
                '$label • ${station.type.toUpperCase()}',
                style: TextStyle(
                  fontSize: 12,
                  color: Colors.grey.shade600,
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildRouteConnector() {
    return Row(
      children: [
        SizedBox(width: 12),
        Container(
          width: 2,
          height: 20,
          color: Colors.green.shade300,
        ),
        SizedBox(width: 10),
        Icon(Icons.arrow_downward, color: Colors.green.shade400, size: 16),
      ],
    );
  }

  Widget _buildCreateOrderButton() {
    final canCreate = _selectedFromStation != null &&
        _selectedToStation != null &&
        _AMRSerialController.text.isNotEmpty &&
        !_isCreating;

    return SizedBox(
      height: 50,
      child: ElevatedButton.icon(
        onPressed: canCreate ? _createOrder : null,
        icon: _isCreating
            ? SizedBox(
                width: 20,
                height: 20,
                child: CircularProgressIndicator(
                  strokeWidth: 2,
                  valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
                ),
              )
            : Icon(Icons.send),
        label: Text(
          _isCreating ? 'Creating...' : 'Create Order',
          style: TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
          ),
        ),
        style: ElevatedButton.styleFrom(
          backgroundColor: Colors.green,
          foregroundColor: Colors.white,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(12),
          ),
        ),
      ),
    );
  }

  double _calculateDistance(MapShape from, MapShape to) {
    final dx = from.center.x - to.center.x;
    final dy = from.center.y - to.center.y;
    return (dx * dx + dy * dy).abs();
  }

  Color _getStationTypeColor(String type) {
    switch (type.toLowerCase()) {
      case 'pickup':
      case 'pick_up':
        return Colors.green;
      case 'drop':
        return Colors.blue;
      case 'charging':
      case 'charge_1':
      case 'charge_2':
        return Colors.orange;
      case 'waypoint':
        return Colors.purple;
      default:
        return Colors.grey;
    }
  }

  Future<void> _createOrder() async {
    setState(() {
      _isCreating = true;
    });

    try {
      // Validate selections
      if (_selectedFromStation == null || _selectedToStation == null) {
        throw Exception('Please select both from and to stations');
      }

      print('🔧 Creating order with stations:');
      print('From: ${_selectedFromStation!.name} (${_selectedFromStation!.type})');
      print('To: ${_selectedToStation!.name} (${_selectedToStation!.type})');

      // Create waypoints list with proper type handling and error checking
      final List<Map<String, dynamic>> waypoints = [];
      
      // Add start waypoint
      try {
        final fromCenter = _selectedFromStation!.center;
        waypoints.add({
          'name': _selectedFromStation!.name,
          'type': _selectedFromStation!.type,
          'position': {
            'x': fromCenter.x.toDouble(),
            'y': fromCenter.y.toDouble(),
            'z': fromCenter.z.toDouble(),
          },
          'metadata': {
            'stationId': _selectedFromStation!.id,
            'isStart': true,
          },
        });
      } catch (e) {
        throw Exception('Error processing start station: $e');
      }
      
      // Add end waypoint
      try {
        final toCenter = _selectedToStation!.center;
        waypoints.add({
          'name': _selectedToStation!.name,
          'type': _selectedToStation!.type,
          'position': {
            'x': toCenter.x.toDouble(),
            'y': toCenter.y.toDouble(),
            'z': toCenter.z.toDouble(),
          },
          'metadata': {
            'stationId': _selectedToStation!.id,
            'isEnd': true,
          },
        });
      } catch (e) {
        throw Exception('Error processing end station: $e');
      }

      print('📍 Created ${waypoints.length} waypoints');

      // Create order data with type safety
      final orderData = <String, dynamic>{
        'deviceId': widget.deviceId,
        'name': 'Order ${_selectedFromStation!.name} → ${_selectedToStation!.name}',
        'AMRSerial': _AMRSerialController.text.trim(),
        'type': 'Normal',
        'waypoints': waypoints,
        'route': <String, dynamic>{
          'start': _selectedFromStation!.name,
          'end': _selectedToStation!.name,
          'distance': _calculateDistance(_selectedFromStation!, _selectedToStation!),
        },
      };

      print('📝 Calling API to create order...');

      // Call API to create order with explicit types
      final response = await _apiService.createOrder(
        deviceId: widget.deviceId,
        name: orderData['name'] as String,
        waypoints: waypoints, // Direct waypoints list, not cast
        priority: 5,
        description: 'Created from station selection: ${_selectedFromStation!.name} to ${_selectedToStation!.name}',
      );

      print('📡 API Response: $response');

      if (response['success'] == true) {
        print('✅ Order created successfully, calling callback...');
        widget.onOrderCreated(orderData);

        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(
              content: Text('✅ Order created successfully!'),
              backgroundColor: Colors.green,
            ),
          );

          Navigator.of(context).pop();
        }
      } else {
        throw Exception(response['error'] ?? 'Failed to create order');
      }
    } catch (e) {
      print('❌ Error in _createOrder: $e');
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('❌ Failed to create order: $e'),
            backgroundColor: Colors.red,
          ),
        );
      }
    } finally {
      if (mounted) {
        setState(() {
          _isCreating = false;
        });
      }
    }
  }

  @override
  void dispose() {
    _AMRSerialController.dispose();
    super.dispose();
  }
}

// Custom painter for order creation map
class OrderCreationMapPainter extends CustomPainter {
  final MapData mapData;
  final List<MapShape> availableStations;
  final MapShape? selectedFromStation;
  final MapShape? selectedToStation;
  final double mapScale;
  final Offset mapOffset;
  final Size canvasSize;

  OrderCreationMapPainter({
    required this.mapData,
    required this.availableStations,
    required this.selectedFromStation,
    required this.selectedToStation,
    required this.mapScale,
    required this.mapOffset,
    required this.canvasSize,
  });

  @override
  void paint(Canvas canvas, Size size) {
    canvas.save();
    canvas.translate(mapOffset.dx, mapOffset.dy);
    canvas.scale(mapScale);

    // Draw map background
    final bgPaint = Paint()..color = Colors.grey.shade100;
    canvas.drawRect(
        Rect.fromLTWH(0, 0, size.width / mapScale, size.height / mapScale),
        bgPaint);

    // Draw grid
    _drawGrid(canvas, size);

    // Draw stations with numbers and labels
    _drawStations(canvas, size);

    // Draw route path if both stations selected
    if (selectedFromStation != null && selectedToStation != null) {
      _drawRoute(canvas, size);
    }

    canvas.restore();
  }

  void _drawGrid(Canvas canvas, Size size) {
    final gridPaint = Paint()
      ..color = Colors.grey.shade300
      ..strokeWidth = 0.5;

    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;
    const gridSpacing = 50.0;

    // Vertical lines
    for (double x = 0; x < adjustedWidth; x += gridSpacing) {
      canvas.drawLine(Offset(x, 0), Offset(x, adjustedHeight), gridPaint);
    }

    // Horizontal lines
    for (double y = 0; y < adjustedHeight; y += gridSpacing) {
      canvas.drawLine(Offset(0, y), Offset(adjustedWidth, y), gridPaint);
    }
  }

  void _drawStations(Canvas canvas, Size size) {
    for (int i = 0; i < availableStations.length; i++) {
      final station = availableStations[i];
      final stationPos = _worldToCanvas(station.center, size);

      if (stationPos != null) {
        _drawStation(canvas, stationPos, station, i + 1);
      }
    }
  }

  void _drawStation(
      Canvas canvas, Offset position, MapShape station, int number) {
    final color = _getStationColor(station.type);
    final isSelected =
        station == selectedFromStation || station == selectedToStation;

    // Draw larger circle for selected stations
    final radius = isSelected ? 25.0 : 20.0;

    // Outer glow for selected stations
    if (isSelected) {
      final glowPaint = Paint()
        ..color = color.withOpacity(0.3)
        ..maskFilter = MaskFilter.blur(BlurStyle.normal, 8);
      canvas.drawCircle(position, radius + 10, glowPaint);
    }

    // Main circle
    final mainPaint = Paint()
      ..color = color
      ..style = PaintingStyle.fill;
    canvas.drawCircle(position, radius, mainPaint);

    // Border
    final borderPaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;
    canvas.drawCircle(position, radius, borderPaint);

    // Station number
    final textPainter = TextPainter(
      text: TextSpan(
        text: number.toString(),
        style: TextStyle(
          color: Colors.white,
          fontWeight: FontWeight.bold,
          fontSize: 14,
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    textPainter.layout();
    textPainter.paint(
      canvas,
      Offset(
        position.dx - textPainter.width / 2,
        position.dy - textPainter.height / 2,
      ),
    );

    // Station label
    final labelPainter = TextPainter(
      text: TextSpan(
        text: '${station.name}\n(${station.type})',
        style: TextStyle(
          color: Colors.black87,
          fontSize: 10,
          fontWeight: FontWeight.w600,
        ),
      ),
      textAlign: TextAlign.center,
      textDirection: TextDirection.ltr,
    );
    labelPainter.layout();

    // Label background
    final labelBg = Paint()..color = Colors.white.withOpacity(0.9);
    final labelRect = Rect.fromCenter(
      center: Offset(position.dx, position.dy + radius + 20),
      width: labelPainter.width + 8,
      height: labelPainter.height + 4,
    );
    canvas.drawRRect(
        RRect.fromRectAndRadius(labelRect, Radius.circular(4)), labelBg);

    // Label text
    labelPainter.paint(
      canvas,
      Offset(
        position.dx - labelPainter.width / 2,
        position.dy + radius + 12,
      ),
    );

    // Selection indicator
    if (isSelected) {
      final selectionPaint = Paint()
        ..color = station == selectedFromStation ? Colors.green : Colors.red
        ..style = PaintingStyle.stroke
        ..strokeWidth = 3;
      canvas.drawCircle(position, radius + 5, selectionPaint);
    }
  }

  void _drawRoute(Canvas canvas, Size size) {
    final fromPos = _worldToCanvas(selectedFromStation!.center, size);
    final toPos = _worldToCanvas(selectedToStation!.center, size);

    if (fromPos != null && toPos != null) {
      // Route line
      final routePaint = Paint()
        ..color = Colors.blue.withOpacity(0.6)
        ..strokeWidth = 4
        ..style = PaintingStyle.stroke;

      canvas.drawLine(fromPos, toPos, routePaint);

      // Arrow at end
      _drawArrow(canvas, fromPos, toPos, Colors.blue);
    }
  }

  void _drawArrow(Canvas canvas, Offset start, Offset end, Color color) {
    final direction = (end - start).direction;
    const arrowLength = 15.0;
    const arrowAngle = 0.5;

    final arrowPoint1 = end +
        Offset(
          arrowLength * cos(direction + pi - arrowAngle),
          arrowLength * sin(direction + pi - arrowAngle),
        );

    final arrowPoint2 = end +
        Offset(
          arrowLength * cos(direction + pi + arrowAngle),
          arrowLength * sin(direction + pi + arrowAngle),
        );

    final arrowPaint = Paint()
      ..color = color
      ..strokeWidth = 3
      ..style = PaintingStyle.stroke;

    canvas.drawLine(end, arrowPoint1, arrowPaint);
    canvas.drawLine(end, arrowPoint2, arrowPaint);
  }

  Offset? _worldToCanvas(dynamic worldPos, Size size) {
    try {
      final canvasWidth = size.width / mapScale;
      final canvasHeight = size.height / mapScale;

      // Simple coordinate mapping - adjust based on your map coordinate system
      final x = (worldPos.x + 25) *
          (canvasWidth / 50); // Assuming map is 50x50 world units
      final y = (25 - worldPos.y) * (canvasHeight / 50); // Flip Y axis

      return Offset(x, y);
    } catch (e) {
      return null;
    }
  }

  Color _getStationColor(String type) {
    switch (type.toLowerCase()) {
      case 'pickup':
      case 'pick_up':
        return Colors.green;
      case 'drop':
        return Colors.blue;
      case 'charging':
      case 'charge_1':
      case 'charge_2':
        return Colors.orange;
      case 'waypoint':
        return Colors.purple;
      default:
        return Colors.grey;
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}
