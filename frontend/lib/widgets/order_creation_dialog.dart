// widgets/order_creation_dialog.dart - Order Creation with Map Station Selection
import 'package:flutter/material.dart';
import '../models/map_data.dart';

class OrderCreationDialog extends StatefulWidget {
  final List<Map<String, dynamic>> availableDevices;
  final Map<String, MapData> availableMaps;
  final String? preSelectedDevice;
  final Function(Map<String, dynamic>) onOrderCreated;

  const OrderCreationDialog({
    Key? key,
    required this.availableDevices,
    required this.availableMaps,
    this.preSelectedDevice,
    required this.onOrderCreated,
  }) : super(key: key);

  @override
  State<OrderCreationDialog> createState() => _OrderCreationDialogState();
}

class _OrderCreationDialogState extends State<OrderCreationDialog>
    with TickerProviderStateMixin {
  
  late TabController _tabController;
  late AnimationController _sequenceAnimationController;
  
  // Form controllers
  final _nameController = TextEditingController();
  final _priorityController = TextEditingController(text: '0');
  
  // Selection state
  String? _selectedDeviceId;
  MapData? _selectedMap;
  List<Map<String, dynamic>> _orderSequence = [];
  
  // UI state
  int _currentStep = 0;
  bool _isCreating = false;
  
  // Available stations by type
  Map<String, List<MapShape>> _stationsByType = {};
  
  @override
  void initState() {
    super.initState();
    _initializeControllers();
    _initializeData();
  }
  
  void _initializeControllers() {
    _tabController = TabController(length: 3, vsync: this);
    _sequenceAnimationController = AnimationController(
      duration: const Duration(milliseconds: 300),
      vsync: this,
    );
  }
  
  void _initializeData() {
    // Pre-select device if provided
    if (widget.preSelectedDevice != null) {
      _selectedDeviceId = widget.preSelectedDevice;
      _selectedMap = widget.availableMaps[widget.preSelectedDevice];
      _updateStationsByType();
      _currentStep = 1; // Skip device selection
    }
  }
  
  void _updateStationsByType() {
    if (_selectedMap == null) return;
    
    _stationsByType.clear();
    for (final shape in _selectedMap!.shapes) {
      if (!_stationsByType.containsKey(shape.type)) {
        _stationsByType[shape.type] = [];
      }
      _stationsByType[shape.type]!.add(shape);
    }
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return Dialog(
      insetPadding: EdgeInsets.all(16),
      child: Container(
        width: MediaQuery.of(context).size.width * 0.9,
        height: MediaQuery.of(context).size.height * 0.8,
        child: Scaffold(
          appBar: AppBar(
            title: Text('Create New Order'),
            backgroundColor: Colors.green,
            foregroundColor: Colors.white,
            leading: IconButton(
              icon: Icon(Icons.close),
              onPressed: () => Navigator.of(context).pop(),
            ),
            actions: [
              if (_currentStep > 0)
                TextButton(
                  onPressed: () {
                    setState(() {
                      _currentStep--;
                    });
                  },
                  child: Text('Back', style: TextStyle(color: Colors.white)),
                ),
            ],
          ),
          body: Column(
            children: [
              _buildProgressIndicator(),
              Expanded(
                child: _buildCurrentStep(),
              ),
              _buildBottomActions(),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildProgressIndicator() {
    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.green.shade50,
        border: Border(bottom: BorderSide(color: Colors.green.shade200)),
      ),
      child: Row(
        children: [
          _buildStepIndicator(0, 'Device & Map', Icons.devices),
          _buildStepConnector(),
          _buildStepIndicator(1, 'Build Sequence', Icons.route),
          _buildStepConnector(),
          _buildStepIndicator(2, 'Order Details', Icons.assignment),
        ],
      ),
    );
  }

  Widget _buildStepIndicator(int step, String label, IconData icon) {
    final isActive = _currentStep == step;
    final isCompleted = _currentStep > step;
    
    Color backgroundColor;
    Color iconColor;
    
    if (isCompleted) {
      backgroundColor = Colors.green;
      iconColor = Colors.white;
    } else if (isActive) {
      backgroundColor = Colors.green.shade300;
      iconColor = Colors.white;
    } else {
      backgroundColor = Colors.grey.shade300;
      iconColor = Colors.grey.shade600;
    }

    return Expanded(
      child: Column(
        children: [
          Container(
            width: 40,
            height: 40,
            decoration: BoxDecoration(
              color: backgroundColor,
              shape: BoxShape.circle,
            ),
            child: Icon(
              isCompleted ? Icons.check : icon,
              color: iconColor,
              size: 20,
            ),
          ),
          SizedBox(height: 8),
          Text(
            label,
            style: TextStyle(
              fontSize: 12,
              fontWeight: isActive ? FontWeight.bold : FontWeight.normal,
              color: isActive ? Colors.green.shade700 : Colors.grey.shade600,
            ),
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }

  Widget _buildStepConnector() {
    return Container(
      width: 30,
      height: 2,
      color: Colors.grey.shade300,
    );
  }

  Widget _buildCurrentStep() {
    switch (_currentStep) {
      case 0:
        return _buildDeviceMapSelection();
      case 1:
        return _buildSequenceBuilder();
      case 2:
        return _buildOrderDetails();
      default:
        return Container();
    }
  }

  Widget _buildDeviceMapSelection() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Header
          Container(
            padding: EdgeInsets.all(16),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [Colors.blue.shade50, Colors.blue.shade100],
              ),
              borderRadius: BorderRadius.circular(12),
              border: Border.all(color: Colors.blue.shade200),
            ),
            child: Row(
              children: [
                Icon(Icons.info, color: Colors.blue.shade700),
                SizedBox(width: 12),
                Expanded(
                  child: Text(
                    'Select a device and its map to create an order sequence',
                    style: TextStyle(
                      color: Colors.blue.shade700,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                ),
              ],
            ),
          ),
          
          SizedBox(height: 24),
          
          // Device Selection
          Text(
            'Select Device',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          ),
          SizedBox(height: 12),
          
          ...widget.availableDevices.map((device) {
            final isSelected = _selectedDeviceId == device['id'];
            final hasMap = widget.availableMaps.containsKey(device['id']);
            
            return Card(
              margin: EdgeInsets.symmetric(vertical: 4),
              color: isSelected ? Colors.green.shade50 : null,
              child: ListTile(
                leading: Container(
                  width: 50,
                  height: 50,
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: isSelected 
                          ? [Colors.green.shade400, Colors.green.shade600]
                          : [Colors.grey.shade400, Colors.grey.shade600],
                    ),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Icon(Icons.smart_toy, color: Colors.white),
                ),
                title: Text(
                  device['name'] ?? device['id'],
                  style: TextStyle(
                    fontWeight: isSelected ? FontWeight.bold : FontWeight.normal,
                  ),
                ),
                subtitle: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('ID: ${device['id']}'),
                    Row(
                      children: [
                        Icon(
                          hasMap ? Icons.check_circle : Icons.warning,
                          size: 16,
                          color: hasMap ? Colors.green : Colors.orange,
                        ),
                        SizedBox(width: 4),
                        Text(
                          hasMap ? 'Map Available' : 'No Map',
                          style: TextStyle(
                            color: hasMap ? Colors.green : Colors.orange,
                            fontWeight: FontWeight.w500,
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
                trailing: Radio<String>(
                  value: device['id'],
                  groupValue: _selectedDeviceId,
                  onChanged: hasMap ? (value) {
                    setState(() {
                      _selectedDeviceId = value;
                      _selectedMap = widget.availableMaps[value];
                      _updateStationsByType();
                    });
                  } : null,
                ),
                onTap: hasMap ? () {
                  setState(() {
                    _selectedDeviceId = device['id'];
                    _selectedMap = widget.availableMaps[device['id']];
                    _updateStationsByType();
                  });
                } : null,
              ),
            );
          }).toList(),
          
          if (_selectedMap != null) ...[
            SizedBox(height: 24),
            
            // Map Information
            Container(
              padding: EdgeInsets.all(16),
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [Colors.purple.shade50, Colors.purple.shade100],
                ),
                borderRadius: BorderRadius.circular(12),
                border: Border.all(color: Colors.purple.shade200),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(
                    children: [
                      Icon(Icons.map, color: Colors.purple.shade700),
                      SizedBox(width: 12),
                      Text(
                        'Selected Map Information',
                        style: TextStyle(
                          fontSize: 16,
                          fontWeight: FontWeight.bold,
                          color: Colors.purple.shade700,
                        ),
                      ),
                    ],
                  ),
                  SizedBox(height: 12),
                  
                  Row(
                    children: [
                      Expanded(
                        child: _buildMapInfoItem(
                          'Size',
                          '${_selectedMap!.info.width}×${_selectedMap!.info.height}',
                          Icons.aspect_ratio,
                        ),
                      ),
                      Expanded(
                        child: _buildMapInfoItem(
                          'Resolution',
                          '${_selectedMap!.info.resolution.toStringAsFixed(3)}m/px',
                          Icons.grid_4x4,
                        ),
                      ),
                    ],
                  ),
                  
                  SizedBox(height: 12),
                  
                  // Available stations
                  Text(
                    'Available Stations:',
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                      color: Colors.purple.shade700,
                    ),
                  ),
                  SizedBox(height: 8),
                  
                  if (_stationsByType.isEmpty)
                    Text(
                      'No stations defined on this map',
                      style: TextStyle(
                        color: Colors.orange.shade700,
                        fontStyle: FontStyle.italic,
                      ),
                    )
                  else
                    Wrap(
                      spacing: 8,
                      runSpacing: 4,
                      children: _stationsByType.entries.map((entry) {
                        final color = _getStationTypeColor(entry.key);
                        return Chip(
                          label: Text(
                            '${entry.key.toUpperCase()}: ${entry.value.length}',
                            style: TextStyle(
                              fontSize: 12,
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                          backgroundColor: color.withOpacity(0.2),
                          side: BorderSide(color: color),
                        );
                      }).toList(),
                    ),
                ],
              ),
            ),
          ],
        ],
      ),
    );
  }

  Widget _buildMapInfoItem(String label, String value, IconData icon) {
    return Row(
      children: [
        Icon(icon, size: 16, color: Colors.purple.shade600),
        SizedBox(width: 6),
        Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              label,
              style: TextStyle(
                fontSize: 12,
                color: Colors.purple.shade600,
              ),
            ),
            Text(
              value,
              style: TextStyle(
                fontSize: 14,
                fontWeight: FontWeight.bold,
                color: Colors.purple.shade700,
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildSequenceBuilder() {
    if (_selectedMap == null || _stationsByType.isEmpty) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.warning, size: 64, color: Colors.orange),
            SizedBox(height: 16),
            Text(
              'No Stations Available',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            Text(
              'The selected map has no defined stations.\nEdit the map to add pickup, drop, and charging stations.',
              textAlign: TextAlign.center,
            ),
            SizedBox(height: 16),
            ElevatedButton.icon(
              onPressed: () {
                Navigator.of(context).pop();
                // Navigate to map editor would be called here
              },
              icon: Icon(Icons.edit),
              label: Text('Edit Map'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.orange,
                foregroundColor: Colors.white,
              ),
            ),
          ],
        ),
      );
    }

    return Column(
      children: [
        // Sequence display
        Container(
          height: 120,
          padding: EdgeInsets.all(16),
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [Colors.green.shade50, Colors.green.shade100],
            ),
            border: Border(bottom: BorderSide(color: Colors.green.shade200)),
          ),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'Order Sequence (${_orderSequence.length}/4 steps)',
                style: TextStyle(
                  fontSize: 16,
                  fontWeight: FontWeight.bold,
                  color: Colors.green.shade700,
                ),
              ),
              SizedBox(height: 8),
              Expanded(
                child: _orderSequence.isEmpty
                    ? Center(
                        child: Text(
                          'Select stations below to build your sequence',
                          style: TextStyle(
                            color: Colors.grey.shade600,
                            fontStyle: FontStyle.italic,
                          ),
                        ),
                      )
                    : SingleChildScrollView(
                        scrollDirection: Axis.horizontal,
                        child: Row(
                          children: _orderSequence.asMap().entries.map((entry) {
                            final index = entry.key;
                            final station = entry.value;
                            return Row(
                              children: [
                                _buildSequenceStep(index + 1, station),
                                if (index < _orderSequence.length - 1)
                                  Container(
                                    margin: EdgeInsets.symmetric(horizontal: 8),
                                    child: Icon(Icons.arrow_forward, color: Colors.green.shade700),
                                  ),
                              ],
                            );
                          }).toList(),
                        ),
                      ),
              ),
            ],
          ),
        ),
        
        // Station selection
        Expanded(
          child: SingleChildScrollView(
            child: Column(
              children: _stationsByType.entries.map((entry) {
                return _buildStationTypeSection(entry.key, entry.value);
              }).toList(),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildSequenceStep(int step, Map<String, dynamic> station) {
    final color = _getStationTypeColor(station['type']);
    
    return GestureDetector(
      onTap: () => _removeFromSequence(step - 1),
      child: Container(
        padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        decoration: BoxDecoration(
          gradient: LinearGradient(
            colors: [color.withOpacity(0.8), color],
          ),
          borderRadius: BorderRadius.circular(20),
          boxShadow: [
            BoxShadow(
              color: color.withOpacity(0.3),
              blurRadius: 4,
              offset: Offset(0, 2),
            ),
          ],
        ),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Container(
              width: 20,
              height: 20,
              decoration: BoxDecoration(
                color: Colors.white,
                shape: BoxShape.circle,
              ),
              child: Center(
                child: Text(
                  step.toString(),
                  style: TextStyle(
                    color: color,
                    fontSize: 12,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ),
            ),
            SizedBox(width: 8),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  station['name'],
                  style: TextStyle(
                    color: Colors.white,
                    fontWeight: FontWeight.bold,
                    fontSize: 12,
                  ),
                ),
                Text(
                  station['type'].toUpperCase(),
                  style: TextStyle(
                    color: Colors.white.withOpacity(0.8),
                    fontSize: 10,
                  ),
                ),
              ],
            ),
            SizedBox(width: 4),
            Icon(Icons.close, color: Colors.white, size: 16),
          ],
        ),
      ),
    );
  }

  Widget _buildStationTypeSection(String type, List<MapShape> stations) {
    final color = _getStationTypeColor(type);
    final icon = _getStationTypeIcon(type);
    
    return Container(
      margin: EdgeInsets.all(8),
      decoration: BoxDecoration(
        border: Border.all(color: color.withOpacity(0.3)),
        borderRadius: BorderRadius.circular(12),
      ),
      child: Column(
        children: [
          // Section header
          Container(
            padding: EdgeInsets.all(12),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [color.withOpacity(0.1), color.withOpacity(0.05)],
              ),
              borderRadius: BorderRadius.vertical(top: Radius.circular(12)),
            ),
            child: Row(
              children: [
                Icon(icon, color: color),
                SizedBox(width: 8),
                Text(
                  '${type.toUpperCase()} STATIONS (${stations.length})',
                  style: TextStyle(
                    fontWeight: FontWeight.bold,
                    color: color,
                  ),
                ),
              ],
            ),
          ),
          
          // Station list
          ...stations.map((station) => _buildStationTile(station, color)),
        ],
      ),
    );
  }

  Widget _buildStationTile(MapShape station, Color color) {
    final isSelected = _orderSequence.any((s) => s['id'] == station.id);
    final canAdd = _orderSequence.length < 4 && !isSelected;
    
    return ListTile(
      leading: Container(
        width: 40,
        height: 40,
        decoration: BoxDecoration(
          color: isSelected ? color : color.withOpacity(0.2),
          borderRadius: BorderRadius.circular(8),
        ),
        child: Icon(
          _getStationTypeIcon(station.type),
          color: isSelected ? Colors.white : color,
        ),
      ),
      title: Text(
        station.name,
        style: TextStyle(
          fontWeight: isSelected ? FontWeight.bold : FontWeight.normal,
        ),
      ),
      subtitle: Text(
        'Position: (${station.center.x.toStringAsFixed(1)}, ${station.center.y.toStringAsFixed(1)})',
      ),
      trailing: isSelected
          ? Icon(Icons.check_circle, color: Colors.green)
          : canAdd
              ? IconButton(
                  icon: Icon(Icons.add_circle, color: color),
                  onPressed: () => _addToSequence(station),
                )
              : Icon(Icons.block, color: Colors.grey),
      onTap: canAdd ? () => _addToSequence(station) : null,
    );
  }

  Widget _buildOrderDetails() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Order name
          TextField(
            controller: _nameController,
            decoration: InputDecoration(
              labelText: 'Order Name',
              hintText: 'e.g., Warehouse A to B Delivery',
              border: OutlineInputBorder(),
              prefixIcon: Icon(Icons.assignment),
            ),
          ),
          
          SizedBox(height: 16),
          
          // Priority
          TextField(
            controller: _priorityController,
            decoration: InputDecoration(
              labelText: 'Priority (0-10)',
              hintText: '0 = Low, 10 = High',
              border: OutlineInputBorder(),
              prefixIcon: Icon(Icons.priority_high),
            ),
            keyboardType: TextInputType.number,
          ),
          
          SizedBox(height: 24),
          
          // Order summary
          Container(
            padding: EdgeInsets.all(16),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [Colors.blue.shade50, Colors.blue.shade100],
              ),
              borderRadius: BorderRadius.circular(12),
              border: Border.all(color: Colors.blue.shade200),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Order Summary',
                  style: TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                    color: Colors.blue.shade700,
                  ),
                ),
                SizedBox(height: 12),
                
                _buildSummaryRow('Device', _getSelectedDeviceName()),
                _buildSummaryRow('Map', 'Available (${_selectedMap?.shapes.length ?? 0} stations)'),
                _buildSummaryRow('Sequence Steps', '${_orderSequence.length}'),
                
                if (_orderSequence.isNotEmpty) ...[
                  SizedBox(height: 12),
                  Text(
                    'Execution Sequence:',
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                      color: Colors.blue.shade700,
                    ),
                  ),
                  SizedBox(height: 8),
                  ..._orderSequence.asMap().entries.map((entry) {
                    final index = entry.key;
                    final station = entry.value;
                    return Container(
                      margin: EdgeInsets.symmetric(vertical: 2),
                      padding: EdgeInsets.all(8),
                      decoration: BoxDecoration(
                        color: Colors.white.withOpacity(0.7),
                        borderRadius: BorderRadius.circular(8),
                      ),
                      child: Row(
                        children: [
                          Container(
                            width: 24,
                            height: 24,
                            decoration: BoxDecoration(
                              color: _getStationTypeColor(station['type']),
                              shape: BoxShape.circle,
                            ),
                            child: Center(
                              child: Text(
                                '${index + 1}',
                                style: TextStyle(
                                  color: Colors.white,
                                  fontSize: 12,
                                  fontWeight: FontWeight.bold,
                                ),
                              ),
                            ),
                          ),
                          SizedBox(width: 12),
                          Icon(
                            _getStationTypeIcon(station['type']),
                            color: _getStationTypeColor(station['type']),
                            size: 20,
                          ),
                          SizedBox(width: 8),
                          Expanded(
                            child: Text(
                              '${station['name']} (${station['type'].toUpperCase()})',
                              style: TextStyle(fontWeight: FontWeight.w600),
                            ),
                          ),
                        ],
                      ),
                    );
                  }),
                ],
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildSummaryRow(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 4),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          SizedBox(
            width: 100,
            child: Text(
              '$label:',
              style: TextStyle(
                fontWeight: FontWeight.w600,
                color: Colors.blue.shade700,
              ),
            ),
          ),
          Expanded(
            child: Text(
              value,
              style: TextStyle(color: Colors.blue.shade800),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildBottomActions() {
    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.white,
        border: Border(top: BorderSide(color: Colors.grey.shade300)),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 4,
            offset: Offset(0, -2),
          ),
        ],
      ),
      child: Row(
        children: [
          if (_currentStep > 0)
            Expanded(
              child: OutlinedButton(
                onPressed: () {
                  setState(() {
                    _currentStep--;
                  });
                },
                child: Text('Previous'),
              ),
            ),
          
          if (_currentStep > 0) SizedBox(width: 16),
          
          Expanded(
            child: ElevatedButton(
              onPressed: _canProceed() ? _handleNextOrCreate : null,
              style: ElevatedButton.styleFrom(
                backgroundColor: _currentStep == 2 ? Colors.green : Colors.blue,
                foregroundColor: Colors.white,
                padding: EdgeInsets.symmetric(vertical: 16),
              ),
              child: _isCreating
                  ? SizedBox(
                      width: 20,
                      height: 20,
                      child: CircularProgressIndicator(
                        strokeWidth: 2,
                        valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
                      ),
                    )
                  : Text(
                      _currentStep == 2 ? 'Create Order' : 'Next',
                      style: TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
            ),
          ),
        ],
      ),
    );
  }

  // Helper methods
  bool _canProceed() {
    switch (_currentStep) {
      case 0:
        return _selectedDeviceId != null && _selectedMap != null;
      case 1:
        return _orderSequence.isNotEmpty;
      case 2:
        return _nameController.text.isNotEmpty && !_isCreating;
      default:
        return false;
    }
  }

  void _handleNextOrCreate() {
    if (_currentStep < 2) {
      setState(() {
        _currentStep++;
      });
    } else {
      _createOrder();
    }
  }

  void _addToSequence(MapShape station) {
    if (_orderSequence.length < 4) {
      setState(() {
        _orderSequence.add({
          'id': station.id,
          'name': station.name,
          'type': station.type,
          'position': {
            'x': station.center.x,
            'y': station.center.y,
          },
        });
      });
      _sequenceAnimationController.forward();
    }
  }

  void _removeFromSequence(int index) {
    setState(() {
      _orderSequence.removeAt(index);
    });
  }

  String _getSelectedDeviceName() {
    final device = widget.availableDevices.firstWhere(
      (d) => d['id'] == _selectedDeviceId,
      orElse: () => {'name': _selectedDeviceId},
    );
    return device['name'] ?? _selectedDeviceId ?? 'Unknown';
  }

  void _createOrder() async {
    setState(() {
      _isCreating = true;
    });

    try {
      final orderData = {
        'deviceId': _selectedDeviceId,
        'name': _nameController.text.trim(),
        'priority': int.tryParse(_priorityController.text) ?? 0,
        'waypoints': _orderSequence,
      };

      await Future.delayed(Duration(milliseconds: 500)); // Simulate API call
      
      widget.onOrderCreated(orderData);
      Navigator.of(context).pop();
      
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Failed to create order: $e'),
          backgroundColor: Colors.red,
        ),
      );
    } finally {
      setState(() {
        _isCreating = false;
      });
    }
  }

  // Color and icon helpers
  Color _getStationTypeColor(String type) {
    switch (type) {
      case 'pickup': return Colors.green;
      case 'drop': return Colors.blue;
      case 'charging': return Colors.orange;
      case 'waypoint': return Colors.purple;
      default: return Colors.grey;
    }
  }

  IconData _getStationTypeIcon(String type) {
    switch (type) {
      case 'pickup': return Icons.outbox;
      case 'drop': return Icons.inbox;
      case 'charging': return Icons.battery_charging_full;
      case 'waypoint': return Icons.place;
      default: return Icons.location_on;
    }
  }

  @override
  void dispose() {
    _nameController.dispose();
    _priorityController.dispose();
    _tabController.dispose();
    _sequenceAnimationController.dispose();
    super.dispose();
  }
}