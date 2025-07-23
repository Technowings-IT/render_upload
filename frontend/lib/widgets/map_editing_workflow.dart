// widgets/map_editing_workflow.dart - Complete Map Editing Workflow
import 'package:flutter/material.dart';
import '../services/api_service.dart';
import '../models/map_data.dart';
import 'map_selection.dart';

// Define Position class if not already imported
class Position {
  final double x;
  final double y;
  final double z;

  Position({
    required this.x,
    required this.y,
    required this.z,
  });
}

class MapEditingWorkflow extends StatefulWidget {
  final String deviceId;
  final MapData? currentMapData;
  final Function(MapData) onMapUpdated;

  const MapEditingWorkflow({
    Key? key,
    required this.deviceId,
    this.currentMapData,
    required this.onMapUpdated,
  }) : super(key: key);

  @override
  _MapEditingWorkflowState createState() => _MapEditingWorkflowState();
}

class _MapEditingWorkflowState extends State<MapEditingWorkflow>
    with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  
  late TabController _tabController;
  
  // Workflow state
  WorkflowStep _currentStep = WorkflowStep.convert;
  bool _isProcessing = false;
  String? _workflowError;
  
  // Map data
  String? _convertedMapName;
  List<LocationMarker> _locationMarkers = [];
  
  // Deployment configuration
  final TextEditingController _finalMapNameController = TextEditingController();
  final TextEditingController _deploymentNotesController = TextEditingController();
  bool _autoLoad = true;
  Map<String, dynamic> _piConfig = {};
  
  // Location editing
  LocationType _selectedLocationType = LocationType.pickup;
  final TextEditingController _locationNameController = TextEditingController();
  Position? _pendingLocationPosition;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 4, vsync: this);
    _loadDeploymentConfig();
    
    // Set default final map name
    _finalMapNameController.text = 'edited_map_${widget.deviceId}_${DateTime.now().millisecondsSinceEpoch}';
  }

  @override
  void dispose() {
    _tabController.dispose();
    _finalMapNameController.dispose();
    _deploymentNotesController.dispose();
    _locationNameController.dispose();
    super.dispose();
  }

  Future<void> _loadDeploymentConfig() async {
    try {
      final config = await _apiService.getDeploymentConfig(widget.deviceId);
      if (config['success'] == true) {
        setState(() {
          _piConfig = config['config'] ?? {};
        });
      }
    } catch (e) {
      print('❌ Error loading deployment config: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      height: MediaQuery.of(context).size.height * 0.8,
      child: Column(
        children: [
          _buildWorkflowHeader(),
          _buildProgressIndicator(),
          Expanded(child: _buildWorkflowContent()),
          _buildWorkflowActions(),
        ],
      ),
    );
  }

  Widget _buildWorkflowHeader() {
    return Container(
      padding: EdgeInsets.all(20),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.purple.shade700, Colors.purple.shade500],
        ),
        borderRadius: BorderRadius.vertical(top: Radius.circular(16)),
      ),
      child: Row(
        children: [
          Icon(Icons.edit_location, color: Colors.white, size: 28),
          SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Map Editing Workflow',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 20,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                Text(
                  'Convert → Edit → Deploy to Raspberry Pi',
                  style: TextStyle(
                    color: Colors.white.withOpacity(0.9),
                    fontSize: 14,
                  ),
                ),
              ],
            ),
          ),
          if (_isProcessing)
            SizedBox(
              width: 20,
              height: 20,
              child: CircularProgressIndicator(
                strokeWidth: 2,
                valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildProgressIndicator() {
    final steps = WorkflowStep.values;
    final currentIndex = steps.indexOf(_currentStep);
    
    return Container(
      padding: EdgeInsets.all(16),
      child: Row(
        children: steps.asMap().entries.map((entry) {
          final index = entry.key;
          final step = entry.value;
          final isActive = index <= currentIndex;
          final isCurrent = index == currentIndex;
          
          return Expanded(
            child: Row(
              children: [
                Expanded(
                  child: Column(
                    children: [
                      Container(
                        width: 32,
                        height: 32,
                        decoration: BoxDecoration(
                          color: isActive ? Colors.purple : Colors.grey.shade300,
                          shape: BoxShape.circle,
                          border: isCurrent ? Border.all(color: Colors.purple.shade700, width: 2) : null,
                        ),
                        child: Center(
                          child: isActive
                              ? Icon(
                                  _getStepIcon(step),
                                  color: Colors.white,
                                  size: 16,
                                )
                              : Text(
                                  '${index + 1}',
                                  style: TextStyle(
                                    color: Colors.grey.shade600,
                                    fontWeight: FontWeight.bold,
                                  ),
                                ),
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        _getStepName(step),
                        style: TextStyle(
                          fontSize: 10,
                          color: isActive ? Colors.purple.shade700 : Colors.grey.shade600,
                          fontWeight: isCurrent ? FontWeight.bold : FontWeight.normal,
                        ),
                        textAlign: TextAlign.center,
                      ),
                    ],
                  ),
                ),
                if (index < steps.length - 1)
                  Container(
                    width: 20,
                    height: 2,
                    color: isActive ? Colors.purple : Colors.grey.shade300,
                    margin: EdgeInsets.only(bottom: 20),
                  ),
              ],
            ),
          );
        }).toList(),
      ),
    );
  }

  Widget _buildWorkflowContent() {
    if (_workflowError != null) {
      return _buildErrorView();
    }

    switch (_currentStep) {
      case WorkflowStep.convert:
        return _buildConvertStep();
      case WorkflowStep.edit:
        return _buildEditStep();
      case WorkflowStep.configure:
        return _buildConfigureStep();
      case WorkflowStep.deploy:
        return _buildDeployStep();
    }
  }

  Widget _buildErrorView() {
    return Center(
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.error, size: 64, color: Colors.red),
            SizedBox(height: 16),
            Text(
              'Workflow Error',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            Text(
              _workflowError!,
              textAlign: TextAlign.center,
              style: TextStyle(color: Colors.red),
            ),
            SizedBox(height: 16),
            ElevatedButton(
              onPressed: () {
                setState(() {
                  _workflowError = null;
                  _currentStep = WorkflowStep.convert;
                });
              },
              child: Text('Start Over'),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildConvertStep() {
    return Padding(
      padding: EdgeInsets.all(20),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Step 1: Convert to PGM Format',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          ),
          SizedBox(height: 8),
          Text(
            'Convert your current map data to PGM+YAML format for editing.',
            style: TextStyle(color: Colors.grey.shade600),
          ),
          SizedBox(height: 20),
          
          if (widget.currentMapData != null) ...[
            Card(
              child: Padding(
                padding: EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Current Map Data',
                      style: TextStyle(fontWeight: FontWeight.bold),
                    ),
                    SizedBox(height: 8),
                    _buildMapInfoRow('Device ID', widget.currentMapData!.deviceId),
                    _buildMapInfoRow('Dimensions', '${widget.currentMapData!.info.width}×${widget.currentMapData!.info.height}'),
                    _buildMapInfoRow('Resolution', '${widget.currentMapData!.info.resolution}m/px'),
                    _buildMapInfoRow('Shapes', '${widget.currentMapData!.shapes.length}'),
                    _buildMapInfoRow('Last Updated', _formatDateTime(widget.currentMapData!.timestamp)),
                  ],
                ),
              ),
            ),
          ] else ...[
            Card(
              child: Padding(
                padding: EdgeInsets.all(16),
                child: Row(
                  children: [
                    Icon(Icons.warning, color: Colors.orange),
                    SizedBox(width: 12),
                    Expanded(
                      child: Text(
                        'No current map data available. You can still proceed with saved maps.',
                        style: TextStyle(color: Colors.orange.shade700),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
          
          Spacer(),
          
          Center(
            child: ElevatedButton.icon(
              onPressed: _isProcessing ? null : _startConversion,
              icon: Icon(Icons.transform),
              label: Text('Convert to PGM Format'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.purple,
                foregroundColor: Colors.white,
                padding: EdgeInsets.symmetric(horizontal: 32, vertical: 16),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEditStep() {
    return Padding(
      padding: EdgeInsets.all(20),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Step 2: Add Location Markers',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          ),
          SizedBox(height: 8),
          Text(
            'Add pickup, drop, charging, and home locations to your map.',
            style: TextStyle(color: Colors.grey.shade600),
          ),
          SizedBox(height: 20),
          
          // Location type selector
          Row(
            children: [
              Text('Location Type: ', style: TextStyle(fontWeight: FontWeight.bold)),
              SizedBox(width: 8),
              DropdownButton<LocationType>(
                value: _selectedLocationType,
                onChanged: (value) {
                  setState(() {
                    _selectedLocationType = value!;
                  });
                },
                items: LocationType.values.map((type) {
                  return DropdownMenuItem(
                    value: type,
                    child: Row(
                      children: [
                        Text(_apiService.getLocationTypeIcon(type.name)),
                        SizedBox(width: 8),
                        Text(type.name.toUpperCase()),
                      ],
                    ),
                  );
                }).toList(),
              ),
            ],
          ),
          
          SizedBox(height: 16),
          
          // Add location form
          Card(
            child: Padding(
              padding: EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    'Add New Location',
                    style: TextStyle(fontWeight: FontWeight.bold),
                  ),
                  SizedBox(height: 12),
                  
                  TextField(
                    controller: _locationNameController,
                    decoration: InputDecoration(
                      labelText: 'Location Name',
                      hintText: 'e.g., Main Pickup Point',
                      border: OutlineInputBorder(),
                    ),
                  ),
                  
                  SizedBox(height: 12),
                  
                  Row(
                    children: [
                      Expanded(
                        child: ElevatedButton.icon(
                          onPressed: _selectLocationOnMap,
                          icon: Icon(Icons.location_on),
                          label: Text('Select on Map'),
                        ),
                      ),
                      SizedBox(width: 12),
                      Expanded(
                        child: ElevatedButton.icon(
                          onPressed: _canAddLocation() ? _addLocation : null,
                          icon: Icon(Icons.add),
                          label: Text('Add Location'),
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
          
          SizedBox(height: 16),
          
          // Current locations list
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Added Locations (${_locationMarkers.length})',
                  style: TextStyle(fontWeight: FontWeight.bold),
                ),
                SizedBox(height: 8),
                
                Expanded(
                  child: _locationMarkers.isEmpty
                      ? Center(
                          child: Text(
                            'No locations added yet.\nTap "Select on Map" to add locations.',
                            textAlign: TextAlign.center,
                            style: TextStyle(color: Colors.grey),
                          ),
                        )
                      : ListView.builder(
                          itemCount: _locationMarkers.length,
                          itemBuilder: (context, index) {
                            final marker = _locationMarkers[index];
                            return _buildLocationTile(marker, index);
                          },
                        ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildConfigureStep() {
    return Padding(
      padding: EdgeInsets.all(20),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Step 3: Configure Deployment',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          ),
          SizedBox(height: 8),
          Text(
            'Set the final map name and deployment options.',
            style: TextStyle(color: Colors.grey.shade600),
          ),
          SizedBox(height: 20),
          
          // Final map name
          TextField(
            controller: _finalMapNameController,
            decoration: InputDecoration(
              labelText: 'Final Map Name',
              hintText: 'Enter the name for deployment',
              border: OutlineInputBorder(),
              prefixIcon: Icon(Icons.label),
            ),
          ),
          
          SizedBox(height: 16),
          
          // Deployment notes
          TextField(
            controller: _deploymentNotesController,
            decoration: InputDecoration(
              labelText: 'Deployment Notes (Optional)',
              hintText: 'Add any notes about this deployment...',
              border: OutlineInputBorder(),
              prefixIcon: Icon(Icons.note),
            ),
            maxLines: 3,
          ),
          
          SizedBox(height: 16),
          
          // Auto-load option
          CheckboxListTile(
            title: Text('Auto-load on Raspberry Pi'),
            subtitle: Text('Automatically load this map in ROS after deployment'),
            value: _autoLoad,
            onChanged: (value) {
              setState(() {
                _autoLoad = value ?? true;
              });
            },
            contentPadding: EdgeInsets.zero,
          ),
          
          SizedBox(height: 16),
          
          // Raspberry Pi configuration
          Card(
            child: Padding(
              padding: EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(
                    children: [
                      Icon(Icons.settings, color: Colors.blue),
                      SizedBox(width: 8),
                      Text(
                        'Raspberry Pi Configuration',
                        style: TextStyle(fontWeight: FontWeight.bold),
                      ),
                    ],
                  ),
                  SizedBox(height: 12),
                  
                  _buildConfigRow('Host', _piConfig['piHost'] ?? 'Not configured'),
                  _buildConfigRow('Port', _piConfig['piPort']?.toString() ?? 'Not configured'),
                  _buildConfigRow('User', _piConfig['piUser'] ?? 'Not configured'),
                  _buildConfigRow('Map Directory', _piConfig['piMapDir'] ?? 'Not configured'),
                  
                  SizedBox(height: 8),
                  
                  TextButton.icon(
                    onPressed: _editPiConfiguration,
                    icon: Icon(Icons.edit),
                    label: Text('Edit Configuration'),
                  ),
                ],
              ),
            ),
          ),
          
          Spacer(),
          
          // Summary
          Card(
            color: Colors.purple.shade50,
            child: Padding(
              padding: EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    'Deployment Summary',
                    style: TextStyle(fontWeight: FontWeight.bold, color: Colors.purple.shade700),
                  ),
                  SizedBox(height: 8),
                  Text('• Map Name: ${_finalMapNameController.text}'),
                  Text('• Locations Added: ${_locationMarkers.length}'),
                  Text('• Auto-load: ${_autoLoad ? "Yes" : "No"}'),
                  Text('• Target: ${_piConfig['piHost'] ?? "Not configured"}'),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDeployStep() {
    return Padding(
      padding: EdgeInsets.all(20),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Step 4: Deploy to Raspberry Pi',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          ),
          SizedBox(height: 8),
          Text(
            'Deploy your edited map to the Raspberry Pi and start navigation.',
            style: TextStyle(color: Colors.grey.shade600),
          ),
          SizedBox(height: 20),
          
          Expanded(
            child: Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(
                    Icons.rocket_launch,
                    size: 64,
                    color: Colors.purple,
                  ),
                  SizedBox(height: 16),
                  Text(
                    'Ready to Deploy!',
                    style: TextStyle(
                      fontSize: 20,
                      fontWeight: FontWeight.bold,
                      color: Colors.purple.shade700,
                    ),
                  ),
                  SizedBox(height: 8),
                  Text(
                    'Your map has been converted and edited.\nClick "Deploy" to send it to the Raspberry Pi.',
                    textAlign: TextAlign.center,
                    style: TextStyle(color: Colors.grey.shade600),
                  ),
                  SizedBox(height: 32),
                  
                  ElevatedButton.icon(
                    onPressed: _isProcessing ? null : _deployMap,
                    icon: Icon(Icons.cloud_upload),
                    label: Text('Deploy to Raspberry Pi'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.purple,
                      foregroundColor: Colors.white,
                      padding: EdgeInsets.symmetric(horizontal: 32, vertical: 16),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMapInfoRow(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          Text('$label: ', style: TextStyle(fontWeight: FontWeight.w500)),
          Text(value, style: TextStyle(color: Colors.grey.shade700)),
        ],
      ),
    );
  }

  Widget _buildConfigRow(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          SizedBox(
            width: 80,
            child: Text('$label:', style: TextStyle(fontWeight: FontWeight.w500)),
          ),
          Expanded(
            child: Text(
              value,
              style: TextStyle(color: Colors.grey.shade700),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildLocationTile(LocationMarker marker, int index) {
    return Card(
      margin: EdgeInsets.only(bottom: 8),
      child: ListTile(
        leading: Container(
          width: 40,
          height: 40,
          decoration: BoxDecoration(
            color: Color(int.parse('FF${_apiService.getLocationTypeColor(marker.type).substring(1)}', radix: 16)),
            shape: BoxShape.circle,
          ),
          child: Center(
            child: Text(
              _apiService.getLocationTypeIcon(marker.type),
              style: TextStyle(fontSize: 16),
            ),
          ),
        ),
        title: Text(marker.name),
        subtitle: Text(
          '${marker.type.toUpperCase()} • (${marker.position.x.toStringAsFixed(2)}, ${marker.position.y.toStringAsFixed(2)})',
        ),
        trailing: IconButton(
          onPressed: () => _removeLocation(index),
          icon: Icon(Icons.delete, color: Colors.red),
        ),
      ),
    );
  }

  Widget _buildWorkflowActions() {
    return Container(
      padding: EdgeInsets.all(20),
      decoration: BoxDecoration(
        border: Border(top: BorderSide(color: Colors.grey.shade300)),
      ),
      child: Row(
        children: [
          if (_currentStep != WorkflowStep.convert)
            TextButton(
              onPressed: _isProcessing ? null : _previousStep,
              child: Text('Previous'),
            ),
          
          Spacer(),
          
          if (_currentStep != WorkflowStep.deploy)
            ElevatedButton(
              onPressed: _canProceedToNext() && !_isProcessing ? _nextStep : null,
              child: Text('Next'),
            ),
        ],
      ),
    );
  }

  // Helper methods
  IconData _getStepIcon(WorkflowStep step) {
    switch (step) {
      case WorkflowStep.convert:
        return Icons.transform;
      case WorkflowStep.edit:
        return Icons.edit_location;
      case WorkflowStep.configure:
        return Icons.settings;
      case WorkflowStep.deploy:
        return Icons.cloud_upload;
    }
  }

  String _getStepName(WorkflowStep step) {
    switch (step) {
      case WorkflowStep.convert:
        return 'Convert';
      case WorkflowStep.edit:
        return 'Edit';
      case WorkflowStep.configure:
        return 'Configure';
      case WorkflowStep.deploy:
        return 'Deploy';
    }
  }

  String _formatDateTime(DateTime dateTime) {
    return '${dateTime.day}/${dateTime.month}/${dateTime.year} ${dateTime.hour}:${dateTime.minute.toString().padLeft(2, '0')}';
  }

  bool _canProceedToNext() {
    switch (_currentStep) {
      case WorkflowStep.convert:
        return _convertedMapName != null;
      case WorkflowStep.edit:
        return true; // Can proceed with or without locations
      case WorkflowStep.configure:
        return _finalMapNameController.text.isNotEmpty;
      case WorkflowStep.deploy:
        return false; // No next step
    }
  }

  bool _canAddLocation() {
    return _locationNameController.text.isNotEmpty && _pendingLocationPosition != null;
  }

  // Action methods
  Future<void> _startConversion() async {
    setState(() {
      _isProcessing = true;
      _workflowError = null;
    });

    try {
      final result = await _apiService.convertMapToPGM(
        deviceId: widget.deviceId,
        mapName: 'workflow_${DateTime.now().millisecondsSinceEpoch}',
      );

      if (result['success'] == true) {
        setState(() {
          _convertedMapName = result['outputMapName'];
          _currentStep = WorkflowStep.edit;
        });
      } else {
        setState(() {
          _workflowError = result['error'] ?? 'Conversion failed';
        });
      }
    } catch (e) {
      setState(() {
        _workflowError = 'Conversion error: $e';
      });
    } finally {
      setState(() {
        _isProcessing = false;
      });
    }
  }

  void _selectLocationOnMap() {
    // TODO: Implement map tap selection
    // For now, set a dummy position
    setState(() {
      _pendingLocationPosition = Position(
        x: 0.0 + (_locationMarkers.length * 2.0), // Offset for demonstration
        y: 0.0 + (_locationMarkers.length * 1.5),
        z: 0.0,
      );
    });
    
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('Position selected: (${_pendingLocationPosition!.x.toStringAsFixed(2)}, ${_pendingLocationPosition!.y.toStringAsFixed(2)})'),
        duration: Duration(seconds: 2),
      ),
    );
  }

  void _addLocation() {
    if (!_canAddLocation()) return;

    final marker = LocationMarker(
      name: _locationNameController.text,
      type: _selectedLocationType.name,
      position: _pendingLocationPosition!,
    );

    setState(() {
      _locationMarkers.add(marker);
      _locationNameController.clear();
      _pendingLocationPosition = null;
    });
  }

  void _removeLocation(int index) {
    setState(() {
      _locationMarkers.removeAt(index);
    });
  }

  void _editPiConfiguration() {
    // TODO: Implement Pi configuration dialog
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('Pi configuration editing not implemented yet'),
        duration: Duration(seconds: 2),
      ),
    );
  }

  Future<void> _deployMap() async {
    setState(() {
      _isProcessing = true;
      _workflowError = null;
    });

    try {
      final locations = _locationMarkers.map((marker) {
        return _apiService.createLocationMarker(
          name: marker.name,
          type: marker.type,
          position: {
            'x': marker.position.x,
            'y': marker.position.y,
            'z': marker.position.z,
          },
        );
      }).toList();

      final result = await _apiService.completeMapDeploymentWorkflow(
        deviceId: widget.deviceId,
        finalMapName: _finalMapNameController.text,
        locations: locations,
        piConfig: _piConfig,
        deploymentNotes: _deploymentNotesController.text,
        autoLoad: _autoLoad,
      );

      if (result['success'] == true) {
        // Show success dialog
        showDialog(
          context: context,
          builder: (context) => AlertDialog(
            title: Row(
              children: [
                Icon(Icons.check_circle, color: Colors.green),
                SizedBox(width: 8),
                Text('Deployment Successful'),
              ],
            ),
            content: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('Your map has been successfully deployed to the Raspberry Pi!'),
                SizedBox(height: 12),
                Text('Map Name: ${_finalMapNameController.text}'),
                Text('Locations Added: ${_locationMarkers.length}'),
                Text('Auto-load: ${_autoLoad ? "Enabled" : "Disabled"}'),
              ],
            ),
            actions: [
              TextButton(
                onPressed: () {
                  Navigator.of(context).pop(); // Close dialog
                  Navigator.of(context).pop(); // Close workflow
                },
                child: Text('Close'),
              ),
              ElevatedButton(
                onPressed: () {
                  Navigator.of(context).pop(); // Close dialog
                  _showMapSelectionForNavigation();
                },
                child: Text('Start Navigation'),
              ),
            ],
          ),
        );
      } else {
        setState(() {
          _workflowError = result['error'] ?? 'Deployment failed';
        });
      }
    } catch (e) {
      setState(() {
        _workflowError = 'Deployment error: $e';
      });
    } finally {
      setState(() {
        _isProcessing = false;
      });
    }
  }

  void _showMapSelectionForNavigation() {
    showMapSelectionDialog(
      context: context,
      deviceId: widget.deviceId,
      onMapSelected: (mapName, mapSource) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('Navigation started with map: $mapName'),
            backgroundColor: Colors.green,
          ),
        );
      },
    );
  }

  void _nextStep() {
    final currentIndex = WorkflowStep.values.indexOf(_currentStep);
    if (currentIndex < WorkflowStep.values.length - 1) {
      setState(() {
        _currentStep = WorkflowStep.values[currentIndex + 1];
      });
    }
  }

  void _previousStep() {
    final currentIndex = WorkflowStep.values.indexOf(_currentStep);
    if (currentIndex > 0) {
      setState(() {
        _currentStep = WorkflowStep.values[currentIndex - 1];
      });
    }
  }
}

// Enums and data classes
enum WorkflowStep { convert, edit, configure, deploy }

enum LocationType { pickup, drop, charging, home, waypoint }

class LocationMarker {
  final String name;
  final String type;
  final Position position;

  LocationMarker({
    required this.name,
    required this.type,
    required this.position,
  });
}

// Utility function to show the workflow dialog
Future<void> showMapEditingWorkflow({
  required BuildContext context,
  required String deviceId,
  MapData? currentMapData,
  required Function(MapData) onMapUpdated,
}) async {
  return showDialog(
    context: context,
    barrierDismissible: false,
    builder: (context) => Dialog(
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Container(
        width: MediaQuery.of(context).size.width * 0.9,
        height: MediaQuery.of(context).size.height * 0.9,
        child: MapEditingWorkflow(
          deviceId: deviceId,
          currentMapData: currentMapData,
          onMapUpdated: onMapUpdated,
        ),
      ),
    ),
  );
}