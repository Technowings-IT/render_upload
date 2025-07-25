// widgets/enhanced_map_editing_workflow.dart - Complete Enhanced Workflow
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'dart:typed_data';
import '../services/api_service.dart';
import '../models/map_data.dart';
import 'pgm_map_editor.dart';

class EnhancedMapEditingWorkflow extends StatefulWidget {
  final String deviceId;
  final MapData? currentMapData;
  final Function(MapData) onMapUpdated;
  final VoidCallback? onWorkflowComplete;

  const EnhancedMapEditingWorkflow({
    Key? key,
    required this.deviceId,
    this.currentMapData,
    required this.onMapUpdated,
    this.onWorkflowComplete,
  }) : super(key: key);

  @override
  _EnhancedMapEditingWorkflowState createState() => _EnhancedMapEditingWorkflowState();
}

class _EnhancedMapEditingWorkflowState extends State<EnhancedMapEditingWorkflow>
    with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  
  late TabController _tabController;
  late AnimationController _progressAnimationController;
  late AnimationController _stepTransitionController;
  
  // Workflow state
  WorkflowStep _currentStep = WorkflowStep.preparation;
  bool _isProcessing = false;
  String? _workflowError;
  double _overallProgress = 0.0;
  
  // Step completion tracking
  Map<WorkflowStep, bool> _stepCompleted = {};
  Map<WorkflowStep, Map<String, dynamic>> _stepResults = {};
  
  // Map editing data
  String? _convertedMapName;
  MapData? _editingMapData;
  Uint8List? _editedMapImage;
  List<LocationPointData> _locationPoints = [];
  
  // Configuration
  final TextEditingController _finalMapNameController = TextEditingController();
  final TextEditingController _deploymentNotesController = TextEditingController();
  Map<String, dynamic> _piConfig = {};
  Map<String, dynamic> _exportSettings = {};
  bool _autoLoadOnPi = true;
  bool _createBackup = true;
  bool _validateBeforeDeploy = true;
  
  // Advanced options
  bool _optimizeForEditing = true;
  bool _enhanceContrast = false;
  bool _enableCollaboration = false;
  String _selectedExportFormat = 'pgm';
  
  @override
  void initState() {
    super.initState();
    
    _tabController = TabController(length: 6, vsync: this);
    
    _progressAnimationController = AnimationController(
      duration: Duration(milliseconds: 1000),
      vsync: this,
    );
    
    _stepTransitionController = AnimationController(
      duration: Duration(milliseconds: 300),
      vsync: this,
    );
    
    _initializeWorkflow();
  }

  @override
  void dispose() {
    _tabController.dispose();
    _progressAnimationController.dispose();
    _stepTransitionController.dispose();
    _finalMapNameController.dispose();
    _deploymentNotesController.dispose();
    super.dispose();
  }

  void _initializeWorkflow() {
    // Initialize step completion tracking
    for (final step in WorkflowStep.values) {
      _stepCompleted[step] = false;
      _stepResults[step] = {};
    }
    
    // Set default map name
    _finalMapNameController.text = 'edited_map_${widget.deviceId}_${DateTime.now().millisecondsSinceEpoch}';
    
    // Load initial configuration
    _loadWorkflowConfiguration();
  }

  Future<void> _loadWorkflowConfiguration() async {
    try {
      final config = await _apiService.getDeploymentConfig(widget.deviceId);
      if (config['success'] == true) {
        setState(() {
          _piConfig = config['config'] ?? {};
        });
      }
    } catch (e) {
      print('Failed to load configuration: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Dialog(
      backgroundColor: Colors.transparent,
      child: Container(
        width: MediaQuery.of(context).size.width * 0.95,
        height: MediaQuery.of(context).size.height * 0.9,
        decoration: BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.circular(20),
          boxShadow: [
            BoxShadow(
              color: Colors.black.withOpacity(0.3),
              blurRadius: 20,
              offset: Offset(0, 10),
            ),
          ],
        ),
        child: Column(
          children: [
            _buildEnhancedHeader(),
            _buildProgressIndicator(),
            _buildStepNavigation(),
            Expanded(child: _buildStepContent()),
            _buildActionBar(),
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedHeader() {
    return Container(
      padding: EdgeInsets.all(24),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.deepPurple.shade800,
            Colors.deepPurple.shade600,
            Colors.purple.shade500,
          ],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      child: Row(
        children: [
          // Logo/Icon
          Container(
            width: 60,
            height: 60,
            decoration: BoxDecoration(
              color: Colors.white.withOpacity(0.2),
              borderRadius: BorderRadius.circular(15),
            ),
            child: Icon(
              Icons.auto_fix_high,
              color: Colors.white,
              size: 32,
            ),
          ),
          
          SizedBox(width: 20),
          
          // Title and description
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Enhanced Map Editor',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 24,
                    fontWeight: FontWeight.bold,
                    letterSpacing: 0.5,
                  ),
                ),
                SizedBox(height: 4),
                Text(
                  'Professional-grade map editing with GIMP-like tools',
                  style: TextStyle(
                    color: Colors.white.withOpacity(0.9),
                    fontSize: 14,
                    fontWeight: FontWeight.w400,
                  ),
                ),
                SizedBox(height: 8),
                
                // Feature highlights
                Wrap(
                  spacing: 12,
                  children: [
                    _buildFeatureChip('🎨 Advanced Tools'),
                    _buildFeatureChip('📍 Location Points'),
                    _buildFeatureChip('🚀 Auto Deploy'),
                    _buildFeatureChip('💾 Backup & Restore'),
                  ],
                ),
              ],
            ),
          ),
          
          // Status indicator
          if (_isProcessing)
            Container(
              width: 40,
              height: 40,
              child: CircularProgressIndicator(
                strokeWidth: 3,
                valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
              ),
            )
          else
            IconButton(
              onPressed: () => Navigator.of(context).pop(),
              icon: Icon(Icons.close, color: Colors.white, size: 28),
              tooltip: 'Close Editor',
            ),
        ],
      ),
    );
  }

  Widget _buildFeatureChip(String text) {
    return Container(
      padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.2),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.white.withOpacity(0.3)),
      ),
      child: Text(
        text,
        style: TextStyle(
          color: Colors.white,
          fontSize: 11,
          fontWeight: FontWeight.w500,
        ),
      ),
    );
  }

  Widget _buildProgressIndicator() {
    return Container(
      padding: EdgeInsets.all(20),
      child: Column(
        children: [
          // Overall progress bar
          Row(
            children: [
              Icon(Icons.timeline, color: Colors.deepPurple),
              SizedBox(width: 12),
              Text(
                'Workflow Progress',
                style: TextStyle(
                  fontSize: 16,
                  fontWeight: FontWeight.bold,
                  color: Colors.deepPurple.shade700,
                ),
              ),
              Spacer(),
              Text(
                '${(_overallProgress * 100).toInt()}%',
                style: TextStyle(
                  fontSize: 14,
                  fontWeight: FontWeight.bold,
                  color: Colors.deepPurple.shade600,
                ),
              ),
            ],
          ),
          
          SizedBox(height: 12),
          
          // Animated progress bar
          AnimatedBuilder(
            animation: _progressAnimationController,
            builder: (context, child) {
              return LinearProgressIndicator(
                value: _overallProgress * _progressAnimationController.value,
                backgroundColor: Colors.deepPurple.shade100,
                valueColor: AlwaysStoppedAnimation<Color>(Colors.deepPurple),
                minHeight: 8,
              );
            },
          ),
          
          SizedBox(height: 16),
          
          // Step indicators
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: WorkflowStep.values.map((step) {
              return _buildStepIndicator(step);
            }).toList(),
          ),
        ],
      ),
    );
  }

  Widget _buildStepIndicator(WorkflowStep step) {
    final isActive = step == _currentStep;
    final isCompleted = _stepCompleted[step] == true;
    final stepIndex = WorkflowStep.values.indexOf(step);
    final currentIndex = WorkflowStep.values.indexOf(_currentStep);
    
    Color backgroundColor;
    Color iconColor;
    IconData icon;
    
    if (isCompleted) {
      backgroundColor = Colors.green;
      iconColor = Colors.white;
      icon = Icons.check;
    } else if (isActive) {
      backgroundColor = Colors.deepPurple;
      iconColor = Colors.white;
      icon = _getStepIcon(step);
    } else if (stepIndex < currentIndex) {
      backgroundColor = Colors.grey.shade400;
      iconColor = Colors.white;
      icon = _getStepIcon(step);
    } else {
      backgroundColor = Colors.grey.shade200;
      iconColor = Colors.grey.shade600;
      icon = _getStepIcon(step);
    }
    
    return Column(
      children: [
        AnimatedContainer(
          duration: Duration(milliseconds: 300),
          width: isActive ? 56 : 48,
          height: isActive ? 56 : 48,
          decoration: BoxDecoration(
            color: backgroundColor,
            shape: BoxShape.circle,
            boxShadow: isActive ? [
              BoxShadow(
                color: backgroundColor.withOpacity(0.4),
                blurRadius: 8,
                offset: Offset(0, 4),
              ),
            ] : null,
          ),
          child: Icon(
            icon,
            color: iconColor,
            size: isActive ? 28 : 24,
          ),
        ),
        
        SizedBox(height: 8),
        
        Text(
          _getStepName(step),
          style: TextStyle(
            fontSize: 10,
            fontWeight: isActive ? FontWeight.bold : FontWeight.normal,
            color: isActive ? Colors.deepPurple.shade700 : Colors.grey.shade600,
          ),
          textAlign: TextAlign.center,
        ),
      ],
    );
  }

  Widget _buildStepNavigation() {
    return Container(
      height: 50,
      child: TabBar(
        controller: _tabController,
        isScrollable: true,
        labelColor: Colors.deepPurple,
        unselectedLabelColor: Colors.grey,
        indicatorColor: Colors.deepPurple,
        indicatorWeight: 3,
        labelStyle: TextStyle(fontWeight: FontWeight.bold),
        tabs: WorkflowStep.values.map((step) {
          return Tab(
            text: _getStepName(step),
            icon: Icon(_getStepIcon(step), size: 20),
          );
        }).toList(),
        onTap: (index) {
          _navigateToStep(WorkflowStep.values[index]);
        },
      ),
    );
  }

  Widget _buildStepContent() {
    if (_workflowError != null) {
      return _buildErrorView();
    }

    return AnimatedSwitcher(
      duration: Duration(milliseconds: 300),
      child: _getStepWidget(_currentStep),
    );
  }

  Widget _getStepWidget(WorkflowStep step) {
    switch (step) {
      case WorkflowStep.preparation:
        return _buildPreparationStep();
      case WorkflowStep.conversion:
        return _buildConversionStep();
      case WorkflowStep.editing:
        return _buildEditingStep();
      case WorkflowStep.annotation:
        return _buildAnnotationStep();
      case WorkflowStep.validation:
        return _buildValidationStep();
      case WorkflowStep.deployment:
        return _buildDeploymentStep();
    }
  }

  Widget _buildPreparationStep() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(24),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildStepHeader(
            'Preparation',
            'Configure your editing session and verify map data',
            Icons.settings,
          ),
          
          SizedBox(height: 24),
          
          // Current map status
          _buildStatusCard(
            'Current Map Status',
            widget.currentMapData != null 
              ? _buildMapStatusContent()
              : _buildNoMapContent(),
            widget.currentMapData != null ? Colors.green : Colors.orange,
          ),
          
          SizedBox(height: 20),
          
          // Editing options
          _buildOptionsCard(
            'Editing Options',
            [
              _buildSwitchOption(
                'Optimize for Editing',
                'Enhance map for better editing experience',
                _optimizeForEditing,
                (value) => setState(() => _optimizeForEditing = value),
              ),
              _buildSwitchOption(
                'Enhance Contrast',
                'Improve visibility of map features',
                _enhanceContrast,
                (value) => setState(() => _enhanceContrast = value),
              ),
              _buildSwitchOption(
                'Enable Collaboration',
                'Allow real-time collaborative editing',
                _enableCollaboration,
                (value) => setState(() => _enableCollaboration = value),
              ),
            ],
          ),
          
          SizedBox(height: 20),
          
          // Export format selection
          _buildSelectionCard(
            'Export Format',
            'Choose the output format for your edited map',
            ['pgm', 'png', 'pdf', 'svg'],
            _selectedExportFormat,
            (value) => setState(() => _selectedExportFormat = value),
          ),
        ],
      ),
    );
  }

  Widget _buildConversionStep() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(24),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildStepHeader(
            'Map Conversion',
            'Convert your map to an editable format',
            Icons.transform,
          ),
          
          SizedBox(height: 24),
          
          if (!_stepCompleted[WorkflowStep.conversion]!) ...[
            // Conversion settings
            Card(
              elevation: 2,
              child: Padding(
                padding: EdgeInsets.all(20),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Conversion Settings',
                      style: TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                    
                    SizedBox(height: 16),
                    
                    TextField(
                      controller: _finalMapNameController,
                      decoration: InputDecoration(
                        labelText: 'Map Name',
                        hintText: 'Enter a name for your edited map',
                        border: OutlineInputBorder(),
                        prefixIcon: Icon(Icons.label),
                      ),
                    ),
                    
                    SizedBox(height: 16),
                    
                    // Advanced conversion options
                    ExpansionTile(
                      title: Text('Advanced Options'),
                      children: [
                        CheckboxListTile(
                          title: Text('Preserve Original Quality'),
                          subtitle: Text('Maintain highest possible quality'),
                          value: true,
                          onChanged: null,
                        ),
                        CheckboxListTile(
                          title: Text('Generate Thumbnails'),
                          subtitle: Text('Create preview images'),
                          value: true,
                          onChanged: null,
                        ),
                      ],
                    ),
                  ],
                ),
              ),
            ),
            
            SizedBox(height: 24),
            
            // Start conversion button
            Center(
              child: ElevatedButton.icon(
                onPressed: _isProcessing ? null : _startConversion,
                icon: Icon(Icons.play_arrow),
                label: Text('Start Conversion'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.deepPurple,
                  foregroundColor: Colors.white,
                  padding: EdgeInsets.symmetric(horizontal: 32, vertical: 16),
                  textStyle: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
                ),
              ),
            ),
          ] else ...[
            // Conversion completed
            _buildSuccessCard(
              'Conversion Completed',
              'Your map has been successfully converted and is ready for editing.',
              _stepResults[WorkflowStep.conversion]!,
            ),
          ],
        ],
      ),
    );
  }

  Widget _buildEditingStep() {
    return Column(
      children: [
        Container(
          padding: EdgeInsets.all(16),
          child: Row(
            children: [
              Icon(Icons.edit, color: Colors.deepPurple),
              SizedBox(width: 12),
              Text(
                'Map Editor',
                style: TextStyle(
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                  color: Colors.deepPurple.shade700,
                ),
              ),
              Spacer(),
              ElevatedButton.icon(
                onPressed: _openFullEditor,
                icon: Icon(Icons.open_in_full),
                label: Text('Full Editor'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.deepPurple,
                  foregroundColor: Colors.white,
                ),
              ),
            ],
          ),
        ),
        
        Expanded(
          child: _editingMapData != null
            ? EnhancedPGMMapEditor(
                mapData: _editingMapData,
                onMapChanged: (updatedMap) {
                  setState(() {
                    _editingMapData = updatedMap;
                    _stepCompleted[WorkflowStep.editing] = true;
                  });
                  _updateProgress();
                },
                deviceId: widget.deviceId,
              )
            : Center(
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Icon(Icons.map, size: 64, color: Colors.grey),
                    SizedBox(height: 16),
                    Text(
                      'Complete conversion step first',
                      style: TextStyle(
                        fontSize: 16,
                        color: Colors.grey.shade600,
                      ),
                    ),
                  ],
                ),
              ),
        ),
      ],
    );
  }

  Widget _buildAnnotationStep() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(24),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildStepHeader(
            'Location Points',
            'Add pickup, drop, charging, and home positions',
            Icons.place,
          ),
          
          SizedBox(height: 24),
          
          // Location points management
          Row(
            children: [
              Expanded(
                flex: 2,
                child: _buildLocationPointsList(),
              ),
              SizedBox(width: 20),
              Expanded(
                flex: 1,
                child: _buildLocationPointsTools(),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildValidationStep() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(24),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildStepHeader(
            'Validation',
            'Verify your map before deployment',
            Icons.verified,
          ),
          
          SizedBox(height: 24),
          
          // Validation options
          Card(
            elevation: 2,
            child: Padding(
              padding: EdgeInsets.all(20),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    'Validation Checks',
                    style: TextStyle(
                      fontSize: 18,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  
                  SizedBox(height: 16),
                  
                  CheckboxListTile(
                    title: Text('Map Quality Check'),
                    subtitle: Text('Verify map quality and completeness'),
                    value: true,
                    onChanged: null,
                  ),
                  
                  CheckboxListTile(
                    title: Text('Location Accessibility'),
                    subtitle: Text('Ensure all points are reachable'),
                    value: true,
                    onChanged: null,
                  ),
                  
                  CheckboxListTile(
                    title: Text('Path Planning Test'),
                    subtitle: Text('Test navigation between points'),
                    value: _validateBeforeDeploy,
                    onChanged: (value) => setState(() => _validateBeforeDeploy = value!),
                  ),
                  
                  SizedBox(height: 20),
                  
                  Center(
                    child: ElevatedButton.icon(
                      onPressed: _isProcessing ? null : _runValidation,
                      icon: Icon(Icons.play_arrow),
                      label: Text('Run Validation'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.green,
                        foregroundColor: Colors.white,
                        padding: EdgeInsets.symmetric(horizontal: 32, vertical: 16),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
          
          if (_stepCompleted[WorkflowStep.validation]!) ...[
            SizedBox(height: 20),
            _buildValidationResults(),
          ],
        ],
      ),
    );
  }

  Widget _buildDeploymentStep() {
    return SingleChildScrollView(
      padding: EdgeInsets.all(24),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _buildStepHeader(
            'Deployment',
            'Deploy your edited map to the Raspberry Pi',
            Icons.cloud_upload,
          ),
          
          SizedBox(height: 24),
          
          // Deployment configuration
          Row(
            children: [
              Expanded(
                child: _buildDeploymentConfig(),
              ),
              SizedBox(width: 20),
              Expanded(
                child: _buildDeploymentSummary(),
              ),
            ],
          ),
          
          SizedBox(height: 24),
          
          // Deploy button
          Center(
            child: ElevatedButton.icon(
              onPressed: _isProcessing ? null : _deployMap,
              icon: Icon(Icons.rocket_launch),
              label: Text('Deploy to Raspberry Pi'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.deepPurple,
                foregroundColor: Colors.white,
                padding: EdgeInsets.symmetric(horizontal: 40, vertical: 20),
                textStyle: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
              ),
            ),
          ),
        ],
      ),
    );
  }

  // Helper widget builders
  Widget _buildStepHeader(String title, String description, IconData icon) {
    return Row(
      children: [
        Container(
          width: 60,
          height: 60,
          decoration: BoxDecoration(
            color: Colors.deepPurple.shade100,
            borderRadius: BorderRadius.circular(15),
          ),
          child: Icon(
            icon,
            color: Colors.deepPurple,
            size: 30,
          ),
        ),
        
        SizedBox(width: 16),
        
        Expanded(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                title,
                style: TextStyle(
                  fontSize: 22,
                  fontWeight: FontWeight.bold,
                  color: Colors.deepPurple.shade700,
                ),
              ),
              SizedBox(height: 4),
              Text(
                description,
                style: TextStyle(
                  fontSize: 14,
                  color: Colors.grey.shade600,
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildStatusCard(String title, Widget content, Color accentColor) {
    return Card(
      elevation: 2,
      child: Container(
        padding: EdgeInsets.all(20),
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(8),
          border: Border(
            left: BorderSide(color: accentColor, width: 4),
          ),
        ),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.info, color: accentColor),
                SizedBox(width: 8),
                Text(
                  title,
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                    color: accentColor,
                  ),
                ),
              ],
            ),
            SizedBox(height: 16),
            content,
          ],
        ),
      ),
    );
  }

  Widget _buildMapStatusContent() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        _buildInfoRow('Device ID', widget.currentMapData!.deviceId),
        _buildInfoRow('Dimensions', '${widget.currentMapData!.info.width}×${widget.currentMapData!.info.height}'),
        _buildInfoRow('Resolution', '${widget.currentMapData!.info.resolution} m/px'),
        _buildInfoRow('Shapes', '${widget.currentMapData!.shapes.length}'),
        _buildInfoRow('Last Updated', _formatDateTime(widget.currentMapData!.timestamp)),
      ],
    );
  }

  Widget _buildNoMapContent() {
    return Column(
      children: [
        Icon(Icons.warning, color: Colors.orange, size: 48),
        SizedBox(height: 12),
        Text(
          'No map data available',
          style: TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
            color: Colors.orange.shade700,
          ),
        ),
        SizedBox(height: 8),
        Text(
          'You can still create a new map or load from templates.',
          style: TextStyle(color: Colors.grey.shade600),
        ),
      ],
    );
  }

  Widget _buildOptionsCard(String title, List<Widget> options) {
    return Card(
      elevation: 2,
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              title,
              style: TextStyle(
                fontSize: 16,
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: 16),
            ...options,
          ],
        ),
      ),
    );
  }

  Widget _buildSwitchOption(String title, String description, bool value, Function(bool) onChanged) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 8),
      child: Row(
        children: [
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: TextStyle(
                    fontWeight: FontWeight.w500,
                  ),
                ),
                SizedBox(height: 4),
                Text(
                  description,
                  style: TextStyle(
                    fontSize: 12,
                    color: Colors.grey.shade600,
                  ),
                ),
              ],
            ),
          ),
          Switch(
            value: value,
            onChanged: onChanged,
            activeColor: Colors.deepPurple,
          ),
        ],
      ),
    );
  }

  Widget _buildSelectionCard(String title, String description, List<String> options, String selected, Function(String) onChanged) {
    return Card(
      elevation: 2,
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              title,
              style: TextStyle(
                fontSize: 16,
                fontWeight: FontWeight.bold,
              ),
            ),
            SizedBox(height: 8),
            Text(
              description,
              style: TextStyle(
                color: Colors.grey.shade600,
              ),
            ),
            SizedBox(height: 16),
            Wrap(
              spacing: 8,
              children: options.map((option) {
                final isSelected = option == selected;
                return ChoiceChip(
                  label: Text(option.toUpperCase()),
                  selected: isSelected,
                  onSelected: (selected) {
                    if (selected) onChanged(option);
                  },
                  selectedColor: Colors.deepPurple.shade100,
                  labelStyle: TextStyle(
                    color: isSelected ? Colors.deepPurple : Colors.grey.shade700,
                    fontWeight: isSelected ? FontWeight.bold : FontWeight.normal,
                  ),
                );
              }).toList(),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSuccessCard(String title, String description, Map<String, dynamic> results) {
    return Card(
      elevation: 2,
      color: Colors.green.shade50,
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.check_circle, color: Colors.green, size: 32),
                SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        title,
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: Colors.green.shade700,
                        ),
                      ),
                      SizedBox(height: 4),
                      Text(
                        description,
                        style: TextStyle(
                          color: Colors.green.shade600,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            
            if (results.isNotEmpty) ...[
              SizedBox(height: 16),
              ...results.entries.map((entry) {
                return Padding(
                  padding: EdgeInsets.symmetric(vertical: 2),
                  child: Row(
                    children: [
                      Text('${entry.key}: ', style: TextStyle(fontWeight: FontWeight.w500)),
                      Text('${entry.value}'),
                    ],
                  ),
                );
              }).toList(),
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildLocationPointsList() {
    return Card(
      elevation: 2,
      child: Column(
        children: [
          Container(
            padding: EdgeInsets.all(16),
            decoration: BoxDecoration(
              color: Colors.deepPurple.shade50,
              borderRadius: BorderRadius.vertical(top: Radius.circular(8)),
            ),
            child: Row(
              children: [
                Icon(Icons.place, color: Colors.deepPurple),
                SizedBox(width: 8),
                Text(
                  'Location Points (${_locationPoints.length})',
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                    color: Colors.deepPurple.shade700,
                  ),
                ),
              ],
            ),
          ),
          
          Container(
            height: 300,
            child: _locationPoints.isEmpty
              ? Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Icon(Icons.location_off, size: 48, color: Colors.grey),
                      SizedBox(height: 12),
                      Text(
                        'No location points added yet',
                        style: TextStyle(color: Colors.grey.shade600),
                      ),
                    ],
                  ),
                )
              : ListView.builder(
                  itemCount: _locationPoints.length,
                  itemBuilder: (context, index) {
                    return _buildLocationPointItem(_locationPoints[index], index);
                  },
                ),
          ),
        ],
      ),
    );
  }

  Widget _buildLocationPointItem(LocationPointData point, int index) {
    final typeColor = _getLocationTypeColor(point.type);
    
    return ListTile(
      leading: Container(
        width: 40,
        height: 40,
        decoration: BoxDecoration(
          color: typeColor.withOpacity(0.2),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: typeColor),
        ),
        child: Center(
          child: Text(
            _getLocationTypeIcon(point.type),
            style: TextStyle(fontSize: 18),
          ),
        ),
      ),
      title: Text(point.name),
      subtitle: Text(
        '${point.type.toUpperCase()} • (${point.position['x']?.toStringAsFixed(1)}, ${point.position['y']?.toStringAsFixed(1)})',
      ),
      trailing: PopupMenuButton<String>(
        onSelected: (value) {
          switch (value) {
            case 'edit':
              _editLocationPoint(point);
              break;
            case 'delete':
              _deleteLocationPoint(index);
              break;
          }
        },
        itemBuilder: (context) => [
          PopupMenuItem(value: 'edit', child: Text('Edit')),
          PopupMenuItem(value: 'delete', child: Text('Delete')),
        ],
      ),
    );
  }

  Widget _buildLocationPointsTools() {
    return Column(
      children: [
        Card(
          elevation: 2,
          child: Padding(
            padding: EdgeInsets.all(16),
            child: Column(
              children: [
                Text(
                  'Add Location Point',
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                
                SizedBox(height: 16),
                
                ...LocationPointType.values.map((type) {
                  return Padding(
                    padding: EdgeInsets.symmetric(vertical: 4),
                    child: ElevatedButton.icon(
                      onPressed: () => _addLocationPoint(type),
                      icon: Text(_getLocationTypeIcon(type.name)),
                      label: Text(type.displayName),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: _getLocationTypeColor(type.name).withOpacity(0.1),
                        foregroundColor: _getLocationTypeColor(type.name),
                        minimumSize: Size(double.infinity, 40),
                      ),
                    ),
                  );
                }).toList(),
              ],
            ),
          ),
        ),
        
        SizedBox(height: 16),
        
        Card(
          elevation: 2,
          child: Padding(
            padding: EdgeInsets.all(16),
            child: Column(
              children: [
                Text(
                  'Quick Actions',
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                
                SizedBox(height: 16),
                
                ElevatedButton.icon(
                  onPressed: _clearAllLocationPoints,
                  icon: Icon(Icons.clear_all),
                  label: Text('Clear All'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.red.shade100,
                    foregroundColor: Colors.red,
                    minimumSize: Size(double.infinity, 40),
                  ),
                ),
                
                SizedBox(height: 8),
                
                ElevatedButton.icon(
                  onPressed: _importLocationPoints,
                  icon: Icon(Icons.file_upload),
                  label: Text('Import'),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.blue.shade100,
                    foregroundColor: Colors.blue,
                    minimumSize: Size(double.infinity, 40),
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildValidationResults() {
    final results = _stepResults[WorkflowStep.validation]!;
    final isValid = results['valid'] == true;
    
    return Card(
      elevation: 2,
      color: isValid ? Colors.green.shade50 : Colors.red.shade50,
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(
                  isValid ? Icons.check_circle : Icons.error,
                  color: isValid ? Colors.green : Colors.red,
                  size: 32,
                ),
                SizedBox(width: 12),
                Text(
                  isValid ? 'Validation Passed' : 'Validation Failed',
                  style: TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                    color: isValid ? Colors.green.shade700 : Colors.red.shade700,
                  ),
                ),
              ],
            ),
            
            SizedBox(height: 16),
            
            if (results['details'] != null) ...[
              ...results['details'].entries.map((entry) {
                return _buildValidationDetail(entry.key, entry.value);
              }).toList(),
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildValidationDetail(String check, dynamic result) {
    final passed = result['passed'] == true;
    
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          Icon(
            passed ? Icons.check : Icons.close,
            color: passed ? Colors.green : Colors.red,
            size: 20,
          ),
          SizedBox(width: 8),
          Expanded(
            child: Text(
              check.replaceAll('_', ' ').toUpperCase(),
              style: TextStyle(
                fontWeight: FontWeight.w500,
              ),
            ),
          ),
          if (result['score'] != null)
            Text(
              '${result['score']}%',
              style: TextStyle(
                fontWeight: FontWeight.bold,
                color: passed ? Colors.green : Colors.red,
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildDeploymentConfig() {
    return Card(
      elevation: 2,
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Deployment Configuration',
              style: TextStyle(
                fontSize: 16,
                fontWeight: FontWeight.bold,
              ),
            ),
            
            SizedBox(height: 16),
            
            TextField(
              controller: _deploymentNotesController,
              decoration: InputDecoration(
                labelText: 'Deployment Notes',
                hintText: 'Add notes about this deployment...',
                border: OutlineInputBorder(),
                prefixIcon: Icon(Icons.note),
              ),
              maxLines: 3,
            ),
            
            SizedBox(height: 16),
            
            CheckboxListTile(
              title: Text('Auto-load on Raspberry Pi'),
              subtitle: Text('Automatically activate the map after deployment'),
              value: _autoLoadOnPi,
              onChanged: (value) => setState(() => _autoLoadOnPi = value!),
            ),
            
            CheckboxListTile(
              title: Text('Create Backup'),
              subtitle: Text('Backup existing map before deployment'),
              value: _createBackup,
              onChanged: (value) => setState(() => _createBackup = value!),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildDeploymentSummary() {
    return Card(
      elevation: 2,
      color: Colors.deepPurple.shade50,
      child: Padding(
        padding: EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Deployment Summary',
              style: TextStyle(
                fontSize: 16,
                fontWeight: FontWeight.bold,
                color: Colors.deepPurple.shade700,
              ),
            ),
            
            SizedBox(height: 16),
            
            _buildSummaryItem('Map Name', _finalMapNameController.text),
            _buildSummaryItem('Device ID', widget.deviceId),
            _buildSummaryItem('Location Points', '${_locationPoints.length}'),
            _buildSummaryItem('Export Format', _selectedExportFormat.toUpperCase()),
            _buildSummaryItem('Auto-load', _autoLoadOnPi ? 'Yes' : 'No'),
            _buildSummaryItem('Backup', _createBackup ? 'Yes' : 'No'),
            
            if (_piConfig.isNotEmpty) ...[
              SizedBox(height: 12),
              Text(
                'Raspberry Pi:',
                style: TextStyle(fontWeight: FontWeight.bold),
              ),
              _buildSummaryItem('Host', _piConfig['piHost'] ?? 'Not configured'),
              _buildSummaryItem('User', _piConfig['piUser'] ?? 'Not configured'),
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildSummaryItem(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          Text('$label: ', style: TextStyle(fontWeight: FontWeight.w500)),
          Expanded(
            child: Text(
              value,
              style: TextStyle(color: Colors.grey.shade700),
              overflow: TextOverflow.ellipsis,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildErrorView() {
    return Center(
      child: Padding(
        padding: EdgeInsets.all(32),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.error, size: 64, color: Colors.red),
            SizedBox(height: 16),
            Text(
              'Workflow Error',
              style: TextStyle(
                fontSize: 24,
                fontWeight: FontWeight.bold,
                color: Colors.red.shade700,
              ),
            ),
            SizedBox(height: 8),
            Text(
              _workflowError!,
              textAlign: TextAlign.center,
              style: TextStyle(
                color: Colors.red.shade600,
                fontSize: 16,
              ),
            ),
            SizedBox(height: 24),
            ElevatedButton.icon(
              onPressed: () {
                setState(() {
                  _workflowError = null;
                  _currentStep = WorkflowStep.preparation;
                });
              },
              icon: Icon(Icons.refresh),
              label: Text('Restart Workflow'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.red,
                foregroundColor: Colors.white,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildActionBar() {
    return Container(
      padding: EdgeInsets.all(20),
      decoration: BoxDecoration(
        color: Colors.grey[50],
        border: Border(top: BorderSide(color: Colors.grey[300]!)),
        borderRadius: BorderRadius.vertical(bottom: Radius.circular(20)),
      ),
      child: Row(
        children: [
          // Help button
          TextButton.icon(
            onPressed: _showHelp,
            icon: Icon(Icons.help_outline),
            label: Text('Help'),
          ),
          
          SizedBox(width: 16),
          
          // Reset button
          TextButton.icon(
            onPressed: _resetWorkflow,
            icon: Icon(Icons.refresh),
            label: Text('Reset'),
          ),
          
          Spacer(),
          
          // Navigation buttons
          if (_currentStep != WorkflowStep.preparation)
            TextButton(
              onPressed: _isProcessing ? null : _previousStep,
              child: Text('Previous'),
            ),
          
          SizedBox(width: 16),
          
          if (_currentStep != WorkflowStep.deployment)
            ElevatedButton(
              onPressed: _canProceedToNext() && !_isProcessing ? _nextStep : null,
              child: Text('Next'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.deepPurple,
                foregroundColor: Colors.white,
              ),
            )
          else
            ElevatedButton(
              onPressed: _allStepsCompleted() && !_isProcessing ? _completeWorkflow : null,
              child: Text('Complete'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.green,
                foregroundColor: Colors.white,
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildInfoRow(String label, String value) {
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

  // Utility methods
  IconData _getStepIcon(WorkflowStep step) {
    switch (step) {
      case WorkflowStep.preparation:
        return Icons.settings;
      case WorkflowStep.conversion:
        return Icons.transform;
      case WorkflowStep.editing:
        return Icons.edit;
      case WorkflowStep.annotation:
        return Icons.place;
      case WorkflowStep.validation:
        return Icons.verified;
      case WorkflowStep.deployment:
        return Icons.cloud_upload;
    }
  }

  String _getStepName(WorkflowStep step) {
    switch (step) {
      case WorkflowStep.preparation:
        return 'Preparation';
      case WorkflowStep.conversion:
        return 'Conversion';
      case WorkflowStep.editing:
        return 'Editing';
      case WorkflowStep.annotation:
        return 'Annotation';
      case WorkflowStep.validation:
        return 'Validation';
      case WorkflowStep.deployment:
        return 'Deployment';
    }
  }

  String _formatDateTime(DateTime dateTime) {
    return '${dateTime.day}/${dateTime.month}/${dateTime.year} ${dateTime.hour}:${dateTime.minute.toString().padLeft(2, '0')}';
  }

  Color _getLocationTypeColor(String type) {
    switch (type.toLowerCase()) {
      case 'pickup':
        return Colors.green;
      case 'drop':
        return Colors.blue;
      case 'home':
        return Colors.orange;
      case 'charging':
        return Colors.yellow.shade700;
      case 'waypoint':
        return Colors.purple;
      default:
        return Colors.grey;
    }
  }

  String _getLocationTypeIcon(String type) {
    switch (type.toLowerCase()) {
      case 'pickup':
        return '📦';
      case 'drop':
        return '📍';
      case 'home':
        return '🏠';
      case 'charging':
        return '🔋';
      case 'waypoint':
        return '📌';
      default:
        return '📍';
    }
  }

  bool _canProceedToNext() {
    switch (_currentStep) {
      case WorkflowStep.preparation:
        return true; // Always can proceed from preparation
      case WorkflowStep.conversion:
        return _stepCompleted[WorkflowStep.conversion] == true;
      case WorkflowStep.editing:
        return _editingMapData != null;
      case WorkflowStep.annotation:
        return true; // Can proceed with or without location points
      case WorkflowStep.validation:
        return _stepCompleted[WorkflowStep.validation] == true;
      case WorkflowStep.deployment:
        return false; // No next step
    }
  }

  bool _allStepsCompleted() {
    return _stepCompleted.values.every((completed) => completed);
  }

  void _updateProgress() {
    final completedSteps = _stepCompleted.values.where((completed) => completed).length;
    final newProgress = completedSteps / WorkflowStep.values.length;
    
    setState(() {
      _overallProgress = newProgress;
    });
    
    _progressAnimationController.animateTo(1.0);
  }

  // Action methods
  void _navigateToStep(WorkflowStep step) {
    setState(() {
      _currentStep = step;
    });
    _tabController.animateTo(WorkflowStep.values.indexOf(step));
  }

  void _nextStep() {
    final currentIndex = WorkflowStep.values.indexOf(_currentStep);
    if (currentIndex < WorkflowStep.values.length - 1) {
      _navigateToStep(WorkflowStep.values[currentIndex + 1]);
    }
  }

  void _previousStep() {
    final currentIndex = WorkflowStep.values.indexOf(_currentStep);
    if (currentIndex > 0) {
      _navigateToStep(WorkflowStep.values[currentIndex - 1]);
    }
  }

  Future<void> _startConversion() async {
    setState(() {
      _isProcessing = true;
      _workflowError = null;
    });

    try {
      final result = await _apiService.convertMapToPGMAdvanced(
        deviceId: widget.deviceId,
        mapName: _finalMapNameController.text,
        optimizeForEditing: _optimizeForEditing,
        // conversionOptions: {
        //   'enhanceContrast': _enhanceContrast,
        //   'preserveDetails': true,
        // },
      );

      if (result['success'] == true) {
        setState(() {
          _convertedMapName = result['outputMapName'];
          _editingMapData = widget.currentMapData; // Use existing or converted data
          _stepCompleted[WorkflowStep.conversion] = true;
          _stepResults[WorkflowStep.conversion] = {
            'mapName': result['outputMapName'],
            'fileSize': result['editingMetadata']?['fileSize'] ?? 'Unknown',
            'optimized': _optimizeForEditing.toString(),
          };
        });
        _updateProgress();
        _nextStep();
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

  void _openFullEditor() {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (context) => Scaffold(
          appBar: AppBar(
            title: Text('Enhanced Map Editor'),
            backgroundColor: Colors.deepPurple,
            foregroundColor: Colors.white,
          ),
          body: EnhancedPGMMapEditor(
            mapData: _editingMapData,
            onMapChanged: (updatedMap) {
              setState(() {
                _editingMapData = updatedMap;
                _stepCompleted[WorkflowStep.editing] = true;
              });
              _updateProgress();
            },
            deviceId: widget.deviceId,
          ),
        ),
      ),
    );
  }

  Future<void> _runValidation() async {
    setState(() {
      _isProcessing = true;
    });

    try {
      final result = await _apiService.validateMapQuality(
        deviceId: widget.deviceId,
        mapName: _convertedMapName ?? _finalMapNameController.text,
        validationRules: [
          'connectivity',
          'reachability',
          'obstacleConsistency',
          'locationPointAccessibility',
          'navigationSafety',
        ],
      );

      setState(() {
        _stepCompleted[WorkflowStep.validation] = result['validation']['valid'];
        _stepResults[WorkflowStep.validation] = result['validation'];
      });
      _updateProgress();
    } catch (e) {
      setState(() {
        _workflowError = 'Validation error: $e';
      });
    } finally {
      setState(() {
        _isProcessing = false;
      });
    }
  }

  Future<void> _deployMap() async {
    setState(() {
      _isProcessing = true;
    });

    try {
      final result = await _apiService.completeMapDeploymentWorkflow(
        deviceId: widget.deviceId,
        finalMapName: _finalMapNameController.text,
        locations: _locationPoints.map((point) => point.toJson()).toList(),
        piConfig: _piConfig,
        deploymentNotes: _deploymentNotesController.text,
        autoLoad: _autoLoadOnPi,
      );

      if (result['success'] == true) {
        setState(() {
          _stepCompleted[WorkflowStep.deployment] = true;
          _stepResults[WorkflowStep.deployment] = result;
        });
        _updateProgress();
        
        // Show success dialog
        _showCompletionDialog();
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

  void _addLocationPoint(LocationPointType type) {
    final point = LocationPointData(
      id: DateTime.now().millisecondsSinceEpoch.toString(),
      name: '${type.displayName} ${_locationPoints.length + 1}',
      type: type.name,
      position: {'x': 0.0, 'y': 0.0, 'z': 0.0},
      orientation: {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
      properties: {},
      createdAt: DateTime.now(),
    );

    setState(() {
      _locationPoints.add(point);
      _stepCompleted[WorkflowStep.annotation] = _locationPoints.isNotEmpty;
    });
    _updateProgress();
  }

  void _editLocationPoint(LocationPointData point) {
    // Show edit dialog
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Edit Location Point'),
        content: Text('Location point editing dialog would go here'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Save'),
          ),
        ],
      ),
    );
  }

  void _deleteLocationPoint(int index) {
    setState(() {
      _locationPoints.removeAt(index);
      _stepCompleted[WorkflowStep.annotation] = _locationPoints.isNotEmpty;
    });
    _updateProgress();
  }

  void _clearAllLocationPoints() {
    setState(() {
      _locationPoints.clear();
      _stepCompleted[WorkflowStep.annotation] = false;
    });
    _updateProgress();
  }

  void _importLocationPoints() {
    // Implementation for importing location points
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text('Import functionality coming soon')),
    );
  }

  void _showCompletionDialog() {
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (context) => AlertDialog(
        title: Row(
          children: [
            Icon(Icons.celebration, color: Colors.green, size: 32),
            SizedBox(width: 12),
            Text('Workflow Complete!'),
          ],
        ),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Your enhanced map has been successfully deployed!'),
            SizedBox(height: 16),
            Text('Summary:', style: TextStyle(fontWeight: FontWeight.bold)),
            Text('• Map: ${_finalMapNameController.text}'),
            Text('• Location Points: ${_locationPoints.length}'),
            Text('• Format: ${_selectedExportFormat.toUpperCase()}'),
            Text('• Auto-loaded: ${_autoLoadOnPi ? "Yes" : "No"}'),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
              Navigator.of(context).pop();
              widget.onWorkflowComplete?.call();
            },
            child: Text('Close'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              // Could navigate to monitoring or testing
            },
            child: Text('Monitor Deployment'),
          ),
        ],
      ),
    );
  }

  void _showHelp() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text('Workflow Help'),
        content: Text('Help documentation would go here'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Close'),
          ),
        ],
      ),
    );
  }

  void _resetWorkflow() {
    setState(() {
      for (final step in WorkflowStep.values) {
        _stepCompleted[step] = false;
        _stepResults[step] = {};
      }
      _currentStep = WorkflowStep.preparation;
      _overallProgress = 0.0;
      _workflowError = null;
      _locationPoints.clear();
      _editingMapData = null;
      _convertedMapName = null;
    });
    _progressAnimationController.reset();
  }

  void _completeWorkflow() {
    widget.onWorkflowComplete?.call();
    Navigator.of(context).pop();
  }
}

// Enhanced workflow steps
enum WorkflowStep {
  preparation,
  conversion,
  editing,
  annotation,
  validation,
  deployment,
}

// Utility function to show the enhanced workflow
Future<void> showEnhancedMapEditingWorkflow({
  required BuildContext context,
  required String deviceId,
  MapData? currentMapData,
  required Function(MapData) onMapUpdated,
  VoidCallback? onWorkflowComplete,
}) async {
  return showDialog(
    context: context,
    barrierDismissible: false,
    builder: (context) => EnhancedMapEditingWorkflow(
      deviceId: deviceId,
      currentMapData: currentMapData,
      onMapUpdated: onMapUpdated,
      onWorkflowComplete: onWorkflowComplete,
    ),
  );
}