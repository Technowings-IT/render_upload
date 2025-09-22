// widgets/advanced_map_controls.dart - Complete UI for map and costmap management
import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../services/web_socket_service.dart';
import '../services/api_service.dart';

class AdvancedMapControls extends StatefulWidget {
  final String deviceId;
  final Function(Map<String, dynamic>) onSettingsChanged;
  final Map<String, dynamic>? currentSettings;

  const AdvancedMapControls({
    Key? key,
    required this.deviceId,
    required this.onSettingsChanged,
    this.currentSettings,
  }) : super(key: key);

  @override
  _AdvancedMapControlsState createState() => _AdvancedMapControlsState();
}

class _AdvancedMapControlsState extends State<AdvancedMapControls>
    with TickerProviderStateMixin {
  final WebSocketService _wsService = WebSocketService();
  final ApiService _apiService = ApiService();

  // Settings
  Map<String, dynamic> _settings = {
    'showGlobalCostmap': true,
    'showLocalCostmap': true,
    'showStaticMap': true,
    'showRobotTrail': true,
    'showObstacles': true,
    'globalCostmapOpacity': 0.6,
    'localCostmapOpacity': 0.8,
    'staticMapOpacity': 0.9,
    'costmapColorScheme': 'default',
    'trailLength': 200,
    'autoCenter': true,
    'gridVisible': true,
    'coordinatesVisible': true,
  };

  // Animation controllers
  late AnimationController _panelController;
  late AnimationController _exportController;
  bool _panelExpanded = false;
  bool _isExporting = false;

  // Available maps
  List<Map<String, dynamic>> _availableMaps = [];
  String? _selectedMapName;

  // Real-time status
  Map<String, bool> _layerStatus = {
    'staticMap': false,
    'globalCostmap': false,
    'localCostmap': false,
    'robotPosition': false,
  };

  @override
  void initState() {
    super.initState();

    _panelController = AnimationController(
      duration: const Duration(milliseconds: 300),
      vsync: this,
    );

    _exportController = AnimationController(
      duration: const Duration(milliseconds: 2000),
      vsync: this,
    );

    // Initialize settings
    if (widget.currentSettings != null) {
      _settings.addAll(widget.currentSettings!);
    }

    _loadAvailableMaps();
    _subscribeToStatusUpdates();
  }

  @override
  void dispose() {
    _panelController.dispose();
    _exportController.dispose();
    super.dispose();
  }

  void _subscribeToStatusUpdates() {
    _wsService.realTimeData.listen((data) {
      if (data['deviceId'] == widget.deviceId) {
        setState(() {
          switch (data['type']) {
            case 'map_update':
              _layerStatus['staticMap'] = true;
              break;
            case 'global_costmap_update':
              _layerStatus['globalCostmap'] = true;
              break;
            case 'local_costmap_update':
              _layerStatus['localCostmap'] = true;
              break;
            case 'position_update':
            case 'odometry_update':
              _layerStatus['robotPosition'] = true;
              break;
          }
        });
      }
    });
  }

  Future<void> _loadAvailableMaps() async {
    try {
      final maps = await _apiService.getAvailableMaps(widget.deviceId);
      setState(() {
        _availableMaps = maps;
        if (maps.isNotEmpty && _selectedMapName == null) {
          _selectedMapName = maps.first['name'];
        }
      });
    } catch (e) {
      print(' Failed to load available maps: $e');
    }
  }

  void _updateSetting(String key, dynamic value) {
    setState(() {
      _settings[key] = value;
    });
    widget.onSettingsChanged(_settings);
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.white,
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, -2),
          ),
        ],
      ),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          _buildMainControls(),
          AnimatedSize(
            duration: const Duration(milliseconds: 300),
            child: _panelExpanded
                ? _buildAdvancedPanel()
                : const SizedBox.shrink(),
          ),
        ],
      ),
    );
  }

  Widget _buildMainControls() {
    return Container(
      padding: const EdgeInsets.all(16),
      child: Column(
        children: [
          // Header with expand/collapse
          Row(
            children: [
              Icon(Icons.layers, color: Colors.blue.shade700),
              const SizedBox(width: 8),
              Text(
                'Map Controls',
                style: TextStyle(
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                  color: Colors.blue.shade700,
                ),
              ),
              const Spacer(),
              _buildStatusIndicators(),
              const SizedBox(width: 16),
              IconButton(
                onPressed: () {
                  setState(() {
                    _panelExpanded = !_panelExpanded;
                  });
                  if (_panelExpanded) {
                    _panelController.forward();
                  } else {
                    _panelController.reverse();
                  }
                },
                icon: AnimatedRotation(
                  turns: _panelExpanded ? 0.5 : 0,
                  duration: const Duration(milliseconds: 300),
                  child: const Icon(Icons.expand_more),
                ),
              ),
            ],
          ),

          const SizedBox(height: 16),

          // Quick layer toggles
          _buildQuickToggles(),

          const SizedBox(height: 16),

          // Action buttons
          _buildActionButtons(),
        ],
      ),
    );
  }

  Widget _buildStatusIndicators() {
    return Row(
      children: [
        _buildStatusDot('Map', _layerStatus['staticMap']!, Colors.grey),
        const SizedBox(width: 8),
        _buildStatusDot('Global', _layerStatus['globalCostmap']!, Colors.blue),
        const SizedBox(width: 8),
        _buildStatusDot('Local', _layerStatus['localCostmap']!, Colors.red),
        const SizedBox(width: 8),
        _buildStatusDot('Robot', _layerStatus['robotPosition']!, Colors.green),
      ],
    );
  }

  Widget _buildStatusDot(String label, bool active, Color color) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          width: 8,
          height: 8,
          decoration: BoxDecoration(
            color: active ? color : Colors.grey.shade300,
            shape: BoxShape.circle,
            boxShadow: active
                ? [
                    BoxShadow(
                      color: color.withOpacity(0.5),
                      blurRadius: 4,
                      spreadRadius: 1,
                    ),
                  ]
                : null,
          ),
        ),
        const SizedBox(width: 4),
        Text(
          label,
          style: TextStyle(
            fontSize: 10,
            color: active ? color : Colors.grey.shade600,
            fontWeight: active ? FontWeight.bold : FontWeight.normal,
          ),
        ),
      ],
    );
  }

  Widget _buildQuickToggles() {
    return Row(
      children: [
        _buildLayerToggle(
          'Static Map',
          _settings['showStaticMap'],
          Colors.grey.shade600,
          Icons.map,
          (value) => _updateSetting('showStaticMap', value),
        ),
        const SizedBox(width: 12),
        _buildLayerToggle(
          'Global',
          _settings['showGlobalCostmap'],
          Colors.blue,
          Icons.public,
          (value) => _updateSetting('showGlobalCostmap', value),
        ),
        const SizedBox(width: 12),
        _buildLayerToggle(
          'Local',
          _settings['showLocalCostmap'],
          Colors.red,
          Icons.near_me,
          (value) => _updateSetting('showLocalCostmap', value),
        ),
        const SizedBox(width: 12),
        _buildLayerToggle(
          'Trail',
          _settings['showRobotTrail'],
          Colors.orange,
          Icons.timeline,
          (value) => _updateSetting('showRobotTrail', value),
        ),
      ],
    );
  }

  Widget _buildLayerToggle(
    String label,
    bool value,
    Color color,
    IconData icon,
    Function(bool) onChanged,
  ) {
    return Expanded(
      child: GestureDetector(
        onTap: () => onChanged(!value),
        child: Container(
          padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 12),
          decoration: BoxDecoration(
            color: value ? color.withOpacity(0.1) : Colors.grey.shade100,
            border: Border.all(
              color: value ? color : Colors.grey.shade300,
              width: 2,
            ),
            borderRadius: BorderRadius.circular(8),
          ),
          child: Column(
            children: [
              Icon(
                icon,
                color: value ? color : Colors.grey.shade400,
                size: 20,
              ),
              const SizedBox(height: 4),
              Text(
                label,
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: value ? FontWeight.bold : FontWeight.normal,
                  color: value ? color : Colors.grey.shade600,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildActionButtons() {
    return Row(
      children: [
        // Export button with animation
        Expanded(
          child: AnimatedBuilder(
            animation: _exportController,
            builder: (context, child) {
              return ElevatedButton.icon(
                onPressed: _isExporting ? null : _exportMap,
                icon: _isExporting
                    ? SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          valueColor:
                              AlwaysStoppedAnimation<Color>(Colors.white),
                          value: _exportController.value,
                        ),
                      )
                    : const Icon(Icons.download),
                label: Text(_isExporting ? 'Exporting...' : 'Export PGM'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.green,
                  foregroundColor: Colors.white,
                  padding: const EdgeInsets.symmetric(vertical: 12),
                ),
              );
            },
          ),
        ),

        const SizedBox(width: 12),

        // Upload button
        Expanded(
          child: ElevatedButton.icon(
            onPressed: _selectedMapName != null ? _uploadMap : null,
            icon: const Icon(Icons.upload),
            label: const Text('Upload to AMR'),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.purple,
              foregroundColor: Colors.white,
              padding: const EdgeInsets.symmetric(vertical: 12),
            ),
          ),
        ),

        const SizedBox(width: 12),

        // Clear costmap button
        ElevatedButton(
          onPressed: _clearCostmaps,
          style: ElevatedButton.styleFrom(
            backgroundColor: Colors.orange,
            foregroundColor: Colors.white,
            padding: const EdgeInsets.all(12),
          ),
          child: const Icon(Icons.clear_all),
        ),
      ],
    );
  }

  Widget _buildAdvancedPanel() {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.grey.shade50,
        border: Border(
          top: BorderSide(color: Colors.grey.shade300),
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Opacity controls
          _buildOpacityControls(),

          const SizedBox(height: 16),

          // Color scheme and appearance
          _buildAppearanceControls(),

          const SizedBox(height: 16),

          // Map management
          _buildMapManagement(),

          const SizedBox(height: 16),

          // Advanced options
          _buildAdvancedOptions(),
        ],
      ),
    );
  }

  Widget _buildOpacityControls() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          'Layer Opacity',
          style: TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
            color: Colors.grey.shade700,
          ),
        ),
        const SizedBox(height: 8),
        if (_settings['showStaticMap'])
          _buildOpacitySlider(
            'Static Map',
            _settings['staticMapOpacity'],
            Colors.grey.shade600,
            (value) => _updateSetting('staticMapOpacity', value),
          ),
        if (_settings['showGlobalCostmap'])
          _buildOpacitySlider(
            'Global Costmap',
            _settings['globalCostmapOpacity'],
            Colors.blue,
            (value) => _updateSetting('globalCostmapOpacity', value),
          ),
        if (_settings['showLocalCostmap'])
          _buildOpacitySlider(
            'Local Costmap',
            _settings['localCostmapOpacity'],
            Colors.red,
            (value) => _updateSetting('localCostmapOpacity', value),
          ),
      ],
    );
  }

  Widget _buildOpacitySlider(
    String label,
    double value,
    Color color,
    Function(double) onChanged,
  ) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          SizedBox(
            width: 100,
            child: Text(
              label,
              style: TextStyle(fontSize: 12, color: color),
            ),
          ),
          Expanded(
            child: SliderTheme(
              data: SliderTheme.of(context).copyWith(
                activeTrackColor: color,
                thumbColor: color,
                inactiveTrackColor: color.withOpacity(0.3),
                trackHeight: 4,
                thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 8),
              ),
              child: Slider(
                value: value,
                onChanged: onChanged,
                min: 0.0,
                max: 1.0,
                divisions: 20,
              ),
            ),
          ),
          SizedBox(
            width: 40,
            child: Text(
              '${(value * 100).toInt()}%',
              style: TextStyle(fontSize: 12, color: color),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildAppearanceControls() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          'Appearance',
          style: TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
            color: Colors.grey.shade700,
          ),
        ),
        const SizedBox(height: 8),

        // Color scheme selector
        Row(
          children: [
            const Text('Color Scheme: '),
            const SizedBox(width: 8),
            DropdownButton<String>(
              value: _settings['costmapColorScheme'],
              items: const [
                DropdownMenuItem(value: 'default', child: Text('Default')),
                DropdownMenuItem(value: 'thermal', child: Text('Thermal')),
                DropdownMenuItem(value: 'grayscale', child: Text('Grayscale')),
                DropdownMenuItem(value: 'rainbow', child: Text('Rainbow')),
              ],
              onChanged: (value) => _updateSetting('costmapColorScheme', value),
            ),
          ],
        ),

        const SizedBox(height: 8),

        // Trail length
        Row(
          children: [
            const Text('Trail Length: '),
            Expanded(
              child: Slider(
                value: _settings['trailLength'].toDouble(),
                min: 50,
                max: 1000,
                divisions: 19,
                label: '${_settings['trailLength']} points',
                onChanged: (value) =>
                    _updateSetting('trailLength', value.toInt()),
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildMapManagement() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          'Map Management',
          style: TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
            color: Colors.grey.shade700,
          ),
        ),
        const SizedBox(height: 8),

        // Available maps dropdown
        Row(
          children: [
            const Text('Select Map: '),
            const SizedBox(width: 8),
            Expanded(
              child: DropdownButtonFormField<String>(
                value: _selectedMapName,
                decoration: const InputDecoration(
                  border: OutlineInputBorder(),
                  contentPadding:
                      EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                  isDense: true,
                ),
                items: _availableMaps.map((map) {
                  return DropdownMenuItem<String>(
                    value: map['name'],
                    child: Text(
                      map['name'],
                      style: const TextStyle(fontSize: 12),
                    ),
                  );
                }).toList(),
                onChanged: (value) {
                  setState(() {
                    _selectedMapName = value;
                  });
                },
              ),
            ),
            const SizedBox(width: 8),
            IconButton(
              onPressed: _loadAvailableMaps,
              icon: const Icon(Icons.refresh, size: 20),
              tooltip: 'Refresh Maps',
            ),
          ],
        ),

        const SizedBox(height: 8),

        // Map operations
        Row(
          children: [
            _buildMapActionButton(
              'Process',
              Icons.tune,
              Colors.blue,
              _processMap,
            ),
            const SizedBox(width: 8),
            _buildMapActionButton(
              'Download',
              Icons.file_download,
              Colors.green,
              _downloadMap,
            ),
            const SizedBox(width: 8),
            _buildMapActionButton(
              'Delete',
              Icons.delete,
              Colors.red,
              _deleteMap,
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildMapActionButton(
    String label,
    IconData icon,
    Color color,
    VoidCallback? onPressed,
  ) {
    return Expanded(
      child: ElevatedButton.icon(
        onPressed: onPressed,
        icon: Icon(icon, size: 16),
        label: Text(label, style: const TextStyle(fontSize: 12)),
        style: ElevatedButton.styleFrom(
          backgroundColor: color,
          foregroundColor: Colors.white,
          padding: const EdgeInsets.symmetric(vertical: 8),
        ),
      ),
    );
  }

  Widget _buildAdvancedOptions() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          'Advanced Options',
          style: TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
            color: Colors.grey.shade700,
          ),
        ),
        const SizedBox(height: 8),

        // Checkboxes for advanced features
        _buildCheckboxOption(
          'Auto Center on Robot',
          _settings['autoCenter'],
          (value) => _updateSetting('autoCenter', value),
        ),
        _buildCheckboxOption(
          'Show Grid',
          _settings['gridVisible'],
          (value) => _updateSetting('gridVisible', value),
        ),
        _buildCheckboxOption(
          'Show Coordinates',
          _settings['coordinatesVisible'],
          (value) => _updateSetting('coordinatesVisible', value),
        ),
        _buildCheckboxOption(
          'Show Obstacles',
          _settings['showObstacles'],
          (value) => _updateSetting('showObstacles', value),
        ),
      ],
    );
  }

  Widget _buildCheckboxOption(
    String label,
    bool value,
    Function(bool) onChanged,
  ) {
    return Row(
      children: [
        Checkbox(
          value: value,
          onChanged: (newValue) => onChanged(newValue ?? false),
        ),
        Text(label),
      ],
    );
  }

  // Action methods
  Future<void> _exportMap() async {
    setState(() => _isExporting = true);
    _exportController.reset();
    _exportController.forward();

    try {
      final result = await _apiService.exportMapToPGM(
        deviceId: widget.deviceId,
        includeGlobalCostmap: _settings['showGlobalCostmap'],
        includeLocalCostmap: _settings['showLocalCostmap'],
        includeStaticMap: _settings['showStaticMap'],
      );

      if (result['success']) {
        _showSuccessSnackBar('Map exported successfully!');
        await _loadAvailableMaps();
      } else {
        _showErrorSnackBar('Export failed: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Export failed: $e');
    } finally {
      setState(() => _isExporting = false);
    }
  }

  Future<void> _uploadMap() async {
    if (_selectedMapName == null) return;

    try {
      final result = await _apiService.uploadMapToAMR(
        deviceId: widget.deviceId,
        mapName: _selectedMapName!,
        setAsActiveMap: true,
      );

      if (result['success']) {
        _showSuccessSnackBar('Map uploaded to AMR successfully!');
      } else {
        _showErrorSnackBar('Upload failed: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Upload failed: $e');
    }
  }

  Future<void> _clearCostmaps() async {
    try {
      final result = await _apiService.clearCostmap(
        deviceId: widget.deviceId,
        costmapType: 'both',
      );

      if (result['success']) {
        _showSuccessSnackBar('Costmaps cleared successfully!');
      } else {
        _showErrorSnackBar('Clear failed: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Clear failed: $e');
    }
  }

  Future<void> _processMap() async {
    if (_selectedMapName == null) return;

    // Show processing options dialog
    final operation = await _showProcessingDialog();
    if (operation == null) return;

    try {
      final result = await _apiService.processMap(
        deviceId: widget.deviceId,
        mapName: _selectedMapName!,
        operation: operation,
        operationParams: {'kernelSize': 3},
      );

      if (result['success']) {
        _showSuccessSnackBar('Map processed successfully!');
        await _loadAvailableMaps();
      } else {
        _showErrorSnackBar('Processing failed: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Processing failed: $e');
    }
  }

  Future<String?> _showProcessingDialog() async {
    return showDialog<String>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('Process Map'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            const Text('Select processing operation:'),
            const SizedBox(height: 16),
            ...['erode', 'dilate', 'open', 'close'].map(
              (op) => ListTile(
                title: Text(op.toUpperCase()),
                onTap: () => Navigator.of(context).pop(op),
              ),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: const Text('Cancel'),
          ),
        ],
      ),
    );
  }

  Future<void> _downloadMap() async {
    if (_selectedMapName == null) return;

    try {
      final result = await _apiService.downloadPGMFile(
        deviceId: widget.deviceId,
        fileName: '$_selectedMapName.pgm',
      );

      if (result['success']) {
        _showSuccessSnackBar('Map download started!');
      } else {
        _showErrorSnackBar('Download failed: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Download failed: $e');
    }
  }

  Future<void> _deleteMap() async {
    if (_selectedMapName == null) return;

    final confirmed = await _showDeleteConfirmDialog();
    if (!confirmed) return;

    try {
      final result = await _apiService.deleteMap(
        deviceId: widget.deviceId,
        mapName: _selectedMapName!,
      );

      if (result['success']) {
        _showSuccessSnackBar('Map deleted successfully!');
        setState(() => _selectedMapName = null);
        await _loadAvailableMaps();
      } else {
        _showErrorSnackBar('Delete failed: ${result['error']}');
      }
    } catch (e) {
      _showErrorSnackBar('Delete failed: $e');
    }
  }

  Future<bool> _showDeleteConfirmDialog() async {
    return await showDialog<bool>(
          context: context,
          builder: (context) => AlertDialog(
            title: const Text('Delete Map'),
            content:
                Text('Are you sure you want to delete "$_selectedMapName"?'),
            actions: [
              TextButton(
                onPressed: () => Navigator.of(context).pop(false),
                child: const Text('Cancel'),
              ),
              TextButton(
                onPressed: () => Navigator.of(context).pop(true),
                style: TextButton.styleFrom(foregroundColor: Colors.red),
                child: const Text('Delete'),
              ),
            ],
          ),
        ) ??
        false;
  }

  void _showSuccessSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.green,
        behavior: SnackBarBehavior.floating,
      ),
    );
  }

  void _showErrorSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.red,
        behavior: SnackBarBehavior.floating,
        duration: const Duration(seconds: 4),
      ),
    );
  }
}
