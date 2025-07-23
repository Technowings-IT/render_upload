// widgets/map_selection_dialog.dart - Map Selection Dialog for Navigation
import 'package:flutter/material.dart';
import '../services/api_service.dart';

class MapSelectionDialog extends StatefulWidget {
  final String deviceId;
  final Function(String mapName, String mapSource) onMapSelected;
  final bool showDeploymentOptions;

  const MapSelectionDialog({
    Key? key,
    required this.deviceId,
    required this.onMapSelected,
    this.showDeploymentOptions = true,
  }) : super(key: key);

  @override
  _MapSelectionDialogState createState() => _MapSelectionDialogState();
}

class _MapSelectionDialogState extends State<MapSelectionDialog>
    with TickerProviderStateMixin {
  final ApiService _apiService = ApiService();
  
  late TabController _tabController;
  
  List<Map<String, dynamic>> _localMaps = [];
  List<Map<String, dynamic>> _piMaps = [];
  List<Map<String, dynamic>> _savedMaps = [];
  
  bool _isLoading = true;
  String? _error;
  String? _selectedMapName;
  String? _selectedMapSource;
  
  // Deployment options
  bool _deployToPi = false;
  bool _autoLoad = true;
  final TextEditingController _deploymentNotesController = TextEditingController();
  
  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 3, vsync: this);
    _loadAvailableMaps();
  }

  @override
  void dispose() {
    _tabController.dispose();
    _deploymentNotesController.dispose();
    super.dispose();
  }

  Future<void> _loadAvailableMaps() async {
    setState(() {
      _isLoading = true;
      _error = null;
    });

    try {
      final result = await _apiService.getNavigationMapOptions(widget.deviceId);
      
      if (result['success'] == true) {
        setState(() {
          _localMaps = List<Map<String, dynamic>>.from(result['localMaps'] ?? []);
          _piMaps = List<Map<String, dynamic>>.from(result['piMaps'] ?? []);
          _savedMaps = List<Map<String, dynamic>>.from(result['savedMaps'] ?? []);
          _isLoading = false;
        });
      } else {
        setState(() {
          _error = result['error'] ?? 'Failed to load maps';
          _isLoading = false;
        });
      }
    } catch (e) {
      setState(() {
        _error = 'Error loading maps: $e';
        _isLoading = false;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Dialog(
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Container(
        width: MediaQuery.of(context).size.width * 0.8,
        height: MediaQuery.of(context).size.height * 0.7,
        child: Column(
          children: [
            _buildHeader(),
            Expanded(child: _buildContent()),
            _buildActions(),
          ],
        ),
      ),
    );
  }

  Widget _buildHeader() {
    return Container(
      padding: EdgeInsets.all(20),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [Colors.blue.shade700, Colors.blue.shade500],
        ),
        borderRadius: BorderRadius.vertical(top: Radius.circular(16)),
      ),
      child: Row(
        children: [
          Icon(Icons.map, color: Colors.white, size: 28),
          SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Select Map for Navigation',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 20,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                Text(
                  'Device: ${widget.deviceId}',
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
            icon: Icon(Icons.close, color: Colors.white),
          ),
        ],
      ),
    );
  }

  Widget _buildContent() {
    if (_isLoading) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            CircularProgressIndicator(),
            SizedBox(height: 16),
            Text('Loading available maps...'),
          ],
        ),
      );
    }

    if (_error != null) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.error, size: 64, color: Colors.red),
            SizedBox(height: 16),
            Text(
              'Error Loading Maps',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 8),
            Text(_error!, textAlign: TextAlign.center),
            SizedBox(height: 16),
            ElevatedButton(
              onPressed: _loadAvailableMaps,
              child: Text('Retry'),
            ),
          ],
        ),
      );
    }

    return Column(
      children: [
        _buildTabBar(),
        Expanded(child: _buildTabContent()),
        if (widget.showDeploymentOptions) _buildDeploymentOptions(),
      ],
    );
  }

  Widget _buildTabBar() {
    return Container(
      child: TabBar(
        controller: _tabController,
        labelColor: Colors.blue.shade700,
        unselectedLabelColor: Colors.grey,
        indicatorColor: Colors.blue.shade700,
        tabs: [
          Tab(
            icon: Icon(Icons.storage),
            text: 'Local Maps (${_localMaps.length})',
          ),
          Tab(
            icon: Icon(Icons.cloud),
            text: 'Pi Maps (${_piMaps.length})',
          ),
          Tab(
            icon: Icon(Icons.save),
            text: 'Saved Maps (${_savedMaps.length})',
          ),
        ],
      ),
    );
  }

  Widget _buildTabContent() {
    return TabBarView(
      controller: _tabController,
      children: [
        _buildMapList(_localMaps, 'local'),
        _buildMapList(_piMaps, 'pi'),
        _buildMapList(_savedMaps, 'saved'),
      ],
    );
  }

  Widget _buildMapList(List<Map<String, dynamic>> maps, String source) {
    if (maps.isEmpty) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(
              source == 'local' ? Icons.storage :
              source == 'pi' ? Icons.cloud :
              Icons.save,
              size: 64,
              color: Colors.grey,
            ),
            SizedBox(height: 16),
            Text(
              'No ${source} maps available',
              style: TextStyle(fontSize: 16, color: Colors.grey),
            ),
            if (source == 'pi')
              Padding(
                padding: EdgeInsets.only(top: 8),
                child: Text(
                  'Deploy maps to Raspberry Pi first',
                  style: TextStyle(fontSize: 12, color: Colors.grey),
                ),
              ),
          ],
        ),
      );
    }

    return ListView.builder(
      padding: EdgeInsets.all(16),
      itemCount: maps.length,
      itemBuilder: (context, index) {
        final map = maps[index];
        return _buildMapTile(map, source);
      },
    );
  }

  Widget _buildMapTile(Map<String, dynamic> map, String source) {
    final mapName = map['name'] ?? 'Unknown Map';
    final isSelected = _selectedMapName == mapName && _selectedMapSource == source;
    
    return Card(
      elevation: isSelected ? 8 : 2,
      margin: EdgeInsets.only(bottom: 8),
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(12),
        side: BorderSide(
          color: isSelected ? Colors.blue : Colors.transparent,
          width: 2,
        ),
      ),
      child: ListTile(
        contentPadding: EdgeInsets.all(16),
        leading: Container(
          width: 48,
          height: 48,
          decoration: BoxDecoration(
            color: _getSourceColor(source).withOpacity(0.1),
            borderRadius: BorderRadius.circular(8),
          ),
          child: Icon(
            _getSourceIcon(source),
            color: _getSourceColor(source),
            size: 24,
          ),
        ),
        title: Text(
          mapName,
          style: TextStyle(
            fontWeight: FontWeight.bold,
            color: isSelected ? Colors.blue.shade700 : Colors.black87,
          ),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            SizedBox(height: 4),
            _buildMapDetails(map, source),
            if (isSelected)
              Container(
                margin: EdgeInsets.only(top: 8),
                padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                decoration: BoxDecoration(
                  color: Colors.blue.shade100,
                  borderRadius: BorderRadius.circular(4),
                ),
                child: Text(
                  'Selected for navigation',
                  style: TextStyle(
                    color: Colors.blue.shade700,
                    fontSize: 12,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ),
          ],
        ),
        trailing: isSelected
            ? Icon(Icons.check_circle, color: Colors.blue.shade700)
            : Icon(Icons.radio_button_unchecked, color: Colors.grey),
        onTap: () {
          setState(() {
            _selectedMapName = mapName;
            _selectedMapSource = source;
          });
        },
      ),
    );
  }

  Widget _buildMapDetails(Map<String, dynamic> map, String source) {
    final details = <Widget>[];

    // Common details
    if (map['size'] != null) {
      details.add(_buildDetailChip('Size', map['size'].toString(), Icons.storage));
    }

    if (map['modified'] != null) {
      details.add(_buildDetailChip('Modified', map['modified'].toString(), Icons.schedule));
    }

    // Source-specific details
    switch (source) {
      case 'local':
        if (map['metadata'] != null) {
          final metadata = map['metadata'];
          if (metadata['resolution'] != null) {
            details.add(_buildDetailChip(
              'Resolution', 
              '${metadata['resolution']}m/px', 
              Icons.grid_on
            ));
          }
        }
        break;
        
      case 'pi':
        details.add(_buildDetailChip('Source', 'Raspberry Pi', Icons.cloud));
        break;
        
      case 'saved':
        if (map['shapes'] != null) {
          details.add(_buildDetailChip(
            'Locations', 
            '${map['shapes']} markers', 
            Icons.location_on
          ));
        }
        if (map['savedAt'] != null) {
          final savedAt = DateTime.tryParse(map['savedAt'].toString());
          if (savedAt != null) {
            details.add(_buildDetailChip(
              'Saved', 
              _formatDate(savedAt), 
              Icons.save
            ));
          }
        }
        break;
    }

    return Wrap(
      spacing: 4,
      runSpacing: 4,
      children: details,
    );
  }

  Widget _buildDetailChip(String label, String value, IconData icon) {
    return Container(
      padding: EdgeInsets.symmetric(horizontal: 6, vertical: 2),
      decoration: BoxDecoration(
        color: Colors.grey.shade100,
        borderRadius: BorderRadius.circular(4),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 12, color: Colors.grey.shade600),
          SizedBox(width: 2),
          Text(
            '$label: $value',
            style: TextStyle(
              fontSize: 10,
              color: Colors.grey.shade700,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDeploymentOptions() {
    if (_selectedMapSource == 'pi') {
      return Container(); // No deployment needed for Pi maps
    }

    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.grey.shade50,
        border: Border(top: BorderSide(color: Colors.grey.shade300)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(Icons.cloud_upload, color: Colors.blue.shade700),
              SizedBox(width: 8),
              Text(
                'Deployment Options',
                style: TextStyle(
                  fontWeight: FontWeight.bold,
                  color: Colors.blue.shade700,
                ),
              ),
            ],
          ),
          SizedBox(height: 12),
          
          CheckboxListTile(
            title: Text('Deploy to Raspberry Pi'),
            subtitle: Text('Send this map to the Pi before starting navigation'),
            value: _deployToPi,
            onChanged: (value) {
              setState(() {
                _deployToPi = value ?? false;
              });
            },
            contentPadding: EdgeInsets.zero,
          ),
          
          if (_deployToPi) ...[
            CheckboxListTile(
              title: Text('Auto-load on Pi'),
              subtitle: Text('Automatically load this map in ROS after deployment'),
              value: _autoLoad,
              onChanged: (value) {
                setState(() {
                  _autoLoad = value ?? true;
                });
              },
              contentPadding: EdgeInsets.zero,
            ),
            
            TextField(
              controller: _deploymentNotesController,
              decoration: InputDecoration(
                labelText: 'Deployment Notes (Optional)',
                hintText: 'Add any notes about this deployment...',
                border: OutlineInputBorder(),
                contentPadding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
              ),
              maxLines: 2,
            ),
          ],
        ],
      ),
    );
  }

  Widget _buildActions() {
    return Container(
      padding: EdgeInsets.all(20),
      decoration: BoxDecoration(
        border: Border(top: BorderSide(color: Colors.grey.shade300)),
      ),
      child: Row(
        children: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          SizedBox(width: 12),
          
          ElevatedButton(
            onPressed: _selectedMapName != null ? _refreshMaps : null,
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                Icon(Icons.refresh, size: 16),
                SizedBox(width: 4),
                Text('Refresh'),
              ],
            ),
          ),
          
          Spacer(),
          
          ElevatedButton(
            onPressed: _selectedMapName != null ? _startNavigation : null,
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.blue.shade700,
              foregroundColor: Colors.white,
              padding: EdgeInsets.symmetric(horizontal: 24, vertical: 12),
            ),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                Icon(Icons.navigation, size: 16),
                SizedBox(width: 8),
                Text(_deployToPi ? 'Deploy & Start' : 'Start Navigation'),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Color _getSourceColor(String source) {
    switch (source) {
      case 'local':
        return Colors.orange;
      case 'pi':
        return Colors.green;
      case 'saved':
        return Colors.purple;
      default:
        return Colors.grey;
    }
  }

  IconData _getSourceIcon(String source) {
    switch (source) {
      case 'local':
        return Icons.storage;
      case 'pi':
        return Icons.cloud;
      case 'saved':
        return Icons.save;
      default:
        return Icons.map;
    }
  }

  String _formatDate(DateTime date) {
    final now = DateTime.now();
    final difference = now.difference(date);

    if (difference.inDays > 0) {
      return '${difference.inDays}d ago';
    } else if (difference.inHours > 0) {
      return '${difference.inHours}h ago';
    } else if (difference.inMinutes > 0) {
      return '${difference.inMinutes}m ago';
    } else {
      return 'Just now';
    }
  }

  Future<void> _refreshMaps() async {
    await _loadAvailableMaps();
  }

  Future<void> _startNavigation() async {
    if (_selectedMapName == null || _selectedMapSource == null) return;

    try {
      // Show loading
      showDialog(
        context: context,
        barrierDismissible: false,
        builder: (context) => AlertDialog(
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              CircularProgressIndicator(),
              SizedBox(height: 16),
              Text(_deployToPi ? 'Deploying and starting navigation...' : 'Starting navigation...'),
            ],
          ),
        ),
      );

      // Deploy to Pi if requested
      if (_deployToPi && _selectedMapSource != 'pi') {
        await _apiService.deployMapToRaspberryPi(
          deviceId: widget.deviceId,
          mapName: _selectedMapName!,
          autoLoad: _autoLoad,
        );
      }

      // Start navigation
      final result = await _apiService.startNavigationWithMap(
        deviceId: widget.deviceId,
        mapName: _selectedMapName!,
        mapSource: _selectedMapSource!,
      );

      // Close loading dialog
      Navigator.of(context).pop();

      if (result['success'] == true) {
        // Close selection dialog and notify parent
        Navigator.of(context).pop();
        widget.onMapSelected(_selectedMapName!, _selectedMapSource!);
        
        // Show success message
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('Navigation started with map: $_selectedMapName'),
            backgroundColor: Colors.green,
          ),
        );
      } else {
        // Show error
        showDialog(
          context: context,
          builder: (context) => AlertDialog(
            title: Text('Navigation Failed'),
            content: Text(result['error'] ?? 'Unknown error occurred'),
            actions: [
              TextButton(
                onPressed: () => Navigator.of(context).pop(),
                child: Text('OK'),
              ),
            ],
          ),
        );
      }
    } catch (e) {
      // Close loading dialog if open
      Navigator.of(context).pop();
      
      // Show error
      showDialog(
        context: context,
        builder: (context) => AlertDialog(
          title: Text('Navigation Failed'),
          content: Text('Error: $e'),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: Text('OK'),
            ),
          ],
        ),
      );
    }
  }
}

// Utility function to show the map selection dialog
Future<void> showMapSelectionDialog({
  required BuildContext context,
  required String deviceId,
  required Function(String mapName, String mapSource) onMapSelected,
  bool showDeploymentOptions = true,
}) async {
  return showDialog(
    context: context,
    barrierDismissible: false,
    builder: (context) => MapSelectionDialog(
      deviceId: deviceId,
      onMapSelected: onMapSelected,
      showDeploymentOptions: showDeploymentOptions,
    ),
  );
}