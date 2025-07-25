// widgets/ros2_saved_maps_screen.dart - Screen for browsing ROS2 saved maps
import 'package:flutter/material.dart';
import '../services/api_service.dart';

class ROS2SavedMapsScreen extends StatefulWidget {
  final String deviceId;
  final Function(String mapName) onMapSelected;

  const ROS2SavedMapsScreen({
    Key? key,
    required this.deviceId,
    required this.onMapSelected,
  }) : super(key: key);

  @override
  _ROS2SavedMapsScreenState createState() => _ROS2SavedMapsScreenState();
}

class _ROS2SavedMapsScreenState extends State<ROS2SavedMapsScreen> {
  final ApiService _apiService = ApiService();

  List<Map<String, dynamic>> _ros2Maps = [];
  bool _isLoading = true;
  String? _error;
  String _selectedDirectory = '/tmp/saved_maps';

  @override
  void initState() {
    super.initState();
    _loadROS2SavedMaps();
  }

  Future<void> _loadROS2SavedMaps() async {
    setState(() {
      _isLoading = true;
      _error = null;
    });

    try {
      final maps = await _apiService.getROS2SavedMaps(
        widget.deviceId,
        directory: _selectedDirectory,
      );
      
      setState(() {
        _ros2Maps = maps;
        _isLoading = false;
      });

      print('✅ Loaded ${maps.length} ROS2 saved maps');
    } catch (e) {
      setState(() {
        _error = 'Failed to load ROS2 saved maps: $e';
        _isLoading = false;
      });
      print('❌ Error loading ROS2 saved maps: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('ROS2 Saved Maps (${_ros2Maps.length})'),
        actions: [
          IconButton(
            onPressed: _loadROS2SavedMaps,
            icon: const Icon(Icons.refresh),
          ),
        ],
      ),
      body: Column(
        children: [
          _buildDirectorySelector(),
          Expanded(child: _buildBody()),
        ],
      ),
    );
  }

  Widget _buildDirectorySelector() {
    return Container(
      padding: const EdgeInsets.all(16),
      color: Colors.grey.shade100,
      child: Row(
        children: [
          const Icon(Icons.folder, color: Colors.blue),
          const SizedBox(width: 8),
          const Text('Directory: '),
          Expanded(
            child: DropdownButton<String>(
              value: _selectedDirectory,
              isExpanded: true,
              items: const [
                DropdownMenuItem(value: '/tmp/saved_maps', child: Text('/tmp/saved_maps')),
                DropdownMenuItem(value: '/home/piros/maps', child: Text('/home/piros/maps')),
                DropdownMenuItem(value: '/opt/ros/maps', child: Text('/opt/ros/maps')),
              ],
              onChanged: (value) {
                if (value != null) {
                  setState(() {
                    _selectedDirectory = value;
                  });
                  _loadROS2SavedMaps();
                }
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildBody() {
    if (_isLoading) {
      return const Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            CircularProgressIndicator(),
            SizedBox(height: 16),
            Text('Loading ROS2 saved maps...'),
          ],
        ),
      );
    }

    if (_error != null) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Icon(Icons.error, size: 64, color: Colors.red),
            const SizedBox(height: 16),
            Text(_error!),
            const SizedBox(height: 16),
            ElevatedButton(
              onPressed: _loadROS2SavedMaps,
              child: const Text('Retry'),
            ),
          ],
        ),
      );
    }

    if (_ros2Maps.isEmpty) {
      return const Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.rocket_launch, size: 64, color: Colors.grey),
            SizedBox(height: 16),
            Text('No ROS2 saved maps found'),
            SizedBox(height: 8),
            Text(
              'Use the Control page to save maps via ROS2 command',
              style: TextStyle(color: Colors.grey),
              textAlign: TextAlign.center,
            ),
          ],
        ),
      );
    }

    return ListView.builder(
      padding: const EdgeInsets.all(16),
      itemCount: _ros2Maps.length,
      itemBuilder: (context, index) {
        final map = _ros2Maps[index];
        return _buildMapCard(map);
      },
    );
  }

  Widget _buildMapCard(Map<String, dynamic> map) {
    final mapName = map['name'] ?? 'Unknown Map';
    final modified = map['modified'] ?? 'Unknown';
    final size = map['size'] ?? 'Unknown';

    return Card(
      margin: const EdgeInsets.only(bottom: 12),
      child: ListTile(
        contentPadding: const EdgeInsets.all(16),
        leading: Container(
          width: 48,
          height: 48,
          decoration: BoxDecoration(
            gradient: LinearGradient(
              colors: [Colors.green.shade400, Colors.green.shade600],
            ),
            borderRadius: BorderRadius.circular(8),
          ),
          child: const Icon(Icons.rocket_launch, color: Colors.white),
        ),
        title: Text(
          mapName,
          style: const TextStyle(fontWeight: FontWeight.bold),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const SizedBox(height: 4),
            Text('Size: $size'),
            Text('Modified: $modified'),
            Text('Directory: ${map['directory']}'),
          ],
        ),
        trailing: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            ElevatedButton.icon(
              onPressed: () => _loadMap(mapName),
              icon: const Icon(Icons.open_in_new, size: 16),
              label: const Text('Load'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.green,
                foregroundColor: Colors.white,
                padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Future<void> _loadMap(String mapName) async {
    try {
      // Show loading dialog
      showDialog(
        context: context,
        barrierDismissible: false,
        builder: (context) => const AlertDialog(
          content: Row(
            children: [
              CircularProgressIndicator(),
              SizedBox(width: 16),
              Text('Loading ROS2 map...'),
            ],
          ),
        ),
      );

      // Close loading dialog
      Navigator.of(context).pop();

      // Close maps screen and notify parent
      Navigator.of(context).pop();
      widget.onMapSelected(mapName);

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Loading ROS2 map: $mapName'),
          backgroundColor: Colors.green,
        ),
      );
    } catch (e) {
      Navigator.of(context).pop(); // Close loading dialog

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Load failed: $e'),
          backgroundColor: Colors.red,
        ),
      );
    }
  }
}