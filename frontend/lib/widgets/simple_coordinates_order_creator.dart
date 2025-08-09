// widgets/simple_coordinate_order_creator.dart - Enhanced Responsive Version
import 'package:flutter/material.dart';
import '../models/map_data.dart';
import '../models/odom.dart' as odom;
import '../services/api_service.dart';

class SimpleCoordinateOrderCreator extends StatefulWidget {
  final String deviceId;
  final MapData mapData;
  final Function(Map<String, dynamic>) onOrderCreated;

  const SimpleCoordinateOrderCreator({
    Key? key,
    required this.deviceId,
    required this.mapData,
    required this.onOrderCreated,
  }) : super(key: key);

  @override
  State<SimpleCoordinateOrderCreator> createState() =>
      _SimpleCoordinateOrderCreatorState();
}

class _SimpleCoordinateOrderCreatorState
    extends State<SimpleCoordinateOrderCreator> {
  final ApiService _apiService = ApiService();

  // State variables
  List<Map<String, dynamic>> _coordinates = [];
  bool _isCreatingOrder = false;
  bool _isExecutingOrder = false;
  int _currentCoordinateIndex = 0;
  bool _showSavedOrderPreview = false;
  Map<String, dynamic>? _lastSavedOrder;

  // Map selection functionality
  List<Map<String, dynamic>> _availableMaps = [];
  String? _selectedMapName;
  MapData? _selectedMapData;
  bool _isLoadingMaps = false;
  bool _isLoadingMapData = false;

  // Map interaction
  double _mapScale = 1.0;
  Offset _mapOffset = Offset.zero;
  Size _canvasSize = Size(600, 500);

  // Pan/drag state management
  bool _isPanning = false;
  Offset? _lastPanPosition;

  // Form controllers
  final TextEditingController _orderNameController = TextEditingController();
  final TextEditingController _coordinateNameController =
      TextEditingController();

  @override
  void initState() {
    super.initState();
    _orderNameController.text =
        'Order ${DateTime.now().millisecondsSinceEpoch}';

    // Initialize with YAML origin values
    print('🚀 Initializing with YAML origin values: (-2.02, -5.21)');

    _selectedMapData = MapData(
      deviceId: widget.deviceId,
      timestamp: DateTime.now(),
      info: MapInfo(
        width: widget.mapData.info.width,
        height: widget.mapData.info.height,
        resolution: widget.mapData.info.resolution,
        origin: odom.Position(x: -2.02, y: -5.21, z: 0.0),
        originOrientation: widget.mapData.info.originOrientation,
      ),
      occupancyData: widget.mapData.occupancyData,
      shapes: widget.mapData.shapes,
      version: widget.mapData.version,
    );

    _loadAvailableMaps();
  }

  // Device type detection
  DeviceType _getDeviceType(BuildContext context) {
    final screenWidth = MediaQuery.of(context).size.width;
    final screenHeight = MediaQuery.of(context).size.height;
    final diagonal =
        (screenWidth * screenWidth + screenHeight * screenHeight) / 160;
    final diagonalInches = diagonal / 160;

    if (diagonalInches < 7.0) {
      return DeviceType.mobile;
    } else if (diagonalInches < 10.0) {
      return DeviceType.tablet;
    } else {
      return DeviceType.laptop;
    }
  }

  // Responsive values based on device type
  ResponsiveValues _getResponsiveValues(DeviceType deviceType) {
    switch (deviceType) {
      case DeviceType.mobile:
        return ResponsiveValues(
          mapFlexRatio: 1,
          controlFlexRatio: 1,
          isHorizontalLayout: false,
          cardPadding: 12.0,
          buttonHeight: 56.0,
          fontSize: 14.0,
          titleFontSize: 18.0,
          iconSize: 20.0,
          coordinateRadius: 18.0,
          bottomSheetHeight: 0.7,
        );
      case DeviceType.tablet:
        return ResponsiveValues(
          mapFlexRatio: 2,
          controlFlexRatio: 1,
          isHorizontalLayout: true,
          cardPadding: 16.0,
          buttonHeight: 52.0,
          fontSize: 15.0,
          titleFontSize: 20.0,
          iconSize: 22.0,
          coordinateRadius: 20.0,
          bottomSheetHeight: 0.6,
        );
      case DeviceType.laptop:
        return ResponsiveValues(
          mapFlexRatio: 3,
          controlFlexRatio: 2,
          isHorizontalLayout: true,
          cardPadding: 20.0,
          buttonHeight: 48.0,
          fontSize: 16.0,
          titleFontSize: 22.0,
          iconSize: 24.0,
          coordinateRadius: 22.0,
          bottomSheetHeight: 0.5,
        );
    }
  }

  @override
  Widget build(BuildContext context) {
    final deviceType = _getDeviceType(context);
    final responsive = _getResponsiveValues(deviceType);
    final screenSize = MediaQuery.of(context).size;
    final isPortrait = screenSize.height > screenSize.width;

    return Scaffold(
      appBar: AppBar(
        title: Text(
          'Create Order',
          style: TextStyle(
            fontSize: responsive.titleFontSize,
            fontWeight: FontWeight.w600,
          ),
        ),
        backgroundColor: Colors.blue.shade700,
        foregroundColor: Colors.white,
        elevation: 2,
        shadowColor: Colors.blue.shade200,
      ),
      body: responsive.isHorizontalLayout && !isPortrait
          ? _buildHorizontalLayout(responsive)
          : _buildVerticalLayout(responsive),
    );
  }

  Widget _buildHorizontalLayout(ResponsiveValues responsive) {
    return Row(
      children: [
        // LEFT: Map
        Expanded(
          flex: responsive.mapFlexRatio,
          child: _buildEnhancedMap(responsive),
        ),
        // RIGHT: Control Panel
        Expanded(
          flex: responsive.controlFlexRatio,
          child: _buildEnhancedControlPanel(responsive),
        ),
      ],
    );
  }

  Widget _buildVerticalLayout(ResponsiveValues responsive) {
    return Column(
      children: [
        // TOP: Map
        Expanded(
          flex: 2,
          child: _buildEnhancedMap(responsive),
        ),
        // BOTTOM: Control Panel
        Expanded(
          flex: 1,
          child: _buildEnhancedControlPanel(responsive),
        ),
      ],
    );
  }

  Widget _buildEnhancedMap(ResponsiveValues responsive) {
    return Container(
      margin: EdgeInsets.all(responsive.cardPadding * 0.5),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [Colors.blue.shade50, Colors.white],
        ),
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(
            color: Colors.blue.shade100.withOpacity(0.5),
            blurRadius: 10,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: ClipRRect(
        borderRadius: BorderRadius.circular(16),
        child: LayoutBuilder(
          builder: (context, constraints) {
            _canvasSize = Size(constraints.maxWidth, constraints.maxHeight);

            return Stack(
              children: [
                // Map Canvas
                GestureDetector(
                  onTapDown: _handleMapTap,
                  onScaleStart: _handleScaleStart,
                  onScaleUpdate: _handleScaleUpdate,
                  onScaleEnd: _handleScaleEnd,
                  child: CustomPaint(
                    size: Size(constraints.maxWidth, constraints.maxHeight),
                    painter: EnhancedMapPainter(
                      mapData: _selectedMapData ?? widget.mapData,
                      coordinates: _coordinates,
                      mapScale: _mapScale,
                      mapOffset: _mapOffset,
                      currentExecutingIndex:
                          _isExecutingOrder ? _currentCoordinateIndex : -1,
                      canvasSize:
                          Size(constraints.maxWidth, constraints.maxHeight),
                      savedOrder: _lastSavedOrder,
                      showSavedOrderPreview: _showSavedOrderPreview,
                      responsive: responsive,
                    ),
                  ),
                ),

                // Loading overlay
                if (_isLoadingMapData) _buildLoadingOverlay(responsive),

                // Map info overlay
                if (_selectedMapData != null && !_isLoadingMapData)
                  _buildMapInfoOverlay(responsive),

                // Coordinate count overlay
                if (_coordinates.isNotEmpty)
                  _buildCoordinateCountOverlay(responsive),

                // Map controls
                _buildMapControls(responsive),

                // Instructions
                if (!_isPanning && _coordinates.isEmpty)
                  _buildInstructionsOverlay(responsive),
              ],
            );
          },
        ),
      ),
    );
  }

  Widget _buildEnhancedControlPanel(ResponsiveValues responsive) {
    return Container(
      padding: EdgeInsets.all(responsive.cardPadding),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
          colors: [Colors.grey.shade50, Colors.white],
        ),
        borderRadius: responsive.isHorizontalLayout
            ? BorderRadius.only(
                topLeft: Radius.circular(16), bottomLeft: Radius.circular(16))
            : BorderRadius.only(
                topLeft: Radius.circular(16), topRight: Radius.circular(16)),
        boxShadow: [
          BoxShadow(
            color: Colors.grey.shade200,
            blurRadius: 8,
            offset: Offset(-2, 0),
          ),
        ],
      ),
      child: SingleChildScrollView(
        // ✅ Made scrollable to prevent overflow
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          mainAxisSize: MainAxisSize.min, // ✅ Use minimum space needed
          children: [
            // Order Name
            _buildEnhancedTextField(
              controller: _orderNameController,
              label: 'Order Name',
              icon: Icons.label_outline,
              responsive: responsive,
            ),

            SizedBox(height: responsive.cardPadding),

            // Map Selection
            _buildMapSelectionCard(responsive),

            SizedBox(height: responsive.cardPadding),

            // Status/Instructions
            _buildStatusCard(responsive),

            SizedBox(height: responsive.cardPadding),

            // Saved Order Preview
            if (_showSavedOrderPreview && _lastSavedOrder != null) ...[
              _buildSavedOrderPreview(responsive),
              SizedBox(height: responsive.cardPadding),
            ],

            // Coordinate List Header
            _buildCoordinateListHeader(responsive),

            SizedBox(height: responsive.cardPadding * 0.5),

            // Coordinate List - ✅ Fixed height container instead of Expanded
            Container(
              height: 200, // ✅ Fixed height to prevent overflow
              child: _buildCoordinateList(responsive),
            ),

            // Action Buttons
            if (_coordinates.isNotEmpty) ...[
              SizedBox(height: responsive.cardPadding),
              _buildActionButtons(responsive),
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildEnhancedTextField({
    required TextEditingController controller,
    required String label,
    required IconData icon,
    required ResponsiveValues responsive,
  }) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.blue.shade200, width: 1.5),
        boxShadow: [
          BoxShadow(
            color: Colors.blue.shade100.withOpacity(0.3),
            blurRadius: 4,
            offset: Offset(0, 2),
          ),
        ],
      ),
      child: TextField(
        controller: controller,
        style: TextStyle(fontSize: responsive.fontSize),
        decoration: InputDecoration(
          labelText: label,
          prefixIcon: Icon(icon,
              color: Colors.blue.shade600, size: responsive.iconSize),
          border: InputBorder.none,
          contentPadding: EdgeInsets.all(responsive.cardPadding),
          labelStyle: TextStyle(color: Colors.blue.shade700),
        ),
      ),
    );
  }

  Widget _buildMapSelectionCard(ResponsiveValues responsive) {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.indigo.shade50,
            Colors.white,
            Colors.purple.shade50,
          ],
        ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(
          color: Colors.indigo.shade200,
          width: 1.5,
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.indigo.shade100.withOpacity(0.4),
            blurRadius: 8,
            offset: Offset(0, 3),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Header with improved styling
          Container(
            padding: EdgeInsets.all(responsive.cardPadding),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [Colors.indigo.shade600, Colors.purple.shade600],
              ),
              borderRadius: BorderRadius.only(
                topLeft: Radius.circular(16),
                topRight: Radius.circular(16),
              ),
            ),
            child: Row(
              children: [
                Container(
                  padding: EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    color: Colors.white.withOpacity(0.2),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Icon(
                    Icons.layers_outlined,
                    color: Colors.white,
                    size: responsive.iconSize,
                  ),
                ),
                SizedBox(width: 12),
                Text(
                  'Select Map',
                  style: TextStyle(
                    fontSize: responsive.fontSize + 2,
                    fontWeight: FontWeight.bold,
                    color: Colors.white,
                  ),
                ),
                Spacer(),
                if (_selectedMapName != null)
                  Container(
                    padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                    decoration: BoxDecoration(
                      color: Colors.white.withOpacity(0.2),
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: Text(
                      'Selected',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: responsive.fontSize - 2,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                  ),
              ],
            ),
          ),

          // Content area
          if (_isLoadingMaps)
            _buildLoadingIndicator('Loading maps...', responsive)
          else if (_availableMaps.isEmpty)
            _buildNoMapsFound(responsive)
          else
            _buildMapDropdown(responsive),
        ],
      ),
    );
  }

  Widget _buildStatusCard(ResponsiveValues responsive) {
    if (_isLoadingMapData) {
      return _buildInfoCard(
        'Loading map data...',
        Icons.hourglass_empty,
        Colors.orange,
        responsive,
        showProgress: true,
      );
    }

    return _buildInfoCard(
      'Select a map above, then tap on map to add coordinates.\nCoordinates will be executed in order.',
      Icons.info_outline,
      Colors.blue,
      responsive,
      children: [
        SizedBox(height: responsive.cardPadding * 0.5),
        _buildDebugInfo(responsive),
      ],
    );
  }

  Widget _buildInfoCard(
    String message,
    IconData icon,
    Color color,
    ResponsiveValues responsive, {
    bool showProgress = false,
    List<Widget> children = const [],
  }) {
    return Container(
      padding: EdgeInsets.all(responsive.cardPadding),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.2), width: 1.5),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              if (showProgress)
                SizedBox(
                  width: responsive.iconSize,
                  height: responsive.iconSize,
                  child: CircularProgressIndicator(strokeWidth: 2),
                )
              else
                Icon(icon, color: color, size: responsive.iconSize),
              SizedBox(width: 8),
              Expanded(
                child: Text(
                  message,
                  style: TextStyle(
                    fontSize: responsive.fontSize,
                    color: color,
                  ),
                ),
              ),
            ],
          ),
          ...children,
        ],
      ),
    );
  }

  Widget _buildCoordinateListHeader(ResponsiveValues responsive) {
    return Row(
      children: [
        Icon(Icons.place,
            color: Colors.blue.shade600, size: responsive.iconSize),
        SizedBox(width: 8),
        Text(
          'Coordinates (${_coordinates.length})',
          style: TextStyle(
            fontSize: responsive.titleFontSize,
            fontWeight: FontWeight.bold,
            color: Colors.grey.shade800,
          ),
        ),
        Spacer(),
        if (_coordinates.isNotEmpty)
          Container(
            padding: EdgeInsets.symmetric(horizontal: 12, vertical: 6),
            decoration: BoxDecoration(
              color: Colors.blue.shade100,
              borderRadius: BorderRadius.circular(20),
            ),
            child: Text(
              '${_coordinates.length}',
              style: TextStyle(
                fontSize: responsive.fontSize,
                fontWeight: FontWeight.bold,
                color: Colors.blue.shade700,
              ),
            ),
          ),
      ],
    );
  }

  Widget _buildCoordinateList(ResponsiveValues responsive) {
    if (_coordinates.isEmpty) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(
              Icons.add_location_alt_outlined,
              size: responsive.iconSize * 3,
              color: Colors.grey.shade400,
            ),
            SizedBox(height: responsive.cardPadding),
            Text(
              'No coordinates added yet',
              style: TextStyle(
                fontSize: responsive.fontSize + 2,
                fontWeight: FontWeight.w500,
                color: Colors.grey.shade600,
              ),
            ),
            SizedBox(height: 8),
            Text(
              'Select a map and tap on it to start',
              textAlign: TextAlign.center,
              style: TextStyle(
                fontSize: responsive.fontSize,
                color: Colors.grey.shade500,
              ),
            ),
          ],
        ),
      );
    }

    return ListView.builder(
      itemCount: _coordinates.length,
      itemBuilder: (context, index) =>
          _buildEnhancedCoordinateItem(index, responsive),
    );
  }

  Widget _buildEnhancedCoordinateItem(int index, ResponsiveValues responsive) {
    final coord = _coordinates[index];
    final isExecuting = _isExecutingOrder && index == _currentCoordinateIndex;
    final isCompleted = _isExecutingOrder && index < _currentCoordinateIndex;

    Color cardColor = Colors.white;
    Color accentColor = Colors.blue;
    IconData statusIcon = Icons.radio_button_unchecked;

    if (isExecuting) {
      cardColor = Colors.orange.shade50;
      accentColor = Colors.orange;
      statusIcon = Icons.play_circle_filled;
    } else if (isCompleted) {
      cardColor = Colors.green.shade50;
      accentColor = Colors.green;
      statusIcon = Icons.check_circle;
    }

    return Container(
      margin: EdgeInsets.only(bottom: 8),
      decoration: BoxDecoration(
        color: cardColor,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: accentColor.withOpacity(0.2), width: 1.5),
        boxShadow: [
          BoxShadow(
            color: accentColor.withOpacity(0.3),
            blurRadius: 4,
            offset: Offset(0, 2),
          ),
        ],
      ),
      child: ListTile(
        contentPadding: EdgeInsets.all(responsive.cardPadding * 0.75),
        leading: Container(
          width: responsive.coordinateRadius * 2,
          height: responsive.coordinateRadius * 2,
          decoration: BoxDecoration(
            color: accentColor,
            shape: BoxShape.circle,
            boxShadow: [
              BoxShadow(
                color: accentColor.withOpacity(0.3),
                blurRadius: 4,
                offset: Offset(0, 2),
              ),
            ],
          ),
          child: Center(
            child: isExecuting
                ? SizedBox(
                    width: responsive.iconSize,
                    height: responsive.iconSize,
                    child: CircularProgressIndicator(
                      color: Colors.white,
                      strokeWidth: 2,
                    ),
                  )
                : isCompleted
                    ? Icon(Icons.check,
                        color: Colors.white, size: responsive.iconSize)
                    : Text(
                        '${index + 1}',
                        style: TextStyle(
                          color: Colors.white,
                          fontWeight: FontWeight.bold,
                          fontSize: responsive.fontSize,
                        ),
                      ),
          ),
        ),
        title: Text(
          coord['name'],
          style: TextStyle(
            fontSize: responsive.fontSize + 1,
            fontWeight: FontWeight.w600,
          ),
        ),
        subtitle: Text(
          '(${coord['x'].toStringAsFixed(2)}, ${coord['y'].toStringAsFixed(2)}) meters',
          style: TextStyle(
            fontSize: responsive.fontSize - 1,
            color: Colors.grey.shade600,
          ),
        ),
        trailing: _isExecutingOrder
            ? Icon(statusIcon, color: accentColor, size: responsive.iconSize)
            : PopupMenuButton<String>(
                icon: Icon(Icons.more_vert, size: responsive.iconSize),
                onSelected: (value) {
                  if (value == 'edit') {
                    _editCoordinate(index, responsive);
                  } else if (value == 'delete') {
                    _deleteCoordinate(index, responsive);
                  }
                },
                itemBuilder: (context) => [
                  PopupMenuItem(
                    value: 'edit',
                    child: Row(
                      children: [
                        Icon(Icons.edit, size: responsive.iconSize - 2),
                        SizedBox(width: 8),
                        Text('Edit',
                            style: TextStyle(fontSize: responsive.fontSize)),
                      ],
                    ),
                  ),
                  PopupMenuItem(
                    value: 'delete',
                    child: Row(
                      children: [
                        Icon(Icons.delete,
                            size: responsive.iconSize - 2, color: Colors.red),
                        SizedBox(width: 8),
                        Text('Delete',
                            style: TextStyle(
                                fontSize: responsive.fontSize,
                                color: Colors.red)),
                      ],
                    ),
                  ),
                ],
              ),
      ),
    );
  }

  Widget _buildActionButtons(ResponsiveValues responsive) {
    return Column(
      children: [
        SizedBox(height: responsive.cardPadding),

        if (!_isExecutingOrder) ...[
          // Save Order Button
          _buildEnhancedButton(
            onPressed: _isCreatingOrder ? null : _saveOrder,
            icon: _isCreatingOrder ? null : Icons.save,
            label: _isCreatingOrder ? 'Saving...' : 'Save Order',
            color: Colors.green,
            isLoading: _isCreatingOrder,
            responsive: responsive,
          ),

          SizedBox(height: 8),

          // Start Order Button
          _buildEnhancedButton(
            onPressed: _startOrderExecution,
            icon: Icons.play_arrow,
            label: 'Start Order Execution',
            color: Colors.blue,
            responsive: responsive,
          ),
        ] else ...[
          // Execution Status
          _buildExecutionStatus(responsive),

          SizedBox(height: responsive.cardPadding),

          // Stop Execution Button
          _buildEnhancedButton(
            onPressed: _stopOrderExecution,
            icon: Icons.stop,
            label: 'Stop Execution',
            color: Colors.red,
            responsive: responsive,
          ),
        ],

        SizedBox(height: 8),

        // Clear All Button
        _buildEnhancedButton(
          onPressed: _isExecutingOrder ? null : _clearAllCoordinates,
          icon: Icons.clear_all,
          label: 'Clear All',
          color: Colors.grey,
          isOutlined: true,
          responsive: responsive,
        ),
      ],
    );
  }

  Widget _buildEnhancedButton({
    required VoidCallback? onPressed,
    IconData? icon,
    required String label,
    required Color color,
    bool isLoading = false,
    bool isOutlined = false,
    required ResponsiveValues responsive,
  }) {
    return SizedBox(
      width: double.infinity,
      height: responsive.buttonHeight,
      child: isOutlined
          ? OutlinedButton.icon(
              onPressed: onPressed,
              icon: isLoading
                  ? SizedBox(
                      width: responsive.iconSize,
                      height: responsive.iconSize,
                      child: CircularProgressIndicator(strokeWidth: 2),
                    )
                  : Icon(icon, size: responsive.iconSize),
              label: Text(
                label,
                style: TextStyle(
                  fontSize: responsive.fontSize,
                  fontWeight: FontWeight.w600,
                ),
              ),
              style: OutlinedButton.styleFrom(
                side: BorderSide(color: color, width: 2),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(12),
                ),
              ),
            )
          : ElevatedButton.icon(
              onPressed: onPressed,
              icon: isLoading
                  ? SizedBox(
                      width: responsive.iconSize,
                      height: responsive.iconSize,
                      child: CircularProgressIndicator(
                        color: Colors.white,
                        strokeWidth: 2,
                      ),
                    )
                  : Icon(icon, size: responsive.iconSize),
              label: Text(
                label,
                style: TextStyle(
                  fontSize: responsive.fontSize,
                  fontWeight: FontWeight.w600,
                ),
              ),
              style: ElevatedButton.styleFrom(
                backgroundColor: color,
                foregroundColor: Colors.white,
                elevation: 4,
                shadowColor: color.withOpacity(0.4),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(12),
                ),
              ),
            ),
    );
  }

  // Overlay widgets
  Widget _buildLoadingOverlay(ResponsiveValues responsive) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.9),
        borderRadius: BorderRadius.circular(16),
      ),
      child: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            CircularProgressIndicator(),
            SizedBox(height: responsive.cardPadding),
            Text(
              'Loading map data...',
              style: TextStyle(
                fontSize: responsive.fontSize + 2,
                fontWeight: FontWeight.w500,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildMapInfoOverlay(ResponsiveValues responsive) {
    return Positioned(
      top: responsive.cardPadding,
      left: responsive.cardPadding,
      child: Container(
        padding: EdgeInsets.all(responsive.cardPadding * 0.75),
        decoration: BoxDecoration(
          color: Colors.black87,
          borderRadius: BorderRadius.circular(8),
        ),
        child: Text(
          'Map: ${_selectedMapName ?? "Default"} | ${_selectedMapData!.info.width}×${_selectedMapData!.info.height} | Device: ${widget.deviceId}',
          style: TextStyle(
            color: Colors.white,
            fontSize: responsive.fontSize - 2,
            fontWeight: FontWeight.w500,
          ),
        ),
      ),
    );
  }

  Widget _buildCoordinateCountOverlay(ResponsiveValues responsive) {
    return Positioned(
      top: responsive.cardPadding,
      right: responsive.cardPadding,
      child: Container(
        padding: EdgeInsets.all(responsive.cardPadding * 0.75),
        decoration: BoxDecoration(
          color: Colors.blue.shade700.withOpacity(0.9),
          borderRadius: BorderRadius.circular(8),
        ),
        child: Text(
          '${_coordinates.length} coordinate${_coordinates.length == 1 ? '' : 's'}',
          style: TextStyle(
            color: Colors.white,
            fontSize: responsive.fontSize - 2,
            fontWeight: FontWeight.w500,
          ),
        ),
      ),
    );
  }

  Widget _buildMapControls(ResponsiveValues responsive) {
    return Positioned(
      bottom: responsive.cardPadding,
      right: responsive.cardPadding,
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          _buildMapControlButton(
            Icons.zoom_in,
            'Zoom In',
            () => setState(() => _mapScale = (_mapScale * 1.5).clamp(0.3, 5.0)),
            responsive,
          ),
          SizedBox(height: 4),
          _buildMapControlButton(
            Icons.zoom_out,
            'Zoom Out',
            () => setState(() => _mapScale = (_mapScale / 1.5).clamp(0.3, 5.0)),
            responsive,
          ),
          SizedBox(height: 4),
          _buildMapControlButton(
            Icons.center_focus_strong,
            'Reset View',
            () => setState(() {
              _mapScale = 1.0;
              _mapOffset = Offset.zero;
            }),
            responsive,
            color: Colors.green,
          ),
          SizedBox(height: 8),
          Container(
            padding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
            decoration: BoxDecoration(
              color: Colors.black87,
              borderRadius: BorderRadius.circular(4),
            ),
            child: Text(
              '${(_mapScale * 100).toStringAsFixed(0)}%',
              style: TextStyle(
                color: Colors.white,
                fontSize: responsive.fontSize - 3,
                fontWeight: FontWeight.w500,
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMapControlButton(
    IconData icon,
    String tooltip,
    VoidCallback onPressed,
    ResponsiveValues responsive, {
    Color color = Colors.blue,
  }) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(8),
        boxShadow: [
          BoxShadow(
            color: Colors.black26,
            blurRadius: 4,
            offset: Offset(0, 2),
          ),
        ],
      ),
      child: IconButton(
        onPressed: onPressed,
        icon: Icon(icon, color: color, size: responsive.iconSize),
        tooltip: tooltip,
        iconSize: responsive.iconSize,
      ),
    );
  }

  Widget _buildInstructionsOverlay(ResponsiveValues responsive) {
    return Positioned(
      bottom: responsive.cardPadding,
      left: responsive.cardPadding,
      child: Container(
        padding: EdgeInsets.all(responsive.cardPadding),
        decoration: BoxDecoration(
          color: Colors.blue.shade50.withOpacity(0.95),
          borderRadius: BorderRadius.circular(12),
          border: Border.all(color: Colors.blue.shade200),
          boxShadow: [
            BoxShadow(
              color: Colors.blue.shade100.withOpacity(0.5),
              blurRadius: 8,
              offset: Offset(0, 4),
            ),
          ],
        ),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          mainAxisSize: MainAxisSize.min,
          children: [
            Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                Icon(Icons.touch_app,
                    color: Colors.blue, size: responsive.iconSize),
                SizedBox(width: 6),
                Text(
                  'Map Controls',
                  style: TextStyle(
                    fontWeight: FontWeight.bold,
                    color: Colors.blue.shade700,
                    fontSize: responsive.fontSize,
                  ),
                ),
              ],
            ),
            SizedBox(height: 4),
            Text(
              '• Drag to pan around\n• Pinch to zoom\n• Tap to place coordinates',
              style: TextStyle(
                fontSize: responsive.fontSize - 1,
                color: Colors.blue.shade600,
              ),
            ),
          ],
        ),
      ),
    );
  }

  // Helper widgets for control panel
  Widget _buildLoadingIndicator(String message, ResponsiveValues responsive) {
    return Padding(
      padding: EdgeInsets.all(responsive.cardPadding),
      child: Row(
        children: [
          SizedBox(
            width: responsive.iconSize,
            height: responsive.iconSize,
            child: CircularProgressIndicator(strokeWidth: 2),
          ),
          SizedBox(width: 8),
          Text(message, style: TextStyle(fontSize: responsive.fontSize)),
        ],
      ),
    );
  }

  Widget _buildNoMapsFound(ResponsiveValues responsive) {
    return Container(
      margin: EdgeInsets.all(responsive.cardPadding),
      padding: EdgeInsets.all(responsive.cardPadding * 1.5),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.orange.shade200),
        boxShadow: [
          BoxShadow(
            color: Colors.orange.shade100.withOpacity(0.3),
            blurRadius: 4,
            offset: Offset(0, 2),
          ),
        ],
      ),
      child: Column(
        children: [
          Container(
            padding: EdgeInsets.all(16),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [Colors.orange.shade100, Colors.amber.shade100],
              ),
              shape: BoxShape.circle,
            ),
            child: Icon(
              Icons.explore_off_outlined,
              color: Colors.orange.shade600,
              size: responsive.iconSize * 2,
            ),
          ),
          SizedBox(height: 12),
          Text(
            'No saved maps found',
            style: TextStyle(
              color: Colors.orange.shade700,
              fontWeight: FontWeight.bold,
              fontSize: responsive.fontSize + 1,
            ),
          ),
          SizedBox(height: 6),
          Text(
            'Using default map for coordinate placement',
            textAlign: TextAlign.center,
            style: TextStyle(
              color: Colors.grey.shade600,
              fontSize: responsive.fontSize - 1,
            ),
          ),
          SizedBox(height: 12),
          ElevatedButton.icon(
            onPressed: _loadAvailableMaps,
            icon: Icon(Icons.refresh, size: responsive.iconSize - 2),
            label: Text('Retry',
                style: TextStyle(fontSize: responsive.fontSize - 1)),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.orange.shade500,
              foregroundColor: Colors.white,
              padding: EdgeInsets.symmetric(horizontal: 16, vertical: 8),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(8),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMapDropdown(ResponsiveValues responsive) {
    return Container(
      margin: EdgeInsets.all(responsive.cardPadding),
      padding: EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.indigo.shade200),
        boxShadow: [
          BoxShadow(
            color: Colors.indigo.shade100.withOpacity(0.3),
            blurRadius: 4,
            offset: Offset(0, 2),
          ),
        ],
      ),
      child: DropdownButtonHideUnderline(
        child: DropdownButton<String>(
          value: _selectedMapName,
          isExpanded: true,
          hint: Row(
            children: [
              Icon(Icons.explore_outlined,
                  color: Colors.grey.shade500, size: responsive.iconSize),
              SizedBox(width: 8),
              Text(
                'Choose a map',
                style: TextStyle(
                  fontSize: responsive.fontSize,
                  color: Colors.grey.shade600,
                ),
              ),
            ],
          ),
          style: TextStyle(
            fontSize: responsive.fontSize,
            color: Colors.grey.shade800,
            fontWeight: FontWeight.w500,
          ),
          icon: Icon(Icons.keyboard_arrow_down, color: Colors.indigo.shade400),
          items: _availableMaps.map((map) {
            final isSelected = map['name'] == _selectedMapName;
            return DropdownMenuItem<String>(
              value: map['name'],
              child: Container(
                padding: EdgeInsets.symmetric(vertical: 8, horizontal: 4),
                decoration: BoxDecoration(
                  color:
                      isSelected ? Colors.indigo.shade50 : Colors.transparent,
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Row(
                  children: [
                    Container(
                      padding: EdgeInsets.all(6),
                      decoration: BoxDecoration(
                        gradient: LinearGradient(
                          colors: isSelected
                              ? [Colors.indigo.shade400, Colors.purple.shade400]
                              : [Colors.grey.shade400, Colors.grey.shade500],
                        ),
                        borderRadius: BorderRadius.circular(6),
                      ),
                      child: Icon(
                        Icons.map,
                        color: Colors.white,
                        size: responsive.iconSize - 2,
                      ),
                    ),
                    SizedBox(width: 12),
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Text(
                            map['name'] ?? 'Unnamed Map',
                            overflow: TextOverflow.ellipsis,
                            style: TextStyle(
                              fontWeight: isSelected
                                  ? FontWeight.bold
                                  : FontWeight.w500,
                              color: isSelected
                                  ? Colors.indigo.shade700
                                  : Colors.grey.shade700,
                            ),
                          ),
                          if (map['timestamp'] != null)
                            Text(
                              'Last updated: ${_formatTimestamp(map['timestamp'])}',
                              style: TextStyle(
                                fontSize: responsive.fontSize - 3,
                                color: Colors.grey.shade500,
                              ),
                            ),
                        ],
                      ),
                    ),
                    if (isSelected)
                      Icon(
                        Icons.check_circle,
                        color: Colors.indigo.shade600,
                        size: responsive.iconSize,
                      ),
                  ],
                ),
              ),
            );
          }).toList(),
          onChanged: (String? newValue) {
            if (newValue != null && newValue != _selectedMapName) {
              setState(() => _selectedMapName = newValue);
              _loadSelectedMapData(newValue);
            }
          },
        ),
      ),
    );
  }

  // Helper method to format timestamp
  String _formatTimestamp(String? timestamp) {
    if (timestamp == null) return 'Unknown';
    try {
      final date = DateTime.parse(timestamp);
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
    } catch (e) {
      return 'Unknown';
    }
  }

  Widget _buildDebugInfo(ResponsiveValues responsive) {
    return Container(
      padding: EdgeInsets.all(responsive.cardPadding * 0.5),
      decoration: BoxDecoration(
        color: Colors.grey.shade100,
        borderRadius: BorderRadius.circular(4),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Debug Info:',
            style: TextStyle(
              fontSize: responsive.fontSize - 4,
              fontWeight: FontWeight.bold,
              color: Colors.grey.shade700,
            ),
          ),
          Text('Device: ${widget.deviceId}',
              style: TextStyle(
                  fontSize: responsive.fontSize - 4,
                  color: Colors.grey.shade600)),
          Text('Available Maps: ${_availableMaps.length}',
              style: TextStyle(
                  fontSize: responsive.fontSize - 4,
                  color: Colors.grey.shade600)),
          Text('Selected: ${_selectedMapName ?? "None"}',
              style: TextStyle(
                  fontSize: responsive.fontSize - 4,
                  color: Colors.grey.shade600)),
          Text(
              'Map Data: ${_selectedMapData != null ? "Loaded" : "Using Default"}',
              style: TextStyle(
                  fontSize: responsive.fontSize - 4,
                  color: Colors.grey.shade600)),
        ],
      ),
    );
  }

  Widget _buildSavedOrderPreview(ResponsiveValues responsive) {
    return Container(
      padding: EdgeInsets.all(responsive.cardPadding),
      decoration: BoxDecoration(
        color: Colors.green.shade50,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.green.shade200),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(Icons.check_circle,
                  color: Colors.green.shade700, size: responsive.iconSize),
              SizedBox(width: 8),
              Text(
                'Previously Saved Order',
                style: TextStyle(
                  fontSize: responsive.fontSize,
                  fontWeight: FontWeight.bold,
                  color: Colors.green.shade700,
                ),
              ),
              Spacer(),
              IconButton(
                icon: Icon(Icons.visibility_off,
                    color: Colors.green.shade600, size: responsive.iconSize),
                onPressed: () => setState(() => _showSavedOrderPreview = false),
                tooltip: 'Hide saved order preview',
                padding: EdgeInsets.all(4),
                constraints: BoxConstraints(),
              ),
            ],
          ),
          SizedBox(height: 8),
          Text(
            '✅ "${_lastSavedOrder!['name']}" with ${(_lastSavedOrder!['coordinates'] as List).length} waypoints',
            style: TextStyle(
                fontSize: responsive.fontSize - 1,
                color: Colors.green.shade600),
          ),
          SizedBox(height: 8),
          Text(
            'Green markers on the map show the saved order. The order is available on the dashboard.',
            style: TextStyle(
              fontSize: responsive.fontSize - 2,
              color: Colors.green.shade600,
              fontStyle: FontStyle.italic,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildExecutionStatus(ResponsiveValues responsive) {
    return Container(
      padding: EdgeInsets.all(responsive.cardPadding),
      decoration: BoxDecoration(
        color: Colors.orange.shade50,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.orange.shade200),
      ),
      child: Column(
        children: [
          Text(
            'Executing Order...',
            style: TextStyle(
              fontSize: responsive.fontSize + 2,
              fontWeight: FontWeight.bold,
              color: Colors.orange.shade700,
            ),
          ),
          SizedBox(height: 8),
          Text(
            'Coordinate ${_currentCoordinateIndex + 1} of ${_coordinates.length}',
            style: TextStyle(fontSize: responsive.fontSize),
          ),
          SizedBox(height: 8),
          LinearProgressIndicator(
            value: (_currentCoordinateIndex + 1) / _coordinates.length,
            backgroundColor: Colors.grey.shade300,
            valueColor: AlwaysStoppedAnimation<Color>(Colors.orange),
          ),
        ],
      ),
    );
  }

  // Keep all the existing method implementations but update dialog styling
  void _editCoordinate(int index, ResponsiveValues responsive) {
    _coordinateNameController.text = _coordinates[index]['name'];

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: Text('Edit Coordinate',
            style: TextStyle(fontSize: responsive.titleFontSize)),
        content: TextField(
          controller: _coordinateNameController,
          style: TextStyle(fontSize: responsive.fontSize),
          decoration: InputDecoration(
            labelText: 'Coordinate Name',
            border: OutlineInputBorder(borderRadius: BorderRadius.circular(8)),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child:
                Text('Cancel', style: TextStyle(fontSize: responsive.fontSize)),
          ),
          ElevatedButton(
            onPressed: () {
              setState(() {
                _coordinates[index]['name'] =
                    _coordinateNameController.text.trim();
              });
              Navigator.of(context).pop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.blue),
            child: Text('Save',
                style: TextStyle(
                    fontSize: responsive.fontSize, color: Colors.white)),
          ),
        ],
      ),
    );
  }

  void _deleteCoordinate(int index, ResponsiveValues responsive) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: Text('Delete Coordinate',
            style: TextStyle(fontSize: responsive.titleFontSize)),
        content: Text('Delete "${_coordinates[index]['name']}"?',
            style: TextStyle(fontSize: responsive.fontSize)),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child:
                Text('Cancel', style: TextStyle(fontSize: responsive.fontSize)),
          ),
          ElevatedButton(
            onPressed: () {
              setState(() => _coordinates.removeAt(index));
              Navigator.of(context).pop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child: Text('Delete',
                style: TextStyle(
                    fontSize: responsive.fontSize, color: Colors.white)),
          ),
        ],
      ),
    );
  }

  void _clearAllCoordinates() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: Text('Clear All Coordinates'),
        content: Text('Remove all coordinates?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              setState(() => _coordinates.clear());
              Navigator.of(context).pop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
            child: Text('Clear All', style: TextStyle(color: Colors.white)),
          ),
        ],
      ),
    );
  }

  // Keep all existing coordinate mapping and API methods unchanged
  Future<void> _loadAvailableMaps() async {
    setState(() => _isLoadingMaps = true);

    try {
      print('🔍 Loading maps for device: ${widget.deviceId}');
      final maps = await _apiService.getSavedMapsEnhanced(widget.deviceId);

      setState(() {
        _availableMaps = maps;
        if (_availableMaps.isNotEmpty && _selectedMapName == null) {
          _selectedMapName = _availableMaps.first['name'];
          _loadSelectedMapData(_selectedMapName!);
        }
      });

      if (_availableMaps.isEmpty) {
        await _tryAlternativeMapLoading();
      }
    } catch (e) {
      await _tryAlternativeMapLoading();
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
            content: Text('Failed to load maps: $e'),
            backgroundColor: Colors.orange),
      );
    } finally {
      setState(() => _isLoadingMaps = false);
    }
  }

  Future<void> _tryAlternativeMapLoading() async {
    try {
      final regularMaps = await _apiService.getSavedMaps(widget.deviceId);
      if (regularMaps.isNotEmpty) {
        setState(() {
          _availableMaps = regularMaps;
          if (_selectedMapName == null && _availableMaps.isNotEmpty) {
            _selectedMapName = _availableMaps.first['name'];
            _loadSelectedMapData(_selectedMapName!);
          }
        });
        return;
      }

      final mapDataResponse = await _apiService.getMapData(widget.deviceId);
      if (mapDataResponse['success'] == true) {
        setState(() {
          _availableMaps = [
            {
              'name': 'Current Map',
              'deviceId': widget.deviceId,
              'timestamp': DateTime.now().toIso8601String()
            }
          ];
          _selectedMapName = 'Current Map';
          _selectedMapData = widget.mapData;
        });
        return;
      }

      setState(() {
        _availableMaps = [
          {
            'name': 'Default Map',
            'deviceId': widget.deviceId,
            'timestamp': DateTime.now().toIso8601String()
          }
        ];
        _selectedMapName = 'Default Map';
        _selectedMapData = widget.mapData;
      });
    } catch (e) {
      setState(() {
        _availableMaps = [
          {
            'name': 'Default Map',
            'deviceId': widget.deviceId,
            'timestamp': DateTime.now().toIso8601String()
          }
        ];
        _selectedMapName = 'Default Map';
        _selectedMapData = widget.mapData;
      });
    }
  }

  Future<void> _loadSelectedMapData(String mapName) async {
    setState(() => _isLoadingMapData = true);

    try {
      final mapData = await _apiService.getMapData(widget.deviceId);

      if (mapData['success'] == true && mapData['mapData'] != null) {
        final rawMapData = mapData['mapData'];
        final mapInfo = rawMapData['info'] ?? {};
        final width = mapInfo['width'] ?? 164;
        final height = mapInfo['height'] ?? 145;
        final resolution = mapInfo['resolution'] ?? 0.05;

        var originX = -2.02;
        var originY = -5.21;

        if (mapInfo.containsKey('origin')) {
          final rawOrigin = mapInfo['origin'];
          if (rawOrigin is List && rawOrigin.length >= 2) {
            originX = (rawOrigin[0] as num?)?.toDouble() ?? -2.02;
            originY = (rawOrigin[1] as num?)?.toDouble() ?? -5.21;
          } else if (rawOrigin is Map) {
            originX = (rawOrigin['x'] as num?)?.toDouble() ?? -2.02;
            originY = (rawOrigin['y'] as num?)?.toDouble() ?? -5.21;
          }
        }

        List<int> occupancyData = [];
        final rawOccupancy =
            rawMapData['occupancyData'] ?? rawMapData['data'] ?? [];
        if (rawOccupancy is List) {
          occupancyData =
              rawOccupancy.map((e) => (e as num?)?.toInt() ?? -1).toList();
        }

        setState(() {
          _selectedMapData = MapData(
            deviceId: widget.deviceId,
            timestamp: DateTime.now(),
            info: MapInfo(
              width: width,
              height: height,
              resolution: resolution,
              origin: odom.Position(x: originX, y: originY, z: 0.0),
              originOrientation: odom.Orientation(x: 0, y: 0, z: 0, w: 1),
            ),
            occupancyData: occupancyData,
            shapes: [],
            version: 1,
          );
        });
      } else {
        throw Exception(
            'Invalid map data response: ${mapData['error'] ?? 'Unknown error'}');
      }
    } catch (e) {
      setState(() {
        _selectedMapData = MapData(
          deviceId: widget.deviceId,
          timestamp: DateTime.now(),
          info: MapInfo(
            width: 164,
            height: 145,
            resolution: 0.05,
            origin: odom.Position(x: -2.02, y: -5.21, z: 0.0),
            originOrientation: odom.Orientation(x: 0, y: 0, z: 0, w: 1),
          ),
          occupancyData: widget.mapData.occupancyData,
          shapes: [],
          version: 1,
        );
      });

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
            content: Text('Using fallback map data with YAML origin'),
            backgroundColor: Colors.orange),
      );
    } finally {
      setState(() => _isLoadingMapData = false);
    }
  }

  // Keep all existing gesture handlers, coordinate conversion, and API methods unchanged
  void _handleMapTap(TapDownDetails details) {
    if (_isExecutingOrder || _isPanning || _isLoadingMapData) return;

    final coordinates = _screenToMapCoordinates(details.localPosition);
    _showCoordinateNameDialog(coordinates);
  }

  void _handleScaleStart(ScaleStartDetails details) {
    _isPanning = false;
    _lastPanPosition = details.localFocalPoint;
  }

  void _handleScaleUpdate(ScaleUpdateDetails details) {
    setState(() {
      if (details.scale != 1.0) {
        final newScale = (_mapScale * details.scale).clamp(0.3, 5.0);
        if (newScale != _mapScale) {
          _mapScale = newScale;
        }
      }

      if (details.scale == 1.0 && _lastPanPosition != null) {
        _isPanning = true;
        final delta = details.localFocalPoint - _lastPanPosition!;
        _mapOffset += delta;
        _lastPanPosition = details.localFocalPoint;
      } else {
        _lastPanPosition = details.localFocalPoint;
        _mapOffset += details.focalPointDelta;
      }
    });
  }

  void _handleScaleEnd(ScaleEndDetails details) {
    _lastPanPosition = null;
    if (_isPanning) {
      Future.delayed(Duration(milliseconds: 100), () {
        _isPanning = false;
      });
    }
  }

  void _showCoordinateNameDialog(Map<String, double> coordinates) {
    _coordinateNameController.clear();

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: Text('Add Coordinate'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(
                'Position: (${coordinates['x']!.toStringAsFixed(2)}, ${coordinates['y']!.toStringAsFixed(2)}) meters'),
            SizedBox(height: 16),
            TextField(
              controller: _coordinateNameController,
              decoration: InputDecoration(
                labelText: 'Coordinate Name',
                hintText: 'e.g., Point A, Station 1',
                border:
                    OutlineInputBorder(borderRadius: BorderRadius.circular(8)),
              ),
              autofocus: true,
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              _addCoordinate(
                  coordinates, _coordinateNameController.text.trim());
              Navigator.of(context).pop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.blue),
            child: Text('Add', style: TextStyle(color: Colors.white)),
          ),
        ],
      ),
    );
  }

  void _addCoordinate(Map<String, double> coordinates, String name) {
    setState(() {
      _coordinates.add({
        'name': name.isNotEmpty ? name : 'Point ${_coordinates.length + 1}',
        'x': coordinates['x']!,
        'y': coordinates['y']!,
      });
    });

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
          content: Text('Coordinate added!'), backgroundColor: Colors.green),
    );
  }

  Map<String, double> _screenToMapCoordinates(Offset screenPosition) {
    final mapData = _selectedMapData ?? widget.mapData;
    final adjustedX = (screenPosition.dx - _mapOffset.dx) / _mapScale;
    final adjustedY = (screenPosition.dy - _mapOffset.dy) / _mapScale;

    final resolution = mapData.info.resolution;
    final originX = mapData.info.origin.x;
    final originY = mapData.info.origin.y;
    final mapWidth = mapData.info.width;
    final mapHeight = mapData.info.height;

    final mapPhysicalWidth = mapWidth * resolution;
    final mapPhysicalHeight = mapHeight * resolution;

    final minX = originX;
    // final maxX = originX + mapPhysicalWidth; // ✅ Removed unused variable
    final minY = originY;
    // final maxY = originY + mapPhysicalHeight; // ✅ Removed unused variable

    final canvasWidth = _canvasSize.width / _mapScale;
    final canvasHeight = _canvasSize.height / _mapScale;

    final worldX = minX + (adjustedX / canvasWidth) * mapPhysicalWidth;
    final worldY = minY + (adjustedY / canvasHeight) * mapPhysicalHeight;

    return {'x': worldX, 'y': worldY};
  }

  // Keep all existing API methods (saveOrder, startOrderExecution, etc.) unchanged
  Future<void> _saveOrder() async {
    setState(() => _isCreatingOrder = true);

    try {
      final orderData = {
        'id': 'order_${DateTime.now().millisecondsSinceEpoch}',
        'name': _orderNameController.text.trim(),
        'deviceId': widget.deviceId,
        'coordinates': _coordinates,
        'status': 'pending',
        'currentCoordinate': 0,
        'createdAt': DateTime.now().toIso8601String(),
      };

      final response = await _apiService.createSimpleCoordinateOrder(
        deviceId: widget.deviceId,
        name: orderData['name'] as String,
        coordinates: _coordinates,
      );

      if (response['success'] == true) {
        widget.onOrderCreated(orderData);
        await _showOrderSavedDialog(orderData);
      } else {
        throw Exception(response['error'] ?? 'Failed to save order');
      }
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
            content: Text('❌ Save failed: $e'), backgroundColor: Colors.red),
      );
    } finally {
      setState(() => _isCreatingOrder = false);
    }
  }

  Future<void> _startOrderExecution() async {
    setState(() {
      _isExecutingOrder = true;
      _currentCoordinateIndex = 0;
    });

    try {
      await _executeNextCoordinate();
    } catch (e) {
      _showRestartDialog('Execution failed: $e');
    }
  }

  Future<void> _executeNextCoordinate() async {
    if (_currentCoordinateIndex >= _coordinates.length) {
      _showCompletionDialog();
      return;
    }

    final coord = _coordinates[_currentCoordinateIndex];

    try {
      final response = await _apiService.publishGoalToTargetPose(
        deviceId: widget.deviceId,
        x: coord['x'],
        y: coord['y'],
        orientation: 0.0,
      );

      if (response['success'] == true) {
        await _waitForNavigationFeedback();
      } else {
        throw Exception(response['error'] ?? 'Failed to publish goal');
      }
    } catch (e) {
      _showRestartDialog(
          'Failed to execute coordinate ${_currentCoordinateIndex + 1}: $e');
    }
  }

  Future<void> _waitForNavigationFeedback() async {
    await Future.delayed(Duration(seconds: 3));
    final success = DateTime.now().millisecond % 2 == 0;

    if (success) {
      setState(() => _currentCoordinateIndex++);
      await _executeNextCoordinate();
    } else {
      _showRestartDialog(
          'Navigation to coordinate ${_currentCoordinateIndex + 1} failed');
    }
  }

  void _stopOrderExecution() {
    setState(() {
      _isExecutingOrder = false;
      _currentCoordinateIndex = 0;
    });
  }

  void _showRestartDialog(String error) {
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: Text('❌ Execution Failed'),
        content: Text('$error\n\nWould you like to restart the order?'),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
              _stopOrderExecution();
            },
            child: Text('Cancel'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              _startOrderExecution();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.blue),
            child: Text('Restart Order', style: TextStyle(color: Colors.white)),
          ),
        ],
      ),
    );
  }

  void _showCompletionDialog() {
    setState(() => _isExecutingOrder = false);

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: Text('✅ Order Completed'),
        content: Text('All coordinates have been executed successfully!'),
        actions: [
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              Navigator.of(context).pop();
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.green),
            child: Text('OK', style: TextStyle(color: Colors.white)),
          ),
        ],
      ),
    );
  }

  Future<void> _showOrderSavedDialog(Map<String, dynamic> orderData) async {
    setState(() {
      _lastSavedOrder = orderData;
      _showSavedOrderPreview = true;
    });

    return showDialog(
      context: context,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: Row(
          children: [
            Icon(Icons.check_circle, color: Colors.green, size: 28),
            SizedBox(width: 8),
            Text('✅ Order Saved!'),
          ],
        ),
        content: Container(
          width: double.maxFinite,
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'Your coordinate order has been saved successfully!',
                style: TextStyle(fontSize: 16, fontWeight: FontWeight.w500),
              ),
              SizedBox(height: 16),
              Container(
                padding: EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: Colors.blue.shade50,
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: Colors.blue.shade200),
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      children: [
                        Icon(Icons.label,
                            color: Colors.blue.shade700, size: 20),
                        SizedBox(width: 8),
                        Text(
                          'Order Name:',
                          style: TextStyle(
                            fontWeight: FontWeight.bold,
                            color: Colors.blue.shade700,
                          ),
                        ),
                      ],
                    ),
                    SizedBox(height: 4),
                    Text(orderData['name'] as String,
                        style: TextStyle(fontSize: 16)),
                    SizedBox(height: 12),
                    Row(
                      children: [
                        Icon(Icons.place,
                            color: Colors.blue.shade700, size: 20),
                        SizedBox(width: 8),
                        Text(
                          'Coordinates:',
                          style: TextStyle(
                            fontWeight: FontWeight.bold,
                            color: Colors.blue.shade700,
                          ),
                        ),
                      ],
                    ),
                    SizedBox(height: 4),
                    Text(
                      '${(orderData['coordinates'] as List).length} waypoints',
                      style: TextStyle(fontSize: 16),
                    ),
                  ],
                ),
              ),
              SizedBox(height: 12),
              Text(
                'The order is now available on the dashboard and ready for execution.',
                style: TextStyle(
                  fontSize: 14,
                  color: Colors.grey.shade600,
                  fontStyle: FontStyle.italic,
                ),
              ),
            ],
          ),
        ),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
              _resetOrderForm();
            },
            child: Text('Create Another Order'),
          ),
          ElevatedButton(
            onPressed: () {
              Navigator.of(context).pop();
              Navigator.of(context).pop();
              Future.delayed(Duration(milliseconds: 500), () {
                if (context.mounted) {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: Text(
                          '💡 Your order "${orderData['name']}" is now visible in the Orders section below!'),
                      backgroundColor: Colors.blue,
                      duration: Duration(seconds: 4),
                      action: SnackBarAction(
                        label: 'Got it',
                        textColor: Colors.white,
                        onPressed: () {},
                      ),
                    ),
                  );
                }
              });
            },
            style: ElevatedButton.styleFrom(backgroundColor: Colors.blue),
            child:
                Text('Go to Dashboard', style: TextStyle(color: Colors.white)),
          ),
        ],
      ),
    );
  }

  void _resetOrderForm() {
    setState(() {
      _coordinates.clear();
      _isCreatingOrder = false;
      _isExecutingOrder = false;
      _currentCoordinateIndex = 0;
      _showSavedOrderPreview = false;
      _lastSavedOrder = null;
      _orderNameController.text =
          'Order ${DateTime.now().millisecondsSinceEpoch}';
      _coordinateNameController.clear();
    });

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('✨ Ready to create a new order!'),
        backgroundColor: Colors.blue,
        duration: Duration(seconds: 2),
      ),
    );
  }

  @override
  void dispose() {
    _orderNameController.dispose();
    _coordinateNameController.dispose();
    super.dispose();
  }
}

// Enhanced Map Painter with responsive design
class EnhancedMapPainter extends CustomPainter {
  final MapData mapData;
  final List<Map<String, dynamic>> coordinates;
  final double mapScale;
  final Offset mapOffset;
  final int currentExecutingIndex;
  final Size canvasSize;
  final Map<String, dynamic>? savedOrder;
  final bool showSavedOrderPreview;
  final ResponsiveValues responsive;

  EnhancedMapPainter({
    required this.mapData,
    required this.coordinates,
    required this.mapScale,
    required this.mapOffset,
    required this.currentExecutingIndex,
    required this.canvasSize,
    this.savedOrder,
    this.showSavedOrderPreview = false,
    required this.responsive,
  });

  @override
  void paint(Canvas canvas, Size size) {
    canvas.save();
    canvas.translate(mapOffset.dx, mapOffset.dy);
    canvas.scale(mapScale);

    _drawRealCoordinateGrid(canvas, size);
    _drawFittedMap(canvas, size);

    if (showSavedOrderPreview && savedOrder != null) {
      _drawSavedOrderPreview(canvas, size);
    }

    _drawCoordinates(canvas, size);
    canvas.restore();
  }

  void _drawRealCoordinateGrid(Canvas canvas, Size size) {
    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;

    final resolution = mapData.info.resolution;
    final originX = mapData.info.origin.x;
    final originY = mapData.info.origin.y;
    final mapWidth = mapData.info.width;
    final mapHeight = mapData.info.height;

    final mapPhysicalWidth = mapWidth * resolution;
    final mapPhysicalHeight = mapHeight * resolution;
    final minX = originX;
    final maxX = originX + mapPhysicalWidth;
    final minY = originY;
    final maxY = originY + mapPhysicalHeight;

    double gridSpacing = 1.0;
    if (mapPhysicalWidth < 5.0) {
      gridSpacing = 0.5;
    } else if (mapPhysicalWidth > 20.0) {
      gridSpacing = 2.0;
    }

    // Background
    final bgPaint = Paint()..color = Colors.grey.shade50;
    canvas.drawRect(
        Rect.fromLTWH(0, 0, adjustedWidth, adjustedHeight), bgPaint);

    final pixelsPerMeterX = adjustedWidth / mapPhysicalWidth;
    final pixelsPerMeterY = adjustedHeight / mapPhysicalHeight;

    // Minor grid
    final minorGridPaint = Paint()
      ..color = Colors.blue.withOpacity(0.1)
      ..strokeWidth = 0.5;

    final minorSpacing = gridSpacing / 4;
    for (double worldX = minX; worldX <= maxX; worldX += minorSpacing) {
      final pixelX = (worldX - minX) * pixelsPerMeterX;
      canvas.drawLine(
          Offset(pixelX, 0), Offset(pixelX, adjustedHeight), minorGridPaint);
    }

    for (double worldY = minY; worldY <= maxY; worldY += minorSpacing) {
      final pixelY = (worldY - minY) * pixelsPerMeterY;
      canvas.drawLine(
          Offset(0, pixelY), Offset(adjustedWidth, pixelY), minorGridPaint);
    }

    // Major grid
    final majorGridPaint = Paint()
      ..color = Colors.blue.withOpacity(0.3)
      ..strokeWidth = 1.0;

    for (double worldX = _roundToGrid(minX, gridSpacing);
        worldX <= maxX;
        worldX += gridSpacing) {
      final pixelX = (worldX - minX) * pixelsPerMeterX;
      canvas.drawLine(
          Offset(pixelX, 0), Offset(pixelX, adjustedHeight), majorGridPaint);
    }

    for (double worldY = _roundToGrid(minY, gridSpacing);
        worldY <= maxY;
        worldY += gridSpacing) {
      final pixelY = (worldY - minY) * pixelsPerMeterY;
      canvas.drawLine(
          Offset(0, pixelY), Offset(adjustedWidth, pixelY), majorGridPaint);
    }

    // Axes
    final axisPaint = Paint()
      ..color = Colors.red.withOpacity(0.6)
      ..strokeWidth = 1.5;

    if (minX <= 0 && maxX >= 0) {
      final originPixelX = (0 - minX) * pixelsPerMeterX;
      canvas.drawLine(Offset(originPixelX, 0),
          Offset(originPixelX, adjustedHeight), axisPaint);
    }

    if (minY <= 0 && maxY >= 0) {
      final originPixelY = (0 - minY) * pixelsPerMeterY;
      canvas.drawLine(Offset(0, originPixelY),
          Offset(adjustedWidth, originPixelY), axisPaint);
    }

    _drawRealAxisLabels(canvas, size, pixelsPerMeterX, pixelsPerMeterY, minX,
        maxX, minY, maxY, gridSpacing);
  }

  double _roundToGrid(double value, double gridSpacing) {
    return (value / gridSpacing).ceil() * gridSpacing;
  }

  void _drawRealAxisLabels(
      Canvas canvas,
      Size size,
      double pixelsPerMeterX,
      double pixelsPerMeterY,
      double minX,
      double maxX,
      double minY,
      double maxY,
      double gridSpacing) {
    final textStyle = TextStyle(
      color: Colors.blue.shade700,
      fontSize: responsive.fontSize - 4,
      fontWeight: FontWeight.bold,
    );

    // X-axis labels
    for (double worldX = _roundToGrid(minX, gridSpacing);
        worldX <= maxX;
        worldX += gridSpacing) {
      final pixelX = (worldX - minX) * pixelsPerMeterX;
      final textPainter = TextPainter(
        text: TextSpan(text: worldX.toStringAsFixed(1), style: textStyle),
        textDirection: TextDirection.ltr,
      );
      textPainter.layout();
      textPainter.paint(canvas, Offset(pixelX - textPainter.width / 2, -18));
    }

    // Y-axis labels
    for (double worldY = _roundToGrid(minY, gridSpacing);
        worldY <= maxY;
        worldY += gridSpacing) {
      final pixelY = (worldY - minY) * pixelsPerMeterY;
      final textPainter = TextPainter(
        text: TextSpan(text: worldY.toStringAsFixed(1), style: textStyle),
        textDirection: TextDirection.ltr,
      );
      textPainter.layout();
      textPainter.paint(canvas,
          Offset(-textPainter.width - 5, pixelY - textPainter.height / 2));
    }
  }

  void _drawFittedMap(Canvas canvas, Size size) {
    try {
      if (mapData.occupancyData.isEmpty) return;

      final adjustedWidth = size.width / mapScale;
      final adjustedHeight = size.height / mapScale;

      final mapWidth = mapData.info.width;
      final mapHeight = mapData.info.height;
      final resolution = mapData.info.resolution;

      final mapPhysicalWidth = mapWidth * resolution;
      final mapPhysicalHeight = mapHeight * resolution;

      final scaleX = adjustedWidth / mapPhysicalWidth;
      final scaleY = adjustedHeight / mapPhysicalHeight;
      final cellSize = (scaleX < scaleY ? scaleX : scaleY).clamp(0.5, 10.0);

      for (int y = 0; y < mapHeight; y++) {
        for (int x = 0; x < mapWidth; x++) {
          final index = y * mapWidth + x;

          if (index < mapData.occupancyData.length) {
            final occupancyValue = mapData.occupancyData[index];
            Color cellColor = _getOccupancyColor(occupancyValue);

            final screenX = x * cellSize;
            final screenY = y * cellSize;

            final cellPaint = Paint()..color = cellColor;
            final cellRect =
                Rect.fromLTWH(screenX, screenY, cellSize, cellSize);

            if (cellColor != Colors.white) {
              canvas.drawRect(cellRect, cellPaint);
            }
          }
        }
      }
    } catch (e) {
      print('❌ Error drawing fitted map: $e');
    }
  }

  Color _getOccupancyColor(int occupancyValue) {
    if (occupancyValue == -1) {
      return Colors.grey.shade300;
    } else if (occupancyValue <= 10) {
      return Colors.white;
    } else if (occupancyValue >= 90) {
      return Colors.black;
    } else {
      final probability = occupancyValue / 100.0;
      final greyValue = (255 * (1.0 - probability)).round().clamp(0, 255);
      return Color.fromARGB(255, greyValue, greyValue, greyValue);
    }
  }

  void _drawCoordinates(Canvas canvas, Size size) {
    for (int i = 0; i < coordinates.length; i++) {
      final coord = coordinates[i];
      final screenPos = _worldToScreen(coord['x'], coord['y'], size);

      if (screenPos != null) {
        _drawCoordinate(canvas, screenPos, i + 1, i == currentExecutingIndex,
            i < currentExecutingIndex);

        if (i < coordinates.length - 1) {
          final nextCoord = coordinates[i + 1];
          final nextScreenPos =
              _worldToScreen(nextCoord['x'], nextCoord['y'], size);
          if (nextScreenPos != null) {
            _drawConnectionLine(canvas, screenPos, nextScreenPos);
          }
        }
      }
    }
  }

  Offset? _worldToScreen(double worldX, double worldY, Size size) {
    final adjustedWidth = size.width / mapScale;
    final adjustedHeight = size.height / mapScale;

    final resolution = mapData.info.resolution;
    final originX = mapData.info.origin.x;
    final originY = mapData.info.origin.y;
    final mapWidth = mapData.info.width;
    final mapHeight = mapData.info.height;

    final mapPhysicalWidth = mapWidth * resolution;
    final mapPhysicalHeight = mapHeight * resolution;
    final minX = originX;
    final minY = originY;

    final screenX = ((worldX - minX) / mapPhysicalWidth) * adjustedWidth;
    final screenY = ((worldY - minY) / mapPhysicalHeight) * adjustedHeight;

    return Offset(screenX, screenY);
  }

  void _drawCoordinate(Canvas canvas, Offset position, int number,
      bool isExecuting, bool isCompleted) {
    final radius = responsive.coordinateRadius;
    Color color = Colors.red;

    if (isExecuting) color = Colors.orange;
    if (isCompleted) color = Colors.green;

    // Shadow
    final shadowPaint = Paint()
      ..color = Colors.black38
      ..maskFilter = MaskFilter.blur(BlurStyle.normal, 3);
    canvas.drawCircle(position + Offset(1, 1), radius, shadowPaint);

    // Circle
    final circlePaint = Paint()..color = color;
    canvas.drawCircle(position, radius, circlePaint);

    // Border
    final borderPaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;
    canvas.drawCircle(position, radius, borderPaint);

    // Number
    final textPainter = TextPainter(
      text: TextSpan(
        text: number.toString(),
        style: TextStyle(
          color: Colors.white,
          fontWeight: FontWeight.bold,
          fontSize: responsive.fontSize,
          shadows: [
            Shadow(
              offset: Offset(0.5, 0.5),
              blurRadius: 1,
              color: Colors.black54,
            ),
          ],
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
  }

  void _drawConnectionLine(Canvas canvas, Offset from, Offset to) {
    final linePaint = Paint()
      ..color = Colors.red.withOpacity(0.7)
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    canvas.drawLine(from, to, linePaint);
    _drawArrow(canvas, from, to);
  }

  void _drawArrow(Canvas canvas, Offset from, Offset to) {
    final direction = to - from;
    final distance = direction.distance;
    if (distance == 0) return;

    final normalizedDirection = direction / distance;
    final arrowLength = 8.0;

    final arrowPoint1 = to -
        normalizedDirection * arrowLength +
        Offset(-normalizedDirection.dy * arrowLength * 0.4,
            normalizedDirection.dx * arrowLength * 0.4);
    final arrowPoint2 = to -
        normalizedDirection * arrowLength +
        Offset(normalizedDirection.dy * arrowLength * 0.4,
            -normalizedDirection.dx * arrowLength * 0.4);

    final arrowPaint = Paint()
      ..color = Colors.red.withOpacity(0.7)
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    canvas.drawLine(to, arrowPoint1, arrowPaint);
    canvas.drawLine(to, arrowPoint2, arrowPaint);
  }

  void _drawSavedOrderPreview(Canvas canvas, Size size) {
    if (savedOrder == null) return;

    final savedCoordinates = savedOrder!['coordinates'] as List<dynamic>;
    if (savedCoordinates.isEmpty) return;

    for (int i = 0; i < savedCoordinates.length; i++) {
      final coord = savedCoordinates[i] as Map<String, dynamic>;
      final screenPos = _worldToScreen(coord['x'], coord['y'], size);

      if (screenPos != null) {
        _drawSavedCoordinate(canvas, screenPos, i + 1);

        if (i < savedCoordinates.length - 1) {
          final nextCoord = savedCoordinates[i + 1] as Map<String, dynamic>;
          final nextScreenPos =
              _worldToScreen(nextCoord['x'], nextCoord['y'], size);
          if (nextScreenPos != null) {
            _drawSavedConnectionLine(canvas, screenPos, nextScreenPos);
          }
        }
      }
    }

    _drawSavedOrderLabel(canvas, size, savedOrder!['name'] as String);
  }

  void _drawSavedCoordinate(Canvas canvas, Offset position, int number) {
    final radius = responsive.coordinateRadius + 2;

    // Shadow
    final shadowPaint = Paint()
      ..color = Colors.black26
      ..maskFilter = MaskFilter.blur(BlurStyle.normal, 3);
    canvas.drawCircle(position + Offset(1, 1), radius, shadowPaint);

    // Circle
    final circlePaint = Paint()..color = Colors.green.withOpacity(0.7);
    canvas.drawCircle(position, radius, circlePaint);

    // Border
    final borderPaint = Paint()
      ..color = Colors.white.withOpacity(0.8)
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;
    canvas.drawCircle(position, radius, borderPaint);

    // Number
    final textPainter = TextPainter(
      text: TextSpan(
        text: number.toString(),
        style: TextStyle(
          color: Colors.white,
          fontWeight: FontWeight.bold,
          fontSize: responsive.fontSize + 1,
          shadows: [
            Shadow(
              offset: Offset(1, 1),
              blurRadius: 2,
              color: Colors.black45,
            ),
          ],
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
  }

  void _drawSavedConnectionLine(Canvas canvas, Offset from, Offset to) {
    final linePaint = Paint()
      ..color = Colors.green.withOpacity(0.5)
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    canvas.drawLine(from, to, linePaint);
    _drawSavedArrow(canvas, from, to);
  }

  void _drawSavedArrow(Canvas canvas, Offset from, Offset to) {
    final direction = to - from;
    final distance = direction.distance;
    if (distance == 0) return;

    final normalizedDirection = direction / distance;
    final arrowLength = 8.0;

    final arrowPoint1 = to -
        normalizedDirection * arrowLength +
        Offset(-normalizedDirection.dy * arrowLength * 0.5,
            normalizedDirection.dx * arrowLength * 0.5);
    final arrowPoint2 = to -
        normalizedDirection * arrowLength +
        Offset(normalizedDirection.dy * arrowLength * 0.5,
            -normalizedDirection.dx * arrowLength * 0.5);

    final arrowPaint = Paint()
      ..color = Colors.green.withOpacity(0.5)
      ..strokeWidth = 1.5
      ..style = PaintingStyle.stroke;

    canvas.drawLine(to, arrowPoint1, arrowPaint);
    canvas.drawLine(to, arrowPoint2, arrowPaint);
  }

  void _drawSavedOrderLabel(Canvas canvas, Size size, String orderName) {
    final labelText = '✅ Saved: $orderName';
    final textPainter = TextPainter(
      text: TextSpan(
        text: labelText,
        style: TextStyle(
          color: Colors.green.shade700,
          fontWeight: FontWeight.bold,
          fontSize: responsive.fontSize,
          backgroundColor: Colors.white.withOpacity(0.9),
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    textPainter.layout();

    final labelPosition = Offset(
      (size.width / mapScale) - textPainter.width - 16,
      16,
    );

    // Background
    final bgPaint = Paint()..color = Colors.white.withOpacity(0.9);
    final bgRect = Rect.fromLTWH(
      labelPosition.dx - 8,
      labelPosition.dy - 4,
      textPainter.width + 16,
      textPainter.height + 8,
    );
    canvas.drawRRect(
      RRect.fromRectAndRadius(bgRect, Radius.circular(8)),
      bgPaint,
    );

    // Border
    final borderPaint = Paint()
      ..color = Colors.green.shade300
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;
    canvas.drawRRect(
      RRect.fromRectAndRadius(bgRect, Radius.circular(8)),
      borderPaint,
    );

    textPainter.paint(canvas, labelPosition);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

// Device type and responsive value classes
enum DeviceType { mobile, tablet, laptop }

class ResponsiveValues {
  final int mapFlexRatio;
  final int controlFlexRatio;
  final bool isHorizontalLayout;
  final double cardPadding;
  final double buttonHeight;
  final double fontSize;
  final double titleFontSize;
  final double iconSize;
  final double coordinateRadius;
  final double bottomSheetHeight;

  ResponsiveValues({
    required this.mapFlexRatio,
    required this.controlFlexRatio,
    required this.isHorizontalLayout,
    required this.cardPadding,
    required this.buttonHeight,
    required this.fontSize,
    required this.titleFontSize,
    required this.iconSize,
    required this.coordinateRadius,
    required this.bottomSheetHeight,
  });
}
