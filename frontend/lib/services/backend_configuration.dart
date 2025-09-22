/// Backend configuration management for different deployment modes
enum BackendMode {
  live, // Production live backend
  local, // Local development backend
  custom, // Custom URL backend
}

class BackendConfiguration {
  static const String _liveBackendUrl =
      'https://fleetos-backend-frp4.onrender.com';
  static const String _localBackendUrl = 'http://localhost:3000';

  BackendMode _currentMode = BackendMode.live;
  String _customUrl = '';

  // Configuration properties
  Duration _connectionTimeout = const Duration(seconds: 30);
  Map<String, String> _defaultHeaders = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
  };

  // ==========================================
  // MODE MANAGEMENT
  // ==========================================

  /// Set live backend mode (production)
  void setLiveMode() {
    _currentMode = BackendMode.live;
    _customUrl = '';
    _connectionTimeout = const Duration(seconds: 30);
    _defaultHeaders = {
      'Content-Type': 'application/json',
      'Accept': 'application/json',
    };
  }

  /// Set local backend mode (development)
  void setLocalMode() {
    _currentMode = BackendMode.local;
    _customUrl = '';
    _connectionTimeout = const Duration(seconds: 10);
    _defaultHeaders = {
      'Content-Type': 'application/json',
      'Accept': 'application/json',
    };
  }

  /// Set custom URL mode
  void setCustomUrl(String url) {
    _currentMode = BackendMode.custom;
    _customUrl = url.replaceAll('/api', ''); // Remove /api suffix if present
    _connectionTimeout = const Duration(seconds: 15);
    _defaultHeaders = {
      'Content-Type': 'application/json',
      'Accept': 'application/json',
    };
  }

  // ==========================================
  // GETTERS
  // ==========================================

  BackendMode get currentMode => _currentMode;

  bool get isLiveMode => _currentMode == BackendMode.live;
  bool get isLocalMode => _currentMode == BackendMode.local;
  bool get isCustomMode => _currentMode == BackendMode.custom;

  String get httpBaseUrl {
    switch (_currentMode) {
      case BackendMode.live:
        return _liveBackendUrl;
      case BackendMode.local:
        return _localBackendUrl;
      case BackendMode.custom:
        return _customUrl;
    }
  }

  String get apiBaseUrl {
    switch (_currentMode) {
      case BackendMode.live:
        return '$_liveBackendUrl/api';
      case BackendMode.local:
        return '$_localBackendUrl/api';
      case BackendMode.custom:
        return '$_customUrl/api';
    }
  }

  String get webSocketUrl {
    switch (_currentMode) {
      case BackendMode.live:
        return _liveBackendUrl.replaceAll('https://', 'wss://');
      case BackendMode.local:
        return _localBackendUrl.replaceAll('http://', 'ws://');
      case BackendMode.custom:
        return _customUrl
            .replaceAll('https://', 'wss://')
            .replaceAll('http://', 'ws://');
    }
  }

  bool get isSecureConnection {
    switch (_currentMode) {
      case BackendMode.live:
        return true;
      case BackendMode.local:
        return false;
      case BackendMode.custom:
        return _customUrl.startsWith('https://');
    }
  }

  Duration get connectionTimeout => _connectionTimeout;
  Map<String, String> get defaultHeaders => Map.unmodifiable(_defaultHeaders);

  // ==========================================
  // UTILITY METHODS
  // ==========================================

  /// Print current configuration for debugging
  void printConfiguration() {
    print(' Backend Configuration:');
    print('   Mode: ${_currentMode.name.toUpperCase()}');
    print('   HTTP Base URL: $httpBaseUrl');
    print('   API Base URL: $apiBaseUrl');
    print('   WebSocket URL: $webSocketUrl');
    print('   Secure Connection: $isSecureConnection');
    print('   Timeout: ${_connectionTimeout.inSeconds}s');
    print('   Headers: $_defaultHeaders');
  }

  /// Get configuration summary
  Map<String, dynamic> getConfigurationSummary() {
    return {
      'mode': _currentMode.name,
      'httpBaseUrl': httpBaseUrl,
      'apiBaseUrl': apiBaseUrl,
      'webSocketUrl': webSocketUrl,
      'isSecureConnection': isSecureConnection,
      'connectionTimeout': _connectionTimeout.inSeconds,
      'defaultHeaders': _defaultHeaders,
    };
  }

  /// Reset to default configuration
  void reset() {
    setLiveMode();
  }

  /// Validate current configuration
  bool isValidConfiguration() {
    try {
      final uri = Uri.parse(httpBaseUrl);
      return uri.hasScheme && uri.hasAuthority;
    } catch (e) {
      return false;
    }
  }
}
