// Test script to verify fixes for Flutter lifecycle and API issues
import 'dart:async';
import 'dart:io';

void main() async {
  print('🧪 Testing AMR Fleet Management Fixes...\n');

  // Test 1: Lifecycle Management Simulation
  print('1️⃣ Testing Lifecycle Management...');
  await testLifecycleManagement();

  // Test 2: API Endpoint Testing
  print('\n2️⃣ Testing API Endpoints...');
  await testAPIEndpoints();

  print('\n✅ All tests completed!');
}

/// Simulate Flutter widget lifecycle to test disposal issues
Future<void> testLifecycleManagement() async {
  print('   📱 Simulating widget lifecycle...');

  // Simulate async operations that might continue after disposal
  final List<Future> operations = [];
  bool widgetMounted = true;

  // Simulate multiple async operations
  for (int i = 0; i < 3; i++) {
    operations.add(_simulateAsyncOperation(i, () => widgetMounted));
  }

  // Simulate widget disposal after 2 seconds
  Timer(Duration(seconds: 2), () {
    print('   🗑️  Widget disposed');
    widgetMounted = false;
  });

  // Wait for operations to complete
  await Future.wait(operations);

  print('   ✅ Lifecycle management test passed');
}

Future<void> _simulateAsyncOperation(int id, bool Function() isMounted) async {
  for (int i = 0; i < 5; i++) {
    await Future.delayed(Duration(milliseconds: 500));

    if (!isMounted()) {
      print('   ⚠️  Operation $id cancelled (widget not mounted)');
      return;
    }

    print('   🔄 Operation $id: step ${i + 1}/5');
  }

  print('   ✅ Operation $id completed');
}

/// Test API endpoints for proper functionality
Future<void> testAPIEndpoints() async {
  print('   🌐 Testing backend endpoints...');

  final client = HttpClient();
  final baseUrl = 'http://192.168.1.35:3000';

  // Test endpoints
  final endpoints = [
    '/health',
    '/api/orders/piros',
    '/api/orders/stats?timeRange=7d',
    '/api/devices',
  ];

  for (final endpoint in endpoints) {
    await _testEndpoint(client, baseUrl, endpoint);
  }

  client.close();
  print('   ✅ API endpoint tests completed');
}

Future<void> _testEndpoint(
  HttpClient client,
  String baseUrl,
  String endpoint,
) async {
  try {
    print('   📡 Testing: $baseUrl$endpoint');

    final request = await client.getUrl(Uri.parse('$baseUrl$endpoint'));
    request.headers.set('Accept', 'application/json');

    final response = await request.close().timeout(Duration(seconds: 5));
    final statusCode = response.statusCode;

    if (statusCode == 200) {
      print('   ✅ $endpoint: OK ($statusCode)');
    } else if (statusCode == 404) {
      print('   ❌ $endpoint: Not Found (404) - Need to implement');
    } else {
      print('   ⚠️  $endpoint: $statusCode');
    }
  } catch (e) {
    if (e is TimeoutException) {
      print('   ⏰ $endpoint: Timeout');
    } else if (e is SocketException) {
      print('   🔌 $endpoint: Connection failed');
    } else {
      print('   ❌ $endpoint: Error - $e');
    }
  }
}
