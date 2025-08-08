import 'dart:async';
import 'dart:io';
import 'dart:convert';
import 'dart:math';
import 'package:http/http.dart' as http;
import 'package:multicast_dns/multicast_dns.dart';

class AMRDevice {
  final String id;
  final String name;
  final String ipAddress;
  final int port;
  final String discoveryMethod;
  final List<String> services;
  final Map<String, dynamic>? metadata;

  AMRDevice({
    required this.id,
    required this.name,
    required this.ipAddress,
    required this.port,
    required this.discoveryMethod,
    required this.services,
    this.metadata,
  });

  @override
  String toString() =>
      'AMRDevice($name @ $ipAddress:$port via $discoveryMethod)';
}

class NetworkDiscoveryService {
  static final NetworkDiscoveryService _instance =
      NetworkDiscoveryService._internal();
  factory NetworkDiscoveryService() => _instance;
  NetworkDiscoveryService._internal();

  final StreamController<List<AMRDevice>> _discoveredDevicesController =
      StreamController<List<AMRDevice>>.broadcast();

  Stream<List<AMRDevice>> get discoveredDevices =>
      _discoveredDevicesController.stream;

  List<AMRDevice> _devices = [];
  bool _isDiscovering = false;

  // AMR-specific ports to scan
  static const List<int> AMR_PORTS = [3000, 8080, 80, 22, 11311, 9090];

  // AMR service identifiers
  static const List<String> AMR_SERVICES = [
    '_http._tcp',
    '_ros._tcp',
    '_AMR._tcp',
    '_websocket._tcp',
    '_ssh._tcp'
  ];

  Future<List<AMRDevice>> discoverDevices({
    Duration timeout = const Duration(seconds: 30),
    List<int>? customPorts,
    bool useMDNS = true,
    bool useNetworkScan = true,
    bool useBroadcast = true,
    bool useKnownIPs = true,
  }) async {
    if (_isDiscovering) {
      print('🔄 Discovery already in progress...');
      return _devices;
    }

    _isDiscovering = true;
    _devices.clear();

    print('🔍 Starting AMR device discovery...');

    try {
      // Get network info
      final networkInfo = await _getNetworkInfo();
      if (networkInfo == null) {
        print('❌ Could not determine network information');
        return [];
      }

      print('📡 Scanning subnet: ${networkInfo['subnet']}');

      final List<Future<List<AMRDevice>>> discoveryMethods = [];

      // 1. Check known AMR IPs first (fastest)
      if (useKnownIPs) {
        discoveryMethods.add(_discoverKnownIPs(customPorts ?? AMR_PORTS));
      }

      // 2. mDNS discovery
      if (useMDNS) {
        discoveryMethods.add(_discoverViaMDNS(timeout));
      }

      // 3. Network scan
      if (useNetworkScan) {
        discoveryMethods.add(_discoverViaNetworkScan(
            networkInfo['subnet']!, customPorts ?? AMR_PORTS, timeout));
      }

      // 4. UDP broadcast
      if (useBroadcast) {
        discoveryMethods.add(_discoverViaBroadcast(timeout));
      }

      // Run all discovery methods in parallel
      final results = await Future.wait(discoveryMethods, eagerError: false);

      // Combine results and remove duplicates
      final allDevices = <AMRDevice>[];
      for (final deviceList in results) {
        allDevices.addAll(deviceList);
      }

      _devices = _removeDuplicateDevices(allDevices);

      print('✅ Discovery complete. Found ${_devices.length} devices');
      _discoveredDevicesController.add(_devices);

      return _devices;
    } catch (e) {
      print('❌ Discovery error: $e');
      return [];
    } finally {
      _isDiscovering = false;
    }
  }

  // Method 1: Check known AMR IPs
  Future<List<AMRDevice>> _discoverKnownIPs(List<int> ports) async {
    print('🎯 Checking known AMR IPs...');

    // Add your known AMR IPs here
    final knownIPs = [
      '192.168.208.79', // Current backend IP (confirmed working)
      '192.168.208.29', // Your current AMR
      '192.168.0.100', // Common AMR IP
      '10.0.0.100', // Common AMR IP
    ];

    final devices = <AMRDevice>[];

    for (final ip in knownIPs) {
      for (final port in ports) {
        try {
          if (await _isPortOpen(ip, port, timeout: Duration(seconds: 2))) {
            final device = await _identifyAMRDevice(ip, port, 'known_ip');
            if (device != null) {
              devices.add(device);
              print('✅ Found known AMR: ${device.name} at $ip:$port');
            }
          }
        } catch (e) {
          // Skip failed connections
        }
      }
    }

    return devices;
  }

  // Method 2: mDNS Discovery (Fixed)
  Future<List<AMRDevice>> _discoverViaMDNS(Duration timeout) async {
    print('🔍 Starting mDNS discovery...');

    final devices = <AMRDevice>[];

    try {
      final MDnsClient client = MDnsClient();
      await client.start();

      final List<Future<void>> serviceLookups = [];

      for (final service in AMR_SERVICES) {
        serviceLookups.add(_lookupMDNSService(client, service, devices));
      }

      // Wait for all service lookups with timeout
      await Future.wait(serviceLookups).timeout(
        timeout,
        onTimeout: () {
          print('⏰ mDNS discovery timeout after ${timeout.inSeconds}s');
          return <void>[]; // Return an empty list to satisfy the return type
        },
      );

      client.stop();
      print('✅ mDNS discovery completed. Found ${devices.length} devices');
    } catch (e) {
      print('❌ mDNS discovery failed: $e');
    }

    return devices;
  }

  Future<void> _lookupMDNSService(
      MDnsClient client, String service, List<AMRDevice> devices) async {
    try {
      await for (final PtrResourceRecord ptr
          in client.lookup<PtrResourceRecord>(
        ResourceRecordQuery.serverPointer(service),
      )) {
        final serviceName = ptr.domainName;

        // Get SRV record for port and target
        await for (final SrvResourceRecord srv
            in client.lookup<SrvResourceRecord>(
          ResourceRecordQuery.service(serviceName),
        )) {
          // Get A record for IP address
          await for (final IPAddressResourceRecord a
              in client.lookup<IPAddressResourceRecord>(
            ResourceRecordQuery.addressIPv4(srv.target),
          )) {
            try {
              final device =
                  await _identifyAMRDevice(a.address.address, srv.port, 'mdns');
              if (device != null &&
                  !devices.any((d) => d.ipAddress == device.ipAddress)) {
                devices.add(device);
                print('✅ Found mDNS AMR: ${device.name}');
              }
            } catch (e) {
              // Skip invalid devices
            }
          }
        }
      }
    } catch (e) {
      print('❌ Error looking up mDNS service $service: $e');
    }
  }

  // Method 3: Enhanced Network Scan
  Future<List<AMRDevice>> _discoverViaNetworkScan(
      String subnet, List<int> ports, Duration timeout) async {
    print('🔍 Starting network scan on $subnet.x...');

    final devices = <AMRDevice>[];
    final baseIP = subnet.split('.').take(3).join('.');

    // Scan common AMR IP ranges
    final scanRanges = [
      {'start': 70, 'end': 90}, // Common AMR range
      {'start': 100, 'end': 120}, // Common device range
      {'start': 200, 'end': 210}, // High range
    ];

    final scanTasks = <Future<void>>[];

    for (final range in scanRanges) {
      for (int i = range['start']!; i <= range['end']!; i++) {
        final ip = '$baseIP.$i';

        scanTasks.add(_scanIPForAMR(ip, ports, devices));

        // Batch processing to avoid overwhelming the network
        if (scanTasks.length >= 10) {
          await Future.wait(scanTasks, eagerError: false);
          scanTasks.clear();

          // Small delay between batches
          await Future.delayed(Duration(milliseconds: 100));
        }
      }
    }

    // Process remaining tasks
    if (scanTasks.isNotEmpty) {
      await Future.wait(scanTasks, eagerError: false);
    }

    print('✅ Network scan completed. Found ${devices.length} devices');
    return devices;
  }

  Future<void> _scanIPForAMR(
      String ip, List<int> ports, List<AMRDevice> devices) async {
    for (final port in ports) {
      try {
        if (await _isPortOpen(ip, port, timeout: Duration(seconds: 1))) {
          final device = await _identifyAMRDevice(ip, port, 'network_scan');
          if (device != null &&
              !devices.any((d) => d.ipAddress == device.ipAddress)) {
            devices.add(device);
            print('✅ Found network AMR: ${device.name} at $ip:$port');
          }
        }
      } catch (e) {
        // Skip failed connections
      }
    }
  }

  // Method 4: UDP Broadcast Discovery
  Future<List<AMRDevice>> _discoverViaBroadcast(Duration timeout) async {
    print('🔍 Starting UDP broadcast discovery...');

    final devices = <AMRDevice>[];

    try {
      final socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 0);
      socket.broadcastEnabled = true;

      // Send AMR discovery broadcast
      final message = utf8.encode(json.encode({
        'type': 'AMR_discovery',
        'timestamp': DateTime.now().toIso8601String(),
        'sender': 'flutter_app',
      }));

      // Broadcast on common AMR ports
      final broadcastPorts = [8888, 9999, 12345];
      for (final port in broadcastPorts) {
        socket.send(message, InternetAddress('255.255.255.255'), port);
      }

      // Listen for responses
      final responseCompleter = Completer<void>();
      late StreamSubscription subscription;

      subscription = socket.listen((RawSocketEvent event) {
        if (event == RawSocketEvent.read) {
          final datagram = socket.receive();
          if (datagram != null) {
            try {
              final response = utf8.decode(datagram.data);
              final data = json.decode(response);

              if (data['type'] == 'AMR_response') {
                final device = AMRDevice(
                  id: data['id'] ?? 'unknown',
                  name: data['name'] ?? 'AMR Device',
                  ipAddress: datagram.address.address,
                  port: data['port'] ?? 3000,
                  discoveryMethod: 'udp_broadcast',
                  services:
                      List<String>.from(data['services'] ?? ['broadcast']),
                  metadata: data,
                );

                if (!devices.any((d) => d.ipAddress == device.ipAddress)) {
                  devices.add(device);
                  print('✅ Found broadcast AMR: ${device.name}');
                }
              }
            } catch (e) {
              // Skip invalid responses
            }
          }
        }
      });

      // Wait for timeout
      Timer(timeout, () {
        if (!responseCompleter.isCompleted) {
          responseCompleter.complete();
        }
      });

      await responseCompleter.future;
      subscription.cancel();
      socket.close();
    } catch (e) {
      print('❌ UDP broadcast discovery failed: $e');
    }

    print('✅ Broadcast discovery completed. Found ${devices.length} devices');
    return devices;
  }

  // AMR Device Identification
  Future<AMRDevice?> _identifyAMRDevice(
      String ip, int port, String discoveryMethod) async {
    try {
      // Try to get AMR info via HTTP
      final response = await http.get(
        Uri.parse('http://$ip:$port/api/health'),
        headers: {'Accept': 'application/json'},
      ).timeout(Duration(seconds: 3));

      if (response.statusCode == 200) {
        try {
          final data = json.decode(response.body);

          // Check if this looks like an AMR API
          if (data['success'] == true ||
              data.toString().toLowerCase().contains('AMR') ||
              data.toString().toLowerCase().contains('fleet') ||
              data.toString().toLowerCase().contains('robot')) {
            return AMRDevice(
              id: data['data']?['deviceId'] ?? 'AMR_${ip.replaceAll('.', '_')}',
              name: data['data']?['name'] ?? 'AMR at $ip',
              ipAddress: ip,
              port: port,
              discoveryMethod: discoveryMethod,
              services: ['http', 'AMR_api'],
              metadata: {
                'health_endpoint': true,
                'api_version': data['data']?['version'],
                'ros2_status': data['data']?['ros2Status'],
                'connected_devices': data['data']?['connectedDevices'],
              },
            );
          }
        } catch (parseError) {
          // If JSON parse fails, still consider it if status is 200
          return AMRDevice(
            id: 'AMR_${ip.replaceAll('.', '_')}',
            name: 'Device at $ip',
            ipAddress: ip,
            port: port,
            discoveryMethod: discoveryMethod,
            services: ['http'],
            metadata: {'response_code': response.statusCode},
          );
        }
      }

      // Check for other AMR indicators
      if (response.statusCode == 404 || response.statusCode == 401) {
        // Might be an AMR with different endpoint structure
        return AMRDevice(
          id: 'device_${ip.replaceAll('.', '_')}',
          name: 'Possible AMR at $ip',
          ipAddress: ip,
          port: port,
          discoveryMethod: discoveryMethod,
          services: ['http'],
          metadata: {'response_code': response.statusCode},
        );
      }
    } catch (e) {
      // Check if port is open but no HTTP service
      if (await _isPortOpen(ip, port, timeout: Duration(seconds: 1))) {
        return AMRDevice(
          id: 'device_${ip.replaceAll('.', '_')}',
          name: 'Device at $ip',
          ipAddress: ip,
          port: port,
          discoveryMethod: discoveryMethod,
          services: ['tcp'],
          metadata: {'port_open': true},
        );
      }
    }

    return null;
  }

  // Network Utilities
  Future<Map<String, String>?> _getNetworkInfo() async {
    try {
      for (final interface in await NetworkInterface.list()) {
        for (final addr in interface.addresses) {
          if (addr.type == InternetAddressType.IPv4 && !addr.isLoopback) {
            final ip = addr.address;
            final subnet = ip.split('.').take(3).join('.');
            return {
              'interface': interface.name,
              'ip': ip,
              'subnet': subnet,
            };
          }
        }
      }
    } catch (e) {
      print('❌ Error getting network info: $e');
    }
    return null;
  }

  Future<bool> _isPortOpen(String ip, int port, {Duration? timeout}) async {
    try {
      final socket = await Socket.connect(
        ip,
        port,
        timeout: timeout ?? Duration(seconds: 2),
      );
      socket.destroy();
      return true;
    } catch (e) {
      return false;
    }
  }

  List<AMRDevice> _removeDuplicateDevices(List<AMRDevice> devices) {
    final seen = <String>{};
    final unique = <AMRDevice>[];

    for (final device in devices) {
      final key = '${device.ipAddress}:${device.port}';
      if (!seen.contains(key)) {
        seen.add(key);
        unique.add(device);
      }
    }

    return unique;
  }

  void dispose() {
    _discoveredDevicesController.close();
  }
}
