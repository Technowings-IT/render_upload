#!/bin/bash

# AGV Fleet Management - Connection Fix Script

echo "🔧 AGV Fleet Management Connection Fix"
echo "====================================="

cd "/home/vidit-agrawal/ agv-fleet-management "

# Get current IP
CURRENT_IP=$(ip addr show | grep "inet " | grep -v "127.0.0.1" | awk '{print $2}' | cut -d'/' -f1 | head -1)
echo "📍 Current machine IP: $CURRENT_IP"

# Test backend
echo "🏥 Testing backend..."
if curl -s --connect-timeout 5 "http://$CURRENT_IP:3000/health" > /dev/null; then
    echo "✅ Backend is running correctly at http://$CURRENT_IP:3000"
else
    echo "❌ Backend is not responding. Please start the backend first:"
    echo "   cd backend && npm start"
    exit 1
fi

# Clean Flutter cache
echo ""
echo "🧹 Cleaning Flutter cache..."
cd frontend
flutter clean

# Get dependencies
echo "📦 Getting Flutter dependencies..."
flutter pub get

# Check if the frontend IP configuration is correct
echo ""
echo "🔍 Checking Flutter configuration..."
CONFIGURED_IP=$(grep -r "DEFAULT_SERVER_IP" lib/ | grep -o "192\.168\.0\.[0-9]\+" | head -1)
echo "   Configured IP in Flutter: $CONFIGURED_IP"
echo "   Current machine IP: $CURRENT_IP"

if [ "$CONFIGURED_IP" = "$CURRENT_IP" ]; then
    echo "✅ Flutter configuration matches current IP"
else
    echo "⚠️  Flutter configuration does not match current IP"
    echo "   Updating configuration..."
    
    # Update main.dart with correct IP
    sed -i "s/DEFAULT_SERVER_IP = '192\.168\.0\.[0-9]\+'/DEFAULT_SERVER_IP = '$CURRENT_IP'/g" lib/main.dart
    
    # Update other files
    sed -i "s/192\.168\.0\.[0-9]\+:3000/$CURRENT_IP:3000/g" lib/screens/connect_screen.dart
    sed -i "s/192\.168\.0\.[0-9]\+:3000/$CURRENT_IP:3000/g" lib/screens/settings_screen.dart
    sed -i "s/192\.168\.0\.[0-9]\+:3000/$CURRENT_IP:3000/g" lib/screens/control_page.dart
    
    echo "✅ Configuration updated"
fi

# Kill any running Flutter processes
echo ""
echo "🔄 Stopping any running Flutter processes..."
pkill -f "flutter" 2>/dev/null || true
pkill -f "dart" 2>/dev/null || true

echo ""
echo "✅ Fix completed! Ready to restart Flutter app."
echo ""
echo "🚀 To start the Flutter app:"
echo "   cd /home/vidit-agrawal/ agv-fleet-management /frontend"
echo "   flutter run -d linux"
echo ""
echo "📱 Or for web:"
echo "   flutter run -d chrome --web-hostname 0.0.0.0 --web-port 8080"
echo ""
echo "🔗 Backend URL: http://$CURRENT_IP:3000"
echo "🔌 WebSocket URL: ws://$CURRENT_IP:3000"
