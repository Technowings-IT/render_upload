#!/bin/bash

# AGV Fleet Management - Connection Test Script

echo "🔧 AGV Fleet Management Connection Test"
echo "========================================"

# Get current machine IP
CURRENT_IP=$(ip addr show | grep "inet " | grep -v "127.0.0.1" | awk '{print $2}' | cut -d'/' -f1 | head -1)
echo "📍 Current machine IP: $CURRENT_IP"

# Test backend health
echo ""
echo "🏥 Testing backend health..."
HEALTH_URL="http://$CURRENT_IP:3000/health"
echo "   URL: $HEALTH_URL"

if curl -s --connect-timeout 5 "$HEALTH_URL" > /dev/null; then
    echo "   ✅ Backend is healthy"
    curl -s "$HEALTH_URL" | jq '.' 2>/dev/null || curl -s "$HEALTH_URL"
else
    echo "   ❌ Backend health check failed"
fi

# Test API endpoints
echo ""
echo "🔌 Testing API endpoints..."
API_ENDPOINTS=(
    "api/devices"
    "api/orders/piros"
    "api/orders/stats?timeRange=7d"
    "api/analytics?type=battery&timeRange=1h&deviceId=piros"
)

for endpoint in "${API_ENDPOINTS[@]}"; do
    URL="http://$CURRENT_IP:3000/$endpoint"
    echo "   Testing: $endpoint"
    
    if curl -s --connect-timeout 5 "$URL" > /dev/null; then
        echo "     ✅ $endpoint - OK"
    else
        echo "     ❌ $endpoint - FAILED"
    fi
done

# Check if port 3000 is open
echo ""
echo "🔍 Checking port 3000..."
if netstat -tlnp 2>/dev/null | grep :3000 > /dev/null; then
    echo "   ✅ Port 3000 is open and listening"
    netstat -tlnp 2>/dev/null | grep :3000
else
    echo "   ❌ Port 3000 is not open"
fi

# Check firewall status
echo ""
echo "🛡️ Checking firewall status..."
if command -v ufw >/dev/null 2>&1; then
    UFW_STATUS=$(ufw status 2>/dev/null | head -1)
    echo "   UFW: $UFW_STATUS"
    
    if echo "$UFW_STATUS" | grep -q "active"; then
        echo "   ⚠️  UFW is active - check if port 3000 is allowed"
        ufw status | grep 3000 || echo "   Port 3000 not explicitly allowed"
    fi
elif command -v firewall-cmd >/dev/null 2>&1; then
    FIREWALL_STATUS=$(firewall-cmd --state 2>/dev/null || echo "inactive")
    echo "   Firewalld: $FIREWALL_STATUS"
else
    echo "   No common firewall tools detected"
fi

# Network connectivity test
echo ""
echo "🌐 Network connectivity test..."
if ping -c 1 -W 1 "$CURRENT_IP" > /dev/null 2>&1; then
    echo "   ✅ Can ping own IP ($CURRENT_IP)"
else
    echo "   ❌ Cannot ping own IP ($CURRENT_IP)"
fi

# Summary
echo ""
echo "📊 Summary for Flutter App Configuration:"
echo "   Backend URL: http://$CURRENT_IP:3000"
echo "   WebSocket URL: ws://$CURRENT_IP:3000"
echo "   Health endpoint: http://$CURRENT_IP:3000/health"
echo ""
echo "🔧 Next steps if connection still fails:"
echo "   1. Clear Flutter app data/cache"
echo "   2. Restart the Flutter app completely"
echo "   3. Check if Flutter app has cached wrong IP addresses"
echo "   4. Make sure no VPN or proxy is interfering"
