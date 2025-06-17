#!/bin/bash

echo "Testing PX4 startup command..."

cd /px4_ws/src/PX4-Autopilot

# Test PX4 help first
echo "=== PX4 Help ==="
./build/px4_sitl_default/bin/px4 -h

echo ""
echo "=== Testing PX4 startup ==="

# Create minimal startup script
cat > /tmp/test_rcS << 'EOF'
#!/bin/sh
echo "PX4 startup script running..."
uorb start
param load
simulator start -c 4560
sleep 5
EOF

chmod +x /tmp/test_rcS

# Test the command
echo "Running: ./build/px4_sitl_default/bin/px4 -d -s /tmp/test_rcS -w /px4_ws/src/PX4-Autopilot /px4_ws/src/PX4-Autopilot"
./build/px4_sitl_default/bin/px4 -d -s /tmp/test_rcS -w /px4_ws/src/PX4-Autopilot /px4_ws/src/PX4-Autopilot &

PID=$!
echo "PX4 started with PID: $PID"

sleep 5

if kill -0 $PID 2>/dev/null; then
    echo "SUCCESS: PX4 is running"
    kill $PID
else
    echo "ERROR: PX4 process died"
fi 