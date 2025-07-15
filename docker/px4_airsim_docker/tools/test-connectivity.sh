#!/bin/bash

echo "Testing AirSim connectivity for ultra-swarm..."
echo "============================================="

# Test if ports are open
echo -e "\n1. Checking if AirSim ports are listening:"
for port in {4561..4569}; do
    if nc -zv localhost $port 2>&1 | grep -q "succeeded"; then
        echo "✓ Port $port is OPEN"
    else
        echo "✗ Port $port is CLOSED"
    fi
done

# Check AirSim API
echo -e "\n2. Testing AirSim API connectivity:"
if nc -zv localhost 41451 2>&1 | grep -q "succeeded"; then
    echo "✓ AirSim API port 41451 is OPEN"
else
    echo "✗ AirSim API port 41451 is CLOSED"
fi

# Check processes
echo -e "\n3. Looking for AirSim/Unreal processes:"
if pgrep -f "Blocks|AirSim|UnrealEngine" > /dev/null; then
    echo "✓ Found AirSim/Unreal process"
    pgrep -f "Blocks|AirSim|UnrealEngine" -a
else
    echo "✗ No AirSim/Unreal process found"
fi

echo -e "\n4. Docker container status:"
docker ps --format "table {{.Names}}\t{{.Status}}" | grep px4-swarm

echo -e "\nTroubleshooting steps:"
echo "======================"
echo "1. Make sure AirSim is running:"
echo "   cd /mnt/l/cosys-airsim/Unreal/Environments/Blocks"
echo "   ./Blocks.sh"
echo ""
echo "2. Verify settings.json is correct:"
echo "   cat ~/Documents/AirSim/settings.json | grep -E 'TcpPort|VehicleType'"
echo ""
echo "3. Check for firewall issues:"
echo "   sudo ufw status"
echo ""
echo "4. Test with a single drone first:"
echo "   docker-compose -f docker-compose.ultra-swarm.yml down"
echo "   docker-compose -f docker-compose.ultra-swarm.yml up px4-swarm-1-drone-1"