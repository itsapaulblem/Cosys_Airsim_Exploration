#!/bin/bash

# Quick Start Guide for WSL2 + Windows AirSim Setup
# This script guides you through the complete setup process

echo "🚁 AirSim Ultra-Swarm Quick Start for WSL2"
echo "=========================================="
echo
echo "This guide will help you set up AirSim on Windows with PX4 in WSL2 Docker."
echo

# Step 1: Environment Detection
echo "Step 1: Environment Detection"
echo "-----------------------------"

BASE_DIR="$(dirname "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)")"
WSL2_DETECTOR="$BASE_DIR/../docker_clean/config_generator/tools/wsl2_detector.py"
if [[ -f "$WSL2_DETECTOR" ]]; then
    python3 "$WSL2_DETECTOR" --status
else
    echo "⚠️  WSL2 detector not found. Manual setup required."
fi

echo
read -p "Press Enter to continue..."

# Step 2: Check Windows AirSim Status
echo
echo "Step 2: Windows AirSim Setup"
echo "----------------------------"
echo "Please ensure the following on your Windows machine:"
echo
echo "1. Copy ultra-swarm settings to AirSim:"
echo "   Copy: docker_clean/ultra_swarm/settings.json"
echo "   To:   C:\\Users\\[YourName]\\Documents\\AirSim\\settings.json"
echo
echo "2. Start AirSim on Windows (e.g., run Blocks.exe)"
echo
echo "3. Configure Windows Firewall (PowerShell as Administrator):"
echo "   cd L:\\Cosys-AirSim\\docker\\px4_airsim_docker_v2"
echo "   .\\windows-firewall-setup.ps1"
echo

read -p "Have you completed these steps? (y/n): " windows_ready

if [[ "$windows_ready" != "y" && "$windows_ready" != "Y" ]]; then
    echo
    echo "❌ Please complete the Windows setup steps before continuing."
    echo "   Run this script again when ready."
    exit 1
fi

# Step 3: Test Connectivity
echo
echo "Step 3: Testing Connectivity"
echo "----------------------------"

WINDOWS_IP=$(ip route | grep default | awk '{print $3}')
echo "Detected Windows host IP: $WINDOWS_IP"

echo -n "Testing AirSim API connection... "
if timeout 3 nc -zv $WINDOWS_IP 41451 2>&1 | grep -q "succeeded"; then
    echo "✅ SUCCESS!"
else
    echo "❌ FAILED!"
    echo
    echo "Troubleshooting tips:"
    echo "- Verify AirSim is running on Windows"
    echo "- Check Windows firewall configuration"
    echo "- Ensure settings.json has: \"ApiServerEndpoint\": \"0.0.0.0:41451\""
    echo
    read -p "Continue anyway? (y/n): " continue_anyway
    if [[ "$continue_anyway" != "y" && "$continue_anyway" != "Y" ]]; then
        exit 1
    fi
fi

# Step 4: Configuration Update
echo
echo "Step 4: Updating Configuration"
echo "------------------------------"

if [[ -f "$WSL2_DETECTOR" ]]; then
    echo "Auto-updating configuration files..."
    python3 "$WSL2_DETECTOR" --auto-update "$BASE_DIR"
else
    echo "Manual configuration update..."
    sed -i "s/PX4_SIM_HOSTNAME: .*/PX4_SIM_HOSTNAME: $WINDOWS_IP/" "$BASE_DIR/docker-compose.ultra-swarm.yml"
fi

echo "✅ Configuration updated"

# Step 5: Launch Options
echo
echo "Step 5: Launch Options"
echo "---------------------"
echo "Choose how many drones to start:"
echo
echo "1) Single drone (testing)"
echo "2) 3 drones (basic swarm)"
echo "3) 9 drones (full swarm)"
echo "4) Manual command"
echo

read -p "Enter your choice (1-4): " choice

case $choice in
    1)
        echo
        echo "🚀 Starting single drone..."
        "$(dirname "$0")/start-ultra-swarm-wsl.sh" single
        ;;
    2)
        echo
        echo "🚀 Starting 3 drones..."
        "$(dirname "$0")/start-ultra-swarm-wsl.sh" test-3
        ;;
    3)
        echo
        echo "🚀 Starting full swarm (9 drones)..."
        "$(dirname "$0")/start-ultra-swarm-wsl.sh" swarm1-full
        ;;
    4)
        echo
        echo "Available commands:"
        echo "  ./tools/start-ultra-swarm-wsl.sh single      # 1 drone"
        echo "  ./tools/start-ultra-swarm-wsl.sh test-3      # 3 drones"
        echo "  ./tools/start-ultra-swarm-wsl.sh swarm1-full # 9 drones"
        echo "  ./tools/start-ultra-swarm-wsl.sh status      # Check status"
        echo "  ./tools/start-ultra-swarm-wsl.sh logs [name] # View logs"
        echo "  ./tools/start-ultra-swarm-wsl.sh stop        # Stop all"
        ;;
    *)
        echo "Invalid choice. Please run the script again."
        ;;
esac

echo
echo "📚 Useful Commands:"
echo "  Monitor logs:    ./tools/start-ultra-swarm-wsl.sh logs px4-swarm-1-drone-1"
echo "  Check status:    ./tools/start-ultra-swarm-wsl.sh status"
echo "  Stop all:        ./tools/start-ultra-swarm-wsl.sh stop"
echo
echo "🎯 Next Steps:"
echo "  - Use Python/MATLAB clients to connect to ports 4561-4569"
echo "  - Check drone connections in AirSim"
echo "  - Monitor container logs for any issues"
echo
echo "Happy flying! 🚁"