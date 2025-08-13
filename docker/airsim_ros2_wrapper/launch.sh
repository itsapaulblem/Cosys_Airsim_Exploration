#!/bin/bash

# AirSim ROS2 VNC Container Launcher
# Simplified launcher for the VNC-enabled ROS2 environment

set -e

echo "🚁 AirSim ROS2 VNC Container Launcher"
echo "====================================="

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop."
    exit 1
fi

# Navigate to script directory
cd "$(dirname "$0")"

echo "📁 Working directory: $(pwd)"

# Check if docker-compose.yml exists
if [ ! -f "docker-compose.yml" ]; then
    echo "❌ docker-compose.yml not found!"
    exit 1
fi

echo "🔧 Starting AirSim ROS2 VNC container..."

# Start the container
docker-compose up -d

# Check if container started successfully
if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Container started successfully!"
    echo ""
    echo "🖥️  VNC Access:"
    echo "   URL: localhost:5901"
    echo "   Password: airsim"
    echo ""
    echo "📊 Container Status:"
    docker-compose ps
    echo ""
    echo "💡 Useful Commands:"
    echo "   View logs:     docker-compose logs -f"
    echo "   Stop:          docker-compose stop"
    echo "   Restart:       docker-compose restart"
    echo "   Enter shell:   docker exec -it airsim-ros2-vnc bash"
    echo ""
    echo "🚁 Ready for ROS2 development!"
else
    echo "❌ Failed to start container"
    echo "📋 Check logs with: docker-compose logs"
    exit 1
fi