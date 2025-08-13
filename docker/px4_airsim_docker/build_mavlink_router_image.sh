#!/bin/bash

# Build script for PX4 AirSim Docker image with MAVLink Router
# This creates the enhanced image supporting external network access

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🐳 Building PX4 AirSim Docker image with MAVLink Router..."
echo "📁 Build context: $SCRIPT_DIR"

# Check if Dockerfile exists
if [ ! -f "Dockerfile.mavlink-router-slim" ]; then
    echo "❌ Dockerfile.mavlink-router-slim not found in $SCRIPT_DIR"
    exit 1
fi

# Check if base image exists
if ! docker images px4-airsim:slim | grep -q slim; then
    echo "⚠️  Base image px4-airsim:slim not found"
    echo "🔧 Please build the slim base image first:"
    echo "   docker build -f Dockerfile.slim -t px4-airsim:slim ."
    exit 1
fi

# Check if required scripts exist
required_scripts=(
    "scripts/mavlink-router-template.conf"
    "scripts/run_airsim_sitl_mavlink_router.sh"
    "scripts/generate_mavlink_config.py"
)

for script in "${required_scripts[@]}"; do
    if [ ! -f "$script" ]; then
        echo "❌ Required script not found: $script"
        exit 1
    fi
done

echo "✅ All required files found"

# Build the image
echo "🔨 Building Docker image..."
echo "   Image: px4-airsim:mavlink-router"
echo "   Context: ."
echo "   Dockerfile: Dockerfile.mavlink-router-slim"
echo "   Base Image: px4-airsim:slim"

docker build \
    -f Dockerfile.mavlink-router-slim \
    -t px4-airsim:mavlink-router \
    . || {
    echo "❌ Docker build failed"
    exit 1
}

echo "✅ Docker image built successfully: px4-airsim:mavlink-router"

# Verify the image
echo "🔍 Verifying image..."
docker images px4-airsim:mavlink-router

# Test MAVLink Router installation
echo "🧪 Testing MAVLink Router installation..."
docker run --rm px4-airsim:mavlink-router mavlink-routerd --help > /dev/null && {
    echo "✅ MAVLink Router verified in image"
} || {
    echo "⚠️ MAVLink Router test failed"
}

# Display image size
image_size=$(docker images px4-airsim:mavlink-router --format "table {{.Size}}" | tail -1)
echo "📊 Image size: $image_size"

echo ""
echo "🎯 Next steps:"
echo "1. Start containers with MAVLink Router:"
echo "   docker-compose -f docker-compose-mavlink-router.yml up px4-bridge-drone-1"
echo ""
echo "2. Test external connectivity:"
echo "   • QGroundControl: Connect to localhost:14550 (UDP)"
echo "   • MAVSDK/API: Connect to localhost:14540 (UDP)"
echo "   • TCP Clients: Connect to localhost:5761 (TCP)"
echo ""
echo "3. Monitor MAVLink Router logs:"
echo "   docker logs px4-bridge-drone-1"

echo "✅ Build completed successfully!"