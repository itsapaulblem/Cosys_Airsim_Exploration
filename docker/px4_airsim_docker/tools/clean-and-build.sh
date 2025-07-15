#!/bin/bash

echo "🧹 Cleaning up existing images and containers..."

# Stop and remove any running containers
docker-compose -f docker-compose.host-network.yml down 2>/dev/null

# Remove existing images with this name
docker rmi px4-ultra-swarm-host:latest 2>/dev/null

# Clean up build cache
docker builder prune -f

echo "✅ Cleanup complete!"
echo ""

# Now build the image
./build-host-network.sh