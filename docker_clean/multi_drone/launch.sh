#!/bin/bash
echo "================================================"
echo "   Unified AirSim Docker Launcher"
echo "   Generated for 2 vehicle(s)"
echo "================================================"
echo

echo "[INFO] Creating shared data directory..."
mkdir -p shared_data

echo "[INFO] Building Docker images..."
docker-compose -f docker-compose.yml build

if [ $? -ne 0 ]; then
    echo "[ERROR] Failed to build Docker images!"
    exit 1
fi

echo "[INFO] Cleaning up existing containers..."
docker-compose -f docker-compose.yml down > /dev/null 2>&1

echo "[INFO] Starting 2 vehicle instance(s)..."
docker-compose -f docker-compose.yml up -d

if [ $? -ne 0 ]; then
    echo "[ERROR] Failed to start vehicle instances!"
    exit 1
fi

echo
echo "================================================"
echo "   2 Vehicle(s) Started Successfully!"
echo "================================================"
echo

echo "Port Configuration:"
echo "  PX4_Drone1: TCP=4561 | MAVLink=14541/14581 | QGC=14550"
echo "  PX4_Drone2: TCP=4562 | MAVLink=14542/14582 | QGC=14551"

echo
echo "Management Commands:"
echo "  View logs:        docker-compose -f docker-compose.yml logs"
echo "  Stop all:         docker-compose -f docker-compose.yml down"
echo "  Container status: docker-compose -f docker-compose.yml ps"
echo
