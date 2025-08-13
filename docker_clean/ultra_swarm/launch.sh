#!/bin/bash
echo "================================================"
echo "   Unified AirSim Docker Launcher"
echo "   Generated for 9 vehicle(s)"
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

echo "[INFO] Starting 9 vehicle instance(s)..."
docker-compose -f docker-compose.yml up -d

if [ $? -ne 0 ]; then
    echo "[ERROR] Failed to start vehicle instances!"
    exit 1
fi

echo
echo "================================================"
echo "   9 Vehicle(s) Started Successfully!"
echo "================================================"
echo

echo "Port Configuration:"
echo "  PX4_Swarm1_Drone1: TCP=4561 | MAVLink=14550/18570 | QGC=14550"
echo "  PX4_Swarm1_Drone2: TCP=4562 | MAVLink=14551/18571 | QGC=14551"
echo "  PX4_Swarm1_Drone3: TCP=4563 | MAVLink=14552/18572 | QGC=14552"
echo "  PX4_Swarm1_Drone4: TCP=4564 | MAVLink=14553/18573 | QGC=14553"
echo "  PX4_Swarm1_Drone5: TCP=4565 | MAVLink=14554/18574 | QGC=14554"
echo "  PX4_Swarm1_Drone6: TCP=4566 | MAVLink=14555/18575 | QGC=14555"
echo "  PX4_Swarm1_Drone7: TCP=4567 | MAVLink=14556/18576 | QGC=14556"
echo "  PX4_Swarm1_Drone8: TCP=4568 | MAVLink=14557/18577 | QGC=14557"
echo "  PX4_Swarm1_Drone9: TCP=4569 | MAVLink=14558/18578 | QGC=14558"

echo
echo "Management Commands:"
echo "  View logs:        docker-compose -f docker-compose.yml logs"
echo "  Stop all:         docker-compose -f docker-compose.yml down"
echo "  Container status: docker-compose -f docker-compose.yml ps"
echo
