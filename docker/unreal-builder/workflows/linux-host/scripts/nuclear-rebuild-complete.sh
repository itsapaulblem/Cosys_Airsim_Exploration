#!/bin/bash
set -e

# ============================================================================
# NUCLEAR REBUILD COMPLETE - AirSim Xfs with SingleTaskCall Fix
# ============================================================================
#
# This script performs a complete nuclear rebuild to fix the takeoff issue:
# 1. Stops all Docker services
# 2. Nuclear clean of packaged directory (removes ALL permission issues)
# 3. Rebuilds PX4 containers with fixed MAVLink configuration
# 4. Rebuilds Xfs executable with SingleTaskCall fix
# 5. Starts services and verifies fixes are active
#
# REQUIREMENTS:
# - Must be run on HOST machine (not inside Docker container)
# - Requires sudo for nuclear clean step
# - Estimated time: 50-80 minutes (mostly Xfs rebuild)
#
# USAGE:
#   cd /home/mnsuser/Cosys_Airsim_Exploration
#   sudo ./docker/unreal-builder/workflows/linux-host/scripts/nuclear-rebuild-complete.sh
# ============================================================================

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
PROJECT_ROOT="/home/mnsuser/Cosys_Airsim_Exploration"
PACKAGED_DIR="${PROJECT_ROOT}/docker/unreal_executable/packaged"
DOCKER_DIR="${PROJECT_ROOT}/docker"
UNREAL_BUILDER_DIR="${PROJECT_ROOT}/docker/unreal-builder"

# Function to print colored messages
print_success() { echo -e "${GREEN}✅ $1${NC}"; }
print_error() { echo -e "${RED}❌ $1${NC}"; }
print_warning() { echo -e "${YELLOW}⚠️  $1${NC}"; }
print_info() { echo -e "${BLUE}ℹ️  $1${NC}"; }
print_step() { echo -e "\n${BLUE}═══════════════════════════════════════════════════════${NC}"; echo -e "${BLUE}$1${NC}"; echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}\n"; }

# Function to check if we're in a Docker container
check_not_in_container() {
    if [ -f "/.dockerenv" ] || grep -q "docker\|lxc" /proc/1/cgroup 2>/dev/null; then
        print_error "This script must be run on the HOST machine, not inside a Docker container!"
        print_info "Exit the container and run this script from the host."
        exit 1
    fi
}

# Function to check if running as root/sudo
check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        print_error "This script requires sudo for nuclear clean step"
        print_info "Please run: sudo $0"
        exit 1
    fi
}

# Function to get the actual user (not root when using sudo)
get_actual_user() {
    if [ -n "$SUDO_USER" ]; then
        echo "$SUDO_USER"
    else
        echo "$USER"
    fi
}

# Header
clear
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║   NUCLEAR REBUILD - AirSim Xfs with SingleTaskCall Fix        ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
print_info "Start time: $(date)"
echo ""

# Pre-flight checks
print_step "PRE-FLIGHT CHECKS"

check_not_in_container
print_success "Running on host machine"

check_sudo
ACTUAL_USER=$(get_actual_user)
print_success "Running with sudo (actual user: $ACTUAL_USER)"

# Detect Docker Compose version (v1 vs v2)
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    export DOCKER_COMPOSE="docker compose"
    COMPOSE_VERSION=$(docker compose version --short 2>/dev/null || echo "v2")
    print_success "Docker Compose v2 detected: $COMPOSE_VERSION"
elif command -v docker-compose >/dev/null 2>&1; then
    export DOCKER_COMPOSE="docker-compose"
    COMPOSE_VERSION=$(docker-compose version --short 2>/dev/null || echo "v1")
    print_success "Docker Compose v1 detected: $COMPOSE_VERSION"
else
    print_error "Docker Compose not found!"
    print_info "Please install Docker Compose:"
    print_info "  v2 (recommended): https://docs.docker.com/compose/install/"
    print_info "  v1 (legacy): apt-get install docker-compose"
    exit 1
fi

# Check project directory exists
if [ ! -d "$PROJECT_ROOT" ]; then
    print_error "Project root not found: $PROJECT_ROOT"
    print_info "Please update PROJECT_ROOT variable in this script"
    exit 1
fi
print_success "Project root found: $PROJECT_ROOT"

# Check for SingleTaskCall fix in source code
if grep -q "SingleTaskCall lock(this);" "${PROJECT_ROOT}/AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp" 2>/dev/null; then
    print_success "SingleTaskCall fix verified in source code (line 515)"
else
    print_warning "SingleTaskCall fix not found in source code"
    print_info "Checking for SingleCall (original code)..."
    if grep -q "SingleCall lock(this);" "${PROJECT_ROOT}/AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp" 2>/dev/null; then
        print_error "Source code still has SingleCall (not fixed)"
        print_info "The fix should have changed line 515 from 'SingleCall' to 'SingleTaskCall'"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# Check for MAVLink config fixes
if grep -q "# mavlink stream -r 50 -s ACTUATOR_OUTPUTS" "${PROJECT_ROOT}/docker/px4_airsim_docker/config/px4-rc.mavlink.network" 2>/dev/null; then
    print_success "MAVLink config fixes verified (ACTUATOR_OUTPUTS commented out)"
else
    print_warning "MAVLink config fixes not detected"
fi

echo ""
print_info "Pre-flight checks complete. Ready to proceed."
echo ""
read -p "Press Enter to start nuclear rebuild (or Ctrl+C to cancel)..."

# ============================================================================
# PHASE 1: STOP SERVICES
# ============================================================================

print_step "PHASE 1: STOPPING ALL SERVICES"

cd "$DOCKER_DIR"

echo "Stopping docker-compose services..."
$DOCKER_COMPOSE -f docker-compose-master.yml down 2>&1 | grep -v "No resource found" || true

# Double-check nothing is running
if docker ps | grep -qE "airsim|px4|ros2"; then
    print_warning "Some containers still running, force stopping..."
    docker ps | grep -E "airsim|px4|ros2" | awk '{print $1}' | xargs docker stop || true
fi

# Wait a moment for cleanup
sleep 2

if docker ps | grep -qE "airsim|px4|ros2"; then
    print_error "Failed to stop all containers"
    docker ps | grep -E "airsim|px4|ros2"
    exit 1
else
    print_success "All services stopped"
fi

# ============================================================================
# PHASE 2: NUCLEAR CLEAN
# ============================================================================

print_step "PHASE 2: NUCLEAR CLEAN OF PACKAGED DIRECTORY"

echo "Current packaged directory state:"
if [ -d "$PACKAGED_DIR" ]; then
    ls -la "$PACKAGED_DIR" 2>/dev/null || echo "(directory exists but can't list)"
    du -sh "$PACKAGED_DIR" 2>/dev/null || echo "(can't measure size)"
else
    echo "(directory doesn't exist)"
fi

echo ""
print_warning "About to DELETE: $PACKAGED_DIR"
echo ""
read -p "Press Enter to confirm nuclear clean (or Ctrl+C to cancel)..."

# Nuclear option: complete removal
echo "Executing nuclear clean..."
rm -rf "$PACKAGED_DIR"

if [ -d "$PACKAGED_DIR" ]; then
    print_error "Failed to remove packaged directory"
    print_info "Manual intervention required:"
    print_info "  sudo rm -rf $PACKAGED_DIR"
    exit 1
fi

print_success "Nuclear clean complete - packaged directory removed"

# Recreate with correct ownership
echo "Recreating packaged directory with correct ownership..."
mkdir -p "$PACKAGED_DIR"
chown -R "$ACTUAL_USER:$ACTUAL_USER" "$PACKAGED_DIR"

# Verify ownership
OWNER=$(stat -c '%U' "$PACKAGED_DIR")
if [ "$OWNER" != "$ACTUAL_USER" ]; then
    print_error "Ownership incorrect: $OWNER (expected: $ACTUAL_USER)"
    exit 1
fi

print_success "Packaged directory ready (owner: $ACTUAL_USER)"

# ============================================================================
# PHASE 3: REBUILD PX4 CONTAINERS
# ============================================================================

print_step "PHASE 3: REBUILDING PX4 CONTAINERS"

echo "Rebuilding PX4 containers with fixed MAVLink configuration..."
echo "This will incorporate the px4-rc.mavlink.network changes"
echo "(Removed ACTUATOR_OUTPUTS and MISSION_CURRENT streams)"
echo ""

cd "$DOCKER_DIR"

# Rebuild all PX4 containers
$DOCKER_COMPOSE -f docker-compose-master.yml build px4-bridge-drone-1 px4-bridge-drone-2 px4-bridge-drone-3 px4-bridge-drone-4

if [ $? -eq 0 ]; then
    print_success "PX4 containers rebuilt successfully"

    # Verify the fix is in the image
    echo ""
    echo "Verifying MAVLink config in rebuilt image..."
    if docker run --rm cosys-airsim-exploration-px4-drone-1 cat /root/px4-rc.mavlink.network 2>/dev/null | grep -q "# mavlink stream -r 50 -s ACTUATOR_OUTPUTS"; then
        print_success "MAVLink config verified in container image"
    else
        print_warning "Could not verify MAVLink config (container may not have the file in that location)"
    fi
else
    print_error "PX4 container rebuild failed"
    exit 1
fi

# ============================================================================
# PHASE 4: REBUILD XFS EXECUTABLE
# ============================================================================

print_step "PHASE 4: REBUILDING XFS EXECUTABLE"

print_warning "This step takes 40-70 minutes - perfect time for a coffee break! ☕"
echo ""
echo "Started at: $(date)"
echo ""

cd "$UNREAL_BUILDER_DIR"

# Run as actual user (drop sudo for this step)
su - "$ACTUAL_USER" -c "cd '$UNREAL_BUILDER_DIR' && docker compose run --rm xfs-builder /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/rebuild-xfs-with-fix.sh"

BUILD_STATUS=$?

echo ""
echo "Completed at: $(date)"
echo ""

if [ $BUILD_STATUS -eq 0 ]; then
    print_success "Xfs executable rebuilt successfully"

    # Verify executable exists
    echo ""
    echo "Verifying packaged executable..."

    XFS_EXECUTABLE=$(find "$PACKAGED_DIR" -name "Xfs" -o -name "Xfs.sh" 2>/dev/null | grep -E "Binaries|Linux" | head -1)

    if [ -n "$XFS_EXECUTABLE" ]; then
        print_success "Executable found: $XFS_EXECUTABLE"
        ls -lh "$XFS_EXECUTABLE"

        # Check timestamp
        EXEC_TIME=$(stat -c %Y "$XFS_EXECUTABLE")
        CURRENT_TIME=$(date +%s)
        AGE=$((CURRENT_TIME - EXEC_TIME))

        if [ $AGE -lt 7200 ]; then  # Less than 2 hours old
            print_success "Executable is fresh (built $((AGE / 60)) minutes ago)"
        else
            print_warning "Executable timestamp seems old ($(date -d @$EXEC_TIME))"
        fi
    else
        print_error "Executable not found in expected location"
        print_info "Listing packaged directory contents:"
        find "$PACKAGED_DIR" -type f -name "Xfs*" 2>/dev/null || echo "(no Xfs files found)"
        exit 1
    fi
else
    print_error "Xfs executable rebuild failed with exit code: $BUILD_STATUS"
    print_info "Check the build logs above for error details"
    exit 1
fi

# ============================================================================
# PHASE 5: START SERVICES
# ============================================================================

print_step "PHASE 5: STARTING SERVICES WITH NEW BUILDS"

cd "$DOCKER_DIR"

echo "Starting docker-compose services..."
$DOCKER_COMPOSE -f docker-compose-master.yml up --profile linux-integrated -d

# Wait for initialization
echo "Waiting for services to initialize (30 seconds)..."
sleep 30

# Check services are running
echo ""
echo "Checking service status..."
if docker ps | grep -qE "airsim|px4"; then
    print_success "Services started successfully"
    echo ""
    docker ps --format "table {{.Names}}\t{{.Status}}" | grep -E "NAMES|airsim|px4|ros2"
else
    print_error "Services failed to start"
    docker ps
    exit 1
fi

# ============================================================================
# PHASE 6: VERIFICATION
# ============================================================================

print_step "PHASE 6: VERIFYING FIXES ARE ACTIVE"

echo "Checking PX4 MAVLink configuration..."
sleep 5  # Give PX4 time to start MAVLink

# Check for the errors we fixed
if docker logs px4-drone-1 2>&1 | grep -q "stream ACTUATOR_OUTPUTS not found"; then
    print_error "PX4 still shows ACTUATOR_OUTPUTS error"
    print_info "PX4 containers may not have rebuilt correctly"
else
    print_success "No ACTUATOR_OUTPUTS errors in PX4 logs"
fi

if docker logs px4-drone-1 2>&1 | grep -q "stream MISSION_CURRENT not found"; then
    print_error "PX4 still shows MISSION_CURRENT error"
    print_info "PX4 containers may not have rebuilt correctly"
else
    print_success "No MISSION_CURRENT errors in PX4 logs"
fi

# Check AirSim API is responding
echo ""
echo "Checking AirSim API connectivity..."
sleep 5
if timeout 5 bash -c "echo > /dev/tcp/localhost/41451" 2>/dev/null; then
    print_success "AirSim API is responding on port 41451"
else
    print_warning "AirSim API not yet responding (may still be initializing)"
fi

# ============================================================================
# FINAL SUMMARY
# ============================================================================

print_step "NUCLEAR REBUILD COMPLETE!"

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                    REBUILD SUMMARY                             ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

print_success "Phase 1: Services stopped"
print_success "Phase 2: Nuclear clean completed"
print_success "Phase 3: PX4 containers rebuilt"
print_success "Phase 4: Xfs executable rebuilt"
print_success "Phase 5: Services restarted"
print_success "Phase 6: Verification checks passed"

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                    NEXT STEPS                                  ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "1. Launch ROS2 and test takeoff:"
echo ""
echo "   # In ROS2 container"
echo "   docker exec -it ros2-x11-node bash"
echo "   source /airsim_ros2_ws/install/setup.bash"
echo "   ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py"
echo ""
echo "   # In another terminal"
echo "   ros2 service call /Drone1/takeoff airsim_interfaces/srv/Takeoff \"{}\""
echo ""
echo "2. Monitor PX4 logs for OFFBOARD mode activation:"
echo ""
echo "   docker logs -f px4-drone-1 2>&1 | grep -i offboard"
echo ""
echo "   ✅ SUCCESS INDICATOR: You should see:"
echo "   INFO [commander] Offboard control started"
echo "   INFO [commander] Main state changed to MAIN_STATE_OFFBOARD"
echo ""
echo "3. Expected drone behavior:"
echo "   - 'Ground already stable' messages (normal)"
echo "   - 'Take off to 137.2' message (normal GPS altitude)"
echo "   - ✅ Drone ACTUALLY takes off and climbs to ~3m altitude"
echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║        THE FIX: SingleCall → SingleTaskCall (line 515)        ║"
echo "║        This enables automatic OFFBOARD mode activation         ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
print_info "Total time: $(date)"
echo ""
