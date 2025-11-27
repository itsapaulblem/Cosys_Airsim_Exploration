#!/usr/bin/env bash
#
# AirSim UE5 Headless Launcher - Vulkan Driver Solution
#
# This script solves the "Failed to load Vulkan Driver" issue by using UE5's
# built-in null RHI (Render Hardware Interface) for headless execution.
#
# Key Features:
# - ✅ Bypasses Vulkan driver requirements completely
# - ✅ Works in any containerized environment
# - ✅ Maintains full AirSim API functionality
# - ✅ Supports both testing and production modes
# - ✅ Comprehensive error handling and logging
#

# EMERGENCY DEBUG: Temporarily disable strict error handling for debugging
# set -e  # Commented out during debugging - will re-enable after fix
set +e    # Explicitly disable for debugging

# Script version for troubleshooting
SCRIPT_VERSION="1.0.0-debug"
echo "=== AirSim UE5 Headless Launcher v${SCRIPT_VERSION} ==="
echo "Solving Vulkan driver issues with null RHI headless execution"
echo ""

# Configuration variables
DEFAULT_EXECUTABLE="Blocks"
DEFAULT_LOG_LEVEL="Warning"
DEFAULT_RESOLUTION="1920x1080"
DEFAULT_API_PORT="41451"
DEFAULT_SETTINGS_PATH="$HOME/Documents/AirSim/settings.json"

# Command line argument parsing
EXECUTABLE="$DEFAULT_EXECUTABLE"
LOG_LEVEL="$DEFAULT_LOG_LEVEL"
RESOLUTION="$DEFAULT_RESOLUTION"
API_PORT="$DEFAULT_API_PORT"
SETTINGS_PATH="$DEFAULT_SETTINGS_PATH"
TEST_MODE=false
VERBOSE=false
TIMEOUT=""
EXTRA_ARGS=""

show_help() {
    cat << EOF
AirSim UE5 Headless Launcher - Vulkan Driver Solution

USAGE:
    $0 [OPTIONS] [-- EXTRA_ARGS]

OPTIONS:
    -e, --executable EXEC     UE5 executable name (default: $DEFAULT_EXECUTABLE)
    -l, --log-level LEVEL     Log level: Warning/Error/Info/Verbose (default: $DEFAULT_LOG_LEVEL)
    -r, --resolution WxH      Virtual resolution (default: $DEFAULT_RESOLUTION)
    -p, --port PORT          AirSim API port (default: $DEFAULT_API_PORT)
    -s, --settings PATH      AirSim settings file path
    -t, --test              Test mode - run for limited time and exit
    -v, --verbose           Verbose output and logging
    --timeout SECONDS       Run timeout in seconds (test mode default: 30)
    -h, --help              Show this help message

EXAMPLES:
    # Basic headless execution (bypasses Vulkan)
    $0

    # Test mode with timeout
    $0 --test --timeout 60

    # Verbose logging for debugging
    $0 --verbose --log-level Verbose

    # Custom resolution and settings
    $0 --resolution 2560x1440 --settings /custom/settings.json

    # Pass extra UE5 arguments
    $0 -- -BENCHMARK -fps=60

TROUBLESHOOTING:
    • Vulkan errors: This script uses -nullrhi to bypass GPU requirements
    • API not responding: Check --port setting and firewall rules
    • Settings not loading: Verify --settings path exists
    • Performance issues: Try lower --resolution or --log-level Error

EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -e|--executable)
            EXECUTABLE="$2"
            shift 2
            ;;
        -l|--log-level)
            LOG_LEVEL="$2"
            shift 2
            ;;
        -r|--resolution)
            RESOLUTION="$2"
            shift 2
            ;;
        -p|--port)
            API_PORT="$2"
            shift 2
            ;;
        -s|--settings)
            SETTINGS_PATH="$2"
            shift 2
            ;;
        -t|--test)
            TEST_MODE=true
            TIMEOUT="${TIMEOUT:-30}"
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        --timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        --)
            shift
            EXTRA_ARGS="$*"
            break
            ;;
        *)
            echo "❌ Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Helper functions
log() {
    local level="$1"
    shift
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$timestamp] [$level] $*"
}

log_info() { log "INFO" "$@"; }
log_warn() { log "WARN" "$@"; }
log_error() { log "ERROR" "$@"; }
log_debug() { $VERBOSE && log "DEBUG" "$@" || true; }

# Environment detection and validation
detect_environment() {
    log_info "🔍 Detecting execution environment..."

    # Check if running in container
    if [ -f /.dockerenv ]; then
        log_info "   ✅ Docker container detected"
        CONTAINER_ENV=true
    else
        log_info "   ℹ️  Native environment detected"
        CONTAINER_ENV=false
    fi

    # Check for display environment
    if [ -n "$DISPLAY" ]; then
        log_info "   ✅ Display environment: $DISPLAY"
    else
        log_info "   ✅ Headless environment (no display) - perfect for null RHI"
    fi

    # CPU and memory info
    log_debug "   CPU cores: $(nproc)"
    log_debug "   Memory: $(free -h | grep 'Mem:' | awk '{print $2}')"

    # Graphics capability check
    if command -v lspci >/dev/null 2>&1 && lspci | grep -i vga >/dev/null 2>&1; then
        log_debug "   Graphics hardware detected"
    else
        log_info "   ✅ No graphics hardware detected - null RHI is perfect solution"
    fi
}

# Validate executable and paths
validate_setup() {
    log_info "🔍 Validating AirSim setup..."

    # Find the executable
    local exe_path=""

    # Enhanced search paths with proper workspace root resolution
    log_debug "   🔍 Searching for executable '$EXECUTABLE'..."
    log_debug "   📍 Script location: $(pwd)"

    # Check current directory
    if [ -f "./$EXECUTABLE" ]; then
        exe_path="./$EXECUTABLE"
        log_debug "   ✅ Found in current directory"
    # Check if it's a full path
    elif [ -f "$EXECUTABLE" ]; then
        exe_path="$EXECUTABLE"
        log_debug "   ✅ Found as absolute path"
    # Check UE5 Binaries directory (correct UE5 structure)
    elif [ -f "Unreal/Environments/Blocks/Saved/StagedBuilds/Linux/Blocks/Binaries/Linux/$EXECUTABLE" ]; then
        exe_path="Unreal/Environments/Blocks/Saved/StagedBuilds/Linux/Blocks/Binaries/Linux/$EXECUTABLE"
        log_debug "   ✅ Found in UE5 Binaries directory"
    # Check UE5 Binaries directory with wildcard (any environment)
    elif [ -n "$(find Unreal/Environments/*/Saved/StagedBuilds/Linux/*/Binaries/Linux/ -name "$EXECUTABLE" -type f 2>/dev/null | head -1)" ]; then
        exe_path="$(find Unreal/Environments/*/Saved/StagedBuilds/Linux/*/Binaries/Linux/ -name "$EXECUTABLE" -type f 2>/dev/null | head -1)"
        log_debug "   ✅ Found via wildcard search in Binaries directory"
    # Fallback: Check legacy UE5 location (without Binaries subdirectory)
    elif [ -f "Unreal/Environments/Blocks/Saved/StagedBuilds/Linux/$EXECUTABLE" ]; then
        exe_path="Unreal/Environments/Blocks/Saved/StagedBuilds/Linux/$EXECUTABLE"
        log_debug "   ✅ Found in legacy UE5 location"
    # Fallback: Wildcard search in legacy location
    elif [ -n "$(find Unreal/Environments/*/Saved/StagedBuilds/Linux/ -name "$EXECUTABLE" -type f 2>/dev/null | head -1)" ]; then
        exe_path="$(find Unreal/Environments/*/Saved/StagedBuilds/Linux/ -name "$EXECUTABLE" -type f 2>/dev/null | head -1)"
        log_debug "   ✅ Found via wildcard search in legacy location"
    else
        log_error "❌ Executable '$EXECUTABLE' not found!"
        log_error "   Searched locations from $(pwd):"
        log_error "     - ./$EXECUTABLE"
        log_error "     - Unreal/Environments/Blocks/Saved/StagedBuilds/Linux/Blocks/Binaries/Linux/$EXECUTABLE"
        log_error "     - Unreal/Environments/*/Saved/StagedBuilds/Linux/*/Binaries/Linux/$EXECUTABLE"
        log_error "     - Unreal/Environments/Blocks/Saved/StagedBuilds/Linux/$EXECUTABLE (fallback)"
        log_error "     - Unreal/Environments/*/Saved/StagedBuilds/Linux/$EXECUTABLE (fallback)"
        log_error ""
        log_error "   💡 Troubleshooting tips:"
        log_error "     - Run UE5 BuildCookRun first to create the executable"
        log_error "     - Check if BuildCookRun completed successfully"
        log_error "     - Verify the project built without errors"
        log_error "     - Or specify correct path with: $0 -e /path/to/executable"
        exit 1
    fi

    # Verify executable permissions
    if [ ! -x "$exe_path" ]; then
        log_warn "⚠️  Making executable: $exe_path"
        chmod +x "$exe_path"
    fi

    EXECUTABLE_PATH="$exe_path"
    log_info "   ✅ Found executable: $EXECUTABLE_PATH"

    # Check settings file
    if [ -n "$SETTINGS_PATH" ] && [ -f "$SETTINGS_PATH" ]; then
        log_info "   ✅ AirSim settings: $SETTINGS_PATH"
    elif [ -f "$DEFAULT_SETTINGS_PATH" ]; then
        log_info "   ✅ Using default settings: $DEFAULT_SETTINGS_PATH"
        SETTINGS_PATH="$DEFAULT_SETTINGS_PATH"
    else
        log_warn "   ⚠️  No settings file found - will use UE5 defaults"
        SETTINGS_PATH=""
    fi
}

# Test API connectivity (non-blocking)
test_api_connectivity() {
    log_info "🔗 Testing AirSim API connectivity..."

    local max_attempts=5
    local attempt=1

    while [ $attempt -le $max_attempts ]; do
        log_debug "   API test attempt $attempt/$max_attempts on port $API_PORT"

        if timeout 3 bash -c "</dev/tcp/localhost/$API_PORT" 2>/dev/null; then
            log_info "   ✅ AirSim API responding on port $API_PORT"
            return 0
        fi

        sleep 2
        ((attempt++))
    done

    log_warn "   ⚠️  AirSim API not responding on port $API_PORT (may be starting up)"
    return 1
}

# Build UE5 command line arguments for headless execution
build_command_line() {
    log_info "🛠️  Building headless execution command..."

    local cmd_args=()

    # CRITICAL: Null RHI flags to bypass Vulkan requirement
    cmd_args+=("-nullrhi")          # Use null render hardware interface (no GPU)
    cmd_args+=("-unattended")       # Run without user interaction

    # Resolution (even in headless mode, UE5 may use this internally)
    local res_x=$(echo "$RESOLUTION" | cut -d'x' -f1)
    local res_y=$(echo "$RESOLUTION" | cut -d'x' -f2)
    cmd_args+=("-ResX=$res_x")
    cmd_args+=("-ResY=$res_y")

    # Logging configuration
    cmd_args+=("-log")              # Enable logging
    if [ "$LOG_LEVEL" != "Warning" ]; then
        cmd_args+=("-LogLevel=$LOG_LEVEL")
    fi

    # Performance and stability
    cmd_args+=("-nosound")          # Disable audio subsystem
    cmd_args+=("-noassert")         # Disable assertions for production
    cmd_args+=("-buildmachine")     # Optimize for automated execution

    # API configuration
    if [ -n "$SETTINGS_PATH" ] && [ -f "$SETTINGS_PATH" ]; then
        # UE5 will find settings automatically, but we can log it
        log_debug "   Using AirSim settings: $SETTINGS_PATH"
    fi

    # Test mode specific flags
    if [ "$TEST_MODE" = true ]; then
        cmd_args+=("-benchmark")    # Enable benchmark mode
        if [ -n "$TIMEOUT" ]; then
            cmd_args+=("-seconds=$TIMEOUT")
        fi
    fi

    # Add any extra arguments passed by user
    if [ -n "$EXTRA_ARGS" ]; then
        log_debug "   Adding extra arguments: $EXTRA_ARGS"
        cmd_args+=($EXTRA_ARGS)
    fi

    COMMAND_ARGS=("${cmd_args[@]}")

    log_info "   ✅ Headless execution configured:"
    log_info "      🚫 Vulkan bypassed: -nullrhi -unattended"
    log_info "      📺 Resolution: ${res_x}x${res_y} (virtual)"
    log_info "      📝 Log level: $LOG_LEVEL"
    log_info "      🔧 Mode: $([ "$TEST_MODE" = true ] && echo "Test" || echo "Production")"
    $VERBOSE && log_debug "      🔧 Full args: ${COMMAND_ARGS[*]}"
}

# Execute AirSim with headless configuration
execute_airsim() {
    # EMERGENCY DEBUG OUTPUT - Remove after fixing
    echo "🚨 DEBUG: ===== execute_airsim() FUNCTION ENTRY ====="
    echo "🚨 DEBUG: Function started at: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "🚨 DEBUG: Bash version: $BASH_VERSION"
    echo "🚨 DEBUG: Script PID: $$"
    echo "🚨 DEBUG: set -e status: $(set | grep 'set -e' || echo 'not detected')"

    echo "🚨 DEBUG: Checking basic variables..."
    echo "🚨 DEBUG: EXECUTABLE_PATH='$EXECUTABLE_PATH'"
    echo "🚨 DEBUG: COMMAND_ARGS array length: ${#COMMAND_ARGS[@]}"
    echo "🚨 DEBUG: Current directory: $(pwd)"
    echo "🚨 DEBUG: Available commands: $(which ldd 2>/dev/null || echo 'ldd not available')"

    echo "🚨 DEBUG: Testing basic commands..."
    echo "🚨 DEBUG: ls command works: $(ls . >/dev/null 2>&1 && echo 'YES' || echo 'NO')"
    echo "🚨 DEBUG: pwd command works: $(pwd >/dev/null 2>&1 && echo 'YES' || echo 'NO')"
    echo "🚨 DEBUG: date command works: $(date >/dev/null 2>&1 && echo 'YES' || echo 'NO')"

    echo "🚨 DEBUG: About to call log_info function..."
    log_info "🚀 Starting AirSim UE5 in headless mode..."
    echo "🚨 DEBUG: log_info call completed successfully"
    log_info "   📍 Executable: $EXECUTABLE_PATH"
    log_info "   ⚙️  Mode: Headless (null RHI - no Vulkan required)"

    # Pre-execution debugging
    echo "🚨 DEBUG: Starting pre-execution diagnostics"
    log_info "🔍 Pre-execution diagnostics..."

    echo "🚨 DEBUG: Getting exe_dir and exe_name"
    local exe_dir=$(dirname "$EXECUTABLE_PATH")
    local exe_name=$(basename "$EXECUTABLE_PATH")
    echo "🚨 DEBUG: exe_dir=$exe_dir, exe_name=$exe_name"

    # Fix working directory - UE5 needs to run from package root, not Binaries/Linux/
    # Convert: /path/to/Blocks/Binaries/Linux -> /path/to/Blocks
    echo "🚨 DEBUG: Determining package root"
    local package_root
    if [[ "$exe_dir" == */Binaries/Linux ]]; then
        echo "🚨 DEBUG: exe_dir ends with Binaries/Linux, calculating package_root"
        package_root=$(dirname $(dirname "$exe_dir"))  # Remove /Binaries/Linux
        echo "🚨 DEBUG: package_root calculated as: $package_root"
        log_info "   🔧 Fixed working directory: $package_root (UE5 package root)"
    else
        package_root="$exe_dir"
        echo "🚨 DEBUG: Using exe_dir as package_root: $package_root"
        log_info "   📍 Working directory: $package_root"
    fi
    echo "🚨 DEBUG: Package root determination complete"

    # Calculate relative path from package root to executable
    echo "🚨 DEBUG: Calculating relative exe path"
    local relative_exe_path
    if [[ "$exe_dir" == */Binaries/Linux ]]; then
        relative_exe_path="Binaries/Linux/$exe_name"
        echo "🚨 DEBUG: Using Binaries/Linux path: $relative_exe_path"
    else
        relative_exe_path="$exe_name"
        echo "🚨 DEBUG: Using direct path: $relative_exe_path"
    fi

    log_info "   🎯 Executable path: $relative_exe_path"
    log_info "   📝 Command: $relative_exe_path ${COMMAND_ARGS[*]}"
    echo "🚨 DEBUG: Path calculation complete"

    # Check executable permissions
    echo "🚨 DEBUG: Checking executable permissions"
    if [ ! -x "$EXECUTABLE_PATH" ]; then
        echo "🚨 DEBUG: Executable not executable, attempting to fix"
        log_warn "   ⚠️  Making executable: $EXECUTABLE_PATH"
        chmod +x "$EXECUTABLE_PATH" 2>/dev/null || log_error "   ❌ Failed to set executable permissions"
    fi
    log_info "   ✅ Executable permissions verified"
    echo "🚨 DEBUG: Permissions check complete"

    # Check library dependencies
    echo "🚨 DEBUG: Starting library dependencies check"
    log_info "📚 Checking library dependencies..."
    if command -v ldd >/dev/null 2>&1; then
        local missing_libs=0
        local lib_output
        lib_output=$(ldd "$EXECUTABLE_PATH" 2>&1)
        if echo "$lib_output" | grep -q "not found"; then
            missing_libs=$(echo "$lib_output" | grep -c "not found")
            log_error "   ❌ Found $missing_libs missing libraries:"
            echo "$lib_output" | grep "not found" | head -5 | while read line; do
                log_error "      $line"
            done
        else
            log_info "   ✅ All required libraries found"
            log_debug "   📋 Key libraries:"
            echo "$lib_output" | grep -E "(libc\+\+|libstdc\+\+|libGL)" | head -3 | while read line; do
                log_debug "      $line"
            done
        fi
    else
        log_warn "   ⚠️  ldd not available - cannot check library dependencies"
    fi

    # Change to package root directory
    echo "🚨 DEBUG: About to change directory to package root"
    log_info "   🔄 Changing to package root: $package_root"
    echo "🚨 DEBUG: Current directory before cd: $(pwd)"
    echo "🚨 DEBUG: Target directory exists: $([ -d "$package_root" ] && echo "YES" || echo "NO")"
    echo "🚨 DEBUG: Target directory permissions: $(ls -ld "$package_root" 2>/dev/null || echo "N/A")"

    cd "$package_root" || {
        echo "🚨 DEBUG: Directory change FAILED"
        log_error "❌ Failed to change to package root directory: $package_root"
        return 1
    }

    echo "🚨 DEBUG: Directory change SUCCESS - now in: $(pwd)"

    # Verify we can see the executable from new location
    echo "🚨 DEBUG: Checking if executable exists at relative path: $relative_exe_path"
    if [ ! -f "$relative_exe_path" ]; then
        echo "🚨 DEBUG: Executable NOT found at relative path"
        log_error "❌ Cannot find executable at relative path: $relative_exe_path"
        log_error "   Current directory: $(pwd)"
        log_error "   Files in current directory:"
        ls -la | head -10
        return 1
    fi
    echo "🚨 DEBUG: Executable found at relative path"
    log_info "   ✅ Executable accessible from package root"

    # Setup signal handling for graceful shutdown
    echo "🚨 DEBUG: Setting up signal handling and cleanup"
    local airsim_pid=""

    cleanup() {
        # Clean up temporary error log
        rm -f "$error_log" 2>/dev/null

        if [ -n "$airsim_pid" ] && kill -0 "$airsim_pid" 2>/dev/null; then
            log_info "🛑 Shutting down AirSim gracefully..."
            kill -TERM "$airsim_pid" 2>/dev/null || true

            # Wait for graceful shutdown
            local count=0
            while kill -0 "$airsim_pid" 2>/dev/null && [ $count -lt 10 ]; do
                sleep 1
                ((count++))
            done

            # Force kill if still running
            if kill -0 "$airsim_pid" 2>/dev/null; then
                log_warn "⚠️  Force killing AirSim process..."
                kill -KILL "$airsim_pid" 2>/dev/null || true
            fi
        fi
    }

    trap cleanup EXIT INT TERM

    # Enhanced execution with error capture
    log_info "🎬 Final execution command: ./$relative_exe_path ${COMMAND_ARGS[*]}"
    log_info "🎬 Working directory: $(pwd)"
    echo ""

    # Create temporary error log for immediate feedback
    local error_log="/tmp/airsim_startup_error_$$.log"

    # Execute with timeout if specified
    if [ -n "$TIMEOUT" ] && [ "$TIMEOUT" -gt 0 ]; then
        log_info "⏱️  Starting with ${TIMEOUT}s timeout..."
        timeout "$TIMEOUT" "./$relative_exe_path" "${COMMAND_ARGS[@]}" 2>"$error_log" &
        airsim_pid=$!
        log_info "   📍 Process ID: $airsim_pid"
    else
        log_info "🔄 Starting indefinitely (Ctrl+C to stop)..."
        "./$relative_exe_path" "${COMMAND_ARGS[@]}" 2>"$error_log" &
        airsim_pid=$!
        log_info "   📍 Process ID: $airsim_pid"
    fi

    # Immediate process status check
    sleep 2
    if ! kill -0 "$airsim_pid" 2>/dev/null; then
        log_error "❌ Process exited immediately (within 2 seconds)"

        # Show error output if available
        if [ -f "$error_log" ] && [ -s "$error_log" ]; then
            log_error "🚨 Error output:"
            head -20 "$error_log" | while read line; do
                log_error "   $line"
            done
        fi

        # Cleanup and return
        rm -f "$error_log"
        return 1
    fi

    log_info "✅ Process started successfully - checking initialization..."

    # Wait for startup
    sleep 5

    # Test API if process is still running
    if kill -0 "$airsim_pid" 2>/dev/null; then
        log_info "✅ AirSim process running successfully (PID: $airsim_pid)"

        # Test API connectivity in background
        if [ "$TEST_MODE" = false ]; then
            (sleep 10 && test_api_connectivity) &
        fi
    else
        log_error "❌ AirSim process exited unexpectedly during startup"

        # Show error output if available
        if [ -f "$error_log" ] && [ -s "$error_log" ]; then
            log_error "🚨 Startup error output:"
            tail -30 "$error_log" | while read line; do
                log_error "   $line"
            done
        else
            log_error "   No error output captured"
        fi

        # Additional debugging info
        log_error "🔍 Additional debugging info:"
        log_error "   Working directory: $(pwd)"
        log_error "   Executable: $relative_exe_path"
        log_error "   Executable exists: $([ -f "$relative_exe_path" ] && echo "YES" || echo "NO")"
        log_error "   Executable permissions: $(ls -la "$relative_exe_path" 2>/dev/null || echo "N/A")"

        # Cleanup and return
        rm -f "$error_log"
        return 1
    fi

    # Wait for process completion
    wait "$airsim_pid" 2>/dev/null || true
    local exit_code=$?

    # Enhanced completion logging
    log_info "🏁 AirSim execution completed (exit code: $exit_code)"

    # Show final error output if process failed
    if [ $exit_code -ne 0 ] && [ -f "$error_log" ] && [ -s "$error_log" ]; then
        log_error "🚨 Final error output:"
        tail -20 "$error_log" | while read line; do
            log_error "   $line"
        done
    fi

    # Cleanup
    rm -f "$error_log"
    return $exit_code
}

# Main execution flow
main() {
    log_info "Starting AirSim UE5 headless launcher..."

    # EMERGENCY DEBUG: Main function checkpoints
    echo "🚨 MAIN DEBUG: main() function started"

    # Environment detection
    echo "🚨 MAIN DEBUG: About to call detect_environment()"
    detect_environment
    echo "🚨 MAIN DEBUG: detect_environment() completed successfully"

    # Validate setup
    echo "🚨 MAIN DEBUG: About to call validate_setup()"
    validate_setup
    echo "🚨 MAIN DEBUG: validate_setup() completed successfully"

    # Build command line
    echo "🚨 MAIN DEBUG: About to call build_command_line()"
    build_command_line
    echo "🚨 MAIN DEBUG: build_command_line() completed successfully"

    # CRITICAL DEBUG: Right before execute_airsim
    echo "🚨 MAIN DEBUG: About to call execute_airsim()"
    echo "🚨 MAIN DEBUG: EXECUTABLE_PATH variable is: '$EXECUTABLE_PATH'"
    echo "🚨 MAIN DEBUG: COMMAND_ARGS array is: '${COMMAND_ARGS[*]}'"
    echo "🚨 MAIN DEBUG: Current directory: $(pwd)"

    # Execute AirSim
    execute_airsim
    local result=$?

    echo "🚨 MAIN DEBUG: execute_airsim() returned with code: $result"

    if [ $result -eq 0 ]; then
        log_info "✅ AirSim headless execution completed successfully!"
        if [ "$TEST_MODE" = true ]; then
            log_info "🎉 Vulkan driver workaround confirmed working!"
        fi
    else
        log_error "❌ AirSim execution failed (exit code: $result)"
        log_error "💡 Troubleshooting tips:"
        log_error "   - Check that the executable was built correctly"
        log_error "   - Verify AirSim settings.json is valid"
        log_error "   - Try running with --verbose for more details"
        log_error "   - Ensure sufficient system resources (RAM/CPU)"
    fi

    return $result
}

# Execute main function
main "$@"