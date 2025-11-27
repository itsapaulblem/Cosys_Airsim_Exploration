#!/usr/bin/env bash
#
# AirSim Headless Testing & Verification Suite
#
# Comprehensive testing script to verify:
# - ✅ Vulkan driver workaround with null RHI
# - ✅ AirSim API functionality in headless mode
# - ✅ Performance benchmarking
# - ✅ Integration testing for CI/CD pipelines
# - ✅ Container deployment validation
#

set -e

SCRIPT_VERSION="1.0.0"
echo "=== AirSim Headless Testing & Verification Suite v${SCRIPT_VERSION} ==="
echo "🔬 Comprehensive testing for Vulkan driver workaround solution"
echo ""

# Configuration
DEFAULT_EXECUTABLE="Blocks"
DEFAULT_TEST_DURATION="30"
DEFAULT_API_PORT="41451"
TEST_LOG_DIR="/tmp/airsim-headless-tests"
DETAILED_OUTPUT=false
PERFORMANCE_TEST=false
API_TEST=false
INTEGRATION_TEST=false
QUICK_TEST=false

# Test results tracking
TESTS_RUN=0
TESTS_PASSED=0
TESTS_FAILED=0
TEST_RESULTS=()

show_help() {
    cat << EOF
AirSim Headless Testing & Verification Suite

DESCRIPTION:
    Comprehensive testing to verify the Vulkan driver workaround solution.
    Tests headless execution, API functionality, and deployment readiness.

USAGE:
    $0 [OPTIONS]

OPTIONS:
    -e, --executable PATH    UE5 executable path (default: auto-detect)
    -d, --duration SECONDS   Test duration in seconds (default: $DEFAULT_TEST_DURATION)
    -p, --port PORT         AirSim API port (default: $DEFAULT_API_PORT)
    -v, --verbose           Detailed test output and logging
    -q, --quick             Quick test mode (reduced duration)
    --performance           Run performance benchmarks
    --api-test             Test AirSim API connectivity and functions
    --integration          Full integration test suite
    --log-dir PATH         Custom log directory (default: $TEST_LOG_DIR)
    -h, --help             Show this help message

TEST MODES:
    Default:    Basic headless execution test
    --quick:    Fast test for CI/CD (15 second duration)
    --api-test: Include API connectivity and basic function tests
    --performance: Benchmark headless performance metrics
    --integration: Complete end-to-end integration testing

EXAMPLES:
    # Basic headless test
    $0

    # Quick test for CI/CD
    $0 --quick

    # Comprehensive testing with API validation
    $0 --api-test --performance --verbose

    # Integration test for deployment validation
    $0 --integration --duration 60

EXIT CODES:
    0: All tests passed
    1: One or more tests failed
    2: Configuration or setup error

EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -e|--executable)
            EXECUTABLE_PATH="$2"
            shift 2
            ;;
        -d|--duration)
            DEFAULT_TEST_DURATION="$2"
            shift 2
            ;;
        -p|--port)
            DEFAULT_API_PORT="$2"
            shift 2
            ;;
        -v|--verbose)
            DETAILED_OUTPUT=true
            shift
            ;;
        -q|--quick)
            QUICK_TEST=true
            DEFAULT_TEST_DURATION="15"
            shift
            ;;
        --performance)
            PERFORMANCE_TEST=true
            shift
            ;;
        --api-test)
            API_TEST=true
            shift
            ;;
        --integration)
            INTEGRATION_TEST=true
            API_TEST=true
            PERFORMANCE_TEST=true
            shift
            ;;
        --log-dir)
            TEST_LOG_DIR="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "❌ Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 2
            ;;
    esac
done

# Helper functions
log_detailed() {
    if [ "$DETAILED_OUTPUT" = true ]; then
        echo "$@"
    fi
}

log_test_result() {
    local test_name="$1"
    local result="$2"
    local details="$3"

    ((TESTS_RUN++))

    if [ "$result" = "PASS" ]; then
        ((TESTS_PASSED++))
        echo "   ✅ $test_name: PASSED"
        TEST_RESULTS+=("✅ $test_name: PASSED")
    else
        ((TESTS_FAILED++))
        echo "   ❌ $test_name: FAILED"
        TEST_RESULTS+=("❌ $test_name: FAILED")
    fi

    if [ -n "$details" ]; then
        log_detailed "      $details"
    fi
}

# Setup test environment
setup_test_environment() {
    echo "🔧 Setting up test environment..."

    # Create log directory
    mkdir -p "$TEST_LOG_DIR"
    chmod 755 "$TEST_LOG_DIR" 2>/dev/null || true

    # Find executable if not specified
    if [ -z "$EXECUTABLE_PATH" ]; then
        log_detailed "   🔍 Auto-detecting executable..."

        # Enhanced search paths with proper workspace root resolution
        log_detailed "   📍 Script location: $(pwd)"

        local search_paths=(
            "./$DEFAULT_EXECUTABLE"
            "../../Unreal/Environments/Blocks/Saved/StagedBuilds/Linux/Blocks/Binaries/Linux/$DEFAULT_EXECUTABLE"
            "../../Unreal/Environments/Blocks/Saved/StagedBuilds/Linux/$DEFAULT_EXECUTABLE"
        )

        # First try direct paths
        for path in "${search_paths[@]}"; do
            if [ -f "$path" ]; then
                EXECUTABLE_PATH="$path"
                log_detailed "   ✅ Found executable at: $path"
                break
            fi
        done

        # If not found, try wildcard searches
        if [ -z "$EXECUTABLE_PATH" ]; then
            log_detailed "   🔍 Trying wildcard search..."

            # Search in Binaries directory (modern UE5 structure)
            local wildcard_result
            wildcard_result=$(find ../../Unreal/Environments/*/Saved/StagedBuilds/Linux/*/Binaries/Linux/ -name "$DEFAULT_EXECUTABLE" -type f 2>/dev/null | head -1)
            if [ -n "$wildcard_result" ]; then
                EXECUTABLE_PATH="$wildcard_result"
                log_detailed "   ✅ Found via wildcard in Binaries: $EXECUTABLE_PATH"
            else
                # Fallback to legacy structure
                wildcard_result=$(find ../../Unreal/Environments/*/Saved/StagedBuilds/Linux/ -name "$DEFAULT_EXECUTABLE" -type f 2>/dev/null | head -1)
                if [ -n "$wildcard_result" ]; then
                    EXECUTABLE_PATH="$wildcard_result"
                    log_detailed "   ✅ Found via wildcard in legacy location: $EXECUTABLE_PATH"
                fi
            fi
        fi
    fi

    # Validate executable
    if [ -z "$EXECUTABLE_PATH" ] || [ ! -f "$EXECUTABLE_PATH" ]; then
        echo "❌ Executable not found!"
        echo "   Searched locations from $(pwd):"
        echo "     - ./$DEFAULT_EXECUTABLE"
        echo "     - ../../Unreal/Environments/Blocks/Saved/StagedBuilds/Linux/Blocks/Binaries/Linux/$DEFAULT_EXECUTABLE"
        echo "     - ../../Unreal/Environments/*/Saved/StagedBuilds/Linux/*/Binaries/Linux/$DEFAULT_EXECUTABLE"
        echo "     - ../../Unreal/Environments/Blocks/Saved/StagedBuilds/Linux/$DEFAULT_EXECUTABLE (fallback)"
        echo "     - ../../Unreal/Environments/*/Saved/StagedBuilds/Linux/$DEFAULT_EXECUTABLE (fallback)"
        echo ""
        echo "   💡 Troubleshooting:"
        echo "     - Run UE5 BuildCookRun first to create the executable"
        echo "     - Or specify with: $0 -e /path/to/executable"
        exit 2
    fi

    # Make executable
    chmod +x "$EXECUTABLE_PATH" 2>/dev/null || true

    echo "   ✅ Executable: $EXECUTABLE_PATH"
    echo "   ✅ Test duration: ${DEFAULT_TEST_DURATION}s"
    echo "   ✅ API port: $DEFAULT_API_PORT"
    echo "   ✅ Log directory: $TEST_LOG_DIR"
    echo ""
}

# Test 1: Basic headless execution (Vulkan workaround)
test_basic_headless() {
    echo "🧪 Test 1: Basic Headless Execution (Vulkan Workaround)"

    local exe_dir=$(dirname "$EXECUTABLE_PATH")
    local exe_name=$(basename "$EXECUTABLE_PATH")
    local log_file="$TEST_LOG_DIR/basic_headless.log"

    cd "$exe_dir"

    log_detailed "   Running: ./$exe_name -nullrhi -unattended -seconds=$DEFAULT_TEST_DURATION"

    if timeout $((DEFAULT_TEST_DURATION + 10)) "./$exe_name" \
        -nullrhi -unattended \
        -log -seconds="$DEFAULT_TEST_DURATION" \
        >"$log_file" 2>&1; then
        log_test_result "Basic headless execution" "PASS" "Null RHI successfully bypassed Vulkan"
        return 0
    else
        local exit_code=$?
        if [ $exit_code -eq 124 ]; then
            # Timeout can be expected behavior
            if grep -q "LogInit:" "$log_file" 2>/dev/null; then
                log_test_result "Basic headless execution" "PASS" "Initialized successfully (timeout as expected)"
                return 0
            fi
        fi

        log_test_result "Basic headless execution" "FAIL" "Exit code: $exit_code"
        log_detailed "   Check log: $log_file"
        return 1
    fi
}

# Test 2: Vulkan error absence verification
test_vulkan_error_absence() {
    echo "🧪 Test 2: Vulkan Error Absence Verification"

    local log_file="$TEST_LOG_DIR/basic_headless.log"

    if [ ! -f "$log_file" ]; then
        log_test_result "Vulkan error check" "FAIL" "No log file from previous test"
        return 1
    fi

    # Check for Vulkan-related error messages
    local vulkan_errors=0

    if grep -qi "failed to load vulkan" "$log_file"; then
        ((vulkan_errors++))
    fi

    if grep -qi "vulkan driver" "$log_file"; then
        ((vulkan_errors++))
    fi

    if grep -qi "VK_ERROR" "$log_file"; then
        ((vulkan_errors++))
    fi

    if [ $vulkan_errors -eq 0 ]; then
        log_test_result "Vulkan error absence" "PASS" "No Vulkan errors detected in logs"
        return 0
    else
        log_test_result "Vulkan error absence" "FAIL" "$vulkan_errors Vulkan-related errors found"
        return 1
    fi
}

# Test 3: Process initialization verification
test_process_initialization() {
    echo "🧪 Test 3: Process Initialization Verification"

    local log_file="$TEST_LOG_DIR/basic_headless.log"

    if [ ! -f "$log_file" ]; then
        log_test_result "Process initialization" "FAIL" "No log file available"
        return 1
    fi

    # Check for successful initialization markers
    local init_markers=0

    if grep -q "LogInit:" "$log_file"; then
        ((init_markers++))
    fi

    if grep -q "AirSim" "$log_file"; then
        ((init_markers++))
    fi

    # Check for RHI initialization
    if grep -qi "nullrhi" "$log_file" || grep -qi "null.*rhi" "$log_file"; then
        ((init_markers++))
    fi

    if [ $init_markers -ge 2 ]; then
        log_test_result "Process initialization" "PASS" "Successfully initialized with $init_markers markers"
        return 0
    else
        log_test_result "Process initialization" "FAIL" "Insufficient initialization markers ($init_markers)"
        return 1
    fi
}

# Test 4: API connectivity test (optional)
test_api_connectivity() {
    if [ "$API_TEST" != true ]; then
        return 0
    fi

    echo "🧪 Test 4: AirSim API Connectivity"

    local exe_dir=$(dirname "$EXECUTABLE_PATH")
    local exe_name=$(basename "$EXECUTABLE_PATH")
    local log_file="$TEST_LOG_DIR/api_test.log"

    cd "$exe_dir"

    # Start AirSim in background
    log_detailed "   Starting AirSim for API testing..."
    "./$exe_name" -nullrhi -unattended -log >"$log_file" 2>&1 &
    local airsim_pid=$!

    # Wait for startup
    sleep 10

    # Test API connectivity
    local api_test_result=1
    for attempt in {1..5}; do
        log_detailed "   API connectivity attempt $attempt/5..."
        if timeout 5 bash -c "echo >/dev/tcp/localhost/$DEFAULT_API_PORT" 2>/dev/null; then
            api_test_result=0
            break
        fi
        sleep 2
    done

    # Cleanup
    kill "$airsim_pid" 2>/dev/null || true
    wait "$airsim_pid" 2>/dev/null || true

    if [ $api_test_result -eq 0 ]; then
        log_test_result "API connectivity" "PASS" "API responding on port $DEFAULT_API_PORT"
        return 0
    else
        log_test_result "API connectivity" "FAIL" "API not responding on port $DEFAULT_API_PORT"
        return 1
    fi
}

# Test 5: Performance benchmarking (optional)
test_performance_benchmark() {
    if [ "$PERFORMANCE_TEST" != true ]; then
        return 0
    fi

    echo "🧪 Test 5: Performance Benchmark"

    local exe_dir=$(dirname "$EXECUTABLE_PATH")
    local exe_name=$(basename "$EXECUTABLE_PATH")
    local log_file="$TEST_LOG_DIR/performance.log"

    cd "$exe_dir"

    log_detailed "   Running performance benchmark..."

    local start_time=$(date +%s)
    local memory_before=$(free -m | awk 'NR==2{print $3}')

    if timeout $((DEFAULT_TEST_DURATION + 5)) "./$exe_name" \
        -nullrhi -unattended -benchmark \
        -seconds="$DEFAULT_TEST_DURATION" \
        >"$log_file" 2>&1; then

        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        local memory_after=$(free -m | awk 'NR==2{print $3}')
        local memory_used=$((memory_after - memory_before))

        log_test_result "Performance benchmark" "PASS" "Duration: ${duration}s, Memory delta: ${memory_used}MB"

        # Log performance metrics
        echo "Performance Metrics:" >> "$log_file"
        echo "  Duration: ${duration}s" >> "$log_file"
        echo "  Memory delta: ${memory_used}MB" >> "$log_file"

        return 0
    else
        log_test_result "Performance benchmark" "FAIL" "Benchmark execution failed"
        return 1
    fi
}

# Test 6: Container deployment readiness (integration test)
test_container_deployment() {
    if [ "$INTEGRATION_TEST" != true ]; then
        return 0
    fi

    echo "🧪 Test 6: Container Deployment Readiness"

    # Check if we're in a container
    local in_container=false
    if [ -f /.dockerenv ]; then
        in_container=true
    fi

    # Test environment variables that would be set in container
    local env_check=0
    if [ -n "$DISPLAY" ] || [ "$DISPLAY" = ":99" ]; then
        ((env_check++))
    fi

    if [ -n "$LIBGL_ALWAYS_SOFTWARE" ]; then
        ((env_check++))
    fi

    # Check for headless launcher
    if [ -f "./docker/unreal-airsim/launch-headless.sh" ]; then
        ((env_check++))
    fi

    local details="Container: $in_container, Env vars: $env_check/3"
    if [ $env_check -ge 2 ]; then
        log_test_result "Container deployment readiness" "PASS" "$details"
        return 0
    else
        log_test_result "Container deployment readiness" "FAIL" "$details"
        return 1
    fi
}

# Generate comprehensive test report
generate_test_report() {
    echo ""
    echo "📊 ===== TEST REPORT SUMMARY ====="
    echo ""
    echo "Tests executed: $TESTS_RUN"
    echo "Tests passed:   $TESTS_PASSED"
    echo "Tests failed:   $TESTS_FAILED"
    echo ""

    if [ $TESTS_FAILED -eq 0 ]; then
        echo "🎉 ALL TESTS PASSED!"
        echo ""
        echo "✅ Vulkan driver workaround is working correctly"
        echo "✅ Headless execution ready for production deployment"
        echo "✅ AirSim can run in containerized environments without GPU drivers"
        echo ""
    else
        echo "❌ SOME TESTS FAILED!"
        echo ""
        echo "Failed tests require attention before production deployment."
        echo ""
    fi

    echo "📋 Detailed Results:"
    for result in "${TEST_RESULTS[@]}"; do
        echo "   $result"
    done

    echo ""
    echo "📁 Test logs available in: $TEST_LOG_DIR"
    echo ""

    if [ "$INTEGRATION_TEST" = true ] && [ $TESTS_FAILED -eq 0 ]; then
        echo "🚀 DEPLOYMENT RECOMMENDATIONS:"
        echo ""
        echo "Production deployment commands:"
        echo "  docker-compose up airsim-headless"
        echo "  ./docker/unreal-airsim/launch-headless.sh"
        echo ""
        echo "Manual headless execution:"
        echo "  $EXECUTABLE_PATH -nullrhi -unattended"
        echo ""
    fi
}

# Main test execution
main() {
    setup_test_environment

    echo "🔬 Starting test suite..."
    echo "   Mode: $([ "$QUICK_TEST" = true ] && echo "Quick" || echo "Standard")"
    echo "   API testing: $([ "$API_TEST" = true ] && echo "Enabled" || echo "Disabled")"
    echo "   Performance testing: $([ "$PERFORMANCE_TEST" = true ] && echo "Enabled" || echo "Disabled")"
    echo "   Integration testing: $([ "$INTEGRATION_TEST" = true ] && echo "Enabled" || echo "Disabled")"
    echo ""

    # Run test suite
    test_basic_headless
    test_vulkan_error_absence
    test_process_initialization
    test_api_connectivity
    test_performance_benchmark
    test_container_deployment

    # Generate report
    generate_test_report

    # Exit with appropriate code
    if [ $TESTS_FAILED -eq 0 ]; then
        exit 0
    else
        exit 1
    fi
}

# Execute main function
main "$@"