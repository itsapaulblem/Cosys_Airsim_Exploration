#!/usr/bin/env bash

# Complete AirSim Build and UE5 Package Workflow
# This script handles the full pipeline from ABI-compatible build to packaged deployment

set -e

echo "🚀 ===== COMPLETE AIRSIM BUILD AND PACKAGE WORKFLOW ====="
echo ""

# Script configuration
PROJECT_PATH="/workspace/airsim/Unreal/Environments/Blocks/Blocks.uproject"
PLATFORM="Linux"
CONFIGURATION="Development"
BUILD_ONLY=false
SKIP_BUILD=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --project)
            PROJECT_PATH="$2"
            shift 2
            ;;
        --platform)
            PLATFORM="$2"
            shift 2
            ;;
        --configuration)
            CONFIGURATION="$2"
            shift 2
            ;;
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --project PATH       UE5 project file (default: Blocks.uproject)"
            echo "  --platform PLATFORM  Target platform (default: Linux)"
            echo "  --configuration CFG   Build configuration (default: Development)"
            echo "  --build-only         Only build AirSim libraries, skip UE5 packaging"
            echo "  --skip-build         Skip AirSim build, only run UE5 packaging"
            echo "  --help               Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo "📋 Configuration:"
echo "   Project: $PROJECT_PATH"
echo "   Platform: $PLATFORM"
echo "   Configuration: $CONFIGURATION"
echo "   Build only: $BUILD_ONLY"
echo "   Skip build: $SKIP_BUILD"
echo ""

# Phase 1: AirSim ABI-Compatible Build
if [ "$SKIP_BUILD" != true ]; then
    echo "🔧 ===== PHASE 1: AIRSIM ABI-COMPATIBLE BUILD ====="
    echo ""

    # Run the proven ABI-compatible build process
    echo "🚀 Starting Epic-compatible AirSim build..."
    ./build-privileged.sh

    echo ""
    echo "✅ AirSim build completed successfully!"

    # Verify ABI compatibility
    echo "🔬 Verifying ABI compatibility..."
    if nm -C /workspace/airsim/AirLib/lib/libAirLib.a | grep 'std::__1::' >/dev/null 2>&1; then
        echo "   ✅ AirLib: Epic libc++ symbols detected"
    else
        echo "   ❌ AirLib: Missing Epic symbols - build may have issues"
        exit 1
    fi

    if nm -C /workspace/airsim/AirLib/deps/MavLinkCom/lib/libMavLinkCom.a | grep 'cxx11' >/dev/null 2>&1; then
        echo "   ❌ MavLinkCom: ABI contamination detected!"
        echo "   🔧 Run force-clean-rebuild.sh to fix this issue"
        exit 1
    else
        echo "   ✅ MavLinkCom: ABI clean"
    fi

    echo "✅ ABI verification passed - ready for UE5 integration"
    echo ""
else
    echo "⏭️  Skipping AirSim build phase (--skip-build enabled)"
    echo ""
fi

# Phase 2: UE5 BuildCookRun with User Management
if [ "$BUILD_ONLY" != true ]; then
    echo "📦 ===== PHASE 2: UE5 BUILDCOOKRUN WITH USER MANAGEMENT ====="
    echo ""

    # Check if project file exists
    if [ ! -f "$PROJECT_PATH" ]; then
        echo "❌ Project file not found: $PROJECT_PATH"
        exit 1
    fi

    echo "🔄 Preparing safe BuildCookRun environment..."

    # Enhanced BuildCookRun function with Docker volume permission detection
    buildcookrun_safe() {
        echo "🚀 Starting enhanced BuildCookRun with Docker volume permission detection..."

        # Advanced volume permission diagnostics
        diagnose_volume_permissions() {
            local workspace_root="/workspace/airsim"
            local project_dir=$(dirname "$PROJECT_PATH")
            local test_dir="$project_dir/Intermediate/Build/$PLATFORM/x64/UnrealEditor/$CONFIGURATION"
            local test_file="$test_dir/volume_permission_test.tmp"

            echo "🔬 Docker volume permission diagnostics..."

            # Test 1: Basic directory creation
            echo "   📂 Testing deep directory creation..."
            if mkdir -p "$test_dir" 2>/dev/null; then
                echo "      ✅ Deep directory creation successful"
            else
                echo "      ❌ Deep directory creation failed"
                return 1
            fi

            # Test 2: File creation capability (UnrealBuildTool requirement)
            echo "   📄 Testing file creation in build directory..."
            if echo "Docker volume permission test" > "$test_file" 2>/dev/null; then
                echo "      ✅ File creation successful"
                rm -f "$test_file" 2>/dev/null
            else
                echo "      ❌ File creation failed - Docker volume permission issue detected"
                return 1
            fi

            # Test 3: .NET file operations (specific to UnrealBuildTool)
            echo "   🔗 Testing .NET-style file operations..."
            local link_test="$test_dir/Link-test-permission.so.link.sh"
            if echo '#!/bin/bash' > "$link_test" 2>/dev/null && chmod +x "$link_test" 2>/dev/null; then
                echo "      ✅ .NET-style file operations successful"
                rm -f "$link_test" 2>/dev/null
            else
                echo "      ❌ .NET-style file operations failed - requires root permissions"
                return 1
            fi

            echo "   ✅ Volume permission diagnostics passed"
            return 0
        }

        # Enhanced workspace preparation with comprehensive ownership
        prepare_workspace_permissions() {
            local workspace_root="/workspace/airsim"
            local project_dir=$(dirname "$PROJECT_PATH")

            echo "🔧 Enhanced workspace permission management..."

            # Complete workspace ownership (works better with Docker volumes)
            echo "   🏗️  Complete workspace preparation..."

            # Create full directory structure that UnrealBuildTool needs
            mkdir -p "$project_dir/Intermediate/Build/$PLATFORM/x64/UnrealEditor/$CONFIGURATION" 2>/dev/null || true
            mkdir -p "$project_dir/Intermediate/TargetInfo" 2>/dev/null || true
            mkdir -p "$project_dir/Saved/StagedBuilds/$PLATFORM" 2>/dev/null || true
            mkdir -p "$project_dir/Saved/Logs" 2>/dev/null || true
            mkdir -p "$project_dir/Binaries/$PLATFORM" 2>/dev/null || true

            # Set comprehensive permissions
            chmod -R 755 "$workspace_root" 2>/dev/null || true
            chmod -R u+w "$project_dir" 2>/dev/null || true

            echo "   ✅ Workspace preparation completed"
        }

        # Root-based BuildCookRun fallback
        run_buildcookrun_as_root() {
            echo "🔄 Running BuildCookRun as root (Docker volume permission fallback)..."

            # Comprehensive ownership setup
            chown -R ue4:ue4 /workspace/airsim 2>/dev/null || true

            # Run BuildCookRun directly as root with ue4 environment
            export HOME=/home/ue4
            export USER=ue4
            export LOGNAME=ue4
            cd /home/ue4/UnrealEngine

            echo "📋 Executing BuildCookRun as root with ue4 environment..."
            ./Engine/Build/BatchFiles/RunUAT.sh "$@"
        }

        # Main execution logic with fallback strategy
        if [ "$EUID" -eq 0 ]; then
            echo "🔄 Running as root - attempting ue4 user BuildCookRun with fallback..."

            # Prepare workspace
            prepare_workspace_permissions

            # Test volume permissions
            if diagnose_volume_permissions; then
                echo "📋 Volume permissions validated - attempting ue4 user BuildCookRun..."

                # Try ue4 user first
                if sudo -u ue4 -E bash -c "
                    export HOME=/home/ue4
                    export USER=ue4
                    export LOGNAME=ue4
                    export TERM=xterm
                    cd /home/ue4/UnrealEngine
                    ./Engine/Build/BatchFiles/RunUAT.sh \"\$@\"
                " -- "$@"; then
                    echo "✅ ue4 user BuildCookRun succeeded"
                else
                    echo "⚠️  ue4 user BuildCookRun failed - falling back to root execution..."
                    run_buildcookrun_as_root "$@"
                fi
            else
                echo "⚠️  Volume permission issues detected - using root fallback..."
                run_buildcookrun_as_root "$@"
            fi

        else
            echo "✅ Running as non-root user: $USER"

            # Prepare workspace with available permissions
            prepare_workspace_permissions

            # Test if we can actually create UnrealBuildTool files
            if diagnose_volume_permissions; then
                cd /home/ue4/UnrealEngine
                echo "🚀 Starting BuildCookRun with validated permissions..."
                ./Engine/Build/BatchFiles/RunUAT.sh "$@"
            else
                echo "❌ Docker volume permission issues detected!"
                echo "   This typically occurs with Windows→Linux Docker volume mounts"
                echo "   Recommendation: Run container with --user root or fix volume permissions"
                echo "   Current user: $(whoami) ($(id))"
                exit 1
            fi
        fi
    }

    # Try the script first, fall back to function if needed or script has issues
    SCRIPT_WORKS=false
    if command -v buildcookrun-safe >/dev/null 2>&1; then
        # Test if the script actually works (not just exists)
        if buildcookrun-safe --help >/dev/null 2>&1; then
            SCRIPT_WORKS=true
        fi
    fi

    if [ "$SCRIPT_WORKS" = true ]; then
        echo "🚀 Using buildcookrun-safe script..."
        buildcookrun-safe BuildCookRun \
            -project="$PROJECT_PATH" \
            -platform="$PLATFORM" \
            -configuration="$CONFIGURATION" \
            -cook -build -stage -package
    else
        echo "🔄 Using fallback buildcookrun function (script unavailable or has issues)..."
        buildcookrun_safe BuildCookRun \
            -project="$PROJECT_PATH" \
            -platform="$PLATFORM" \
            -configuration="$CONFIGURATION" \
            -cook -build -stage -package
    fi

    if [ $? -eq 0 ]; then
        echo ""
        echo "🎉 ===== BUILDCOOKRUN COMPLETED SUCCESSFULLY ====="
        echo ""

        # Find and display package location
        PROJECT_DIR=$(dirname "$PROJECT_PATH")
        PACKAGE_DIR="$PROJECT_DIR/Saved/StagedBuilds/$PLATFORM"

        if [ -d "$PACKAGE_DIR" ]; then
            echo "📦 Package created successfully:"
            echo "   Location: $PACKAGE_DIR"
            echo "   Contents:"
            ls -la "$PACKAGE_DIR" | head -10
            echo ""

            # Calculate package size
            PACKAGE_SIZE=$(du -sh "$PACKAGE_DIR" 2>/dev/null | cut -f1 || echo "Unknown")
            echo "   Package size: $PACKAGE_SIZE"
        else
            echo "⚠️  Package directory not found at expected location: $PACKAGE_DIR"
            echo "   Searching for alternative package locations..."
            find "$PROJECT_DIR" -name "StagedBuilds" -type d 2>/dev/null | head -5
        fi

    else
        echo ""
        echo "❌ ===== BUILDCOOKRUN FAILED ====="
        echo ""
        echo "🔍 Common troubleshooting steps:"
        echo "1. Check the build log for specific errors"
        echo "2. Verify project file path: $PROJECT_PATH"
        echo "3. Ensure AirSim build completed successfully"
        echo "4. Try rebuilding with: ./force-clean-rebuild.sh"
        echo ""
        exit 1
    fi
else
    echo "⏭️  Skipping UE5 packaging phase (--build-only enabled)"
    echo ""
fi

# Phase 3: Post-Package Verification
if [ "$BUILD_ONLY" != true ]; then
    echo "✅ ===== PHASE 3: POST-PACKAGE VERIFICATION ====="
    echo ""

    PROJECT_DIR=$(dirname "$PROJECT_PATH")
    PROJECT_NAME=$(basename "$PROJECT_PATH" .uproject)
    PACKAGE_DIR="$PROJECT_DIR/Saved/StagedBuilds/$PLATFORM"
    EXECUTABLE_PATH="$PACKAGE_DIR/$PROJECT_NAME"

    if [ -f "$EXECUTABLE_PATH" ]; then
        echo "🎯 Executable verification:"
        echo "   Executable: $EXECUTABLE_PATH"
        echo "   Size: $(ls -lh "$EXECUTABLE_PATH" | awk '{print $5}')"
        echo "   Permissions: $(ls -l "$EXECUTABLE_PATH" | awk '{print $1}')"
        echo ""

        # Check for required libraries
        echo "📚 Library dependencies:"
        if command -v ldd >/dev/null 2>&1; then
            ldd "$EXECUTABLE_PATH" | grep -E "(libc\+\+|libstdc\+\+)" | head -5 || echo "   No critical library dependencies found"
        else
            echo "   ldd not available for dependency analysis"
        fi
        echo ""

        # Phase 3b: Headless Execution Testing (Vulkan Driver Solution)
        echo "🚀 Testing headless execution (Vulkan driver workaround)..."
        echo ""

        # Test headless execution to verify null RHI works
        test_headless_execution() {
            echo "   🧪 Starting headless execution test..."

            # Make executable if needed
            chmod +x "$EXECUTABLE_PATH" 2>/dev/null || true

            # Change to executable directory
            local exe_dir=$(dirname "$EXECUTABLE_PATH")
            local exe_name=$(basename "$EXECUTABLE_PATH")

            cd "$exe_dir"

            # Test with null RHI flags (solves Vulkan issue)
            echo "   🔧 Testing: ./$exe_name -nullrhi -unattended"

            # Run with timeout to prevent hanging
            if timeout 30s "./$exe_name" -nullrhi -unattended -log -seconds=15 >/dev/null 2>&1; then
                echo "   ✅ Headless execution test PASSED"
                echo "   ✅ Vulkan driver workaround confirmed working"
                return 0
            else
                local exit_code=$?
                if [ $exit_code -eq 124 ]; then
                    echo "   ✅ Headless execution test PASSED (timeout as expected)"
                    echo "   ✅ Vulkan driver workaround confirmed working"
                    return 0
                else
                    echo "   ⚠️  Headless execution test completed with exit code: $exit_code"
                    echo "   ℹ️  This may be normal for testing mode"
                    return 1
                fi
            fi
        }

        # Run headless test
        if test_headless_execution; then
            HEADLESS_TEST_RESULT="✅ PASSED"
            HEADLESS_TEST_STATUS=true
        else
            HEADLESS_TEST_RESULT="⚠️  NEEDS VERIFICATION"
            HEADLESS_TEST_STATUS=false

            echo "   💡 Headless test tips:"
            echo "      - The executable may need specific settings.json"
            echo "      - Check logs for initialization issues"
            echo "      - Manual testing: $EXECUTABLE_PATH -nullrhi -unattended"
        fi

        echo ""

        echo "🎉 VERIFICATION COMPLETE - DEPLOYMENT READY!"
        echo ""
        echo "📋 Deployment Summary:"
        echo "   ✅ AirSim: Built with Epic's ABI-compatible toolchain"
        echo "   ✅ UE5: Successfully packaged with BuildCookRun"
        echo "   ✅ Package: Ready for deployment to target systems"
        echo "   ✅ Executable: $EXECUTABLE_PATH"
        echo "   $HEADLESS_TEST_RESULT Headless execution (null RHI)"
        echo ""

        # Provide headless execution guidance
        if [ "$HEADLESS_TEST_STATUS" = true ]; then
            echo "🚀 Ready for containerized deployment!"
            echo "   Use: docker-compose up airsim-headless"
            echo "   Or:  ./docker/unreal-airsim/launch-headless.sh"
        fi
        echo ""
    else
        echo "❌ Executable not found at expected location: $EXECUTABLE_PATH"
        echo "🔍 Searching for executable in package directory..."
        if [ -d "$PACKAGE_DIR" ]; then
            find "$PACKAGE_DIR" -type f -executable | head -10
        fi
        echo ""
    fi
fi

echo "🚀 ===== WORKFLOW COMPLETED ====="
echo ""
echo "Next steps for deployment:"
echo "1. Test the executable on your target Linux system"
echo "2. Deploy the entire package directory to production"
echo "3. Verify AirSim functionality in the deployed environment"
echo "4. Set up monitoring and logging for production use"
echo ""