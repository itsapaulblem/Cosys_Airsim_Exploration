#!/bin/bash
# Fix NumPy headers for ROS2 interface generation
# This script resolves the persistent numpy/ndarrayobject.h compilation errors

set -e

echo "========================================"
echo "NumPy Headers Fix for ROS2 Interfaces"
echo "========================================"

# Function to find NumPy include path
find_numpy_path() {
    python3 -c "import numpy; print(numpy.get_include())" 2>/dev/null || echo ""
}

# Function to check if header exists
check_header() {
    local path="$1/numpy/ndarrayobject.h"
    [ -f "$path" ] && echo "$path" || echo ""
}

# Get NumPy include path
NUMPY_PATH=$(find_numpy_path)
echo "NumPy include path: $NUMPY_PATH"

if [ -z "$NUMPY_PATH" ]; then
    echo "❌ ERROR: Could not determine NumPy include path"
    echo "Installing NumPy..."
    python3 -m pip install numpy
    NUMPY_PATH=$(find_numpy_path)
fi

# Check if NumPy headers exist
HEADER_PATH=$(check_header "$NUMPY_PATH")
echo "NumPy header path: $HEADER_PATH"

if [ -z "$HEADER_PATH" ]; then
    echo "❌ ERROR: NumPy headers not found at expected location"
    echo "Installing development packages..."
    
    # Install system NumPy development packages
    apt-get update -q
    apt-get install -y python3-dev python3-numpy-dev libpython3-dev
    
    # Try to find headers again
    NUMPY_PATH=$(find_numpy_path)
    HEADER_PATH=$(check_header "$NUMPY_PATH")
fi

# Create standard include directories
echo "📁 Creating standard include directories..."
mkdir -p /usr/include/numpy /usr/local/include/numpy

# Link NumPy headers to standard locations
if [ -n "$HEADER_PATH" ] && [ -f "$HEADER_PATH" ]; then
    echo "🔗 Linking NumPy headers to system locations..."
    
    # Link to /usr/include/numpy
    ln -sf "$NUMPY_PATH/numpy"/* /usr/include/numpy/ 2>/dev/null || true
    
    # Link to /usr/local/include/numpy  
    ln -sf "$NUMPY_PATH/numpy"/* /usr/local/include/numpy/ 2>/dev/null || true
    
    echo "✅ NumPy headers linked successfully"
else
    echo "❌ ERROR: Could not find NumPy headers to link"
    exit 1
fi

# Verify headers are accessible
echo "🔍 Verifying header accessibility..."
VERIFICATION_PATHS=(
    "/usr/include/numpy/ndarrayobject.h"
    "/usr/local/include/numpy/ndarrayobject.h"
    "$NUMPY_PATH/numpy/ndarrayobject.h"
)

FOUND_HEADER=""
for path in "${VERIFICATION_PATHS[@]}"; do
    if [ -f "$path" ]; then
        echo "✅ Found: $path"
        FOUND_HEADER="$path"
        break
    else
        echo "❌ Not found: $path"
    fi
done

if [ -z "$FOUND_HEADER" ]; then
    echo "❌ CRITICAL ERROR: NumPy headers still not accessible"
    exit 1
fi

# Set up environment variables for current session
echo "🔧 Setting up environment variables..."
export CPLUS_INCLUDE_PATH="/usr/include:/usr/local/include:/usr/include/python3.10:$CPLUS_INCLUDE_PATH"
export C_INCLUDE_PATH="/usr/include:/usr/local/include:/usr/include/python3.10:$C_INCLUDE_PATH"
export CMAKE_CXX_FLAGS="-I/usr/include -I/usr/local/include -I/usr/include/python3.10"
export CMAKE_C_FLAGS="-I/usr/include -I/usr/local/include -I/usr/include/python3.10"

# Write environment variables to bashrc for persistence
echo "💾 Making environment variables persistent..."
cat >> /home/Aortz/.bashrc << 'EOF'

# NumPy headers for ROS2 interface generation
export CPLUS_INCLUDE_PATH="/usr/include:/usr/local/include:/usr/include/python3.10:$CPLUS_INCLUDE_PATH"
export C_INCLUDE_PATH="/usr/include:/usr/local/include:/usr/include/python3.10:$C_INCLUDE_PATH"  
export CMAKE_CXX_FLAGS="-I/usr/include -I/usr/local/include -I/usr/include/python3.10"
export CMAKE_C_FLAGS="-I/usr/include -I/usr/local/include -I/usr/include/python3.10"
EOF

echo "✅ NumPy headers fix completed successfully!"
echo
echo "🚀 Next steps:"
echo "   1. Clean build directory: rm -rf build install log"
echo "   2. Source ROS2 environment: source /opt/ros/humble/setup.bash"
echo "   3. Rebuild interfaces: colcon build --packages-select airsim_interfaces mission_search_interfaces"
echo
echo "========================================"