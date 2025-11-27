#!/bin/bash
# Installation script for MNSTEVV CLI on Linux/macOS

echo
echo "============================================"
echo "  MNSTEVV CLI Installation"
echo "============================================"
echo

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    if ! command -v python &> /dev/null; then
        echo "ERROR: Python is not installed or not in PATH"
        echo "Please install Python 3.7+ and try again"
        exit 1
    else
        PYTHON_CMD="python"
    fi
else
    PYTHON_CMD="python3"
fi

echo "Python detected:"
$PYTHON_CMD --version

# Check if pip is available  
if ! command -v pip3 &> /dev/null; then
    if ! command -v pip &> /dev/null; then
        echo "ERROR: pip is not installed or not in PATH"
        exit 1
    else
        PIP_CMD="pip"
    fi
else
    PIP_CMD="pip3"
fi

echo
echo "Installing dependencies..."

# Try to install, handling externally managed environment
if $PIP_CMD install -r requirements.txt 2>&1 | grep -q "externally-managed-environment"; then
    echo "Detected externally managed environment, using --break-system-packages flag"
    $PIP_CMD install --break-system-packages -r requirements.txt

    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to install dependencies"
        exit 1
    fi
else
    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to install dependencies"
        exit 1
    fi
fi

echo
echo "Installing MNSTEVV CLI..."

# Try to install, handling externally managed environment
if $PIP_CMD install -e . 2>&1 | grep -q "externally-managed-environment"; then
    echo "Detected externally managed environment, using --break-system-packages flag"
    $PIP_CMD install --break-system-packages -e .

    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to install MNSTEVV CLI"
        exit 1
    fi
else
    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to install MNSTEVV CLI"
        exit 1
    fi
fi

echo
echo "============================================"
echo "  Installation completed successfully!"
echo "============================================"
echo

echo "Quick test:"
mnstevv --version

echo
echo "Usage examples:"
echo "  mnstevv up --num_drones 3         # Start 3 drones"
echo "  mnstevv status                    # Check status"
echo "  mnstevv --help                    # Show all commands"
echo