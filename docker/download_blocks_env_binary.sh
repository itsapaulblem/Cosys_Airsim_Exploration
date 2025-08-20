#!/bin/bash

# Download AirSim Blocks Environment Binary for Docker
echo "🚀 Downloading AirSim Blocks Environment Binary"
echo "================================================"

BLOCKS_DIR="./blocks_env"
BLOCKS_URL="https://github.com/Cosys-Lab/Cosys-AirSim/releases/download/v1.8.1/LinuxBlocks.tar.gz"

# Create directory if it doesn't exist
mkdir -p $BLOCKS_DIR

# Check if already downloaded
if [ -d "$BLOCKS_DIR/LinuxBlocks" ]; then
    echo "✅ Blocks environment already exists at $BLOCKS_DIR"
    echo "💡 To re-download, remove the directory: rm -rf $BLOCKS_DIR"
    exit 0
fi

echo "📥 Downloading Blocks environment..."
cd $BLOCKS_DIR

# Download the binary
wget -O LinuxBlocks.tar.gz $BLOCKS_URL

if [ $? -eq 0 ]; then
    echo "✅ Download completed successfully"
    
    echo "📦 Extracting archive..."
    tar -xzf LinuxBlocks.tar.gz
    
    if [ $? -eq 0 ]; then
        echo "✅ Extraction completed"
        
        # Make the binary executable
        chmod +x LinuxBlocks/Linux/Blocks.sh
        
        # Clean up
        rm LinuxBlocks.tar.gz
        
        echo "🎉 Blocks environment ready!"
        echo "📁 Location: $BLOCKS_DIR/LinuxBlocks"
        echo "🚀 Run script: $BLOCKS_DIR/LinuxBlocks/Linux/Blocks.sh"
    else
        echo "❌ Failed to extract archive"
        exit 1
    fi
else
    echo "❌ Failed to download Blocks environment"
    echo "🔗 URL: $BLOCKS_URL"
    exit 1
fi

echo ""
echo "🐳 Next steps:"
echo "1. Build AirSim Docker image: docker-compose -f docker-compose-full-stack.yml build"
echo "2. Start full stack: docker-compose -f docker-compose-full-stack.yml --profile full-stack up" 