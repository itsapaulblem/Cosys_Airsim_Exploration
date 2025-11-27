#!/bin/bash
# DeepSORT Checkpoint Download Helper Script
#
# This script helps you download and install the DeepSORT checkpoint file
# required for YOLOv7+DeepSORT object tracking.

set -e

CHECKPOINT_DIR="$(dirname "$0")/YOLOv7-DeepSORT-Object-Tracking/deep_sort_pytorch/deep_sort/deep/checkpoint"
CHECKPOINT_FILE="$CHECKPOINT_DIR/ckpt.t7"

echo "========================================"
echo "DeepSORT Checkpoint Download Helper"
echo "========================================"
echo ""

# Check if checkpoint already exists
if [ -f "$CHECKPOINT_FILE" ]; then
    echo "✅ Checkpoint file already exists at: $CHECKPOINT_FILE"
    echo "   File size: $(du -h "$CHECKPOINT_FILE" | cut -f1)"
    echo ""
    echo "If you want to re-download, delete this file first."
    exit 0
fi

echo "The DeepSORT checkpoint file (ckpt.t7) is required for person tracking."
echo "This file must be downloaded manually from Google Drive."
echo ""
echo "📥 Download Instructions:"
echo "   1. Open this link in your browser:"
echo "      https://drive.google.com/drive/folders/1kna8eWGrSfzaR6DtNJ8_GchGgPMv3VC8"
echo ""
echo "   2. Look for 'ckpt.t7' or a zip file containing it"
echo "      (May be in a subfolder or inside 'deep_sort_pytorch.zip')"
echo ""
echo "   3. Download the file to your Downloads folder"
echo ""
echo "   4. Run ONE of these commands based on what you downloaded:"
echo ""
echo "      # If you downloaded ckpt.t7 directly:"
echo "      cp ~/Downloads/ckpt.t7 \"$CHECKPOINT_FILE\""
echo ""
echo "      # If you downloaded deep_sort_pytorch.zip:"
echo "      unzip ~/Downloads/deep_sort_pytorch.zip -d /tmp/"
echo "      cp /tmp/deep_sort_pytorch/deep_sort/deep/checkpoint/ckpt.t7 \"$CHECKPOINT_FILE\""
echo ""
echo "   5. Re-run this script to verify the installation"
echo ""
echo "📍 Target location:"
echo "   $CHECKPOINT_FILE"
echo ""
echo "Alternative: Direct download with file ID (if known):"
echo "   wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=FILE_ID' -O \"$CHECKPOINT_FILE\""
echo ""
echo "========================================"
