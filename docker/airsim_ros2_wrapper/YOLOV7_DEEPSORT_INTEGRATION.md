# YOLOv7+DeepSORT Integration for AirSim ROS2

## Overview

This document describes the complete integration of YOLOv7 object detection and DeepSORT multi-object tracking into the AirSim ROS2 Docker environment for 4-camera 360° drone person tracking.

## What Was Integrated

### 1. YOLOv7 Object Detection
- **Model**: yolov7.pt (73MB)
- **Source**: https://github.com/WongKinYiu/yolov7
- **Auto-downloaded** during Docker build from GitHub releases
- Provides real-time object detection for person tracking

### 2. DeepSORT Multi-Object Tracking
- **Library**: deep_sort_pytorch from https://github.com/ZQPei/deep_sort_pytorch
- **Auto-cloned** during Docker build
- Provides persistent ID tracking across frames
- **Manual download required**: checkpoint file `ckpt.t7` (see below)

### 3. Python AI Stack
All required dependencies automatically installed:
- PyTorch 2.9.0+cpu (CPU-optimized for Docker)
- NumPy, SciPy, Pandas, Matplotlib, Seaborn
- OpenCV, Pillow, PyYAML
- TensorBoard, tqdm, psutil, ipython
- **easydict** (for DeepSORT config parsing)
- thop (for FLOPs computation)

## Dockerfile.x11 Changes

### Line 208: Added `easydict` dependency
```dockerfile
'tensorboard>=2.4.1' 'ipython' 'psutil' 'easydict' && \
```

### Lines 100-131: Complete YOLOv7+DeepSORT setup
```dockerfile
# Download YOLOv7 weights and apply necessary fixes
RUN cd ./src/airsim_ros_pkgs/scripts/YOLOv7-DeepSORT-Object-Tracking && \
    # Download YOLOv7 weights (73MB)
    wget -q --show-progress https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7.pt && \

    # FIX: google_utils.py path bug
    # Original bug: lowercased entire file path, breaking directory names
    # Fixed: only strip/clean path, don't lowercase
    sed -i "s/file = Path(str(file).strip().replace(\"'\", '').lower())/file = Path(str(file).strip().replace(\"'\", ''))/" ./utils/google_utils.py && \

    # Initialize git repository (YOLOv7 checks git tags for version)
    git init && \
    git config user.email "docker@airsim.local" && \
    git config user.name "Docker Build" && \
    git add . && \
    git commit -m "YOLOv7+DeepSORT for AirSim ROS2" && \
    git tag v1.0
```

## Critical Bug Fixed

### google_utils.py Path Handling Bug

**Location**: `utils/google_utils.py:21`

**Original Code** (BROKEN):
```python
file = Path(str(file).strip().replace("'", '').lower())
```

**Problem**: Lowercased the entire file path, converting:
- `YOLOv7-DeepSORT-Object-Tracking/yolov7.pt` → `yolov7-deepsort-object-tracking/yolov7.pt`
- This caused file existence checks to fail
- YOLOv7 would attempt to download weights and run `git tag` commands
- Git commands would fail because directory structure was wrong

**Fixed Code**:
```python
file = Path(str(file).strip().replace("'", ''))  # FIX: Don't lowercase path
```

**Result**: File path preserved correctly, YOLOv7 loads weights successfully

## Manual Download Required

### DeepSORT Checkpoint File

The DeepSORT checkpoint file (`ckpt.t7`) **cannot be auto-downloaded** due to Google Drive restrictions.

**Download Instructions**:
1. Visit: https://drive.google.com/drive/folders/1kna8eWGrSfzaR6DtNJ8_GchGgPMv3VC8
2. Download `ckpt.t7` or the zip containing it
3. Run the helper script:
   ```bash
   cd /home/mnsuser/PaulSim/Cosys_Airsim_Exploration/ros2/src/airsim_ros_pkgs/scripts
   ./download_deepsort_checkpoint.sh
   ```
4. Follow the on-screen instructions to copy the file

**Target Location**:
```
ros2/src/airsim_ros_pkgs/scripts/YOLOv7-DeepSORT-Object-Tracking/
  └── deep_sort_pytorch/
      └── deep_sort/
          └── deep/
              └── checkpoint/
                  └── ckpt.t7  ← Place file here
```

**Without this file**: Motion detection will work with "YOLOv7 only" mode (detection but no tracking)
**With this file**: Full "YOLOv7+DeepSORT" mode (detection + persistent tracking IDs)

## Building the Docker Image

### Option 1: Convenience Script (Recommended)
```bash
cd /home/mnsuser/PaulSim/Cosys_Airsim_Exploration
./airsim_ros2_docker.bat build
```

### Option 2: Direct Docker Compose
```bash
cd docker/airsim_ros2_wrapper/Linux
docker-compose -f docker-compose.x11.yml build
```

**Build Time**: ~15-20 minutes (includes downloading 73MB YOLOv7 weights)
**Final Image Size**: ~5.7GB

## Verification

### 1. Check YOLOv7 Weights
```bash
docker exec ros2-x11-node ls -lh /airsim_ros2_ws/src/airsim_ros_pkgs/scripts/YOLOv7-DeepSORT-Object-Tracking/*.pt
```
**Expected**: `yolov7.pt` (73MB)

### 2. Check DeepSORT Checkpoint
```bash
docker exec ros2-x11-node ls -lh /airsim_ros2_ws/src/airsim_ros_pkgs/scripts/YOLOv7-DeepSORT-Object-Tracking/deep_sort_pytorch/deep_sort/deep/checkpoint/ckpt.t7
```
**Expected**: `ckpt.t7` (if manually downloaded)

### 3. Check Python Dependencies
```bash
docker exec ros2-x11-node python3 -c "import torch, easydict; print(f'PyTorch: {torch.__version__}'); print('easydict: OK')"
```
**Expected**:
```
PyTorch: 2.9.0+cpu
easydict: OK
```

### 4. Check Git Repository
```bash
docker exec ros2-x11-node bash -c "cd /airsim_ros2_ws/src/airsim_ros_pkgs/scripts/YOLOv7-DeepSORT-Object-Tracking && git tag"
```
**Expected**: `v1.0`

## Running Motion Detection

### Launch Motion Detection Node
```bash
docker exec ros2-x11-node bash -c "source /opt/ros/humble/setup.bash && source /airsim_ros2_ws/install/setup.bash && python3 /airsim_ros2_ws/src/airsim_ros_pkgs/scripts/motion_detection_node.py"
```

### Expected Output (YOLOv7 Only - without ckpt.t7)
```
[INFO] [DEBUG] YOLOv7 model loaded successfully
[WARN] [DEBUG] Using YOLOv7 only
[INFO]  YOLOv7+DeepSORT: disabled
```

### Expected Output (Full YOLOv7+DeepSORT - with ckpt.t7)
```
[INFO] [DEBUG] YOLOv7 model loaded successfully
[INFO] [DEBUG] YOLOv7 + DeepSORT initialized successfully
[INFO]  YOLOv7+DeepSORT: enabled
```

## Architecture Details

### Motion Detection Node
- **Location**: `ros2/src/airsim_ros_pkgs/scripts/motion_detection_node.py`
- **Cameras**: 4-camera 360° coverage (front, right, back, left)
- **Topics**:
  - Input: `/drone1/camera{0-3}/image`
  - Output: `/detection_visualization_cam{0-3}`
  - Target: `/target_detection`
- **Features**:
  - Real-time person detection with YOLOv7
  - Persistent ID tracking with DeepSORT
  - Automatic camera switching for target following
  - OpenCV fallback if AI stack unavailable

### Configuration
- **Settings**: `/home/mnsuser/Documents/AirSim/settings.json`
- **4 Cameras**:
  - Camera_0 (front): Yaw 0°
  - Camera_1 (right): Yaw 90°
  - Camera_2 (back): Yaw 180°
  - Camera_3 (left): Yaw -90°

## Troubleshooting

### Issue: "YOLOv7+DeepSORT: disabled"
**Cause**: Missing `ckpt.t7` checkpoint file
**Solution**: Download from Google Drive (see Manual Download section)

### Issue: "ModuleNotFoundError: No module named 'easydict'"
**Cause**: Old Docker image built before easydict was added
**Solution**: Rebuild Docker image with updated Dockerfile

### Issue: "fatal: not a git repository"
**Cause**: Git repository not initialized in YOLOv7 directory
**Solution**: Rebuild Docker image (git init is now automated)

### Issue: ImportError for torch, YOLOv7 models
**Cause**: Volume mount issues or missing Python packages
**Solution**: Rebuild Docker image, check PyTorch installation

### Issue: NumPy/SciPy binary incompatibility
**Cause**: Version mismatch between cv_bridge and scipy
**Solution**: Already fixed in Dockerfile with proper version constraints

## Performance

### YOLOv7 Inference (CPU)
- **Model**: yolov7.pt
- **Device**: CPU (torch.device('cpu'))
- **Input Size**: 640x640
- **Speed**: ~100-200ms per frame (CPU-dependent)

### DeepSORT Tracking
- **Feature Extractor**: ResNet18 (ckpt.t7)
- **Tracking**: Kalman filter + Hungarian algorithm
- **ID Persistence**: Maintains IDs across occlusions
- **Speed**: ~20-50ms per frame

### Total Pipeline
- **Detection + Tracking**: ~120-250ms per frame
- **Framerate**: ~4-8 FPS per camera (CPU-limited)
- **4 Cameras**: Processed sequentially

## Future Improvements

1. **GPU Support**: Add CUDA-enabled PyTorch for 10x+ speedup
2. **Model Optimization**: Use yolov7-tiny.pt for faster inference
3. **Parallel Processing**: Multi-threaded camera processing
4. **Alternative Checkpoint**: Host ckpt.t7 on GitHub for auto-download
5. **TensorRT**: Optimize YOLOv7 model with TensorRT

## References

- **YOLOv7**: https://github.com/WongKinYiu/yolov7
- **YOLOv7-DeepSORT**: https://github.com/MuhammadMoinFaisal/YOLOv7-DeepSORT-Object-Tracking
- **DeepSORT PyTorch**: https://github.com/ZQPei/deep_sort_pytorch
- **AirSim**: https://github.com/Cosys-Lab/Cosys-AirSim

## Support

For issues related to:
- **YOLOv7 model**: Check WongKinYiu/yolov7 repository
- **DeepSORT tracking**: Check ZQPei/deep_sort_pytorch repository
- **Docker integration**: Check this documentation and Dockerfile.x11
- **Motion detection**: Check motion_detection_node.py implementation

---

**Last Updated**: 2025-10-15
**Docker Image**: airsim-vnc-ros2:latest
**ROS2 Version**: Humble
**PyTorch Version**: 2.9.0+cpu
