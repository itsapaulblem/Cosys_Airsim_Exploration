# YOLOv10 Detection Service - Lightweight ML Container
# Purpose: Isolated object detection service for ROS2 microservices architecture
# Eliminates dependency conflicts with control/AirSim container

FROM ros:humble-ros-base

# Minimal environment setup
ENV PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    DEBIAN_FRONTEND=noninteractive

# Install only essential system packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    git \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Install Python ML stack (NO version conflicts - isolated environment!)
# IMPORTANT: Use NumPy 1.23.x for both cv_bridge ABI and YOLOv10 compatibility
# - cv_bridge in ROS2 Humble: compiled with NumPy 1.x (incompatible with 2.x)
# - YOLOv10 README requirement: numpy<1.24
# - Resolution: Use numpy 1.23.5 (satisfies both <1.24 and <2.0)
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
        opencv-python \
        'numpy>=1.23,<1.24' \
        Pillow \
        PyYAML \
        && echo "[INFO] Base dependencies installed with NumPy 1.23.x for dual compatibility"

# Install YOLOv10-specific dependencies (before PyTorch to control versions)
RUN pip3 install --no-cache-dir \
        onnx==1.14.0 \
        onnxruntime==1.15.1 \
        pycocotools==2.0.7 \
        onnxsim==0.4.36 \
        py-cpuinfo==9.0.0 \
        gdown \
        && echo "[INFO] YOLOv10 dependencies installed"

# Install PyTorch 2.8.0 with CUDA 12.8 for RTX 5080 (sm_120) support
# CRITICAL: CUDA 12.8+ required for Blackwell architecture (sm_120 compute capability)
# PyTorch 2.8.0 stable includes full sm_120 support compiled with cu128
# PERFORMANCE: GPU acceleration provides 100x speedup over CPU (3700ms → 37ms per frame)
RUN pip3 install --no-cache-dir \
        torch==2.8.0 torchvision --index-url https://download.pytorch.org/whl/cu128 \
        && echo "[SUCCESS] PyTorch 2.8.0 with CUDA 12.8 installed for RTX 5080 (sm_120) support"

# Install YOLOv10 and dependencies
RUN pip3 install --no-cache-dir \
        ultralytics \
        huggingface_hub \
        safetensors \
        && echo "[INFO] YOLOv10 dependencies installed"

# Install YOLOv10 from official repo
RUN pip3 install --no-cache-dir git+https://github.com/THU-MIG/yolov10.git && \
    echo "[SUCCESS] YOLOv10 installed"

# CRITICAL: Force-reinstall NumPy 1.23.x after all other packages
# PyTorch/ultralytics/YOLOv10 may pull in NumPy 2.x which breaks cv_bridge ABI
# This MUST be the last pip install to ensure dual compatibility
RUN pip3 install --no-cache-dir --force-reinstall 'numpy>=1.23,<1.24' && \
    echo "[SUCCESS] NumPy 1.23.x forced for YOLOv10 + cv_bridge compatibility"

# Install easydict AFTER NumPy pinning to avoid version conflicts
RUN pip3 install --no-cache-dir easydict && \
    echo "[SUCCESS] EasyDict installed"

# DeepSORT checkpoint will be provided via volume mount from host
# Google Drive folder (for manual download): https://drive.google.com/drive/folders/1kna8eWGrSfzaR6DtNJ8_GchGgPMv3VC8
# The deep_sort_pytorch directory should be placed in the YOLOv10-DeepSORT/yolov10/ directory on host
# Alternative: Download during runtime via entrypoint if not present
RUN echo "[INFO] DeepSORT checkpoint should be provided via volume mount or exist in yolov10/ directory"

# Create workspace
WORKDIR /detection_ws

# Copy YOLOv10-DeepSORT integration code (mounted at runtime)
# This will be volume-mounted from host: ros2/src/airsim_ros_pkgs/scripts/YOLOv10-DeepSORT

# Install ROS2 Python dependencies (must be installed via apt, not pip)
# Note: rclpy is already included in ros:humble-ros-base base image
# These packages are not available on PyPI - they're ROS2 system packages
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-sensor-msgs \
        ros-humble-cv-bridge \
        ros-humble-geographic-msgs \
        && rm -rf /var/lib/apt/lists/* \
        && echo "[INFO] ROS2 Python dependencies installed"

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Health check - verify YOLOv10 can be imported
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD python3 -c "from ultralytics import YOLOv10; import rclpy" || exit 1

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/detection_ws/detection_service_node.py"]
