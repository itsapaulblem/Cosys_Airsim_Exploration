FROM ghcr.io/epicgames/unreal-engine:dev-5.5.4

# Switch to root user to fix permission issues
USER root

# Fix apt lists directory permissions and recreate
RUN mkdir -p /var/lib/apt/lists/partial && \
    chmod 755 /var/lib/apt/lists /var/lib/apt/lists/partial && \
    chown -R root:root /var/lib/apt

# Install ONLY essential build utilities - NO TOOLCHAIN PACKAGES
# Epic's container already has the exact toolchain we need
RUN apt-get update && apt-get install -y \
    # Essential build utilities only
    cmake \
    make \
    unzip \
    rsync \
    # System libraries (but NO dev packages that bring headers)
    libvulkan1 \
    vulkan-tools \
    # Clean up package cache
    && rm -rf /var/lib/apt/lists/*

# CRITICAL: Remove any system toolchain packages that might conflict
# This ensures ONLY Epic's bundled toolchain is available
RUN apt-get remove -y --purge \
    build-essential \
    gcc \
    g++ \
    clang* \
    libc++-dev \
    libc++abi-dev \
    libstdc++-dev \
    cpp \
    2>/dev/null || true

# Mask system compiler paths to prevent accidental usage
RUN mkdir -p /usr/local/bin && \
    echo '#!/bin/bash\necho "ERROR: Use Epic'\''s bundled toolchain, not system tools"; exit 1' > /usr/local/bin/gcc && \
    echo '#!/bin/bash\necho "ERROR: Use Epic'\''s bundled toolchain, not system tools"; exit 1' > /usr/local/bin/g++ && \
    echo '#!/bin/bash\necho "ERROR: Use Epic'\''s bundled toolchain, not system tools"; exit 1' > /usr/local/bin/clang && \
    echo '#!/bin/bash\necho "ERROR: Use Epic'\''s bundled toolchain, not system tools"; exit 1' > /usr/local/bin/clang++ && \
    chmod +x /usr/local/bin/gcc /usr/local/bin/g++ /usr/local/bin/clang /usr/local/bin/clang++

# Epic's UE5.5.4 bundled toolchain paths (verified in container)
ENV EPIC_TOOLCHAIN_ROOT="/home/ue4/UnrealEngine/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v23_clang-18.1.0-rockylinux8/x86_64-unknown-linux-gnu"
ENV EPIC_CLANG_PATH="${EPIC_TOOLCHAIN_ROOT}/bin"
ENV EPIC_LIBCPP_INCLUDE="/home/ue4/UnrealEngine/Engine/Source/ThirdParty/Unix/LibCxx/include/c++/v1"
ENV EPIC_LIBCPP_LIB="/home/ue4/UnrealEngine/Engine/Source/ThirdParty/Unix/LibCxx/lib/Unix/x86_64-unknown-linux-gnu"

# Force ONLY Epic's toolchain usage - NO system alternatives
ENV CC="${EPIC_CLANG_PATH}/clang"
ENV CXX="${EPIC_CLANG_PATH}/clang++"
ENV CMAKE_C_COMPILER="${EPIC_CLANG_PATH}/clang"
ENV CMAKE_CXX_COMPILER="${EPIC_CLANG_PATH}/clang++"

# Epic's libc++ configuration with complete ABI isolation
ENV CXXFLAGS="-stdlib=libc++ -nostdinc++ -I${EPIC_LIBCPP_INCLUDE} -D_LIBCPP_ABI_UNSTABLE -D_LIBCPP_DISABLE_AVAILABILITY -D_GLIBCXX_USE_CXX11_ABI=0"
ENV CMAKE_CXX_FLAGS="-stdlib=libc++ -nostdinc++ -I${EPIC_LIBCPP_INCLUDE} -D_LIBCPP_ABI_UNSTABLE -D_LIBCPP_DISABLE_AVAILABILITY -D_GLIBCXX_USE_CXX11_ABI=0"
ENV LDFLAGS="-L${EPIC_LIBCPP_LIB} -lc++ -lc++abi -Wl,--as-needed"
ENV CMAKE_EXE_LINKER_FLAGS="-L${EPIC_LIBCPP_LIB} -lc++ -lc++abi -Wl,--as-needed"
ENV CMAKE_SHARED_LINKER_FLAGS="-L${EPIC_LIBCPP_LIB} -lc++ -lc++abi -Wl,--as-needed"

# Verify Epic's toolchain is available and working
RUN echo "🔍 Verifying Epic's bundled toolchain..." && \
    ls -la "${EPIC_CLANG_PATH}/" && \
    "${EPIC_CLANG_PATH}/clang" --version && \
    echo "✅ Epic's toolchain verification complete"

# Set workspace directory
WORKDIR /workspace

# Create directories for build outputs and ensure proper permissions
RUN mkdir -p /workspace/build && \
    mkdir -p /workspace/output && \
    mkdir -p /workspace/airsim && \
    chown -R ue4:ue4 /workspace

# Add user switching utilities and UE5 runtime environment setup
RUN echo '#!/bin/bash\necho "Switching to ue4 user for UE5 operations..."\nexec sudo -u ue4 -E "$@"' > /usr/local/bin/ue4-exec && \
    chmod +x /usr/local/bin/ue4-exec

# Install sudo for user switching
RUN apt-get update && apt-get install -y sudo && \
    echo "ue4 ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    rm -rf /var/lib/apt/lists/*

# Environment setup for UE5 cook process (fixes recursive initialization)
ENV UE_LOG_LEVEL=Warning
ENV UE_DISABLE_CRASH_ANALYTICS=1
ENV UE_DISABLE_GPU_CRASH_DEBUGGING=1
ENV MALLOC_CHECK_=0
ENV MALLOC_PERTURB_=0

# Create UE5-compatible runtime script
RUN cat > /usr/local/bin/buildcookrun-safe << 'EOF'
#!/bin/bash
set -e

echo "🚀 Starting safe BuildCookRun with proper user privileges..."

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "🔄 Detected root user - switching to ue4 for UE5 operations..."

    # Ensure workspace permissions are correct
    chown -R ue4:ue4 /workspace/airsim 2>/dev/null || true

    # Switch to ue4 user for the actual BuildCookRun
    exec sudo -u ue4 -E bash -c "
        export HOME=/home/ue4
        export USER=ue4
        export LOGNAME=ue4
        cd /home/ue4/UnrealEngine

        echo '📋 Running BuildCookRun as ue4 user...'
        ./Engine/Build/BatchFiles/RunUAT.sh \"\$@\"
    " -- "$@"
else
    echo "✅ Running as non-root user: $USER"
    cd /home/ue4/UnrealEngine
    ./Engine/Build/BatchFiles/RunUAT.sh "$@"
fi
EOF
RUN chmod +x /usr/local/bin/buildcookrun-safe

# Create build-only script (can run as root)
RUN cat > /usr/local/bin/build-only << 'EOF'
#!/bin/bash
set -e

echo "🔧 Running build-only operations (ABI-compatible)..."

# Build operations can run as root - they don't have the UE5 restriction
cd /home/ue4/UnrealEngine

echo "📋 Running build-only commands..."
./Engine/Build/BatchFiles/RunUAT.sh "$@" -nocompile -nobuild
EOF
RUN chmod +x /usr/local/bin/build-only

# Switch back to ue4 user as default
USER ue4
WORKDIR /home/ue4

# Default command
CMD ["/bin/bash"]