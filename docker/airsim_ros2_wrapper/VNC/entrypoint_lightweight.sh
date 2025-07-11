#!/bin/bash

set -e

# Function to print colored messages
print_info() {
    echo -e "\e[32m[INFO]\e[0m $1"
}

print_warning() {
    echo -e "\e[33m[WARNING]\e[0m $1"
}

print_error() {
    echo -e "\e[31m[ERROR]\e[0m $1"
}

# Set default values
USER=${USER:-ubuntu}
PASSWD=${PASSWD:-ubuntu}
RESOLUTION=${RESOLUTION:-1920x1080}
VNC_PORT=${VNC_PORT:-5901}
DISPLAY_NUM=${DISPLAY_NUM:-1}

print_info "=== AirSim ROS2 VNC Container Starting ==="
print_info "User: $USER"
print_info "VNC Resolution: $RESOLUTION"
print_info "VNC Port: $VNC_PORT"
print_info "Display: :$DISPLAY_NUM"

# Ensure user exists
if ! id "$USER" &>/dev/null; then
    print_info "Creating user: $USER"
    useradd -m -s /bin/bash "$USER"
    echo "$USER:$PASSWD" | chpasswd
    usermod -aG sudo "$USER"
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
fi

# Get user home directory
USER_HOME=$(getent passwd "$USER" | cut -d: -f6)
VNC_DIR="$USER_HOME/.vnc"

print_info "Setting up VNC for user: $USER"

# Create VNC directory as the user
sudo -u "$USER" mkdir -p "$VNC_DIR"

# Set VNC password
echo "$PASSWD" | sudo -u "$USER" vncpasswd -f > "$VNC_DIR/passwd"
sudo -u "$USER" chmod 600 "$VNC_DIR/passwd"

# Create VNC startup script for XFCE
cat > "$VNC_DIR/xstartup" << EOF
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export DISPLAY=:$DISPLAY_NUM

# Start dbus if available
if [ -x /usr/bin/dbus-launch ]; then
    eval \$(dbus-launch --sh-syntax --exit-with-session)
fi

# Load X resources
xrdb \$HOME/.Xresources 2>/dev/null || true

# Set background color (optional)
xsetroot -solid grey 2>/dev/null || true

# Start window manager
startxfce4 &

# Keep session alive
wait
EOF

sudo -u "$USER" chmod +x "$VNC_DIR/xstartup"

# Create VNC config file
cat > "$VNC_DIR/config" << EOF
session=xfce
geometry=$RESOLUTION
localhost=no
alwaysshared=yes
EOF

sudo -u "$USER" chmod 644 "$VNC_DIR/config"

# Ensure proper ownership
chown -R "$USER:$USER" "$VNC_DIR"

# Setup ROS2 environment for the user
print_info "Setting up ROS2 environment"
if [ ! -f "$USER_HOME/.bashrc" ]; then
    sudo -u "$USER" touch "$USER_HOME/.bashrc"
fi

# Add ROS2 setup to user's bashrc if not already present
if ! grep -q "source /opt/ros/humble/setup.bash" "$USER_HOME/.bashrc"; then
    echo "source /opt/ros/humble/setup.bash" >> "$USER_HOME/.bashrc"
fi

if ! grep -q "source /airsim_ros2_ws/install/setup.bash" "$USER_HOME/.bashrc"; then
    echo "source /airsim_ros2_ws/install/setup.bash" >> "$USER_HOME/.bashrc"
fi

# Create .Xauthority file
sudo -u "$USER" touch "$USER_HOME/.Xauthority"
sudo -u "$USER" chmod 600 "$USER_HOME/.Xauthority"

# Set up logging directory
mkdir -p /var/log/supervisor
chown "$USER:$USER" /var/log/supervisor

# Fix workspace permissions (critical for colcon build)
print_info "Setting up workspace permissions for user: $USER"
mkdir -p /airsim_ros2_ws/log /airsim_ros2_ws/build /airsim_ros2_ws/install
chown -R "$USER:$USER" /airsim_ros2_ws/log /airsim_ros2_ws/build /airsim_ros2_ws/install /airsim_ros2_ws/src
chmod -R 755 /airsim_ros2_ws/log /airsim_ros2_ws/build /airsim_ros2_ws/install

# Kill any existing VNC processes
print_info "Cleaning up existing VNC processes"
sudo -u "$USER" vncserver -kill :$DISPLAY_NUM 2>/dev/null || true
pkill Xvnc 2>/dev/null || true
pkill Xtigervnc 2>/dev/null || true
pkill vncserver 2>/dev/null || true
sleep 2

# Remove any existing lock files
rm -f /tmp/.X${DISPLAY_NUM}-lock 2>/dev/null || true
rm -f /tmp/.X11-unix/X${DISPLAY_NUM} 2>/dev/null || true

# Create X11 socket directory if it doesn't exist
mkdir -p /tmp/.X11-unix
chmod 1777 /tmp/.X11-unix

# Test VNC password file
print_info "Testing VNC configuration"
if [ ! -f "$VNC_DIR/passwd" ]; then
    print_error "VNC password file not found!"
    exit 1
fi

# Start VNC server manually first to test
print_info "Starting VNC server on display :$DISPLAY_NUM"
sudo -u "$USER" bash <<EOF
export DISPLAY=:$DISPLAY_NUM
cd "$USER_HOME"
vncserver :$DISPLAY_NUM \
    -geometry $RESOLUTION \
    -depth 24 \
    -rfbport $VNC_PORT \
    -localhost no \
    -SecurityTypes VncAuth \
    -rfbauth "$VNC_DIR/passwd" \
    -xstartup "$VNC_DIR/xstartup" \
    -AlwaysShared \
    -AcceptKeyEvents \
    -AcceptPointerEvents \
    -AcceptCutText \
    -SendCutText
EOF

# Check if VNC started successfully
sleep 3
if pgrep -f "X.*vnc.*:$DISPLAY_NUM" > /dev/null; then
    print_info "VNC server started successfully on display :$DISPLAY_NUM"
else
    print_error "VNC server failed to start. Checking logs..."
    
    # Try to get VNC log
    VNC_LOG="$USER_HOME/.vnc/$(hostname):${DISPLAY_NUM}.log"
    if [ -f "$VNC_LOG" ]; then
        print_error "VNC Log contents:"
        cat "$VNC_LOG"
    fi
    
    # Try alternative VNC startup method
    print_warning "Trying alternative VNC startup method..."
    sudo -u "$USER" bash <<EOF
        export DISPLAY=:$DISPLAY_NUM
        cd "$USER_HOME"
        vncserver :$DISPLAY_NUM \
            -geometry $RESOLUTION \
            -depth 24 \
            -rfbport $VNC_PORT \
            -localhost no \
            -SecurityTypes VncAuth \
            -rfbauth "$VNC_DIR/passwd" \
            -xstartup "$VNC_DIR/xstartup" \
            -AlwaysShared \
            -AcceptKeyEvents \
            -AcceptPointerEvents \
            -AcceptCutText \
            -SendCutText
EOF
    
    sleep 3
    
    # Start the window manager separately
    sudo -u "$USER" -i bash -c "
        export DISPLAY=:$DISPLAY_NUM
        cd '$USER_HOME'
        startxfce4 &
    " &
    
    sleep 5
fi

# Verify VNC is running
# if pgrep -f "Xvnc.*:$DISPLAY_NUM" > /dev/null; then
#     print_info "VNC server started successfully on port $VNC_PORT"
# else
#     print_error "Failed to start VNC server"
#     exit 1
# fi

# Create desktop shortcuts with proper permissions
DESKTOP_DIR="$USER_HOME/Desktop"
sudo -u "$USER" mkdir -p "$DESKTOP_DIR"

# Create temporary files first, then move them with proper ownership
TEMP_DIR="/tmp/desktop_files"
mkdir -p "$TEMP_DIR"

# AirSim ROS2 launcher shortcut
cat > "$TEMP_DIR/AirSim_ROS2.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=AirSim ROS2
Comment=Launch AirSim ROS2 Wrapper
Exec=xfce4-terminal -e /launch_airsim_ros2.sh
Icon=applications-science
Terminal=true
Categories=Development;
EOF

# RViz2 shortcut
cat > "$TEMP_DIR/RViz2.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=RViz2
Comment=Launch RViz2
Exec=bash -c "source /opt/ros/humble/setup.bash && source /airsim_ros2_ws/install/setup.bash && rviz2"
Icon=applications-graphics
Terminal=false
Categories=Development;
EOF

# VSCodium shortcut
cat > "$TEMP_DIR/VSCodium.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=VSCodium
Comment=Code Editor
Exec=codium
Icon=vscodium
Terminal=false
Categories=Development;
EOF

# Terminal shortcut
cat > "$TEMP_DIR/Terminal.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Terminal
Comment=Terminal
Exec=xfce4-terminal
Icon=utilities-terminal
Terminal=false
Categories=System;
EOF

# Move files with proper ownership and permissions
for file in "$TEMP_DIR"/*.desktop; do
    if [ -f "$file" ]; then
        filename=$(basename "$file")
        cp "$file" "$DESKTOP_DIR/$filename"
        chown "$USER:$USER" "$DESKTOP_DIR/$filename"
        chmod +x "$DESKTOP_DIR/$filename"
    fi
done

# Clean up temporary directory
rm -rf "$TEMP_DIR"

# Ensure entire Desktop directory has correct ownership
chown -R "$USER:$USER" "$DESKTOP_DIR"

print_info "VNC server is running on port $VNC_PORT"
print_info "Connect using: VNC Viewer -> localhost:$VNC_PORT"
print_info "Password: $PASSWD"

# Keep the container running
print_info "Container is ready. VNC server will continue running."

# Monitor VNC process and restart if needed
while true; do
    # Check for both Xvnc and Xtigervnc processes
    if ! pgrep -f "X.*vnc.*:$DISPLAY_NUM" > /dev/null; then
        print_warning "VNC server stopped. Restarting..."
        
        # Clean up any existing lock files and processes first
        sudo -u "$USER" vncserver -kill :$DISPLAY_NUM 2>/dev/null || true
        rm -f /tmp/.X${DISPLAY_NUM}-lock 2>/dev/null || true
        rm -f /tmp/.X11-unix/X${DISPLAY_NUM} 2>/dev/null || true
        sleep 2
        
        # Start VNC server
        sudo -u "$USER" bash <<EOF
            export DISPLAY=:$DISPLAY_NUM
            cd "$USER_HOME"
            vncserver :$DISPLAY_NUM \
                -geometry $RESOLUTION \
                -depth 24 \
                -rfbport $VNC_PORT \
                -localhost no \
                -SecurityTypes VncAuth \
                -rfbauth "$VNC_DIR/passwd" \
                -xstartup "$VNC_DIR/xstartup" \
                -AlwaysShared \
                -AcceptKeyEvents \
                -AcceptPointerEvents \
                -AcceptCutText \
                -SendCutText
EOF
        
        if [ $? -eq 0 ]; then
            print_info "VNC server restarted successfully"
        else
            print_error "Failed to restart VNC server"
        fi
    fi
    sleep 30
done