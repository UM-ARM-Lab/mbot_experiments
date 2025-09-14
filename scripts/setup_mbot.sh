#!/bin/bash

# Script to set up MBot with ARMLAB fork (TTL=1)
# Can be run from desktop - will SSH into robot to perform setup

set -e

echo "🚀 Setting up MBot with ARMLAB fork (TTL=1)..."
echo ""

# Configuration
ROBOT_HOST="mbot@arm-mbot"
ROBOT_WS_DIR="~/mbot_ws"

# Check if we're on the MBot or desktop
if [ "$(hostname)" == "arm-mbot" ]; then
    echo "✅ Running directly on arm-mbot"
    REMOTE_EXEC=""
else
    echo "🖥️  Running from desktop, will SSH to robot"
    
    # Test SSH connection
    echo "🔍 Testing SSH connection to robot..."
    if ! ssh -o ConnectTimeout=5 "$ROBOT_HOST" "echo 'SSH connection successful'" > /dev/null 2>&1; then
        echo "❌ Cannot connect to robot via SSH"
        echo "   Make sure:"
        echo "   1. Robot is powered on and connected to network"
        echo "   2. SSH is enabled on robot"
        echo "   3. SSH keys are set up (or password authentication works)"
        echo "   4. Robot hostname 'arm-mbot' is resolvable"
        exit 1
    fi
    echo "✅ SSH connection to robot successful"
    
    REMOTE_EXEC="ssh $ROBOT_HOST"
fi

# Execute setup commands on robot (either locally or via SSH)
echo "📁 Checking for mbot_lcm_base directory..."
if ! $REMOTE_EXEC "cd $ROBOT_WS_DIR && [ -d mbot_lcm_base ]"; then
    echo "❌ mbot_lcm_base not found in $ROBOT_WS_DIR on robot."
    echo "   Please ensure mbot_lcm_base exists in the robot's workspace:"
    echo "   ssh $ROBOT_HOST"
    echo "   cd $ROBOT_WS_DIR"
    echo "   ls  # Should show mbot_lcm_base"
    exit 1
fi

echo "✅ Found mbot_lcm_base directory on robot"

# Check if this is already the ARMLAB fork
echo "🔍 Checking git remote configuration..."
REMOTE_URL=$($REMOTE_EXEC "cd $ROBOT_WS_DIR/mbot_lcm_base && git remote get-url origin")
if [[ "$REMOTE_URL" == *"UM-ARM-Lab"* ]]; then
    echo "✅ Already using ARMLAB fork"
else
    echo "🔄 Updating to ARMLAB fork..."
    $REMOTE_EXEC "cd $ROBOT_WS_DIR/mbot_lcm_base && git remote set-url origin https://github.com/UM-ARM-Lab/mbot_lcm_base.git"
    $REMOTE_EXEC "cd $ROBOT_WS_DIR/mbot_lcm_base && git fetch origin"
    $REMOTE_EXEC "cd $ROBOT_WS_DIR/mbot_lcm_base && git checkout main"
    echo "✅ Updated to ARMLAB fork"
fi

# Verify TTL configuration
echo "🔍 Verifying TTL configuration..."
TIMESYNC_TTL=$($REMOTE_EXEC "cd $ROBOT_WS_DIR/mbot_lcm_base && grep -o 'ttl=[0-9]' timesync/include/timesync/lcm_config.h | cut -d'=' -f2")
SERIAL_TTL=$($REMOTE_EXEC "cd $ROBOT_WS_DIR/mbot_lcm_base && grep -o 'ttl=[0-9]' mbot_lcm_serial/include/mbot_lcm_serial/lcm_config.h | cut -d'=' -f2")

if [ "$TIMESYNC_TTL" = "1" ] && [ "$SERIAL_TTL" = "1" ]; then
    echo "✅ TTL already set to 1 in both config files"
else
    echo "❌ TTL not set to 1. Please check the fork configuration."
    echo "   Timesync TTL: $TIMESYNC_TTL"
    echo "   Serial TTL: $SERIAL_TTL"
    exit 1
fi

echo ""
echo "🔧 Building and installing mbot_lcm_base on robot..."

# Build and install
$REMOTE_EXEC "cd $ROBOT_WS_DIR/mbot_lcm_base && ./scripts/install.sh"

echo "✅ Build and installation completed"

echo ""
echo "🎉 MBot setup completed!"
echo ""
echo "📋 What was configured on the robot:"
echo "   - Updated to ARMLAB fork with TTL=1"
echo "   - Built and installed mbot_lcm_base"
echo "   - MBot can now send/receive messages on the local network"
echo ""
echo "🔄 Next steps:"
echo "   1. Restart any running MBot LCM processes on the robot:"
if [ -n "$REMOTE_EXEC" ]; then
    echo "      ssh $ROBOT_HOST"
fi
echo "      sudo systemctl restart mbot-lcm-serial"
echo "      sudo systemctl restart mbot-bridge"
echo ""
echo "💡 Note: The TTL=1 setting allows LCM messages to be sent/received"
echo "   across the local network between desktop and robot."
echo ""
echo "🧪 Test the connection:"
echo "   - Run: ./start_robot_service.sh"
echo "   - Robot should now respond to motion commands from desktop"
