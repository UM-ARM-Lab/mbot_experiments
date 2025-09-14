#!/bin/bash
#
# Start Robot Motion Controller Service
# Run this from the desktop to start the robot service via SSH
#

set -e

echo "🤖 Starting MBot Robot Motion Controller Service"
echo "=================================================="

# Configuration
ROBOT_HOST="mbot@arm-mbot"
ROBOT_SCRIPTS_DIR="~/mbot_ws/scripts"
DESKTOP_SCRIPTS_DIR="$(pwd)"

echo "🔍 Checking SSH connection to robot..."
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

echo ""
echo "📁 Copying updated files to robot..."
echo "   From: $DESKTOP_SCRIPTS_DIR"
echo "   To: $ROBOT_SCRIPTS_DIR"

# Copy updated script files to robot
scp "$DESKTOP_SCRIPTS_DIR/onboard_controller.py" "$ROBOT_HOST:$ROBOT_SCRIPTS_DIR"
scp "$DESKTOP_SCRIPTS_DIR/config.py" "$ROBOT_HOST:$ROBOT_SCRIPTS_DIR"
scp "$DESKTOP_SCRIPTS_DIR/exp_utils.py" "$ROBOT_HOST:$ROBOT_SCRIPTS_DIR"

# Copy LCM message files (ensure robot has latest generated files)
echo "📦 Copying LCM message files..."
scp "../mbot_lcm_msgs/motion_command_t.py" "$ROBOT_HOST:$ROBOT_SCRIPTS_DIR"
scp "../mbot_lcm_msgs/motion_status_t.py" "$ROBOT_HOST:$ROBOT_SCRIPTS_DIR"
scp "../mbot_lcm_msgs/twist2D_t.py" "$ROBOT_HOST:$ROBOT_SCRIPTS_DIR"
scp "../mbot_lcm_msgs/mbot_analog_t.py" "$ROBOT_HOST:$ROBOT_SCRIPTS_DIR"

# Copy vicon message files (needed for velocity estimation)
echo "📦 Copying vicon message files..."
scp "../external/vicon2lcm/vicon_msgs/vicon_msgs/vicon_twist_t.py" "$ROBOT_HOST:$ROBOT_SCRIPTS_DIR"
scp "../external/vicon2lcm/vicon_msgs/vicon_msgs/vicon_state_t.py" "$ROBOT_HOST:$ROBOT_SCRIPTS_DIR"

# Verify critical files were copied
echo "🔍 Verifying LCM message files on robot..."
ssh "$ROBOT_HOST" "ls -la $ROBOT_SCRIPTS_DIR/motion_status_t.py $ROBOT_SCRIPTS_DIR/motion_command_t.py $ROBOT_SCRIPTS_DIR/vicon_twist_t.py" || echo "⚠️  Warning: Some LCM message files may not be present on robot"

echo "✅ All files copied successfully"

echo ""
echo "🚀 Starting robot motion controller service on robot..."
echo "   This will open an SSH session to the robot"
echo "   The service will run persistently and listen for motion commands"
echo "   Press Ctrl+C in the SSH session to stop the robot service"
echo "   (This will close the SSH connection)"
echo ""

# SSH into robot and start the service
ssh -t "$ROBOT_HOST" "cd /home/mbot/mbot_ws/scripts && echo '🤖 Robot motion controller starting...' && python3 onboard_controller.py"