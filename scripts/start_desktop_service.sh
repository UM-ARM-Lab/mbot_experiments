#!/bin/bash
#
# Start MBot Monitor Service
# Run this on the desktop to start the persistent monitor service
#

set -e

echo "🖥️  Starting MBot Monitor Service"
echo "=" * 40

# Check if we're on the desktop (not robot)
HOSTNAME=$(hostname)
if [ "$HOSTNAME" = "arm-mbot" ]; then
    echo "⚠️  Warning: This script is designed to run on the desktop, not on arm-mbot"
    echo "   Continue anyway? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "❌ Aborted"
        exit 1
    fi
fi

# Check for required files with correct paths
echo "🔍 Checking for required files..."

# Scripts that should be in current directory
SCRIPTS=("mbot_monitor.py" "exp_utils.py")
for file in "${SCRIPTS[@]}"; do
    if [ ! -f "$file" ]; then
        echo "❌ Missing script: $file"
        echo "💡 Make sure you're running from the scripts/ directory"
        exit 1
    fi
done

# Check for LCM message bindings in proper locations
VICON_MSG_DIR="../external/vicon2lcm/vicon_msgs/vicon_msgs"
MBOT_MSG_DIR="../mbot_lcm_msgs"

if [ ! -d "$VICON_MSG_DIR" ] || [ ! -f "$VICON_MSG_DIR/vicon_state_t.py" ]; then
    echo "❌ Vicon LCM messages not found in $VICON_MSG_DIR"
    echo "💡 Run ./scripts/setup_desktop.sh to generate LCM bindings"
    exit 1
fi

if [ ! -d "$MBOT_MSG_DIR" ] || [ ! -f "$MBOT_MSG_DIR/twist2D_t.py" ]; then
    echo "❌ MBot LCM messages not found in $MBOT_MSG_DIR"
    echo "💡 Run ./scripts/setup_desktop.sh to generate LCM bindings"
    exit 1
fi

echo "✅ All required files found"

# Check LCM environment
if [ -z "$LCM_DEFAULT_URL" ]; then
    echo "🔧 Setting LCM_DEFAULT_URL..."
    export LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=1"
    echo "   LCM_DEFAULT_URL=$LCM_DEFAULT_URL"
else
    echo "✅ LCM_DEFAULT_URL already set: $LCM_DEFAULT_URL"
fi

# Check for Python dependencies
echo "🔍 Checking Python dependencies..."
python3 -c "import matplotlib, lcm, numpy" 2>/dev/null || {
    echo "❌ Missing Python dependencies"
    echo "   Install with: pip3 install matplotlib lcm numpy"
    exit 1
}
echo "✅ Python dependencies OK"

# Start the monitor service
echo ""
echo "🚀 Starting monitor service..."
echo "   This service provides:"
echo "   - Real-time robot visualization"
echo "   - Safety zone monitoring" 
echo "   - Emergency stop capability"
echo "   - Motion status display"
echo ""
echo "💡 Monitor runs as persistent service - leave this terminal open"
echo "   Use Ctrl+C or emergency button to stop"
echo ""

# Run the monitor with default experiment name if none provided
EXP_NAME=${1:-"sysID"}
python3 mbot_monitor.py --exp_name "$EXP_NAME"