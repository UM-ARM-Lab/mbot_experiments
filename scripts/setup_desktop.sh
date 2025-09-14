#!/bin/bash

# Script to set up desktop for MBot LCM communication
# This should be run on your desktop computer

set -e

echo "🚀 Setting up desktop for MBot LCM communication..."
echo ""

# Check if we're in the right directory
if [ ! -d "external/mbot_lcm_base/mbot_msgs/lcmtypes" ]; then
    echo "❌ MBot LCM message types not found."
    echo "   Please run this script from the mbot_experiments root directory."
    exit 1
fi

echo "📁 Found MBot LCM message types in external/mbot_lcm_base/mbot_msgs/lcmtypes/"

# Check if lcm-gen is available
if ! command -v lcm-gen &> /dev/null; then
    echo "❌ lcm-gen not found. Please install LCM first:"
    echo "   sudo apt-get install liblcm-dev"
    exit 1
fi

echo "✅ lcm-gen found"

echo ""
echo "🔧 Step 1: Setting up desktop multicast configuration..."

# Set desktop LCM multicast URL to receive MBot messages
export LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=1"

# Add to bashrc if not already there
if ! grep -q "LCM_DEFAULT_URL.*ttl=1" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# MBot LCM communication" >> ~/.bashrc
    echo "export LCM_DEFAULT_URL=\"udpm://239.255.76.67:7667?ttl=1\"" >> ~/.bashrc
    echo "✅ Added LCM_DEFAULT_URL to ~/.bashrc"
else
    echo "ℹ️  LCM_DEFAULT_URL already in ~/.bashrc"
fi

echo "✅ Desktop multicast configured to receive MBot messages"

echo ""
echo "🔧 Step 2: Generating LCM message types..."

# Generate Python bindings for MBot LCM messages
echo "🔧 Generating Python bindings for MBot LCM messages..."
cd external/mbot_lcm_base/mbot_msgs

if [ -d "lcmtypes" ]; then
    echo "   Generating Python bindings for all MBot messages (including motion_command)..."
    lcm-gen -p lcmtypes/*.lcm
    
    # Check if mbot_lcm_msgs directory was created
    if [ -d "mbot_lcm_msgs" ]; then
        echo "✅ Python bindings generated in mbot_lcm_msgs/"
        echo "   Generated files:"
        ls -la mbot_lcm_msgs/ | head -5
        echo "   ... and $(ls mbot_lcm_msgs/*.py | wc -l) total Python files"
    else
        echo "❌ Failed to generate Python bindings"
        exit 1
    fi
else
    echo "❌ lcmtypes directory not found in mbot_lcm_base/mbot_msgs"
    exit 1
fi

# Generate Python bindings for Vicon LCM messages
echo "🔧 Generating Python bindings for Vicon LCM messages..."
cd ../vicon2lcm

if [ -d "lcm_msgs" ]; then
    echo "   Generating Python bindings for Vicon messages..."
    lcm-gen -p lcm_msgs/vicon_state.lcm lcm_msgs/twist3D.lcm
    
    # Check if vicon_msgs directory was created
    if [ -d "vicon_msgs" ]; then
        echo "✅ Python bindings generated in vicon_msgs/"
        echo "   Generated files:"
        ls -la vicon_msgs/
    else
        echo "❌ Failed to generate Python bindings"
        exit 1
    fi
else
    echo "❌ lcm_msgs directory not found in vicon2lcm"
    exit 1
fi

# Generate Java bindings for all MBot message types
echo "🔧 Generating Java bindings for MBot message types..."
cd ../mbot_lcm_base/mbot_msgs/lcmtypes

# Create temporary directory for generation
TEMP_DIR=$(mktemp -d)
echo "📁 Using temporary directory: $TEMP_DIR"

# Generate Java for each .lcm file
for lcm_file in *.lcm; do
    if [ -f "$lcm_file" ]; then
        echo "   Generating Java for: $lcm_file"
        lcm-gen -j --jpath "$TEMP_DIR" "$lcm_file"
    fi
done

echo "✅ Java bindings generated"

# Compile Java classes
echo "🔧 Compiling Java classes..."
cd "$TEMP_DIR"

# Check if files were generated in a subdirectory
if [ -d "mbot_lcm_msgs" ]; then
    echo "   Java files generated in mbot_lcm_msgs/ subdirectory"
    cd mbot_lcm_msgs
else
    echo "   Java files generated in current directory"
fi

# List all generated Java files
echo "   Generated Java files:"
ls -la *.java

# Try to compile with system lcm.jar
lcm_jar=$(find /usr -name "lcm.jar" 2>/dev/null | head -1)
if [ -n "$lcm_jar" ]; then
    echo "   Using LCM JAR: $lcm_jar"
    javac -cp "$lcm_jar" *.java
else
    echo "⚠️  Warning: Could not find system lcm.jar, trying alternative paths..."
    # Try common LCM installation paths
    for lcm_jar in /usr/share/java/lcm.jar /usr/local/share/java/lcm.jar ~/.local/share/java/lcm.jar; do
        if [ -f "$lcm_jar" ]; then
            echo "   Using LCM JAR: $lcm_jar"
            javac -cp "$lcm_jar" *.java
            break
        fi
    done
fi

echo "✅ Java classes compiled"

# Create JAR file with proper package structure
echo "🔧 Creating JAR file..."
# The classes should be in mbot_lcm_msgs/ directory structure
mkdir -p mbot_lcm_msgs
mv *.class mbot_lcm_msgs/
jar cf mbot_lcm_msgs.jar mbot_lcm_msgs/

# Create target directory
TARGET_DIR="$HOME/.local/share/java"
mkdir -p "$TARGET_DIR"

# Copy JAR to target location
cp mbot_lcm_msgs.jar "$TARGET_DIR/"
echo "✅ JAR installed to $TARGET_DIR/mbot_lcm_msgs.jar"

# Add to CLASSPATH in bashrc if not already there
if ! grep -q "mbot_lcm_msgs.jar" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# MBot LCM message types" >> ~/.bashrc
    echo "export CLASSPATH=\$CLASSPATH:$TARGET_DIR/mbot_lcm_msgs.jar" >> ~/.bashrc
    echo "✅ Added to ~/.bashrc"
else
    echo "ℹ️  Already in ~/.bashrc"
fi

# Clean up temporary directory
cd /
rm -rf "$TEMP_DIR"
echo "🧹 Cleaned up temporary files"

echo ""
echo "🎉 Desktop setup completed!"
echo ""
echo "📋 What was configured:"
echo "   - Desktop LCM multicast URL set to udpm://239.255.76.67:7667?ttl=1"
echo "   - Python bindings for MBot LCM messages generated"
echo "   - Python bindings for Vicon LCM messages generated"
echo "   - Java bindings for all MBot message types generated"
echo "   - JAR file installed: $TARGET_DIR/mbot_lcm_msgs.jar"
echo "   - CLASSPATH updated with MBot message types"
echo ""
echo "🔄 Next steps:"
echo "   1. Use lcm-spy to view MBot messages (should now be decodable!)"
echo ""
echo "💡 Note: The desktop is now configured to receive and decode MBot LCM messages."
echo "   Make sure the MBot is also configured with TTL=1 using the setup_mbot.sh script."
echo ""
