#!/bin/bash
set -e

echo "Setting up WPILib 2026 environment..."

# Create wpilib directory and symlink
mkdir -p ~/wpilib
ln -sf /opt/wpilib/2026 ~/wpilib/2026

echo "WPILib 2026 environment ready!"
echo "Extensions will be installed when you attach to the container..."
