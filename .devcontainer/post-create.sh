#!/bin/bash
set -e

echo "Setting up WPILib 2026 environment..."

# Create wpilib directory and symlink
mkdir -p ~/wpilib
ln -sf /opt/wpilib/2026 ~/wpilib/2026

# Install WPILib extensions from .vsix files
echo "Installing WPILib extensions..."
for vsix in /opt/wpilib/2026/vsCodeExtensions/*.vsix; do
    echo "Installing $(basename "$vsix")..."
    ~/.vscode-server/bin/*/bin/code-server --install-extension "$vsix" --force
done

echo "WPILib 2026 environment ready!"
