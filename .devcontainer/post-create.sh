#!/bin/bash
set -e

echo "Setting up WPILib 2026 environment..."

# Create wpilib directory and symlink
mkdir -p ~/wpilib
ln -sf /opt/wpilib/2026 ~/wpilib/2026

# Install WPILib extensions from .vsix files
echo "Installing WPILib extensions..."
CODE_SERVER=$(find ~/.vscode-remote/bin -name code-server -type f 2>/dev/null | head -n 1)
if [ -n "$CODE_SERVER" ]; then
    for vsix in /opt/wpilib/2026/vsCodeExtensions/*.vsix; do
        echo "Installing $(basename "$vsix")..."
        "$CODE_SERVER" --install-extension "$vsix" --force
    done
else
    echo "Warning: code-server not found, skipping extension installation"
fi

echo "WPILib 2026 environment ready!"
