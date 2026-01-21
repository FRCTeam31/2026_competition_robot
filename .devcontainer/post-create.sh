#!/bin/bash
set -e

echo "Setting up WPILib 2026 environment..."

# Create wpilib directory and symlink
mkdir -p ~/wpilib
ln -sf /opt/wpilib/2026 ~/wpilib/2026

# Install WPILib extensions from .vsix files
echo "Installing WPILib extensions..."
echo "DEBUG: HOME=$HOME"
echo "DEBUG: Checking for .vscode-remote directory..."
ls -la $HOME/.vscode-remote/ 2>&1 || echo "DEBUG: .vscode-remote directory does not exist"
echo "DEBUG: Looking for code-server..."
ls -la $HOME/.vscode-remote/bin/*/bin/code-server 2>&1 || echo "DEBUG: code-server pattern did not match"

CODE_SERVER=$(echo $HOME/.vscode-remote/bin/*/bin/code-server)
echo "DEBUG: CODE_SERVER=$CODE_SERVER"
echo "DEBUG: Testing if file exists..."
if [ -f "$CODE_SERVER" ]; then
    echo "DEBUG: code-server found at $CODE_SERVER"
    for vsix in /opt/wpilib/2026/vsCodeExtensions/*.vsix; do
        echo "Installing $(basename "$vsix")..."
        "$CODE_SERVER" --install-extension "$vsix" --force
    done
else
    echo "Warning: code-server not found, skipping extension installation"
    echo "DEBUG: This likely means the .vscode-remote directory hasn't been created yet during post-create"
fi

echo "WPILib 2026 environment ready!"
