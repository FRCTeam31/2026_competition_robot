#!/bin/bash

echo "Running post-attach setup..."

# Check if WPILib extensions have already been installed
if [ -n "$WPILIB_EXTENSIONS_INSTALLED" ]; then
    echo "WPILib extensions already installed, skipping..."
    exit 0
fi

# Check if code-server is available
CODE_SERVER=$(echo $HOME/.vscode-remote/bin/*/bin/code-server)
if [ ! -f "$CODE_SERVER" ]; then
    echo "VS Code server not ready yet, skipping extension installation..."
    echo "Extensions will be installed on next attach."
    exit 0
fi

echo "Installing WPILib extensions..."
INSTALL_SUCCESS=true

for vsix in /opt/wpilib/2026/vsCodeExtensions/*.vsix; do
    if [ -f "$vsix" ]; then
        echo "Installing $(basename "$vsix")..."
        if ! "$CODE_SERVER" --install-extension "$vsix" --force; then
            echo "Warning: Failed to install $(basename "$vsix")"
            INSTALL_SUCCESS=false
        fi
    fi
done

if [ "$INSTALL_SUCCESS" = true ]; then
    # Mark extensions as installed by creating a marker file
    # (environment variables don't persist between sessions)
    touch "$HOME/.wpilib-extensions-installed"
    echo "WPILib extensions installed successfully!"
else
    echo "Some extensions failed to install. They will be retried on next attach."
fi
