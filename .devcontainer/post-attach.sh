#!/bin/bash

echo "Running post-attach setup..."

# Check if WPILib extensions have already been installed
if [ -f "$HOME/.wpilib-extensions-installed" ]; then
    echo "WPILib extensions already installed, skipping..."
    exit 0
fi

echo "Installing WPILib extensions..."
INSTALL_SUCCESS=true

for vsix in /opt/wpilib/2026/vsCodeExtensions/*.vsix; do
    if [ -f "$vsix" ]; then
        echo "Installing $(basename "$vsix")..."
        if ! code --install-extension "$vsix" --force; then
            echo "Warning: Failed to install $(basename "$vsix")"
            INSTALL_SUCCESS=false
        fi
    fi
done

if [ "$INSTALL_SUCCESS" = true ]; then
    # Mark extensions as installed by creating a marker file
    touch "$HOME/.wpilib-extensions-installed"
    echo "WPILib extensions installed successfully!"
    echo "Please reload the window for icons to display correctly."
    echo "You can reload by pressing Ctrl+Shift+P and selecting 'Developer: Reload Window'"
else
    echo "Some extensions failed to install. They will be retried on next attach."
fi
