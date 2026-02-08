#!/bin/bash

# Determine OS and set destination
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    DEST="$HOME/Library/Application Support/AdvantageScope/userAssets"
elif [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
    # Windows (Git Bash)
    DEST="$APPDATA/AdvantageScope/userAssets"
else
    # Linux
    DEST="$HOME/.config/AdvantageScope/userAssets"
fi

SOURCE="./advantagescope-assets"

# Create destination directory if it doesn't exist
mkdir -p "$DEST"

# Copy assets
echo "Copying AdvantageScope assets..."
cp -r "$SOURCE"/* "$DEST/"
echo "AdvantageScope assets copied successfully!"