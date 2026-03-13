#!/bin/bash
# Build KittyV1 Fin Calculator as a macOS .app bundle and .dmg
# Run this on a Mac with Python 3 and pip installed.
#
# Usage:  chmod +x build_macos.sh && ./build_macos.sh

set -e

APP_NAME="KittyV1_FinCalc"
SCRIPT="Simulation/rocket_physics.py"
DMG_NAME="${APP_NAME}.dmg"
DIST_DIR="dist"
BUILD_DIR="build"

echo "==> Installing dependencies..."
pip3 install --quiet pyinstaller matplotlib

echo "==> Building .app bundle with PyInstaller..."
pyinstaller \
    --name "$APP_NAME" \
    --windowed \
    --onedir \
    --noconfirm \
    --clean \
    --add-data "Simulation/rocket_physics_defaults.json:Simulation" 2>/dev/null || \
pyinstaller \
    --name "$APP_NAME" \
    --windowed \
    --onedir \
    --noconfirm \
    --clean \
    "$SCRIPT"

echo "==> Creating .dmg..."
# Remove old dmg if present
rm -f "$DMG_NAME"

# Create a temporary directory for the dmg contents
DMG_STAGING="dmg_staging"
rm -rf "$DMG_STAGING"
mkdir -p "$DMG_STAGING"

# Copy .app bundle into staging
cp -R "${DIST_DIR}/${APP_NAME}.app" "$DMG_STAGING/" 2>/dev/null || \
cp -R "${DIST_DIR}/${APP_NAME}" "$DMG_STAGING/${APP_NAME}.app"

# Create a symlink to /Applications for drag-install
ln -s /Applications "$DMG_STAGING/Applications"

# Build the .dmg
hdiutil create \
    -volname "$APP_NAME" \
    -srcfolder "$DMG_STAGING" \
    -ov \
    -format UDZO \
    "$DMG_NAME"

# Cleanup
rm -rf "$DMG_STAGING"

echo ""
echo "==> Done! Created: $DMG_NAME"
echo "    Transfer this file to your Mac and double-click to install."
