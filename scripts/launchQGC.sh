#!/bin/bash

if [[ "$(uname)" != "Linux" ]]; then
    exit 0
fi

ARCH=$(uname -m)
if [[ "$ARCH" != "x86_64" ]]; then
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

QGC_DIR="$HOME/Downloads"
QGC_APPIMAGE="$QGC_DIR/QGroundControl.AppImage"
mkdir -p "$QGC_DIR"

if [ -e "$QGC_APPIMAGE" ]; then
    chmod +x "$QGC_APPIMAGE"
    "$QGC_APPIMAGE" >/dev/null 2>&1 & 
else
    QGC_URL="https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage"
    curl -L "$QGC_URL" -o "$QGC_APPIMAGE"
    chmod +x "$QGC_APPIMAGE"
    "$QGC_APPIMAGE" >/dev/null 2>&1 & 
fi

echo "Launched QGC."