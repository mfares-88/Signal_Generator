#!/usr/bin/env bash
# SignalGen Companion — headless debug build
# Usage: ./build.sh [additional gradlew args...]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Activate the Android toolchain (JDK 17, Gradle 8.7, SDK 34)
# shellcheck source=/dev/null
source ~/android-toolchain/env.sh

cd "$SCRIPT_DIR"

echo "=== Building SignalGen Companion (assembleDebug) ==="
./gradlew assembleDebug --no-daemon "$@"

APK="$SCRIPT_DIR/app/build/outputs/apk/debug/app-debug.apk"
if [ -f "$APK" ]; then
    echo ""
    echo "=== Build succeeded: $APK ==="
    echo "=== Verifying APK signing ==="
    apksigner verify --verbose "$APK"
else
    echo "ERROR: APK not found at $APK" >&2
    exit 1
fi
