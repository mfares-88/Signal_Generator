# SignalGen Companion — Android App

Android companion app for the ESP32-S3 automotive CKP/CMP signal generator.
Connects over Wi-Fi (TCP port 3333, NDJSON protocol) and provides a larger-screen
view of the board's HOME / CUSTOM / ADVANCED functionality plus an interactive
digital waveform preview.

## Prerequisites

- Android toolchain at `~/android-toolchain/` (JDK 17, Gradle 8.7, SDK 34)
- Source `~/android-toolchain/env.sh` to activate the environment

## Build (headless / CI)

```bash
source ~/android-toolchain/env.sh
cd android
./gradlew assembleDebug --no-daemon
```

Output APK: `app/build/outputs/apk/debug/app-debug.apk`

## Verify APK signing

```bash
source ~/android-toolchain/env.sh
apksigner verify --verbose app/build/outputs/apk/debug/app-debug.apk
```

Expected: `Verifies` (V2 signing).

## Run unit tests

```bash
source ~/android-toolchain/env.sh
cd android
./gradlew testDebugUnitTest --no-daemon
```

## Install on device

```bash
source ~/android-toolchain/env.sh
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

## Stack

| Component | Version |
|---|---|
| AGP | 8.5.2 |
| Kotlin | 1.9.24 |
| Compose Compiler | 1.5.14 |
| Compose BOM | 2024.06.00 |
| Gradle | 8.7 |
| compileSdk / targetSdk | 34 |
| minSdk | 29 (Android 10) |
| JDK | 17 |

## Wi-Fi connection flow

1. **SoftAP (first run):** Board broadcasts `SignalGen-XXXX` (WPA2). Join via the
   Connection screen — the app uses `WifiNetworkSpecifier` and binds the process
   to that network before opening the TCP socket.
2. **Provisioning:** Enter home SSID + password. The board ACKs immediately, then
   joins home Wi-Fi. The AP socket will drop — this is expected. The board's IP
   appears on the LCD label.
3. **Home network (subsequent runs):** The app discovers the board via mDNS
   (`_siggen._tcp`) or a manually entered IP read from the board's LCD.

## On-device verification (deferred — requires physical phone + board)

See `_3.User-Verification-Instructions/` in the project root.
