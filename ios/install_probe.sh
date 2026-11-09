#!/bin/sh
# Build RC Probe and install it on the iPhone plugged into this Mac (Developer Mode on, "Trust this computer" accepted).
set -e
cd "$(dirname "$0")"
DEV=$(xcrun devicectl list devices 2>/dev/null | awk '/iPhone/ && /available|connected/ {for(i=1;i<=NF;i++) if ($i ~ /^[0-9A-F]{8}-[0-9A-F-]{20,}$/) {print $i; exit}}')
[ -n "$DEV" ] || { echo "No iPhone found. Plug it in, unlock it, tap Trust, and turn on Developer Mode."; xcrun devicectl list devices; exit 1; }
xcodegen generate -q
xcodebuild -project FlightLab.xcodeproj -scheme RCProbe -destination "id=$(xcrun devicectl device info details --device "$DEV" 2>/dev/null | awk -F': ' '/udid/ {print $2; exit}')" \
  -derivedDataPath build -allowProvisioningUpdates -quiet build
xcrun devicectl device install app --device "$DEV" "build/Build/Products/Debug-iphoneos/RC Probe.app"
xcrun devicectl device process launch --device "$DEV" com.clarksaben.rcprobe || echo "Installed. Open RC Probe on the phone (Settings > General > VPN & Device Management may ask you to trust the developer first)."
