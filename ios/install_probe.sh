#!/bin/sh
# Build RC Probe and install it on the iPhone plugged into this Mac (Developer Mode on, "Trust this computer" accepted).
set -e
cd "$(dirname "$0")"
# Real devices only: xctrace lists them before "== Simulators ==", as "Name (iOS version) (UDID)".
UDID=$(xcrun xctrace list devices 2>/dev/null | awk '/== Simulators ==/ {exit} /iPhone/ {gsub(/[()]/, "", $NF); print $NF; exit}')
[ -n "$UDID" ] || { echo "No iPhone found. Plug it in, unlock it, tap Trust, and turn on Developer Mode."; exit 1; }
echo "iPhone $UDID"
xcodegen generate -q
xcodebuild -project FlightLab.xcodeproj -scheme RCProbe -destination "id=$UDID" -derivedDataPath build -allowProvisioningUpdates -quiet build
xcrun devicectl device install app --device "$UDID" "build/Build/Products/Debug-iphoneos/RC Probe.app"
xcrun devicectl device process launch --device "$UDID" com.clarksaben.rcprobe \
  || echo "Installed. If it won't open: Settings > General > VPN & Device Management > trust the developer."
