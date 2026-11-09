#!/bin/sh
# Build StarSplat and upload it to TestFlight (build number is bumped automatically by App Store Connect).
set -e
cd "$(dirname "$0")"
xcodegen generate -q
rm -rf build/StarSplat.xcarchive build/export
xcodebuild -project FlightLab.xcodeproj -scheme FlightLab -configuration Release -destination 'generic/platform=iOS' \
  -archivePath build/StarSplat.xcarchive -allowProvisioningUpdates -quiet archive
xcodebuild -exportArchive -archivePath build/StarSplat.xcarchive -exportOptionsPlist ExportOptions.plist \
  -exportPath build/export -allowProvisioningUpdates 2>&1 | grep -E 'error|Upload succeeded|EXPORT'
