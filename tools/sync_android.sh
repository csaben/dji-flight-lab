#!/bin/bash
# Snapshot the Flight Lab Android code out of the DJI SDK clone (which isn't part of this repo):
#   android/flightlab/          our package, copied as-is
#   android/res/                our new resource files
#   android/sdk-sample.patch    our edits to DJI's sample app (apply inside Mobile-SDK-Android-V5)
set -euo pipefail
cd "$(dirname "$0")/.."
S=Mobile-SDK-Android-V5/SampleCode-V5/android-sdk-v5-sample/src/main
rm -rf android/flightlab android/res; mkdir -p android/flightlab android/res/xml
cp "$S"/java/dji/sampleV5/aircraft/flightlab/*.kt android/flightlab/
cp "$S"/res/xml/network_security_config.xml android/res/xml/
(cd Mobile-SDK-Android-V5 && git diff) > android/sdk-sample.patch
echo "synced: $(ls android/flightlab | wc -l | tr -d ' ') Kotlin files, patch $(wc -l < android/sdk-sample.patch | tr -d ' ') lines"
