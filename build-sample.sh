#!/usr/bin/env bash
# Build DJI's MSDK v5 sample app (com.dji.sampleV5.aircraft), injecting the
# app key from .env so it never gets written into the cloned DJI repo.
# Usage: ./build-sample.sh [install]
set -euo pipefail
cd "$(dirname "$0")"

set -a; source .env; set +a
: "${DJI_APP_KEY:?DJI_APP_KEY missing from .env}"

export JAVA_HOME="$(/usr/libexec/java_home -v 17)"
export ANDROID_HOME="$HOME/Library/Android/sdk"

cd Mobile-SDK-Android-V5/SampleCode-V5/android-sdk-v5-as
./gradlew :sample:assembleDebug -PAIRCRAFT_API_KEY="$DJI_APP_KEY"

APK="$(ls -t ../android-sdk-v5-sample/build/outputs/apk/debug/*.apk | head -1)"
echo "APK: $APK"

if [[ "${1:-}" == "install" ]]; then
  adb install -r "$APK"
fi
