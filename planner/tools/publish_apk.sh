#!/bin/bash
# Build Flight Lab and publish it to cradle so installed apps offer the update.
#   planner/tools/publish_apk.sh "what changed"        build + publish
#   planner/tools/publish_apk.sh --no-build "notes"    publish the last build
# Download page: https://cradle.tailb1b51d.ts.net:8444/dl/
# rsync onto the same file each time, so only the changed parts of the APK cross the tailnet.
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
SDK="$HERE/../Mobile-SDK-Android-V5/SampleCode-V5"
APK="$SDK/android-sdk-v5-sample/build/outputs/apk/debug/sample-debug.apk"
META="$SDK/android-sdk-v5-sample/build/outputs/apk/debug/output-metadata.json"
SSH="ssh -i $HOME/.ssh/openone_ed25519"
DEST="cradle:Code/dji-planner/dl"

build=1
if [[ "${1:-}" == "--no-build" ]]; then build=0; shift; fi
notes="${1:-}"

if (( build )); then
  export JAVA_HOME="$(/usr/libexec/java_home -v 17)" ANDROID_HOME="$HOME/Library/Android/sdk"
  # The DJI app key lives only in .env (never in the repo); without it the SDK can't register.
  DJI_APP_KEY="$(set -a; source "$HERE/../.env"; echo "${DJI_APP_KEY:-}")"
  [[ -n "$DJI_APP_KEY" ]] || { echo "DJI_APP_KEY missing from .env"; exit 1; }
  before=$(stat -f %m "$APK" 2>/dev/null || echo 0)
  if ! (cd "$SDK/android-sdk-v5-as" && ./gradlew :sample:assembleDebug -q -PAIRCRAFT_API_KEY="$DJI_APP_KEY" > /tmp/flightlab_build.log 2>&1); then
    grep -E "^e:|What went wrong" -A2 /tmp/flightlab_build.log | head -20; echo "BUILD FAILED, nothing published"; exit 1
  fi
  [[ $(stat -f %m "$APK") -gt $before ]] || { echo "APK was not rebuilt, nothing published"; exit 1; }
fi
[[ -f "$APK" ]] || { echo "no APK at $APK"; exit 1; }

read -r code name < <(python3 -c 'import json,sys; e=json.load(open(sys.argv[1]))["elements"][0]; print(e["versionCode"], e["versionName"])' "$META")
size=$(stat -f %z "$APK")
echo "Publishing Flight Lab $name ($code), $((size / 1000000)) MB"

$SSH cradle 'mkdir -p ~/Code/dji-planner/dl'
rsync -a --partial --progress -e "$SSH" "$APK" "$DEST/flightlab.apk"
# Fast mirror on this Mac (the phone reaches it directly; cradle goes via a Tailscale relay).
MIRROR="$HERE/dl-mirror"; mkdir -p "$MIRROR"
cp "$APK" "$MIRROR/flightlab.apk.part" && mv "$MIRROR/flightlab.apk.part" "$MIRROR/flightlab.apk"
# version.json last, so an app never sees a version whose APK isn't fully there yet.
VJSON=$(python3 -c 'import json,sys; print(json.dumps({"versionCode": int(sys.argv[1]), "versionName": sys.argv[2], "apk": "flightlab.apk", "size": int(sys.argv[3]), "notes": sys.argv[4], "mirrors": [{"name": "Mac", "url": "https://cradle-m1.tailb1b51d.ts.net:8990/flightlab.apk"}]}))' "$code" "$name" "$size" "$notes")
echo "$VJSON" > "$MIRROR/version.json"
echo "$VJSON" | $SSH cradle 'cat > ~/Code/dji-planner/dl/version.json'
echo "Done: https://cradle.tailb1b51d.ts.net:8444/dl/"
