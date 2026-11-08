#!/usr/bin/env python3
"""Fast Flight Lab update mirror on the Mac (the phone reaches the Mac directly; cradle goes via a
Tailscale relay). Serves planner/dl-mirror on the Mac's tailnet IP. Run by launchd
(~/Library/LaunchAgents/com.flightlab.mirror.plist); publish_apk.sh fills the folder."""
import http.server
import os
import sys
import time

DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dl-mirror")
PORT = 8989   # local only; https://cradle-m1.tailb1b51d.ts.net:8990 via tailscale serve


class H(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                      ".apk": "application/vnd.android.package-archive", ".json": "application/json"}

    def __init__(self, *a, **k):
        super().__init__(*a, directory=DIR, **k)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


IP = "127.0.0.1"   # tailscale serve exposes it as https://cradle-m1.tailb1b51d.ts.net:8990

try:
    srv = http.server.ThreadingHTTPServer((IP, PORT), H)
except OSError:
    time.sleep(30)   # Tailscale not up yet; launchd restarts us
    sys.exit(1)
srv.serve_forever()
