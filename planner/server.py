#!/usr/bin/env python3
"""Mission planner backend: static page, mission store, cached OSM building proxy.

GET  /                         planner page
GET  /api/missions             list saved missions (newest first)
GET  /api/missions/latest      the most recently saved mission (what the phone loads)
GET  /api/missions/<name>      one mission
POST /api/missions             save a mission (JSON body) -> {"name": ...}
GET  /api/buildings?bbox=s,w,n,e   OSM buildings in bbox (cached; tries several Overpass mirrors)
GET  /api/lib/<routes|places>          list saved items (newest first)
GET  /api/lib/<kind>/<slug>            one item
POST /api/lib/<kind>                   save {name, ...} (same name overwrites) -> {slug}
DELETE /api/lib/<kind>/<slug>          delete one

Run: python3 server.py --host 127.0.0.1 --port 8968   (fronted by `tailscale serve` HTTPS)
"""
import argparse
import hashlib
import json
import re
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
STATIC = HERE / "static"
APPLOGS = HERE / "applogs"   # diagnostics posted by the Flight Lab app
DL = HERE / "dl"          # Flight Lab APK + version.json (published by tools/publish_apk.sh)
MISSIONS = HERE / "missions"
CACHE = HERE / "cache"
CALIB = HERE / "calib"
LIB = HERE / "library"          # saved routes and places: library/<kind>/<slug>.json
LIB_KINDS = ("routes", "places", "races")
OVERPASS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
]
NAME_RE = re.compile(r"[^A-Za-z0-9._-]")
SIM_UPSTREAM = "http://127.0.0.1:8967"  # dji-rl `djirl serve`
TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8", ".css": "text/css"}


def fetch_buildings(s, w, n, e):
    key = hashlib.sha1(f"{s:.5f},{w:.5f},{n:.5f},{e:.5f}".encode()).hexdigest()[:16]
    cached = CACHE / f"bld_{key}.json"
    if cached.exists():
        return json.loads(cached.read_text()), "cache"
    q = f'[out:json][timeout:60];(way["building"]({s},{w},{n},{e});relation["building"]({s},{w},{n},{e}););out tags geom;'
    body = urllib.parse.urlencode({"data": q}).encode()
    errors = []
    for attempt in range(2):
        for url in OVERPASS:
            try:
                req = urllib.request.Request(url, data=body, headers={"User-Agent": "dji-planner/0.1 (personal drone route planner)"})
                with urllib.request.urlopen(req, timeout=70) as r:
                    data = json.loads(r.read())
                CACHE.mkdir(exist_ok=True)
                cached.write_text(json.dumps(data))
                return data, url
            except Exception as ex:  # mirror busy / timeout: try the next one
                errors.append(f"{url}: {ex}")
        time.sleep(3 * (attempt + 1))
    raise RuntimeError("; ".join(errors[-4:]))


DL_PAGE = """<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Flight Lab</title><style>
:root{color-scheme:dark}body{margin:0;min-height:100vh;display:flex;align-items:center;justify-content:center;background:#0d0f12;color:#e8eaed;font:16px -apple-system,Roboto,sans-serif}
.c{max-width:420px;padding:28px;text-align:center}h1{font-size:26px;font-weight:600;margin:0 0 6px}.v{color:#9aa0a6;margin-bottom:26px}
a.b{display:inline-block;padding:14px 30px;border-radius:16px;background:#ffffff1a;border:1px solid #ffffff2e;color:#e8eaed;text-decoration:none;font-weight:600}
p{color:#9aa0a6;font-size:14px;line-height:1.5;margin-top:26px;text-align:left}.n{white-space:pre-wrap;color:#c4c7c5;font-size:14px;margin-top:18px}
</style></head><body><div class=c><h1>Flight Lab</h1><div class=v>{{VER}}</div>
<a class=b href="/dl/flightlab.apk">Download for Android</a><div style="margin-top:12px"><a href="https://cradle-m1.tailb1b51d.ts.net:8990/flightlab.apk" style="color:#9aa0a6;font-size:14px">Faster: from the Mac mirror</a></div><div class=n>{{NOTES}}</div>
<p>Open the downloaded file and allow installs from your browser when asked. After that the app updates itself: it checks here on launch, and Settings has a Check for updates button. The sandbox inside it loads live from cradle, so web changes show up on the next open without reinstalling.</p></div></body></html>"""


# ---- FAA airspace (public FAA ArcGIS services) for recreational (Part 107 exception, 44809) flying ----
FAA = "https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/arcgis/rest/services/"
_air_cache = {}
CLASS_RANK = {"B": 0, "C": 1, "D": 2, "E": 3}


def _faa(svc, lat, lon, fields="*"):
    q = urllib.parse.urlencode(dict(geometry=f"{lon},{lat}", geometryType="esriGeometryPoint", inSR=4326,
                                    spatialRel="esriSpatialRelIntersects", outFields=fields, returnGeometry="false", f="json"))
    with urllib.request.urlopen(FAA + svc + "/FeatureServer/0/query?" + q, timeout=15) as r:
        d = json.load(r)
    if "error" in d:
        raise RuntimeError(f"{svc}: {d['error'].get('message')}")
    return [f["attributes"] for f in d.get("features", [])]


def airspace(lat, lon):
    key = (round(lat, 4), round(lon, 4))
    hit = _air_cache.get(key)
    if hit and time.time() - hit[0] < 3600:
        return hit[1]
    notes, restricted = [], []
    surface = []
    for a in _faa("Class_Airspace", lat, lon, "NAME,CLASS,LOCAL_TYPE,LOWER_VAL,LOWER_CODE,UPPER_VAL"):
        # Only airspace that starts at the surface matters below 400 ft AGL.
        if (a.get("LOWER_VAL") or 0) == 0 and a.get("CLASS") in CLASS_RANK:
            surface.append(a)
    surface.sort(key=lambda a: CLASS_RANK[a["CLASS"]])
    grid = _faa("FAA_UAS_FacilityMap_Data", lat, lon)
    g = grid[0] if grid else None
    for svc, label in (("Special_Use_Airspace", "Special use airspace"), ("Prohibited_Areas", "Prohibited area"),
                       ("National_Defense_Airspace_TFR_Areas", "National defense TFR"),
                       ("Part_Time_National_Security_UAS_Flight_Restrictions", "National security UAS restriction")):
        try:
            for a in _faa(svc, lat, lon):
                restricted.append(f"{label}: {a.get('NAME') or a.get('Name') or a.get('name') or 'unnamed'}")
        except Exception as e:  # one service down shouldn't hide the rest
            notes.append(f"Couldn't check {label.lower()} ({e}).")
    cls = surface[0]["CLASS"] if surface else "G"
    controlled = cls in CLASS_RANK
    ceiling_ft = 400
    if controlled:
        grid_ft = int(g["CEILING"]) if g and g.get("CEILING") is not None else 0
        ceiling_ft = min(400, grid_ft)
        airports = [g.get(f"APT{i}_ICAO") or g.get(f"APT{i}_FAAID") for i in (1, 2, 3, 4, 5)] if g else []
        airports = [a for a in airports if a]
        laanc = bool(g and any(g.get(f"APT{i}_LAANC") for i in (1, 2, 3, 4, 5)))
        if ceiling_ft > 0 and laanc:
            verdict, level = f"Class {cls} controlled airspace: get a LAANC authorization first (up to {ceiling_ft} ft here).", "warn"
        elif ceiling_ft > 0:
            verdict, level = f"Class {cls} controlled airspace: needs FAA DroneZone authorization (grid {ceiling_ft} ft, no LAANC).", "bad"
        else:
            verdict, level = f"Class {cls} controlled airspace with a 0 ft grid: no automatic authorization here.", "bad"
    else:
        airports, laanc = [], False
        verdict, level = "Class G uncontrolled airspace: recreational flying up to 400 ft AGL, no authorization needed.", "ok"
    if restricted:
        level = "bad"
        verdict = "Restricted: " + "; ".join(restricted) + ". Don't fly here."
    out = {
        "class": cls, "controlled": controlled, "surface_areas": [a["NAME"] for a in surface],
        "grid_ceiling_ft": int(g["CEILING"]) if g and g.get("CEILING") is not None else None,
        "ceiling_ft": ceiling_ft, "ceiling_m": round(ceiling_ft * 0.3048, 1), "laanc": laanc, "airports": airports,
        "restricted": restricted, "level": level, "verdict": verdict, "notes": notes,
        "rules": ["Recreational: max 400 ft AGL (lower if your authorization says so)", "Keep it in visual line of sight",
                  "Pass TRUST and carry the certificate", "Yield to manned aircraft; not over people or moving vehicles",
                  "Remote ID: the Mini 3 under 250 g needs none for recreational flying", "Check TFRs on the day (B4UFLY / Aloft)"],
        "checked": time.time(), "source": "FAA UAS Facility Maps + Class Airspace (ArcGIS)",
    }
    _air_cache[key] = (time.time(), out)
    return out


def _scene_geo(name):
    with urllib.request.urlopen(SIM_UPSTREAM + "/api/sim/scenes", timeout=10) as r:
        d = json.load(r)
    for s in (d["scenes"] if isinstance(d, dict) else d):
        if s["name"] == name and (s.get("geo") or {}).get("lat0") is not None:
            return s["geo"]
    return None


class H(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _proxy(self, method):
        """Forward /sim/* to the dji-rl sim server (plain HTTP on localhost) so the HTTPS page can use it."""
        target = SIM_UPSTREAM + self.path[len("/sim"):] or "/"
        n = int(self.headers.get("Content-Length", "0") or 0)
        body = self.rfile.read(n) if n else None
        req = urllib.request.Request(target, data=body, method=method)
        if self.headers.get("Content-Type"):
            req.add_header("Content-Type", self.headers["Content-Type"])
        try:
            with urllib.request.urlopen(req, timeout=180) as r:
                data, code, hdrs = r.read(), r.status, r.headers
        except urllib.error.HTTPError as e:
            data, code, hdrs = e.read(), e.code, e.headers
        except Exception as e:
            return self._send(502, {"error": f"sim server unavailable: {e}"})
        self.send_response(code)
        for k, v in hdrs.items():
            if k.lower() not in ("transfer-encoding", "connection", "content-length", "server", "date"):
                self.send_header(k, v)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _dl(self, p):
        name = p[len("/dl/"):] if p.startswith("/dl/") else ""
        if name in ("", "index.html"):
            try:
                v = json.loads((DL / "version.json").read_text())
            except Exception:
                v = None
            body = DL_PAGE.replace("{{VER}}", f"{v['versionName']} &middot; {v.get('size', 0) / 1e6:.0f} MB" if v else "not published yet")
            body = body.replace("{{NOTES}}", (v or {}).get("notes", "").replace("<", "&lt;"))
            return self._send(200, body, "text/html; charset=utf-8")
        f = DL / Path(name).name
        if not f.is_file():
            return self._send(404, {"error": "not found"})
        if f.suffix == ".json":
            return self._send(200, f.read_bytes())
        try:  # never hand out an APK that is still uploading (its size must match version.json)
            want = json.loads((DL / "version.json").read_text()).get("size")
        except Exception:
            want = None
        if want is None or f.stat().st_size != want:
            return self._send(503, "Flight Lab is being published right now. Try again in a few minutes.", "text/plain; charset=utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/vnd.android.package-archive")
        self.send_header("Content-Length", str(f.stat().st_size))
        self.send_header("Content-Disposition", 'attachment; filename="FlightLab.apk"')
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        with f.open("rb") as fh:
            while chunk := fh.read(1 << 20):
                self.wfile.write(chunk)

    def _lib(self, method):
        parts = urlparse(self.path).path.split("/")[3:]   # /api/lib/<kind>[/<slug>]
        kind = parts[0] if parts else ""
        if kind not in LIB_KINDS:
            return self._send(404, {"error": "unknown library"})
        d = LIB / kind
        slug = NAME_RE.sub("_", parts[1])[:60] if len(parts) > 1 and parts[1] else ""
        if method == "GET" and not slug:
            items = []
            for f in sorted(d.glob("*.json"), key=lambda f: f.stat().st_mtime, reverse=True) if d.exists() else []:
                try:
                    j = json.loads(f.read_text())
                except Exception:
                    continue
                items.append({"slug": f.stem, "name": j.get("name", f.stem), "scene": j.get("scene"), "saved": j.get("saved"), "kind": j.get("kind"),
                              "points": len(j.get("pts", [])) if kind == "routes" else None,
                              "gates": len(j.get("gates", [])) if kind == "races" else None,
                              "best_s": (j.get("best") or {}).get("time_s") if kind == "races" else None,
                              "ai": j.get("ai") if kind == "races" else None})
            return self._send(200, items)
        if method == "GET":
            f = d / f"{slug}.json"
            return self._send(200, f.read_bytes()) if f.exists() else self._send(404, {"error": "not found"})
        if method == "DELETE":
            f = d / f"{slug}.json"
            if f.exists():
                f.unlink()
            return self._send(200, {"deleted": slug})
        try:
            n = int(self.headers.get("Content-Length", "0"))
            assert n < 2_000_000
            j = json.loads(self.rfile.read(n))
            name = str(j.get("name", "")).strip()[:60]
            assert name and isinstance(j, dict)
        except Exception:
            return self._send(400, {"error": "need a JSON object with a name"})
        slug = NAME_RE.sub("_", name.lower()).strip("_")[:60] or "item"
        j["name"], j["saved"] = name, time.strftime("%Y-%m-%dT%H:%M:%S")
        d.mkdir(parents=True, exist_ok=True)
        (d / f"{slug}.json").write_text(json.dumps(j, indent=2))
        return self._send(200, {"slug": slug, "name": name})

    def do_DELETE(self):
        if urlparse(self.path).path.startswith("/api/lib/"):
            return self._lib("DELETE")
        return self._send(405, {"error": "method not allowed"})

    def do_GET(self):
        u = urlparse(self.path)
        p = u.path
        if p.startswith("/api/lib/"):
            return self._lib("GET")
        if p == "/api/airspace":
            q = dict(urllib.parse.parse_qsl(u.query))
            try:
                if q.get("lat") and q.get("lon"):
                    lat, lon = float(q["lat"]), float(q["lon"])
                else:
                    g = _scene_geo(q.get("scene", ""))
                    if not g:
                        return self._send(404, {"error": "scene has no GPS alignment"})
                    lat, lon = g["lat0"], g["lon0"]
                return self._send(200, airspace(lat, lon))
            except Exception as e:
                return self._send(502, {"error": f"FAA lookup failed: {e}"})
        if p == "/dl" or p.startswith("/dl/"):
            return self._dl(p)
        if p == "/sim" or p.startswith("/sim/"):
            return self._proxy("GET")
        if p == "/" or p.endswith((".html", ".js", ".css")):
            f = STATIC / ("index.html" if p == "/" else Path(p).name)
            if not f.exists():
                return self._send(404, {"error": "not found"})
            return self._send(200, f.read_bytes(), TYPES.get(f.suffix, "application/octet-stream"))
        if p == "/api/missions":
            items = sorted((m for m in MISSIONS.glob("*.json") if m.name != "latest.json"),
                           key=lambda m: m.stat().st_mtime, reverse=True)
            return self._send(200, [{"name": m.name, "saved": m.stat().st_mtime} for m in items])
        if p.startswith("/api/missions/"):
            name = NAME_RE.sub("_", Path(p).name)
            f = MISSIONS / (name if name.endswith(".json") else name + ".json")
            if not f.exists():
                return self._send(404, {"error": "no such mission"})
            return self._send(200, f.read_bytes())
        if p.startswith("/api/calib/"):
            f = CALIB / (NAME_RE.sub("_", Path(p).name) + ".json")
            return self._send(200, f.read_bytes() if f.exists() else b"{}")
        if p == "/api/buildings":
            try:
                s, w, n, e = (float(x) for x in parse_qs(u.query)["bbox"][0].split(","))
            except Exception:
                return self._send(400, {"error": "bbox=s,w,n,e required"})
            if (n - s) * (e - w) > 0.0004:  # ~2 km x 2 km cap
                return self._send(400, {"error": "area too large; zoom in or use fewer, closer pins"})
            try:
                data, src = fetch_buildings(s, w, n, e)
            except Exception as ex:
                return self._send(502, {"error": f"building data unavailable: {ex}"})
            data["_source"] = src
            return self._send(200, data)
        # Anything else belongs to the embedded sim viewer (it uses absolute paths like /frame).
        self.path = "/sim" + self.path
        return self._proxy("GET")

    def do_POST(self):
        if urlparse(self.path).path == "/api/applog":
            n = min(int(self.headers.get("Content-Length", "0") or 0), 8 << 20)
            APPLOGS.mkdir(exist_ok=True)
            f = APPLOGS / time.strftime("%Y%m%d_%H%M%S.txt")
            f.write_bytes(self.rfile.read(n))
            for old in sorted(APPLOGS.glob("*.txt"))[:-50]:
                old.unlink()
            return self._send(200, {"ok": True, "saved": f.name})
        if urlparse(self.path).path.startswith("/api/lib/"):
            return self._lib("POST")
        if urlparse(self.path).path.startswith("/sim/"):
            return self._proxy("POST")
        if urlparse(self.path).path.startswith("/api/calib/"):
            try:
                n = int(self.headers.get("Content-Length", "0"))
                c = json.loads(self.rfile.read(n))
                assert isinstance(c, dict) and 0.5 < float(c.get("scale_corr", 1)) < 2
            except Exception:
                return self._send(400, {"error": "invalid calibration"})
            CALIB.mkdir(exist_ok=True)
            c["saved"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            (CALIB / (NAME_RE.sub("_", Path(urlparse(self.path).path).name) + ".json")).write_text(json.dumps(c, indent=2))
            return self._send(200, c)
        if urlparse(self.path).path != "/api/missions":
            self.path = "/sim" + self.path
            return self._proxy("POST")
        try:
            n = int(self.headers.get("Content-Length", "0"))
            if n > 5_000_000:
                return self._send(413, {"error": "too large"})
            m = json.loads(self.rfile.read(n))
            assert m.get("version") == 1 and m.get("route") and m.get("home")
        except Exception:
            return self._send(400, {"error": "invalid mission json"})
        MISSIONS.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        name = f"{stamp}_{NAME_RE.sub('_', str(m.get('name') or 'mission'))[:40]}.json"
        m["saved_name"] = name
        text = json.dumps(m, indent=2)
        (MISSIONS / name).write_text(text)
        (MISSIONS / "latest.json").write_text(text)
        self._send(200, {"name": name})

    def log_message(self, fmt, *args):
        print("%s - %s" % (self.address_string(), fmt % args), flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8968)
    a = ap.parse_args()
    MISSIONS.mkdir(exist_ok=True)
    print(f"planner on http://{a.host}:{a.port}", flush=True)
    ThreadingHTTPServer((a.host, a.port), H).serve_forever()
