"""Video library for the app's Videos tab: recordings that reached cradle (data/raw/<date>/{drone,phone}/),
small 720p previews for playback over the tailnet, thumbnails, titles, and "generate a splat from this"
requests. Splats are only ever made on request: a job is forwarded to dji-rl's splat service
(POST {SIM}/api/sim/splat/jobs) and queued here until that service answers.

Layout: videos/meta.json (titles, hidden, jobs), videos/cache/<id>.{mp4,jpg,json} (previews, probes).
"""
from __future__ import annotations

import csv
import hashlib
import json
import subprocess
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

RAW = Path.home() / "Code" / "dji-rl" / "data" / "raw"
EXTS = {".h264", ".h265", ".mp4", ".mov"}


class Videos:
    def __init__(self, root: Path, sim: str):
        self.root, self.sim = root, sim
        self.cache = root / "cache"
        self.cache.mkdir(parents=True, exist_ok=True)
        self.meta_f = root / "meta.json"
        self.lock = threading.Lock()
        self.work: list[str] = []          # ids waiting for a preview
        self.work_cv = threading.Condition()
        self.failed: dict[str, str] = {}
        threading.Thread(target=self._previewer, daemon=True, name="video-previews").start()
        threading.Thread(target=self._jobs_loop, daemon=True, name="splat-jobs").start()

    # ------------------------------------------------------------ metadata
    def _meta(self) -> dict:
        try:
            return json.loads(self.meta_f.read_text())
        except Exception:
            return {"videos": {}, "jobs": []}

    def _save(self, m: dict):
        tmp = self.meta_f.with_suffix(".tmp")
        tmp.write_text(json.dumps(m, indent=1))
        tmp.replace(self.meta_f)

    # ------------------------------------------------------------ scan
    def _files(self) -> dict[str, Path]:
        out = {}
        if RAW.exists():
            for f in RAW.glob("*/*/*"):
                if f.suffix.lower() in EXTS and f.parent.name in ("drone", "phone") and f.stat().st_size > 200_000:
                    out[hashlib.sha1(str(f.relative_to(RAW)).encode()).hexdigest()[:12]] = f
        return out

    def path(self, vid: str) -> Path | None:
        return self._files().get(vid)

    def _telemetry(self, f: Path) -> list[Path]:
        """App recordings: flightlab_<stamp>.stream.h264 pairs with telemetry/flightlab_<stamp>*.csv."""
        if not f.name.startswith("flightlab_"):
            return []
        stamp = f.name.split(".")[0]
        return sorted((f.parent.parent / "telemetry").glob(f"{stamp}*.csv"))

    def _fps(self, f: Path) -> float:
        for t in self._telemetry(f):
            if t.name.endswith(".video.csv"):
                try:
                    with t.open() as fh:
                        r = csv.reader(fh)
                        next(r, None)
                        for row in r:
                            if len(row) >= 8 and float(row[7]) > 0:
                                return float(row[7])
                except Exception:
                    pass
        return 30.0

    def _probe(self, vid: str, f: Path) -> dict:
        c = self.cache / f"{vid}.json"
        if c.exists():
            return json.loads(c.read_text())
        src = self.cache / f"{vid}.mp4"
        if not src.exists() and f.suffix.lower() in (".h264", ".h265"):
            return {}   # raw stream: no container to read a duration from until the preview exists
        target = src if src.exists() else f
        info = {}
        try:
            r = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                                "stream=width,height:format=duration", "-of", "json", str(target)],
                               capture_output=True, text=True, timeout=30)
            j = json.loads(r.stdout or "{}")
            s = (j.get("streams") or [{}])[0]
            info = {"width": s.get("width"), "height": s.get("height"),
                    "duration": round(float(j.get("format", {}).get("duration") or 0), 1) or None}
        except Exception:
            pass
        if src.exists() and info.get("duration"):   # previews are final; raw streams only get a duration via their preview
            c.write_text(json.dumps(info))
        return info

    def _upstream_jobs(self) -> list[dict]:
        """Splat jobs started elsewhere (e.g. on cradle directly), so their videos show them too. Cached 10 s."""
        now = time.time()
        if now - getattr(self, "_up_t", 0) > 10:
            try:
                self._up = self._get("/api/sim/splat/jobs")
            except Exception:
                self._up = getattr(self, "_up", [])
            self._up_t = now
        return self._up

    def list(self) -> list[dict]:
        m = self._meta()
        jobs = list(m.get("jobs", []))
        mine = {j.get("upstream") for j in jobs}
        root = RAW.parent.parent
        for u in self._upstream_jobs():
            if u.get("job") in mine:
                continue
            vids = []
            for v in u.get("videos", []) + [s_.get("video", "") for s_ in u.get("sources", [])]:
                pth = Path(v) if v.startswith("/") else root / v
                try:
                    vids.append(hashlib.sha1(str(pth.relative_to(RAW)).encode()).hexdigest()[:12])
                except ValueError:
                    pass
            jobs.append({"id": u.get("job"), "videos": vids, "name": u.get("name"), "method": u.get("method"), "state": u.get("state"),
                         "message": f"{u.get('stage')}: {u.get('message') or ''}" if u.get("state") == "running" else u.get("message"),
                         "progress": u.get("progress"), "scene": u.get("scene"), "created": u.get("started")})
        out = []
        for vid, f in self._files().items():
            vm = m["videos"].get(vid, {})
            if vm.get("hidden"):
                continue
            st = f.stat()
            info = self._probe(vid, f)
            prev = self.cache / f"{vid}.mp4"
            out.append({
                "id": vid, "file": f.name, "date": f.parent.parent.name, "source": f.parent.name,
                "size": st.st_size, "mtime": int(st.st_mtime), **info,
                "title": vm.get("title") or "", "notes": vm.get("notes") or "",
                "preview": "ready" if prev.exists() else ("failed" if vid in self.failed else "pending"),
                "telemetry": len(self._telemetry(f)) > 0,
                "splats": [{k: j.get(k) for k in ("id", "name", "method", "state", "message", "progress", "scene", "created")}
                           for j in jobs if vid in j.get("videos", [])],
            })
        out.sort(key=lambda v: v["mtime"], reverse=True)
        return out

    def update(self, vid: str, patch: dict) -> dict:
        with self.lock:
            m = self._meta()
            vm = m["videos"].setdefault(vid, {})
            for k in ("title", "notes", "hidden"):
                if k in patch:
                    vm[k] = patch[k] if k == "hidden" else str(patch[k])[:200]
            self._save(m)
            return vm

    # ------------------------------------------------------------ previews (720p H.264 with faststart, for phones)
    def preview(self, vid: str) -> Path | None:
        p = self.cache / f"{vid}.mp4"
        if p.exists():
            return p
        with self.work_cv:
            if vid not in self.work and vid not in self.failed:
                self.work.append(vid)
                self.work_cv.notify()
        return None

    def thumb(self, vid: str) -> Path | None:
        t = self.cache / f"{vid}.jpg"
        if t.exists():
            return t
        src = self.preview(vid)
        if not src:
            return None
        subprocess.run(["ffmpeg", "-y", "-v", "error", "-ss", "2", "-i", str(src), "-frames:v", "1",
                        "-vf", "scale=480:-2", str(t)], timeout=60)
        return t if t.exists() else None

    def _previewer(self):
        while True:
            with self.work_cv:
                while not self.work:
                    self.work_cv.wait()
                vid = self.work[0]
            f = self.path(vid)
            out = self.cache / f"{vid}.mp4"
            if f and not out.exists():
                tmp = self.cache / f"{vid}.part.mp4"
                raw = f.suffix.lower() in (".h264", ".h265")
                cmd = ["ffmpeg", "-y", "-v", "error"] + (["-fflags", "+genpts", "-r", f"{self._fps(f):g}"] if raw else []) + [
                    "-i", str(f), "-vf", "scale=-2:'min(720,ih)'", "-c:v", "h264_nvenc", "-preset", "p4", "-b:v", "4M",
                    "-an", "-movflags", "+faststart", str(tmp)]
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=3600)
                if r.returncode != 0:   # no NVENC free: software fallback
                    cmd[cmd.index("h264_nvenc")] = "libx264"
                    cmd[cmd.index("p4")] = "veryfast"
                    r = subprocess.run(cmd, capture_output=True, text=True, timeout=3600)
                if r.returncode == 0 and tmp.exists() and tmp.stat().st_size > 0:
                    tmp.replace(out)
                else:
                    self.failed[vid] = (r.stderr or "")[-300:]
                    tmp.unlink(missing_ok=True)
            with self.work_cv:
                if self.work and self.work[0] == vid:
                    self.work.pop(0)

    # ------------------------------------------------------------ splat jobs
    def request_splat(self, vid: str, name: str, method: str) -> dict:
        f = self.path(vid)
        if not f:
            raise KeyError("no such video")
        job = {"id": hashlib.sha1(f"{vid}{name}{time.time()}".encode()).hexdigest()[:10], "videos": [vid], "name": name,
               "method": method, "state": "queued", "message": "Waiting for cradle's splat service",
               "created": time.strftime("%Y-%m-%dT%H:%M:%S"), "upstream": None, "progress": 0,
               "paths": [str(f.relative_to(RAW.parent.parent))], "telemetry": [str(t.relative_to(RAW.parent.parent)) for t in self._telemetry(f)]}
        with self.lock:
            m = self._meta()
            m.setdefault("jobs", []).append(job)
            self._save(m)
        self._tick()
        return job

    def jobs(self) -> list[dict]:
        return self._meta().get("jobs", [])

    def _post(self, path, body):
        req = urllib.request.Request(self.sim + path, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=15) as r:
            return json.loads(r.read())

    def _get(self, path):
        with urllib.request.urlopen(self.sim + path, timeout=15) as r:
            return json.loads(r.read())

    def _tick(self):
        with self.lock:
            m = self._meta()
            changed = False
            for j in m.get("jobs", []):
                try:
                    if j["state"] == "queued" and not j.get("upstream"):
                        r = self._post("/api/sim/splat/jobs", {"videos": j["paths"], "telemetry": j["telemetry"], "name": j["name"], "method": j["method"]})
                        j["upstream"], j["message"] = r.get("job"), "Sent to cradle's splat service"
                        changed = True
                    elif j.get("upstream") and j["state"] not in ("done", "failed"):
                        r = self._get(f"/api/sim/splat/jobs/{j['upstream']}")
                        if r.get("stage") and r.get("state") == "running":
                            r["message"] = f"{r['stage']}: {r.get('message') or ''}".strip(": ")
                        for k in ("state", "message", "progress", "scene", "warnings", "psnr_db"):
                            if k in r and r[k] != j.get(k):
                                j[k] = r[k]
                                changed = True
                except urllib.error.HTTPError as e:
                    if e.code in (400, 409) and not j.get("upstream"):   # bad name / name taken: tell the user, don't retry
                        try:
                            why = json.loads(e.read()).get("error") or e.reason
                        except Exception:
                            why = e.reason
                        j["state"], j["message"], changed = "failed", str(why)[:160], True
                    elif j.get("message") != f"Retrying: HTTP {e.code}":
                        j["message"], changed = f"Retrying: HTTP {e.code}", True
                except Exception as e:   # service not there yet (404) or busy: try again later
                    msg = "Waiting for cradle's splat service" if "404" in str(e) or "refused" in str(e) else f"Retrying: {e}"[:120]
                    if j.get("message") != msg and j["state"] == "queued":
                        j["message"], changed = msg, True
            if changed:
                self._save(m)

    def _jobs_loop(self):
        while True:
            time.sleep(20)
            try:
                self._tick()
            except Exception:
                pass
