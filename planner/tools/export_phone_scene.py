"""Export a dji-rl scene for the on-phone sandbox (rendered in the browser, no server round trip).

Writes <out>/<scene>/:
    scene.splat   antimatter15 .splat (32 B/splat: pos f32x3, scale f32x3, rgba u8x4, quat u8x4 wxyz),
                  the same culled set the viewer renders (sky shell / haze removed), in scene ENU metres
    esdf.bin      gzip of uint8 SDF on a coarser grid, x fastest: sdf_m = (q - 40) / 10, capped at 3 m
    meta.json     grid shape/origin/voxel, spawn, camera, versions

Run with the dji-rl venv on cradle:
    ~/Code/dji-rl/.venv/bin/python tools/export_phone_scene.py house-full [--voxel 0.2] [--device cuda:0]
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

RL = Path.home() / "Code" / "dji-rl"
sys.path.insert(0, str(RL / "src"))
from djirl.scene import SH_C0  # noqa: E402
from djirl.sim_api import SimAPI  # noqa: E402

Q_SCALE, Q_ZERO, Q_MAX = 10.0, 40, 3.0   # 10 cm steps, -4 m .. +3 m (farther is just "clear")


def export(name: str, out: Path, voxel: float, device: str) -> dict:
    api = SimAPI(device)
    s = api.scene(name)
    g = api.view_gaussians(s)
    n = len(g.means)

    # ---- splat
    rgb = (g.sh[:, 0] * SH_C0 + 0.5).clamp(0, 1)
    rgba = torch.cat([rgb, g.opacities[:, None]], -1)
    # biggest / most opaque first: a partially streamed file still looks right
    order = torch.argsort(-(g.scales.prod(-1) * g.opacities))
    rec = np.zeros(n, dtype=[("p", "<f4", 3), ("s", "<f4", 3), ("c", "u1", 4), ("q", "u1", 4)])
    rec["p"] = g.means[order].cpu().numpy()
    rec["s"] = g.scales[order].cpu().numpy()
    rec["c"] = (rgba[order] * 255).round().clamp(0, 255).to(torch.uint8).cpu().numpy()
    q = torch.nn.functional.normalize(g.quats[order], dim=-1)
    rec["q"] = (q * 128 + 128).round().clamp(0, 255).to(torch.uint8).cpu().numpy()
    d = out / name
    d.mkdir(parents=True, exist_ok=True)
    (d / "scene.splat").write_bytes(rec.tobytes())

    # ---- collision grid: resample the ESDF at the coarser voxel (trilinear), quantise to uint8
    e = s.esdf
    lo, hi = e.origin, e.upper
    dims = (torch.floor((hi - lo) / voxel).long() + 1).tolist()
    xs, ys, zs = (torch.arange(k, device=e.sdf.device) * voxel + lo[i] for i, k in enumerate(dims))
    grid = np.empty((dims[2], dims[1], dims[0]), np.uint8)   # [z][y][x] -> x fastest in memory
    Y, X = torch.meshgrid(ys, xs, indexing="ij")
    for k, z in enumerate(zs):
        pts = torch.stack([X, Y, torch.full_like(X, float(z))], -1)
        v = e.query(pts).clamp(max=Q_MAX)
        grid[k] = (v * Q_SCALE + Q_ZERO).round().clamp(0, 255).to(torch.uint8).cpu().numpy()
    (d / "esdf.bin").write_bytes(gzip.compress(grid.tobytes(), 9))

    w = s.waypoints.cpu()
    p0, p1 = w[0], w[1 % len(w)]
    meta = {
        "scene": name, "splats": n, "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "esdf": {"dims": dims, "origin": [round(float(x), 4) for x in lo.tolist()], "voxel": voxel,
                 "q_scale": Q_SCALE, "q_zero": Q_ZERO, "max_m": Q_MAX, "layout": "uint8 gzip, x fastest then y then z"},
        "spawn": {"pos": [round(float(x), 3) for x in p0.tolist()],
                  "yaw": float(torch.atan2(p1[1] - p0[1], p1[0] - p0[0]))},
        "camera": {"hfov_deg": 82.0, "near": 0.05, "far": 200.0, "sky": [0.62, 0.75, 0.90]},
        "drone_r": 0.25,
    }
    meta["version"] = hashlib.sha1(((d / "scene.splat").read_bytes()[:1 << 20] + (d / "esdf.bin").read_bytes()[:1 << 20]
                                    + json.dumps(meta["esdf"]).encode())).hexdigest()[:12]
    (d / "meta.json").write_text(json.dumps(meta, indent=1))
    return {"splats": n, "splat_mb": round((d / "scene.splat").stat().st_size / 1e6, 1),
            "esdf_mb": round((d / "esdf.bin").stat().st_size / 1e6, 1), "dims": dims}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("scenes", nargs="+")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "phone"))
    ap.add_argument("--voxel", type=float, default=0.2)
    ap.add_argument("--device", default="cuda:0")
    a = ap.parse_args()
    for nm in a.scenes:
        t = time.time()
        print(nm, export(nm, Path(a.out), a.voxel, a.device), f"{time.time() - t:.0f}s", flush=True)
