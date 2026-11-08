// On-phone sandbox: renders the splat and simulates the drone in the browser, so flying doesn't wait on
// the network. Mirrors dji-rl's viewer Session (djirl/viewer.py) closely enough that the sandbox page can
// drive either one: same stick model (dynamics.py StickDynamics, 30 Hz, 200 ms delay), same ESDF contact
// test (drone radius 0.25 m, block + bounce), same replay semantics (/replay t=play&from= / t=<fixed>),
// same camera (pinhole, 82 deg HFOV, ENU, chase = 6 m back, 3 m up).
// Scene assets come from /phone/<scene>/ (planner/tools/export_phone_scene.py) and are cached on the phone.
import * as THREE from 'three';
import { SparkRenderer, SplatMesh } from '@sparkjsdev/spark';

const P = { v_h_max: 8, v_up_max: 3, v_down_max: 2.5, yaw_rate_max: 1.6, tau: 0.4, tau_z: 0.3, tau_yaw: 0.2, a_acc: 4, a_brake: 6, a_z: 4 };
const CONTROL_DT = 1 / 30, SUB = 2, DELAY = 6, DRONE_R = 0.25;
const wrap = a => ((a + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;

class Dynamics {
  constructor() { this.reset([0, 0, 0], 0); }
  reset(pos, yaw, vel) {
    this.pos = [...pos]; this.vel = vel ? [...vel] : [0, 0, 0]; this.yaw = yaw; this.yawRate = 0;
    this.buf = Array.from({ length: DELAY + 1 }, () => [0, 0, 0, 0]);
  }
  step(action) {
    this.buf.shift(); this.buf.push(action.map(v => Math.max(-1, Math.min(1, v))));
    const a = this.buf[0], c = Math.cos(this.yaw), s = Math.sin(this.yaw), dt = CONTROL_DT / SUB;
    const fwd = a[0] * P.v_h_max, right = a[1] * P.v_h_max;
    let cx = c * fwd + s * right, cy = s * fwd - c * right;
    const cn = Math.hypot(cx, cy); if (cn > P.v_h_max) { cx *= P.v_h_max / cn; cy *= P.v_h_max / cn; }
    const cz = a[2] >= 0 ? a[2] * P.v_up_max : a[2] * P.v_down_max, rCmd = -a[3] * P.yaw_rate_max;
    const subs = [];
    for (let i = 0; i < SUB; i++) {
      let ax = (cx - this.vel[0]) / P.tau, ay = (cy - this.vel[1]) / P.tau;
      const cap = (ax * this.vel[0] + ay * this.vel[1]) < 0 ? P.a_brake : P.a_acc, an = Math.hypot(ax, ay);
      if (an > cap) { ax *= cap / an; ay *= cap / an; }
      const az = Math.max(-P.a_z, Math.min(P.a_z, (cz - this.vel[2]) / P.tau_z));
      this.vel = [this.vel[0] + ax * dt, this.vel[1] + ay * dt, this.vel[2] + az * dt];
      this.yawRate += (rCmd - this.yawRate) / P.tau_yaw * dt;
      this.yaw = wrap(this.yaw + this.yawRate * dt);
      this.pos = [this.pos[0] + this.vel[0] * dt, this.pos[1] + this.vel[1] * dt, this.pos[2] + this.vel[2] * dt];
      subs.push(this.pos);
    }
    return subs;
  }
  bodyVel() {
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw), [vx, vy, vz] = this.vel;
    return [c * vx + s * vy, s * vx - c * vy, vz];
  }
}

class Esdf {
  constructor(meta, bytes) {
    const e = meta.esdf; this.d = e.dims; this.o = e.origin; this.v = e.voxel; this.q = bytes;
    this.qs = e.q_scale; this.qz = e.q_zero; this.max = e.max_m;
    this.upper = this.o.map((o, i) => o + this.v * (this.d[i] - 1));
  }
  inBounds(p) { return p.every((x, i) => x >= this.o[i] && x <= this.upper[i]); }
  // trilinear, border-clamped like torch grid_sample(padding_mode="border")
  query(x, y, z) {
    const [nx, ny, nz] = this.d, cl = (v, n) => Math.max(0, Math.min(n - 1, v));
    const fx = cl((x - this.o[0]) / this.v, nx), fy = cl((y - this.o[1]) / this.v, ny), fz = cl((z - this.o[2]) / this.v, nz);
    const x0 = Math.min(Math.floor(fx), nx - 2), y0 = Math.min(Math.floor(fy), ny - 2), z0 = Math.min(Math.floor(fz), nz - 2);
    const tx = fx - x0, ty = fy - y0, tz = fz - z0, sy = nx, sz = nx * ny, q = this.q;
    const at = (i, j, k) => q[(z0 + k) * sz + (y0 + j) * sy + x0 + i];
    const c00 = at(0, 0, 0) * (1 - tx) + at(1, 0, 0) * tx, c10 = at(0, 1, 0) * (1 - tx) + at(1, 1, 0) * tx;
    const c01 = at(0, 0, 1) * (1 - tx) + at(1, 0, 1) * tx, c11 = at(0, 1, 1) * (1 - tx) + at(1, 1, 1) * tx;
    const v = (c00 * (1 - ty) + c10 * ty) * (1 - tz) + (c01 * (1 - ty) + c11 * ty) * tz;
    return (v - this.qz) / this.qs;
  }
}

async function cached(url, onProgress) {
  let cache = null;
  try { cache = await caches.open('phonesim-v1'); const hit = await cache.match(url); if (hit) return new Uint8Array(await hit.arrayBuffer()); } catch {}
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`);
  const total = +res.headers.get('Content-Length') || 0, reader = res.body.getReader(), parts = []; let got = 0;
  for (;;) { const { done, value } = await reader.read(); if (done) break; parts.push(value); got += value.length; onProgress?.(got, total); }
  const out = new Uint8Array(got); let o = 0; for (const p of parts) { out.set(p, o); o += p.length; }
  try {
    if (cache) {
      for (const k of await cache.keys()) if (k.url.split('?')[0] === new URL(url, location.href).href.split('?')[0]) await cache.delete(k);   // drop old versions
      await cache.put(url, new Response(out, { headers: { 'Content-Type': 'application/octet-stream' } }));
    }
  } catch {}
  return out;
}
async function gunzip(bytes) {
  const s = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'));
  return new Uint8Array(await new Response(s).arrayBuffer());
}

export class PhoneSim {
  static trajs = new Map();   // replay id -> trajectory (the page registers /api/sim/fly results here)

  static async available(scene) {
    try { const r = await fetch(`/phone/${encodeURIComponent(scene)}/meta.json`, { cache: 'no-store' }); return r.ok ? await r.json() : null; } catch { return null; }
  }

  static async open(scene, canvas, onProgress) {
    const meta = await PhoneSim.available(scene);
    if (!meta) throw new Error(`${scene} isn't exported for the phone`);
    const base = `/phone/${encodeURIComponent(scene)}`, v = `?v=${meta.version}`;
    let a = 0, b = 0, at = 0, bt = 0; const prog = () => onProgress?.(a + b, at + bt);
    const [splat, esdfGz] = await Promise.all([
      cached(`${base}/scene.splat${v}`, (g, t) => { a = g; at = t; prog(); }),
      cached(`${base}/esdf.bin${v}`, (g, t) => { b = g; bt = t; prog(); }),
    ]);
    const sim = new PhoneSim(meta, canvas, splat, new Esdf(meta, await gunzip(esdfGz)));
    await sim.mesh.initialized;
    return sim;
  }

  constructor(meta, canvas, splatBytes, esdf) {
    this.meta = meta; this.scene = meta.scene; this.esdf = esdf; this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: false, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.25));
    this.three = new THREE.Scene();
    const sky = meta.camera.sky; this.three.background = new THREE.Color(sky[0], sky[1], sky[2]);
    this.camera = new THREE.PerspectiveCamera(60, 16 / 9, meta.camera.near, meta.camera.far);
    this.camera.up.set(0, 0, 1);
    this.spark = new SparkRenderer({ renderer: this.renderer });
    this.three.add(this.spark);
    this.mesh = new SplatMesh({ fileBytes: splatBytes, fileType: 'splat' });
    this.three.add(this.mesh);
    this.dyn = new Dynamics();
    this.mode = 'manual'; this.collisions = 0; this.inContact = false; this.flash = 0; this.acc = 0;
    this.replay = null; this.lastT = performance.now() / 1000;
    this.reset();
  }

  reset() { const s = this.meta.spawn; this.dyn.reset(s.pos, s.yaw); }

  // ---- the viewer's /mode and /replay endpoints
  setMode(mode) {
    if (mode !== 'manual') return;
    if (this.mode === 'replay' && this.replay) { const r = this.replayPose(); this.dyn.reset(r.pos, r.yaw, r.vel); }   // fly on from exactly where the replay is
    this.mode = 'manual'; this.acc = 0;
  }
  replayCall(q) {
    let id = q.get('id') || '';
    if (q.get('ghost')) id = `ghost:${q.get('ghost')}`;
    const t = q.get('t');
    if (t != null && this.mode === 'replay' && this.replay?.id === id) {
      this.replay.fixed = t === 'play' ? null : +t;
      if (t === 'play') this.replay.t0 = performance.now() / 1000 - +(q.get('from') || 0);
      return true;
    }
    const tr = PhoneSim.trajs.get(id) || window.simTrajs?.get(id);
    if (!tr) return false;
    this.replay = { ...tr, id, t0: performance.now() / 1000, fixed: null };
    this.mode = 'replay';
    return true;
  }
  replayPose() {
    const R = this.replay, tr = R.traj, dur = tr[tr.length - 1].t;
    let t = R.fixed != null ? R.fixed : (performance.now() / 1000 - R.t0) % (dur + 2);
    t = Math.min(Math.max(t, 0), dur);
    let lo = 0, hi = tr.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (tr[m].t <= t) lo = m; else hi = m; }
    const A = tr[lo], B = tr[Math.min(lo + 1, tr.length - 1)], w = B.t > A.t ? (t - A.t) / (B.t - A.t) : 0;
    const L = k => (A[k] ?? 0) * (1 - w) + (B[k] ?? A[k] ?? 0) * w;
    const yaw = A.yaw + w * wrap(B.yaw - A.yaw);
    const gp = A.gimbal_pitch_deg == null ? null : A.gimbal_pitch_deg * (1 - w) + (B.gimbal_pitch_deg ?? A.gimbal_pitch_deg) * w;
    return { pos: [L('x'), L('y'), L('z')], vel: [L('vx'), L('vy'), L('vz')], yaw, t, dur, gp, poi: A.poi ?? -1 };
  }

  stepManual(sticks, dt, noclip) {
    this.acc += dt;
    let touching = false, n = 0;
    while (this.acc >= CONTROL_DT && n < 15) {
      this.acc -= CONTROL_DT; n++;
      const prev = this.dyn.pos, prevV = this.dyn.vel, subs = this.dyn.step(sticks);
      const d0 = this.esdf.query(prev[0], prev[1], prev[2]), d1 = Math.min(...subs.map(p => this.esdf.query(p[0], p[1], p[2])));
      const out = !this.esdf.inBounds(this.dyn.pos);
      if (d1 < DRONE_R || out) {
        touching = true;
        if (!this.inContact) { this.collisions++; this.inContact = true; }
        this.flash = performance.now();
        // Solid: block only moves that go deeper (or leave the map), so a drone caught in a bad patch of
        // the collision mesh can always back or climb out. Pass-through (noclip) never blocks.
        if (!noclip && (out || d1 < d0 - 0.01)) { this.dyn.pos = prev; this.dyn.vel = prevV.map(v => -0.3 * v); this.acc = 0; break; }
      }
    }
    if (n && !touching) this.inContact = false;
  }

  /** One frame: advance, render, return the same state object the viewer sends in X-State. */
  frame(sticks, { pitch = -10, cam = 'fpv', noclip = false } = {}) {
    const now = performance.now() / 1000, dt = Math.min(now - this.lastT, 0.5); this.lastT = now;
    let replayInfo = null;
    if (this.mode === 'replay' && this.replay) {
      const r = this.replayPose();
      this.dyn.pos = r.pos; this.dyn.vel = r.vel; this.dyn.yaw = r.yaw;
      replayInfo = { id: this.replay.id, t: +r.t.toFixed(2), duration: r.dur, paused: this.replay.fixed != null, poi: r.poi, gimbal_pitch_deg: r.gp,
        ...(this.replay.race_t0 != null ? { race_t: +(r.t + this.replay.race_t0).toFixed(2), time_s: this.replay.time_s } : {}) };
      if (r.gp != null) pitch = r.gp;
    } else {
      this.stepManual(sticks, dt, noclip);
    }
    this.render(pitch, cam);
    const [x, y, z] = this.dyn.pos;
    let agl = null;
    for (let d = 0; d <= 30; d += 0.25) if (this.esdf.query(x, y, z - d) < 0.1) { agl = d; break; }
    const clr = this.esdf.query(x, y, z);
    return {
      pos: [x, y, z].map(v => +v.toFixed(2)), yaw_deg: +(this.dyn.yaw * 180 / Math.PI).toFixed(1),
      speed: +Math.hypot(...this.dyn.vel).toFixed(2), vel_body: this.dyn.bodyVel().map(v => +v.toFixed(2)),
      agl, clearance: +clr.toFixed(2), clearance_capped: clr >= this.esdf.max - 0.05,
      collisions: this.collisions, mode: this.mode, waypoint: 0, wp_dist: 0, scene: this.scene,
      replay: replayInfo, cam, local: true, flash: performance.now() - this.flash < 250,
    };
  }

  render(pitchDeg, cam) {
    const W = this.canvas.clientWidth || 1, H = this.canvas.clientHeight || 1;
    if (this.canvas.width !== Math.round(W * this.renderer.getPixelRatio()) || this.canvas.height !== Math.round(H * this.renderer.getPixelRatio())) this.renderer.setSize(W, H, false);
    // keep the viewer's 82 deg horizontal FOV whatever the screen shape
    const hf = this.meta.camera.hfov_deg * Math.PI / 180, aspect = W / H;
    this.camera.aspect = aspect; this.camera.fov = 2 * Math.atan(Math.tan(hf / 2) / aspect) * 180 / Math.PI;
    this.camera.updateProjectionMatrix();
    const yaw = this.dyn.yaw; let p = this.dyn.pos, pitch = pitchDeg * Math.PI / 180;
    if (cam === 'chase') { p = [p[0] - 6 * Math.cos(yaw), p[1] - 6 * Math.sin(yaw), p[2] + 3]; pitch = -Math.atan2(3, 6); }
    this.camera.position.set(p[0], p[1], p[2]);
    this.camera.lookAt(p[0] + Math.cos(yaw) * Math.cos(pitch), p[1] + Math.sin(yaw) * Math.cos(pitch), p[2] + Math.sin(pitch));
    this.renderer.render(this.three, this.camera);
  }

  dispose() { try { this.renderer.setAnimationLoop(null); this.renderer.dispose(); this.mesh.dispose?.(); } catch {} }
}
window.PhoneSim = PhoneSim;
window.dispatchEvent(new Event('phonesim-ready'));
