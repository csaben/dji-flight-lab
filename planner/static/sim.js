// House sim: pick A and B on a top-down render of the splat, preview the planned path and the
// simulated flight (non-learned controller + the sim's stick dynamics), then allow launch.
// Talks to dji-rl's `djirl serve` through the planner's /sim proxy.
(function () {
  const $ = id => document.getElementById(id);
  const S = {
    scenes: [], scene: null, meta: null, img: null, occ: null,
    a: null, b: null, via: [], pois: [], tool: 'ab', measure: [], calib: { scale_corr: 1, yaw_trim_deg: 0, measured: false, heading_checked: false }, preview: null, previewKey: null, t: 0, playing: false, busy: false,
  };

  function key() {
    return JSON.stringify([S.scene, S.a, S.via, S.b, // points carry their own alt
       $('simLoop').checked, +$('simAlt').value, +$('simSpeed').value, +$('simClear').value, $('simAir').checked, S.pois, $('simPoiMode').value, $('simStyle').value, +$('simFpvTilt').value, $('simUseMargin').checked, S.calib]);
  }

  function say(msg, cls = '') { const s = $('simStatus'); s.textContent = msg; s.className = cls; }

  async function j(url, opts) {
    const r = await fetch(url, opts);
    const t = await r.text();
    let d; try { d = JSON.parse(t); } catch { d = { error: t.slice(0, 200) }; }
    if (!r.ok) throw new Error(d.error || d.reason || r.status);
    return d;
  }

  function loadImage(url) {
    return new Promise((res, rej) => { const i = new Image(); i.onload = () => res(i); i.onerror = () => rej(new Error('image failed: ' + url)); i.src = url; });
  }

  async function init() {
    try {
      S.scenes = await j('/sim/api/sim/scenes');
    } catch (e) {
      say(`Sim server not reachable yet (${e.message}). dji-rl is adding the A-to-B endpoints.`, 'warn');
      return;
    }
    const sel = $('simScene');
    sel.innerHTML = S.scenes.map(s => `<option>${s.name}</option>`).join('');
    const pref = S.scenes.find(s => s.name === 'house-worldlabs-refined') || S.scenes[0];
    if (pref) { sel.value = pref.name; await pickScene(pref.name); }
  }

  async function pickScene(name) {
    S.scene = name; S.a = S.b = null; S.via = []; setTimeout(renderRoute, 0); S.pois = []; S.measure = []; renderPois(); invalidate(); loadCalib();
    say('Rendering top-down view…');
    try {
      S.meta = await j(`/sim/api/sim/topdown.json?scene=${encodeURIComponent(name)}`);
      S.img = await loadImage(`/sim/api/sim/topdown?scene=${encodeURIComponent(name)}&px=1024`);
      await loadOcc();
      say('Tap the image to set A (start), then B (goal).\nRed = blocked at this altitude. Orange = can\'t take off or land there (something overhead).');
    } catch (e) { say(`Couldn't load scene: ${e.message}`, 'bad'); }
    draw();
  }

  function maskBits(img) {
    const c = document.createElement('canvas');
    c.width = img.naturalWidth; c.height = img.naturalHeight;
    const g = c.getContext('2d');
    g.drawImage(img, 0, 0);
    const d = g.getImageData(0, 0, c.width, c.height).data, out = new Uint8Array(c.width * c.height);
    for (let i = 0; i < out.length; i++) out[i] = d[i * 4] > 127 ? 1 : 0;
    return { w: c.width, h: c.height, bits: out };
  }

  const occUrl = (alt, clear, extra = '') =>
    `/sim/api/sim/occupancy?scene=${encodeURIComponent(S.scene)}&alt=${alt}&clear=${clear}&px=1024${extra}`;

  /** Where takeoff/landing is impossible: blocked anywhere from near the ground up to cruise alt. */
  async function loadColumn(alt) {
    try { return maskBits(await loadImage(occUrl(alt, 0.35, '&mode=column'))); } catch { /* fall back */ }
    const hs = [];
    for (let z = 1; z < alt; z += 1) hs.push(z);
    hs.push(alt);
    const masks = await Promise.all(hs.map(z => loadImage(occUrl(z, 0.35)).then(maskBits).catch(() => null)));
    const ok = masks.filter(Boolean);
    if (!ok.length) return null;
    const out = { w: ok[0].w, h: ok[0].h, bits: new Uint8Array(ok[0].bits.length), approx: true };
    for (const m of ok) for (let i = 0; i < out.bits.length; i++) out.bits[i] |= m.bits[i];
    return out;
  }

  function overlay(cruise, column) {
    const m = cruise || column, c = document.createElement('canvas');
    c.width = m.w; c.height = m.h;
    const g = c.getContext('2d'), im = g.createImageData(m.w, m.h), d = im.data;
    for (let i = 0; i < m.w * m.h; i++) {
      const k = i * 4;
      if (cruise && cruise.bits[i]) { d[k] = 240; d[k + 1] = 60; d[k + 2] = 60; d[k + 3] = 255; }        // red: blocked at cruise alt
      else if (column && column.bits[i]) { d[k] = 245; d[k + 1] = 160; d[k + 2] = 40; d[k + 3] = 200; } // orange: can't take off / land here
    }
    g.putImageData(im, 0, 0);
    return c;
  }

  // Cruise-altitude masks are cached per (scene, alt, clearance). The red overlay is composed from
  // distance rings around A, because the real-flight margin grows with distance from A.
  const maskCache = new Map();
  function cachedMask(alt, clear, extra = '') {
    const k = `${S.scene}|${alt}|${clear}|${extra}`;
    if (!maskCache.has(k)) maskCache.set(k, loadImage(occUrl(alt, clear, extra)).then(maskBits).catch(() => null));
    return maskCache.get(k);
  }

  /** 2D Euclidean distance transform (Felzenszwalb-Huttenlocher), in pixels, to the nearest set bit. */
  function edt2d(bits, w, h) {
    const INF = 1e12, f = new Float64Array(Math.max(w, h)), out = new Float64Array(w * h);
    const v = new Int32Array(Math.max(w, h)), z = new Float64Array(Math.max(w, h) + 1);
    function pass(n, get, set) {
      for (let q = 0; q < n; q++) f[q] = get(q);
      let k = 0; v[0] = 0; z[0] = -INF; z[1] = INF;
      for (let q = 1; q < n; q++) {
        let sx = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
        while (sx <= z[k]) { k--; sx = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]); }
        k++; v[k] = q; z[k] = sx; z[k + 1] = INF;
      }
      k = 0;
      for (let q = 0; q < n; q++) { while (z[k + 1] < q) k++; set(q, (q - v[k]) * (q - v[k]) + f[v[k]]); }
    }
    for (let y = 0; y < h; y++) pass(w, x => bits[y * w + x] ? 0 : INF, (x, d) => { out[y * w + x] = d; });
    for (let x = 0; x < w; x++) pass(h, y => out[y * w + x], (y, d) => { out[y * w + x] = d; });
    for (let i = 0; i < out.length; i++) out[i] = Math.sqrt(out[i]);
    return out;
  }

  // Blocked at cruise altitude = within the normal 3D clearance of something (the server's mask), OR
  // horizontally closer to such an obstacle than the real-flight margin at that spot. The margin is
  // horizontal only (scale/heading error), so the ground below never counts against it.
  async function composeCruise(alt) {
    const raw = await cachedMask(alt, +$('simClear').value);
    if (!raw) return null;
    const m = marginParams(), M = S.meta;
    if (!m.base && !m.rate) return raw;
    const k = `${S.scene}|${alt}|${+$('simClear').value}`;
    if (S.edtKey !== k) { S.edt = edt2d(raw.bits, raw.w, raw.h); S.edtKey = k; }
    const mpp = (M.x1 - M.x0) / raw.w, out = { w: raw.w, h: raw.h, bits: new Uint8Array(raw.w * raw.h) };
    const sy = (M.y1 - M.y0) / raw.h, ax = S.a?.x ?? 0, ay = S.a?.y ?? 0;
    for (let v = 0; v < raw.h; v++) {
      const y = M.y1 - (v + 0.5) * sy;
      for (let u = 0; u < raw.w; u++) {
        const i = v * raw.w + u;
        if (raw.bits[i]) { out.bits[i] = 1; continue; }
        const need = m.base + (S.a ? m.rate * Math.hypot(M.x0 + (u + 0.5) * mpp - ax, y - ay) : 0);
        out.bits[i] = S.edt[i] * mpp < need ? 1 : 0;
      }
    }
    return out;
  }

  let occSeq = 0;
  async function loadOcc() {
    if (!S.scene || !S.meta) return;
    const seq = ++occSeq, alt = +$('simAlt').value;
    try {
      const [cruise, column] = await Promise.all([composeCruise(alt), cachedMask(alt, 0.35, '&mode=column').then(m => m || loadColumn(alt))]);
      if (seq !== occSeq) return; // a newer request superseded this one
      S.cruise = cruise; S.column = column;
      S.occ = cruise || column ? overlay(cruise, column) : null;
    } catch { /* overlay is optional */ }
    draw();
  }

  /** Mask lookup and nearest-free search, in mask pixels. */
  function maskAt(m, x, y) {
    const u = Math.floor((x - S.meta.x0) / (S.meta.x1 - S.meta.x0) * m.w), v = Math.floor((S.meta.y1 - y) / (S.meta.y1 - S.meta.y0) * m.h);
    if (u < 0 || v < 0 || u >= m.w || v >= m.h) return 1;
    return m.bits[v * m.w + u];
  }

  function nearestFree(p, maxM = 10, needColumn = true) {
    const air = $('simAir').checked;
    const blocked = (x, y) => (!air && needColumn && S.column && maskAt(S.column, x, y)) || (S.cruise && maskAt(S.cruise, x, y));
    if (!blocked(p.x, p.y)) return { p, moved: 0 };
    const m = S.column || S.cruise, step = (S.meta.x1 - S.meta.x0) / m.w;
    for (let r = step; r <= maxM; r += step) {
      const n = Math.max(8, Math.ceil(2 * Math.PI * r / step));
      for (let k = 0; k < n; k++) {
        const a = 2 * Math.PI * k / n, q = { x: +(p.x + r * Math.cos(a)).toFixed(2), y: +(p.y + r * Math.sin(a)).toFixed(2) };
        if (!blocked(q.x, q.y)) return { p: q, moved: r };
      }
    }
    return null;
  }

  // ---- coordinates: canvas px <-> image px <-> ENU metres ----
  function fit() {
    const c = $('simCanvas'), W = c.width, H = c.height, iw = S.meta.px_w, ih = S.meta.px_h;
    const s = Math.min(W / iw, H / ih);
    return { s, ox: (W - iw * s) / 2, oy: (H - ih * s) / 2 };
  }
  function enuToCanvas(x, y) {
    const m = S.meta, f = fit();
    const u = (x - m.x0) / (m.x1 - m.x0) * m.px_w, v = (m.y1 - y) / (m.y1 - m.y0) * m.px_h;
    return [f.ox + u * f.s, f.oy + v * f.s];
  }
  function canvasToEnu(cx, cy) {
    const m = S.meta, f = fit();
    const u = (cx - f.ox) / f.s, v = (cy - f.oy) / f.s;
    if (u < 0 || v < 0 || u > m.px_w || v > m.px_h) return null;
    return { x: +(m.x0 + u / m.px_w * (m.x1 - m.x0)).toFixed(2), y: +(m.y1 - v / m.px_h * (m.y1 - m.y0)).toFixed(2) };
  }

  function clearColor(c) {
    if (c == null) return '#4c8dff';
    if (c < 0.5) return '#f05a5a';
    if (c < 1.5) return '#e3a03a';
    return '#3fb950';
  }

  const POI_COLORS = ['#c084fc', '#22d3ee', '#f472b6', '#facc15', '#a3e635', '#fb923c'];
  const poiColor = i => POI_COLORS[i % POI_COLORS.length];

  function renderRoute() {
    const ol = $('simRouteList');
    if (!ol) return;
    const pts = [['A', S.a], ...S.via.map((p, i) => [String(i + 1), p]), ['B', S.b]].filter(([, p]) => p);
    ol.innerHTML = '';
    pts.forEach(([label, p]) => {
      const li = document.createElement('li');
      li.innerHTML = `<div class="pin"><b style="min-width:16px">${label}</b> <input type="number" step="0.5" min="1" max="120" value="${p.alt ?? +$('simAlt').value}" title="height m"> m
        ${label === 'A' ? '' : '<button title="remove">×</button>'}</div>`;
      li.querySelector('input').onchange = ev => { p.alt = +ev.target.value; invalidate(); };
      const del = li.querySelector('button');
      if (del) del.onclick = () => {
        if (p === S.b) S.b = S.via.pop() || null; else S.via.splice(S.via.indexOf(p), 1);
        renderRoute(); invalidate();
      };
      ol.appendChild(li);
    });
    $('simRouteEmpty').hidden = pts.length > 0;
  }

  function renderPois() {
    const ol = $('simPoiList');
    ol.innerHTML = '';
    S.pois.forEach((p, i) => {
      const li = document.createElement('li');
      li.innerHTML = `<div class="pin"><span style="color:${poiColor(i)}">&#9670;</span> <span class="muted">${p.label}</span>
        <input type="number" step="0.5" placeholder="ground" value="${p.z ?? ''}" title="height m (blank = ground)"> m <button title="remove">×</button></div>`;
      li.querySelector('input').onchange = ev => { const v = ev.target.value.trim(); p.z = v === '' ? null : +v; invalidate(); };
      li.querySelector('button').onclick = () => { S.pois.splice(i, 1); S.pois.forEach((q, k) => q.label = `P${k + 1}`); renderPois(); invalidate(); };
      ol.appendChild(li);
    });
    $('simPoiEmpty').hidden = S.pois.length > 0;
  }

  function setTool(t) {
    S.tool = t;
    document.querySelectorAll('#simTool button').forEach(b => b.classList.toggle('on', b.dataset.v === t));
    say(t === 'poi' ? 'Tap the image to add look-at points. The camera will aim at them during the flight.' : t === 'measure' ? 'Tap two landmarks you can measure on the ground (e.g. driveway corners, 20 m+ apart). You\'ll be asked for the real distance.' : 'Tap to set A (takeoff), then keep tapping to add waypoints. The last point is B.');
  }

  // ---- real-world alignment ----
  // Errors between splat and reality: scale (splat metres vs real metres), heading (splat north vs
  // true north) and position. Position drops out because the phone flies the route relative to
  // where it takes off (put the drone on A). Scale and heading errors grow with distance from A,
  // so the safety margin does too.
  function uncertainty() {
    const c = S.calib;
    let u = { scale: c.measured ? 0.02 : 0.06, headingDeg: c.heading_checked ? 2 : 5, base: 1.0, src: c.measured ? 'measured' : 'not calibrated' };
    // A GPS fit from a Flight Lab capture carries 1-sigma errors; use 2 sigma (it doesn't capture constant GPS bias).
    const g = S.scenes?.find(x => x.name === S.scene)?.geo;
    if (g && g.scale_sigma != null && g.heading_sigma_deg != null) {
      const gs = 2 * g.scale_sigma, gh = 2 * g.heading_sigma_deg;
      if (gs + Math.sin(gh * Math.PI / 180) < u.scale + Math.sin(u.headingDeg * Math.PI / 180))
        u = { scale: +gs.toFixed(4), headingDeg: +gh.toFixed(2), base: Math.max(1.0, +(2 * (g.residual_m || 0)).toFixed(1)), src: 'from GPS fit' };
    }
    return u;
  }

  function marginFor(distM) {
    const u = uncertainty();
    return +(u.base + distM * (u.scale + Math.sin(u.headingDeg * Math.PI / 180))).toFixed(1);
  }

  function routeReach() {
    if (!S.a || !S.b) return 0;
    const pts = S.preview?.path?.map(p => ({ x: p[0], y: p[1] })) || [...S.via, S.b];
    return Math.max(...pts.map(p => Math.hypot(p.x - S.a.x, p.y - S.a.y)));
  }

  function marginParams() {
    if (!$('simUseMargin').checked) return { base: 0, rate: 0 };
    const u = uncertainty();
    return { base: u.base, rate: +(u.scale + Math.sin(u.headingDeg * Math.PI / 180)).toFixed(4) };
  }
  function currentMargin() { return $('simUseMargin').checked ? marginFor(routeReach()) : 0; } // at the farthest point

  function updateMarginInfo() {
    const u = uncertainty(), d = routeReach();
    if (S.meta) loadOcc(); // red rings follow A and the margin settings
    $('simMarginInfo').textContent = !S.a || !S.b ? 'Set a route (A to B) to see the real-flight margin.' :
      `Real-flight margin grows with distance from A: ${marginFor(0)} m at A, ${marginFor(d)} m at the farthest point ${Math.round(d)} m out (scale ±${+(u.scale * 100).toFixed(1)}%, ${u.src}, heading ±${u.headingDeg}°).` +
      (u.src !== 'not calibrated' ? '' : ' Use Measure to calibrate scale and shrink this.');
  }

  async function loadCalib() {
    try {
      const c = await j(`/api/calib/${encodeURIComponent(S.scene)}`);
      S.calib = { scale_corr: 1, yaw_trim_deg: 0, measured: false, heading_checked: false, ...c };
    } catch { S.calib = { scale_corr: 1, yaw_trim_deg: 0, measured: false, heading_checked: false }; }
    $('simScale').value = S.calib.scale_corr; $('simYawTrim').value = S.calib.yaw_trim_deg; $('simHeadingOk').checked = !!S.calib.heading_checked;
    const sc = S.scenes.find(x => x.name === S.scene);
    const g = sc?.geo;
    $('simFitInfo').textContent = g ? `Splat-to-GPS fit: origin ${g.lat0.toFixed(6)}, ${g.lon0.toFixed(6)}${g.scale ? ` · scale ×${(+g.scale).toFixed(3)}` : ''}${g.residual_m ? ` · residual ${(+g.residual_m).toFixed(1)} m` : ''}. ${g.note || ''}` :
      'This scene has no GPS alignment, so it can only be used in the sim.';
    updateMarginInfo();
  }

  async function saveCalib() {
    S.calib.scale_corr = +$('simScale').value; S.calib.yaw_trim_deg = +$('simYawTrim').value; S.calib.heading_checked = $('simHeadingOk').checked;
    try { await j(`/api/calib/${encodeURIComponent(S.scene)}`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(S.calib) }); } catch (e) { say(`Couldn't save calibration: ${e.message}`, 'bad'); }
    invalidate(); updateMarginInfo();
  }

  /** Sim ENU (relative to A) -> real metres east/north/up relative to takeoff, with calibration applied. */
  function toReal(x, y, z) {
    const k = S.calib.scale_corr, t = S.calib.yaw_trim_deg * Math.PI / 180;
    const dx = (x - S.a.x) * k, dy = (y - S.a.y) * k;
    return { e_m: +(dx * Math.cos(t) - dy * Math.sin(t)).toFixed(2), n_m: +(dx * Math.sin(t) + dy * Math.cos(t)).toFixed(2), up_m: +(z * k).toFixed(2) };
  }

  function draw() {
    const c = $('simCanvas'), r = c.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    c.width = Math.max(1, r.width * dpr); c.height = Math.max(1, r.height * dpr);
    const g = c.getContext('2d');
    g.fillStyle = '#111'; g.fillRect(0, 0, c.width, c.height);
    if (!S.meta || !S.img) return;
    const f = fit();
    g.drawImage(S.img, f.ox, f.oy, S.meta.px_w * f.s, S.meta.px_h * f.s);
    if (S.occ && $('simShowOcc').checked) {
      g.globalAlpha = 0.4; g.drawImage(S.occ, f.ox, f.oy, S.meta.px_w * f.s, S.meta.px_h * f.s); g.globalAlpha = 1;
    }
    const lw = 2 * dpr;
    const P = S.preview;
    if (P && P.path) {
      g.setLineDash([6 * dpr, 5 * dpr]); g.strokeStyle = 'rgba(255,255,255,.9)'; g.lineWidth = lw;
      g.beginPath(); P.path.forEach(([x, y], i) => { const [px, py] = enuToCanvas(x, y); i ? g.lineTo(px, py) : g.moveTo(px, py); }); g.stroke();
      g.setLineDash([]);
    }
    if (P && P.traj) {
      g.lineWidth = lw * 1.6;
      for (let i = 1; i < P.traj.length; i++) {
        const p0 = P.traj[i - 1], p1 = P.traj[i];
        const [x0, y0] = enuToCanvas(p0.x, p0.y), [x1, y1] = enuToCanvas(p1.x, p1.y);
        g.strokeStyle = $('simColorBy').value === 'poi' && S.pois.length ? (p1.poi >= 0 ? poiColor(p1.poi) : '#888') : clearColor(p1.clear); g.beginPath(); g.moveTo(x0, y0); g.lineTo(x1, y1); g.stroke();
      }
      const k = Math.min(P.traj.length - 1, Math.round(S.t * 10));
      const d = P.traj[k];
      if (d) {
        const [dx, dy] = enuToCanvas(d.x, d.y);
        const tgt = d.poi >= 0 ? (P.pois || S.pois)[d.poi] : null;
        if (tgt) {
          const [tx, ty] = enuToCanvas(tgt.x, tgt.y);
          g.setLineDash([4 * dpr, 4 * dpr]); g.strokeStyle = poiColor(d.poi); g.lineWidth = 1.5 * dpr;
          g.beginPath(); g.moveTo(dx, dy); g.lineTo(tx, ty); g.stroke(); g.setLineDash([]);
        }
        g.fillStyle = '#fff'; g.strokeStyle = '#000'; g.lineWidth = dpr;
        g.save(); g.translate(dx, dy); g.rotate(-d.yaw);
        g.beginPath(); g.moveTo(10 * dpr, 0); g.lineTo(-6 * dpr, 6 * dpr); g.lineTo(-6 * dpr, -6 * dpr); g.closePath(); g.fill(); g.stroke();
        g.restore();
        $('simReadout').textContent = `t ${d.t.toFixed(1)} s · alt ${d.z.toFixed(1)} m · speed ${Math.hypot(d.vx, d.vy).toFixed(1)} m/s · clearance ${d.clear == null ? '--' : d.clear.toFixed(2) + ' m'}${d.gimbal_pitch_deg != null ? ` · gimbal ${d.gimbal_pitch_deg.toFixed(0)}°` : ''}${tgt ? ` · looking at ${tgt.label || 'P' + (d.poi + 1)}` : ''}`;
      }
    }
    const userRoute = [S.a, ...S.via, S.b, $('simLoop').checked && S.b ? S.a : null].filter(Boolean);
    if (userRoute.length > 1 && !P) {
      g.strokeStyle = 'rgba(255,255,255,.75)'; g.lineWidth = 1.5 * dpr; g.setLineDash([3 * dpr, 4 * dpr]);
      g.beginPath(); userRoute.forEach((pt, i) => { const [px, py] = enuToCanvas(pt.x, pt.y); i ? g.lineTo(px, py) : g.moveTo(px, py); }); g.stroke(); g.setLineDash([]);
    }
    S.via.forEach((pt, i) => {
      const [px, py] = enuToCanvas(pt.x, pt.y);
      g.fillStyle = '#4c8dff'; g.strokeStyle = '#000'; g.lineWidth = dpr;
      g.beginPath(); g.arc(px, py, 8 * dpr, 0, 7); g.fill(); g.stroke();
      g.fillStyle = '#fff'; g.font = `bold ${11 * dpr}px system-ui`; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(String(i + 1), px, py + 0.5);
      g.font = `${10 * dpr}px system-ui`; g.lineWidth = 3 * dpr; g.strokeStyle = '#000'; g.strokeText(`${pt.alt ?? ''} m`, px, py + 16 * dpr); g.fillText(`${pt.alt ?? ''} m`, px, py + 16 * dpr);
    });
    for (const [pt, label, col] of [[S.a, 'A', '#3fb950'], [S.b, 'B', '#f05a5a']]) {
      if (!pt) continue;
      const [px, py] = enuToCanvas(pt.x, pt.y);
      g.fillStyle = col; g.strokeStyle = '#000'; g.lineWidth = dpr;
      g.beginPath(); g.arc(px, py, 9 * dpr, 0, 7); g.fill(); g.stroke();
      g.fillStyle = '#000'; g.font = `bold ${12 * dpr}px system-ui`; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(label, px, py + 0.5);
    }
    if (S.measure.length) {
      const pts = S.measure.map(p => enuToCanvas(p.x, p.y));
      g.strokeStyle = '#fff'; g.fillStyle = '#fff'; g.lineWidth = 2 * dpr;
      g.beginPath(); pts.forEach(([px, py], i) => i ? g.lineTo(px, py) : g.moveTo(px, py)); g.stroke();
      pts.forEach(([px, py]) => { g.beginPath(); g.arc(px, py, 4 * dpr, 0, 7); g.fill(); });
      if (S.measure.length === 2) {
        const d = Math.hypot(S.measure[1].x - S.measure[0].x, S.measure[1].y - S.measure[0].y);
        g.font = `bold ${12 * dpr}px system-ui`; g.textAlign = 'center'; g.lineWidth = 3 * dpr; g.strokeStyle = '#000';
        const mx = (pts[0][0] + pts[1][0]) / 2, my = (pts[0][1] + pts[1][1]) / 2 - 10 * dpr;
        g.strokeText(`${d.toFixed(2)} m in splat`, mx, my); g.fillText(`${d.toFixed(2)} m in splat`, mx, my);
      }
    }
    S.pois.forEach((p, i) => {
      const [px, py] = enuToCanvas(p.x, p.y), r = 9 * dpr;
      g.fillStyle = poiColor(i); g.strokeStyle = '#000'; g.lineWidth = dpr;
      g.beginPath(); g.moveTo(px, py - r); g.lineTo(px + r, py); g.lineTo(px, py + r); g.lineTo(px - r, py); g.closePath(); g.fill(); g.stroke();
      g.fillStyle = '#000'; g.font = `bold ${10 * dpr}px system-ui`; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(String(i + 1), px, py + 0.5);
    });
  }

  function invalidate() {
    if (S.scene) setTimeout(updateMarginInfo, 0);
    S.preview = null; S.previewKey = null; S.t = 0; S.playing = false;
    $('simScrub').value = 0; $('simScrub').disabled = true;
    $('simWatch').disabled = true; $('simWatchFpv').disabled = true; $('simReadout').textContent = '';
    updateLaunch();
    draw();
  }

  function updateLaunch() {
    const ok = S.preview && S.preview.ok && S.previewKey === key() && S.preview.reached && !S.preview.collisions && $('simUseMargin').checked && !S.preview.viaIgnored && !S.preview.marginIgnored && !S.preview.altIgnored;
    $('simLaunch').disabled = !ok;
    $('simLaunchWhy').textContent = ok ? 'Preview matches the current settings.' :
      S.preview && S.previewKey !== key() ? 'Settings changed since the preview. Preview again.' :
      S.preview && S.preview.altIgnored ? 'The preview ignored per-point heights (server not updated yet).' :
      S.preview && S.preview.marginIgnored ? "The preview didn't apply the real-flight margin (server not updated yet)." :
      S.preview && S.preview.viaIgnored ? 'The preview ignored your waypoints (server not updated yet).' :
      S.preview && !$('simUseMargin').checked ? 'Turn on the real-flight safety margin and preview again to launch.' :
      S.preview && S.preview.collisions ? 'Preview hit something. Raise altitude or clearance and preview again.' :
      S.preview && !S.preview.reached ? "Preview didn't reach B." :
      'Preview the path first.';
  }

  async function preview() {
    if (!S.a || !S.b) return say('Set A and at least one more point first.', 'warn');
    S.busy = true; $('simPreview').disabled = true;
    say('Planning and simulating…');
    const k = key();
    try {
      const loop = $('simLoop').checked;
      const body = { scene: S.scene, a: S.a, b: loop ? S.a : S.b, via: loop ? [...S.via, S.b] : S.via, // each point has .alt alt_m: +$('simAlt').value, speed_mps: +$('simSpeed').value, clear_m: +$('simClear').value, clear_base_h: marginParams().base, clear_growth_per_m: marginParams().rate, clear_origin: S.a, takeoff_land: !$('simAir').checked,
        pois: S.pois.map(p => ({ x: p.x, y: p.y, z: p.z, label: p.label })), poi_mode: $('simPoiMode').value,
        flight_style: $('simStyle').value, fpv_gimbal_deg: +$('simFpvTilt').value };
      S.previewBody = body;
      const d = await j('/sim/api/sim/fly', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
      S.preview = d; S.previewKey = k; S.t = 0;
      d.viaIgnored = body.via.length > 0 && !Array.isArray(d.via);
      d.altIgnored = [S.a, ...S.via, S.b].some(p => p.alt !== +$('simAlt').value) && !d.per_point_alt;
      d.marginIgnored = (body.clear_growth_per_m > 0 || body.clear_base_h > 0) && d.clear_growth_per_m == null;
      if (!d.ok) { say(`No path: ${d.reason || 'unknown'}`, 'bad'); }
      else {
        const dur = d.duration_s ?? (d.traj.length / 10);
        $('simScrub').max = dur; $('simScrub').disabled = false; $('simWatch').disabled = !d.id; $('simWatchFpv').disabled = !d.id;
        say([
          `Planned ${Math.round(d.length_m)} m, flown in ${dur.toFixed(1)} s.`,
          `Closest approach ${d.min_clear_m?.toFixed(2)} m · collisions ${d.collisions} · ${d.reached ? `reached B (±${(d.final_err_m ?? 0).toFixed(2)} m)` : 'did NOT reach B'}`,
          'White dashed = planned path. Coloured = simulated flight (green > 1.5 m clear, amber 0.5–1.5 m, red < 0.5 m).',
        ].join('\n'), d.collisions || !d.reached ? 'bad' : (d.min_clear_m < 1.5 ? 'warn' : ''));
        if ($('simStyle').value === 'fpv' && d.style !== 'fpv') say($('simStatus').textContent + '\nFPV style requested; the sim server will fly it smoothly once dji-rl ships it (this preview used normal style).', 'warn');
        else if ($('simStyle').value === 'fpv' && S.pois.length) say($('simStatus').textContent + '\nLook-at points are ignored in FPV style (camera faces forward).');
        if ($('simStyle').value === 'normal' && S.pois.length && !d.traj.some(t => t.gimbal_pitch_deg != null)) say($('simStatus').textContent + '\nLook-at points sent; the sim server will aim the camera once dji-rl ships it.', 'warn');
        if (d.marginIgnored) say($('simStatus').textContent + `\nThe sim server doesn't apply the distance-growing margin yet (it planned with only ${body.clear_m} m clearance), so launch stays disabled for now.`, 'bad');
        if (d.altIgnored) say($('simStatus').textContent + `\nThe sim server doesn't support per-point heights yet; this preview flew every leg at ${body.alt_m} m. Launch stays disabled until it does.`, 'bad');
        if (d.viaIgnored) say($('simStatus').textContent + `\nThe sim server doesn't support waypoints yet, so this preview flew straight from A to ${loop ? 'A' : 'B'}, ignoring your ${body.via.length} waypoint(s). Launch is disabled until dji-rl ships multi-waypoint routes.`, 'bad');
        play();
      }
    } catch (e) { say(`Preview failed: ${e.message}`, 'bad'); }
    S.busy = false; $('simPreview').disabled = false;
    updateLaunch(); draw();
  }

  // The overhead playback is the master clock; when the 3D view is open, its replay follows.
  function syncReplay(t, playing) {
    if ($('simFrame').hidden || !S.preview?.id) return;
    const q = playing ? `t=play&from=${t.toFixed(2)}` : `t=${t.toFixed(2)}`;
    fetch(`/sim/replay?id=${encodeURIComponent(S.preview.id)}&${q}`).then(r => {
      if (!r.ok && !S.replayWarned) {
        S.replayWarned = true;
        r.json().catch(() => ({})).then(e => say(`The 3D view couldn't load this replay (sim server ${r.status}${e.error ? ': ' + e.error : ''}). ${r.status === 503 ? 'The GPU is busy with training; try again in a few minutes. ' : ''}The overhead playback still works.`, 'bad'));
      }
      if (r.ok) S.replayWarned = false;
    }).catch(() => {});
  }

  function play() {
    if (!S.preview || !S.preview.traj) return;
    const dur = +$('simScrub').max;
    if (S.t >= dur) S.t = 0;
    S.playing = true;
    syncReplay(S.t, true);
    const t0 = performance.now() - S.t * 1000;
    let lastSync = performance.now();
    const step = now => {
      if (!S.playing) return;
      S.t = Math.min(dur, (now - t0) / 1000);
      $('simScrub').value = S.t; draw();
      if (now - lastSync > 5000) { syncReplay(S.t, true); lastSync = now; } // re-align against drift
      if (S.t < dur) requestAnimationFrame(step);
      else { S.playing = false; syncReplay(dur, false); $('simPlayPause').textContent = 'Play'; }
    };
    $('simPlayPause').textContent = 'Pause';
    requestAnimationFrame(step);
  }

  function pause() { S.playing = false; syncReplay(S.t, false); $('simPlayPause').textContent = 'Play'; }

  function watch3d(cam = 'chase') {
    if (!S.preview?.id) return;
    const zmax = S.scenes.find(x => x.name === S.scene)?.bounds?.zmax;
    const hi = Math.max(...[S.a, ...S.via, S.b].map(p => p.alt ?? +$('simAlt').value));
    if (zmax && hi > zmax + 2) say(`Heads up: this route flies up to ${hi} m but the splat only has detail up to about ${zmax} m. Above that the 3D view is mostly haze from the splat's sky shell; the overhead view and collision checks are still valid.`, 'warn');
    const f = $('simFrame');
    f.hidden = false;
    S.playing = false; S.t = 0;
    f.onload = () => { S.t = 0; play(); }; // start both views together once the 3D page is up
    f.src = `/sim/?replay=${encodeURIComponent(S.preview.id)}&scene=${encodeURIComponent(S.scene)}&cam=${cam}`;
    setTimeout(draw, 0);
  }

  function enuToLatLon(x, y) {
    const geo = S.scenes.find(s => s.name === S.scene)?.geo;
    if (!geo) return null;
    const yaw = (geo.yaw_deg || 0) * Math.PI / 180;
    const e = x * Math.cos(yaw) - y * Math.sin(yaw), n = x * Math.sin(yaw) + y * Math.cos(yaw);
    return { lat: geo.lat0 + n / 110540, lon: geo.lon0 + e / (111320 * Math.cos(geo.lat0 * Math.PI / 180)) };
  }

  async function launch() {
    updateLaunch();
    if ($('simLaunch').disabled) return;
    const P = S.preview, A = enuToLatLon(S.a.x, S.a.y), B = enuToLatLon(S.b.x, S.b.y);
    if (!A || !B) return say("This scene has no GPS alignment, so it can't become a real mission.", 'bad');
    const alt = +$('simAlt').value;
    // Keep every vertex: the planned path is already a sparse polyline, and dropping vertices could cut corners through obstacles.
    const pts = P.path;
    // route_rel heights are relative to the takeoff spot (DJI's altitude reads 0 on the ground there).
    const groundA = (P.traj?.[0] && !$('simAir').checked) ? Math.max(0, P.traj[0].z - 0.35) : 0;
    const route = pts.map((p, i) => { const ll = enuToLatLon(p[0], p[1]); return { lat: +ll.lat.toFixed(7), lon: +ll.lon.toFixed(7), alt_m: +p[2].toFixed(1), kind: i === 0 ? 'home' : i === pts.length - 1 ? 'waypoint' : 'detour' }; });
    const mission = {
      version: 1, name: `sim_${S.scene}_AtoB`, created: new Date().toISOString(),
      frame: 'route_rel is metres east/north/up relative to the takeoff point (place the drone on A). lat/lon are for reference only.',
      loop: $('simLoop').checked,
      home: A,
      limits: { ceiling_m: Math.max(alt, ...P.path.map(p => p[2])) + 5, min_alt_m: alt, clear_h_m: +$('simClear').value, clear_v_m: +$('simClear').value, cruise_speed_mps: +$('simSpeed').value, corridor_m: +$('simClear').value },
      obstacle_handoff: $('simHandoff').value,
      failsafe: { on_link_loss: 'return_to_home', on_low_battery_pct: 25, on_rc_input: 'latch_hand_to_rc' },
      waypoints: [...S.via, S.b].map(p => ({ ...enuToLatLon(p.x, p.y), alt_m: p.alt ?? alt })),
      route,
      route_rel: pts.map(p => toReal(p[0], p[1], p[2] - groundA)),
      pois_rel: S.pois.map(p => ({ ...toReal(p.x, p.y, ((P.pois?.find(q => q.label === p.label)?.z) ?? p.z ?? 1) - groundA), label: p.label })),
      takeoff_land: !$('simAir').checked,
      alignment: { scale_corr: S.calib.scale_corr, yaw_trim_deg: S.calib.yaw_trim_deg, measured: S.calib.measured, heading_checked: S.calib.heading_checked, margin_base_m: marginParams().base, margin_growth_per_m: marginParams().rate, clear_m: +$('simClear').value, max_margin_m: currentMargin() },
      legs: P.legs || route.slice(1).map((p, i) => ({ from: i, to: i + 1, alt: p.alt_m, length_m: null })),
      pois: S.pois.map(p => { const ll = enuToLatLon(p.x, p.y); return { lat: +ll.lat.toFixed(7), lon: +ll.lon.toFixed(7), z_m: (P.pois?.find(q => q.label === p.label)?.z) ?? p.z, label: p.label }; }),
      poi_mode: $('simPoiMode').value,
      flight_style: $('simStyle').value, fpv_gimbal_deg: +$('simFpvTilt').value,
      camera_schedule: P.traj.filter((_, i) => i % 5 === 0).map(t => ({ t: t.t, yaw_deg: +(90 - t.yaw * 180 / Math.PI).toFixed(1), gimbal_pitch_deg: t.gimbal_pitch_deg ?? null, poi: t.poi ?? -1 })),
      sim_request: S.previewBody,
      sim_preview: { scene: S.scene, id: P.id, length_m: P.length_m, duration_s: P.duration_s, min_clear_m: P.min_clear_m, collisions: P.collisions },
      buildings_source: `splat:${S.scene}`, warnings: [],
    };
    const r = await j('/api/missions', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(mission) });
    say(`Mission saved as ${r.name} with its preview attached. Open Flight Lab → Mission on the phone to load and fly it.`);
  }

  function wire() {
    $('simScene').onchange = e => pickScene(e.target.value);
    $('simCanvas').onclick = e => {
      if (!S.meta) return;
      const c = $('simCanvas'), r = c.getBoundingClientRect(), dpr = c.width / r.width;
      const p = canvasToEnu((e.clientX - r.left) * dpr, (e.clientY - r.top) * dpr);
      if (!p) return;
      if (S.tool === 'measure') {
        if (S.measure.length >= 2) S.measure = [];
        S.measure.push(p); draw();
        if (S.measure.length === 2) {
          const d = Math.hypot(S.measure[1].x - S.measure[0].x, S.measure[1].y - S.measure[0].y);
          setTimeout(() => {
            const real = prompt(`These two points are ${d.toFixed(2)} m apart in the splat.\nEnter the real distance you measured on the ground (metres), or cancel:`);
            if (!real || !(+real > 0)) return;
            const k = +real / d;
            if (k < 0.5 || k > 2) return say(`That implies a ${k.toFixed(2)}× scale error, which is implausible. Check the points and the measurement.`, 'bad');
            S.calib.measurements = [...(S.calib.measurements || []), { splat_m: +d.toFixed(3), real_m: +real, a: S.measure[0], b: S.measure[1] }];
            const ms = S.calib.measurements, kk = ms.reduce((s2, m) => s2 + m.real_m, 0) / ms.reduce((s2, m) => s2 + m.splat_m, 0);
            $('simScale').value = kk.toFixed(4); S.calib.measured = true;
            saveCalib();
            say(`Scale correction ×${kk.toFixed(4)} from ${ms.length} measurement(s). Longer distances (20 m+) give a better calibration.`);
          }, 50);
        }
        return;
      }
      if (S.tool === 'poi') {
        const h = $('simPoiH').value.trim();
        S.pois.push({ ...p, z: h === '' ? null : +h, label: `P${S.pois.length + 1}` });
        renderPois(); invalidate();
        say(`Added look-at point P${S.pois.length}. Add more, or switch back to A / B.`);
        return;
      }
      const which = !S.a ? 'A' : !S.b ? 'B' : `B (point ${S.via.length + 2})`;
      const lands = which === 'A' || !$('simLoop').checked;  // B is only a landing spot when not looping home
      const what = $('simAir').checked || !lands ? `clear at ${$('simAlt').value} m` : `clear from the ground to ${$('simAlt').value} m`;
      let note = '';
      if (S.cruise || S.column) {
        const f = nearestFree(p, 10, lands);
        if (!f) { say(`No spot ${what} within 10 m of there. Pick open ground (not red or orange).`, 'warn'); return; }
        if (f.moved) note = `${which} moved ${f.moved.toFixed(1)} m to the nearest spot ${what}. `;
        Object.assign(p, f.p);
      }
      p.alt = +$('simAlt').value;
      if (!S.a) S.a = p;
      else if (!S.b) S.b = p;
      else { S.via.push(S.b); S.b = p; }
      renderRoute();
      invalidate();
      say(note + (S.b ? `Route: A → ${S.via.length ? S.via.map((_, i) => i + 1).join(' → ') + ' → ' : ''}B${$('simLoop').checked ? ' → A' : ''}. Keep tapping to add points, or Plan & preview.` : 'Tap to set the next point.'), note ? 'warn' : '');
    };
    ['simAlt', 'simClear'].forEach(id => $(id).onchange = () => { invalidate(); loadOcc(); });
    $('simAir').onchange = invalidate;
    $('simLoop').onchange = invalidate;
    $('simUndo').onclick = () => { if (S.b) { S.b = S.via.pop() || null; } else S.a = null; renderRoute(); invalidate(); };
    $('simClearRoute').onclick = () => { S.a = S.b = null; S.via = []; renderRoute(); invalidate(); say('Route cleared. Tap to set A.'); };
    $('simSpeed').onchange = invalidate;
    $('simShowOcc').onchange = draw;
    $('simColorBy').onchange = draw;
    $('simPoiMode').onchange = invalidate;
    $('simStyle').onchange = () => { $('simFpvRow').hidden = $('simStyle').value !== 'fpv'; invalidate(); };
    $('simFpvTilt').onchange = invalidate;
    $('simScale').onchange = () => { S.calib.measured = S.calib.measured || +$('simScale').value !== 1; saveCalib(); };
    $('simYawTrim').onchange = saveCalib;
    $('simHeadingOk').onchange = saveCalib;
    $('simUseMargin').onchange = () => { invalidate(); updateMarginInfo(); };
    document.querySelectorAll('#simTool button').forEach(b => b.onclick = () => setTool(b.dataset.v));
    $('simPoiClear').onclick = () => { S.pois = []; renderPois(); invalidate(); };
    $('simPreview').onclick = preview;
    $('simWatch').onclick = () => watch3d('chase');
    $('simWatchFpv').onclick = () => watch3d('fpv');
    $('simLaunch').onclick = launch;
    $('simScrub').oninput = e => { S.playing = false; S.t = +e.target.value; draw(); syncReplay(S.t, false); $('simPlayPause').textContent = 'Play'; };
    $('simReplay').onclick = () => { S.t = 0; play(); };
    $('simPlayPause').onclick = () => S.playing ? pause() : play();
    window.addEventListener('resize', draw);
  }

  window.SimTab = { init, wire, draw, started: false, _state: S, _preview: preview, _invalidate: invalidate, _nearestFree: nearestFree };
})();
