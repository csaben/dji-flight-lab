// StarSplat icon generator: original low-poly sky creature over a faceted "glass sky".
// Pure string output (SVG 1024x1024), usable in the browser (window.StarIcons) and in node (module.exports).
(function (root) {
  const S = 1024;

  function rng(seed) { let s = seed >>> 0; return () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296); }
  const hex = h => [1, 3, 5].map(i => parseInt(h.slice(i, i + 2), 16));
  const toHex = c => '#' + c.map(v => Math.max(0, Math.min(255, Math.round(v))).toString(16).padStart(2, '0')).join('');
  function grad(stops, t) {
    t = Math.max(0, Math.min(1, t));
    const k = Math.min(stops.length - 2, Math.floor(t * (stops.length - 1)));
    const u = t * (stops.length - 1) - k, a = hex(stops[k]), b = hex(stops[k + 1]);
    return a.map((v, i) => v + (b[i] - v) * u);
  }
  const poly = (pts, fill, extra = '') =>
    `<polygon points="${pts.map(p => p[0].toFixed(1) + ',' + p[1].toFixed(1)).join(' ')}" fill="${fill}" stroke="${fill}" stroke-width="1.2" stroke-linejoin="round"${extra}/>`;

  // Faceted background: jittered grid, two triangles per cell, shaded along a diagonal gradient.
  function mesh(stops, seed, { n = 6, jitter = 0.32, shade = 0.07, angle = 0.25 } = {}) {
    const r = rng(seed), step = S / n, P = [];
    for (let j = 0; j <= n; j++) {
      P.push([]);
      for (let i = 0; i <= n; i++) {
        const edgeX = i === 0 || i === n, edgeY = j === 0 || j === n;
        P[j].push([i * step + (edgeX ? 0 : (r() - 0.5) * 2 * jitter * step), j * step + (edgeY ? 0 : (r() - 0.5) * 2 * jitter * step)]);
      }
    }
    let out = '';
    for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) {
      const a = P[j][i], b = P[j][i + 1], c = P[j + 1][i + 1], d = P[j + 1][i];
      for (const tri of (i + j) % 2 ? [[a, b, c], [a, c, d]] : [[a, b, d], [b, c, d]]) {
        const cx = (tri[0][0] + tri[1][0] + tri[2][0]) / 3, cy = (tri[0][1] + tri[1][1] + tri[2][1]) / 3;
        const t = (cy / S) * (1 - angle) + (cx / S) * angle;
        const col = grad(stops, t).map(v => v * (1 + (r() - 0.5) * 2 * shade));
        out += poly(tri, toHex(col));
      }
    }
    return out;
  }

  // Glass: soft top sheen with a curved lower edge, plus a faint inner rim.
  function glass(strength = 0.38) {
    return `<defs><linearGradient id="sheen" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0" stop-color="#fff" stop-opacity="${strength}"/><stop offset="1" stop-color="#fff" stop-opacity="0"/></linearGradient></defs>
      <path d="M0 0H1024V330C760 430 300 470 0 400Z" fill="url(#sheen)"/>
      <rect x="10" y="10" width="1004" height="1004" rx="220" fill="none" stroke="#fff" stroke-opacity="0.18" stroke-width="6"/>`;
  }

  const PAL = {
    classic: { r1: '#f2495d', r2: '#d62a40', r3: '#a91b30', r4: '#7a1022', w1: '#ffffff', w2: '#e6ecf4', w3: '#c3cede', eye: '#17204a' },
    warm:    { r1: '#f4505a', r2: '#d7303e', r3: '#ab1f2e', r4: '#7c1424', w1: '#fff8f0', w2: '#ffe4cf', w3: '#f0c1a3', eye: '#2a1840' },
    mono:    { r1: '#ffffff', r2: '#eef1f6', r3: '#d3d9e4', r4: '#b4bdcd', w1: '#ffffff', w2: '#f6f8fb', w3: '#dfe5ee', eye: '#c41e36' },
  };

  // Side / three-quarter view, nose to the right. Low-poly swept-wing glider with a crest fin.
  const SIDE = (() => {
    const p = {
      H: [300, 0], HT: [215, -48], HB: [225, 30], E: [248, -12], N1: [130, -50], N2: [140, 45],
      B1: [20, -45], B2: [10, 62], B3: [-120, -28], B4: [-110, 45], T: [-230, 6], TT: [-370, 46], TF: [-330, -36],
      C1: [118, -128], FW1: [95, -40], FW2: [-60, -36], FWT: [-150, -262], FWM: [-96, -126],
      NW1: [82, 20], NW2: [-72, 36], NWT: [-186, 232], NWM: [-128, 108], NWL: [24, 116],
    };
    return [
      // far wing (behind)
      ['FW1', 'FWT', 'FWM', 'r3'], ['FW1', 'FWM', 'FW2', 'r4'],
      // tail
      ['B3', 'TT', 'T', 'r2'], ['T', 'TT', 'B4', 'w3'], ['T', 'TF', 'TT', 'r4'],
      // body
      ['N1', 'B1', 'N2', 'r2'], ['B1', 'B2', 'N2', 'w2'], ['B1', 'B3', 'B2', 'r3'], ['B3', 'B4', 'B2', 'w3'],
      // head + crest
      ['H', 'HT', 'E', 'r1'], ['HT', 'N1', 'E', 'r2'], ['E', 'N1', 'N2', 'r3'], ['E', 'N2', 'HB', 'w1'], ['H', 'E', 'HB', 'w2'],
      ['HT', 'C1', 'N1', 'r4'],
      // near wing (front): red leading edge, white underside
      ['NW1', 'NWL', 'NWT', 'r1'], ['NW1', 'NWT', 'NWM', 'w1'], ['NW1', 'NWM', 'NW2', 'w2'],
    ].map(([a, b, c, k]) => ({ pts: [p[a], p[b], p[c]], k }))
      .concat([{ pts: [[236, -16], [258, -8], [238, -2]], k: 'eye' }]);
  })();

  // Head-on, wings spread. Built as the right half, mirrored with lighter shades on the left (light from the left).
  const FRONT = (() => {
    const half = [
      [[60, -50], [360, 40], [190, 74], 'r1', 'r2'], [[60, -50], [190, 74], [40, 62], 'r2', 'r3'],   // wing
      [[48, -150], [126, -254], [18, -186], 'r3', 'r4'],                                             // crest fin
      [[0, -212], [48, -150], [0, -120], 'r1', 'r2'], [[0, -120], [48, -150], [62, -40], 'r2', 'r3'], // head
      [[0, -120], [62, -40], [0, -20], 'w1', 'w2'], [[0, -20], [62, -40], [38, 92], 'w2', 'w3'],     // chest
      [[0, -20], [38, 92], [0, 142], 'w1', 'w2'],
      [[0, 142], [38, 92], [22, 182], 'r2', 'r3'], [[0, 142], [22, 182], [0, 300], 'r3', 'r4'],     // tail
      [[14, -150], [38, -162], [30, -140], 'eye', 'eye'],
    ];
    const out = [];
    for (const [a, b, c, kr, kl] of half) {
      out.push({ pts: [[-a[0], a[1]], [-b[0], b[1]], [-c[0], c[1]]], k: kl === 'eye' ? 'eye' : kl.replace(/\d/, d => String(Math.max(1, +d - 1))) });
      out.push({ pts: [a, b, c], k: kr === 'eye' ? 'eye' : kl });
    }
    return out;
  })();

  // Minimal: seven triangles, nose to the right (mirror it in the transform to face left).
  const MINI = [
    [[90, -30], [-200, -250], [-60, -20], 'r3'],                 // far wing
    [[-60, -20], [-340, 30], [-40, 50], 'r3'],                   // tail
    [[190, -40], [-60, -20], [170, 40], 'r2'],                   // back
    [[-60, -20], [-40, 50], [170, 40], 'w2'],                    // belly
    [[300, 0], [190, -40], [170, 40], 'r1'],                     // head
    [[190, -40], [110, -112], [130, -34], 'r4'],                 // crest
    [[80, 20], [-220, 212], [-40, 40], 'w1'],                    // near wing
    [[232, -12], [252, -4], [232, 4], 'eye'],
  ].map(([a, b, c, k]) => ({ pts: [a, b, c], k }));
  // Same, with a red leading edge on the near wing and a split tail: nine triangles.
  const MINI2 = MINI.slice(0, 6).concat([
    [[80, 20], [-220, 212], [-40, 40], 'w1'], [[80, 20], [-40, 120], [-220, 212], 'r1'],
    [[-340, 30], [-300, -40], [-200, 10], 'r4'], [[232, -12], [252, -4], [232, 4], 'eye'],
  ].map(([a, b, c, k]) => ({ pts: [a, b, c], k })));

  function creature(model, pal, transform, glow) {
    const body = model.map(f => poly(f.pts, pal[f.k])).join('');
    const g = glow ? `<g transform="${transform}" filter="url(#glow)" opacity="${glow.o}">${model.map(f => poly(f.pts, glow.c)).join('')}</g>` : '';
    return g + `<g transform="${transform}">${body}</g>`;
  }

  function stars(seed, count, big) {
    const r = rng(seed); let s = '';
    for (let i = 0; i < count; i++) {
      const x = r() * S, y = r() * S * 0.9, k = 3 + r() * 7, o = 0.45 + r() * 0.55;
      s += poly([[x, y - k * 1.8], [x + k * 0.55, y], [x, y + k * 1.8], [x - k * 0.55, y]], '#fff', ` opacity="${o.toFixed(2)}"`);
      s += poly([[x - k * 1.8, y], [x, y - k * 0.55], [x + k * 1.8, y], [x, y + k * 0.55]], '#fff', ` opacity="${o.toFixed(2)}"`);
    }
    for (const [x, y, k] of big) {
      s += poly([[x, y - k * 2.4], [x + k * 0.5, y - k * 0.5], [x + k * 2.4, y], [x + k * 0.5, y + k * 0.5], [x, y + k * 2.4], [x - k * 0.5, y + k * 0.5], [x - k * 2.4, y], [x - k * 0.5, y - k * 0.5]], '#fff8e0');
    }
    return s;
  }

  const GLOW = `<defs><filter id="glow" x="-30%" y="-30%" width="160%" height="160%"><feGaussianBlur stdDeviation="26"/></filter></defs>`;
  const SIDE_T = 'translate(530 590) rotate(-26) scale(1.14)';

  const VARIANTS = [
    { id: 'A', name: 'Daybreak', note: 'Glider climbing through a clear morning sky.',
      svg: () => mesh(['#3d8ff2', '#7fc0ff', '#d6efff'], 11) + creature(SIDE, PAL.classic, SIDE_T) + glass() },
    { id: 'B', name: 'Starfield', note: 'Night sky with faceted stars and a soft glow.',
      svg: () => GLOW + mesh(['#070c26', '#16205a', '#3b2b72'], 23, { shade: 0.1 }) + stars(5, 26, [[180, 200, 16], [820, 150, 11], [860, 820, 9]])
        + creature(SIDE, PAL.classic, SIDE_T, { c: '#ff6b8a', o: 0.55 }) + glass(0.22) },
    { id: 'C', name: 'Sunset', note: 'Dusk gradient, warm white underside.',
      svg: () => mesh(['#2a3584', '#a8508f', '#ff9a62', '#ffd08a'], 37, { angle: 0.1 }) + creature(SIDE, PAL.warm, SIDE_T) + glass(0.3) },
    { id: 'D', name: 'Head-on', note: 'Wings spread, symmetric, reads well at small sizes.',
      svg: () => mesh(['#2f7fe0', '#6fb4ff', '#c4e6ff'], 51, { angle: 0 }) + creature(FRONT, PAL.classic, 'translate(512 500) scale(1.18)') + glass() },
    { id: 'E', name: 'Glass orb', note: 'Sky bubble on red, the glider inside.',
      svg: () => mesh(['#e5485c', '#c31f37', '#8f1326'], 64, { shade: 0.06 })
        + `<defs><radialGradient id="orb" cx="0.42" cy="0.32" r="0.75"><stop offset="0" stop-color="#e9f7ff"/><stop offset="0.55" stop-color="#8ccaff"/><stop offset="1" stop-color="#3a86e6"/></radialGradient>
           <clipPath id="oc"><circle cx="512" cy="512" r="360"/></clipPath>
           <linearGradient id="osh" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#fff" stop-opacity="0.75"/><stop offset="1" stop-color="#fff" stop-opacity="0"/></linearGradient></defs>
           <circle cx="512" cy="530" r="372" fill="#000" opacity="0.18"/><circle cx="512" cy="512" r="360" fill="url(#orb)"/>
           <g clip-path="url(#oc)">${creature(SIDE, PAL.classic, 'translate(522 548) rotate(-26) scale(0.86)')}</g>
           <ellipse cx="452" cy="300" rx="230" ry="110" fill="url(#osh)"/>
           <circle cx="512" cy="512" r="360" fill="none" stroke="#fff" stroke-opacity="0.55" stroke-width="8"/>` },
    { id: 'F', name: 'Mono', note: 'White glider on faceted red. Minimal.',
      svg: () => mesh(['#ec4a5e', '#cf2a41', '#a3182e'], 78) + creature(SIDE, PAL.mono, SIDE_T) + glass(0.25) },
  ];

  const svgOf = v => `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1024 1024" width="1024" height="1024">${v.svg()}</svg>`;
  // Creature only, transparent background (for the pixel-art versions).
  const sprite = (model, pal, transform) =>
    `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1024 1024" width="1024" height="1024">${creature({ front: FRONT, mini: MINI, mini2: MINI2 }[model] || SIDE, PAL[pal], transform)}</svg>`;
  const api = { VARIANTS, svgOf, sprite, PAL, SIDE_T };
  if (typeof module !== 'undefined') module.exports = api; else root.StarIcons = api;
})(this);
