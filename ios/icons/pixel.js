// Pixel-art versions of the StarSplat glider: the vector sprite rasterised onto a 64x64 grid, snapped to its palette,
// outlined in dark ink, over a dithered pixel sky. Scaled up with nearest-neighbour, so 1 cell = 16 px at 1024.
(function (root) {
  let N = 64;
  const hex = h => [1, 3, 5].map(i => parseInt(h.slice(i, i + 2), 16));
  const BAYER = [[0, 8, 2, 10], [12, 4, 14, 6], [3, 11, 1, 9], [15, 7, 13, 5]];

  function sky(ctx, bands, seed, extras) {
    const img = ctx.createImageData(N, N), d = img.data, cols = bands.map(hex);
    for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
      const t = (y / (N - 1)) * (cols.length - 1), k = Math.min(cols.length - 2, Math.floor(t)), f = t - k;
      const c = f * 16 > BAYER[y & 3][x & 3] + 0.5 ? cols[k + 1] : cols[k];   // ordered dither between bands
      const i = (y * N + x) * 4; d[i] = c[0]; d[i + 1] = c[1]; d[i + 2] = c[2]; d[i + 3] = 255;
    }
    ctx.putImageData(img, 0, 0);
    let s = seed >>> 0; const r = () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296);
    if (extras.stars) for (let i = 0; i < extras.stars; i++) {
      const x = Math.floor(r() * N), y = Math.floor(r() * N * 0.85);
      ctx.fillStyle = r() < 0.3 ? '#fff6c8' : '#ffffff'; ctx.fillRect(x, y, 1, 1);
      if (r() < 0.18) { ctx.globalAlpha = 0.6; ctx.fillRect(x - 1, y, 1, 1); ctx.fillRect(x + 1, y, 1, 1); ctx.fillRect(x, y - 1, 1, 1); ctx.fillRect(x, y + 1, 1, 1); ctx.globalAlpha = 1; }
    }
    if (extras.clouds) for (const [cx, cy, w] of extras.clouds) {
      ctx.fillStyle = extras.cloudShade || '#cfe6fb'; ctx.fillRect(cx, cy + 2, w, 2);
      ctx.fillStyle = '#ffffff'; ctx.fillRect(cx + 1, cy + 1, w - 2, 2); ctx.fillRect(cx + 3, cy, Math.max(2, w - 7), 1);
    }
    if (extras.sun) { const [sx, sy, rr] = extras.sun; ctx.fillStyle = extras.sunColor;
      for (let y = -rr; y <= rr; y++) for (let x = -rr; x <= rr; x++) if (x * x + y * y <= rr * rr + 1) ctx.fillRect(sx + x, sy + y, 1, 1); }
  }

  function loadSprite(svg) {
    return new Promise((ok, bad) => { const im = new Image(); im.onload = () => ok(im); im.onerror = bad;
      im.src = 'data:image/svg+xml;charset=utf-8,' + encodeURIComponent(svg); });
  }

  // Draw the sprite at 64x64, snap each pixel to the nearest palette colour, alpha to 0/1, then outline.
  async function spriteLayer(svg, pal, ink, glow) {
    const c = document.createElement('canvas'); c.width = c.height = N;
    const x = c.getContext('2d'); x.imageSmoothingEnabled = true;
    x.drawImage(await loadSprite(svg), 0, 0, N, N);
    const img = x.getImageData(0, 0, N, N), d = img.data, P = Object.values(pal).map(hex);
    const solid = new Uint8Array(N * N);
    for (let i = 0; i < N * N; i++) {
      if (d[i * 4 + 3] < 110) { d[i * 4 + 3] = 0; continue; }
      let best = P[0], bd = 1e9;
      for (const p of P) { const e = (p[0] - d[i * 4]) ** 2 + (p[1] - d[i * 4 + 1]) ** 2 + (p[2] - d[i * 4 + 2]) ** 2; if (e < bd) { bd = e; best = p; } }
      d[i * 4] = best[0]; d[i * 4 + 1] = best[1]; d[i * 4 + 2] = best[2]; d[i * 4 + 3] = 255; solid[i] = 1;
    }
    const ic = hex(ink), gc = glow ? hex(glow) : null;
    const near = (i, r) => { const x0 = i % N, y0 = (i / N) | 0;
      for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
        if (Math.abs(dx) + Math.abs(dy) > r) continue; const xx = x0 + dx, yy = y0 + dy;
        if (xx >= 0 && yy >= 0 && xx < N && yy < N && solid[yy * N + xx]) return true; } return false; };
    for (let i = 0; i < N * N; i++) {
      if (solid[i]) continue;
      if (near(i, 1)) { d[i * 4] = ic[0]; d[i * 4 + 1] = ic[1]; d[i * 4 + 2] = ic[2]; d[i * 4 + 3] = 255; }
      else if (gc && near(i, N >= 64 ? 2 : 1 + 1)) { d[i * 4] = gc[0]; d[i * 4 + 1] = gc[1]; d[i * 4 + 2] = gc[2]; d[i * 4 + 3] = 150; }
    }
    x.putImageData(img, 0, 0);
    return c;
  }

  const I = root.StarIcons;
  const PIXEL = [
    { id: 'P1', name: 'Pixel day', note: 'Side glider, dithered blue sky, pixel clouds.',
      sky: { bands: ['#3c7fe0', '#5d9ff0', '#8cc4fb', '#c4e5ff'], clouds: [[4, 46, 14], [44, 52, 16], [46, 8, 12]] },
      model: 'side', pal: 'classic', t: 'translate(520 540) rotate(-24) scale(1.16)', ink: '#22122a' },
    { id: 'P2', name: 'Pixel night', note: 'Side glider on a starry night, pink rim glow.',
      sky: { bands: ['#0a0d2a', '#141a4a', '#25245e', '#3a2d72'], stars: 46 },
      model: 'side', pal: 'classic', t: 'translate(520 540) rotate(-24) scale(1.16)', ink: '#0a0716', glow: '#ff6b9a' },
    { id: 'P3', name: 'Pixel head-on', note: 'Wings spread, symmetric sprite, day sky.',
      sky: { bands: ['#2f74d6', '#579bef', '#8fc6fb', '#cfeaff'], clouds: [[2, 50, 14], [48, 44, 14]] },
      model: 'front', pal: 'classic', t: 'translate(512 492) scale(1.22)', ink: '#22122a' },
    { id: 'P4', name: 'Pixel sunset', note: 'Warm dusk bands with a low pixel sun.',
      sky: { bands: ['#2b2f78', '#7a3f8c', '#d4607a', '#ff9a5a', '#ffd27a'], sun: [50, 50, 6], sunColor: '#fff0b0' },
      model: 'side', pal: 'warm', t: 'translate(520 540) rotate(-24) scale(1.16)', ink: '#2a1030' },
    { id: 'P5', name: 'Pixel head-on night', note: 'Spread wings over stars, the most "StarSplat".',
      sky: { bands: ['#090c28', '#151b4c', '#2a2766', '#43307a'], stars: 52 },
      model: 'front', pal: 'classic', t: 'translate(512 492) scale(1.22)', ink: '#0a0716', glow: '#ff6b9a' },
    { id: 'P6', name: 'Night, 7 triangles', note: 'Facing left. Seven flat triangles, nothing else.',
      sky: { bands: ['#0a0d2a', '#141a4a', '#25245e', '#3a2d72'], stars: 46 },
      model: 'mini', pal: 'classic', t: 'translate(500 545) scale(-1 1) rotate(-22) scale(1.22)', ink: '#0a0716', glow: '#ff6b9a' },
    { id: 'P7', name: 'Night, big pixels', note: 'Same seven triangles on a 32-cell grid: chunkier.',
      grid: 32, sky: { bands: ['#0a0d2a', '#141a4a', '#25245e', '#3a2d72'], stars: 16 },
      model: 'mini', pal: 'classic', t: 'translate(500 545) scale(-1 1) rotate(-22) scale(1.22)', ink: '#0a0716', glow: '#ff6b9a' },
    { id: 'P8', name: 'Night, 9 triangles', note: 'Facing left, red wing edge and a tail fin.',
      sky: { bands: ['#0a0d2a', '#141a4a', '#25245e', '#3a2d72'], stars: 46 },
      model: 'mini2', pal: 'classic', t: 'translate(500 545) scale(-1 1) rotate(-22) scale(1.22)', ink: '#0a0716', glow: '#ff6b9a' },
    { id: 'P9', name: 'Night dive', note: 'Seven triangles, facing left, swooping down.',
      sky: { bands: ['#0a0d2a', '#141a4a', '#25245e', '#3a2d72'], stars: 46 },
      model: 'mini', pal: 'classic', t: 'translate(500 500) scale(-1 1) rotate(18) scale(1.22)', ink: '#0a0716', glow: '#ff6b9a' },
  ];

  async function render(v, size = 1024) {
    N = v.grid || 64;
    const base = document.createElement('canvas'); base.width = base.height = N;
    const bx = base.getContext('2d');
    sky(bx, v.sky.bands, v.id.charCodeAt(1) * 977, v.sky);
    bx.drawImage(await spriteLayer(I.sprite(v.model, v.pal, v.t), I.PAL[v.pal], v.ink, v.glow), 0, 0);
    const out = document.createElement('canvas'); out.width = out.height = size;
    const ox = out.getContext('2d'); ox.imageSmoothingEnabled = false; ox.drawImage(base, 0, 0, size, size);
    return out;
  }

  root.PixelIcons = { PIXEL, render };
})(this);
