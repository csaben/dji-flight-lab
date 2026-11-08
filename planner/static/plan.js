// Route planning with building avoidance. No map dependencies, so it can be tested with node.
//
// Model: flat local frame around home (metres east/north), altitudes in metres above the
// takeoff point (AGL, assuming roughly flat ground). Each leg is flown vertical-then-horizontal:
// the drone changes to the leg's cruise altitude in place, then flies straight at that altitude.
//
// Avoidance per leg:
//   1. Buildings within horizontal clearance of the leg whose roof + vertical clearance is above
//      the cruise altitude "block" the leg.
//   2. If the highest blocker can be cleared under the ceiling, climb: cruise = roof + clearance.
//   3. Otherwise detour horizontally around buildings that can't be overflown (shortest path on a
//      visibility graph over their inflated convex hulls), then apply 1-2 to each sub-leg.
(function (root) {
  const R_LAT = 110540, R_LON = 111320;

  function makeFrame(lat0, lon0) {
    const k = Math.cos(lat0 * Math.PI / 180);
    return {
      toXY: (lat, lon) => [(lon - lon0) * k * R_LON, (lat - lat0) * R_LAT],
      toLL: (x, y) => [lat0 + y / R_LAT, lon0 + x / (k * R_LON)],
    };
  }

  // ---- building heights from OSM tags ----
  function parseLen(v) {
    if (v == null) return null;
    const s = String(v).trim().toLowerCase();
    const ft = s.match(/^([\d.]+)\s*(ft|')/);
    if (ft) return parseFloat(ft[1]) * 0.3048;
    const m = s.match(/^([\d.]+)/);
    return m ? parseFloat(m[1]) : null;
  }

  function buildingHeight(tags, fallback) {
    const h = parseLen(tags.height) ?? parseLen(tags['building:height']);
    if (h) return { h, estimated: false };
    const lv = parseFloat(tags['building:levels']);
    if (lv > 0) return { h: lv * 3.2 + (parseFloat(tags['roof:levels']) || 0) * 2.5 + 1.5, estimated: true };
    return { h: fallback, estimated: true };
  }

  /** Overpass JSON -> [{id, ring:[[lat,lon]...], height, estimated, name}] */
  function parseBuildings(osm, fallbackHeight) {
    const out = [];
    for (const el of osm.elements || []) {
      let ring = null;
      if (el.type === 'way' && el.geometry) ring = el.geometry.map(p => [p.lat, p.lon]);
      else if (el.type === 'relation' && el.members) {
        const outer = el.members.find(m => m.role === 'outer' && m.geometry);
        if (outer) ring = outer.geometry.map(p => [p.lat, p.lon]);
      }
      if (!ring || ring.length < 3) continue;
      const { h, estimated } = buildingHeight(el.tags || {}, fallbackHeight);
      out.push({ id: `${el.type}/${el.id}`, ring, height: h, estimated, name: (el.tags || {}).name || '' });
    }
    return out;
  }

  // ---- 2D geometry ----
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1]];
  const cross = (a, b) => a[0] * b[1] - a[1] * b[0];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1];
  const len = a => Math.hypot(a[0], a[1]);
  const dist = (a, b) => len(sub(a, b));

  function segSegIntersect(p1, p2, q1, q2, eps = 1e-9) {
    const r = sub(p2, p1), s = sub(q2, q1), d = cross(r, s);
    if (Math.abs(d) < eps) return false;
    const t = cross(sub(q1, p1), s) / d, u = cross(sub(q1, p1), r) / d;
    return t > eps && t < 1 - eps && u > eps && u < 1 - eps;
  }

  function pointSegDist(p, a, b) {
    const ab = sub(b, a), l2 = dot(ab, ab);
    const t = l2 ? Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2)) : 0;
    return dist(p, [a[0] + t * ab[0], a[1] + t * ab[1]]);
  }

  function segSegDist(a, b, c, d) {
    if (segSegIntersect(a, b, c, d, 0)) return 0;
    return Math.min(pointSegDist(a, c, d), pointSegDist(b, c, d), pointSegDist(c, a, b), pointSegDist(d, a, b));
  }

  function pointInPoly(p, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const [xi, yi] = poly[i], [xj, yj] = poly[j];
      if ((yi > p[1]) !== (yj > p[1]) && p[0] < (xj - xi) * (p[1] - yi) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
  }

  function segPolyDist(a, b, poly) {
    if (pointInPoly(a, poly) || pointInPoly(b, poly)) return 0;
    let m = Infinity;
    for (let i = 0; i < poly.length; i++) m = Math.min(m, segSegDist(a, b, poly[i], poly[(i + 1) % poly.length]));
    return m;
  }

  function convexHull(pts) {
    const p = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    if (p.length < 3) return p;
    const lower = [], upper = [];
    for (const q of p) { while (lower.length >= 2 && cross(sub(lower.at(-1), lower.at(-2)), sub(q, lower.at(-2))) <= 0) lower.pop(); lower.push(q); }
    for (const q of p.reverse()) { while (upper.length >= 2 && cross(sub(upper.at(-1), upper.at(-2)), sub(q, upper.at(-2))) <= 0) upper.pop(); upper.push(q); }
    return lower.slice(0, -1).concat(upper.slice(0, -1)); // CCW
  }

  /** Offset a CCW convex hull outward by d (mitre joins, capped). */
  function inflate(hull, d) {
    const n = hull.length, out = [];
    const normal = (a, b) => { const e = sub(b, a), l = len(e) || 1; return [e[1] / l, -e[0] / l]; };
    for (let i = 0; i < n; i++) {
      const prev = hull[(i - 1 + n) % n], cur = hull[i], next = hull[(i + 1) % n];
      const n1 = normal(prev, cur), n2 = normal(cur, next);
      let bis = [n1[0] + n2[0], n1[1] + n2[1]];
      const bl = len(bis) || 1;
      bis = [bis[0] / bl, bis[1] / bl];
      const c = Math.max(dot(bis, n1), 0.35);
      out.push([cur[0] + bis[0] * d / c, cur[1] + bis[1] * d / c]);
    }
    return out;
  }

  /** True if segment a-b passes through the interior of a convex polygon. */
  function segCrossesInterior(a, b, poly) {
    for (let i = 0; i < poly.length; i++) if (segSegIntersect(a, b, poly[i], poly[(i + 1) % poly.length])) return true;
    const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    // Edges along the hull boundary are fine; strictly-inside midpoints are not.
    if (!pointInPoly(mid, poly)) return false;
    for (let i = 0; i < poly.length; i++) if (pointSegDist(mid, poly[i], poly[(i + 1) % poly.length]) < 1e-6) return false;
    return true;
  }

  function shortestDetour(a, b, obstacles) {
    const nodes = [a, b];
    for (const o of obstacles) for (const v of o) if (!obstacles.some(p => p !== o && pointInPoly(v, p))) nodes.push(v);
    const visible = (p, q) => !obstacles.some(o => segCrossesInterior(p, q, o));
    const N = nodes.length, D = new Array(N).fill(Infinity), prev = new Array(N).fill(-1), done = new Array(N).fill(false);
    D[0] = 0;
    for (let it = 0; it < N; it++) {
      let u = -1;
      for (let i = 0; i < N; i++) if (!done[i] && (u < 0 || D[i] < D[u])) u = i;
      if (u < 0 || D[u] === Infinity) break;
      if (u === 1) break;
      done[u] = true;
      for (let v = 0; v < N; v++) {
        if (done[v] || v === u) continue;
        const w = D[u] + dist(nodes[u], nodes[v]);
        if (w < D[v] && visible(nodes[u], nodes[v])) { D[v] = w; prev[v] = u; }
      }
    }
    if (D[1] === Infinity) return null;
    const path = [];
    for (let v = 1; v !== -1; v = prev[v]) path.push(nodes[v]);
    return path.reverse();
  }

  // ---- planner ----
  /**
   * pins: [{lat, lon, alt}] user waypoints (home is added as start and end).
   * opts: {ceiling, clearH, clearV, minAlt}
   * returns {route:[{lat,lon,alt,kind}], legs:[{from,to,alt,reason,length}], warnings:[], stats}
   */
  function plan(home, pins, buildings, opts) {
    const { ceiling, clearH, clearV, minAlt } = opts;
    const F = makeFrame(home.lat, home.lon);
    const B = buildings.map(b => {
      const poly = b.ring.map(([la, lo]) => F.toXY(la, lo));
      return { ...b, poly, hull: convexHull(poly) };
    });
    const hard = B.filter(b => b.height + clearV > ceiling);
    const hardHulls = hard.map(b => inflate(b.hull, clearH * 1.15));
    const warnings = [];
    if (hard.length) warnings.push(`${hard.length} building(s) can't be overflown under the ${ceiling} m ceiling; routing around them.`);

    const stops = [{ ...home, alt: Math.max(minAlt, home.alt ?? minAlt), kind: 'home' },
      ...pins.map(p => ({ ...p, alt: Math.max(minAlt, Math.min(p.alt, ceiling)), kind: 'waypoint' })),
      { ...home, alt: Math.max(minAlt, home.alt ?? minAlt), kind: 'home' }];

    const route = [{ lat: stops[0].lat, lon: stops[0].lon, alt: stops[0].alt, kind: 'home' }];
    const legs = [];

    function legAlt(a, b, base) {
      const blockers = B.filter(bd => bd.height + clearV > base && segPolyDist(a, b, bd.poly) < clearH);
      if (!blockers.length) return { alt: base, reason: 'clear', blockers: [] };
      const need = Math.max(...blockers.map(bd => bd.height)) + clearV;
      if (need <= ceiling) return { alt: Math.ceil(need), reason: `climb over ${blockers.length} building(s)`, blockers };
      return null;
    }

    for (let i = 0; i < stops.length - 1; i++) {
      const s0 = stops[i], s1 = stops[i + 1];
      const a = F.toXY(s0.lat, s0.lon), b = F.toXY(s1.lat, s1.lon);
      const base = Math.max(s0.alt, s1.alt);
      let pts = [a, b];
      if (!legAlt(a, b, base)) {
        // A stop inside a tall building's clearance zone can't be detoured to; flag it and
        // route around everything else.
        const obstacles = hardHulls.filter(h => !pointInPoly(a, h) && !pointInPoly(b, h));
        for (const [k, s] of [[i, s0], [i + 1, s1]]) {
          if (hardHulls.some(h => pointInPoly(F.toXY(s.lat, s.lon), h))) {
            const label = s.kind === 'home' ? 'Home' : `Waypoint ${k}`;
            const msg = `${label} is within ${clearH} m of a building taller than the ceiling. Move it.`;
            if (!warnings.includes(msg)) warnings.push(msg);
          }
        }
        const d = shortestDetour(a, b, obstacles);
        if (d) pts = d;
        else warnings.push(`Leg ${i + 1}: no path around tall buildings found; check the route.`);
      }
      for (let j = 0; j < pts.length - 1; j++) {
        const p = pts[j], q = pts[j + 1];
        let la = legAlt(p, q, base);
        if (!la) {
          la = { alt: ceiling, reason: 'BLOCKED at ceiling', blockers: [] };
          warnings.push(`Leg ${i + 1}: a segment still passes a building taller than the ceiling.`);
        }
        const [lat, lon] = F.toLL(q[0], q[1]);
        const last = j === pts.length - 2;
        route.push(last ? { lat: s1.lat, lon: s1.lon, alt: s1.alt, kind: s1.kind } : { lat, lon, alt: la.alt, kind: 'detour' });
        legs.push({ from: route.length - 2, to: route.length - 1, alt: la.alt, reason: pts.length > 2 ? `detour; ${la.reason}` : la.reason,
          length: Math.round(dist(p, q)), blockers: la.blockers.map(x => x.id) });
      }
    }
    const lengthM = legs.reduce((s, l) => s + l.length, 0);
    return { route, legs, warnings, stats: { lengthM, buildings: B.length, hard: hard.length, estimatedHeights: B.filter(b => b.estimated).length } };
  }

  root.Planner = { plan, parseBuildings, makeFrame, convexHull, inflate, segPolyDist, shortestDetour };
  if (typeof module !== 'undefined') module.exports = root.Planner;
})(typeof window !== 'undefined' ? window : globalThis);
