// Measure tool geometry (route editor, 2026-11-03): pure, no DOM, no MapView. editor.js wires it.
// Everything is map metres. A measurement is B - A: distance, dX, dY in the map frame.
//
// Snap targets come from route geometries shaped like the compiler's steps:
//   { label, start: [x, y, yaw], steps: [{ id, type, start: [x, y, yaw], end: [x, y, yaw],
//                                         centre: [x, y] | null, radius_m, signed_angle_deg, name }] }
// Within the tolerance a point snaps to the best-ranked target (then the nearest of that rank);
// only when no target is in reach does it snap onto a route line (straight or arc).

const MEASURE_RANK = { wait: 0, centre: 1, vertex: 2, path: 3 };
// Closer by less than this is a tie, and ties go to the route listed first (the draft): a saved
// copy of the loaded route sits on the same points to within float noise.
const MEASURE_TIE_M = 1e-4;

function measureDelta(a, b) {
  const dx = b.x - a.x, dy = b.y - a.y;
  return { dx, dy, dist: Math.hypot(dx, dy) };
}

// Nearest point of segment a-b to p (all [x, y]); the ends included.
function measureNearestOnSegment(p, a, b) {
  const vx = b[0] - a[0], vy = b[1] - a[1], L2 = vx * vx + vy * vy;
  const t = L2 > 0 ? Math.max(0, Math.min(1, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2)) : 0;
  return [a[0] + t * vx, a[1] + t * vy];
}

// Nearest point of an arc to p. The arc: centre c, radius r, starting at angle a0 (seen from
// c) and sweeping `sweep` radians (+ccw). Outside its span the nearer end is the answer.
function measureNearestOnArc(p, arc) {
  const { c, r, a0, sweep } = arc, TAU = 2 * Math.PI, sign = sweep >= 0 ? 1 : -1;
  const phi = Math.atan2(p[1] - c[1], p[0] - c[0]);
  const along = (((sign * (phi - a0)) % TAU) + TAU) % TAU;  // angle from the start, in the sweep direction
  const at = th => [c[0] + r * Math.cos(th), c[1] + r * Math.sin(th)];
  if (along <= Math.abs(sweep)) return at(phi);
  const s = at(a0), e = at(a0 + sweep);
  return Math.hypot(p[0] - s[0], p[1] - s[1]) <= Math.hypot(p[0] - e[0], p[1] - e[1]) ? s : e;
}

// An arc step as { c, r, a0, sweep }, or null when the step carries no centre.
function measureArcOf(step) {
  if (!step.centre || !(step.radius_m > 0)) return null;
  const c = step.centre;
  return { c, r: step.radius_m, a0: Math.atan2(step.start[1] - c[1], step.start[0] - c[0]), sweep: (step.signed_angle_deg || 0) * Math.PI / 180 };
}

// Snap targets and route lines of a list of route geometries, plus extra targets
// ({ x, y, kind, label }, e.g. the survey start mark). Earlier routes win exact ties.
function measureSnapSet(routes, extra) {
  const targets = [], paths = [];
  const add = (x, y, kind, label) => { if (Number.isFinite(x) && Number.isFinite(y)) targets.push({ x, y, kind, label }); };
  (routes || []).forEach(rt => {
    const pre = rt.label ? rt.label + ' · ' : '';
    if (rt.start) add(rt.start[0], rt.start[1], 'vertex', pre + 'start');
    (rt.steps || []).forEach(s => {
      const id = pre + s.id;
      if (s.type === 'wait') { add(s.end[0], s.end[1], 'wait', `${id} wait${s.name ? ' ' + s.name : ''}`); return; }
      if (s.type === 'rotate') { add(s.end[0], s.end[1], 'vertex', `${id} turn`); return; }
      add(s.start[0], s.start[1], 'vertex', `${id} start`);
      add(s.end[0], s.end[1], 'vertex', `${id} end`);
      if (s.type === 'arc') {
        const arc = measureArcOf(s);
        if (arc) { add(arc.c[0], arc.c[1], 'centre', `${id} arc centre`); paths.push({ arc, label: `on ${id} arc` }); }
      } else if (s.type === 'straight' || s.type === 'reverse') {
        paths.push({ seg: [s.start.slice(0, 2), s.end.slice(0, 2)], label: `on ${id}` });
      } else if (s.type === 'home' && Array.isArray(s.samples)) {
        for (let k = 1; k < s.samples.length; k++) paths.push({ seg: [s.samples[k - 1].slice(0, 2), s.samples[k].slice(0, 2)], label: `on ${id} home leg` });
      }
    });
  });
  (extra || []).forEach(t => add(t.x, t.y, t.kind, t.label));
  return { targets, paths };
}

// p ([x, y]) snapped within tol metres: { x, y, kind, label }; kind 'free' when nothing is in reach.
function measureSnap(p, set, tol) {
  let best = null, bestD = Infinity;
  if (tol > 0) {
    for (const t of set.targets) {
      const d = Math.hypot(t.x - p[0], t.y - p[1]);
      if (d > tol) continue;
      if (!best || MEASURE_RANK[t.kind] < MEASURE_RANK[best.kind] || (MEASURE_RANK[t.kind] === MEASURE_RANK[best.kind] && d < bestD - MEASURE_TIE_M)) { best = t; bestD = d; }
    }
    if (best) return { x: best.x, y: best.y, kind: best.kind, label: best.label };
    for (const path of set.paths) {
      const q = path.seg ? measureNearestOnSegment(p, path.seg[0], path.seg[1]) : measureNearestOnArc(p, path.arc);
      const d = Math.hypot(q[0] - p[0], q[1] - p[1]);
      if (d <= tol && d < bestD - MEASURE_TIE_M) { best = { x: q[0], y: q[1], kind: 'path', label: path.label }; bestD = d; }
    }
    if (best) return best;
  }
  return { x: p[0], y: p[1], kind: 'free', label: 'free point' };
}

// Points along an arc for drawing, n + 1 of them.
function measureArcPoints(arc, n) {
  return Array.from({ length: n + 1 }, (_, k) => { const th = arc.a0 + arc.sweep * k / n; return [arc.c[0] + arc.r * Math.cos(th), arc.c[1] + arc.r * Math.sin(th)]; });
}
