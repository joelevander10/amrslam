// Runs the real static/measure.js (pure geometry of the route editor's Measure tool) and
// prints one JSON line of results. Driven by test_measure_js.py.
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ctx = { console, Math, Number, Array, Infinity };
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'amr_web', 'static', 'measure.js'), 'utf8') +
  '\nthis.api = { measureDelta, measureNearestOnSegment, measureNearestOnArc, measureArcOf, measureSnapSet, measureSnap, measureArcPoints };', ctx);
const M = ctx.api;
const out = {};
try {
  out.delta = M.measureDelta({ x: 1, y: 2 }, { x: -2, y: 6 });                       // B - A
  out.seg_mid = M.measureNearestOnSegment([1, 1], [0, 0], [2, 0]);
  out.seg_clamped = M.measureNearestOnSegment([5, 1], [0, 0], [2, 0]);
  // a left (ccw) quarter circle of R 1 about (0, 1) from (0, 0) heading +x: ends at (1, 1)
  const arcStep = { id: 's2', type: 'arc', start: [0, 0, 0], end: [1, 1, Math.PI / 2], centre: [0, 1], radius_m: 1, signed_angle_deg: 90 };
  const arc = M.measureArcOf(arcStep);
  out.arc_on = M.measureNearestOnArc([2, 0], arc);                                     // inside the span (-26.6 deg seen from the centre): onto the circle
  out.arc_outside = M.measureNearestOnArc([-2, 1.2], arc);                             // outside the span: the nearer end (0, 0)
  const cw = M.measureArcOf({ type: 'arc', start: [0, 0, 0], end: [1, -1, -Math.PI / 2], centre: [0, -1], radius_m: 1, signed_angle_deg: -90 });
  out.arc_cw_on = M.measureNearestOnArc([2, 0], cw);                                   // a right arc: +45 deg about (0, -1)
  out.arc_cw_outside = M.measureNearestOnArc([-2, 2], cw);
  // a route: straight to (2, 0), wait there, left arc R 1, straight on to (3, 4)
  const route = { label: 'draft', start: [0, 0, 0], steps: [
    { id: 's1', type: 'straight', start: [0, 0, 0], end: [2, 0, 0], centre: null },
    { id: 's2', type: 'wait', start: [2, 0, 0], end: [2, 0, 0], name: 'hitch' },
    { id: 's3', type: 'arc', start: [2, 0, 0], end: [3, 1, Math.PI / 2], centre: [2, 1], radius_m: 1, signed_angle_deg: 90 },
    { id: 's4', type: 'straight', start: [3, 1, Math.PI / 2], end: [3, 4, Math.PI / 2], centre: null },
    { id: 's5', type: 'rotate', start: [3, 4, Math.PI / 2], end: [3, 4, Math.PI], centre: null },
  ] };
  const set = M.measureSnapSet([route], [{ x: 0, y: -0.5, kind: 'vertex', label: 'survey start' }]);
  out.kinds = set.targets.map(t => t.kind + ':' + t.label);
  out.paths = set.paths.map(p => p.label);
  const tol = 0.2;
  out.snap_wait = M.measureSnap([2.05, 0.05], set, tol);        // the wait beats s1 end / s3 start at the same spot
  out.snap_centre = M.measureSnap([2.1, 0.9], set, tol);        // arc centre
  out.snap_vertex = M.measureSnap([3.1, 4.05], set, tol);       // the rotate point
  out.snap_on_line = M.measureSnap([1.0, 0.15], set, tol);      // nothing near: onto s1
  out.snap_on_arc = M.measureSnap([2.75, 0.3], set, tol);       // onto the s3 arc
  out.snap_free = M.measureSnap([1.0, 2.5], set, tol);          // out of reach of everything
  out.snap_zero_tol = M.measureSnap([2.0, 0.0], set, 0);        // snapping off
  out.snap_extra = M.measureSnap([0.05, -0.45], set, tol);      // the survey start mark
  out.arc_points = M.measureArcPoints(arc, 2);
} catch (e) {
  out.error = String(e && e.stack || e);
}
console.log(JSON.stringify(out));
