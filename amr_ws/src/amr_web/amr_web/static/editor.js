// Route editor: constrained straight/turn tools over a MapView. Persists map metres,
// never pixels. Validation and saving happen on the robot (POST); nothing moves.
const view = new MapView(document.getElementById('ed-canvas'));
let mapId = null, mapRev = null, footprint = null, mapsIndex = [];
const DEFAULT_LIMITS = () => ({});
// The one trackless speed (2026-10-02): amr_navigation.route.VEHICLE_V_MAX, rendered into the page.
// Speed keys in older route files are dropped here and by the server; nothing writes them back.
const SPEED_V = +(document.getElementById('ed-speed').dataset.v) || 0.60;
const LEGACY_SPEED_KEYS = ['linear_mps', 'long_linear_mps', 'long_min_length_m', 'arc_linear_mps'];
const routeLimits = l => Object.fromEntries(Object.entries(l || {}).filter(([k]) => !LEGACY_SPEED_KEYS.includes(k)));
let route = { route_id: '', start: null, steps: [], repeat_count: 1, limits: DEFAULT_LIMITS() };
let history = [], future = [], lastResult = null, savedRef = null;
const $ = id => document.getElementById(id);
// Context tokens (review Q15): every awaited server response is applied only if the map
// selection (mapToken) and the draft (draftToken) are still the ones it was made for. A
// map change bumps mapToken; any edit, undo/redo, load or clear bumps draftToken.
let mapToken = 0, draftToken = 0;
const ctx = () => ({ m: mapToken, d: draftToken });
const stillCurrent = c => c.m === mapToken && c.d === draftToken;
const stillSameMap = c => c.m === mapToken;

// An edit invalidates the last validation result: a stale "VALID" must not describe a changed draft.
function snapshot() { history.push(JSON.stringify(route)); if (history.length > 100) history.shift(); future = []; draftToken += 1; lastResult = null; savedRef = null; }
function undo() { if (!history.length) return; future.push(JSON.stringify(route)); route = JSON.parse(history.pop()); draftToken += 1; lastResult = null; savedRef = null; refresh(); }
function redo() { if (!future.length) return; history.push(JSON.stringify(route)); route = JSON.parse(future.pop()); draftToken += 1; lastResult = null; savedRef = null; refresh(); }

// Distances are authored and stored in metres; the operator reads and types them in mm (0 dp).
const mm = m => num(Math.round(m * 1000), 0);
const MIN_STRAIGHT_M = 0.05;
const REVERSE_MAX_M = 2.0;  // amr_navigation.route.REVERSE_MAX_M: the rear is outside the scanner's field
const REVERSE_V = Math.min(SPEED_V / 2, 0.30);  // compiler: REVERSE_SPEED_RATIO, capped at REVERSE_V_MAX_MPS
const ARC_MIN_R = 1.0, ARC_MIN_DEG = 45, ARC_MAX_DEG = 180;  // amr_navigation.route ARC_* bounds
// mirrors amr_navigation compiler.arc_speed: 0.30 m/s (ARC_V_MPS), lowered so v/R <= 0.9 x VEHICLE_ARC_W_MAX
const ARC_V_MPS = 0.30, ARC_YAW_RATE_RATIO = 0.9, VEHICLE_ARC_W_MAX = 0.45;
// Same construction as compiler.arc_centre / arc_pose: the circle to the left for +1 (ccw).
function arcCentre(x, y, yaw, r, sign) { return [x - sign * r * Math.sin(yaw), y + sign * r * Math.cos(yaw)]; }
function arcPose(c, r, yaw0, sign, phi) { const yaw = yaw0 + sign * phi; return { x: c[0] + sign * r * Math.sin(yaw), y: c[1] - sign * r * Math.cos(yaw), yaw }; }
function arcSamples(a, s, n) {  // poses along step s (an arc) from pose a, n+1 of them
  const sign = s.direction === 'ccw' ? 1 : -1, c = arcCentre(a.x, a.y, a.yaw, s.radius_m, sign), th = s.angle_deg * Math.PI / 180;
  return Array.from({ length: n + 1 }, (_, k) => arcPose(c, s.radius_m, a.yaw, sign, th * k / n));
}
const arcSpeed = r => Math.min(ARC_V_MPS, SPEED_V, ARC_YAW_RATE_RATIO * VEHICLE_ARC_W_MAX * r);
// The home leg (2026-11-03): mirrors amr_navigation.scurve.home_leg for the preview - a quintic
// lateral profile in HOME's frame from the route's end pose to the start pose (HOME). The
// compiler's result is the one validated and driven. Bounds mirror amr_navigation.route HOME_*.
const HOME_MAX_RUNUP_M = 5.0, HOME_MAX_HEADING_DEG = 20, HOME_MAX_SLOPE_DEG = 30, HOME_MIN_R = 1.0, HOME_STRAIGHT_TOL_M = 0.001;
const wrapAngle = a => Math.atan2(Math.sin(a), Math.cos(a));
function homeProfile(t, off, run, s0) {  // y, dy/dx, d2y/dx2 at t (0 = start, 1 = HOME)
  const t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t, b = run * s0;
  const y = off * (1 - 10 * t3 + 15 * t4 - 6 * t5) + b * (t - 6 * t3 + 8 * t4 - 3 * t5);
  const dy = off * (-30 * t2 + 60 * t3 - 30 * t4) + b * (1 - 18 * t2 + 32 * t3 - 15 * t4);
  const ddy = off * (-60 * t + 180 * t2 - 120 * t3) + b * (-36 * t + 96 * t2 - 60 * t3);
  return [y, dy / run, ddy / (run * run)];
}
function homeShape(off, run, s0) {  // [steepest |dy/dx|, tightest radius]
  let steep = 0, kmax = 0;
  for (let k = 0; k <= 400; k++) { const [, dy, ddy] = homeProfile(k / 400, off, run, s0); steep = Math.max(steep, Math.abs(dy)); kmax = Math.max(kmax, Math.abs(ddy) / Math.pow(1 + dy * dy, 1.5)); }
  return [steep, kmax > 0 ? 1 / kmax : Infinity];
}
const homeFits = (off, run, s0) => { const [st, r] = homeShape(off, run, s0); return st <= Math.tan(HOME_MAX_SLOPE_DEG * Math.PI / 180) && r >= HOME_MIN_R; };
function homeSuggest(off, run, s0) {
  for (let d = run; d <= HOME_MAX_RUNUP_M + 1e-9; d += 0.05) if (homeFits(off, d, s0)) return `end the route at least ${mm(d)} mm behind HOME`;
  return `it does not fit within ${mm(HOME_MAX_RUNUP_M)} mm: end the route closer to HOME's line`;
}
// { ok, samples [{x, y, yaw}], length, runUp, offset (+ left of HOME's line), err (rad), minR, straight } or { ok: false, why }
function homeLeg(a, h) {
  const c = Math.cos(h.yaw), s = Math.sin(h.yaw), dx = a.x - h.x, dy = a.y - h.y;
  const along = dx * c + dy * s, off = -dx * s + dy * c, err = wrapAngle(a.yaw - h.yaw), run = -along;
  if (run < MIN_STRAIGHT_M) return { ok: false, why: `the route ends ${mm(along)} mm along HOME's heading, not behind HOME: the home leg only drives forward` };
  if (run > HOME_MAX_RUNUP_M + 1e-9) return { ok: false, why: `the route ends ${mm(run)} mm behind HOME; the home leg drives at most ${mm(HOME_MAX_RUNUP_M)} mm: end it closer to HOME` };
  if (Math.abs(err) > HOME_MAX_HEADING_DEG * Math.PI / 180 + 1e-12) return { ok: false, why: `the vehicle faces ${num(err * 180 / Math.PI, 1)}° off HOME's heading; at most ${HOME_MAX_HEADING_DEG}°: add a turn first` };
  const s0 = Math.tan(err);
  if (Math.abs(off) <= HOME_STRAIGHT_TOL_M && Math.abs(run * s0) <= HOME_STRAIGHT_TOL_M) {
    const n = Math.max(1, Math.ceil(run / 0.05));
    const samples = Array.from({ length: n + 1 }, (_, k) => ({ x: h.x - c * run * (1 - k / n), y: h.y - s * run * (1 - k / n), yaw: h.yaw }));
    return { ok: true, samples, length: run, runUp: run, offset: off, err, minR: Infinity, straight: true };
  }
  const [steep, minR] = homeShape(off, run, s0);
  const what = `${mm(Math.abs(off))} mm sideways and ${num(err * 180 / Math.PI, 1)}° over a ${mm(run)} mm run-up`;
  if (steep > Math.tan(HOME_MAX_SLOPE_DEG * Math.PI / 180)) return { ok: false, why: `${what}: the S-curve would point ${num(Math.atan(steep) * 180 / Math.PI, 0)}° off HOME's line (at most ${HOME_MAX_SLOPE_DEG}°); ${homeSuggest(off, run, s0)}` };
  if (minR < HOME_MIN_R) return { ok: false, why: `${what} needs a ${num(minR, 2)} m radius, tighter than ${num(HOME_MIN_R, 2)} m; ${homeSuggest(off, run, s0)}` };
  const n = Math.max(2, Math.ceil(run / (0.05 * Math.cos(Math.atan(steep))))), samples = [];
  for (let k = 0; k <= n; k++) {
    const t = k / n, [y, d] = homeProfile(t, off, run, s0), xl = -run * (1 - t);
    samples.push({ x: h.x + c * xl - s * y, y: h.y + s * xl + c * y, yaw: h.yaw + Math.atan(d) });
  }
  samples[0] = { x: a.x, y: a.y, yaw: a.yaw }; samples[n] = { x: h.x, y: h.y, yaw: h.yaw };
  let length = 0; for (let k = 1; k <= n; k++) length += Math.hypot(samples[k].x - samples[k - 1].x, samples[k].y - samples[k - 1].y);
  return { ok: true, samples, length, runUp: run, offset: off, err, minR, straight: false };
}
const homeText = L => L.straight ? `straight ${mm(L.length)} mm to HOME`
  : `S-curve ${mm(L.length)} mm to HOME: ${mm(Math.abs(L.offset))} mm ${L.offset > 0 ? 'left' : 'right'}, ${num(L.err * 180 / Math.PI, 1)}°, tightest R ${num(L.minR, 2)} m`;
// The leg of step i (a home step) from the pose the route reaches before it.
const homeLegAt = i => homeLeg(poseAfter(i), poseAfter(0));

function setTool(t) {
  const was = view.tool;
  view.tool = t; view._linePreview = null;
  ['tool-start', 'tool-line', 'tool-measure'].forEach(id => $(id).classList.toggle('on', id === 'tool-' + t));
  $('ed-hint').textContent = t === 'start' ? 'Click the start position, drag towards the heading, release.'
    : t === 'line' ? 'Click a point, or type a length in mm and press Add: the straight goes along the current heading.'
    : t === 'measure' ? 'Measure: click A, then B. Snaps to waits, arc centres, step ends and route lines (this draft and the saved routes, drawn faint). Shift or untick Snap = exactly where you click. Esc clears. Drag = pan.'
    : 'Wheel = zoom, drag = pan.';
  if (was === 'measure' && t !== 'measure') clearMeasure();
  if (t === 'measure' && was !== 'measure') { showMeasure(); loadSavedGeometry(); }
  view.draw();
}

// Heading after the last step, from the authored primitives (same rules as the compiler).
function poseAfter(n) {
  if (!route.start) return null;
  let x = route.start.x_m, y = route.start.y_m, yaw = route.start.yaw_deg * Math.PI / 180;
  for (let i = 0; i < n; i++) {
    const s = route.steps[i];
    if (s.type === 'straight') { x = s.to.x_m; y = s.to.y_m; }
    else if (s.type === 'reverse') { x -= Math.cos(yaw) * s.distance_m; y -= Math.sin(yaw) * s.distance_m; }
    else if (s.type === 'arc') { const e = arcSamples({ x, y, yaw }, s, 1)[1]; x = e.x; y = e.y; yaw = e.yaw; }
    else if (s.type === 'rotate') yaw += (s.direction === 'ccw' ? 1 : -1) * s.angle_deg * Math.PI / 180;
    else if (s.type === 'home') { x = route.start.x_m; y = route.start.y_m; yaw = route.start.yaw_deg * Math.PI / 180; }
    // a wait keeps the pose
  }
  return { x, y, yaw };
}
function stepLength(i) {
  const s = route.steps[i]; if (s.type === 'wait') return 0; if (s.type === 'arc') return s.radius_m * s.angle_deg * Math.PI / 180;
  if (s.type === 'home') { const L = homeLegAt(i); return L.ok ? L.length : 0; }
  const a = poseAfter(i), b = poseAfter(i + 1); return Math.hypot(b.x - a.x, b.y - a.y);
}
function nextId() { let i = route.steps.length + 1; while (route.steps.some(s => s.id === 's' + i)) i++; return 's' + i; }

view.onDrag = (a, b, done) => {
  if (view.tool !== 'start') return;
  const yaw = Math.atan2(b[1] - a[1], b[0] - a[0]);
  if (done) { snapshot(); route.start = { x_m: a[0], y_m: a[1], yaw_deg: yaw * 180 / Math.PI }; refresh(); }
  else view._preview = { x: a[0], y: a[1], yaw };
};
// Distance along the current heading from the route's end pose to a map point (null without a start).
function alongTo(w) {
  const p = poseAfter(route.steps.length); if (!p) return null;
  return (w[0] - p.x) * Math.cos(p.yaw) + (w[1] - p.y) * Math.sin(p.yaw);
}
// One straight of `along` metres from the route's end pose, along its heading: the click tool
// and the typed length share it, so both obey the same minimum and land on the same endpoint.
function addStraight(along) {
  const p = poseAfter(route.steps.length); if (!p) { log('set the start pose first', 'bad'); return false; }
  if (!(along >= MIN_STRAIGHT_M)) { log(`a straight only goes forward along the heading, at least ${mm(MIN_STRAIGHT_M)} mm; use a turn to change direction`, 'bad'); return false; }
  const id = nextId(), c = Math.cos(p.yaw), s = Math.sin(p.yaw);
  const v = straightV($('ed-straight-v').value);
  if (v === false) return false;
  const step = { id, type: 'straight', to: { x_m: p.x + c * along, y_m: p.y + s * along } };
  if (v !== null) step.v_mps = v;
  snapshot(); route.steps.push(step); refresh();
  log(`${id}: straight ${mm(along)} mm${v !== null ? ` at ${num(v, 2)} m/s` : ''}`);
  return true;
}
// A straight's own speed (amr_navigation.route.Step.v_mps): '' = the route speed (null),
// STEP_V_MIN..the route speed, else false (logged). A slow zone: the vehicle slows before
// the step and speeds up again after it, without stopping (the steps are chained).
const STEP_V_MIN = 0.10;
function straightV(text) {
  const t = String(text ?? '').trim();
  if (t === '') return null;
  const v = Number(t);
  if (!Number.isFinite(v) || v < STEP_V_MIN || v > SPEED_V + 1e-9) {
    log(`a straight's speed is ${num(STEP_V_MIN, 2)}–${num(SPEED_V, 2)} m/s (empty = the route speed)`, 'bad');
    return false;
  }
  return Math.round(v * 100) / 100;
}
// Speed waypoint: split straight i at `d` metres from its start into two straights on the same
// line; both keep its speed. The first keeps the id (missions and checkpoints name it).
function splitStraight(i, d) {
  const s = route.steps[i];
  if (!s || s.type !== 'straight') { log('pick a straight to split', 'bad'); return; }
  const a = poseAfter(i), L = stepLength(i);
  if (!(d >= MIN_STRAIGHT_M && d <= L - MIN_STRAIGHT_M)) { log(`the waypoint must be ${mm(MIN_STRAIGHT_M)}–${mm(L - MIN_STRAIGHT_M)} mm from the start of ${s.id}`, 'bad'); return; }
  const id = nextId(), tail = { id, type: 'straight', to: { ...s.to } };
  if (s.v_mps != null) tail.v_mps = s.v_mps;
  snapshot();
  route.steps[i] = { ...s, to: { x_m: a.x + Math.cos(a.yaw) * d, y_m: a.y + Math.sin(a.yaw) * d } };
  route.steps.splice(i + 1, 0, tail);
  refresh();
  log(`${s.id} split at ${mm(d)} mm: ${s.id} ${mm(d)} mm + ${id} ${mm(L - d)} mm. Set a speed on either part in the step list.`);
}
$('btn-split').onclick = () => {
  const i = route.steps.findIndex(s => s.id === $('ed-split-step').value), v = $('ed-split-mm').valueAsNumber;
  if (i < 0) { log('pick the straight to split', 'bad'); return; }
  if (!Number.isFinite(v)) { log('type where the waypoint goes, in mm from the start of the straight', 'bad'); return; }
  splitStraight(i, v / 1000);
};
// The speed box of a straight in the step list.
$('ed-steps').addEventListener('change', e => {
  const el = e.target; if (!el.classList || !el.classList.contains('sv')) return;
  const s = route.steps[+el.dataset.i]; if (!s || s.type !== 'straight') return;
  const v = straightV(el.value);
  if (v === false) { el.value = s.v_mps ?? ''; return; }
  snapshot();
  if (v === null) delete s.v_mps; else s.v_mps = v;
  refresh();
  log(`${s.id}: ${v === null ? `route speed ${num(SPEED_V, 2)}` : `${num(v, 2)}`} m/s`);
});
view.onClick = w => {
  if (view.tool === 'measure') { measureClick(w); return; }
  if (view.tool === 'start') { snapshot(); route.start = { x_m: w[0], y_m: w[1], yaw_deg: route.start ? route.start.yaw_deg : 0 }; refresh(); return; }
  if (view.tool === 'line') { const along = alongTo(w); if (along === null) { log('set the start pose first', 'bad'); return; } addStraight(along); }
};
// Preview of the straight a click (or the typed length) would add: metres along the heading, or null.
view.onHover = w => {
  view._linePreview = (view.tool === 'line' && w) ? alongTo(w) : null;
  if (view.tool === 'measure') { measure.hover = w ? snapAt(w) : null; if (measure.a && !measure.b) showMeasure(); }
};
$('ed-len').oninput = e => { const v = e.target.valueAsNumber; view._linePreview = Number.isFinite(v) ? v / 1000 : null; view.draw(); };
$('btn-line-len').onclick = () => {
  const v = $('ed-len').valueAsNumber;
  if (!Number.isFinite(v) || v <= 0) { log('type the straight length in mm first', 'bad'); return; }
  addStraight(v / 1000);
};
$('ed-len').onkeydown = e => { if (e.key === 'Enter') { e.preventDefault(); $('btn-line-len').onclick(); } };
// Reverse: backs up along the current heading, facing forward. Bounded and slow (half the
// speed cap, set by the compiler) because the rear is not covered by the safety scanner.
$('btn-reverse-len').onclick = () => {
  const v = $('ed-len').valueAsNumber;
  if (!Number.isFinite(v) || v <= 0) { log('type the reverse distance in mm first', 'bad'); return; }
  if (!route.start) { log('set the start pose first', 'bad'); return; }
  const d = v / 1000;
  if (d < MIN_STRAIGHT_M || d > REVERSE_MAX_M) { log(`a reverse is ${mm(MIN_STRAIGHT_M)}–${mm(REVERSE_MAX_M)} mm`, 'bad'); return; }
  const id = nextId();
  snapshot(); route.steps.push({ id, type: 'reverse', distance_m: d }); refresh();
  log(`${id}: reverse ${mm(d)} mm (half speed, rear not scanned)`);
};
// Arc: forward along a circle, left or right. Bounds mirror the schema; the speed is capped
// by the compiler so v/R stays under the turn cap (shown in the step list).
function addArc(direction) {
  if (!route.start) { log('set the start pose first', 'bad'); return; }
  const r = $('ed-arc-radius').valueAsNumber, a = $('ed-arc-angle').valueAsNumber;
  if (!(r >= ARC_MIN_R)) { log(`arc radius is at least ${ARC_MIN_R} m (2 x track)`, 'bad'); return; }
  if (!(a >= ARC_MIN_DEG && a <= ARC_MAX_DEG)) { log(`arc angle is ${ARC_MIN_DEG}–${ARC_MAX_DEG}°`, 'bad'); return; }
  const id = nextId();
  snapshot(); route.steps.push({ id, type: 'arc', direction, angle_deg: a, radius_m: r }); refresh();
  log(`${id}: arc ${direction === 'ccw' ? 'L' : 'R'} ${a}° R ${num(r, 2)} m (${num(arcSpeed(r), 2)} m/s)`);
}
// Wait (gls-wait-coding-plan §A.5): stop here until PB Continue or Start - a trolley hitch or
// release point. Placement rules are the compiler's (never first or last, never two in a row,
// never straight after an arc); Validate reports them against the step.
$('btn-wait').onclick = () => {
  if (!route.start) { log('set the start pose first', 'bad'); return; }
  const name = $('ed-wait-name').value.trim(), label = $('ed-wait-label').value.trim();
  if (name.length > 40) { log('a wait name is at most 40 characters', 'bad'); return; }
  if (label && !/^x\d{4}y\d{4}$/.test(label)) { log('a floor label looks like x0100y0000', 'bad'); return; }
  const id = nextId(), step = { id, type: 'wait', name };
  if (label) step.label_id = label;
  snapshot(); route.steps.push(step); refresh();
  log(`${id}: wait${name ? ' at ' + name : ''}${label ? ' (label ' + label + ')' : ''}: press Continue to go on`);
};
// Go home (2026-11-03): the route's last step, from where it ends now to the start pose (HOME): a
// straight when already on HOME's line, else an S-curve. Refused here with the reason when it
// cannot be built; Validate checks it again, with the footprint sweep.
$('btn-home').onclick = () => {
  if (!route.start) { log('set the start pose first', 'bad'); return; }
  if (!route.steps.length) { log('draw the route first: the home leg returns to the start pose from where the route ends', 'bad'); return; }
  if (route.steps.some(s => s.type === 'home')) { log('the route already ends with a home leg', 'bad'); return; }
  const L = homeLegAt(route.steps.length);
  if (!L.ok) { log(`home leg: ${L.why}`, 'bad'); return; }
  const id = nextId();
  snapshot(); route.steps.push({ id, type: 'home' }); refresh();
  log(`${id}: home leg, ${homeText(L)}`);
};
$('btn-arc-l').onclick = () => addArc('ccw');
$('btn-arc-r').onclick = () => addArc('cw');
document.querySelectorAll('button.turn').forEach(b => b.addEventListener('click', () => {
  if (!route.start) { log('set the start pose first', 'bad'); return; }
  snapshot(); route.steps.push({ id: nextId(), type: 'rotate', direction: b.dataset.dir, angle_deg: +b.dataset.ang }); refresh();
}));
$('tool-start').onclick = () => setTool(view.tool === 'start' ? null : 'start');
// Most routes begin where the survey began: one press puts the route start on the map's
// survey mark (position and heading) instead of drawing it by hand.
$('btn-start-survey').onclick = () => {
  const st = view.meta && view.meta.start;
  if (!st || st.x_m === undefined) { log('this map has no survey start mark', 'bad'); return; }
  snapshot(); route.start = { x_m: st.x_m, y_m: st.y_m, yaw_deg: (st.yaw_rad || 0) * 180 / Math.PI }; refresh();
  log(`route start = survey start (${num(st.x_m, 2)}, ${num(st.y_m, 2)}, ${num(route.start.yaw_deg, 1)}°)${st.description ? ' — ' + st.description : ''}`);
};
// A homepost of its own (2026-11-06): the route start where the AMR stands now, parked on the
// homepost and localised on THIS map - the pose the executor's START check will compare with.
$('btn-start-here').onclick = () => {
  const r = lastState && lastState.run, loc = lastState && lastState.localization, md = lastState && lastState.mode;
  const onActive = md && view.meta && md.active_map_id === view.meta.map_id && +md.active_map_revision === +view.meta.revision;
  if (!onActive) { log('the AMR is not running on this map: activate it (Run page) first', 'bad'); return; }
  if (!loc || loc.state_name !== 'READY' || !r || !r.pose_valid) { log(`the AMR is not localised (${loc ? loc.state_name : 'no localisation'}): set its pose on the Run page and wait for READY`, 'bad'); return; }
  snapshot(); route.start = { x_m: +r.pose_x, y_m: +r.pose_y, yaw_deg: +r.pose_yaw * 180 / Math.PI }; refresh();
  log(`route start = the AMR (${num(route.start.x_m, 3)}, ${num(route.start.y_m, 3)}, ${num(route.start.yaw_deg, 1)}°)` +
    (route.steps.length ? ' — the steps keep their end points: Validate before saving' : ''));
};
$('tool-line').onclick = () => setTool(view.tool === 'line' ? null : 'line');
$('tool-measure').onclick = () => setTool(view.tool === 'measure' ? null : 'measure');
$('btn-undo').onclick = undo; $('btn-redo').onclick = redo;
$('btn-clear').onclick = () => { snapshot(); route.steps = []; refresh(); };
$('ed-repeat').onchange = e => { snapshot(); route.repeat_count = +e.target.value; refresh(); };
// Steps an ERROR issue names are drawn red; `info` issues (a sweep crossing mapped cells in a
// dynamic area: passable only if the live scan agrees) never fail validation.
const badSteps = () => new Set((lastResult && lastResult.issues || []).filter(i => i.severity !== 'info').map(i => i.step_id));

function payload() {
  return { schema_version: 1, route_id: $('ed-route-id').value.trim(), revision: 0,
           map: { id: mapId, revision: mapRev, sha256: '' }, frame_id: 'map',
           start: route.start || { x_m: 0, y_m: 0, yaw_deg: 0 }, limits: routeLimits(route.limits), steps: route.steps, repeat_count: route.repeat_count };
}
$('btn-validate').onclick = async () => {
  const c = ctx();
  const { status, data } = await api(`/api/maps/${mapId}/${mapRev}/routes/validate`, payload());
  if (!stillCurrent(c)) { log('validation result discarded: the draft or map changed meanwhile', 'warn'); return; }
  lastResult = data; showResult(data); log(status === 200 ? `valid: ${data.compiled.total_length_m.toFixed(2)} m, ${data.compiled.steps.length} steps` : `invalid: ${data.issues.length} issue(s)`, status === 200 ? '' : 'bad'); refresh();
};
$('btn-save').onclick = async () => {
  const c = ctx(), forMap = { map_id: mapId, map_revision: mapRev };
  const { status, data } = await api(`/api/maps/${mapId}/${mapRev}/routes/save`, payload());
  if (status === 200) {
    log(`saved ${data.route_id} rev${data.revision} (${data.sha256.slice(0, 12)})`);
    // the saved reference names the FULL identity it was saved under, whatever is selected now
    if (stillCurrent(c)) savedRef = Object.assign({ route_id: data.route_id, revision: data.revision }, forMap);
    else log('saved, but the draft changed meanwhile: save again to create a mission from it', 'warn');
    if (stillSameMap(c)) await reloadRouteList(c);
    if (stillSameMap(c) && view.tool === 'measure') await loadSavedGeometry();
  } else if (stillCurrent(c)) { lastResult = data; showResult(data); refresh(); }
};
$('btn-mission').onclick = async () => {
  if (!savedRef) { log('save this draft as a revision first', 'bad'); return; }
  if (savedRef.map_id !== mapId || +savedRef.map_revision !== +mapRev) { log('the saved revision belongs to another map; save on this map first', 'bad'); return; }
  const { status, data } = await api('/api/missions', { map_id: savedRef.map_id, map_revision: savedRef.map_revision, route_id: savedRef.route_id, route_revision: savedRef.revision });
  if (status === 200) log(`mission ${data.mission_id} created` + (data.labels ? ` with label set rev${data.labels.revision}` : ' (no label set on this map: labels are only logged)'));
};
function showResult(d) {
  if (!d) { $('ed-result').innerHTML = '<div class="none">not validated since the last edit</div>'; return; }
  const parts = [];
  const prov = (d.issues || []).filter(i => i.code === 'provisional').length;
  parts.push(`<div class="chips" style="margin-bottom:8px"><span class="chip ${d.ok ? 'ok' : 'bad'}">${d.ok ? 'valid' : 'invalid'}</span>` +
    (prov ? `<span class="chip warn" title="crosses mapped objects in dynamic areas: cleared on the assumption they are empty, and nothing but the safety scanner will stop the vehicle if they are not">${prov} provisional</span>` : '') + '</div>');
  if (d.compiled) {
    parts.push(`<div class="tel-grid">${[['Length', num(d.compiled.total_length_m, 2), 'm'], ['Turns', num(d.compiled.total_turn_deg, 0), '°'],
      ['Closes', d.compiled.closes ? 'YES' : 'NO', 'loop'], ['Speed', num(SPEED_V, 2), 'm/s']]
      .map(([l, v, u]) => `<div class="tel"><span>${l}</span><b>${esc(v)}</b><i>${u}</i></div>`).join('')}</div>`);
  }
  if (d.issues && d.issues.length) {
    parts.push(`<div class="al-log" style="margin-top:8px">${d.issues.map(i => `<div class="al-row standing ${i.severity === 'info' ? 'lv-warn' : 'lv-error'}"><span class="al-lv">${esc(i.step_id || i.code || 'route')}</span>` +
      `<span class="al-msg">${esc(i.message)}</span></div>`).join('')}</div>`);
  }
  $('ed-result').innerHTML = parts.join('');
}
function refresh() {
  const bad = badSteps();
  if (!lastResult) showResult(null);
  view._linePreview = null;  // the route changed: a preview made for the old end pose is void
  $('ed-steps').innerHTML = route.steps.length ? route.steps.map((s, i) => `<div class="row three${bad.has(s.id) ? ' lv-error' : ''}"><span class="k">${esc(s.id)}</span>` +
    (s.type === 'straight' ? `<span class="n">straight <input class="sv" data-i="${i}" type="number" step="0.05" min="${STEP_V_MIN}" max="${num(SPEED_V, 2)}" value="${s.v_mps != null ? num(+s.v_mps, 2) : ''}" placeholder="${num(SPEED_V, 2)}" title="m/s: empty = the route speed; lower = a slow zone" style="width:5.5em"></span><span class="v" title="to ${num(+s.to.x_m, 3)}, ${num(+s.to.y_m, 3)} m">${mm(stepLength(i))}<i>mm</i></span>`
     : s.type === 'reverse' ? `<span class="n">reverse <b title="backs up at half the route speed, at most 0.30 m/s: ${num(REVERSE_V, 2)} m/s">◂${num(REVERSE_V, 2)}</b></span><span class="v">${mm(+s.distance_m)}<i>mm</i></span>`
     : s.type === 'wait' ? `<span class="n">wait</span><span class="v" title="stops until PB Continue or Start">${esc(s.name || '—')}${s.label_id ? ` <i>${esc(s.label_id)}</i>` : ''}</span>`
     : s.type === 'arc' ? `<span class="n">arc ${s.direction === 'ccw' ? 'L' : 'R'} <b title="capped so the yaw rate stays under the turn cap">≤${num(arcSpeed(+s.radius_m), 2)}</b></span><span class="v" title="${mm(stepLength(i))} mm of arc">${esc(s.angle_deg)}° R ${num(+s.radius_m, 2)}<i>m</i></span>`
     : s.type === 'home' ? homeRow(i)
                           : `<span class="n">rotate</span><span class="v">${esc(String(s.direction).toUpperCase())} ${esc(s.angle_deg)}<i>°</i></span>`) + '</div>').join('')
    : '<div class="none">no steps</div>';
  $('ed-repeat').value = route.repeat_count;
  const pick = $('ed-split-step'), keep = pick.value;
  pick.innerHTML = '<option value="">— straight —</option>' + route.steps.map((s, i) => s.type === 'straight'
    ? `<option value="${esc(s.id)}">${esc(s.id)} · ${mm(stepLength(i))} mm${s.v_mps != null ? ` · ${num(+s.v_mps, 2)} m/s` : ''}</option>` : '').join('');
  pick.value = route.steps.some(s => s.id === keep) ? keep : '';
  view.draw();
}
function homeRow(i) {
  const L = homeLegAt(i);
  if (!L.ok) return `<span class="n">home</span><span class="v" title="${esc(L.why)}">cannot reach HOME</span>`;
  const speed = L.straight ? SPEED_V : arcSpeed(L.minR);
  return `<span class="n">home <b title="${esc(homeText(L))}">${L.straight ? 'straight' : 'S-curve'} ${num(speed, 2)}</b></span><span class="v" title="${esc(homeText(L))}">${mm(L.length)}<i>mm</i></span>`;
}
// The areas validation checks (amr_navigation.footprint), drawn from the same polygon + margin.
// A straight sweeps the grown footprint box along the heading; a turn sweeps it through the
// signed angle: the outline is the convex hull of the grown polygon at headings 5 deg apart.
function grown() {
  const xs = footprint.polygon.map(p => p[0]), ys = footprint.polygon.map(p => p[1]), m = footprint.margin_m;
  return [Math.min(...xs) - m, Math.max(...xs) + m, Math.min(...ys) - m, Math.max(...ys) + m];
}
function sweptBox(pose, len) {
  const [x0, x1, y0, y1] = grown(), c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  return [[x0, y0], [x1 + len, y0], [x1 + len, y1], [x0, y1]].map(p => [pose.x + c * p[0] - s * p[1], pose.y + s * p[0] + c * p[1]]);
}
// The band the grown footprint box sweeps along an arc: its outer-side corners forward, then
// its inner-side corners back (not a convex hull: a 180 deg sweep's hull would cover the aisle).
function sweptArc(pose, s) {
  const [x0, x1, y0, y1] = grown(), sign = s.direction === 'ccw' ? 1 : -1;
  const poses = arcSamples(pose, s, Math.max(2, Math.ceil(s.angle_deg / 5)));
  const at = (p, px, py) => [p.x + Math.cos(p.yaw) * px - Math.sin(p.yaw) * py, p.y + Math.sin(p.yaw) * px + Math.cos(p.yaw) * py];
  const outer = sign > 0 ? y0 : y1, inner = sign > 0 ? y1 : y0;  // a left arc's outer side is its right (-y)
  const fwd = [at(poses[0], x0, inner), at(poses[0], x0, outer)];
  poses.forEach(p => fwd.push(at(p, x1, outer)));
  const back = [at(poses[poses.length - 1], x1, inner)];
  poses.slice().reverse().forEach(p => back.push(at(p, x0, inner)));
  return fwd.concat(back);
}
// The home leg's band: the convex hull of the grown box at each sample (the curve turns at most
// 30 deg off HOME's line, so the hull adds little over the true sweep and never hides any of it).
function sweptPath(samples) {
  const [x0, x1, y0, y1] = grown(), pts = [];
  samples.forEach(p => { const c = Math.cos(p.yaw), s = Math.sin(p.yaw); [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].forEach(q => pts.push([p.x + c * q[0] - s * q[1], p.y + s * q[0] + c * q[1]])); });
  return hull(pts);
}
function sweptTurn(pose, signed) {
  const [x0, x1, y0, y1] = grown(), pts = [], n = Math.max(1, Math.ceil(Math.abs(signed) / (Math.PI / 36)));
  for (let k = 0; k <= n; k++) {
    const h = pose.yaw + signed * k / n, c = Math.cos(h), s = Math.sin(h);
    [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].forEach(p => pts.push([pose.x + c * p[0] - s * p[1], pose.y + s * p[0] + c * p[1]]));
  }
  return hull(pts);
}
function hull(pts) { // Andrew monotone chain
  pts = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of pts) { while (lo.length > 1 && cross(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (const p of pts.reverse()) { while (up.length > 1 && cross(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}
view.overlays.push((c, v) => {
  if (!route.start) { if (v._preview) v.arrow(v._preview.x, v._preview.y, v._preview.yaw, 1.0, INK.preview); return; }
  const bad = badSteps();
  let p = poseAfter(0);
  // What validation actually checks (footprint + margin): the standing footprint at the start,
  // the swept box of every straight, the swept hull of every turn. Faint, under the route;
  // red when that step failed.
  if (footprint) {
    const checked = (id, col) => alpha(bad.has(id) ? INK.stop : col, 0.35);
    v.dashed(sweptBox(p, 0), checked(null, INK.pose));
    let a = p;
    route.steps.forEach((s, i) => {
      const b = poseAfter(i + 1);
      if (s.type === 'straight') v.dashed(sweptBox(a, Math.hypot(b.x - a.x, b.y - a.y)), checked(s.id, INK.route));
      else if (s.type === 'reverse') v.dashed(sweptBox(b, Math.hypot(b.x - a.x, b.y - a.y)), checked(s.id, INK.route));  // b is behind a, same heading
      else if (s.type === 'arc') v.dashed(sweptArc(a, s), checked(s.id, INK.route));
      else if (s.type === 'wait') v.dashed(sweptBox(a, 0), checked(s.id, INK.pose));
      else if (s.type === 'home') { const L = homeLeg(a, poseAfter(0)); if (L.ok) v.dashed(sweptPath(L.samples), checked(s.id, INK.route)); }
      else v.dashed(sweptTurn(a, (s.direction === 'ccw' ? 1 : -1) * s.angle_deg * Math.PI / 180), checked(s.id, INK.turn));
      a = b;
    });
  }
  v.arrow(p.x, p.y, p.yaw, 1.0, INK.pose); if (footprint) v.footprint(p.x, p.y, p.yaw, footprint.polygon, alpha(INK.pose, 0.55));
  route.steps.forEach((s, i) => {
    const q = poseAfter(i + 1);
    if (s.type === 'straight') {
      const slow = s.v_mps != null && +s.v_mps < SPEED_V - 1e-9;
      v.line(p.x, p.y, q.x, q.y, bad.has(s.id) ? INK.stop : slow ? INK.turn : INK.route, 3);
      v.text((p.x + q.x) / 2, (p.y + q.y) / 2, `${s.id} ${mm(Math.hypot(q.x - p.x, q.y - p.y))} mm${slow ? ` ${num(+s.v_mps, 2)} m/s` : ''}`, slow ? INK.turn : INK.ink3);
      if (i > 0 && route.steps[i - 1].type === 'straight') v.dot(p.x, p.y, INK.turn, 4);  // a speed waypoint
    }
    else if (s.type === 'reverse') { v.dashed([[p.x, p.y], [q.x, q.y]], bad.has(s.id) ? INK.stop : INK.route); v.dot(q.x, q.y, INK.route, 4); v.text((p.x + q.x) / 2, (p.y + q.y) / 2, `${s.id} ◂ ${mm(Math.hypot(q.x - p.x, q.y - p.y))} mm`, INK.ink3); }
    else if (s.type === 'arc') { const pts = arcSamples(p, s, Math.max(8, Math.ceil(s.angle_deg / 3))); v.polyline(pts.map(t => [t.x, t.y]), bad.has(s.id) ? INK.stop : INK.route, 3); const m = pts[pts.length >> 1]; v.text(m.x, m.y, `${s.id} ⌒ R${num(+s.radius_m, 1)} ${s.angle_deg}°`, INK.ink3); v.arrow(q.x, q.y, q.yaw, 0.6, INK.turn); if (footprint) v.footprint(q.x, q.y, q.yaw, footprint.polygon, alpha(INK.turn, 0.4)); }
    else if (s.type === 'wait') { v.dot(q.x, q.y, bad.has(s.id) ? INK.stop : INK.pose, 7); v.text(q.x, q.y, `${s.id} WAIT ${s.name || ''}`, INK.pose); }
    else if (s.type === 'home') {
      const L = homeLeg(p, q);
      if (L.ok) { v.polyline(L.samples.map(t => [t.x, t.y]), bad.has(s.id) ? INK.stop : INK.route, 3); const m = L.samples[L.samples.length >> 1]; v.text(m.x, m.y, `${s.id} HOME ${L.straight ? '' : 'S '}${mm(L.length)} mm`, INK.ink3); }
      else { v.dashed([[p.x, p.y], [q.x, q.y]], INK.stop); v.text((p.x + q.x) / 2, (p.y + q.y) / 2, `${s.id} home: cannot reach HOME`, INK.stop); }
      v.arrow(q.x, q.y, q.yaw, 0.6, INK.pose);
    }
    else { v.dot(q.x, q.y, bad.has(s.id) ? INK.stop : INK.turn, 6); v.text(q.x, q.y, `${s.id} ${s.direction.toUpperCase()} ${s.angle_deg}°`, INK.turn); v.arrow(q.x, q.y, q.yaw, 0.6, INK.turn); if (footprint) v.footprint(q.x, q.y, q.yaw, footprint.polygon, alpha(INK.turn, 0.4)); }
    p = q;
  });
  if (v._preview && v.tool === 'start') v.arrow(v._preview.x, v._preview.y, v._preview.yaw, 1.0, INK.preview);
  // the straight a click or the typed length would add, with its length in mm
  if (v._linePreview >= MIN_STRAIGHT_M) {
    const e = poseAfter(route.steps.length), q = { x: e.x + Math.cos(e.yaw) * v._linePreview, y: e.y + Math.sin(e.yaw) * v._linePreview };
    v.dashed([[e.x, e.y], [q.x, q.y]], INK.preview); v.dot(q.x, q.y, INK.preview, 4);
    v.text((e.x + q.x) / 2, (e.y + q.y) / 2, `${mm(v._linePreview)} mm`, INK.preview);
  }
  const st = lastState && lastState.localization;
  const md = lastState && lastState.mode;
  const onActive = md && v.meta && md.active_map_id === v.meta.map_id && +md.active_map_revision === +v.meta.revision;  // R12
  if (onActive && st && st.state_name !== 'UNLOCALIZED' && lastState.run && lastState.run.pose_x !== undefined) v.arrow(lastState.run.pose_x, lastState.run.pose_y, lastState.run.pose_yaw, 0.8, INK.accent);
});
async function reloadRouteList(c) {
  c = c || ctx();
  const { data } = await apiGet(`/api/maps/${mapId}/${mapRev}`);
  if (!stillSameMap(c)) return;  // a list for a map no longer selected
  const sel = $('ed-load'); sel.innerHTML = '<option value="">— new —</option>';
  Object.entries(data.routes || {}).forEach(([rid, revs]) => revs.forEach(r => { const o = document.createElement('option'); o.value = `${rid}/${r}`; o.textContent = `${rid} rev${r}`; sel.appendChild(o); }));
  await reloadLabels(c);
}
// The wait's label dropdown (gls-wait-coding-plan §C.1): the newest label set of this map revision.
async function reloadLabels(c) {
  const { data } = await apiGet(`/api/maps/${mapId}/${mapRev}/labels`);
  if (!stillSameMap(c)) return;
  const sel = $('ed-wait-label');
  const labels = (data && data.labels) || [];
  sel.innerHTML = `<option value="">${labels.length ? '— no label —' : '— no labels recorded on this map —'}</option>`;
  labels.filter(l => l.role === 'dest').forEach(l => { const o = document.createElement('option'); o.value = l.label_id; o.textContent = `${l.name} (${l.label_id})`; sel.appendChild(o); });
}
// Remove a saved route (engineer): one revision or all of them, with the missions using them.
async function removeRoute(all) {
  const v = $('ed-load').value;
  if (!v) { log('pick a saved route under Load saved first', 'bad'); return; }
  const [rid, rrev] = v.split('/');
  const ok = await removeConfirmed(`/api/maps/${encodeURIComponent(mapId)}/routes/${encodeURIComponent(rid)}/delete`, { revision: all ? null : +rrev });
  if (!ok) return;
  if (savedRef && savedRef.route_id === rid && (all || savedRef.revision === +rrev)) savedRef = null;  // the draft stays, unsaved
  await reloadRouteList();
}
if ($('btn-route-remove-rev')) $('btn-route-remove-rev').onclick = () => removeRoute(false);
if ($('btn-route-remove')) $('btn-route-remove').onclick = () => removeRoute(true);
$('ed-load').onchange = async e => {
  if (!e.target.value) return;
  const [rid, rrev] = e.target.value.split('/');
  const c = ctx(), forMap = { map_id: mapId, map_revision: mapRev };
  const { data } = await apiGet(`/api/maps/${mapId}/${mapRev}/routes/${rid}/${rrev}`);
  if (!stillCurrent(c)) { log(`load of ${rid} rev${rrev} discarded: the draft or map changed meanwhile`, 'warn'); return; }
  snapshot(); route = { route_id: data.route.route_id, start: data.route.start, steps: data.route.steps, repeat_count: data.route.repeat_count, limits: routeLimits(data.route.limits) };
  $('ed-route-id').value = data.route.route_id; savedRef = Object.assign({ route_id: rid, revision: +rrev }, forMap); lastResult = data; showResult(data); refresh();
  log(`loaded ${rid} rev${rrev}${data.ok ? '' : ' (INVALID on this map revision)'}`, data.ok ? '' : 'bad');
};
$('ed-map').onchange = async e => {
  [mapId, mapRev] = e.target.value.split('/'); mapRev = +mapRev;
  mapToken += 1; draftToken += 1;  // everything in flight for the previous map is void
  const c = ctx();
  route = { route_id: $('ed-route-id').value, start: null, steps: [], repeat_count: 1, limits: DEFAULT_LIMITS() }; history = []; future = []; lastResult = null; savedRef = null;
  savedGeom = { map: -1, routes: [] }; clearMeasure(); refresh();
  await view.load(mapId, mapRev);
  if (!stillSameMap(c)) return;
  await reloadRouteList(c);
  if (view.tool === 'measure') await loadSavedGeometry();
};
// --- Measure (view only, 2026-11-03): two clicks -> distance, dX, dY in map mm, snapped to route
// features (static/measure.js). Never touches the draft, its undo history or the last validation.
view.panTools = new Set(['measure']);
let measure = { a: null, b: null, hover: null }, savedGeom = { map: -1, routes: [] }, snapCache = null, shiftHeld = false;
function clearMeasure() { measure = { a: null, b: null, hover: null }; showMeasure(); }
// The draft in the compiler's step shape: the same poses the overlay draws.
function draftGeometry() {
  if (!route.start) return null;
  const P = [poseAfter(0)];
  route.steps.forEach((s, i) => P.push(poseAfter(i + 1)));
  const steps = route.steps.map((s, i) => {
    const a = P[i], b = P[i + 1], g = { id: s.id, type: s.type, start: [a.x, a.y, a.yaw], end: [b.x, b.y, b.yaw], centre: null, radius_m: 0, signed_angle_deg: 0, name: s.name || '' };
    if (s.type === 'arc') { const sign = s.direction === 'ccw' ? 1 : -1; g.centre = arcCentre(a.x, a.y, a.yaw, +s.radius_m, sign); g.radius_m = +s.radius_m; g.signed_angle_deg = sign * s.angle_deg; }
    if (s.type === 'home') { const L = homeLeg(a, b); if (L.ok) g.samples = L.samples.map(t => [t.x, t.y]); }
    return g;
  });
  return { label: 'draft', start: [P[0].x, P[0].y, P[0].yaw], steps };
}
// Rebuilt only when the draft, the map or the saved routes changed: hover stays cheap.
function snapSet() {
  const key = `${draftToken}/${mapToken}/${savedGeom.map}/${savedGeom.routes.length}`;
  if (snapCache && snapCache.key === key) return snapCache.set;
  const routes = [draftGeometry(), ...savedGeom.routes].filter(Boolean);
  const st = view.meta && view.meta.start, extra = st && st.x_m !== undefined ? [{ x: st.x_m, y: st.y_m, kind: 'vertex', label: 'survey start' }] : [];
  snapCache = { key, set: measureSnapSet(routes, extra) };
  return snapCache.set;
}
// Snap reach: ~14 screen px (22 on a touch panel), whatever the zoom.
function snapTolM() {
  const coarse = window.matchMedia && window.matchMedia('(pointer: coarse)').matches;
  const k = typeof uiScale === 'function' ? uiScale() : 1;
  return view.meta && view.scale > 0 ? (coarse ? 22 : 14) * k * view.meta.resolution / view.scale : 0;
}
function snapAt(w) {
  const on = $('ed-snap').checked !== false && !shiftHeld;
  return on ? measureSnap(w, snapSet(), snapTolM()) : { x: w[0], y: w[1], kind: 'free', label: 'free point' };
}
function measureClick(w) {
  const p = snapAt(w);
  if (!measure.a || measure.b) measure = { a: p, b: null, hover: p };
  else measure.b = p;
  showMeasure();
}
const signedMm = m => (m > 0 ? '+' : '') + mm(m);
function showMeasure() {
  const a = measure.a, b = measure.b || (a ? measure.hover : null);
  if (!a) { $('ed-measure').innerHTML = `<div class="none">${view.tool === 'measure' ? 'click point A on the map' : 'select Measure, then click two points on the map'}</div>`; return; }
  const pt = (n, p) => `<div class="row three"><span class="k">${n}</span><span class="n">${esc(p.label)}</span><span class="v">${num(p.x, 3)}, ${num(p.y, 3)}<i>m</i></span></div>`;
  const parts = [pt('A', a)];
  if (b) {
    const d = measureDelta(a, b);
    parts.unshift(`<div class="tel-grid">${[['Distance', mm(d.dist)], ['ΔX', signedMm(d.dx)], ['ΔY', signedMm(d.dy)]]
      .map(([l, v]) => `<div class="tel"><span>${l}</span><b>${esc(v)}</b><i>mm</i></div>`).join('')}</div>`);
    parts.push(pt(measure.b ? 'B' : 'B?', b));
  }
  $('ed-measure').innerHTML = `<div class="list">${parts.join('')}</div>`;
}
// Newest revision of every saved route on this map (same frame for all its revisions): compiled
// only, never validated or previewed, so asking costs little and moves nothing.
async function loadSavedGeometry() {
  if (!mapId) return;
  const c = ctx();
  const { status, data } = await apiGet(`/api/maps/${mapId}/${mapRev}/routes/geometry`);
  if (!stillSameMap(c)) return;
  const routes = status === 200 && data && Array.isArray(data.routes) ? data.routes.filter(r => !r.error && Array.isArray(r.steps)) : [];
  savedGeom = { map: c.m, routes: routes.map(r => ({ label: `${r.route_id} r${r.revision}`, start: r.start, steps: r.steps })) };
  snapCache = null; view.draw();
}
if (window.addEventListener) {
  window.addEventListener('keydown', e => {
    if (e.key === 'Shift') shiftHeld = true;
    if (e.key === 'Escape' && view.tool === 'measure') { clearMeasure(); view.draw(); }
  });
  window.addEventListener('keyup', e => { if (e.key === 'Shift') shiftHeld = false; });
  window.addEventListener('blur', () => { shiftHeld = false; });
}
// Glyph per snap kind: wait = square, arc centre = cross, step end = ring, on a line = diamond.
function snapGlyph(v, p, color) {
  if (!p || p.kind === 'free') return;
  const c = v.ctx, s = v.worldToScreen(p.x, p.y), r = 7;
  c.save(); c.strokeStyle = color; c.lineWidth = 2; c.beginPath();
  if (p.kind === 'wait') c.rect(s[0] - r, s[1] - r, 2 * r, 2 * r);
  else if (p.kind === 'centre') { c.moveTo(s[0] - r, s[1]); c.lineTo(s[0] + r, s[1]); c.moveTo(s[0], s[1] - r); c.lineTo(s[0], s[1] + r); }
  else if (p.kind === 'vertex') c.arc(s[0], s[1], r, 0, 2 * Math.PI);
  else { c.moveTo(s[0], s[1] - r); c.lineTo(s[0] + r, s[1]); c.lineTo(s[0], s[1] + r); c.lineTo(s[0] - r, s[1]); c.closePath(); }
  c.stroke(); c.restore();
}
view.overlays.push((c, v) => {
  if (v.tool !== 'measure') return;
  const faint = alpha(INK.ink3, 0.7);
  savedGeom.routes.forEach(r => (r.steps || []).forEach(s => {
    if (s.type === 'straight' || s.type === 'reverse') v.line(s.start[0], s.start[1], s.end[0], s.end[1], faint, 1.5);
    else if (s.type === 'arc') { const arc = measureArcOf(s); if (arc) v.polyline(measureArcPoints(arc, Math.max(8, Math.ceil(Math.abs(arc.sweep) / 0.05))), faint, 1.5); }
    else if (s.type === 'wait') v.dot(s.end[0], s.end[1], faint, 4);
    else if (s.type === 'home' && Array.isArray(s.samples)) v.polyline(s.samples, faint, 1.5);
  }));
  const a = measure.a, b = measure.b || (a ? measure.hover : null);
  if (!measure.b) snapGlyph(v, measure.hover, INK.preview);
  if (a && b) {
    const sa = v.worldToScreen(a.x, a.y), sk = v.worldToScreen(b.x, a.y), sb = v.worldToScreen(b.x, b.y);
    c.save(); c.strokeStyle = alpha(INK.accent, 0.6); c.lineWidth = 1; c.setLineDash([4, 4]);
    c.beginPath(); c.moveTo(sa[0], sa[1]); c.lineTo(sk[0], sk[1]); c.lineTo(sb[0], sb[1]); c.stroke(); c.restore();
    v.line(a.x, a.y, b.x, b.y, INK.accent, 2);
    const d = measureDelta(a, b);
    v.text((a.x + b.x) / 2, (a.y + b.y) / 2, `${mm(d.dist)} mm  ΔX ${signedMm(d.dx)}  ΔY ${signedMm(d.dy)}`, INK.accent);
  }
  if (a) { v.dot(a.x, a.y, INK.accent, 4); snapGlyph(v, a, INK.accent); }
  if (measure.b) { v.dot(b.x, b.y, INK.accent, 4); snapGlyph(v, b, INK.accent); }
});
(async () => {
  footprint = (await apiGet('/api/footprint')).data;
  mapsIndex = await fillMapSelect($('ed-map'));
  if ($('ed-map').options.length) { $('ed-map').selectedIndex = 0; $('ed-map').onchange({ target: $('ed-map') }); }
})();
