// Route editor: constrained straight/turn tools over a MapView. Persists map metres,
// never pixels. Validation and saving happen on the robot (POST); nothing moves.
const view = new MapView(document.getElementById('ed-canvas'));
let mapId = null, mapRev = null, footprint = null, mapsIndex = [];
const DEFAULT_LIMITS = () => ({ linear_mps: 0.55, long_linear_mps: 0.85, long_min_length_m: 4.0, arc_linear_mps: 0.40 });
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
const ARC_MIN_R = 1.0, ARC_MIN_DEG = 45, ARC_MAX_DEG = 180;  // amr_navigation.route ARC_* bounds
// mirrors amr_navigation route.py / compiler.arc_speed (2026-09-19): an arc runs at arc_linear_mps
// (absent in older files: 0.40), never above linear_mps, and so that v/R <= 0.9 x VEHICLE_ARC_W_MAX
const ARC_DEFAULT_MPS = 0.40, ARC_YAW_RATE_RATIO = 0.9, VEHICLE_ARC_W_MAX = 0.45;
const arcCap = () => Math.min(route.limits.arc_linear_mps ?? ARC_DEFAULT_MPS, route.limits.linear_mps);
// Same construction as compiler.arc_centre / arc_pose: the circle to the left for +1 (ccw).
function arcCentre(x, y, yaw, r, sign) { return [x - sign * r * Math.sin(yaw), y + sign * r * Math.cos(yaw)]; }
function arcPose(c, r, yaw0, sign, phi) { const yaw = yaw0 + sign * phi; return { x: c[0] + sign * r * Math.sin(yaw), y: c[1] - sign * r * Math.cos(yaw), yaw }; }
function arcSamples(a, s, n) {  // poses along step s (an arc) from pose a, n+1 of them
  const sign = s.direction === 'ccw' ? 1 : -1, c = arcCentre(a.x, a.y, a.yaw, s.radius_m, sign), th = s.angle_deg * Math.PI / 180;
  return Array.from({ length: n + 1 }, (_, k) => arcPose(c, s.radius_m, a.yaw, sign, th * k / n));
}
const arcSpeed = r => Math.min(arcCap(), ARC_YAW_RATE_RATIO * VEHICLE_ARC_W_MAX * r);

function setTool(t) { view.tool = t; view._linePreview = null; ['tool-start', 'tool-line'].forEach(id => $(id).classList.toggle('on', id === 'tool-' + t)); $('ed-hint').textContent = t === 'start' ? 'Click the start position, drag towards the heading, release.' : t === 'line' ? 'Click a point, or type a length in mm and press Add: the straight goes along the current heading.' : 'Wheel = zoom, drag = pan.'; view.draw(); }

// Heading after the last step, from the authored primitives (same rules as the compiler).
function poseAfter(n) {
  if (!route.start) return null;
  let x = route.start.x_m, y = route.start.y_m, yaw = route.start.yaw_deg * Math.PI / 180;
  for (let i = 0; i < n; i++) {
    const s = route.steps[i];
    if (s.type === 'straight') { x = s.to.x_m; y = s.to.y_m; }
    else if (s.type === 'reverse') { x -= Math.cos(yaw) * s.distance_m; y -= Math.sin(yaw) * s.distance_m; }
    else if (s.type === 'arc') { const e = arcSamples({ x, y, yaw }, s, 1)[1]; x = e.x; y = e.y; yaw = e.yaw; }
    else yaw += (s.direction === 'ccw' ? 1 : -1) * s.angle_deg * Math.PI / 180;
  }
  return { x, y, yaw };
}
function stepLength(i) { const s = route.steps[i]; if (s.type === 'arc') return s.radius_m * s.angle_deg * Math.PI / 180; const a = poseAfter(i), b = poseAfter(i + 1); return Math.hypot(b.x - a.x, b.y - a.y); }
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
  snapshot(); route.steps.push({ id, type: 'straight', to: { x_m: p.x + c * along, y_m: p.y + s * along } }); refresh();
  log(`${id}: straight ${mm(along)} mm`);
  return true;
}
view.onClick = w => {
  if (view.tool === 'start') { snapshot(); route.start = { x_m: w[0], y_m: w[1], yaw_deg: route.start ? route.start.yaw_deg : 0 }; refresh(); return; }
  if (view.tool === 'line') { const along = alongTo(w); if (along === null) { log('set the start pose first', 'bad'); return; } addStraight(along); }
};
// Preview of the straight a click (or the typed length) would add: metres along the heading, or null.
view.onHover = w => { view._linePreview = (view.tool === 'line' && w) ? alongTo(w) : null; };
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
$('tool-line').onclick = () => setTool(view.tool === 'line' ? null : 'line');
$('btn-undo').onclick = undo; $('btn-redo').onclick = redo;
$('btn-clear').onclick = () => { snapshot(); route.steps = []; refresh(); };
$('ed-repeat').onchange = e => { snapshot(); route.repeat_count = +e.target.value; refresh(); };
$('ed-speed').onchange = e => { snapshot(); route.limits.linear_mps = +e.target.value; refresh(); };
// Long-straight boost: an empty field is "no boost" (null in the file), never a default 0.70
// smuggled into a route that was drawn without one.
$('ed-speed-long').onchange = e => { snapshot(); route.limits.long_linear_mps = e.target.value === '' ? null : +e.target.value; refresh(); };
$('ed-long-min').onchange = e => { snapshot(); route.limits.long_min_length_m = +e.target.value; refresh(); };
$('ed-speed-arc').onchange = e => { snapshot(); route.limits.arc_linear_mps = +e.target.value; refresh(); };
// The same rule as amr_navigation.compiler.step_speed: strictly LONGER than the threshold.
function stepSpeed(i) {
  const l = route.limits;
  return (l.long_linear_mps != null && stepLength(i) > (l.long_min_length_m ?? 4.0)) ? l.long_linear_mps : l.linear_mps;
}
const boosted = i => route.limits.long_linear_mps != null && stepSpeed(i) === route.limits.long_linear_mps;
// Steps an ERROR issue names are drawn red; `info` issues (a sweep crossing mapped cells in a
// dynamic area: passable only if the live scan agrees) never fail validation.
const badSteps = () => new Set((lastResult && lastResult.issues || []).filter(i => i.severity !== 'info').map(i => i.step_id));

function payload() {
  return { schema_version: 1, route_id: $('ed-route-id').value.trim(), revision: 0,
           map: { id: mapId, revision: mapRev, sha256: '' }, frame_id: 'map',
           start: route.start || { x_m: 0, y_m: 0, yaw_deg: 0 }, limits: route.limits, steps: route.steps, repeat_count: route.repeat_count };
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
  } else if (stillCurrent(c)) { lastResult = data; showResult(data); refresh(); }
};
$('btn-mission').onclick = async () => {
  if (!savedRef) { log('save this draft as a revision first', 'bad'); return; }
  if (savedRef.map_id !== mapId || +savedRef.map_revision !== +mapRev) { log('the saved revision belongs to another map; save on this map first', 'bad'); return; }
  const { status, data } = await api('/api/missions', { map_id: savedRef.map_id, map_revision: savedRef.map_revision, route_id: savedRef.route_id, route_revision: savedRef.revision });
  if (status === 200) log(`mission ${data.mission_id} created`);
};
function showResult(d) {
  if (!d) { $('ed-result').innerHTML = '<div class="none">not validated since the last edit</div>'; return; }
  const parts = [];
  const prov = (d.issues || []).filter(i => i.code === 'provisional').length;
  parts.push(`<div class="chips" style="margin-bottom:8px"><span class="chip ${d.ok ? 'ok' : 'bad'}">${d.ok ? 'valid' : 'invalid'}</span>` +
    (prov ? `<span class="chip warn" title="crosses mapped objects in dynamic areas: a scan return there stops the run">${prov} provisional</span>` : '') + '</div>');
  if (d.compiled) {
    const fast = d.compiled.steps.filter(s => s.type === 'straight' && s.v_mps != null && s.v_mps > route.limits.linear_mps);
    parts.push(`<div class="tel-grid">${[['Length', num(d.compiled.total_length_m, 2), 'm'], ['Turns', num(d.compiled.total_turn_deg, 0), '°'],
      ['Closes', d.compiled.closes ? 'YES' : 'NO', 'loop'], ['Boosted', String(fast.length), fast.length ? `at ${num(fast[0].v_mps, 2)} m/s` : 'straights']]
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
    (s.type === 'straight' ? `<span class="n">straight${boosted(i) ? ` <b title="long straight: runs at ${num(stepSpeed(i), 2)} m/s">▲${num(stepSpeed(i), 2)}</b>` : ''}</span><span class="v" title="to ${num(+s.to.x_m, 3)}, ${num(+s.to.y_m, 3)} m">${mm(stepLength(i))}<i>mm</i></span>`
     : s.type === 'reverse' ? `<span class="n">reverse <b title="backs up at half the speed cap: ${num(route.limits.linear_mps / 2, 2)} m/s">◂${num(route.limits.linear_mps / 2, 2)}</b></span><span class="v">${mm(+s.distance_m)}<i>mm</i></span>`
     : s.type === 'arc' ? `<span class="n">arc ${s.direction === 'ccw' ? 'L' : 'R'} <b title="capped so the yaw rate stays under the turn cap">≤${num(arcSpeed(+s.radius_m), 2)}</b></span><span class="v" title="${mm(stepLength(i))} mm of arc">${esc(s.angle_deg)}° R ${num(+s.radius_m, 2)}<i>m</i></span>`
                           : `<span class="n">rotate</span><span class="v">${esc(String(s.direction).toUpperCase())} ${esc(s.angle_deg)}<i>°</i></span>`) + '</div>').join('')
    : '<div class="none">no steps</div>';
  $('ed-repeat').value = route.repeat_count;
  // every control follows the model (Q15): a loaded route's speed cap shows as loaded, and an
  // option is added if the saved value is not one of the presets
  $('ed-speed').value = route.limits.linear_mps;  // a number input: shows the stored value, never rewrites it
  $('ed-speed-long').value = route.limits.long_linear_mps == null ? '' : route.limits.long_linear_mps;  // empty = no boost
  $('ed-long-min').value = route.limits.long_min_length_m ?? 4.0;
  $('ed-speed-arc').value = route.limits.arc_linear_mps ?? ARC_DEFAULT_MPS;  // shown, not written back unless changed
  view.draw();
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
      else v.dashed(sweptTurn(a, (s.direction === 'ccw' ? 1 : -1) * s.angle_deg * Math.PI / 180), checked(s.id, INK.turn));
      a = b;
    });
  }
  v.arrow(p.x, p.y, p.yaw, 1.0, INK.pose); if (footprint) v.footprint(p.x, p.y, p.yaw, footprint.polygon, alpha(INK.pose, 0.55));
  route.steps.forEach((s, i) => {
    const q = poseAfter(i + 1);
    if (s.type === 'straight') { v.line(p.x, p.y, q.x, q.y, bad.has(s.id) ? INK.stop : INK.route, 3); v.text((p.x + q.x) / 2, (p.y + q.y) / 2, `${s.id} ${mm(Math.hypot(q.x - p.x, q.y - p.y))} mm${boosted(i) ? ` ▲${num(stepSpeed(i), 2)}` : ''}`, INK.ink3); }
    else if (s.type === 'reverse') { v.dashed([[p.x, p.y], [q.x, q.y]], bad.has(s.id) ? INK.stop : INK.route); v.dot(q.x, q.y, INK.route, 4); v.text((p.x + q.x) / 2, (p.y + q.y) / 2, `${s.id} ◂ ${mm(Math.hypot(q.x - p.x, q.y - p.y))} mm`, INK.ink3); }
    else if (s.type === 'arc') { const pts = arcSamples(p, s, Math.max(8, Math.ceil(s.angle_deg / 3))); v.polyline(pts.map(t => [t.x, t.y]), bad.has(s.id) ? INK.stop : INK.route, 3); const m = pts[pts.length >> 1]; v.text(m.x, m.y, `${s.id} ⌒ R${num(+s.radius_m, 1)} ${s.angle_deg}°`, INK.ink3); v.arrow(q.x, q.y, q.yaw, 0.6, INK.turn); if (footprint) v.footprint(q.x, q.y, q.yaw, footprint.polygon, alpha(INK.turn, 0.4)); }
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
}
$('ed-load').onchange = async e => {
  if (!e.target.value) return;
  const [rid, rrev] = e.target.value.split('/');
  const c = ctx(), forMap = { map_id: mapId, map_revision: mapRev };
  const { data } = await apiGet(`/api/maps/${mapId}/${mapRev}/routes/${rid}/${rrev}`);
  if (!stillCurrent(c)) { log(`load of ${rid} rev${rrev} discarded: the draft or map changed meanwhile`, 'warn'); return; }
  snapshot(); route = { route_id: data.route.route_id, start: data.route.start, steps: data.route.steps, repeat_count: data.route.repeat_count, limits: data.route.limits };
  $('ed-route-id').value = data.route.route_id; savedRef = Object.assign({ route_id: rid, revision: +rrev }, forMap); lastResult = data; showResult(data); refresh();
  log(`loaded ${rid} rev${rrev}${data.ok ? '' : ' (INVALID on this map revision)'}`, data.ok ? '' : 'bad');
};
$('ed-map').onchange = async e => {
  [mapId, mapRev] = e.target.value.split('/'); mapRev = +mapRev;
  mapToken += 1; draftToken += 1;  // everything in flight for the previous map is void
  const c = ctx();
  route = { route_id: $('ed-route-id').value, start: null, steps: [], repeat_count: 1, limits: DEFAULT_LIMITS() }; history = []; future = []; lastResult = null; savedRef = null; refresh();
  await view.load(mapId, mapRev);
  if (!stillSameMap(c)) return;
  await reloadRouteList(c);
};
(async () => {
  footprint = (await apiGet('/api/footprint')).data;
  mapsIndex = await fillMapSelect($('ed-map'));
  if ($('ed-map').options.length) { $('ed-map').selectedIndex = 0; $('ed-map').onchange({ target: $('ed-map') }); }
})();
