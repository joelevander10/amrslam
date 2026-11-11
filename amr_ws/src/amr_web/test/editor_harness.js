// Runs the real static/editor.js against a fake DOM, a fake MapView and a scripted api.
// Driven by test_editor_js.py; prints one JSON line of results.
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const els = {};
function el(id, tag) {
  const e = { id, tagName: tag || 'DIV', classList: new Set(), dataset: {}, listeners: {}, textContent: '', value: '', options: undefined,
    innerHTML: '', children: [], addEventListener(ev, fn) { (this.listeners[ev] = this.listeners[ev] || []).push(fn); },
    appendChild(c) { this.children.push(c); }, querySelector() { return null; } };
  e.classList.toggle = function (c, on) { if (on === undefined ? !this.has(c) : on) this.add(c); else this.delete(c); };
  els[id] = e; return e;
}
['ed-canvas', 'tool-start', 'tool-line', 'btn-start-survey', 'btn-undo', 'btn-redo', 'btn-clear', 'btn-validate', 'btn-save',
 'btn-mission', 'ed-steps', 'ed-result', 'ed-hint', 'ed-route-id', 'ed-load', 'ed-map'].forEach(id => el(id));
el('ed-repeat', 'INPUT').value = '1';
el('ed-speed').dataset.v = '0.60';   // read-only: route.VEHICLE_V_MAX rendered by the server (2026-10-02)
// the typed straight length: a number input read through valueAsNumber (NaN when empty), as in the browser
Object.defineProperty(el('ed-len', 'INPUT'), 'valueAsNumber', { get() { return this.value === '' ? NaN : Number(this.value); } });
el('btn-line-len', 'BUTTON'); el('btn-reverse-len', 'BUTTON'); el('btn-arc-l', 'BUTTON'); el('btn-arc-r', 'BUTTON');
for (const [id, v] of [['ed-arc-radius', '1.0'], ['ed-arc-angle', '90']]) {
  const e = el(id, 'INPUT'); e.value = v;
  Object.defineProperty(e, 'valueAsNumber', { get() { return this.value === '' ? NaN : Number(this.value); } });
}
['fl', 'f'].forEach(() => {});
['turn-ccw-45', 'turn-cw-45'].forEach(id => { const b = el(id, 'BUTTON'); b.dataset = { dir: id.includes('ccw') ? 'ccw' : 'cw', ang: '45' }; });
const doc = { getElementById: id => els[id] || el(id), querySelectorAll: () => [], createElement: tag => ({ tagName: tag.toUpperCase(), value: '', textContent: '' }), fonts: { ready: Promise.resolve() } };

let theView = null;
class MapView { constructor() { this.overlays = []; this.meta = null; this.tool = null; theView = this; } draw() {} async load(m, r) { this.meta = { map_id: m, revision: r, frame_id: 'map' }; } }
const scripted = {};
const calls = [];
async function api(url, body) { calls.push({ url, body }); return scripted[url] ? scripted[url](body) : { status: 200, data: {} }; }
const ctx = { document: doc, window: {}, MapView, INK: new Proxy({}, { get: () => '#000000' }), alpha: () => '#00000055',
  esc: v => String(v ?? ''), num: (v, dp) => Number(v).toFixed(dp), log: () => {}, api, apiGet: u => api(u), lastState: null,
  fillMapSelect: async sel => { sel.options = { length: 1 }; sel.selectedIndex = 0; sel.value = 'm1/1'; return []; },
  console, setTimeout, crypto: { randomUUID: () => 'x' } };
vm.createContext(ctx);
el('ed-snap', 'INPUT').checked = true;
// editor.html loads measure.js (the Measure tool's geometry) before editor.js
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'amr_web', 'static', 'measure.js'), 'utf8'), ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'amr_web', 'static', 'editor.js'), 'utf8'), ctx);
const flush = () => new Promise(r => setImmediate(r));

(async () => {
  const out = {};
  scripted['/api/footprint'] = () => ({ status: 200, data: { polygon: [[-0.5, -0.35], [1.1, -0.35], [1.1, 0.35], [-0.5, 0.35]], margin_m: 0.1, reach_m: 1.15 } });
  scripted['/api/maps/m1/1'] = () => ({ status: 200, data: { routes: { slow: [1] } } });
  for (let i = 0; i < 5; i++) await flush();
  out.new_draft_rows = els['ed-steps'].innerHTML;                    // refresh() ran on the new draft, no TypeError
  // load a route saved before 2026-10-02 with its own speeds: they are dropped, other limits kept
  scripted['/api/maps/m1/1/routes/slow/1'] = () => ({ status: 200, data: { ok: true, route: { route_id: 'slow', start: { x_m: 0, y_m: 0, yaw_deg: 0 }, steps: [], repeat_count: 1, limits: { linear_mps: 0.3, long_linear_mps: 0.85, long_min_length_m: 4, arc_linear_mps: 0.4, angular_rad_s: 0.3 } }, issues: [] } });
  els['ed-load'].value = 'slow/1';
  await els['ed-load'].onchange({ target: els['ed-load'] });
  for (let i = 0; i < 3; i++) await flush();
  // a save carries the non-speed limits only
  scripted['/api/maps/m1/1/routes/save'] = body => { out.saved_limits = body.limits; return { status: 200, data: { route_id: 'slow', revision: 2, sha256: 'abcdef123456' } }; };
  await els['btn-save'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  // reset to a new draft (map change) goes back to 0.40
  els['ed-map'].value = 'm1/1';
  await els['ed-map'].onchange({ target: els['ed-map'] });
  for (let i = 0; i < 3; i++) await flush();
  scripted['/api/maps/m1/1/routes/save'] = body => { out.reset_saved = body.limits; return { status: 200, data: { route_id: 'n', revision: 1, sha256: 'abcdef123456' } }; };
  await els['btn-save'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  // --- straights in mm: typed length and clicked point share one rule and one display ---
  scripted['/api/maps/m1/1/routes/save'] = body => { out.mm_saved_steps = JSON.parse(JSON.stringify(body.steps)); return { status: 200, data: { route_id: 'r', revision: 1, sha256: 'abcdef123456' } }; };
  theView.tool = 'start'; theView.onClick([0, 0]);              // start at the origin, heading +x
  out.len_no_number = (els['btn-line-len'].onclick(), els['ed-steps'].innerHTML.includes('no steps'));   // empty input adds nothing
  els['ed-len'].value = '1500'; els['ed-len'].oninput({ target: els['ed-len'] });
  out.len_preview_m = theView._linePreview;                      // typing previews 1.5 m along the heading
  els['btn-line-len'].onclick();
  out.len_step_row = els['ed-steps'].innerHTML;                  // "1500" + "mm", not the endpoint in m
  out.len_preview_after = theView._linePreview;                  // cleared by the edit
  els['ed-len'].value = '20'; els['btn-line-len'].onclick();     // below the 50 mm minimum: refused
  theView.tool = 'line';
  theView.onHover([2.0, 0.7]); out.hover_preview_m = theView._linePreview;   // projection onto the heading: 0.5 m more
  theView.onClick([2.0, 0.7]);                                   // the click lands on the projection, not the point
  theView.onHover(null); out.hover_cleared = theView._linePreview;
  await els['btn-save'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  out.mm_rows = els['ed-steps'].innerHTML;
  // --- one trackless speed: long straights are not marked or sped up ---
  els['ed-len'].value = '4000'; els['btn-line-len'].onclick();
  els['ed-len'].value = '4500'; els['btn-line-len'].onclick();
  out.long_rows = els['ed-steps'].innerHTML;
  // --- reverse: bounded to 2000 mm, stored as a distance, drawn backwards along the heading ---
  const before = els['ed-steps'].innerHTML;
  els['ed-len'].value = '2500'; els['btn-reverse-len'].onclick();     // over the bound: refused
  out.reverse_refused = els['ed-steps'].innerHTML === before;
  els['ed-len'].value = '1200'; els['btn-reverse-len'].onclick();
  out.reverse_rows = els['ed-steps'].innerHTML;
  let savedSteps = null, lastSavedStart = null, lastSavedBranch = null;
  scripted['/api/maps/m1/1/routes/save'] = body => { savedSteps = JSON.parse(JSON.stringify(body.steps)); lastSavedStart = body.start ? JSON.parse(JSON.stringify(body.start)) : null; lastSavedBranch = body.branch ? JSON.parse(JSON.stringify(body.branch)) : null; return { status: 200, data: { route_id: 'r', revision: 3, sha256: 'abcdef123456' } }; };
  await els['btn-save'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  out.reverse_saved_step = savedSteps[savedSteps.length - 1];
  els['ed-len'].value = '1000'; els['btn-line-len'].onclick();       // a straight after the reverse starts from the backed-up pose
  await els['btn-save'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  out.after_reverse_to = savedSteps[savedSteps.length - 1].to;
  // --- arc: bounds, saved shape, and the end pose it leaves for the next straight ---
  const rowsBefore = els['ed-steps'].innerHTML;
  els['ed-arc-radius'].value = '0.5'; els['btn-arc-l'].onclick();         // under 2 x track: refused
  els['ed-arc-radius'].value = '1.0'; els['ed-arc-angle'].value = '30'; els['btn-arc-l'].onclick();   // under 45 deg: refused
  out.arc_refused = els['ed-steps'].innerHTML === rowsBefore;
  els['ed-arc-angle'].value = '90'; els['btn-arc-l'].onclick();          // left 90 deg, R 1 from (10.3, 0) heading +x
  out.arc_rows = els['ed-steps'].innerHTML;
  els['ed-len'].value = '1000'; els['btn-line-len'].onclick();           // then 1 m along the new heading (+y)
  await els['btn-save'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  out.arc_saved_step = savedSteps[savedSteps.length - 2];
  out.after_arc_to = savedSteps[savedSteps.length - 1].to;
  // --- Measure (2026-11-03): view only, snaps to the draft and to saved routes ---
  scripted['/api/maps/m1/1/routes/validate'] = () => ({ status: 200, data: { ok: true, issues: [], compiled: { total_length_m: 1, total_turn_deg: 0, closes: false, steps: [] } } });
  await els['btn-validate'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  const resultBefore = els['ed-result'].innerHTML;
  out.geometry_calls = 0;
  scripted['/api/maps/m1/1/routes/geometry'] = () => { out.geometry_calls += 1; return { status: 200, data: { routes: [
    { route_id: 'other', revision: 2, map_revision: 1, start: [0, 5, 0], steps: [
      { id: 's1', type: 'straight', start: [0, 5, 0], end: [3, 5, 0], centre: null, radius_m: 0, signed_angle_deg: 0, name: '' },
      { id: 's2', type: 'wait', start: [3, 5, 0], end: [3, 5, 0], centre: null, radius_m: 0, signed_angle_deg: 0, name: 'UBC' }] },
    { route_id: 'broken', revision: 1, error: 'bad file' }] } }; };
  Object.assign(theView.meta, { resolution: 0.05 }); theView.scale = 3.5;   // 14 px of reach = 0.20 m
  els['tool-measure'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  out.measure_on = els['tool-measure'].classList.has('on');
  out.pan_tools = [...theView.panTools];
  out.measure_idle = els['ed-measure'].innerHTML;
  theView.onClick([10.35, 1.05]);       // A: the s7 arc's centre (10.3, 1)
  theView.onHover([3.05, 5.02]);        // B follows the cursor: the saved route's wait (3, 5)
  out.measure_live = els['ed-measure'].innerHTML;
  theView.onClick([3.05, 5.02]);
  out.measure_rows = els['ed-measure'].innerHTML;
  out.result_untouched = els['ed-result'].innerHTML === resultBefore;  // a VALID result survives measuring
  els['ed-snap'].checked = false;       // snapping off: exactly where clicked; a third click starts over
  theView.onClick([1.0, 0.3]); theView.onClick([1.5, 0.3]);
  out.free_rows = els['ed-measure'].innerHTML;
  els['ed-snap'].checked = true;
  theView.onClick([5.0, 0.1]);          // nothing in reach but the s3 line: onto it
  out.path_rows = els['ed-measure'].innerHTML;
  els['tool-measure'].onclick();        // tool off clears
  out.measure_cleared = els['ed-measure'].innerHTML;
  els['btn-undo'].onclick();            // measuring added no undo entries: undo still removes s8
  await els['btn-save'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  out.after_undo_ids = savedSteps.map(s => s.id);
  // --- speed waypoints (2026-11-06): split a straight, give a part its own speed ---
  els['btn-clear'].onclick();
  els['ed-len'].value = '6000'; els['btn-line-len'].onclick();          // s1: 6 m at the route speed
  ['ed-split-step', 'ed-split-mm', 'ed-straight-v'].forEach(id => doc.getElementById(id));
  els['ed-split-step'].value = 's1';
  Object.defineProperty(els['ed-split-mm'], 'valueAsNumber', { get() { return this.value === '' ? NaN : Number(this.value); }, configurable: true });
  els['ed-split-mm'].value = '5990'; els['btn-split'].onclick();         // too close to the end: refused
  out.split_refused = !els['ed-steps'].innerHTML.includes('s2');
  els['ed-split-mm'].value = '2000'; els['btn-split'].onclick();         // s1 2 m + s2 4 m
  els['ed-split-step'].value = 's2'; els['ed-split-mm'].value = '1500'; els['btn-split'].onclick();  // s2 1.5 m + s3 2.5 m
  const change = (i, v) => { const t = { classList: new Set(['sv']), dataset: { i: String(i) }, value: v }; t.classList.contains = t.classList.has; els['ed-steps'].listeners.change.forEach(fn => fn({ target: t })); return t; };
  change(1, '0.3');                                                       // s2 = the slow zone
  out.bad_speed_kept = change(2, '5').value;                              // above the route speed: refused, box reset
  els['ed-straight-v'].value = '0.25'; els['ed-len'].value = '1000'; els['btn-line-len'].onclick();  // s4 added slow
  els['ed-straight-v'].value = '';
  out.split_options = els['ed-split-step'].innerHTML;
  await els['btn-save'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  out.split_saved = savedSteps.map(s => ({ id: s.id, x: Math.round(s.to.x_m * 1000), v: s.v_mps ?? null }));
  // --- Start = AMR here (2026-11-06): only localised, READY, on the viewed map ---
  els['btn-clear'].onclick();
  ctx.lastState = { mode: { active_map_id: 'm1', active_map_revision: 1 }, localization: { state_name: 'CHECKING' },
                    run: { pose_valid: true, pose_x: 3.25, pose_y: -1.5, pose_yaw: Math.PI / 2 } };
  const startBefore = JSON.stringify(theView.meta);
  els['btn-start-here'].onclick();
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.here_refused_start = lastSavedStart;
  ctx.lastState.localization.state_name = 'READY';
  els['btn-start-here'].onclick();
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.here_start = lastSavedStart;
  // --- Branch (POS routes, 2026-11-06): main s1 s2 s3(home); POS leaves after s1, stops, backs up, rejoins ---
  scripted['/api/maps/m1/1/routes/main/1'] = () => ({ status: 200, data: { route: { route_id: 'main', start: { x_m: 0, y_m: 0, yaw_deg: 0 }, repeat_count: 1, limits: {},
    steps: [{ id: 's1', type: 'straight', to: { x_m: 4, y_m: 0 } }, { id: 's2', type: 'straight', to: { x_m: 8, y_m: 0 } }, { id: 's3', type: 'home' }] } } });
  ['ed-branch-main', 'ed-branch-leave', 'ed-branch-rejoin', 'ed-branch-note'].forEach(id => doc.getElementById(id));
  els['ed-branch-main'].value = 'main/1'; await els['ed-branch-main'].onchange({ target: els['ed-branch-main'] });
  els['ed-branch-leave'].value = 's1'; els['btn-branch-start'].onclick();
  els['ed-len'].value = '1000'; els['btn-line-len'].onclick();          // b1: 1 m on to the POS
  els['btn-wait'].onclick();                                            // b2: the stop point
  els['btn-reverse-len'].onclick();                                     // b3: back 1 m to the end of s1
  els['ed-branch-rejoin'].value = 's2'; els['btn-branch-rejoin'].onclick();   // wrong main step: refused
  out.branch_open_rows = els['ed-steps'].innerHTML.includes('s2');
  els['ed-branch-rejoin'].value = 's1'; els['btn-branch-rejoin'].onclick();
  els['ed-len'].value = '500'; els['btn-line-len'].onclick();           // closed: no more steps
  els['ed-route-id'].value = 'pos1';
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.branch_saved = savedSteps.map(s => s.id);
  out.branch_meta = lastSavedBranch;
  // --- Stop point on map: one click makes the whole POS route (junction inside s1) ---
  els['btn-undo'].onclick(); els['btn-undo'].onclick();   // whatever the draft was: replaced anyway
  els['ed-branch-main'].value = 'main/1'; await els['ed-branch-main'].onchange({ target: els['ed-branch-main'] });
  els['tool-branchpoint'].onclick();
  out.auto_tool = theView.tool;
  theView.onClick([2.0, 0.1]);                                      // 100 mm off the route: refused
  out.auto_too_close = theView.tool === 'branchpoint';
  els['ed-branch-return'] = els['ed-branch-return'] || doc.getElementById('ed-branch-return');
  els['ed-branch-return'].value = 'turn';
  els['ed-wait-name'] = els['ed-wait-name'] || doc.getElementById('ed-wait-name'); els['ed-wait-name'].value = 'POS 2';
  els['ed-route-id'].value = 'pos2';
  theView.onClick([2.0, 1.5]);                                      // 1.5 m left of s1, 2 m along it
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.auto_steps = savedSteps;
  out.auto_branch = lastSavedBranch;
  // reverse back, right side, at the end of s1 (snaps to the junction s1 / s2)
  els['tool-branchpoint'].onclick(); els['ed-branch-return'].value = 'reverse';
  theView.onClick([4.02, -1.2]);
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.auto_rev_steps = savedSteps;
  out.auto_rev_branch = lastSavedBranch;
  // --- Slow zone on map (2026-11-06): on a route already drawn, A and B clicked ---
  vm.runInContext(`route = { route_id: 'loop', start: { x_m: 0, y_m: 0, yaw_deg: 0 }, repeat_count: 1, limits: {}, branch: null,
    steps: [{ id: 's1', type: 'straight', to: { x_m: 10, y_m: 0 } }, { id: 's2', type: 'rotate', direction: 'ccw', angle_deg: 90 },
            { id: 's3', type: 'straight', to: { x_m: 10, y_m: 5 } }] }; refresh();`, ctx);
  doc.getElementById('ed-slow-v').value = '0.3';
  els['tool-slowzone'].onclick();
  theView.onClick([8.0, 0.2]);                                    // A: 8 m along s1
  theView.onClick([9.9, 2.0]);                                    // B: 2 m along s3, past the turn
  els['ed-route-id'].value = 'loop';
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.slow_steps = savedSteps.map(s => [s.id, s.type, s.to ? [Math.round(s.to.x_m * 1000), Math.round(s.to.y_m * 1000)] : null, s.v_mps ?? null]);
  els['btn-undo'].onclick();                                      // one undo removes the whole zone
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.slow_undone = savedSteps.map(s => s.id);
  // --- the slow zone list (2026-11-09): change its speed, remove it, add one again ---
  els['btn-redo'].onclick();                                      // the zone is back
  out.zone_list = els['ed-slow-list'].innerHTML;
  const fire = (ev, cls, data, value) => { const t = { classList: new Set([cls]), dataset: data, value, closest: () => (ev === 'click' ? t : null) };
    t.classList.contains = t.classList.has; els['ed-slow-list'].listeners[ev].forEach(fn => fn({ target: t })); };
  fire('change', 'zv', { k: '0' }, '0.25');
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.zone_changed = savedSteps.map(s => s.v_mps ?? null);
  fire('click', 'x', { zoneRemove: '0' });
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.zone_removed = savedSteps.map(s => [s.id, s.v_mps ?? null]);
  out.zone_list_after = els['ed-slow-list'].innerHTML;
  doc.getElementById('ed-slow-v').value = '0.4';
  out.tool_still_on = theView.tool;                               // stays on for the next zone
  theView.onClick([1.0, 0.0]); theView.onClick([5.0, 0.0]);       // a new zone on s1
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.zone_again = savedSteps.map(s => [s.id, s.v_mps ?? null]);
  // --- delete steps (2026-11-11): the straights after keep their length; to the end; undo ---
  vm.runInContext(`route = { route_id: 'del', start: { x_m: 0, y_m: 0, yaw_deg: 0 }, repeat_count: 1, limits: {}, steps: [
            { id: 's1', type: 'straight', to: { x_m: 5, y_m: 0 } },
            { id: 's2', type: 'rotate', direction: 'ccw', angle_deg: 90 },
            { id: 's3', type: 'straight', to: { x_m: 5, y_m: 3 } },
            { id: 's4', type: 'rotate', direction: 'ccw', angle_deg: 90 },
            { id: 's5', type: 'straight', to: { x_m: 1, y_m: 3 } }] }; refresh();`, ctx);
  const del = (key, i) => els['ed-steps'].listeners.click.forEach(fn => fn({ target: { dataset: { [key]: String(i) } } }));
  out.del_buttons = (els['ed-steps'].innerHTML.match(/data-del="/g) || []).length;
  del('del', 0);                                                  // drop the first 5 m straight
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.del_first = savedSteps.map(s => [s.id, s.to ? [Math.round(s.to.x_m * 1000), Math.round(s.to.y_m * 1000)] : s.angle_deg]);
  del('delEnd', 1);                                                // s3 and everything after it
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.del_end = savedSteps.map(s => s.id);
  els['btn-undo'].onclick(); els['btn-undo'].onclick();
  await els['btn-save'].onclick(); for (let i = 0; i < 3; i++) await flush();
  out.del_undone = savedSteps.map(s => s.id);
  console.log(JSON.stringify(out));
})().catch(e => { console.log(JSON.stringify({ error: String(e && e.stack || e) })); });
