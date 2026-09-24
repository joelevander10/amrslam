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
el('ed-speed', 'INPUT').value = '0.55';   // a NUMBER input, as in editor.html (not a <select>)
el('ed-speed-long', 'INPUT').value = '0.85'; el('ed-long-min', 'INPUT').value = '4';
el('ed-speed-arc', 'INPUT').value = '0.40';
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
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'amr_web', 'static', 'editor.js'), 'utf8'), ctx);
const flush = () => new Promise(r => setImmediate(r));

(async () => {
  const out = {};
  scripted['/api/footprint'] = () => ({ status: 200, data: { polygon: [[-0.5, -0.35], [1.1, -0.35], [1.1, 0.35], [-0.5, 0.35]], margin_m: 0.1, reach_m: 1.15 } });
  scripted['/api/maps/m1/1'] = () => ({ status: 200, data: { routes: { slow: [1] } } });
  for (let i = 0; i < 5; i++) await flush();
  out.new_draft_speed = els['ed-speed'].value;                       // refresh() ran on the new draft: 0.50, no TypeError
  out.new_draft_long = [els['ed-speed-long'].value, els['ed-long-min'].value];
  out.new_draft_arc = els['ed-speed-arc'].value;
  // load a saved route with 0.3: shown as 0.3, model untouched
  scripted['/api/maps/m1/1/routes/slow/1'] = () => ({ status: 200, data: { ok: true, route: { route_id: 'slow', start: { x_m: 0, y_m: 0, yaw_deg: 0 }, steps: [], repeat_count: 1, limits: { linear_mps: 0.3 } }, issues: [] } });
  els['ed-load'].value = 'slow/1';
  await els['ed-load'].onchange({ target: els['ed-load'] });
  for (let i = 0; i < 3; i++) await flush();
  out.loaded_speed_shown = els['ed-speed'].value;
  out.loaded_long_shown = els['ed-speed-long'].value;                // no long_linear_mps in the file: shown EMPTY, not 0.85
  out.loaded_arc_shown = els['ed-speed-arc'].value;                  // no arc_linear_mps in the file: the 0.40 default, not written
  // a save carries the loaded value, not the default
  scripted['/api/maps/m1/1/routes/save'] = body => { out.saved_limits = body.limits; return { status: 200, data: { route_id: 'slow', revision: 2, sha256: 'abcdef123456' } }; };
  await els['btn-save'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  // reset to a new draft (map change) goes back to 0.40
  els['ed-map'].value = 'm1/1';
  await els['ed-map'].onchange({ target: els['ed-map'] });
  for (let i = 0; i < 3; i++) await flush();
  out.reset_speed = els['ed-speed'].value;
  out.reset_long = els['ed-speed-long'].value;
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
  // --- long-straight boost: the row marks a straight LONGER than the threshold, the payload carries the limits ---
  els['ed-len'].value = '4000'; els['btn-line-len'].onclick();      // exactly 4.000 m: not boosted
  els['ed-len'].value = '4500'; els['btn-line-len'].onclick();      // 4.5 m: boosted
  out.boost_rows = els['ed-steps'].innerHTML;
  let savedLimits = null;
  scripted['/api/maps/m1/1/routes/save'] = body => { savedLimits = JSON.parse(JSON.stringify(body.limits)); return { status: 200, data: { route_id: 'r', revision: 2, sha256: 'abcdef123456' } }; };
  await els['btn-save'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  out.boost_saved_limits = savedLimits;
  els['ed-speed-long'].value = ''; els['ed-speed-long'].onchange({ target: els['ed-speed-long'] });   // boost off
  out.boost_off_rows = els['ed-steps'].innerHTML;
  await els['btn-save'].onclick();
  for (let i = 0; i < 3; i++) await flush();
  out.boost_off_limits = savedLimits;
  // --- reverse: bounded to 2000 mm, stored as a distance, drawn backwards along the heading ---
  const before = els['ed-steps'].innerHTML;
  els['ed-len'].value = '2500'; els['btn-reverse-len'].onclick();     // over the bound: refused
  out.reverse_refused = els['ed-steps'].innerHTML === before;
  els['ed-len'].value = '1200'; els['btn-reverse-len'].onclick();
  out.reverse_rows = els['ed-steps'].innerHTML;
  let savedSteps = null;
  scripted['/api/maps/m1/1/routes/save'] = body => { savedSteps = JSON.parse(JSON.stringify(body.steps)); return { status: 200, data: { route_id: 'r', revision: 3, sha256: 'abcdef123456' } }; };
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
  console.log(JSON.stringify(out));
})().catch(e => { console.log(JSON.stringify({ error: String(e && e.stack || e) })); });
