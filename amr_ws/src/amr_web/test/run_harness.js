// Runs the real static/run.js against a fake DOM: the route preview must be drawn after the
// page load race (the first state poll arrives after the page's own mission load) and must
// follow the mission the executor holds. Driven by test_run_js.py; prints one JSON line.
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const els = {};
function el(id) {
  if (!els[id]) {
    const e = { id, value: '', innerHTML: '', textContent: '', disabled: false, options: [], classList: new Set(), dataset: {},
      appendChild(o) { this.options.push(o); if (this.value === '') this.value = o.value; },
      addEventListener() {} };
    e.classList.remove = e.classList.delete;
    Object.defineProperty(e, 'innerHTML', { get() { return ''; }, set() { this.options = []; this.value = ''; } });
    els[id] = e;
  }
  return els[id];
}
const stateFns = [];
const gets = [];
const missions = [
  { mission_id: 'mis1', map: { id: 'm1', revision: 1 }, route: { id: 'r1', revision: 1 } },
  { mission_id: 'mis2', map: { id: 'm1', revision: 1 }, route: { id: 'r2', revision: 3 } },
];
async function apiGet(url) {
  gets.push(url);
  await null;
  if (url === '/api/footprint') return { status: 200, data: { polygon: [] } };
  if (url === '/api/missions') return { status: 200, data: missions };
  const m = url.match(/routes\/(\w+)\/(\d+)$/);
  if (m) return { status: 200, data: { compiled: { route: m[1], steps: [] } } };
  return { status: 404, data: {} };
}
class MapView { constructor() { this.overlays = []; this.meta = null; } draw() {} async load() {} }
const ctx = {
  console, JSON, Math, Promise,
  document: { getElementById: el, querySelectorAll: () => [], createElement: () => ({ value: '', textContent: '' }) },
  MapView, apiGet, api: async () => ({ status: 200, data: {} }),
  onState: fn => stateFns.push(fn), tiles() {}, num: v => String(v), yesno: v => String(v), log() {},
  operation() {}, esc: s => s, alpha: c => c, INK: {}, LOC_LEVEL: {}, RUN_LEVEL: {}, MODE_LEVEL: {},
  lastState: null, liveOverlay() {}, pollLive() {}, jogpad() {},
  fillMapSelect: async sel => { sel.appendChild({ value: 'm1/1', textContent: 'm1 rev1' }); },
};
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'amr_web', 'static', 'run.js'), 'utf8'), ctx);
const flush = async () => { for (let i = 0; i < 20; i++) await new Promise(r => setImmediate(r)); };
const preview = () => vm.runInContext('preview && preview.route', ctx);
const state = run => ({ mode: { mode_name: 'NAVIGATION', active_map_id: 'm1', active_map_revision: 1, generation: 3 },
                        run: run ? { mission_id: run, state_name: 'READY' } : null });

(async () => {
  const out = {};
  await flush();                                    // the page's own load: no active map known yet
  out.before_state = preview();
  stateFns.forEach(f => f(state('mis2'))); await flush();   // first poll: map active, executor holds mis2
  out.after_first_state = preview();
  out.selected = el('run-mission').value;
  stateFns.forEach(f => f(state('mis2'))); await flush();   // steady polls: nothing reloaded, still drawn
  out.steady = preview();
  const n = gets.length;
  stateFns.forEach(f => f(state('mis2'))); await flush();
  out.no_refetch = gets.length === n;
  stateFns.forEach(f => f(state('mis1'))); await flush();   // another mission loaded (e.g. from another tablet)
  out.followed = preview();
  console.log(JSON.stringify(out));
})().catch(e => console.log(JSON.stringify({ error: String(e && e.stack || e) })));
