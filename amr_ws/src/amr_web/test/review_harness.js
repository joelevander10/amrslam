// Runs the real static/review.js against a fake DOM, a fake MapView and a scripted api.
// Driven by test_review_js.py; prints one JSON line of results.
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const els = {};
function el(id, tag) {
  const e = { id, tagName: tag || 'DIV', classList: new Set(), dataset: {}, listeners: {}, textContent: '', value: '', disabled: false,
    innerHTML: '', options: [], addEventListener(ev, fn) { (this.listeners[ev] = this.listeners[ev] || []).push(fn); } };
  e.classList.toggle = function (c, on) { if (on === undefined ? !this.has(c) : on) this.add(c); else this.delete(c); };
  els[id] = e; return e;
}
['rv-canvas', 'rv-map', 'rv-info', 'tool-dynamic', 'tool-undynamic', 'tool-unknown', 'tool-free', 'rv-tool-note', 'btn-rv-undo',
 'btn-rv-redo', 'btn-rv-clear', 'rv-ops', 'rv-note', 'btn-rv-save', 'rv-msg', 'rv-hint'].forEach(id => el(id));
const doc = { getElementById: id => els[id] || el(id) };

let theView = null;
const loads = [];
class MapView {
  constructor() { this.overlays = []; this.meta = null; this.tool = null; theView = this; }
  draw() { this.overlays.forEach(fn => fn(null, this)); }
  async load(m, r) { loads.push(`${m}/${r}`); this.meta = { map_id: m, revision: +r, resolution: 0.05, dynamic_cells: +r > 1 ? 400 : 0, edits: +r > 1 ? { parent_revision: +r - 1, ops: 1, note: '' } : null }; return true; }
  // overlay helpers the page calls: record, draw nothing
  dashed() {} line() {} fillPolygon() {} text() {}
}
const scripted = {};
const calls = [];
async function api(url, body) { calls.push({ url, body }); return scripted[url] ? scripted[url](body) : { status: 200, data: {} }; }
const logs = [];
let revs = ['m1/1'];
const ctx = { document: doc, window: { location: { search: '?map=m1&rev=1' }, confirm: () => ctx._confirm, addEventListener() {} }, _confirm: true,
  URLSearchParams, MapView, INK: new Proxy({}, { get: () => '#000000' }), alpha: () => '#00000055',
  esc: v => String(v ?? ''), num: (v, dp) => Number(v).toFixed(dp), log: (m, c) => logs.push([m, c || '']), api, apiGet: u => api(u),
  tiles: (id, rows) => { els[id].innerHTML = JSON.stringify(rows); },
  fillMapSelect: async sel => { sel.options = revs.map(v => ({ value: v })); return []; },
  console, setTimeout };
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'amr_web', 'static', 'review.js'), 'utf8'), ctx);
const flush = () => new Promise(r => setImmediate(r));

(async () => {
  const out = {};
  try {
    for (let i = 0; i < 5; i++) await flush();
    out.first_load = loads.slice();                                   // the ?map=&rev= query picks the revision
    out.save_disabled_empty = els['btn-rv-save'].disabled;
    // no tool: a drag does nothing
    theView.onDrag([0, 0], [1, 1], true);
    out.ops_without_tool = els['rv-ops'].innerHTML.includes('no edits');
    // dynamic tool: drag from bottom-right to top-left gives an axis-aligned rectangle
    els['tool-dynamic'].onclick();
    out.tool_on = [theView.tool, els['tool-dynamic'].classList.has('on'), els['rv-tool-note'].textContent.includes('BLOCKED')];
    theView.onDrag([2.0, 1.0], [1.0, 0.5], false);
    out.preview = theView._rect;
    theView.onDrag([2.0, 1.0], [1.0, 0.5], true);
    out.preview_after = theView._rect;
    theView.onDrag([0, 0], [0.02, 1.0], true);                        // thinner than a cell: ignored
    els['tool-unknown'].onclick();
    theView.onDrag([3.0, 0.0], [3.5, 0.4], true);
    out.rows = els['rv-ops'].innerHTML;
    out.save_enabled = !els['btn-rv-save'].disabled;
    // delete the first, undo it back
    els['rv-ops'].onclick({ target: { dataset: { del: '0' } } });
    out.after_delete = (els['rv-ops'].innerHTML.match(/class="row three"/g) || []).length;
    els['btn-rv-undo'].onclick();
    out.after_undo = (els['rv-ops'].innerHTML.match(/class="row three"/g) || []).length;
    // save: one POST carrying the ops in order, then the new revision is selected and edits clear
    scripted['/api/maps/m1/1/edit'] = body => { out.saved = JSON.parse(JSON.stringify(body)); revs = ['m1/1', 'm1/2']; return { status: 200, data: { ok: true, revision: 2, message: 'm1 rev2 saved from rev1 (2 edits)' } }; };
    els['rv-note'].value = ' trolleys ';
    await els['btn-rv-save'].onclick();
    for (let i = 0; i < 5; i++) await flush();
    out.after_save_loads = loads.slice();
    out.after_save_rows = els['rv-ops'].innerHTML;
    out.after_save_msg = els['rv-msg'].textContent;
    out.info = els['rv-info'].innerHTML;
    // a refused save keeps the edits
    theView.onDrag([0, 0], [1, 1], true);
    scripted['/api/maps/m1/2/edit'] = () => ({ status: 422, data: { ok: false, message: 'the edits change nothing' } });
    await els['btn-rv-save'].onclick();
    out.refused_msg = els['rv-msg'].textContent;
    out.refused_rows = (els['rv-ops'].innerHTML.match(/class="row three"/g) || []).length;
    // switching map with pending edits asks first; "no" keeps the selection and the edits
    ctx._confirm = false;
    els['rv-map'].value = 'm1/1';
    await els['rv-map'].onchange({ target: els['rv-map'] });
    out.cancel_switch = [els['rv-map'].value, (els['rv-ops'].innerHTML.match(/class="row three"/g) || []).length, loads.length];
  } catch (e) { out.error = String(e && e.stack || e); }
  console.log(JSON.stringify(out));
})();
