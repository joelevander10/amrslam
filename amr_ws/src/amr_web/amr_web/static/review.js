// Map review (dynamic-mapping plan §1.2): mark dynamic areas and clean up a saved map by
// dragging rectangles. Edits are map-metre polygons held here until saved; saving asks the
// robot to derive a NEW revision (POST /api/maps/<id>/<rev>/edit). The revision on screen is
// never modified and nothing on this page moves the vehicle.
const view = new MapView(document.getElementById('rv-canvas'));
const $ = id => document.getElementById(id);
let mapId = null, mapRev = null, ops = [], history = [], future = [], saving = false, mapToken = 0;

// op per tool, as amr_maps.edit expects it; colours per kind on the canvas and in the list
const TOOLS = {
  dynamic: { btn: $('tool-dynamic'), name: 'dynamic area', note: 'The scan here may change: trolleys, parked forklifts, pallets. Routes may cross the mapped objects in it; any scan return there during a run stops the vehicle (BLOCKED) until you clear it and press Start.' },
  undynamic: { btn: $('tool-undynamic'), name: 'clear dynamic', note: 'Removes the dynamic mark from these cells: mapped objects there block routes again.' },
  unknown: { btn: $('tool-unknown'), name: 'erase to unknown', note: 'Safe erase: the cells become unknown. Unknown blocks routes unless it lies inside a dynamic area.' },
  free: { btn: $('tool-free'), name: 'paint free', note: 'Operator assertion: you state this is open floor (the object is gone for good). Routes may pass; a scan return there still stops a run, but localisation treats the area as open floor.' },
};
const kind = o => o.op === 'paint' ? o.value : o.op;
const opFor = (tool, polygon) => (tool === 'dynamic' || tool === 'undynamic') ? { op: tool, polygon } : { op: 'paint', value: tool, polygon };
function style(k) {
  if (k === 'dynamic') return { fill: alpha(INK.preview, 0.35), stroke: INK.turn };
  if (k === 'undynamic') return { fill: null, stroke: INK.turn, dashed: true };
  if (k === 'unknown') return { fill: alpha(INK.ink3, 0.45), stroke: INK.ink3 };
  return { fill: alpha(INK.pose, 0.25), stroke: INK.pose };
}
// Same polygon as amr_maps.edit.rectangle: axis-aligned in the map frame, corners ccw.
function rect(a, b) {
  const x0 = Math.min(a[0], b[0]), x1 = Math.max(a[0], b[0]), y0 = Math.min(a[1], b[1]), y1 = Math.max(a[1], b[1]);
  return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
}
const size = p => [p[1][0] - p[0][0], p[2][1] - p[1][1]];
const mm = m => num(Math.round(m * 1000), 0);

function snapshot() { history.push(JSON.stringify(ops)); if (history.length > 100) history.shift(); future = []; }
function undo() { if (!history.length) return; future.push(JSON.stringify(ops)); ops = JSON.parse(history.pop()); refresh(); }
function redo() { if (!future.length) return; history.push(JSON.stringify(ops)); ops = JSON.parse(future.pop()); refresh(); }

function setTool(t) {
  view.tool = t; view._rect = null;
  Object.keys(TOOLS).forEach(k => TOOLS[k].btn.classList.toggle('on', k === t));
  $('rv-tool-note').textContent = t ? TOOLS[t].note : '';
  view.draw();
}
Object.keys(TOOLS).forEach(k => { TOOLS[k].btn.onclick = () => setTool(view.tool === k ? null : k); });

view.onDrag = (a, b, done) => {
  if (!TOOLS[view.tool] || !view.meta) return;
  const poly = rect(a, b);
  if (!done) { view._rect = poly; return; }
  view._rect = null;
  const res = view.meta.resolution, [w, h] = size(poly);
  if (w < res || h < res) { log('rectangle smaller than one map cell: ignored', 'warn'); return; }
  snapshot(); ops.push(opFor(view.tool, poly)); refresh();
  log(`edit ${ops.length}: ${TOOLS[view.tool].name} ${mm(w)} × ${mm(h)} mm`);
};
view.onHover = () => { view._rect = null; };  // only called while not dragging

view.overlays.push((c, v) => {
  ops.forEach((o, i) => {
    const st = style(kind(o));
    if (st.dashed) { v.dashed(o.polygon, st.stroke); v.line(o.polygon[0][0], o.polygon[0][1], o.polygon[2][0], o.polygon[2][1], st.stroke, 1); }
    else v.fillPolygon(o.polygon, st.fill, st.stroke);
    v.text(o.polygon[3][0], o.polygon[3][1], String(i + 1), st.stroke);
  });
  if (v._rect && v.tool) { const st = style(v.tool); v.fillPolygon(v._rect, st.fill, null); v.dashed(v._rect, st.stroke); }
});

function refresh() {
  $('rv-ops').innerHTML = ops.length ? ops.map((o, i) => {
    const [w, h] = size(o.polygon), k = kind(o);
    return `<div class="row three"><span class="k">${i + 1}</span><span class="n">${esc(TOOLS[k] ? TOOLS[k].name : k)}</span>` +
      `<span class="v">${mm(w)} × ${mm(h)}<i>mm</i> <button class="tool" data-del="${i}" title="remove this edit">×</button></span></div>`;
  }).join('') : '<div class="none">no edits</div>';
  $('btn-rv-save').disabled = saving || !ops.length;
  view.draw();
}
$('rv-ops').onclick = e => {
  const i = e.target && e.target.dataset ? e.target.dataset.del : undefined;
  if (i === undefined) return;
  snapshot(); ops.splice(+i, 1); refresh();
};
$('btn-rv-undo').onclick = undo; $('btn-rv-redo').onclick = redo;
$('btn-rv-clear').onclick = () => { if (!ops.length) return; snapshot(); ops = []; refresh(); };

function showInfo() {
  const m = view.meta;
  if (!m) { $('rv-info').innerHTML = ''; return; }
  const area = (m.dynamic_cells || 0) * m.resolution * m.resolution, e = m.edits;
  tiles('rv-info', [
    ['Dynamic areas', m.dynamic_cells ? `${num(area, 1)} m²` : 'none', m.dynamic_cells ? `${m.dynamic_cells} cells` : 'in this revision'],
    ['Derived', e ? `from rev${e.parent_revision}` : 'survey', e ? `${e.ops} edit(s)${e.note ? ' · ' + e.note : ''}` : 'as saved'],
  ]);
}

$('btn-rv-save').onclick = async () => {
  if (saving) return;
  if (!ops.length) { log('nothing to save: draw an edit first', 'bad'); return; }
  const forMap = mapId, forRev = mapRev, c = mapToken, sent = ops.slice();
  saving = true; refresh();
  try {
    const { status, data } = await api(`/api/maps/${forMap}/${forRev}/edit`, { ops: sent, note: $('rv-note').value.trim() });
    if (status !== 200) { $('rv-msg').textContent = `not saved: ${data.message || status}`; log(`map edit refused: ${data.message || status}`, 'bad'); return; }
    log(data.message);
    const done = `${data.message}. Select ${forMap} rev${data.revision} on the Run page to drive on it; re-save routes on it in Routes.`;
    if (c !== mapToken) return;  // another map was selected meanwhile: leave that selection alone
    ops = []; history = []; future = []; $('rv-note').value = '';
    await fillMapSelect($('rv-map'));
    $('rv-map').value = shown = `${forMap}/${data.revision}`;
    await selectMap(shown);
    $('rv-msg').textContent = done;  // after selectMap, which clears the message
  } finally { saving = false; refresh(); }
};

async function selectMap(value) {
  [mapId, mapRev] = value.split('/'); mapRev = +mapRev;
  mapToken += 1; const c = mapToken;
  ops = []; history = []; future = []; view._rect = null; $('rv-msg').textContent = ''; refresh();
  const loaded = await view.load(mapId, mapRev);
  if (!loaded || c !== mapToken) return;
  showInfo();
}
let shown = null;
$('rv-map').onchange = async e => {
  if (ops.length && !window.confirm(`Discard ${ops.length} unsaved edit(s) on ${mapId} rev${mapRev}?`)) { e.target.value = shown; return; }
  shown = e.target.value;
  await selectMap(e.target.value);
};
window.addEventListener('beforeunload', e => { if (ops.length) { e.preventDefault(); e.returnValue = ''; } });

(async () => {
  await fillMapSelect($('rv-map'));
  const q = new URLSearchParams(window.location.search), want = q.get('map') && q.get('rev') ? `${q.get('map')}/${q.get('rev')}` : null;
  const opts = Array.from($('rv-map').options || []).map(o => o.value);
  const first = want && opts.includes(want) ? want : opts[opts.length - 1];
  if (!first) { $('rv-msg').textContent = 'no saved maps yet: survey one on the Maps page first'; return; }
  $('rv-map').value = shown = first;
  await selectMap(first);
})();
