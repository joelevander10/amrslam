// Map review (dynamic-mapping plan §1.2): mark dynamic areas and clean up a saved map by
// dragging rectangles. Edits are map-metre polygons held here until saved; saving asks the
// robot to derive a NEW revision (POST /api/maps/<id>/<rev>/edit). The revision on screen is
// never modified and nothing on this page moves the vehicle.
const view = new MapView(document.getElementById('rv-canvas'));
const $ = id => document.getElementById(id);
let mapId = null, mapRev = null, ops = [], history = [], future = [], saving = false, mapToken = 0;
let refLine = null;  // [[x, y], [x, y]] the wall drawn with Reference line, map metres
let changes = null;  // GET .../changes: where the map no longer fits (bottom of this file)
let remapBoxes = [], remapAfter = null, remapCheck = null;  // Re-map area (bottom of this file)

// op per tool, as amr_maps.edit expects it; colours per kind on the canvas and in the list
const TOOLS = {
  despeckle: { btn: $('tool-despeckle'), name: 'remove noise', note: 'Removes lidar noise: inside the rectangle, every occupied dot or blob of up to 4 cells (about 10 × 10 cm) becomes free floor. Walls, racks and anything bigger stay. Drag it over a whole room if you like.' },
  occupied: { btn: $('tool-occupied'), name: 'static obstacle', note: 'A permanent obstacle (wall, fixed rack): the cells become occupied. Routes are refused through it and localisation matches against it. For things that move, use Dynamic area.' },
  dynamic: { btn: $('tool-dynamic'), name: 'dynamic area', note: 'The scan here may change: trolleys, parked forklifts, pallets. Routes may cross the mapped objects in it; any scan return there during a run stops the vehicle (BLOCKED) until you clear it and press Start.' },
  undynamic: { btn: $('tool-undynamic'), name: 'clear dynamic', note: 'Removes the dynamic mark from these cells: mapped objects there block routes again.' },
  unknown: { btn: $('tool-unknown'), name: 'erase to unknown', note: 'Safe erase: the cells become unknown. Unknown blocks routes unless it lies inside a dynamic area.' },
  free: { btn: $('tool-free'), name: 'paint free', note: 'Operator assertion: you state this is open floor (the object is gone for good). Routes may pass; a scan return there still stops a run, but localisation treats the area as open floor.' },
};
const kind = o => o.op === 'paint' ? o.value : o.op;
const opFor = (tool, polygon) => (tool === 'dynamic' || tool === 'undynamic' || tool === 'despeckle') ? { op: tool, polygon } : { op: 'paint', value: tool, polygon };
function style(k) {
  if (k === 'dynamic') return { fill: alpha(INK.preview, 0.35), stroke: INK.turn };
  if (k === 'undynamic') return { fill: null, stroke: INK.turn, dashed: true };
  if (k === 'unknown') return { fill: alpha(INK.ink3, 0.45), stroke: INK.ink3 };
  if (k === 'occupied') return { fill: alpha(INK.stop, 0.35), stroke: INK.stop };
  if (k === 'despeckle') return { fill: null, stroke: INK.pose, dashed: true };
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
  $('tool-align').classList.toggle('on', t === 'align');
  $('tool-remap').classList.toggle('on', t === 'remap');
  $('rv-tool-note').textContent = t && TOOLS[t] ? TOOLS[t].note : t === 'remap' ? remapToolNote() : '';
  view.draw();
}
Object.keys(TOOLS).forEach(k => { TOOLS[k].btn.onclick = () => setTool(view.tool === k ? null : k); });
$('tool-align').onclick = () => setTool(view.tool === 'align' ? null : 'align');

// Rotate map (amr_maps.rotate.align_angle mirrored): the turn that brings the reference line
// onto the NEAREST axis direction, so at most 45 deg either way. + = counter-clockwise.
const wrapA = a => Math.atan2(Math.sin(a), Math.cos(a));
function alignAngle(p, q) {
  const a = Math.atan2(q[1] - p[1], q[0] - p[0]);
  return wrapA(Math.round(a / (Math.PI / 2)) * (Math.PI / 2) - a);
}
const rotDeg = () => { const v = parseFloat($('rv-rot-deg').value); return Number.isFinite(v) ? v : 0; };
function rotNote() {
  const deg = rotDeg();
  if (refLine) {
    const a = Math.atan2(refLine[1][1] - refLine[0][1], refLine[1][0] - refLine[0][0]) * 180 / Math.PI;
    const len = Math.hypot(refLine[1][0] - refLine[0][0], refLine[1][1] - refLine[0][1]);
    const axis = Math.abs(Math.cos((a + deg) * Math.PI / 180)) > 0.7 ? 'X' : 'Y';
    $('rv-rot-note').textContent = `Reference line ${num(len, 2)} m at ${num(a, 2)}°: turning ${deg > 0 ? '+' : ''}${num(deg, 2)}° lays it along ${axis}. A longer line is a more exact angle.`;
  } else if (deg) {
    $('rv-rot-note').textContent = `Turn the map ${deg > 0 ? '+' : ''}${num(deg, 2)}° (${deg > 0 ? 'counter-clockwise' : 'clockwise'}) about its origin.`;
  }
}
$('rv-rot-deg').oninput = () => { rotNote(); refresh(); };

view.onDrag = (a, b, done) => {
  if (view.tool === 'remap' && view.meta) {
    const poly = rect(a, b);
    if (!done) { view._rect = poly; return; }
    view._rect = null;
    const [w, h] = size(poly);
    if (w < 0.3 || h < 0.3) { log('re-map box smaller than 0.3 m: ignored', 'warn'); return; }
    remapBoxes.push(poly); remapStale(); remapAdded(poly); return;
  }
  if (view.tool === 'align' && view.meta) {
    refLine = [a, b];
    if (done) {
      if (Math.hypot(b[0] - a[0], b[1] - a[1]) < 0.3) { refLine = null; log('reference line shorter than 0.3 m: ignored', 'warn'); refresh(); return; }
      $('rv-rot-deg').value = (alignAngle(a, b) * 180 / Math.PI).toFixed(2);
      rotNote(); refresh();
    }
    return;
  }
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
// Where the pointer is, in map metres (2026-11-12): the numbers for a re-map box by numbers.
// A tap on a touch panel shows it too (pointerdown).
(() => {
  const cv = document.getElementById('rv-canvas'), out = $('rv-xy');
  if (!cv || !out || !cv.addEventListener) return;
  const show = e => {
    if (!view.meta) { out.hidden = true; return; }
    const [x, y] = view.screenToWorld(e.offsetX, e.offsetY);
    out.textContent = `x ${num(x, 2)}  y ${num(y, 2)} m`; out.hidden = false;
  };
  cv.addEventListener('pointermove', show); cv.addEventListener('pointerdown', show);
})();

view.overlays.push((c, v) => {
  ops.forEach((o, i) => {
    const st = style(kind(o));
    if (st.dashed) { v.dashed(o.polygon, st.stroke); v.line(o.polygon[0][0], o.polygon[0][1], o.polygon[2][0], o.polygon[2][1], st.stroke, 1); }
    else v.fillPolygon(o.polygon, st.fill, st.stroke);
    v.text(o.polygon[3][0], o.polygon[3][1], String(i + 1), st.stroke);
  });
  if (v._rect && v.tool && TOOLS[v.tool]) { const st = style(v.tool); v.fillPolygon(v._rect, st.fill, null); v.dashed(v._rect, st.stroke); }
  if (v._rect && v.tool === 'remap') { v.fillPolygon(v._rect, alpha(INK.accent, 0.18), null); v.dashed(v._rect, INK.accent); }
  remapBoxes.forEach((b, i) => { v.fillPolygon(b, alpha(INK.accent, 0.18), INK.accent); v.text(b[3][0], b[3][1], `R${i + 1}`, INK.accent); });
  if (refLine) {
    // the wall as drawn, and dashed: where the rotation will lay it (turned about the origin)
    const [p, q] = refLine, t = rotDeg() * Math.PI / 180, c = Math.cos(t), s = Math.sin(t);
    v.line(p[0], p[1], q[0], q[1], INK.turn, 3);
    const r = ([x, y]) => [c * x - s * y, s * x + c * y];
    v.dashed([r(p), r(q)], INK.pose);
  }
});

function refresh() {
  $('rv-ops').innerHTML = ops.length ? ops.map((o, i) => {
    const [w, h] = size(o.polygon), k = kind(o);
    return `<div class="row three"><span class="k">${i + 1}</span><span class="n">${esc(TOOLS[k] ? TOOLS[k].name : k)}</span>` +
      `<span class="v">${mm(w)} × ${mm(h)}<i>mm</i> <button class="tool" data-del="${i}" title="remove this edit">×</button></span></div>`;
  }).join('') : '<div class="none">no edits</div>';
  $('btn-rv-save').disabled = saving || !ops.length;
  const deg = rotDeg(), canRotate = !saving && !ops.length && Math.abs(deg) >= 0.05 && Math.abs(deg) <= 180;
  $('btn-rv-rotate').disabled = !canRotate;
  $('btn-rv-rotate').title = ops.length ? 'Save or clear the area edits first: a rotation is its own revision' : '';
  remapButtons();
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

$('btn-rv-rotate').onclick = async () => {
  const deg = rotDeg();
  if (saving || ops.length || Math.abs(deg) < 0.05) return;
  if (!window.confirm(`Save ${mapId} turned ${deg > 0 ? '+' : ''}${deg.toFixed(2)}° as a new revision? Routes are drawn again on the rotated map.`)) return;
  const forMap = mapId, forRev = mapRev, c = mapToken;
  saving = true; refresh();
  try {
    const { status, data } = await api(`/api/maps/${forMap}/${forRev}/rotate`, { angle_deg: deg, note: $('rv-note').value.trim() });
    if (status !== 200) { $('rv-msg').textContent = `not saved: ${data.message || status}`; log(`map rotation refused: ${data.message || status}`, 'bad'); return; }
    log(data.message);
    if (c !== mapToken) return;
    $('rv-note').value = '';
    await fillMapSelect($('rv-map'));
    $('rv-map').value = shown = `${forMap}/${data.revision}`;
    await selectMap(shown);
    $('rv-msg').textContent = `${data.message}. Use it on the Run page; draw the routes for it in Routes.`;
  } finally { saving = false; refresh(); }
};

async function selectMap(value) {
  [mapId, mapRev] = value.split('/'); mapRev = +mapRev;
  mapToken += 1; const c = mapToken;
  ops = []; history = []; future = []; view._rect = null; $('rv-msg').textContent = '';
  changes = null; remapBoxes = remapRestore(mapId); remapStale();  // the map's boxes, any revision
  refLine = null; $('rv-rot-deg').value = '0'; refresh();
  const loaded = await view.load(mapId, mapRev);
  if (!loaded || c !== mapToken) return;
  showInfo();
  remapNote();
  loadChanges(mapId, mapRev, c);
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


// ---- Changes seen during runs (2026-11-09): GET /api/maps/<id>/<rev>/changes ------------------
// The localization monitor counts, per 20 cm block, mapped obstacles seen through ("gone", red)
// and objects where the map has floor ("new", yellow) while localisation is READY; regions
// group them with what to do. Mark dynamic / Re-map box take a region's box as an edit.
async function loadChanges(id, rev, token) {
  $('rv-chg-list').innerHTML = '<div class="none">loading…</div>';
  const { status, data } = await apiGet(`/api/maps/${id}/${rev}/changes`);
  if (token !== mapToken) return;
  if (status !== 200 || !data.available) {
    changes = null;
    $('rv-chg-note').textContent = 'Nothing recorded on this revision yet: the AMR learns it while it runs routes on it (localised READY).';
    $('rv-chg-list').innerHTML = '<div class="none">no changes recorded</div>';
    view.draw(); return;
  }
  changes = data;
  $('rv-chg-note').textContent = `${data.scans} scans counted, last ${data.updated}. Older sightings fade (half every 3 days).`;
  const KIND = { gone: 'gone', varies: 'comes and goes', new: 'not on the map' };
  $('rv-chg-list').innerHTML = data.regions.length ? data.regions.map((r, i) => {
    const w = r.box[2] - r.box[0], h = r.box[3] - r.box[1];
    return `<div class="row"><span><span class="k">C${i + 1}</span> <span class="v">${esc(KIND[r.kind] || r.kind)}</span> <span class="n">${num(w, 1)} × ${num(h, 1)} m</span><br>` +
      `<span class="n">${esc(r.advice)}</span></span><span>` +
      `<button class="tool" data-chg-show="${i}">Show</button> <button class="tool" data-chg-dyn="${i}" title="Add a Dynamic area edit over this box (then Save as new revision)">Mark dynamic</button> ` +
      `<button class="tool" data-chg-remap="${i}" title="Add this box to the Re-map boxes below">Re-map box</button></span></div>`;
  }).join('') : '<div class="none">no changed areas: the map fits</div>';
  view.draw();
}
const pad = (b, m) => rect([b[0] - m, b[1] - m], [b[2] + m, b[3] + m]);
$('rv-chg-list').onclick = e => {
  const t = e.target && e.target.dataset; if (!t || !changes) return;
  const i = t.chgShow ?? t.chgDyn ?? t.chgRemap; if (i === undefined) return;
  const r = changes.regions[+i], poly = pad(r.box, 0.2);
  if (t.chgShow !== undefined) { zoomTo(r.box); return; }
  if (t.chgDyn !== undefined) { snapshot(); ops.push(opFor('dynamic', poly)); refresh(); log(`C${+i + 1}: dynamic area added - Save as new revision to keep it`); return; }
  remapBoxes.push(poly); remapStale(); log(`C${+i + 1}: added as re-map box R${remapBoxes.length}`);
};
function zoomTo(b) {
  if (!view.meta) return;
  const [u0, v0] = view.worldToPixel(b[0], b[3]), [u1, v1] = view.worldToPixel(b[2], b[1]);
  const w = Math.max(10, Math.abs(u1 - u0)), h = Math.max(10, Math.abs(v1 - v0));
  view.scale = Math.min(view.canvas.width / w, view.canvas.height / h) * 0.5;
  view.ox = view.canvas.width / 2 - (u0 + u1) / 2 * view.scale; view.oy = view.canvas.height / 2 - (v0 + v1) / 2 * view.scale;
  view.draw();
}
$('rv-chg-show').onchange = () => view.draw();
view.overlays.unshift((c, v) => {
  // the re-map preview ("after"), under every other overlay
  if (c && remapAfter && $('rv-remap-after').checked) c.drawImage(remapAfter, v.ox, v.oy, remapAfter.width * v.scale, remapAfter.height * v.scale);
  if (!changes || !$('rv-chg-show').checked) return;
  const s = changes.block_m / 2;
  changes.blocks.forEach(([x, y, gone, nw]) => {
    const a = Math.min(0.8, 0.25 + (gone + nw) / 40);
    v.fillPolygon([[x - s, y - s], [x + s, y - s], [x + s, y + s], [x - s, y + s]], alpha(gone >= nw ? INK.stop : INK.preview, a), null);
  });
  changes.regions.forEach((r, i) => { const b = r.box; v.dashed([[b[0], b[1]], [b[2], b[1]], [b[2], b[3]], [b[0], b[3]]], INK.ink3); v.text(b[0], b[3], `C${i + 1}`, INK.ink3); });
});

// ---- Re-map area (2026-11-09): POST /api/maps/<id>/<rev>/remap[/preview] ----------------------
// Inside the boxes the map becomes what the revision's survey scan (its Continue mapping drive's
// own scans) saw; the server checks it lines up with the map around the boxes first.
$('tool-remap').onclick = () => setTool(view.tool === 'remap' ? null : 'remap');
$('btn-remap-clear').onclick = () => { remapBoxes = []; remapStale(); };
// A box by numbers (2026-11-12): two opposite corners in map metres, same checks as a drawn box.
const RM_XY = ['rv-rm-x0', 'rv-rm-x1', 'rv-rm-y0', 'rv-rm-y1'];
$('btn-remap-add').onclick = () => {
  const v = RM_XY.map(id => parseFloat($(id).value));
  if (!v.every(Number.isFinite)) { log('re-map box: fill in x from / to and y from / to (metres)', 'bad'); return; }
  const poly = rect([v[0], v[2]], [v[1], v[3]]), [w, h] = size(poly);
  if (w < 0.3 || h < 0.3) { log('re-map box smaller than 0.3 m: not added', 'bad'); return; }
  remapBoxes.push(poly); remapStale(); remapAdded(poly);
};
// Every new box: say where it is and, on a revision without a survey scan, what is still needed.
function remapAdded(poly) {
  log(`re-map box R${remapBoxes.length}: x ${num(poly[0][0], 2)} … ${num(poly[1][0], 2)}, y ${num(poly[0][1], 2)} … ${num(poly[2][1], 2)} m`);
  if (!(view.meta && view.meta.survey_scan)) log('kept for this map: to apply it, Continue mapping through that stretch (Maps page), then open the new revision here - the boxes are still there', 'warn');
}
function remapToolNote() {
  return view.meta && view.meta.survey_scan
    ? 'Drag a box over the stretch that changed, reaching into machines or walls that did not. Then Preview and Save under Re-map area.'
    : 'Drag a box over the stretch that changed. This revision has no survey scan yet, so the box is kept for this map until you open the revision a Continue mapping drive saves; Preview and Save work there.';
}
// Re-map boxes belong to the MAP, not one revision (2026-11-12): draw them first, do the
// Continue mapping drive, open the revision it saved - they are still there (also after a reload).
const remapKey = id => `amr.remap.${id}`;
function remapStore() { try { if (mapId) localStorage.setItem(remapKey(mapId), JSON.stringify(remapBoxes)); } catch (e) { /* no storage */ } }
function remapRestore(id) {
  try { const v = JSON.parse(localStorage.getItem(remapKey(id)) || '[]'); return Array.isArray(v) ? v : []; } catch (e) { return []; }
}
function remapNote() {
  const has = view.meta && view.meta.survey_scan;
  $('rv-remap-note').textContent = has
    ? 'This revision carries the scans of the Continue mapping drive that saved it. Drag Re-map boxes over the stretch that changed (reach into machines or walls that did not change), Preview, then Save.'
    : 'No survey scan in this revision: you can draw the boxes now (Re-map box above, or by numbers), they stay with this map. To apply them, on the Maps page Continue mapping from this revision and drive slowly through the changed stretch (back to the mark, Save), then open the revision that saves here: Preview and Save work there.';
  if (view.tool === 'remap') $('rv-tool-note').textContent = remapToolNote();
}
function remapStale() {
  remapAfter = null; remapCheck = null; $('rv-remap-after').checked = false;
  $('rv-remap-check').textContent = ''; $('rv-remap-check').classList.toggle('bad', false);
  $('rv-remap-boxes').innerHTML = remapBoxes.length ? remapBoxes.map((b, i) => {
    const [w, h] = size(b);
    const xy = `x ${num(b[0][0], 2)} … ${num(b[1][0], 2)} · y ${num(b[0][1], 2)} … ${num(b[2][1], 2)}`;
    return `<div class="row three"><span class="k">R${i + 1}</span><span class="n" title="${xy} m">${xy}</span><span class="v">${num(w, 2)} × ${num(h, 2)}<i>m</i> <button class="tool" data-rm-edit="${i}" title="put its corners in the boxes above to change them (removes it here)">✎</button><button class="tool" data-rm="${i}" title="remove this box">×</button></span></div>`;
  }).join('') : '<div class="none">no boxes</div>';
  remapStore(); remapButtons(); view.draw();
}
$('rv-remap-boxes').onclick = e => {
  const d = e.target && e.target.dataset ? e.target.dataset : {};
  if (d.rmEdit !== undefined) {  // back into the number boxes, to adjust and Add again
    const b = remapBoxes[+d.rmEdit]; if (!b) return;
    [b[0][0], b[1][0], b[0][1], b[2][1]].forEach((v, k) => { $(RM_XY[k]).value = v.toFixed(2); });
    remapBoxes.splice(+d.rmEdit, 1); remapStale(); return;
  }
  if (d.rm === undefined) return;
  remapBoxes.splice(+d.rm, 1); remapStale();
};
function remapButtons() {
  const has = !!(view.meta && view.meta.survey_scan);
  $('btn-remap-preview').disabled = saving || !has || !remapBoxes.length;
  $('btn-remap-preview').title = has ? '' : 'needs a revision saved by Continue mapping (see the note above)';
  $('rv-remap-after').disabled = !remapAfter;
  const ok = remapCheck && remapCheck.ok !== false;
  $('btn-remap-save').disabled = saving || !has || !remapBoxes.length || !ok || ops.length > 0;
  $('btn-remap-save').textContent = remapCheck && remapCheck.ok === null ? 'Save re-mapped revision (not checked)' : 'Save re-mapped revision';
  $('btn-remap-save').title = ops.length ? 'Save or clear the area edits first: a re-map is its own revision' : (remapCheck ? '' : 'Preview first');
}
$('rv-remap-after').onchange = () => view.draw();
$('btn-remap-preview').onclick = async () => {
  const forMap = mapId, forRev = mapRev, c = mapToken, boxes = remapBoxes.slice();
  $('rv-remap-check').textContent = 'checking…';
  const { status, data } = await api(`/api/maps/${forMap}/${forRev}/remap/preview`, { boxes });
  if (c !== mapToken) return;
  if (status !== 200) { $('rv-remap-check').textContent = `not possible: ${data.message || status}`; $('rv-remap-check').classList.toggle('bad', true); return; }
  remapCheck = data.check;
  remapAfter = await new Promise(res => { const i = new Image(); i.onload = () => res(i); i.onerror = () => res(null); i.src = data.image; });
  const t = `${data.changed_cells} cells change. ${data.check.text}`;
  $('rv-remap-check').textContent = data.check.ok === false ? `${t}. Not saveable.` : data.check.ok === null ? `${t}. Look at "Show after" carefully before saving.` : t;
  $('rv-remap-check').classList.toggle('bad', data.check.ok !== true);
  $('rv-remap-after').checked = true;
  remapButtons(); view.draw();
};
$('btn-remap-save').onclick = async () => {
  if (saving || !remapCheck) return;
  const forMap = mapId, forRev = mapRev, c = mapToken, force = remapCheck.ok === null;
  if (force && !window.confirm('The alignment could not be checked (nothing fixed around the boxes). Save it anyway, as the preview shows?')) return;
  saving = true; refresh();
  try {
    const { status, data } = await api(`/api/maps/${forMap}/${forRev}/remap`, { boxes: remapBoxes.slice(), note: $('rv-note').value.trim(), force });
    if (status !== 200) { $('rv-remap-check').textContent = `not saved: ${data.message || status}`; $('rv-remap-check').classList.toggle('bad', true); log(`re-map refused: ${data.message || status}`, 'bad'); return; }
    log(data.message);
    if (c !== mapToken) return;
    $('rv-note').value = '';
    remapBoxes = []; remapStale();  // applied: they are done
    await fillMapSelect($('rv-map'));
    $('rv-map').value = shown = `${forMap}/${data.revision}`;
    await selectMap(shown);
    $('rv-msg').textContent = `${data.message}. Validate and save the routes on it in Routes, make their missions again, then use it on the Run page.`;
  } finally { saving = false; refresh(); }
};
