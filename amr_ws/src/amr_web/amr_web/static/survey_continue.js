// Continue survey (2026-10-23): pick a saved revision with a pose graph, say roughly where the
// vehicle stands on it, and start. The session coordinator matches the live scan against the
// map before slam_toolbox loads the graph and refuses when unsure. Moves nothing.
const view = new MapView(document.getElementById('cs-canvas'));
const $ = id => document.getElementById(id);
let mapId = null, mapRev = null, hint = null, mapToken = 0, busy = false;

const deg = r => r * 180 / Math.PI;
function startMark() {
  const s = view.meta && view.meta.start;
  return s && s.x_m !== undefined ? { x: +s.x_m, y: +s.y_m, yaw: +(s.yaw_rad || 0) } : null;
}
// "Returned to start" stores (pose - start) in the map frame: the survey ended at start + review
function surveyEnd() {
  const s = startMark(), r = view.meta && view.meta.review;
  if (!s || !r || r.dx_m === undefined) return null;
  return { x: s.x + +r.dx_m, y: s.y + +r.dy_m, yaw: Math.atan2(Math.sin(s.yaw + +r.dyaw_rad), Math.cos(s.yaw + +r.dyaw_rad)) };
}
function setHint(h, source) {
  hint = h ? { ...h, source } : null;
  if (!hint) $('cs-hint').innerHTML = '<div class="none">no pose yet</div>';
  else tiles('cs-hint', [['x', num(hint.x, 2), 'm', '', 'key'], ['y', num(hint.y, 2), 'm'], ['heading', num(deg(hint.yaw), 1), '°'], ['from', source, '']]);
  refresh();
}
function dropTool() { view.tool = null; $('tool-hint').classList.remove('on'); }
function refresh() {
  const ok = !!(view.meta && view.meta.continuable);
  $('btn-cs-start').disabled = busy || !ok || !hint;
  $('cs-at-end').disabled = !surveyEnd();
  $('cs-at-start').disabled = !startMark();
  view.draw();
}

view.overlays.push((c, v) => {
  const s = startMark();
  if (s) v.arrow(s.x, s.y, s.yaw, 0.8, INK.ink3);
  if (hint) v.arrow(hint.x, hint.y, hint.yaw, 1.0, INK.preview);
});
view.onDrag = (a, b, done) => {
  if (view.tool !== 'hint') return;
  const yaw = Math.atan2(b[1] - a[1], b[0] - a[0]);
  hint = { x: a[0], y: a[1], yaw, source: 'drawn' };
  if (done) { dropTool(); setHint({ x: a[0], y: a[1], yaw }, 'drawn'); }
};
view.onClick = () => log('drag from the vehicle\'s position towards its heading', 'warn');
$('tool-hint').onclick = () => { if (view.tool) { dropTool(); return; } view.tool = 'hint'; $('tool-hint').classList.add('on'); };
$('cs-at-end').onclick = () => { dropTool(); setHint(surveyEnd(), 'survey end'); };
$('cs-at-start').onclick = () => { dropTool(); setHint(startMark(), 'start mark'); };

function showInfo() {
  const m = view.meta, r = m.review || {};
  const from = r.continued_from ? `continued from rev${r.continued_from.revision}` : 'surveyed';
  tiles('cs-info', [
    ['Map', `${m.map_id} rev${m.revision}`, from, '', 'key'],
    ['Size', `${num(m.width * m.resolution, 0)}×${num(m.height * m.resolution, 0)}`, 'm'],
    ['Pose graph', m.continuable ? 'saved' : 'none', m.continuable ? 'can be continued' : 'merged or imported: cannot be continued', m.continuable ? 'ok' : 'bad'],
  ]);
  $('cs-msg').textContent = m.continuable ? '' : 'This revision has no saved pose graph. Pick a surveyed revision.';
}
async function selectMap(value) {
  [mapId, mapRev] = value.split('/'); mapRev = +mapRev;
  mapToken += 1; const c = mapToken;
  dropTool(); hint = null; setHint(null);
  const loaded = await view.load(mapId, mapRev);
  if (!loaded || c !== mapToken) return;
  showInfo(); refresh();
}
$('cs-map').onchange = e => selectMap(e.target.value);

$('btn-cs-start').onclick = async () => {
  if (!hint || busy) return;
  busy = true; refresh();
  $('cs-msg').textContent = `starting: matching the scan on ${mapId} rev${mapRev}, then loading its pose graph…`;
  const body = { map_id: mapId, description: $('cs-desc').value.trim(), continue_revision: mapRev,
                 hint: { x_m: hint.x, y_m: hint.y, yaw_rad: hint.yaw } };
  await operation('/api/survey/start', body, op => {
    busy = false; refresh();
    if (!op) { $('cs-msg').textContent = 'refused: see the log'; return; }
    const ok = op.status_name === 'SUCCEEDED';
    $('cs-msg').textContent = ok ? `${op.message}. Continue on the Maps page.` : `not continued: ${op.message}`;
    if (ok) window.setTimeout(() => { window.location.href = '/maps'; }, 2500);
  });
};

(async () => {
  await fillMapSelect($('cs-map'));
  const q = new URLSearchParams(window.location.search), want = q.get('map') && q.get('rev') ? `${q.get('map')}/${q.get('rev')}` : null;
  const opts = Array.from($('cs-map').options || []).map(o => o.value);
  const first = want && opts.includes(want) ? want : opts[opts.length - 1];
  if (!first) { $('cs-msg').textContent = 'no saved maps yet: survey one on the Maps page first'; return; }
  $('cs-map').value = first;
  await selectMap(first);
})();
