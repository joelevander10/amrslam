// Maps page: survey session controls (asynchronous supervisor operations) and the saved revisions.
const $ = id => document.getElementById(id);
const busy = (on) => ['btn-survey-start', 'btn-survey-returned', 'btn-survey-save', 'btn-survey-abort'].forEach(id => $(id).disabled = on);
function op(path, body) { busy(true); return operation(path, body, () => { busy(false); loadMaps().catch(() => {}); }); }
const MAP_ID_RE = /^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$/;  // same rule as the supervisor (readiness.check_map_id)
$('btn-survey-start').onclick = () => {
  const mapId = $('survey-map-id').value.trim();
  if (!MAP_ID_RE.test(mapId)) {
    // the grey placeholder is not a value: an empty field was the usual cause
    log(mapId ? `map id "${mapId}": letters, digits, '_' and '-' only (start with a letter or digit)`
              : 'type a Map id first (e.g. line_section)', 'bad');
    $('survey-map-id').focus();
    return;
  }
  op('/api/survey/start', { map_id: mapId, description: $('survey-desc').value.trim() });
};
$('btn-survey-returned').onclick = () => op('/api/survey/returned');
$('btn-survey-save').onclick = () => op('/api/survey/save', { note: $('survey-note').value.trim() });
$('btn-survey-abort').onclick = () => op('/api/survey/abort');
const SURVEY_LEVEL = { SAVING: 'warn', RETURN_REVIEW: 'warn' };
const noClosure = '<div class="none">available after "Returned to start"</div>';
onState(st => {
  const mode = st.mode;
  const m = st.mapping;
  if (!mode) { tiles('survey-state', [['Session', '–', 'no supervisor', 'bad', 'key']]); $('survey-msg').textContent = ''; return; }
  if (mode.mode_name !== 'MAPPING') {
    tiles('survey-state', [
      ['Mode', mode.mode_name, mode.phase || '—', MODE_LEVEL[mode.mode_name], 'key'],
      ['Last saved survey', mode.last_survey_map_id ? `${mode.last_survey_map_id} rev${mode.last_survey_revision}` : '—',
       mode.last_survey_map_id ? 'use it from the Run page' : '—'],
    ]);
    $('survey-msg').textContent = mode.reason || '';
    $('closure').innerHTML = noClosure;
    return;
  }
  if (!m) {
    tiles('survey-state', [['Session', 'MAPPING', 'waiting for the session coordinator', 'warn', 'key']]);
    $('survey-msg').textContent = '';
    return;
  }
  tiles('survey-state', [
    ['Session', m.state_name, 'coordinator', SURVEY_LEVEL[m.state_name], 'key'],
    ['Map', m.map_id ? `${m.map_id}${m.revision ? ' rev' + m.revision : ''}` : '—', 'id · revision'],
  ]);
  $('survey-msg').textContent = m.message || '';
  if (m.closure_available) {
    tiles('closure', [
      ['dx', num(m.closure_dx_m, 3), 'm', 'warn'],
      ['dy', num(m.closure_dy_m, 3), 'm', 'warn'],
      ['dyaw', num(m.closure_dyaw_rad * 180 / Math.PI, 2), '°', 'warn'],
    ]);
  } else $('closure').innerHTML = noClosure;
});
// Preset survey moves (survey_move_node): press once; the node checks MANUAL authority itself.
async function surveyMove(kind, value) {
  let r;
  try { r = await api('/api/survey_move', { kind, value }); } catch (e) { $('sm-status').textContent = 'no response'; return; }
  if (r.status !== 200) $('sm-status').textContent = `refused: ${r.data.message || r.status}`;
}
const dist = () => parseFloat($('sm-dist').value);
$('sm-fwd').onclick = () => surveyMove('straight', dist());
$('sm-rev').onclick = () => surveyMove('straight', -dist());
document.querySelectorAll('#sm-spins button, #sm-spins-cw button').forEach(b => { b.onclick = () => surveyMove('rotate', +b.dataset.deg); });
$('sm-stop').onclick = () => api('/api/survey_move/stop', {}).catch(() => {});
const SM_BUTTONS = () => [...document.querySelectorAll('#sm-spins button, #sm-spins-cw button'), $('sm-fwd'), $('sm-rev')];
onState(st => {
  const surveying = st.mode && st.mode.mode_name === 'MAPPING' && st.mapping && st.mapping.state_name === 'MAPPING';
  const s = st.survey_move;
  const moving = !!s && (s.state_name === 'MOVING' || s.state_name === 'SETTLING');
  SM_BUTTONS().forEach(b => { b.disabled = !surveying || moving; });
  $('sm-stop').disabled = !(s && s.state_name === 'MOVING');
  const el = $('sm-status');
  el.classList.remove('bad', 'ok');
  if (!surveying) { el.textContent = 'available while surveying'; return; }
  // the node publishes at 5 Hz: a state older than 2 s is a helper that is not running
  if (s && s.age_s != null && s.age_s > 2.0) {
    SM_BUTTONS().forEach(b => { b.disabled = true; });
    el.textContent = 'move helper not running (it restarts by itself; if not, restart the survey)';
    el.classList.add('bad');
    return;
  }
  if (!s || s.state_name === 'IDLE') { el.textContent = 'ready'; return; }
  const unit = s.kind === 'rotate' ? '°' : ' m';
  const prog = s.kind === 'rotate' ? num(s.progress * 180 / Math.PI, 1) : num(s.progress, 2);
  if (s.state_name === 'MOVING') el.textContent = `${s.label}: ${prog}${unit}`;
  else if (s.state_name === 'SETTLING') el.textContent = `${s.label}: stopped at ${prog}${unit}, checking the map…`;
  else if (s.state_name === 'ABORTED') { el.textContent = `${s.label}: stopped at ${prog}${unit} (${s.reason})`; el.classList.add('bad'); }
  else { el.textContent = `${s.label}: done at ${prog}${unit}. ${s.reason}`; el.classList.add(s.check_done && !s.check_ok ? 'bad' : 'ok'); }
});
async function loadMaps() {
  const { data } = await apiGet('/api/maps');
  if (!data.length) { $('maps-list').innerHTML = '<div class="none">no saved maps yet</div>'; return; }
  const head = '<div class="row head"><span>rev · created · size</span><span>closure review · routes · bundle</span></div>';
  $('maps-list').innerHTML = data.map(m => `<h3>${esc(m.map_id)}</h3><div class="list">${head}` +
    m.revisions.map(r => r.error
      ? `<div class="al-row standing lv-error"><span class="al-lv">rev${esc(r.revision)}</span><span class="al-msg">${esc(r.error)}</span></div>`
      : `<div class="row"><span><span class="k">rev${esc(r.revision)}</span> <span class="n">${esc(r.created)}</span><br>` +
        `<span class="v">${esc(r.width)}×${esc(r.height)}<i>@ ${esc(r.resolution)} m</i></span></span>` +
        `<span><span class="n">${r.review ? `dx ${num(+r.review.dx_m, 2)} · dy ${num(+r.review.dy_m, 2)} · dyaw ${num(+r.review.dyaw_rad * 180 / Math.PI, 1)}° ${esc(r.review.note)}` : 'no review'}</span><br>` +
        `<span class="n">${Object.entries(r.routes || {}).map(([k, v]) => `${esc(k)} rev${esc(v.join(','))}`).join('; ') || 'no routes'}</span> ` +
        `<code>${esc(String(r.sha256).slice(0, 12))}</code><br>` +
        `<span class="n">${derived(r)}</span> ` +
        `<a class="tool" href="/review?map=${encodeURIComponent(m.map_id)}&rev=${encodeURIComponent(r.revision)}" title="Mark dynamic areas (trolleys, parked forklifts) or clean up this map: saves a new revision">Edit areas</a></span></div>`).join('') + '</div>').join('');
}
// "dynamic 4.2 m² · from rev1 (3 edits: dynamic 2, unknown 1)": how a revision came to be
function derived(r) {
  const parts = [];
  if (r.dynamic_cells) parts.push(`dynamic ${num(r.dynamic_cells * r.resolution * r.resolution, 1)} m²`);
  if (r.edits) parts.push(`from rev${esc(r.edits.parent_revision)} (${esc(r.edits.ops)} edits: ${Object.entries(r.edits.counts || {}).map(([k, n]) => `${esc(k)} ${esc(n)}`).join(', ')})${r.edits.note ? ' ' + esc(r.edits.note) : ''}`);
  return parts.join(' · ') || 'no dynamic areas';
}
loadMaps();

// live occupancy + pose while surveying, and the shared jog pad beside the controls
const liveView = new MapView(document.getElementById('live-canvas'));
liveOverlay(liveView);
pollLive(liveView, { map: true });
jogpad(document.getElementById('jog-maps'), { compact: true });
