// Shared helpers: JSON requests, a log line, the 2 Hz state poll every page uses,
// the telemetry rail, tile rendering, and the asynchronous-operation helper
// (unified plan §6.2: accepted != completed).
const REQUEST_TIMEOUT_MS = 8000;  // a hung request must not hang a page (Q11); it rejects like a network error
async function request(path, method, body) {
  const ctl = typeof AbortController === 'function' ? new AbortController() : null;
  const timer = ctl ? setTimeout(() => ctl.abort(), REQUEST_TIMEOUT_MS) : null;
  let r;
  try {
    r = await fetch(path, { method, headers: { 'Content-Type': 'application/json' },
                            body: body === undefined ? undefined : JSON.stringify(body), signal: ctl ? ctl.signal : undefined });
  } finally { if (timer) clearTimeout(timer); }
  let data = null;
  try { data = await r.json(); } catch (e) { data = { ok: false, message: r.statusText }; }
  if (!r.ok && data && data.message) log(data.message, 'bad');
  return { status: r.status, data };
}
// Every value from the robot, a saved file or an operator note goes into innerHTML only
// through esc(): route step ids, survey notes and event text are data, not markup (R30).
const esc = v => String(v ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const api = (path, body) => request(path, 'POST', body || {});
const apiGet = (path) => request(path, 'GET');
function log(msg, cls) {
  const el = document.getElementById('log');
  const t = new Date().toLocaleTimeString();
  el.textContent = `${t}  ${msg}\n` + el.textContent.split('\n').slice(0, 60).join('\n');
}
// Number formatting: fixed decimals, tabular in CSS; "–" for anything missing or nonfinite.
const num = (v, dp) => (typeof v === 'number' && isFinite(v)) ? v.toFixed(dp) : '–';
const yesno = v => v === undefined || v === null ? '–' : (v ? 'YES' : 'NO');

function pill(id, text, cls) { const e = document.getElementById(id); if (!e) return; e.textContent = text; e.className = 'pill ' + (cls || ''); }
// One telemetry tile: <div class="tel"><span>label</span><b>value</b><i>note</i></div>.
function tile(id, value, note, level, stale) {
  const e = typeof id === 'string' ? document.getElementById(id) : id; if (!e) return;
  e.querySelector('b').textContent = value;
  e.querySelector('i').textContent = note || '—';
  e.classList.toggle('warn', level === 'warn');
  e.classList.toggle('bad', level === 'bad');
  e.classList.toggle('stale', !!stale);
}
// A row of tiles into a container: rows = [[label, value, unit, level, extraClass], ...].
function tiles(container, rows) {
  const e = typeof container === 'string' ? document.getElementById(container) : container; if (!e) return;
  e.innerHTML = rows.map(([label, value, unit, level, extra]) =>
    `<div class="tel ${level || ''} ${extra || ''}"><span>${esc(label)}</span><b>${esc(value)}</b><i>${esc(unit || '—')}</i></div>`).join('');
}

const stateListeners = [];
function onState(fn) { stateListeners.push(fn); }
let lastState = null;
const MODE_LEVEL = { STARTING: 'warn', TRANSITIONING: 'warn', FAULT: 'bad', STOPPING: 'bad' };
const LOC_LEVEL = { CHECKING: 'warn', LOST: 'bad' };
const RUN_LEVEL = { BLOCKED: 'warn', PAUSED: 'warn', FAULT: 'bad' };
const FRESH_S = 0.5;  // panel / drives / mux samples older than this are not current

function rail(st) {
  const m = st.mode;
  tile('tel-mode', m ? m.mode_name : '–', m ? (m.phase || '—') : 'no supervisor', m ? MODE_LEVEL[m.mode_name] : 'bad');
  const p = st.panel;
  if (!p) tile('tel-selector', '–', 'no panel image', 'bad');
  else if (p.age_s > FRESH_S) tile('tel-selector', 'STALE', `${num(p.age_s, 1)} s old`, 'bad');
  else if (!p.valid) tile('tel-selector', 'INVALID', `${num(p.age_s, 2)} s`, 'bad');
  else tile('tel-selector', p.mode_auto ? 'AUTO' : 'MANUAL', `${num(p.age_s, 2)} s`);
  const d = st.drives;
  if (!d) tile('tel-drives', '–', 'no drive status', 'bad');
  else if (d.age_s > FRESH_S) tile('tel-drives', 'STALE', `${num(d.age_s, 1)} s old`, 'bad');
  else tile('tel-drives', d.operational ? 'ARMED' : 'OFF', `${d.left || '?'} · ${d.right || '?'}`, d.operational ? '' : 'warn');
  const mx = st.mux, mxStale = !mx || mx.age_s > FRESH_S;
  tile('tel-source', mx ? mx.source : '–', mx ? (mx.inhibited ? 'inhibited' : '—') : 'no mux state', mx && mx.inhibited ? 'warn' : '', mxStale);
  tile('tel-wheels', mx ? `${num(mx.left_rad_s, 2)} / ${num(mx.right_rad_s, 2)}` : '–', 'rad/s · L / R', '', mxStale);
  const l = st.localization;
  tile('tel-loc', l ? l.state_name : '–', st.localization_stale ? 'stale (replaced layer)' : (l ? '—' : 'no layer'),
       l ? LOC_LEVEL[l.state_name] : '', st.localization_stale);
  const r = st.run;
  // the run's own reason (Start refused / ignored, hold, fault) also lands in the log box on
  // every page, where operators look after pressing START (2026-09-24: "no error" while the
  // refusal was only on the Run page)
  const why = r && !st.run_stale ? (r.reason || '') : '';
  if (why && why !== railRunReason) log(`run ${r.state_name}: ${why}`, ['FAULT', 'BLOCKED'].includes(r.state_name) || /refus|ignor/i.test(why) ? 'bad' : '');
  railRunReason = why;
  tile('tel-run', r ? r.state_name : '–', st.run_stale ? 'stale (replaced layer)' : (r ? (r.step_id || '—') : 'no executor'),
       r ? RUN_LEVEL[r.state_name] : '', st.run_stale);
  tile('tel-gen', m ? String(m.generation) : '–', st.lease ? 'lease' : 'no lease', st.lease ? '' : 'warn');
}
let railRunReason = null;
function railStale(on) { const e = document.getElementById('telemetry'); if (e) e.classList.toggle('stale', on); }

async function poll() {
  try {
    const { status, data } = await apiGet('/api/state');
    if (status === 200) {
      lastState = data;
      pill('pill-link', 'connected', 'ok');
      railStale(false);
      rail(data);
      stateListeners.forEach(fn => fn(data));
    } else { pill('pill-link', 'error ' + status, 'bad'); railStale(true); }
  } catch (e) { pill('pill-link', 'disconnected', 'bad'); railStale(true); }
  setTimeout(poll, 500);
}
poll();

// Wi-Fi header readout: a comfort indicator for whoever holds the tablet, not an authority.
async function pollWifi() {
  const el = document.getElementById('wifi'), txt = document.getElementById('wifi-text');
  if (!el) return;
  let w = null;
  try { const { status, data } = await apiGet('/api/wifi'); if (status === 200) w = data; } catch (e) { /* offline */ }
  const bars = w ? w.bars : 0;
  el.querySelectorAll('.bars b').forEach((b, i) => b.classList.toggle('on', i < bars));
  el.className = 'wifi ' + (!w || !w.connected ? 'bad' : bars >= 3 ? 'ok' : bars >= 2 ? '' : 'warn');
  txt.textContent = !w ? '–' : !w.connected ? `${w.iface} no link` : `${w.ssid || w.iface} ${Math.round(w.dbm)} dBm`;
  setTimeout(pollWifi, 2000);
}
pollWifi();

// Internet as the ROBOT sees it (the server probes; the tablet's own link says nothing about
// it). 0.25 Hz: the server caches its probe for 4 s anyway. Informational only.
async function pollNet() {
  const el = document.getElementById('net');
  if (!el) return;
  let n = null;
  try { const { status, data } = await apiGet('/api/internet'); if (status === 200) n = data; } catch (e) { /* offline */ }
  el.className = 'net ' + (!n || n.online == null ? '' : n.online ? 'ok' : 'bad');
  el.title = !n ? 'Internet: robot not reachable'
    : n.online == null ? 'Internet: checking…'
    : n.online ? `Internet: online (${n.via}, ${n.rtt_ms} ms) - rechecked every 4 s` : 'Internet: offline - rechecked every 4 s';
  setTimeout(pollNet, 4000);
}
pollNet();

// Submit an asynchronous supervisor operation and follow it to a terminal status.
// A refresh/reconnect recovers progress through /api/operations/<id>; nothing is resubmitted.
// onDone is called EXACTLY once on every path - success, refusal, network failure, timeout -
// so a page's busy state always unwinds (review Q11). A request that never got a response
// may still have executed on the robot; it is not resubmitted, the outcome is "unknown".
async function operation(path, body, onDone) {
  const rid = (crypto.randomUUID ? crypto.randomUUID() : String(Date.now()));
  let status, data;
  try { ({ status, data } = await api(path, Object.assign({ request_id: rid }, body || {}))); }
  catch (e) { log(`${path}: no response (outcome unknown; check the state before retrying)`, 'bad'); if (onDone) onDone(null); return null; }
  if (status !== 202) { log(data.message || `refused (${status})`, 'bad'); if (onDone) onDone(null); return null; }
  log(`accepted: ${data.message} (operation ${data.operation_id})`);
  return followOperation(data.operation_id, onDone);
}
async function followOperation(id, onDone) {
  const deadline = Date.now() + 300000;
  let gap = 500;
  while (Date.now() < deadline) {
    await new Promise(r => setTimeout(r, gap));
    let status, data;
    try { ({ status, data } = await apiGet(`/api/operations/${id}`)); }
    catch (e) { gap = Math.min(gap * 2, 4000); continue; }  // offline: back off, keep following the same id
    gap = 500;
    if (status !== 200) continue;
    if (data.status_name !== 'PENDING') {
      log(`operation ${id}: ${data.status_name}${data.message ? ' — ' + data.message : ''}`, data.status_name === 'SUCCEEDED' ? '' : 'bad');
      if (onDone) onDone(data);
      return data;
    }
    if (data.phase) tile('tel-mode', '…', data.phase, 'warn');
  }
  log(`operation ${id}: still pending after 5 min (outcome unknown)`, 'bad');
  if (onDone) onDone(null);
  return null;
}
