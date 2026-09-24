// Shared plumbing: API calls, the telemetry poll, and the footer readout.

// The method is explicit, never inferred. An earlier version picked GET when no
// body was passed, which silently turned /api/stop and /api/disarm - both of
// which legitimately take no body - into GETs against POST-only routes. They
// 405'd, and because the stop call swallows its errors, releasing a button
// quietly did nothing and the AGV only stopped when the watchdog expired.
async function request(path, method, body) {
  const opt = {method};
  if (method !== 'GET') {
    opt.headers = {'Content-Type': 'application/json'};
    opt.body = JSON.stringify(body || {});
  }
  const r = await fetch(path, opt);
  let data = {};
  try { data = await r.json(); } catch (e) { /* empty body */ }
  if (!r.ok) throw new Error(data.error || `${r.status} ${r.statusText}`);
  return data;
}

// Every mutating endpoint is a POST; /api/state and /api/config are the reads.
const api = (path, body) => request(path, 'POST', body);
const apiGet = (path) => request(path, 'GET');

const logEl = document.getElementById('log');
function log(msg, cls) {
  const t = new Date().toLocaleTimeString('en-GB');
  logEl.textContent = `${t}  ${msg}\n` + logEl.textContent;
  logEl.textContent = logEl.textContent.split('\n').slice(0, 40).join('\n');
}

// ---- server event log -----------------------------------------------------
// The vehicle stops itself on line loss, sensor timeout and watchdog, so the
// reason has to outlive a page reload. The server keeps the ring buffer; this
// only mirrors it. /api/state carries the high-water mark, so we fetch the
// events themselves only when we have actually fallen behind.
let eventSeq = 0;
let eventBusy = false;

async function syncEvents(latest) {
  if (eventBusy || latest === undefined || latest === null) return;
  if (latest <= eventSeq) return;
  eventBusy = true;
  try {
    const r = await apiGet(`/api/events?since=${eventSeq}`);
    for (const e of r.events) {
      const t = new Date(e.t * 1000).toLocaleTimeString('en-GB');
      const mark = e.level === 'error' ? '!! ' : (e.level === 'warn' ? ' ! ' : '   ');
      logEl.textContent = `${t}${mark}${e.msg}\n` + logEl.textContent;
    }
    logEl.textContent = logEl.textContent.split('\n').slice(0, 60).join('\n');
    eventSeq = r.seq;
  } catch (err) {
    /* a missed sync is retried on the next poll; never disturb the page */
  } finally {
    eventBusy = false;
  }
}

function setPill(text, cls) {
  const p = document.getElementById('link');
  p.textContent = text;
  p.className = 'pill' + (cls ? ' ' + cls : '');
}

// ---- RFID station tags ----------------------------------------------------
// Lives here rather than on one page: reading a tag's four-hex value means
// jogging the vehicle over it by hand, so it belongs on the page with the
// arrows - and every page benefits from seeing the reader is alive.
//
// Every lookup is guarded, so a page showing three tiles and a page showing four
// run the same code. No-ops entirely on a page with no #r-tag.
//
// The reader is on its own thread; this only ever renders what it published.
function showRfid(r) {
  const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
  if (!r || !document.getElementById('r-tag')) return;
  // Live only. The tag shows while it is being READ and clears when the vehicle
  // rolls off it - no last_tag fallback and no age counting up, because a
  // four-hex value sitting on screen with "31.4 s ago" beside it reads as a
  // tag that is still there. The reader holds `tag` for rfid.tag_hold_s, so
  // this is still on screen long enough to write down.
  set('r-tag', r.tag || '–');
  set('r-age', r.tag ? 'reading' : '—');
  set('r-count', r.tags_seen);

  // The standing junction order used to be shown here. It went with tape
  // following - nothing consumes a tag today. The reader itself is still live
  // above, because station identity is what a localiser will want from it.

  // Three states worth distinguishing, because they need different actions:
  // disabled (nothing to do), carrier down (physical), connected but silent
  // (reader wedged, or the protocol is still wrong).
  const link = document.getElementById('r-link');
  if (!link) return;
  let text, colour;
  if (!r.enabled)            { text = 'off';      colour = ''; }
  else if (r.carrier === false) { text = 'NO CABLE'; colour = 'var(--stop)'; }
  else if (!r.connected)     { text = 'down';     colour = 'var(--stop)'; }
  else if (!r.comms_ok)      { text = 'silent';   colour = 'var(--hazard-ink)'; }
  else                       { text = 'ok';       colour = ''; }
  link.textContent = text;
  link.style.color = colour;
  const detail = r.silent ? 'silent — check antenna'
                          : (r.identity || r.detail || '—');
  set('r-detail', detail);
  // Clamped to two lines in CSS, so the full banner lives in the tooltip
  // rather than being lost.
  const dEl = document.getElementById('r-detail');
  if (dEl) dEl.title = detail;
}

// ---- the shared rail ------------------------------------------------------
// Battery, state and alarm, on every page. Rendered here for
// the same reason the verdicts are COMPUTED on the server: four pages deciding
// separately what counts as an alarm is four chances for one of them to say
// everything is fine while the vehicle is stopped.

function setText(id, v) {
  const el = document.getElementById(id);
  if (el) el.textContent = v;
  return el;
}

function renderRail(s) {
  // Battery: the lower of the two drives. canmon already applied the profile's
  // thresholds, so the colour comes from its verdict rather than from a second
  // copy of the limits living in the browser.
  const b = s.battery || {};
  const bv = setText('batt', b.volts === null || b.volts === undefined
                             ? '–' : b.volts.toFixed(1));
  if (bv) bv.style.color = b.state === 'trip' ? 'var(--stop)'
                         : b.state === 'warn' ? 'var(--hazard-ink)' : '';
  setText('batt-note', b.warn_low ? `V · low at ${b.warn_low}` : 'V');

  // State. `mode` is what the vehicle is armed AS; armed is whether it is
  // energised at all. Both, because "AUTO" while disarmed is not the same
  // thing as "AUTO" while running and must not read as it.
  const mode = (s.mode || '—').toUpperCase();
  const st = setText('vstate', s.armed ? mode : 'IDLE');
  if (st) st.style.color = s.armed ? 'var(--ok)' : '';
  setText('vstate-note', s.armed
    ? (s.auto_running ? 'armed · RUNNING' : 'armed')
    : (s.mode ? `${mode.toLowerCase()} selected` : 'not armed'));

  // Alarm. One line; the full list is on /alarms.
  const a = s.alarm || {};
  const al = setText('alarm', a.level === 'error' ? 'ALARM'
                            : a.level === 'warn' ? 'WARN' : 'none');
  if (al) al.style.color = a.level === 'error' ? 'var(--stop)'
                         : a.level === 'warn' ? 'var(--hazard-ink)' : '';
  setText('alarm-note', a.detail || 'nothing outstanding');

  // Yaw rate from the MLS's IMU. One number; the full set is on /monitor.
  // Greyed rather than blanked when stale - the last value is still worth
  // something, a frozen number that looks live is not.
  const imuTile = document.getElementById('rail-imu');
  const imu = s.imu;
  if (imuTile) {
    imuTile.hidden = !(imu && imu.enabled);
    if (!imuTile.hidden) {
      const gz = (imu.gyro_dps || [])[2];
      const el = setText('rail-gz', gz === null || gz === undefined ? '–' : gz.toFixed(1));
      imuTile.classList.toggle('stale', !!imu.stale);
      if (el) el.style.color = imu.stale ? 'var(--stop)' : '';
      setText('rail-gz-note', imu.stale ? '°/s · STALE' : '°/s · gyro z');
    }
  }

  // The horn-and-lights coil. Shown when the profile drives one, so an
  // operator can see the vehicle ASKED for the horn - a horn that is off
  // because nothing commanded it and one that is off because the DIO module
  // is down look the same from the bay.
  const hornTile = document.getElementById('rail-horn');
  const dio = s.dio || {};
  const horn = s.horn;
  if (hornTile) {
    hornTile.hidden = !(horn && horn.enabled);
    if (!hornTile.hidden) {
      const held = (dio.commanded || {})[String(horn.channel)];
      const live = dio.do && dio.do[horn.channel];
      const el = setText('rail-horn-state', held ? 'ON' : 'off');
      if (el) el.style.color = held ? 'var(--hazard-ink)' : '';
      setText('rail-horn-note', !dio.enabled || !dio.comms_ok
        ? 'DIO down'
        : `DO${String(horn.channel).padStart(2, '0')} · coil ${live ? 'on' : 'off'}`);
    }
  }
}

// ---- loop health ----------------------------------------------------------
// work = time the bus thread spent doing things; period = the interval it
// actually achieved. Splitting them says whether a late tick is the controller
// or the bus. Amber once the worst tick runs past 1.5x the target period.
function renderLoopHealth(lp) {
  const work = document.getElementById('loop-work');
  if (!work || !lp) return;
  const period = document.getElementById('loop-period');
  const frames = document.getElementById('loop-frames');
  const starved = document.getElementById('loop-starved');

  if (lp.work_avg_ms === null || lp.work_avg_ms === undefined) {
    work.textContent = '–';
    period.textContent = 'ms work';
  } else {
    work.textContent = `${lp.work_avg_ms.toFixed(1)}/${lp.work_max_ms.toFixed(0)}`;
    period.textContent = `ms avg/max · ${lp.target_ms.toFixed(0)} target`;
    work.style.color = lp.work_max_ms > 1.5 * lp.target_ms ? 'var(--hazard-ink)' : '';
  }

  if (lp.frames_per_tick === null || lp.frames_per_tick === undefined) {
    frames.textContent = '–';
    starved.textContent = 'per tick';
  } else {
    frames.textContent = lp.frames_per_tick.toFixed(1);
    // A starved tick is one where the loop was armed but no new sensor frame
    // had arrived - the PID would be looking at a frame it already used.
    starved.textContent = lp.starved ? `per tick · ${lp.starved} starved`
                                     : 'per tick';
    frames.style.color = lp.starved > 5 ? 'var(--hazard-ink)' : '';
  }
}

// Every page polls this. Polling is READ-ONLY - it cannot keep the vehicle
// alive, because nothing here is latched: manual jogging is held by the
// /api/drive re-POST, which carries its own liveness.
//
// This used to add ?hb=1 on the page driving an auto run, opt-in so that a
// monitor page on a second screen could not hold a run open after the driving
// page was closed. If an autonomous mode returns, that opt-in property has to
// return with it rather than the heartbeat being made unconditional.
const listeners = [];
function onState(fn) { listeners.push(fn); }

async function poll() {
  try {
    const s = await apiGet('/api/state');
    // The pill answers the one question worth having on every page: is this
    // vehicle energised? The bus identity that used to be here ("socketcan:can0")
    // is a commissioning fact, not an operating one, and it moved to /monitor.
    //
    // Both failure states still OUTRANK it and still render red. A dead driver
    // or a missing bus would otherwise be invisible on six of the seven pages,
    // and "DISARMED" is a reassuring word to show while the bus is gone.
    const hw = s.health || {};
    if (hw.system_error) {
      setPill('DRIVER SILENT · ' + hw.system_detail, 'bad');
    } else if (!s.connected) {
      setPill(s.error || 'NO BUS', 'bad');
    } else {
      setPill(s.armed ? 'ARMED' : 'DISARMED', s.armed ? 'ok' : '');
    }

    for (const id of ['1', '2']) {
      const n = s.nodes[id];
      // Stale telemetry looks identical to live telemetry, so say so rather
      // than showing the last statusword as though it were current.
      const silent = ((hw.sources || {})['driver:' + id] || {}).ok === false;
      document.getElementById('rpm-' + id).textContent =
        n.rpm === null ? '–' : n.rpm;
      const st = document.getElementById('st-' + id);
      st.textContent = `node ${id} ${n.label}  `
        + (silent ? 'NOT ANSWERING'
           : n.statusword === null ? '—'
           : `0x${n.statusword.toString(16).toUpperCase().padStart(4, '0')} ${n.state}`)
        + (n.error_reg ? `  ERR 0x${n.error_reg.toString(16).padStart(2, '0')}` : '');
      st.className = 'st' + ((silent || n.error_reg || n.fault) ? ' bad' : '');
    }
    setText('setpoint', `${s.target.left} / ${s.target.right}`);
    // Watchdog and loop health live on /monitor only now. Both are guarded,
    // because an unguarded lookup for a tile that moved throws inside poll()
    // and silently freezes EVERY page's telemetry.
    setText('wd', s.armed ? s.watchdog_s.toFixed(1) : '–');
    renderLoopHealth(s.loop);
    renderRail(s);

    // Stop reasons now arrive through the server event log, which survives a
    // reload; logging them here too would just double every line.
    syncEvents(s.event_seq);

    showRfid(s.rfid);
    listeners.forEach(fn => fn(s));
  } catch (e) {
    setPill('server unreachable', 'bad');
  }
}

setInterval(poll, 200);
poll();


// Operator panel state. Shared by /manual and /auto: both need to show that the
// vehicle can be armed and started from the physical buttons, and above all
// that a latched fault is why nothing is happening.
onState(s => {
  const strip = document.getElementById('panel-strip');
  if (!strip) return;
  const p = s.panel || {};
  strip.hidden = !p.enabled;
  if (!p.enabled) return;

  const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
  // null selector = the DI scan has not produced a trusted image yet, which is
  // not the same as MANUAL and must not be shown as it.
  set('pn-sel', p.selector ? p.selector.toUpperCase() : 'unknown');
  const a = p.last_action;
  set('pn-act', a ? `${a.what} (${a.source})` : '–');

  const f = document.getElementById('pn-fault');
  if (f) {
    f.hidden = !p.fault;
    f.textContent = p.fault ? `FAULT — ${p.fault} · press Reset` : '';
  }
});
