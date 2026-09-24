// Blind run page: SETS an encoder-only plan. Nothing here can start the wheels -
// PB Start runs the plan with the selector in MANUAL, and Reset stops it.

const FIELDS = {
  straight: [['distance_m', 'Distance m', 1]],
  arc: [['radius_m', 'Radius m', 1], ['angle_deg', 'Angle °', 90]],
  pivot: [['angle_deg', 'Angle °', 90]],
  pulses: [['left', 'Left counts', 1080000], ['right', 'Right counts', 1080000]],
};
const segBox = document.getElementById('blind-segments');
const errEl = document.getElementById('blind-error');
const fx = (v, d = 3) => (v === null || v === undefined) ? '–' : Number(v).toFixed(d);

function addSegment(kind = 'straight') {
  if (segBox.children.length >= window.BLIND_MAX_SEGMENTS) return;
  const row = document.createElement('div');
  row.className = 'blind-row blind-seg';
  const sel = document.createElement('select');
  sel.className = 'blind-kind';
  sel.setAttribute('aria-label', 'Segment type');
  Object.keys(FIELDS).forEach(k => {
    const o = document.createElement('option');
    o.value = k; o.textContent = k;
    sel.appendChild(o);
  });
  sel.value = kind;
  const inputs = document.createElement('span');
  inputs.className = 'blind-fields';
  const render = () => {
    inputs.replaceChildren();
    FIELDS[sel.value].forEach(([key, label, dflt]) => {
      const l = document.createElement('label');
      l.textContent = label + ' ';
      const i = document.createElement('input');
      i.type = 'number'; i.step = 'any'; i.dataset.key = key; i.value = dflt;
      l.appendChild(i);
      inputs.appendChild(l);
    });
  };
  sel.addEventListener('change', render);
  const rm = document.createElement('button');
  rm.type = 'button'; rm.className = 'danger'; rm.textContent = 'Remove';
  rm.addEventListener('click', () => row.remove());
  render();
  row.append(sel, inputs, rm);
  segBox.appendChild(row);
}

function readPlan() {
  const segments = [...segBox.querySelectorAll('.blind-seg')].map(row => {
    const seg = {kind: row.querySelector('.blind-kind').value};
    row.querySelectorAll('input').forEach(i => { seg[i.dataset.key] = Number(i.value); });
    return seg;
  });
  const unit = document.getElementById('blind-speed-unit').value;
  return {segments, speed: {[unit]: Number(document.getElementById('blind-speed').value)}};
}

document.getElementById('blind-add').addEventListener('click', () => addSegment());
document.getElementById('blind-set').addEventListener('click', async () => {
  errEl.textContent = '';
  try { showPlan((await api('/api/blind/plan', readPlan())).plan); }
  catch (e) { errEl.textContent = e.message; }
});
document.getElementById('blind-clear').addEventListener('click', async () => {
  errEl.textContent = '';
  try { await api('/api/blind/clear'); showPlan(null); }
  catch (e) { errEl.textContent = e.message; }
});

function describe(kind, spec) {
  return kind + ' ' + Object.entries(spec || {}).map(([k, v]) => `${k}=${v}`).join(' ');
}
function row(values) {
  const tr = document.createElement('tr');
  values.forEach(v => { const td = document.createElement('td'); td.textContent = v; tr.appendChild(td); });
  return tr;
}

// Rebuilt only when the content changes, so a table being read does not
// flicker five times a second.
let shownPlan, shownResults;
function showPlan(plan) {
  const key = JSON.stringify(plan);
  if (key === shownPlan) return;
  shownPlan = key;
  setText('blind-plan-note', plan
    ? `${plan.segments.length} segment(s), ~${fx(plan.duration_s, 1)} s, `
      + `${fx(plan.ref_rpm, 0)} r/min reference (${fx(plan.speed_mps, 3)} m/s)`
    : 'none set');
  document.querySelector('#blind-plan tbody').replaceChildren(
    ...(plan ? plan.segments : []).map((s, i) => row([
      i + 1, describe(s.kind, s.spec), `${s.counts[0]} / ${s.counts[1]}`,
      `${fx(s.left_m)} / ${fx(s.right_m)}`, `${fx(s.rpm[0], 0)} / ${fx(s.rpm[1], 0)}`,
      fx(s.duration_s, 1)])));
}

function showResults(results) {
  const key = JSON.stringify(results);
  if (key === shownResults) return;
  shownResults = key;
  document.querySelector('#blind-results tbody').replaceChildren(
    ...[...results].reverse().map(r => row([
      r.segment, describe(r.kind, r.spec),
      `${fx(r.commanded_distance_m, 4)} m · ${fx(r.commanded_heading_deg, 2)}°`,
      `${fx(r.encoder_distance_m, 4)} m · ${fx(r.encoder_heading_deg, 2)}°`,
      `${r.target_left} / ${r.target_right}`, `${r.final_left} / ${r.final_right}`,
      `${r.error_left} / ${r.error_right}`])));
}

function statusOf(s) {
  const b = s.blind || {}, run = b.run;
  if (b.active && run) {
    return [`BLIND RUN - ${run.phase.toUpperCase()}`,
            `segment ${run.segment} of ${run.segments} · Reset on the panel stops it`];
  }
  if (b.starting_in !== null && b.starting_in !== undefined) {
    return [`STARTING - ${fx(b.starting_in, 1)} s`, 'Reset on the panel cancels'];
  }
  const last = run && run.phase === 'aborted' ? `Last run stopped: ${run.reason}`
             : run && run.phase === 'done' ? 'Last run complete' : null;
  if (!b.plan) return ['NO PLAN SET', last || 'Build a move and press SET'];
  if (s.mode !== 'manual' || !s.armed) {
    return ['PLAN SET - SELECTOR TO MANUAL', 'Start runs the plan only while armed in MANUAL'];
  }
  return ['PLAN SET - PRESS START ON THE PANEL', last || 'Reset on the panel stops a run'];
}

onState(s => {
  const b = s.blind || {}, run = b.run;
  const [status, note] = statusOf(s);
  setText('blind-status', status);
  setText('blind-status-note', note);
  const busy = !!b.active || (b.starting_in !== null && b.starting_in !== undefined);
  document.getElementById('blind-set').disabled = busy;
  document.getElementById('blind-clear').disabled = busy;
  showPlan(b.plan || null);
  setText('br-phase', run ? run.phase : '–');
  setText('br-seg', run ? `segment ${run.segment} of ${run.segments} · ${run.kind}` : 'segment –');
  setText('br-left', run ? fx(run.progress_m[0]) : '–');
  setText('br-left-t', run ? `of ${fx(run.target_m[0])} m` : 'of – m');
  setText('br-right', run ? fx(run.progress_m[1]) : '–');
  setText('br-right-t', run ? `of ${fx(run.target_m[1])} m` : 'of – m');
  setText('br-pose', run ? `${fx(run.pose.x_m)} / ${fx(run.pose.y_m)} · ${fx(run.pose.heading_deg, 1)}` : '–');
  setText('br-speed', run ? fx(run.speed_mps) : '–');
  // The MLS IMU, from the same poll the rail and /monitor use. Greyed when
  // stale, like everywhere else - a frozen number must not look live.
  const tile = document.getElementById('br-imu-tile');
  const imu = s.imu;
  if (tile && imu) {
    const gz = (imu.gyro_dps || [])[2];
    setText('br-gyro', gz === null || gz === undefined ? '–' : fx(gz, 2));
    setText('br-gyro-note', imu.stale ? '°/s · STALE' : '°/s, recorded only');
    tile.classList.toggle('stale', !!imu.stale);
  }
  showResults(b.results || []);
  setText('blind-log', b.log_dir || '—');
});

addSegment();
