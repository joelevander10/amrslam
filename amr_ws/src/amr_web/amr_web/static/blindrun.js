// Blind-run form on /commissioning: one move per plan, backend PV or PP.
// The page only SETS numbers. The job runs on a fresh physical Start under MANUAL.
// Pure helpers live on BR (a `var`, so the test harness can reach them); the DOM
// wiring runs only when the page's form is present.
var BR = (function () {
  const round = (v, dp) => Number(v.toFixed(dp));

  // form values -> { plan, errors }. caps come from /api/commissioning/capabilities.
  function buildPlan(f, caps) {
    const errors = [];
    const n = (v, name) => {
      const x = Number(v);
      if (v === '' || v === null || v === undefined || !isFinite(x)) { errors.push(`${name} must be a number`); return NaN; }
      return x;
    };
    const speed = n(f.speed, 'speed');
    const cap = f.backend === 'pp' ? caps.pp_max_speed_mps : caps.max_speed_mps;
    if (isFinite(speed) && !(speed >= 0.05 && speed <= cap + 1e-9)) errors.push(`speed must be 0.05–${cap} m/s`);
    if (f.backend !== 'pv' && f.backend !== 'pp') errors.push('backend must be PV or PP');
    if (f.backend === 'pp' && !caps.pp_available) errors.push(`PP not available: ${caps.pp_reason || 'locked'}`);
    let seg = null, label = '';
    if (f.kind === 'straight') {
      const d = n(f.distance, 'distance');
      if (isFinite(d) && !(d > 0 && d <= caps.max_distance_m)) errors.push(`distance must be 0–${caps.max_distance_m} m`);
      const sign = f.direction === 'reverse' ? -1 : 1;
      seg = { kind: 'straight', distance_m: round(sign * d, 4) };
      label = `straight-${f.direction === 'reverse' ? 'rev' : 'fwd'}-${d}m`;
    } else if (f.kind === 'rotate') {
      const a = n(f.angle, 'angle');
      if (isFinite(a) && !(a > 0 && a <= 720)) errors.push('angle must be 0–720°');
      const sign = f.direction === 'cw' ? -1 : 1;  // + counter-clockwise, as core/blindrun.py
      seg = { kind: 'pivot', angle_deg: round(sign * a, 3) };
      label = `rotate-${f.direction === 'cw' ? 'cw' : 'ccw'}-${a}deg`;
    } else if (f.kind === 'arc') {
      const a = n(f.angle, 'angle'), r = n(f.radius, 'radius');
      if (isFinite(a) && !(a > 0 && a <= 360)) errors.push('angle must be 0–360°');
      if (isFinite(r) && !(r >= caps.min_arc_radius_m && r <= 50)) errors.push(`radius must be ${caps.min_arc_radius_m.toFixed(3)}–50 m (tighter: use rotate)`);
      const sign = f.side === 'right' ? -1 : 1;  // + turns left; always forwards
      seg = { kind: 'arc', radius_m: round(sign * r, 4), angle_deg: round(a, 3) };
      label = `arc-${f.side === 'right' ? 'right' : 'left'}-${a}deg-r${r}m`;
    } else {
      errors.push('pick a move type');
    }
    const id = `${label}-${f.backend}`.replace(/[^A-Za-z0-9_.-]/g, '_').replace(/^[^A-Za-z0-9]+/, '').slice(0, 64);
    const plan = { id, backend: f.backend, speed: { speed_mps: round(speed, 3) }, segments: seg ? [seg] : [] };
    return { plan, errors };
  }

  // v^2 / 2a on the blind ramp: the distance a normal (ramped) stop takes at `speed`.
  function stopDistance(speed, decel) { return decel > 0 ? speed * speed / (2 * decel) : NaN; }

  // commanded pose change for display, from the planned segment the server returns
  function commanded(planned) {
    const s = planned && planned.segments && planned.segments[0];
    if (!s || !s.commanded) return null;
    return { dx_mm: 1000 * s.commanded.dx_m, dy_mm: 1000 * s.commanded.dy_m, heading_deg: s.commanded.heading_deg };
  }

  return { buildPlan, stopDistance, commanded };
})();

(function () {
  if (typeof document === 'undefined' || !document.getElementById('br-form')) return;
  const $ = id => document.getElementById(id);
  let caps = null, kind = 'straight', lastPlanned = null;
  const PHASE_LEVEL = { PREPARED: 'warn', RUNNING: 'warn', SETTLING: 'warn', ABORTED: 'bad' };

  function form() {
    return {
      kind, backend: $('br-backend').value, speed: $('br-speed').value,
      direction: kind === 'straight' ? $('br-dir').value : $('br-rot').value,
      side: $('br-side').value, distance: $('br-distance').value,
      angle: kind === 'rotate' ? $('br-rot-angle').value : $('br-arc-angle').value, radius: $('br-radius').value,
    };
  }

  function setKind(k) {
    kind = k;
    document.querySelectorAll('#br-kinds button').forEach(b => b.classList.toggle('on', b.dataset.kind === k));
    ['straight', 'rotate', 'arc'].forEach(x => { $(`br-${x}`).hidden = x !== k; });
    refresh();
  }

  function refresh() {
    if (!caps) return;
    const f = form();
    const { errors } = BR.buildPlan(f, caps);
    const v = Number(f.speed);
    const warn = [];
    if (isFinite(v) && v > caps.field_check_above_mps) {
      warn.push(`Above ${caps.field_check_above_mps} m/s: confirm the nanoScan3/FX3 protective field is sized for this speed. ` +
        `A ramped stop takes about ${num(BR.stopDistance(v, caps.decel_mps2), 2)} m, before any reaction time.`);
    }
    $('br-warn').hidden = !warn.length;
    $('br-warn').textContent = warn.join(' ');
    $('br-errors').textContent = errors.join(' · ');
    const pp = $('br-backend').querySelector('option[value="pp"]');
    pp.disabled = !caps.pp_available;
    pp.textContent = caps.pp_available ? 'PP — profile position (in the drive)' : `PP — unavailable: ${caps.pp_reason || 'locked'}`;
    if (!caps.pp_available && $('br-backend').value === 'pp') $('br-backend').value = 'pv';
    $('br-speed').max = String(f.backend === 'pp' ? caps.pp_max_speed_mps : caps.max_speed_mps);
    $('btn-plan').disabled = !!errors.length || !$('br-check').checked;
  }

  function showPlanned(p) {
    lastPlanned = p;
    if (!p) { $('planned').innerHTML = '<div class="none">no plan held</div>'; return; }
    const s = p.segments[0], c = s.commanded;
    const rows = [
      ['backend', p.backend.toUpperCase(), ''],
      ['wheel counts L / R', `${esc(s.counts[0])} / ${esc(s.counts[1])}`, 'driver terms'],
      ['wheel travel L / R', `${num(s.left_m, 4)} / ${num(s.right_m, 4)}`, 'm'],
      ['commanded dx / dy', `${num(1000 * c.dx_m, 1)} / ${num(1000 * c.dy_m, 1)}`, 'mm'],
      ['commanded heading', num(c.heading_deg, 2), '°'],
      ['faster wheel', num(s.dom_rpm, 0), 'motor r/min'],
      ['planned time', num(s.duration_s, 1), 's'],
    ];
    if (p.pp) {
      const w = x => `${esc(x.velocity_rpm)} r/min · ${esc(x.accel_rpm_s)}/${esc(x.decel_rpm_s)} r/min/s`;
      rows.push(['pp left', w(p.pp.left), ''], ['pp right', w(p.pp.right), ''],
        ['pp ratio rounding', num(p.pp.ratio_error * 100, 3), '%']);
    }
    $('planned').innerHTML = rows.map(([k, v, u]) =>
      `<div class="row"><span class="k">${esc(k)}</span><span class="v">${v}<i>${esc(u)}</i></span></div>`).join('');
  }

  async function hold() {
    const { plan, errors } = BR.buildPlan(form(), caps);
    if (errors.length) { log(errors.join('; '), 'bad'); return; }
    const { status, data } = await api('/api/commissioning/plan', { plan });
    log(data.message, status === 200 ? '' : 'bad');
    if (status === 200 && data.planned) showPlanned(data.planned);
  }

  async function loadCaps() {
    const { status, data } = await apiGet('/api/commissioning/capabilities');
    if (status === 200 && data) {
      caps = data;
      $('br-speed-cap').textContent = `max ${caps.max_speed_mps} m/s (PP ${caps.pp_max_speed_mps})`;
      $('br-radius').min = String(caps.min_arc_radius_m);
      $('br-radius-min').textContent = `≥ ${caps.min_arc_radius_m.toFixed(3)} m`;
      refresh();
    }
  }

  async function tick() {
    const { status, data } = await apiGet('/api/commissioning');
    if (status === 200 && data) {
      tiles('job', [
        ['Phase', data.phase_name, data.plan_id || '—', PHASE_LEVEL[data.phase_name], 'key'],
        ['Backend', (data.backend || 'pv').toUpperCase(), data.run_id || '—'],
        ['Segment', `${data.segment}/${data.segments}`, data.kind || '—'],
        ['Progress L', num(data.progress_left_m, 3), 'm'],
        ['Progress R', num(data.progress_right_m, 3), 'm'],
        ['Speed', num(data.speed_mps, 2), 'm/s'],
        ['Heading', num(data.heading_deg, 1), '° encoder'],
        ['Gyro heading', num(data.gyro_heading_deg, 1), '° integrated'],
      ]);
      $('job-reason').textContent = data.reason || '';
      $('job-evidence').innerHTML = data.results_path ? `evidence: <code>${esc(data.results_path)}</code>` : '';
      if (data.results_path && data.results_path !== tick.lastPath) { tick.lastPath = data.results_path; loadHistory(); }
    } else if (status === 200) tiles('job', [['Phase', '–', 'no commissioning node (supervised base only)', 'warn']]);
    setTimeout(tick, 500);
  }

  const fmt = (v, dp) => (typeof v === 'number' && isFinite(v)) ? v.toFixed(dp) : '–';
  function moveText(c) {
    if (!c) return 'multi-segment';
    const sp = c.spec || {};
    if (c.kind === 'straight') return `straight ${fmt(sp.distance_m, 3)} m`;
    if (c.kind === 'pivot') return `rotate ${fmt(sp.angle_deg, 1)}°`;
    if (c.kind === 'arc') return `arc ${fmt(sp.angle_deg, 1)}° r ${fmt(sp.radius_m, 3)} m`;
    return c.kind;
  }

  async function loadHistory() {
    const { status, data } = await apiGet('/api/commissioning/history');
    if (status !== 200 || !Array.isArray(data)) return;
    const sel = $('ms-run'), keep = sel.value;
    sel.innerHTML = data.filter(r => !r.error_reading).map(r =>
      `<option value="${esc(r.run)}">${esc(r.measured ? '✓ ' : '')}${esc(r.backend.toUpperCase())} · ${esc(moveText(r.commanded))} · ${esc(r.phase)} · ${esc(r.run)}</option>`).join('');
    if (keep && data.some(r => r.run === keep)) sel.value = keep;
    showCommanded();
    $('history').innerHTML = '<div class="row br-hist head"><span>backend · move</span><span>speed</span><span>commanded dx/dy/hdg</span><span>measured</span><span>error</span></div>' +
      (data.length ? data.map(r => r.error_reading
        ? `<div class="row br-hist"><span class="k">${esc(r.run)}</span><span class="n">unreadable: ${esc(r.error_reading)}</span></div>`
        : `<div class="row br-hist"><span class="k">${esc(r.backend.toUpperCase())} · ${esc(moveText(r.commanded))}<br><i class="n">${esc(r.phase)}${r.reason ? ' · ' + esc(r.reason) : ''}</i></span>` +
          `<span class="v">${fmt(r.speed_mps, 2)}</span>` +
          `<span class="v">${r.commanded ? `${fmt(r.commanded.dx_mm, 0)} / ${fmt(r.commanded.dy_mm, 0)} / ${fmt(r.commanded.heading_deg, 1)}°` : '–'}</span>` +
          `<span class="v">${r.measured ? `${fmt(r.measured.dx_mm, 0)} / ${fmt(r.measured.dy_mm, 0)} / ${fmt(r.measured.heading_deg, 1)}°` : '–'}</span>` +
          `<span class="v">${r.error ? `${fmt(r.error.dx_mm, 0)} / ${fmt(r.error.dy_mm, 0)} / ${fmt(r.error.heading_deg, 2)}°` : '–'}</span></div>`).join('')
        : '<div class="none">no runs yet</div>');
    loadHistory.rows = data;
  }

  function showCommanded() {
    const r = (loadHistory.rows || []).find(x => x.run === $('ms-run').value);
    const c = r && r.commanded;
    $('ms-commanded').textContent = c
      ? `commanded: dx ${fmt(c.dx_mm, 0)} mm · dy ${fmt(c.dy_mm, 0)} mm · heading ${fmt(c.heading_deg, 1)}°`
      : 'commanded: –';
  }

  async function saveMeasurement() {
    const body = { run: $('ms-run').value, dx_mm: $('ms-dx').value, dy_mm: $('ms-dy').value,
      heading_deg: $('ms-hdg').value, note: $('ms-note').value };
    const { status, data } = await api('/api/commissioning/measurement', body);
    log(data.message, status === 200 ? '' : 'bad');
    if (status === 200) loadHistory();
  }

  document.querySelectorAll('#br-kinds button').forEach(b => { b.onclick = () => setKind(b.dataset.kind); });
  ['br-backend', 'br-speed', 'br-dir', 'br-rot', 'br-side', 'br-distance', 'br-rot-angle', 'br-arc-angle', 'br-radius', 'br-check']
    .forEach(id => { $(id).oninput = refresh; $(id).onchange = refresh; });
  $('btn-plan').onclick = hold;
  $('btn-clear').onclick = () => api('/api/commissioning/clear').then(r => log(r.data.message));
  $('btn-json').onclick = async () => {
    let plan; try { plan = JSON.parse($('plan').value); } catch (e) { log('plan is not valid JSON: ' + e.message, 'bad'); return; }
    const { status, data } = await api('/api/commissioning/plan', { plan });
    log(data.message, status === 200 ? '' : 'bad');
    if (status === 200 && data.planned && data.planned.segments.length === 1) showPlanned(data.planned);
  };
  $('ms-run').onchange = showCommanded;
  $('btn-measure').onclick = saveMeasurement;
  setKind('straight');
  loadCaps();
  setInterval(loadCaps, 2000);  // PP availability follows the drive owner
  loadHistory();
  tick();
})();
