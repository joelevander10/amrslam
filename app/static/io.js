// I/O page: render the digital input and output images as lamps.
//
// Read-only by construction - there is no write endpoint to call, and this
// file deliberately contains no click handlers. Adding one would need an
// arm-state interlock and event logging on the server side first.

const grid = document.getElementById('io-grid');

function paint(prefix, bits, names) {
  for (let i = 0; i < 16; i++) {
    const el = document.getElementById(`${prefix}-${i}`);
    if (el) el.classList.toggle('on', !!(bits && bits[i]));
    // Names come from the profile via the snapshot, so filling them in is a
    // profile edit and a reload - no template change.
    const nm = document.getElementById(`${prefix}-n-${i}`);
    if (nm && names && names[i] !== undefined) nm.textContent = names[i];
  }
}

onState(s => {
  const d = s.dio;
  const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
  if (!d) return;

  paint('di', d.di, d.di_names);
  paint('do', d.do, d.do_names);

  // Outputs this software is driving. Two facts on one line: that the channel
  // is claimed at all, and which way it is being held - so a horn that is off
  // between runs cannot be confused with a horn nothing is wired to.
  const cmd = d.commanded || {};
  for (let i = 0; i < 16; i++) {
    const el = document.getElementById(`do-c-${i}`);
    if (!el) continue;
    const held = cmd[String(i)];
    el.textContent = held === undefined ? '' : (held ? 'held on' : 'held off');
    el.classList.toggle('on', held === true);
  }

  // Three states worth telling apart, because they need different actions:
  // disabled (nothing to do), never connected (address or cable), and
  // connected-but-stale (module wedged, or the switch dropped it).
  let text, colour;
  if (!d.enabled)            { text = 'off';       colour = ''; }
  else if (!d.connected)     { text = 'down';      colour = 'var(--stop)'; }
  else if (!d.comms_ok)      { text = 'stale';     colour = 'var(--hazard-ink)'; }
  else                       { text = 'ok';        colour = ''; }
  const link = document.getElementById('io-link');
  if (link) { link.textContent = text; link.style.color = colour; }

  set('io-detail', d.detail || '—');
  set('io-scans', d.scans);
  set('io-age', d.rx_age_s === null || d.rx_age_s === undefined
                ? 'never scanned' : (d.rx_age_s * 1000).toFixed(0) + ' ms ago');
  const err = document.getElementById('io-errors');
  if (err) { err.textContent = d.errors; err.style.color = d.errors ? 'var(--hazard-ink)' : ''; }

  // Stale must not look like off. Dimming the whole grid is the point: a dead
  // module otherwise renders as 32 contentedly dark lamps.
  if (grid) grid.classList.toggle('stale', d.enabled && !d.comms_ok);
});
