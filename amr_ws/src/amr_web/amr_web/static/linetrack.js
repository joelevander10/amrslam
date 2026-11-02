// The MLS line strip (_line_track.html): Manual, Run tracked and Home's live tiles, so the rail's
// shared amr.js does not reference ids other pages lack.
// LCP1..3 on a bar scaled to the profile's sensor_max_mm. Older than 1 s = no reading.
function lineTrack(t) {
  const box = document.getElementById('line-track');
  const live = !!t && t.age_s <= 1.0;
  box.classList.toggle('stale', !live);
  const max = (t && t.sensor_max_mm) || 100;
  document.getElementById('lt-min').textContent = `−${max}`;
  document.getElementById('lt-max').textContent = `+${max}`;
  document.getElementById('lt-status').textContent = !t ? 'no reading (drive_node not publishing)'
    : !live ? `stale ${t.age_s.toFixed(1)} s`
    : `${t.nlcp_label} · level ${t.track_level}/7 · ${t.line_good ? 'good' : 'weak'} · ${t.polarity}` + (t.marker ? ` · marker ${t.marker}` : '');
  box.querySelectorAll('.lt-dot').forEach(d => {
    const i = +d.dataset.i, on = live && t.valid[i];
    d.hidden = !on;
    if (on) d.style.left = `${50 + 50 * Math.max(-1, Math.min(1, t.lcp_mm[i] / max))}%`;
  });
  document.getElementById('lt-lcps').innerHTML = [0, 1, 2].map(i => {
    const on = live && t.valid[i];
    return `<span class="${on ? '' : 'muted'}">LCP${i + 1} <b>${on ? (t.lcp_mm[i] > 0 ? '+' : '') + t.lcp_mm[i] + ' mm' : '–'}</b></span>`;
  }).join('');
}

// RFID: the reader link (2 Hz heartbeat) and the last tag passes, newest first. The newest
// is lit for RF_FRESH_S so a pass is visible while driving. Missing data is not an error:
// the offline demo server has no reader.
// The big tag and the status line show a tag only while one is UNDER the reader: cleared
// after RF_HOLD_S with no read (2026-10-07: a stale 1170 read as "just passed" during a run).
// The reader's own tag_age_s (since its last read of any tag) plus the heartbeat's age.
const RF_FRESH_S = 3.0;
const RF_HOLD_S = 1.0;
// The one rule for "a tag is under the reader now", shared by this strip and Home's tile.
function rfidReading(r) {
  const link = r && r.link, tags = (r && r.tags) || [];
  const up = !!link && link.age_s <= 2.0 && link.comms_ok;
  const last = tags[0];
  const reading = up && !!last && link.tag_age_s >= 0 && link.tag_age_s + link.age_s <= RF_HOLD_S;
  const why = !link ? 'no reader data' : link.age_s > 2.0 ? `reader silent ${link.age_s.toFixed(1)} s`
    : !link.comms_ok ? 'reader link down' : '';
  return { link, tags, up, last, reading, tag: reading ? last.tag : null, why };
}
// MLS markers (mls-marker-plan 6.1): the same "up / reading now / last" rule for Home's
// Marker tile. A marker passes under the sensor in ~0.2 s, so the code stays MK_HOLD_S.
const MK_HOLD_S = 2.0;
function markerReading(m) {
  const link = m && m.link, events = (m && m.events) || [];
  const up = !!link && link.age_s <= 2.5 && link.markers_ok;
  const last = events[0];
  const code = up && !!last && last.age_s <= MK_HOLD_S ? last.code : null;
  const status = link ? link.status || '' : '';
  const misconfigured = status.startsWith('misconfigured') || status.startsWith('unverified');
  const why = !link ? 'no marker data' : link.age_s > 2.5 ? `marker stream silent ${link.age_s.toFixed(1)} s`
    : misconfigured ? 'sensor not set up: read_mls set-markers'
    : status === 'no stream (sdo)' ? 'MLS on SDO fallback: markers not read'
    : status === 'off' ? 'markers off in the profile' : status || 'markers down';
  return { link, events, up, last, code, misconfigured, why };
}
function rfidTags(r) {
  const box = document.querySelector('.lt-rfid');
  if (!box) return;
  const { link, tags, up, last, reading } = rfidReading(r);
  box.classList.toggle('down', !up);
  // RSSI of the latest read, and a flag when the reader's own configuration (read back on
  // connect) is not what the profile expects: a swapped or factory-reset unit. Display only.
  const mism = (link && link.config_mismatch) || [];
  const status = document.getElementById('rf-status');
  status.textContent = (!link ? 'no reader data (rfid_node not publishing)'
    : link.age_s > 2.0 ? `reader silent ${link.age_s.toFixed(1)} s`
    : !link.comms_ok ? 'reader link down'
    : reading ? `link ok · reading ${last.tag}${rssiText(last.rssi_dbm)} · ${link.seq} read`
    : `link ok · no tag · ${link.seq} read`) + (mism.length ? ' · reader config ≠ profile' : '');
  status.title = [link && link.reader, ...mism].filter(Boolean).join('\n');
  const big = box.querySelector('.rf-last');  // Run tracked only: optional
  if (big) {
    big.classList.toggle('fresh', reading);
    big.querySelector('b').textContent = reading ? last.tag : '––';
    big.querySelector('span').textContent = reading ? `reading · ${clock(last.wall)}` : 'no tag under the reader';
  }
  document.getElementById('rf-tags').innerHTML = tags.length
    ? tags.map((t, k) => `<span class="rf-tag${k === 0 && t.age_s <= RF_FRESH_S ? ' fresh' : ''}" title="pass #${t.seq}${rssiText(t.rssi_dbm)}"><b>${esc(t.tag)}</b> ${clock(t.wall)}</span>`).join('')
    : '<span class="none">pass a tag under the reader</span>';
}
function rssiText(dbm) { return typeof dbm === 'number' ? ` · ${dbm.toFixed(1)} dBm` : ''; }
function ago(s) { return s < 60 ? `${Math.round(s)} s ago` : `${Math.round(s / 60)} min ago`; }
function clock(wall) { const d = new Date(wall * 1000); return d.toTimeString().slice(0, 8); }
