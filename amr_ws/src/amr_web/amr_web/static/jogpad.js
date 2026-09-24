// Shared jog component (unified plan §6.3). Mount with jogpad(container).
//
// Held, not latched: a physical press (pointer down on a pad button, or a key
// down) obtains a server jog session, then refreshes it every 100 ms while the
// input is held. Each refresh carries the single-use ticket the server issued
// with the previous one; releasing the input releases the session BEFORE a zero
// is sent, and anything that could mean "the operator is no longer holding
// this" - blur, pagehide, hidden tab, pointer cancel, a focused text field -
// releases too. There is no timer on the server: if the refreshes stop, the
// command dies 0.2 s later on the robot regardless of what this page does.
function jogpad(root, opts) {
  const o = Object.assign({ compact: false }, opts || {});
  const owner = (crypto.randomUUID ? crypto.randomUUID() : String(Math.random())).slice(0, 32);
  const speeds = [0.10, 0.20, 0.30, 0.40];  // jog.V_MAX 0.40 (2026-09-19)
  // Diagonals drive like the pendant: the FAST wheel at the selected speed, the slow one at
  // TURN_RATIO of it (no wheel above the selection). Spins: +30 % on the 2026-09-18 rule.
  const TRACK_M = 0.487, TURN_RATIO = 0.75, SPIN_MAX = 0.39, SPIN_PER_V = 1.95;
  const CELLS = [
    ['fl', '↖', 'Fwd-L', ''], ['f', '▲', 'Fwd', '↑ / W'], ['fr', '↗', 'Fwd-R', ''],
    ['l', '◀', 'Left', '← / A'], ['stop', '■', 'Stop', 'Space'], ['r', '▶', 'Right', '→ / D'],
    ['bl', '↙', 'Rev-L', ''], ['b', '▼', 'Rev', '↓ / S'], ['br', '↘', 'Rev-R', ''],
  ];
  root.innerHTML = `
    <div class="jog">
      <div class="pad jog-grid${o.compact ? ' compact' : ''}">
        ${CELLS.map(([dir, glyph, name, key]) => `<button class="dir${dir === 'stop' ? ' stopbtn' : ''}" data-dir="${dir}" title="${name}${key ? ' — ' + key : ''}">
          <span class="glyph">${glyph}</span><span class="name">${name}</span><span class="key">${key || '&nbsp;'}</span></button>`).join('')}
      </div>
      <div class="jog-foot">
        <label class="field">Speed <select class="jog-speed">${speeds.map(s => `<option value="${s}" ${s === 0.2 ? 'selected' : ''}>${s.toFixed(2)} m/s</option>`).join('')}</select></label>
        <div class="tel"><span>Jog</span><b class="jog-status">not held</b><i>held, not latched · release = stop</i></div>
      </div>
      <p class="legend keys">Hold a button or <kbd>↑</kbd> <kbd>↓</kbd> <kbd>←</kbd> <kbd>→</kbd> (<kbd>W</kbd> <kbd>A</kbd> <kbd>S</kbd> <kbd>D</kbd>); <kbd>Space</kbd> stops.
        Works only with the physical selector in <b>MANUAL</b> and the supervisor allowing manual control.</p>
    </div>`;
  // Presentational only: which cell is being held. Session logic below does not read it.
  const cells = root.querySelectorAll('.jog-grid button');
  const lit = dir => cells.forEach(b => b.classList.toggle('active', b.dataset.dir === dir));
  const flashStop = () => { lit('stop'); setTimeout(() => { if (!held && !pending) lit(null); }, 150); };
  const status = root.querySelector('.jog-status');
  const speedSel = root.querySelector('.jog-speed');
  const DIRS = { f: [1, 0], b: [-1, 0], l: [0, 1], r: [0, -1], fl: [1, 1], fr: [1, -1], bl: [-1, -1], br: [-1, 1] };
  let held = null;     // {dir, session, ticket, seq, timer}
  let pending = null;  // {dir} while /press is in flight; a release clears it (review R01)
  let releasing = false;

  function body(dir) {
    const v = parseFloat(speedSel.value);
    const [a, b] = DIRS[dir];
    if (a === 0) return { v: 0, w: b * Math.min(SPIN_MAX, v * SPIN_PER_V) };
    if (b === 0) return { v: a * v, w: 0 };
    const slow = v * TURN_RATIO;
    return { v: a * (v + slow) / 2, w: b * (v - slow) / TRACK_M };
  }
  // The physical input is recorded BEFORE the press request goes out, so a release that
  // happens while it is in flight has something to cancel. A press whose input was
  // released meanwhile gives its session straight back and never refreshes it.
  async function press(dir) {
    if (held || pending || releasing) return;
    const token = { dir };
    pending = token;
    lit(dir);
    let st, data;
    try {
      ({ status: st, data } = await api('/api/manual/press', { owner }));
    } catch (e) {
      if (pending === token) { pending = null; lit(null); }
      status.textContent = 'press failed (no connection)';
      return;
    }
    if (pending !== token) {
      if (st === 200) api('/api/manual/release', { session: data.session }).catch(() => {});
      return;
    }
    pending = null;
    if (st !== 200) { lit(null); status.textContent = data.message || 'refused'; return; }
    held = { dir, session: data.session, ticket: data.ticket, seq: 0, timer: null };
    status.textContent = `holding ${dir}`;
    refresh();
  }
  async function refresh() {
    if (!held) return;
    const h = held;
    const b = body(h.dir);
    h.seq += 1;
    let st, data;
    const sent = Date.now();  // fixed 100 ms cadence from send to send, not from reply to send
    try {
      ({ status: st, data } = await api('/api/manual/refresh', { session: h.session, ticket: h.ticket, seq: h.seq, v: b.v, w: b.w }));
    } catch (e) {
      if (held === h) { held = null; lit(null); status.textContent = 'stopped: no connection'; }
      return;  // the robot drops the command 0.2 s after the last refresh by itself
    }
    if (held !== h) return;
    if (st !== 200) { status.textContent = `stopped: ${data.message}`; held = null; lit(null); return; }
    h.ticket = data.ticket;
    status.textContent = `holding ${h.dir}: v ${data.v.toFixed(2)} m/s, w ${data.w.toFixed(2)} rad/s`;
    h.timer = setTimeout(refresh, Math.max(0, 100 - (Date.now() - sent)));
  }
  async function release(why) {
    lit(null);
    if (pending) { pending = null; status.textContent = `released (${why})`; }
    if (!held) return;
    const h = held; held = null; releasing = true;
    if (h.timer) clearTimeout(h.timer);
    try { await api('/api/manual/release', { session: h.session }); } catch (e) { /* robot expires it */ } finally { releasing = false; }
    status.textContent = `released (${why})`;
  }
  cells.forEach(btn => {
    const dir = btn.dataset.dir;
    if (dir === 'stop') { btn.onclick = () => { release('stop'); flashStop(); api('/api/stop').catch(() => {}); }; return; }
    btn.addEventListener('pointerdown', e => { e.preventDefault(); btn.setPointerCapture(e.pointerId); press(dir); });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev => btn.addEventListener(ev, () => release(ev)));
  });
  const KEYS = { ArrowUp: 'f', ArrowDown: 'b', ArrowLeft: 'l', ArrowRight: 'r', w: 'f', s: 'b', a: 'l', d: 'r', W: 'f', S: 'b', A: 'l', D: 'r' };
  const editable = el => !!el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA' || el.tagName === 'SELECT' || el.isContentEditable === true);
  document.addEventListener('keydown', e => {
    // Escape stops wherever focus is (review Q09): a stop key is never "typing"
    if (e.key === 'Escape') { e.preventDefault(); release('key'); flashStop(); api('/api/stop').catch(() => {}); return; }
    if (editable(document.activeElement)) return;  // typing, not driving
    if (e.repeat) return;                          // a held key is one press
    if (e.key === ' ') { e.preventDefault(); release('key'); flashStop(); api('/api/stop').catch(() => {}); return; }
    const dir = KEYS[e.key]; if (!dir) return;
    e.preventDefault(); press(dir);
  });
  document.addEventListener('keyup', e => { if (KEYS[e.key]) release('keyup'); });
  // focus moving into a text field while a key is held: the keyup will be typed, not seen here
  document.addEventListener('focusin', e => { if (editable(e.target)) release('focus'); });
  window.addEventListener('blur', () => release('blur'));
  window.addEventListener('pagehide', () => release('pagehide'));
  document.addEventListener('visibilitychange', () => { if (document.hidden) release('hidden'); });
  return { release };
}
