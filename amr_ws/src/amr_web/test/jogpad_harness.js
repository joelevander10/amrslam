// Runs the real static/jogpad.js against a minimal fake DOM and a deferred fetch.
// Driven by test_jogpad_js.py; prints one JSON line of results.
const fs = require('fs');
const path = require('path');
const vm = require('vm');

function el(tag) {
  const e = { tagName: tag, classList: new Set(), dataset: {}, listeners: {}, textContent: '', value: '0.2', children: [],
    addEventListener(ev, fn) { (this.listeners[ev] = this.listeners[ev] || []).push(fn); },
    setPointerCapture() {}, fire(ev, arg) { (this.listeners[ev] || []).forEach(fn => fn(arg || { preventDefault() {}, pointerId: 1 })); } };
  e.classList.toggle = function (c, on) { if (on === undefined ? !this.has(c) : on) this.add(c); else this.delete(c); };
  e.classList.contains = e.classList.has;
  return e;
}
function makeRoot() {
  const buttons = ['fl', 'f', 'fr', 'l', 'stop', 'r', 'bl', 'b', 'br'].map(d => { const b = el('BUTTON'); b.dataset.dir = d; return b; });
  const status = el('B'), speed = el('SELECT');
  return { buttons, status, set innerHTML(v) { this.html = v; },
    querySelector(sel) { return sel === '.jog-status' ? status : sel === '.jog-speed' ? speed : null; },
    querySelectorAll() { return buttons; } };
}
const doc = el('DOC'); doc.activeElement = null; doc.hidden = false;
const win = el('WIN');
const calls = [];
let pendingPress = [];
async function api(url, body) {
  calls.push({ url, body });
  if (url === '/api/manual/press') return new Promise(res => pendingPress.push(() => res({ status: 200, data: { session: 'S' + calls.length, ticket: 't0' } })));
  if (url === '/api/manual/refresh') return { status: 200, data: { ticket: 't' + calls.length, v: body.v, w: body.w } };
  return { status: 200, data: {} };
}
api.catchable = true;
const timers = [];
const ctx = { document: doc, window: win, crypto: { randomUUID: () => 'owner-uuid' }, console,
  setTimeout: (fn) => { timers.push(fn); return timers.length; }, clearTimeout: (i) => { timers[i - 1] = null; },
  api: (u, b) => { const p = api(u, b); return p; } };
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'amr_web', 'static', 'jogpad.js'), 'utf8'), ctx);
const flush = () => new Promise(r => setImmediate(r));
const runTimers = async (n) => { for (let i = 0; i < n; i++) { const t = timers.splice(0); t.forEach(f => f && f()); await flush(); } };
const nonzeroRefreshes = () => calls.filter(c => c.url === '/api/manual/refresh' && (c.body.v !== 0 || c.body.w !== 0)).length;

(async () => {
  const out = {};
  const root = makeRoot();
  ctx.jogpad(root);
  const fwd = root.buttons[1];

  // 1. release BEFORE the press response arrives: no refresh may ever go out
  for (const ev of ['pointerup', 'pointercancel']) {
    calls.length = 0;
    fwd.fire('pointerdown');
    out[`active_while_pending_${ev}`] = fwd.classList.has('active');
    fwd.fire(ev);
    out[`cleared_${ev}`] = !fwd.classList.has('active');
    pendingPress.splice(0).forEach(r => r()); await flush(); await runTimers(3);
    out[`refreshes_after_${ev}`] = nonzeroRefreshes();
    out[`late_session_released_${ev}`] = calls.some(c => c.url === '/api/manual/release');
  }
  // 2. blur and hidden tab before the response
  for (const kind of ['blur', 'hidden']) {
    calls.length = 0;
    fwd.fire('pointerdown');
    if (kind === 'blur') win.fire('blur'); else { doc.hidden = true; doc.fire('visibilitychange'); doc.hidden = false; }
    pendingPress.splice(0).forEach(r => r()); await flush(); await runTimers(3);
    out[`refreshes_after_${kind}`] = nonzeroRefreshes();
  }
  // 3. a normal hold refreshes and lights the cell; release stops refreshing and clears it
  calls.length = 0;
  fwd.fire('pointerdown');
  pendingPress.splice(0).forEach(r => r()); await flush(); await runTimers(3);
  out.hold_refreshes = nonzeroRefreshes();
  out.hold_active = fwd.classList.has('active');
  fwd.fire('pointerup'); await flush();
  const before = nonzeroRefreshes(); await runTimers(3);
  out.refreshes_after_release = nonzeroRefreshes() - before;
  out.release_cleared = !root.buttons.some(b => b.classList.has('active'));
  // 4. Stop cell sends /api/stop and flashes
  calls.length = 0;
  root.buttons[4].onclick(); await flush();
  out.stop_sent = calls.some(c => c.url === '/api/stop');
  out.stop_flash = root.buttons[4].classList.has('active');
  await runTimers(1);
  out.stop_flash_cleared = !root.buttons[4].classList.has('active');
  // 5. Q09: a held key, then focus moves into a text field: no further refresh, and Escape
  //    pressed INSIDE the field still stops
  calls.length = 0;
  const key = (k, extra) => Object.assign({ key: k, repeat: false, preventDefault() {} }, extra || {});
  doc.fire('keydown', key('w'));
  pendingPress.splice(0).forEach(r => r()); await flush(); await runTimers(2);
  out.key_hold_refreshes = nonzeroRefreshes();
  const input = el('INPUT'); doc.activeElement = input;
  doc.fire('focusin', { target: input });
  const atFocus = nonzeroRefreshes(); await runTimers(3);
  out.refreshes_after_focus_change = nonzeroRefreshes() - atFocus;
  out.released_on_focus = calls.some(c => c.url === '/api/manual/release');
  // a fresh press while typing is ignored, Escape in the field stops
  calls.length = 0;
  doc.fire('keydown', key('w')); await flush(); await runTimers(2);
  out.press_while_typing = calls.some(c => c.url === '/api/manual/press');
  doc.fire('keydown', key('Escape')); await flush();
  out.escape_in_field_stops = calls.some(c => c.url === '/api/stop');
  doc.activeElement = null;
  // 6. Q09: focus change while the press response is still pending
  calls.length = 0;
  doc.fire('keydown', key('w'));
  doc.activeElement = input; doc.fire('focusin', { target: input });
  pendingPress.splice(0).forEach(r => r()); await flush(); await runTimers(3);
  out.refreshes_after_focus_while_pending = nonzeroRefreshes();
  doc.activeElement = null;
  // 7. 2026-09-19 speeds: at 0.40 a diagonal keeps the fast wheel at 0.40 and the slow one at
  //    75 % of it; a spin runs at 0.39 rad/s (+30 %); straight ahead is the selection itself
  root.querySelector('.jog-speed').value = '0.4';
  const holdBody = async (btn) => {
    calls.length = 0;
    btn.fire('pointerdown');
    pendingPress.splice(0).forEach(r => r()); await flush(); await runTimers(2);
    const refs = calls.filter(c => c.url === '/api/manual/refresh' && (c.body.v !== 0 || c.body.w !== 0));
    btn.fire('pointerup'); await flush(); await runTimers(1);
    return refs.length ? refs[refs.length - 1].body : null;
  };
  out.speed_fr = await holdBody(root.buttons[2]);
  out.speed_r = await holdBody(root.buttons[5]);
  out.speed_f = await holdBody(root.buttons[1]);
  root.querySelector('.jog-speed').value = '0.1';
  out.speed_l_slow = await holdBody(root.buttons[3]);
  console.log(JSON.stringify(out));
})();
