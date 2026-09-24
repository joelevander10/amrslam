// Hold-to-drive jog pad.
//
// A held button does NOT set a latch on the server. It re-POSTs /api/drive
// every REPEAT_MS, and the bus thread stops the AGV if it stops hearing them.
// So "release" and "the browser died" are the same event as far as the AGV is
// concerned, which is exactly what you want from a jog control.
//
// Keyboard resolves to a direction through two axes rather than one key per
// button, so the in-between directions come out of a combination:
//
//        ArrowUp                 Up          = FWD        Up + Left  = FWD-L
//   ArrowLeft  ArrowRight        Left        = SPIN-L     Up + Right = FWD-R
//       ArrowDown                Down        = REV        Down+Left  = REV-L
//                                Right       = SPIN-R     Down+Right = REV-R
//
// ARROWS ONLY. WASD used to mirror them and Q/E/Z/C were one-key shortcuts to
// the diagonals; both are gone. Space is kept, because it only ever STOPS -
// removing a stop is the one change to a jog control worth refusing.

const REPEAT_MS = 100;
const pad = document.getElementById('pad');
const buttons = [...pad.querySelectorAll('.dir')];
const byDir = new Map(buttons.map(b => [b.dataset.dir, b]));

// Axis contributions. fwd: +1 forward, -1 reverse. turn: -1 left, +1 right.
const AXIS = {
  ArrowUp:    {fwd:  1},
  ArrowDown:  {fwd: -1},
  ArrowLeft:  {turn: -1},
  ArrowRight: {turn:  1},
};

// (fwd, turn) -> direction. A turn with no forward/reverse component is an
// on-axis spin; with one, it is a curve.
const RESOLVE = {
  '1,0':  'forward',  '1,-1':  'forward_left',  '1,1':  'forward_right',
  '0,0':  null,       '0,-1':  'left',          '0,1':  'right',
  '-1,0': 'reverse',  '-1,-1': 'reverse_left',  '-1,1': 'reverse_right',
};

const down = new Set();     // key codes currently held
let armed = false;
let pointerDir = null;      // direction held by mouse/touch
let held = null;            // direction actually being sent
let timer = null;

function paint() {
  for (const b of buttons) {
    b.disabled = !armed;
    b.classList.toggle('active', armed && b.dataset.dir === held);
  }
}

// --- what the operator is currently asking for ---------------------------
function keyDirection() {
  let fwd = 0, turn = 0, nf = 0, nt = 0;
  for (const code of down) {
    const a = AXIS[code];
    if (!a) continue;
    if (a.fwd  !== undefined) { fwd  += a.fwd;  nf++; }
    if (a.turn !== undefined) { turn += a.turn; nt++; }
  }
  // Opposites held together (Up+Down, Left+Right) cancel to zero on that axis.
  const sign = v => (v > 0 ? 1 : v < 0 ? -1 : 0);
  return RESOLVE[`${sign(fwd)},${sign(turn)}`] || null;
}

function refresh() {
  const want = pointerDir || keyDirection();
  if (want) press(want); else release();
}

// --- talking to the server ------------------------------------------------
function press(dir) {
  if (!armed || held === dir) return;
  held = dir;
  paint();
  clearInterval(timer);
  const tick = async () => {
    if (held !== dir) return;
    try { await api('/api/drive', {dir}); }
    catch (e) { log('drive: ' + e.message); release(); }
  };
  tick();
  timer = setInterval(tick, REPEAT_MS);
}

function release() {
  if (held === null) return;
  held = null;
  clearInterval(timer);
  timer = null;
  paint();
  api('/api/stop').catch(() => {});
}

function panic() {
  down.clear();
  pointerDir = null;
  release();
}

// --- pointer -------------------------------------------------------------
for (const b of buttons) {
  const dir = b.dataset.dir;
  b.addEventListener('pointerdown', e => {
    e.preventDefault();
    b.setPointerCapture?.(e.pointerId);
    if (dir === 'stop') { panic(); return; }
    pointerDir = dir;
    refresh();
  });
  const up = () => { pointerDir = null; refresh(); };
  b.addEventListener('pointerup', up);
  b.addEventListener('pointercancel', up);
  b.addEventListener('lostpointercapture', up);
  b.addEventListener('contextmenu', e => e.preventDefault());
}

// --- keyboard ------------------------------------------------------------
addEventListener('keydown', e => {
  if (e.repeat || e.metaKey || e.ctrlKey || e.altKey) return;
  if (e.code === 'Space') { e.preventDefault(); panic(); return; }
  if (!AXIS[e.code]) return;
  e.preventDefault();          // stop arrow keys scrolling the page
  down.add(e.code);
  refresh();
});

addEventListener('keyup', e => {
  if (!down.delete(e.code)) return;
  e.preventDefault();
  refresh();
});

// --- anything that means "the operator is no longer in control" ----------
// Losing focus mid-press means the keyup never arrives, so the key would
// otherwise stay stuck down. Clear everything rather than trust the set.
addEventListener('blur', panic);
addEventListener('pagehide', panic);
document.addEventListener('visibilitychange', () => { if (document.hidden) panic(); });

// --- arm / disarm --------------------------------------------------------
document.getElementById('disarm').addEventListener('click', async () => {
  panic();
  try { await api('/api/disarm'); log('disarmed — motors de-energised'); }
  catch (e) { log('disarm: ' + e.message); }
});

onState(s => {
  const wasArmed = armed;
  armed = s.armed && s.mode === 'manual';
  if (wasArmed && !armed) { held = null; clearInterval(timer); down.clear(); pointerDir = null; }
  paint();
});

paint();
