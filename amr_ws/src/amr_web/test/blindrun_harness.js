// Runs the real static/blindrun.js pure helpers (BR) without a DOM.
// Driven by test_blindrun_page.py; prints one JSON line of results.
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ctx = { console };  // no `document`: the page wiring must stay out of the way
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'amr_web', 'static', 'blindrun.js'), 'utf8'), ctx);
const BR = ctx.BR;

const caps = { max_speed_mps: 0.8, max_distance_m: 10, min_arc_radius_m: 0.2435 };
const base = { speed: '0.20', direction: 'forward', side: 'left', distance: '1', angle: '90', radius: '1' };
const b = (f, c) => BR.buildPlan(Object.assign({}, base, f), Object.assign({}, caps, c || {}));

try {
  const out = {
    straight_rev: b({ kind: 'straight', direction: 'reverse', distance: '1.5', speed: '0.25' }),
    rotate_cw: b({ kind: 'rotate', direction: 'cw', angle: '90' }),
    arc_right: b({ kind: 'arc', side: 'right', angle: '45', radius: '1.2' }),
    arc_too_tight: b({ kind: 'arc', radius: '0.2' }),
    too_fast: b({ kind: 'straight', speed: '0.85' }),
    blank_distance: b({ kind: 'straight', distance: '' }),
    stop_0p8: BR.stopDistance(0.8, 0.2513),
  };
  console.log(JSON.stringify(JSON.parse(JSON.stringify(out))));
} catch (e) {
  console.log(JSON.stringify({ error: String(e && e.stack || e) }));
}
