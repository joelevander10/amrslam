/* JIS-style electrical sheet engine.
 * Units are millimetres on an A4 portrait sheet (viewBox 0 0 210 297).
 * A sheet page supplies { meta, draw(sheet, X) }; everything else — frame, row gutter,
 * revision grid, title block, symbols, export — lives here so every sheet looks the same. */
(function () {
  const NS = 'http://www.w3.org/2000/svg';
  const INK = '#111';
  const FONT = {
    mono: '"IBM Plex Mono", ui-monospace, monospace',
    sans: '"IBM Plex Sans", Arial, sans-serif',
  };

  // sheet grid
  const G = {
    W: 210, H: 297,
    fx: 14, fy: 8, fx2: 202.8, fy2: 291,   // drawing frame
    row0: 13, pitch: 10.7, rows: 22,        // line numbers 00–21
    gridX: 20, gridY: 240.5, gridCols: 24,  // revision / cross-reference grid
    titleY: 269,
  };

  // standard x positions of a motor branch
  const X = {
    win: 85, ms: 99.5, ol: 112.2, sk: 120.9, wout: 127,
    term: 140.1, mcx: 148.4, mr: 5.2, desc: 163,
  };

  function el(tag, attrs, parent) {
    const n = document.createElementNS(NS, tag);
    for (const k in attrs) n.setAttribute(k, attrs[k]);
    if (parent) parent.appendChild(n);
    return n;
  }

  const FLAG = '#c0392b';

  function Sheet(svg) { this.svg = svg; this.flags = []; }
  const P = Sheet.prototype;

  /* ---------------- primitives ---------------- */
  P.row = function (r) { return G.row0 + G.pitch * r; };
  P.phases = function (y, gap = 4.2) { return [y, y + gap, y + 2 * gap]; };

  P.line = function (x1, y1, x2, y2, w = 0.3, dash) {
    const a = { x1, y1, x2, y2, stroke: INK, 'stroke-width': w, 'stroke-linecap': 'square' };
    if (dash) { a['stroke-dasharray'] = dash; a['stroke-linecap'] = 'butt'; }
    return el('line', a, this.svg);
  };
  P.path = function (d, w = 0.25, fill = 'none') {
    return el('path', { d, stroke: INK, 'stroke-width': w, fill, 'stroke-linejoin': 'miter' }, this.svg);
  };
  P.rect = function (x, y, w, h, sw = 0.25) {
    return el('rect', { x, y, width: w, height: h, fill: '#fff', stroke: INK, 'stroke-width': sw }, this.svg);
  };
  // white-out a gap in a conductor before drawing a symbol over it
  P.knock = function (x1, y1, x2, y2) {
    el('rect', { x: x1, y: y1, width: x2 - x1, height: y2 - y1, fill: '#fff' }, this.svg);
  };
  P.dot = function (x, y, r = 0.55) { el('circle', { cx: x, cy: y, r, fill: INK }, this.svg); };
  P.ring = function (x, y, r = 0.55) {
    el('circle', { cx: x, cy: y, r, fill: '#fff', stroke: INK, 'stroke-width': 0.22 }, this.svg);
  };
  // Review markers: '?2.2A' = value read with doubt, '«MODEL»' = placeholder not yet filled.
  // Both render red and are counted in the toolbar so nothing uncertain ships unnoticed.
  P.text = function (x, y, s, o = {}) {
    if (s === undefined || s === null || s === '') return null;
    s = String(s);
    let fill = INK;
    if (s.startsWith('?')) { s = s.slice(1); fill = FLAG; this.flags.push(s + ' (uncertain)'); }
    else if (s.includes('«')) { fill = FLAG; this.flags.push(s + ' (placeholder)'); }
    const t = el('text', {
      x, y,
      'font-size': (o.size || 2.2) * (this.textScale || 1),
      'font-family': o.sans ? FONT.sans : FONT.mono,
      'font-weight': o.weight || 500,
      'text-anchor': o.anchor || 'start',
      fill,
    }, this.svg);
    if (o.rotate) t.setAttribute('transform', `rotate(${o.rotate} ${x} ${y})`);
    t.textContent = s;
    return t;
  };

  /* ---------------- symbols (horizontal flow unless noted) ---------------- */

  // cable spec: crossing mark with arrowheads on each conductor + two-line label
  P.wire = function (x, ys, l1, l2, pre) {
    const y0 = ys[0], top = y0 - 1.6, last = ys[ys.length - 1];
    this.line(x, top, x, last - 0.9, 0.18);
    ys.forEach(y => this.path(`M${x - 0.45} ${y - 1.15} L${x + 0.45} ${y - 1.15} L${x} ${y} Z`, 0.1, INK));
    const w = Math.max(l1.length, (l2 || '').length + 1) * 1.2 * (this.textScale || 1) + 0.4;
    this.line(x, top, x + 1.3, y0 - 4.7, 0.18);
    this.line(x + 1.3, y0 - 4.7, x + 1.3 + w, y0 - 4.7, 0.18);
    this.text(x + 1.5, y0 - 5.2, l1, { size: 2 });
    if (l2) this.text(x + 2.4, y0 - 2.3, l2, { size: 2 });
    if (pre) this.text(x + 1.5, y0 - 7.6, pre, { size: 2 });
  };

  // moulded-case circuit breaker (MCCB) with thermal trip element
  P.mccb = function (x, ys, name, rating, aux) {
    ys.forEach(y => {
      this.knock(x - 2, y - 0.5, x + 2, y + 0.5);
      this.ring(x - 1.6, y, 0.42); this.ring(x + 1.6, y, 0.42);
      this.path(`M${x - 1.6} ${y - 0.42} A1.75 1.75 0 0 1 ${x + 1.6} ${y - 0.42}`, 0.22);
      this.knock(x + 3.1, y - 0.4, x + 4.7, y + 0.4);
      this.path(`M${x + 3.1} ${y} V${y - 1.1} H${x + 4.7} V${y}`, 0.22);
    });
    const y0 = ys[0], yl = ys[ys.length - 1];
    this.line(x, y0 - 1.46, x, yl - 1.46, 0.15, '0.5 0.4');
    this.text(x, y0 - 3.3, name, { anchor: 'middle' });
    if (rating) this.text(x + 3.4, y0 - 1.6, rating, { size: 1.9 });
    if (aux) this.text(x + 4.4, y0 - 4.6, aux, { size: 1.5 });
  };

  // MCCB on vertical conductors (flow downward)
  P.mccbV = function (xs, y, name, rating) {
    xs.forEach(x => {
      this.knock(x - 0.5, y - 2, x + 0.5, y + 2);
      this.ring(x, y - 1.6, 0.42); this.ring(x, y + 1.6, 0.42);
      this.path(`M${x + 0.42} ${y - 1.6} A1.75 1.75 0 0 1 ${x + 0.42} ${y + 1.6}`, 0.22);
      this.knock(x - 0.4, y + 3.1, x + 0.4, y + 4.7);
      this.path(`M${x} ${y + 3.1} H${x + 1.1} V${y + 4.7} H${x}`, 0.22);
    });
    const xl = xs[xs.length - 1];
    this.line(xs[0] + 1.46, y, xl + 1.46, y, 0.15, '0.5 0.4');
    this.text(xl + 3.6, y - 0.4, name);
    if (rating) this.text(xl + 3.6, y + 3, rating, { size: 1.9 });
  };

  // magnetic contactor main contacts (NO)
  P.contactor = function (x, ys, name, ref) {
    ys.forEach(y => {
      this.knock(x - 0.8, y - 0.5, x + 0.8, y + 0.5);
      this.line(x - 0.8, y - 1.3, x - 0.8, y + 1.3, 0.25);
      this.line(x + 0.8, y - 1.3, x + 0.8, y + 1.3, 0.25);
    });
    this.text(x, ys[0] - 3, name, { anchor: 'middle' });
    if (ref) this.text(x, ys[ys.length - 1] + 3.1, ref, { size: 1.4, anchor: 'middle' });
  };

  // thermal overload relay heaters (outer two phases)
  P.overload = function (x, ys, name, amps, poles = [0, ys.length - 1]) {
    poles.forEach(i => {
      const y = ys[i];
      this.knock(x - 1.5, y - 0.4, x + 1.5, y + 0.4);
      this.path(`M${x - 1.5} ${y} V${y - 1.3} H${x + 1.5} V${y}`, 0.25);
    });
    this.text(x, ys[0] - 3.3, name, { anchor: 'middle' });
    if (amps) this.text(x + 1.9, ys[0] - 0.6, amps, { size: 1.9 });
  };

  // surge killer (CR absorber) across the phases
  P.surge = function (x, ys, name) {
    const [a, b, c] = ys, x2 = x + 2.6;
    const box = (cx, cy) => this.rect(cx - 0.6, cy - 0.9, 1.2, 1.8, 0.2);
    this.line(x, a, x, c, 0.2);
    [a, b, c].forEach(y => this.dot(x, y, 0.4));
    box(x, (a + b) / 2); box(x, (b + c) / 2);
    this.line(x2, a, x2, c, 0.2);
    this.dot(x2, a, 0.4); this.dot(x2, c, 0.4);
    box(x2, (a + b) / 2);
    this.text(x + 1.3, a - 3.3, name, { anchor: 'middle' });
  };

  // normally-open switch (door switch etc.)
  P.noSwitch = function (x, y, name) {
    this.knock(x - 1.8, y - 0.5, x + 1.8, y + 0.5);
    this.ring(x - 1.4, y, 0.42); this.ring(x + 1.4, y, 0.42);
    this.line(x - 1.05, y - 0.25, x + 1.7, y - 1.6, 0.25);
    this.text(x, y - 2.8, name, { anchor: 'middle' });
  };

  P.earth = function (x, y) {
    this.line(x - 1.2, y, x + 1.2, y, 0.25);
    this.line(x - 0.8, y + 0.6, x + 0.8, y + 0.6, 0.25);
    this.line(x - 0.4, y + 1.2, x + 0.4, y + 1.2, 0.25);
  };

  // vertical bus lines; end = 'tee' | 'arrow' | undefined
  P.bus = function (xs, y1, y2, labels, end) {
    // label size: 1.9, reduced only as far as needed to keep a gap between neighbouring labels
    let size = 1.9;
    if (labels && xs.length > 1) {
      const gap = Math.min(...xs.slice(1).map((x, i) => x - xs[i]));
      const len = Math.max(...labels.map(l => String(l).length));
      size = Math.min(size, (gap - 0.6) / (0.6 * len * (this.textScale || 1)));
    }
    xs.forEach((x, i) => {
      this.line(x, y1, x, y2);
      if (labels) this.text(x, y1 - 0.8, labels[i], { size, anchor: 'middle' });
      if (end === 'tee') this.line(x - 0.9, y2, x + 0.9, y2, 0.3);
      if (end === 'arrow') this.path(`M${x - 0.6} ${y2 - 1.3} L${x + 0.6} ${y2 - 1.3} L${x} ${y2} Z`, 0.1, INK);
    });
  };

  // load description; en2 = optional second line
  P.desc = function (x, y, en, en2) {
    this.text(x, y, en, { size: 2.25, weight: 600 });
    if (en2) this.text(x, y + 4.2, en2, { size: 2.25 });
  };

  // motor with terminals, earth and rating (x = branch positions, defaults to X)
  P.motor = function (b, ys, x = X) {
    const cx = x.mcx, cy = ys[1], r = x.mr, yl = ys[ys.length - 1];
    ys.forEach((y, i) => {
      const edge = cx - Math.sqrt(r * r - (y - cy) ** 2);
      this.line(x.term + 0.6, y, edge, y);
      this.ring(x.term, y, 0.6);
      if (b.t) this.text(x.term - 1, y - 0.6, b.t[i], { size: 2, anchor: 'end' });
    });
    el('circle', { cx, cy, r, fill: '#fff', stroke: INK, 'stroke-width': 0.3 }, this.svg);
    this.text(cx, cy + 0.8, b.m, { size: 2.2, weight: 600, anchor: 'middle' });
    const ex = cx + r + 2.4;
    this.path(`M${cx + r * 0.8} ${cy + r * 0.6} L${ex} ${yl + 0.8} V${yl + 3.3}`, 0.25);
    this.ring(ex, yl + 3.9, 0.6);
    this.text(ex + 0.9, yl + 5.3, 'E', { size: 1.4 });
    if (b.kw) this.text(ex + 1, yl + 0.9, b.kw, { size: 2 });
  };

  // complete motor branch: conductors → cable → MS → OL → SK → cable → motor
  // from = { xs: start x per phase, tap: draw junction dots at the start }
  // xo   = optional overrides of the X positions for this branch only
  // Leave b.ol / b.sk / b.w undefined to omit that device.
  P.branch = function (b, ys, from, xo) {
    const x = Object.assign({}, X, xo);
    ys.forEach((y, i) => {
      this.line(from.xs[i], y, x.term - 0.6, y);
      if (from.tap) this.dot(from.xs[i], y);
    });
    if (b.w) this.wire(x.win, ys, b.w, b.wc || 'BLACK');
    this.contactor(x.ms, ys, b.ms, b.ref);
    if (b.ol) this.overload(x.ol, ys, b.ol, b.amps);
    if (b.sk) this.surge(x.sk, ys, b.sk);
    if (b.w) this.wire(x.wout, ys, b.wout || b.w, b.wc || 'BLACK');
    this.motor(b, ys, x);
    this.desc(x.desc, ys[0] + 0.3, b.en, b.en2);
  };

  // labels at the right-hand end of outgoing conductors
  P.endLabels = function (x, ys, labels) {
    ys.forEach((y, i) => this.text(x + 1.2, y + 0.7, labels[i], { size: 2 }));
  };

  /* ---------------- control circuit (horizontal rungs between vertical rails) ----------------
   * Items are { k: kind, x, ...props } placed on a rung. Kinds:
   *   no / nc        relay or aux contact      { name, ref }
   *   pbno / pbnc    push-button               { name }
   *   coil           relay / contactor coil    { name, refs: ['3308', ...] }
   *   lamp           pilot lamp                { name, color }
   *   fuse           fuse                      { name, rating }
   *   wno            wire number above line    { text }
   */
  P.rails = function (xL, xR, y1, y2, labels = []) {
    [xL, xR].forEach((x, i) => {
      this.line(x, y1, x, y2);
      if (labels[i]) this.text(x, y1 - 0.8, labels[i], { size: 1.9, anchor: 'middle' });
    });
  };

  P.item = function (it, y) {
    const x = it.x;
    switch (it.k) {
      case 'no':
      case 'nc':
        this.knock(x - 1, y - 0.5, x + 1, y + 0.5);
        this.line(x - 1, y - 1.4, x - 1, y + 1.4, 0.25);
        this.line(x + 1, y - 1.4, x + 1, y + 1.4, 0.25);
        if (it.k === 'nc') this.line(x - 1.8, y + 1.6, x + 1.8, y - 1.6, 0.22);
        if (it.name) this.text(x, y - 2.4, it.name, { size: 1.9, anchor: 'middle' });
        if (it.ref) this.text(x, y + 3.2, it.ref, { size: 1.4, anchor: 'middle' });
        break;
      case 'pbno':
      case 'pbnc': {
        this.knock(x - 1.9, y - 0.5, x + 1.9, y + 0.5);
        this.ring(x - 1.4, y, 0.42); this.ring(x + 1.4, y, 0.42);
        const by = it.k === 'pbno' ? y - 1.2 : y + 0.6;
        this.line(x - 2, by, x + 2, by, 0.25);
        this.line(x, by, x, y - 2.6, 0.22);
        this.line(x - 0.7, y - 2.6, x + 0.7, y - 2.6, 0.25);
        if (it.name) this.text(x, y - 3.5, it.name, { size: 1.9, anchor: 'middle' });
        break;
      }
      case 'coil':
        this.knock(x - 2.4, y - 0.5, x + 2.4, y + 0.5);
        el('circle', { cx: x, cy: y, r: 2.4, fill: '#fff', stroke: INK, 'stroke-width': 0.28 }, this.svg);
        this.text(x, y + 0.55, it.name, { size: 1.55, weight: 600, anchor: 'middle' });
        if (it.refs) this.text(x, y + 4.4, it.refs.join(' '), { size: 1.35, anchor: 'middle' });
        break;
      case 'lamp':
        this.knock(x - 1.9, y - 0.5, x + 1.9, y + 0.5);
        el('circle', { cx: x, cy: y, r: 1.9, fill: '#fff', stroke: INK, 'stroke-width': 0.25 }, this.svg);
        this.line(x - 1.34, y - 1.34, x + 1.34, y + 1.34, 0.2);
        this.line(x - 1.34, y + 1.34, x + 1.34, y - 1.34, 0.2);
        if (it.name) this.text(x, y - 2.8, it.name, { size: 1.9, anchor: 'middle' });
        if (it.color) this.text(x, y + 4, it.color, { size: 1.4, anchor: 'middle' });
        break;
      case 'fuse':
        this.knock(x - 1.8, y - 0.8, x + 1.8, y + 0.8);
        this.rect(x - 1.8, y - 0.7, 3.6, 1.4, 0.22);
        this.line(x - 1.8, y, x + 1.8, y, 0.18);
        if (it.name) this.text(x, y - 1.8, it.name, { size: 1.9, anchor: 'middle' });
        if (it.rating) this.text(x, y + 3.2, it.rating, { size: 1.5, anchor: 'middle' });
        break;
      case 'wno':
        this.text(x, y - 0.6, it.text, { size: 1.6, anchor: 'middle' });
        break;
    }
  };

  // a conductor from x1 to x2 at y carrying items
  P.seg = function (y, x1, x2, items = []) {
    this.line(x1, y, x2, y);
    items.forEach(it => this.item(it, y));
  };
  // a full rung between rails, with junction dots on both rails
  P.rung = function (y, rails, items = [], desc) {
    this.seg(y, rails[0], rails[1], items);
    this.dot(rails[0], y); this.dot(rails[1], y);
    if (desc) this.desc(rails[1] + 4, y + 0.3, desc.en, desc.en2);
  };
  // parallel (OR) branch: leaves the rung at x1, rejoins at x2, runs at yb
  P.parallel = function (y, yb, x1, x2, items = []) {
    this.line(x1, y, x1, yb); this.line(x2, y, x2, yb);
    this.dot(x1, y); this.dot(x2, y);
    this.seg(yb, x1, x2, items);
  };

  // multi-pole manual switch on horizontal conductors: one push-button style pole per y
  // (bar under the rings = NC, bar over = NO), poles tied by a dashed mechanical link,
  // stem rising from the top pole to the actuator at `top`. Returns the actuator y.
  // Terminal numbers per IEC 60947-5-1 (JIS C 8201-5-1): first digit = pole order (top first),
  // second = function: NC 1–2, NO 3–4 (11-12, 21-22, 33-34 …). nums: false hides them.
  // flip: true swaps the two numbers of each pole (e.g. 14 left, 13 right) when the wiring enters from the right
  P.switchPoles = function (x, poles, head, nums = true, flip = false) {
    const bar = p => (p.k === 'no' ? p.y - 1.2 : p.y + 0.6);
    poles.forEach((p, i) => {
      this.knock(x - 1.9, p.y - 0.5, x + 1.9, p.y + 0.5);
      this.ring(x - 1.4, p.y, 0.42); this.ring(x + 1.4, p.y, 0.42);
      this.line(x - 2, bar(p), x + 2, bar(p), 0.25);
      if (nums) {
        // numbers on the side away from the bar: over an NC pole, under an NO pole
        const f = p.k === 'no' ? 3 : 1, ty = p.k === 'no' ? p.y + 2 : p.y - 0.8;
        const [l, r] = flip ? [f + 1, f] : [f, f + 1];
        this.text(x - 2.3, ty, `${i + 1}${l}`, { size: 1.3, anchor: 'end' });
        this.text(x + 2.3, ty, `${i + 1}${r}`, { size: 1.3 });
      }
    });
    const ys = poles.map(bar), yTop = Math.min(...ys), yBot = Math.max(...ys);
    if (yBot > yTop) this.line(x, yBot, x, yTop, 0.22, '0.7 0.5');
    const top = Math.min(...poles.map(p => p.y)) - head;
    this.line(x, yTop, x, top, 0.22);
    return top;
  };
  // stay-put (latching) mark on a vertical stem: small notch to the left
  P.detent = function (x, y) {
    this.path(`M${x} ${y - 0.7} L${x - 0.9} ${y} L${x} ${y + 0.7}`, 0.2);
  };
  // emergency-stop push-button, NC poles, mushroom head, latching (turn to release)
  // ys: conductor y of each pole (top first)
  P.estop = function (x, ys, name, o = {}) {
    const top = this.switchPoles(x, ys.map(y => ({ y, k: 'nc' })), 4.4, o.nums !== false, o.flip);
    this.detent(x, top + 2);
    this.path(`M${x - 2.1} ${top} A2.1 2.1 0 0 1 ${x + 2.1} ${top} Z`, 0.25, '#fff');
    if (name) this.text(x, top - 3, name, { size: 1.9, anchor: 'middle' });
  };
  // momentary push-button, any mix of poles [{ y, k: 'nc' | 'no' }], push actuator (no latch)
  P.pushbutton = function (x, poles, name, o = {}) {
    const top = this.switchPoles(x, poles, 2.8, o.nums !== false, o.flip);
    this.line(x - 0.9, top, x + 0.9, top, 0.3);
    if (name) this.text(x + 1.8, top + 0.7, name, { size: 1.9 });
  };
  // maintained selector switch: poles [{ y, k: 'nc' | 'no' }], turn actuator + latching mark
  P.selector = function (x, poles, name, o = {}) {
    const top = this.switchPoles(x, poles, 4, o.nums !== false, o.flip);
    this.detent(x, top + 1.8);
    this.path(`M${x - 1.3} ${top - 0.9} L${x - 1.3} ${top} L${x + 1.3} ${top} L${x + 1.3} ${top + 0.9}`, 0.25);
    if (name) this.text(x, top - 1.8, name, { size: 1.9, anchor: 'middle' });
  };

  /* ---------------- sheet furniture ---------------- */
  P.frame = function (m) {
    const s = this;
    el('rect', { x: G.fx, y: G.fy, width: G.fx2 - G.fx, height: G.fy2 - G.fy, fill: 'none', stroke: INK, 'stroke-width': 0.5 }, s.svg);

    // line numbers
    for (let i = 0; i < G.rows; i++) s.text(15.2, s.row(i) + 1, String(i).padStart(2, '0'), { size: 2.8 });

    // proprietary note in the left margin
    if (m.note) s.text(12.2, 289, m.note, { size: 1.45, rotate: -90 });

    // revision / cross-reference grid
    const gx0 = G.gridX, gx1 = G.fx2, gy0 = G.gridY, gy1 = G.titleY;
    const cw = (gx1 - gx0) / G.gridCols;
    el('rect', { x: gx0, y: gy0, width: gx1 - gx0, height: gy1 - gy0, fill: 'none', stroke: INK, 'stroke-width': 0.3 }, s.svg);
    s.line(gx0, gy0 + 4, gx1, gy0 + 4, 0.18);
    for (let c = 1; c < G.gridCols; c++) s.line(gx0 + c * cw, gy0, gx0 + c * cw, gy1, 0.18);
    (m.refs || []).forEach((r, i) => {
      const cx = gx0 + (i + 0.5) * cw;
      s.text(cx, gy0 + 3, r.name, { size: 1.7, anchor: 'middle' });
      s.text(cx, gy0 + 7.6, r.ref, { size: 1.8, anchor: 'middle' });
    });

    // title block
    const ty = [G.titleY, 276.4, 283.7, G.fy2];
    const tx = [17.8, 65.3, 75.9, 101.6, 112.2, 137.1, G.fx2];
    el('rect', { x: tx[0], y: ty[0], width: tx[6] - tx[0], height: ty[3] - ty[0], fill: '#fff', stroke: INK, 'stroke-width': 0.35 }, s.svg);
    for (let i = 1; i <= 5; i++) s.line(tx[i], ty[0], tx[i], ty[3], 0.25);
    [ty[1], ty[2]].forEach(y => s.line(tx[1], y, tx[5], y, 0.2));
    [ty[1], ty[2]].forEach(y => s.line(tx[0], y, tx[1], y, 0.2));   // customer / designer / content
    s.line(tx[5], ty[2], tx[6], ty[2], 0.2);

    // left column: caption top-left, value centred below it
    const cap = (x, row, t) => s.text(x, ty[row] + 2.2, t, { size: 1.3, weight: 600 });
    cap(18.6, 0, 'CUSTOMER'); cap(18.6, 1, 'DESIGNER'); cap(18.6, 2, 'CONTENT');
    s.text(41.5, ty[0] + 6.1, m.customer, { size: 2.5, weight: 600, anchor: 'middle', sans: true });
    s.text(41.5, ty[1] + 6.1, m.designer, { size: 2.5, weight: 600, anchor: 'middle', sans: true });
    s.text(41.5, ty[2] + 6.1, m.content, { size: 2.4, anchor: 'middle', sans: true });

    const cell = (x, row, t) => s.text(x, ty[row] + 4.3, t, { size: 1.45, weight: 600 });
    cell(66.2, 0, 'APPROVAL'); cell(66.2, 1, 'CHECKED'); cell(66.2, 2, 'DESIGNED');
    cell(102.4, 0, 'DATE'); cell(102.4, 1, 'SCALE'); cell(102.4, 2, 'DRAWN');
    [m.approval, m.checked, m.designed].forEach((v, i) => v && s.text(88.75, ty[i] + 5, v, { size: 2.5, anchor: 'middle', sans: true }));
    [m.date, m.scale, m.drawn].forEach((v, i) => v && s.text(124.65, ty[i] + 5, v, { size: 2.5, anchor: 'middle', sans: true }));

    s.text(139.5, 278.3, m.title, { size: 3.4, weight: 600 });
    s.text(183, 274.6, String(m.sheet), { size: 2.8, anchor: 'middle' });
    s.line(186.5, 282.2, 193.5, 271.6, 0.3);
    s.text(197.5, 281, String(m.of), { size: 2.8, anchor: 'middle' });
    s.text(139.5, 289.4, m.model, { size: 3.1, weight: 600, sans: true });
    s.text(166, 289.4, m.dwg, { size: 3.1, sans: true });
    s.text(197, 289.4, String(m.sheet), { size: 3.1, anchor: 'middle', sans: true });
  };

  /* ---------------- render + export ---------------- */
  function render(host, spec) {
    if (location.hash === '#bare') document.body.classList.add('bare');
    const svg = el('svg', { viewBox: `0 0 ${G.W} ${G.H}`, xmlns: NS, role: 'img', 'aria-label': spec.label || spec.meta.content }, host);
    el('rect', { x: 0, y: 0, width: G.W, height: G.H, fill: '#fff' }, svg);
    const sheet = new Sheet(svg);
    sheet.frame(spec.meta);
    sheet.textScale = spec.textScale || 1;   // drawing text only; frame and title block stay fixed
    spec.draw(sheet, X);

    const status = document.getElementById('status');
    if (sheet.flags.length) {
      if (status) status.textContent = `${sheet.flags.length} item(s) in red need review`;
      console.warn('Review before issue:\n' + sheet.flags.join('\n'));
    }
    window.JIS.flags = sheet.flags;
    const png = document.getElementById('png');
    const pdf = document.getElementById('pdf');
    if (pdf) pdf.addEventListener('click', () => window.print());
    if (png) png.addEventListener('click', async () => {
      if (!window.htmlToImage) { status.textContent = 'Export library did not load — check internet connection'; return; }
      png.disabled = true;
      status.textContent = 'Rendering…';
      try {
        await document.fonts.ready;
        const url = await htmlToImage.toPng(host, {
          width: 1240, height: 1754, pixelRatio: 2, backgroundColor: '#ffffff',
          style: { width: '1240px', height: '1754px', margin: '0', boxShadow: 'none' },
        });
        const a = document.createElement('a');
        a.href = url;
        a.download = `${spec.file}.png`;
        document.body.appendChild(a); a.click(); a.remove();
        status.textContent = 'Saved 2480 × 3508 PNG (A4, 300 dpi)';
      } catch (e) {
        console.error(e);
        status.textContent = 'Export failed — see console';
      } finally {
        png.disabled = false;
      }
    });
  }

  window.JIS = { render, G, X };
})();
