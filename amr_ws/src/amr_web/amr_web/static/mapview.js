// MapView: a map bundle image on a canvas with pan/zoom and the three coordinate
// systems the editor needs. Formulas mirror amr_maps.grid.world_to_pixel /
// pixel_to_world exactly (origin, resolution, origin yaw, row inversion).
//   world (x, y) metres  <->  pixel (u, v) image coords, v down  <->  screen (sx, sy)
// Canvas colours are the UI tokens (design language: canvas graphics match the page).
const cssVar = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const INK = {
  paper: cssVar('--paper'), rule: cssVar('--rule'), ink3: cssVar('--ink-3'), accent: cssVar('--accent'),
  route: cssVar('--accent-2'), pose: cssVar('--ok'), scan: cssVar('--stop'), turn: cssVar('--hazard-ink'),
  preview: cssVar('--hazard'), stop: cssVar('--stop'),
};
// hex token + alpha 0..1 -> #rrggbbaa (tokens are #RRGGBB)
const alpha = (hex, a) => hex + Math.round(a * 255).toString(16).padStart(2, '0');

class MapView {
  constructor(canvas) {
    this.canvas = canvas; this.ctx = canvas.getContext('2d');
    this.meta = null; this.img = null; this.scale = 1; this.ox = 0; this.oy = 0;
    this.overlays = []; this._drag = null; this.onClick = null; this.onDrag = null; this.onHover = null;
    canvas.addEventListener('wheel', e => { e.preventDefault(); this.zoomAt(e.offsetX, e.offsetY, e.deltaY < 0 ? 1.15 : 1/1.15); });
    canvas.addEventListener('mousedown', e => { this._drag = { x: e.offsetX, y: e.offsetY, moved: false, btn: e.button, t0: Date.now() }; });
    canvas.addEventListener('mousemove', e => {
      // a plain move with a tool active previews what a click would do (world point, or null)
      if (!this._drag) { if (this.tool && this.onHover) { this.onHover(this.screenToWorld(e.offsetX, e.offsetY)); this.draw(); } return; }
      const dx = e.offsetX - this._drag.x, dy = e.offsetY - this._drag.y;
      if (Math.abs(dx) + Math.abs(dy) > 3) this._drag.moved = true;
      if (this.onDrag && this._drag.btn === 0 && this.tool) { this.onDrag(this.screenToWorld(this._drag.x, this._drag.y), this.screenToWorld(e.offsetX, e.offsetY), false); this.draw(); return; }
      this.ox += dx; this.oy += dy; this._drag.x = e.offsetX; this._drag.y = e.offsetY; this._keepVisible(); this.draw();
    });
    const up = e => {
      if (!this._drag) return;
      const d = this._drag; this._drag = null;
      const w = this.screenToWorld(e.offsetX, e.offsetY);
      if (this.tool && d.btn === 0) {
        if (this.onDrag && d.moved) this.onDrag(this.screenToWorld(d.x, d.y), w, true);
        else if (this.onClick) this.onClick(w);
      }
      this.draw();
    };
    canvas.addEventListener('mouseup', up);
    canvas.addEventListener('mouseleave', () => { this._drag = null; if (this.onHover) { this.onHover(null); this.draw(); } });
    canvas.addEventListener('dblclick', e => { e.preventDefault(); this.fit(); this.draw(); });
    canvas.addEventListener('contextmenu', e => e.preventDefault());
    this._resize(); window.addEventListener('resize', () => { this._resize(); this.draw(); });
    // labels are Plex Mono: redraw once the self-hosted face has loaded
    if (document.fonts && document.fonts.ready) document.fonts.ready.then(() => this.draw());
  }
  _resize() { const r = this.canvas.getBoundingClientRect(); this.canvas.width = Math.max(200, r.width); this.canvas.height = Math.max(200, r.height); }
  // Returns false when a later load() superseded this one while it was in flight: the
  // picture and its metadata always belong to the same, most recently requested map (R12).
  async load(mapId, rev) {
    const token = this._loadToken = {};
    const { data } = await apiGet(`/api/maps/${mapId}/${rev}`);
    if (this._loadToken !== token) return false;
    const im = await new Promise((res, rej) => { const i = new Image(); i.onload = () => res(i); i.onerror = rej; i.src = `/api/maps/${mapId}/${rev}/image.png?s=${data.sha256.slice(0, 8)}`; });
    if (this._loadToken !== token) return false;
    // dynamic areas (trolleys, parked forklifts: the scan there may change) drawn hatched on
    // every page that shows a saved map; a missing mask never blocks the map itself
    const dyn = data.dynamic_cells > 0 ? await new Promise((res) => { const i = new Image(); i.onload = () => res(i); i.onerror = () => res(null); i.src = `/api/maps/${mapId}/${rev}/dynamic.png?s=${data.sha256.slice(0, 8)}`; }) : null;
    if (this._loadToken !== token) return false;
    this.meta = data; this.img = im; this.dyn = dyn ? hatchMask(dyn, INK.preview) : null;
    this.fit(); this.draw();
    return true;
  }
  // Live grid (unified plan §6.4): metadata and image share one snapshot id; the
  // grid may grow and move its origin between snapshots, so meta and image are
  // swapped together. The view is fitted only the first time (or when asked),
  // never on every update.
  async loadLive(meta) {
    if (!meta || !meta.available) return false;
    if (this._liveSnapshot === meta.snapshot && this.meta && this.meta.generation === meta.generation) return false;
    const im = await new Promise((res) => { const i = new Image(); i.onload = () => res(i); i.onerror = () => res(null); i.src = `/api/live/map.png?snapshot=${meta.snapshot}`; });
    if (!im) return false;  // a newer snapshot appeared; the next poll gets it
    const first = !this.img || !this.meta || this.meta.generation !== meta.generation;
    this.meta = meta; this.img = im; this.dyn = null; this._liveSnapshot = meta.snapshot;
    if (first) this.fit();
    this.draw();
    return true;
  }
  clearLive() { this.meta = null; this.img = null; this.dyn = null; this._liveSnapshot = null; this.draw(); }
  fit() {
    if (!this.img) return;
    this.scale = this._fitScale = Math.min(this.canvas.width / this.img.width, this.canvas.height / this.img.height) * 0.95;
    this.ox = (this.canvas.width - this.img.width * this.scale) / 2; this.oy = (this.canvas.height - this.img.height * this.scale) / 2;
  }
  // Zoom is bounded to [fit/4, fit x 40] and a pan always leaves some of the map on
  // screen: a sensitive trackpad or a free-spinning wheel must not lose the picture.
  // Double-click refits.
  zoomAt(sx, sy, f) {
    if (!this.img) return;
    const fit = this._fitScale || this.scale, target = Math.min(fit * 40, Math.max(fit / 4, this.scale * f));
    f = target / this.scale;
    this.ox = sx - (sx - this.ox) * f; this.oy = sy - (sy - this.oy) * f; this.scale = target;
    this._keepVisible(); this.draw();
  }
  _keepVisible() {
    if (!this.img) return;
    const w = this.img.width * this.scale, h = this.img.height * this.scale, keep = 60;
    this.ox = Math.min(this.canvas.width - keep, Math.max(keep - w, this.ox));
    this.oy = Math.min(this.canvas.height - keep, Math.max(keep - h, this.oy));
  }
  // --- coordinates (see amr_maps.grid) ---
  worldToPixel(x, y) {
    const m = this.meta, dx = x - m.origin[0], dy = y - m.origin[1], c = Math.cos(m.origin[2]), s = Math.sin(m.origin[2]);
    const gx = c * dx + s * dy, gy = -s * dx + c * dy;
    return [gx / m.resolution, m.height - gy / m.resolution];
  }
  pixelToWorld(u, v) {
    const m = this.meta, gx = u * m.resolution, gy = (m.height - v) * m.resolution, c = Math.cos(m.origin[2]), s = Math.sin(m.origin[2]);
    return [m.origin[0] + c * gx - s * gy, m.origin[1] + s * gx + c * gy];
  }
  pixelToScreen(u, v) { return [this.ox + u * this.scale, this.oy + v * this.scale]; }
  screenToPixel(sx, sy) { return [(sx - this.ox) / this.scale, (sy - this.oy) / this.scale]; }
  worldToScreen(x, y) { const [u, v] = this.worldToPixel(x, y); return this.pixelToScreen(u, v); }
  screenToWorld(sx, sy) { const [u, v] = this.screenToPixel(sx, sy); return this.pixelToWorld(u, v); }
  // --- drawing ---
  draw() {
    const c = this.ctx; c.fillStyle = INK.paper; c.fillRect(0, 0, this.canvas.width, this.canvas.height);
    if (!this.img) return;
    c.imageSmoothingEnabled = false;
    const w = this.img.width * this.scale, h = this.img.height * this.scale;
    c.drawImage(this.img, this.ox, this.oy, w, h);
    if (this.dyn) c.drawImage(this.dyn, this.ox, this.oy, w, h);
    // unknown cells (205) are close to the paper tone: a hairline frame keeps the map edge visible
    c.strokeStyle = INK.rule; c.lineWidth = 1; c.strokeRect(Math.round(this.ox) - 0.5, Math.round(this.oy) - 0.5, Math.round(w) + 1, Math.round(h) + 1);
    this.overlays.forEach(fn => fn(c, this));
    this.drawStartMark();
  }
  // The survey's start reference (the floor mark the operator surveyed from). Most
  // routes begin there, so it is always drawn on a saved map, on top of the overlays.
  drawStartMark() {
    const st = this.meta && this.meta.start;
    if (!st || st.x_m === undefined) return;
    const c = this.ctx, p = this.worldToScreen(st.x_m, st.y_m), yaw = st.yaw_rad || 0, r = 9;
    c.save();
    c.strokeStyle = INK.accent; c.lineWidth = 2; c.setLineDash([3, 3]);
    c.beginPath(); c.arc(p[0], p[1], r, 0, 2 * Math.PI); c.stroke();
    c.setLineDash([]);
    c.beginPath(); c.moveTo(p[0], p[1]); c.lineTo(p[0] + 2.2 * r * Math.cos(yaw), p[1] - 2.2 * r * Math.sin(yaw)); c.stroke();
    c.font = '600 11px "IBM Plex Mono", monospace'; c.fillStyle = INK.accent;
    c.fillText('SURVEY START', p[0] + r + 4, p[1] + r + 10);
    c.restore();
  }
  // helpers for overlays
  line(x1, y1, x2, y2, color, width) { const c = this.ctx, a = this.worldToScreen(x1, y1), b = this.worldToScreen(x2, y2); c.strokeStyle = color; c.lineWidth = width || 2; c.beginPath(); c.moveTo(a[0], a[1]); c.lineTo(b[0], b[1]); c.stroke(); }
  dot(x, y, color, r) { const c = this.ctx, p = this.worldToScreen(x, y); c.fillStyle = color; c.beginPath(); c.arc(p[0], p[1], r || 4, 0, 2 * Math.PI); c.fill(); }
  text(x, y, s, color) { const c = this.ctx, p = this.worldToScreen(x, y); c.fillStyle = color || INK.ink3; c.font = '500 11px "IBM Plex Mono", monospace'; c.fillText(s, p[0] + 6, p[1] - 6); }
  // A fixed caption in the top-left corner (screen space), e.g. STALE SCAN: stale must never look clear.
  label(text, color, row) { const c = this.ctx; c.font = '600 11px "IBM Plex Mono", monospace'; const y = 10 + (row || 0) * 22, w = c.measureText(text).width + 12;
    c.fillStyle = INK.paper; c.fillRect(8, y, w, 18); c.strokeStyle = color; c.lineWidth = 1; c.strokeRect(8.5, y + 0.5, w - 1, 17); c.fillStyle = color; c.fillText(text, 14, y + 13); }
  arrow(x, y, yaw, len, color) { this.line(x, y, x + len * Math.cos(yaw), y + len * Math.sin(yaw), color, 3); this.dot(x, y, color, 5); }
  points(pts, color) { const c = this.ctx; c.fillStyle = color; pts.forEach(p => { const q = this.worldToScreen(p[0], p[1]); c.fillRect(q[0] - 1, q[1] - 1, 2, 2); }); }
  polyline(pts, color, width) { const c = this.ctx; c.strokeStyle = color; c.lineWidth = width || 2; c.beginPath(); pts.forEach((p, i) => { const s = this.worldToScreen(p[0], p[1]); i ? c.lineTo(s[0], s[1]) : c.moveTo(s[0], s[1]); }); c.stroke(); }
  polygon(pts, color) { const c = this.ctx; c.strokeStyle = color; c.lineWidth = 1.5; c.beginPath(); pts.forEach((p, i) => { const s = this.worldToScreen(p[0], p[1]); i ? c.lineTo(s[0], s[1]) : c.moveTo(s[0], s[1]); }); c.closePath(); c.stroke(); }
  // A filled world polygon with an optional outline (map edits, dynamic areas being drawn).
  fillPolygon(pts, fill, stroke) { const c = this.ctx; c.beginPath(); pts.forEach((p, i) => { const s = this.worldToScreen(p[0], p[1]); i ? c.lineTo(s[0], s[1]) : c.moveTo(s[0], s[1]); }); c.closePath(); if (fill) { c.fillStyle = fill; c.fill(); } if (stroke) { c.strokeStyle = stroke; c.lineWidth = 1.5; c.stroke(); } }
  // Dashed outline of a world polygon: the area a validation check sweeps.
  dashed(pts, color) { const c = this.ctx; c.setLineDash([4, 4]); this.polygon(pts, color); c.setLineDash([]); }
  footprint(x, y, yaw, poly, color) { const c = Math.cos(yaw), s = Math.sin(yaw); this.polygon(poly.map(p => [x + c * p[0] - s * p[1], y + s * p[0] + c * p[1]]), color); }
}
// A dynamic-area mask image (255 = dynamic) as a canvas of the same size: diagonal stripes of
// `hex`, one cell per pixel, so the hatch scales with the map and never hides it.
function hatchMask(img, hex) {
  const w = img.width, h = img.height, cv = document.createElement('canvas'); cv.width = w; cv.height = h;
  const c = cv.getContext('2d'); c.drawImage(img, 0, 0);
  const src = c.getImageData(0, 0, w, h), out = c.createImageData(w, h);
  const r = parseInt(hex.slice(1, 3), 16), g = parseInt(hex.slice(3, 5), 16), b = parseInt(hex.slice(5, 7), 16);
  for (let v = 0, k = 0; v < h; v++) for (let u = 0; u < w; u++, k += 4) {
    if (src.data[k] < 128) continue;
    out.data[k] = r; out.data[k + 1] = g; out.data[k + 2] = b; out.data[k + 3] = (u + v) % 6 < 2 ? 200 : 60;
  }
  c.putImageData(out, 0, 0);
  return cv;
}
async function fillMapSelect(sel) {
  const { data } = await apiGet('/api/maps');
  sel.innerHTML = '';
  data.forEach(m => m.revisions.forEach(r => { if (r.error) return; const o = document.createElement('option'); o.value = `${m.map_id}/${r.revision}`; o.textContent = `${m.map_id} rev${r.revision} (${r.created})`; sel.appendChild(o); }));
  return data;
}
