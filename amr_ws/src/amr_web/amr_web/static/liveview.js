// Live pose + scan overlay and the bounded pollers (unified plan §6.4).
// liveOverlay(view): draws the vehicle pose and the current scan in the view's
// frame only when the frames match; stale (> 1 s) data is drawn dimmed, and a
// generation mismatch draws nothing. pollLive(view, {map, hz}) polls
// /api/live/pose at 5 Hz and, if `map` is set, /api/live/map at 1 Hz, only
// while the page is visible - hidden pages request nothing.
let livePose = null;
function liveOverlay(view) {
  view.overlays.push((c, v) => {
    if (!livePose || !v.meta) return;
    const mode = lastState && lastState.mode;
    const gen = mode ? mode.generation : null;
    if (gen !== null && livePose.generation !== gen) return;
    // every map calls its frame "map": a saved map is only comparable if it is the active one (R12)
    if (v.meta.map_id !== undefined && !(mode && mode.active_map_id === v.meta.map_id && +mode.active_map_revision === +v.meta.revision)) return;
    const frame = v.meta.frame_id || 'map';
    const s = livePose.scan;
    let row = 0;
    if (s && s.frame === frame) {
      const age = ageSince(s), stale = age > 1.0;
      v.points(s.points, stale ? alpha(INK.ink3, 0.5) : INK.scan);
      if (stale) v.label(`STALE SCAN ${age.toFixed(1)} s`, INK.stop, row++);
    }
    const p = livePose.pose;
    if (p && p.frame === frame) {
      const age = ageSince(p), stale = age > 1.0;
      v.arrow(p.x, p.y, p.yaw, 0.6, stale ? INK.ink3 : INK.pose);
      if (stale) v.label(`STALE POSE ${age.toFixed(1)} s`, INK.stop, row++);
    }
  });
}
// Ages shown are the server's age_s at receipt PLUS the browser time elapsed since (Q11): a
// sample that stops being replaced keeps ageing on screen instead of staying "fresh".
function ageSince(sample) { return sample ? sample.age_s + (Date.now() - livePose._received) / 1000 : Infinity; }
function pollLive(view, opts) {
  const o = Object.assign({ map: false, poseHz: 5, mapHz: 1 }, opts || {});
  let tPose = 0, tMap = 0, failures = 0;
  async function tick() {
    try {
      if (!document.hidden) {
        const now = Date.now();
        if (now - tPose >= 1000 / o.poseHz) {
          tPose = now;
          const { status, data } = await apiGet('/api/live/pose');
          if (status === 200) { data._received = Date.now(); livePose = data; failures = 0; }
          view.draw();
        }
        if (o.map && now - tMap >= 1000 / o.mapHz) {
          tMap = now;
          const { status, data } = await apiGet('/api/live/map');
          if (status === 200) { if (data.available) await view.loadLive(data); else if (view._liveSnapshot) view.clearLive(); }
        }
      }
    } catch (e) {
      failures += 1; view.draw();  // redraw so the retained sample ages visibly
    } finally {
      setTimeout(tick, Math.min(100 * Math.pow(2, Math.min(failures, 5)), 3000));  // never stop polling
    }
  }
  tick();
}
