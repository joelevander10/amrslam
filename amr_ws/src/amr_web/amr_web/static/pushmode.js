// Push mode (AMR QR): release / re-apply the drive brakes so the vehicle can be pushed by hand
// (surveying, placing it on a route start). A widget for any page: pushMode(el) fills `el` with
// the state tile and the two buttons. Releasing asks for confirmation; the server refuses it in
// AUTO or with a run under way. Brakes OFF = /drives/disarm: 0 V, direction outputs low,
// brakes released; the jog pad does nothing until Brakes ON. The E-stop still brakes.
function pushMode(el) {
  if (!el) return;
  el.innerHTML = `<h3>Push mode <small>brakes · MANUAL only</small></h3>
    <div class="tel-grid rows" data-push-state></div>
    <div class="bar"><button data-push-off class="tool">Brakes OFF (push)</button><button data-push-on class="tool">Brakes ON (drive)</button></div>
    <p class="note">Brakes off: <b>the vehicle rolls freely</b> - level floor only. Push it slowly; the encoders keep the odometry (and the map) going. Press Brakes ON before driving.</p>`;
  const tile = el.querySelector('[data-push-state]'), off = el.querySelector('[data-push-off]'), on = el.querySelector('[data-push-on]');
  const call = async (path, what) => {
    const { status, data } = await api(path);
    if (status === 200 && data && data.ok) log(`${what}: ${data.message}`);  // failures are logged by api()
  };
  off.onclick = () => {
    if (!window.confirm('Release the brakes? The vehicle will roll freely until you press "Brakes ON".')) return;
    call('/api/brakes/release', 'brakes off');
  };
  on.onclick = () => call('/api/brakes/engage', 'brakes on');
  onState(st => {
    const d = st.drives, pushing = !!d && /push/.test(d.power_reason || '');
    const auto = !!st.panel && st.panel.mode_auto;
    const r = st.run, running = !!r && ['EXECUTING', 'PAUSED', 'BLOCKED', 'WAITING'].includes(r.state_name);
    tiles(tile, [['Brakes', !d ? '–' : pushing ? 'OFF · push' : (d.operational ? 'ON · drive' : 'ON'),
      d ? (d.power_reason || d.left) : 'no drive status', !d ? 'bad' : pushing ? 'warn' : '']]);
    off.disabled = pushing || auto || running;
    off.title = auto ? 'Selector is AUTO: press STOP (MANUAL) first' : running ? 'A run is under way: pause or abort it first' : '';
    on.disabled = !pushing;
  });
}
document.querySelectorAll('[data-push-mode]').forEach(pushMode);
