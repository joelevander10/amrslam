// Parameters page: a filter over server-rendered rows, and nothing else.
//
// There is no fetch and no POST in this file, and there is no endpoint it
// could call: a profile is edited in the JSON and the service restarted. The
// values are rendered once, by the server, from the same module the bus thread
// imports - so what is on the screen is what the vehicle is running on, not a
// copy that has to be kept in step.
//
// screen cannot hold an auto run alive. See api_state() in server.py.

const find = document.getElementById('pm-find');
const notes = document.getElementById('pm-notes');
const count = document.getElementById('pm-count');
const rows = Array.from(document.querySelectorAll('.pm-row'));
const secs = Array.from(document.querySelectorAll('.pm-sec'));

function apply() {
  // Matched against key, exported constant, VALUE and section together, so
  // "192.168" finds the module at that address and "rpm" finds every speed.
  const q = find.value.trim().toLowerCase();
  let shown = 0;
  for (const r of rows) {
    const hit = !q || r.dataset.find.includes(q);
    r.hidden = !hit;
    if (hit) shown++;
  }
  // An empty section is hidden entirely rather than left as a bare heading -
  // twelve headings with nothing under them is not a search result.
  for (const s of secs) {
    s.hidden = !s.querySelector('.pm-row:not([hidden])');
  }
  count.textContent = q ? `${shown} of ${rows.length} shown`
                        : `${rows.length} parameters`;
}

// Expanding every note at once is how you read the page as documentation
// rather than as a lookup. Collapsing again restores the table.
function applyNotes() {
  for (const d of document.querySelectorAll('.pm-note')) d.open = notes.checked;
}

find.addEventListener('input', apply);
notes.addEventListener('change', applyNotes);
apply();
