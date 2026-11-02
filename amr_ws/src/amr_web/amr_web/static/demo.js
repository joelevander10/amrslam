// The DEMO page (templates/demo.html). Read-only: the one request it makes is
// GET /api/demo, whose every word is decided server-side in demo.py. The vehicle
// on display is stationary, so the live side is its safety scanner's view.
(function () {
  'use strict';

  var POLL_MS = 500;
  var STALE_MS = 5000;      // no answer this long: the live side says "standing by"
  var PAUSE_MS = 30000;     // a visitor who taps a card gets to read it

  var $ = function (id) { return document.getElementById(id); };
  var live = document.querySelector('.d-live');
  var statusBox = document.querySelector('.d-status');
  var css = getComputedStyle(document.body);
  var color = function (name) { return css.getPropertyValue(name).trim(); };
  var lastOk = 0;

  // ---- the radar: vehicle outline at the centre, forward = up ----
  var canvas = $('d-radar');
  var ctx = canvas.getContext('2d');
  var outline = [];
  try { outline = JSON.parse(canvas.dataset.outline || '[]'); } catch (e) { outline = []; }
  var viewM = Number(canvas.dataset.view) || 4;
  var points = [], nearPoints = [], status = 'standby';

  function draw() {
    var dpr = window.devicePixelRatio || 1;
    var w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    var cx = w / 2, cy = h / 2;
    var k = (Math.min(w, h) / 2 - 6) / viewM;   // px per metre
    // body frame x forward, y left  ->  screen up, left
    var sx = function (x, y) { return cx - y * k; };
    var sy = function (x, y) { return cy - x * k; };

    // range rings, 1 m apart
    ctx.strokeStyle = color('--grid');
    ctx.lineWidth = 1.5;
    ctx.fillStyle = color('--ink-2');
    ctx.font = '14px "IBM Plex Sans", sans-serif';
    for (var r = 1; r <= viewM; r++) {
      ctx.beginPath();
      ctx.arc(cx, cy, r * k, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillText(r + ' m', cx + r * k * 0.7071 + 4, cy - r * k * 0.7071 - 4);
    }

    // the vehicle
    if (outline.length > 2) {
      ctx.beginPath();
      outline.forEach(function (p, i) {
        if (i) ctx.lineTo(sx(p[0], p[1]), sy(p[0], p[1]));
        else ctx.moveTo(sx(p[0], p[1]), sy(p[0], p[1]));
      });
      ctx.closePath();
      ctx.fillStyle = color('--brand');
      ctx.fill();
      // forward arrow
      var fx = Math.max.apply(null, outline.map(function (p) { return p[0]; }));
      ctx.beginPath();
      ctx.moveTo(cx, sy(fx, 0) + 6);
      ctx.lineTo(cx - 10, sy(fx, 0) + 22);
      ctx.lineTo(cx + 10, sy(fx, 0) + 22);
      ctx.closePath();
      ctx.fillStyle = '#fff';
      ctx.fill();
    }

    // what the scanner sees; what is close to it stands out
    function dots(list, fill, size) {
      ctx.fillStyle = fill;
      list.forEach(function (p) {
        ctx.beginPath();
        ctx.arc(sx(p[0], p[1]), sy(p[0], p[1]), size, 0, Math.PI * 2);
        ctx.fill();
      });
    }
    dots(points, color('--point'), 3);
    dots(nearPoints, color('--near'), 5);
  }
  window.addEventListener('resize', draw);

  function setStatus(s, sentence) {
    status = s;
    statusBox.dataset.status = s;
    live.dataset.status = s;
    $('d-sentence').textContent = sentence;
  }

  function show(d) {
    setStatus(d.status, d.sentence);
    points = d.points || [];
    nearPoints = d.near_points || [];
    $('d-nearest').textContent = d.nearest_m == null ? '–' : Number(d.nearest_m).toFixed(1);
    draw();
  }

  function poll() {
    fetch('/api/demo', { cache: 'no-store' })
      .then(function (r) { if (!r.ok) throw new Error(r.status); return r.json(); })
      .then(function (d) { lastOk = Date.now(); show(d); })
      .catch(function () {
        if (Date.now() - lastOk > STALE_MS) {
          setStatus('standby', 'Standing by for the next demonstration');
          points = [];
          nearPoints = [];
          draw();
        }
      })
      .then(function () { setTimeout(poll, POLL_MS); });
  }
  setStatus('standby', 'Standing by for the next demonstration');
  draw();
  poll();

  // ---- story cards ----
  var cards = Array.prototype.slice.call(document.querySelectorAll('.d-card'));
  var dots = Array.prototype.slice.call(document.querySelectorAll('#d-dots button'));
  var rotateMs = (Number($('d-cards').dataset.rotate) || 8) * 1000;
  var idx = 0, timer = null;

  function go(i) {
    if (!cards.length) return;
    idx = (i + cards.length) % cards.length;
    cards.forEach(function (c, k) { c.classList.toggle('on', k === idx); });
    dots.forEach(function (d, k) { d.classList.toggle('on', k === idx); });
  }
  function schedule(ms) {
    clearTimeout(timer);
    timer = setTimeout(function () { go(idx + 1); schedule(rotateMs); }, ms);
  }
  cards.forEach(function (c) {
    c.addEventListener('click', function () { go(idx + 1); schedule(PAUSE_MS); });
  });
  dots.forEach(function (d, k) {
    d.addEventListener('click', function () { go(k); schedule(PAUSE_MS); });
  });
  schedule(rotateMs);

  // ---- reload: like Ctrl+Shift+R (the panel has no keyboard). Re-download the CSS
  // and JS into the cache first, so a software update shows. ----
  $('d-reload').addEventListener('click', function () {
    var urls = Array.prototype.map.call(
      document.querySelectorAll('link[rel="stylesheet"][href], script[src]'),
      function (e) { return e.href || e.src; });
    Promise.all(urls.map(function (u) {
      return fetch(u, { cache: 'reload' }).catch(function () {});
    })).then(function () { location.reload(); });
  });

  // ---- the corner ✕ goes back to Home ----
  $('d-exit').addEventListener('click', function () { location.href = '/home'; });
})();
