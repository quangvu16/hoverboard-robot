// web_page.h - the drive page this board serves at http://<board>/ (web_drive.h).
//
// One self-contained page: a stick, two RPM sliders, Stop and live status.
// It opens a WebSocket back to the same board on /ws and sends, while the
// stick is held, ten times a second:
//
//   "D <rpm_left> <rpm_right>"   wheel targets, + = forward
//   "S"                          stop (sent a few times on release)
//
// and receives a JSON status line five times a second (web_drive.h).

#pragma once

#include <Arduino.h>

static const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Hoverboard drive</title>
<style>
:root { --bg:#eef1ee; --surface:#fff; --sunk:#e3e8e4; --fg:#17201c; --muted:#5a6861; --line:#cfd7d1;
  --accent:#1f5fd1; --ok:#2f7d4f; --ok-soft:#dcefe2; --bad:#c2410c; --bad-soft:#fbe7dc; --warn:#a16207; --warn-soft:#fbf0d2;
  --ui:system-ui,-apple-system,"Segoe UI",Roboto,Ubuntu,sans-serif; --mono:ui-monospace,"DejaVu Sans Mono",Menlo,Consolas,monospace; }
@media (prefers-color-scheme: dark) { :root { --bg:#101512; --surface:#171e1a; --sunk:#1f2823; --fg:#e3ebe5; --muted:#9aaba1;
  --line:#2e3b34; --accent:#7aa5ff; --ok:#6fcf94; --ok-soft:#1b3325; --bad:#ff9a62; --bad-soft:#3a2418; --warn:#facc15; --warn-soft:#3a3214; color-scheme:dark; } }
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--fg); font:15px/1.45 var(--ui); }
main { max-width:440px; margin:0 auto; padding:14px 16px 24px; display:grid; gap:12px; }
header { display:flex; flex-wrap:wrap; align-items:center; gap:6px 8px; }
h1 { font-size:1.05rem; margin:0 auto 0 0; }
.pill { font:500 .72rem var(--mono); padding:3px 9px; border-radius:999px; background:var(--sunk); color:var(--muted); white-space:nowrap; }
.pill.ok { background:var(--ok-soft); color:var(--ok); } .pill.bad { background:var(--bad-soft); color:var(--bad); }
.pill.warn { background:var(--warn-soft); color:var(--warn); }
.card { background:var(--surface); border:1px solid var(--line); border-radius:14px; padding:14px; display:grid; gap:10px; }
.pad { position:relative; width:min(300px,80vw); aspect-ratio:1; margin:4px auto; border-radius:50%; background:var(--sunk);
  border:1px solid var(--line); touch-action:none; user-select:none; -webkit-user-select:none; cursor:grab; }
.pad::before, .pad::after { content:""; position:absolute; background:var(--line); }
.pad::before { left:50%; top:12%; bottom:12%; width:1px; } .pad::after { top:50%; left:12%; right:12%; height:1px; }
.lab { position:absolute; font:600 .68rem var(--ui); color:var(--muted); letter-spacing:.06em; text-transform:uppercase; pointer-events:none; }
.fw { top:8px; left:0; right:0; text-align:center; } .bk { bottom:8px; left:0; right:0; text-align:center; }
.lf { left:10px; top:50%; transform:translateY(-50%); } .rt { right:10px; top:50%; transform:translateY(-50%); }
.knob { position:absolute; left:50%; top:50%; width:30%; aspect-ratio:1; border-radius:50%; background:var(--accent);
  box-shadow:0 3px 10px rgb(0 0 0/.25); transform:translate(-50%,-50%); transition:transform .15s ease-out; pointer-events:none; }
.pad.on { cursor:grabbing; } .pad.on .knob { transition:none; } .pad.off { opacity:.45; }
.cmd { font:.9rem var(--mono); text-align:center; }
.sl { display:grid; grid-template-columns:5.4em 1fr 4.4em; gap:8px; align-items:center; font-size:.88rem; color:var(--muted); }
.sl input { width:100%; accent-color:var(--accent); } .sl output { font:.82rem var(--mono); text-align:right; color:var(--fg); }
button { font:600 1rem var(--ui); border:0; border-radius:10px; padding:13px; background:var(--bad); color:#fff; cursor:pointer; }
dl { display:grid; grid-template-columns:auto 1fr; gap:4px 12px; margin:0; }
dt { color:var(--muted); font-size:.88rem; } dd { margin:0; font:.88rem var(--mono); text-align:right; }
.note { font-size:.8rem; color:var(--muted); margin:0; }
</style>
</head>
<body>
<main>
  <header>
    <h1>Hoverboard drive</h1>
    <span class="pill" id="pConn">connecting…</span>
    <span class="pill" id="pOwn">wheels ?</span>
    <span class="pill" id="pSonar">sonar ?</span>
    <span class="pill" id="pBat">battery ?</span>
  </header>
  <div class="card">
    <div class="pad off" id="pad" role="application" aria-label="Drive stick: drag to drive, release to stop">
      <span class="lab fw">fwd</span><span class="lab bk">back</span><span class="lab lf">left</span><span class="lab rt">right</span>
      <div class="knob" id="knob"></div>
    </div>
    <div class="cmd" id="cmd">stopped</div>
    <div class="sl"><label for="v">Speed</label><input type="range" id="v" min="5" max="60" step="1" value="25"><output id="vo"></output></div>
    <div class="sl"><label for="w">Turn</label><input type="range" id="w" min="5" max="60" step="1" value="15"><output id="wo"></output></div>
    <button id="stop">Stop</button>
    <p class="note">Keys: W A S D or arrows, Space stops. Release = stop. 25 rpm is about 0.25 m/s on 6.5" wheels.
      While you hold the stick it overrides ROS; ROS gets the wheels back 0.5 s after you let go.</p>
  </div>
  <div class="card">
    <dl>
      <dt>Mainboard</dt><dd id="mb">—</dd>
      <dt>Battery</dt><dd id="bat">—</dd>
      <dt>Wheels (measured)</dt><dd id="rpm">—</dd>
      <dt>Sonar front · left · right</dt><dd id="son">—</dd>
      <dt>Reflex</dt><dd id="rfx">—</dd>
    </dl>
  </div>
</main>
<script>
'use strict';
const $ = id => document.getElementById(id);
const pad = $('pad'), knob = $('knob');
let ws = null, live = false;
const drive = { x: 0, y: 0, pointer: null, keys: new Set(), zeroes: 0 };
const KEYMAP = { w: 'f', arrowup: 'f', s: 'b', arrowdown: 'b', a: 'l', arrowleft: 'l', d: 'r', arrowright: 'r' };

function pill(id, text, cls) { const e = $(id); e.textContent = text; e.className = 'pill' + (cls ? ' ' + cls : ''); }
function connect() {
  pill('pConn', 'connecting…', '');
  const s = new WebSocket('ws://' + location.host + '/ws');
  ws = s;
  s.onopen = () => { live = true; pill('pConn', 'connected', 'ok'); pad.classList.remove('off'); };
  s.onclose = () => {
    if (ws !== s) return;
    live = false; release(); drive.keys.clear(); pad.classList.add('off');
    pill('pConn', 'disconnected', 'bad'); pill('pOwn', 'wheels ?', ''); pill('pSonar', 'sonar ?', ''); pill('pBat', 'battery ?', '');
    setTimeout(connect, 2000);
  };
  s.onmessage = e => { let m; try { m = JSON.parse(e.data); } catch (err) { return; } status(m); };
}
function status(m) {
  if (m.max) { $('v').max = m.max; $('w').max = m.max; limits(); }
  $('mb').textContent = m.mb ? 'connected' : 'no feedback';
  $('bat').textContent = m.mb ? m.b.toFixed(1) + ' V' : '—';
  $('rpm').textContent = m.rl.toFixed(0) + ' / ' + m.rr.toFixed(0) + ' rpm';
  const mm = v => v ? v + ' mm' : '—';
  $('son').textContent = mm(m.sf) + ' · ' + mm(m.sl) + ' · ' + mm(m.sr);
  $('rfx').textContent = m.rfx || 'none';
  pill('pOwn', 'wheels: ' + m.own, m.own === 'web' ? 'ok' : (m.own === 'ROS' ? 'warn' : ''));
  pill('pSonar', m.near ? 'obstacle ahead' : 'sonar clear', m.near ? 'warn' : 'ok');
  const B = { ok: ['battery ok', 'ok'], low: ['battery low: speed limited', 'warn'],
              critical: ['battery critical: stopped', 'bad'] }[m.bat] || ['battery ?', ''];
  pill('pBat', B[0], B[1]);
}

const vMax = () => +$('v').value, wMax = () => +$('w').value;
function limits() { $('vo').textContent = vMax() + ' rpm'; $('wo').textContent = wMax() + ' rpm'; }
$('v').oninput = $('w').oninput = limits; limits();

// Stick right = left wheel faster = turn right.
function wheels() {
  let x = drive.x, y = drive.y;
  if (drive.pointer === null && drive.keys.size) {
    const k = drive.keys;
    y = (k.has('f') ? 1 : 0) - (k.has('b') ? 1 : 0);
    x = (k.has('r') ? 1 : 0) - (k.has('l') ? 1 : 0);
  }
  const v = y * vMax(), w = x * wMax();
  return [Math.round((v + w) * 10) / 10, Math.round((v - w) * 10) / 10];
}
function send(t) { if (live && ws.readyState === 1) ws.send(t); }
function tick() {
  const [l, r] = wheels(), moving = l !== 0 || r !== 0;
  if (moving) { send('D ' + l + ' ' + r); drive.zeroes = 3; }
  else if (drive.zeroes > 0) { send('S'); drive.zeroes--; }
  $('cmd').textContent = moving ? 'L ' + l + ' · R ' + r + ' rpm' : 'stopped';
}
setInterval(tick, 100);

function release() {
  drive.x = drive.y = 0;
  if (drive.pointer !== null) { try { pad.releasePointerCapture(drive.pointer); } catch (e) {} drive.pointer = null; }
  pad.classList.remove('on'); knob.style.transform = '';
}
function stop() { release(); drive.keys.clear(); drive.zeroes = 5; tick(); }
$('stop').onclick = stop;

function move(e) {
  const r = pad.getBoundingClientRect(), half = r.width / 2;
  let x = (e.clientX - r.left - half) / half, y = -(e.clientY - r.top - half) / half;
  const m = Math.hypot(x, y);
  if (m > 1) { x /= m; y /= m; }
  const t = half * 0.7;
  knob.style.transform = `translate(calc(-50% + ${x * t}px), calc(-50% + ${-y * t}px))`;
  if (Math.hypot(x, y) < 0.12) x = y = 0;
  drive.x = x; drive.y = y;
}
pad.addEventListener('pointerdown', e => {
  if (!live) return;
  e.preventDefault();
  try { pad.setPointerCapture(e.pointerId); } catch (err) {}
  drive.pointer = e.pointerId; pad.classList.add('on'); move(e); tick();
});
pad.addEventListener('pointermove', e => { if (e.pointerId === drive.pointer) move(e); });
['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev =>
  pad.addEventListener(ev, e => { if (e.pointerId === drive.pointer) { release(); tick(); } }));

document.addEventListener('keydown', e => {
  if (e.key === ' ') { e.preventDefault(); stop(); return; }
  const d = KEYMAP[e.key.toLowerCase()];
  if (!d || (e.target.type === 'range' && e.key.startsWith('Arrow'))) return;
  e.preventDefault();
  if (live && !drive.keys.has(d)) { drive.keys.add(d); tick(); }
});
document.addEventListener('keyup', e => { const d = KEYMAP[e.key.toLowerCase()]; if (d) drive.keys.delete(d); });
window.addEventListener('blur', stop);
document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
connect();
</script>
</body>
</html>)HTML";
