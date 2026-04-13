#pragma once
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta name=viewport content="width=device-width,initial-scale=1">
<style>
  :root{--btn:88px;--gap:14px}
  *{box-sizing:border-box}
  body{
    font-family:system-ui,-apple-system,Segoe UI,Arial;
    max-width:720px;
    margin:0 auto;
    padding:16px;
    user-select:none;
    -webkit-user-select:none;
    touch-action:none;
    text-align:center;
  }
  h2{margin:8px 0 16px}
  .row{
    display:flex;
    gap:12px;
    align-items:center;
    justify-content:center;
    margin:12px 0;
    flex-wrap:wrap;
  }
  .section{
    border:1px solid #ddd;
    border-radius:16px;
    padding:14px;
    margin:18px 0;
  }
  .section h3{
    margin:0 0 12px;
  }
  .grid3{
    display:grid;
    grid-template-columns:var(--btn) var(--btn) var(--btn);
    gap:var(--gap);
    justify-content:center;
    margin:12px 0;
  }
  button{
    width:var(--btn);
    height:var(--btn);
    border:1px solid #999;
    border-radius:14px;
    background:#fff;
    cursor:pointer;
    font-size:15px;
    font-weight:600;
    display:flex;
    align-items:center;
    justify-content:center;
    text-align:center;
  }
  button:active{
    background:#e8f0ff;
  }
  .pressed{
    background:#dbe8ff;
    border-color:#4a78ff;
  }
  .stop{
    background:#ffecec;
    border-color:#f55;
  }
  .small{
    width:48px;
    height:40px;
    font-size:18px;
  }
  .bar{
    display:flex;
    align-items:center;
    gap:8px;
  }
  .pill{
    background:#eee;
    border-radius:999px;
    padding:6px 14px;
    font-weight:600;
  }
  input[type=range]{
    width:280px;
    height:28px;
  }
  input[type=range]::-webkit-slider-thumb{
    width:28px;
    height:28px;
    border-radius:50%;
    background:#666;
    appearance:none;
    margin-top:-10px;
  }
  input[type=range]::-webkit-slider-runnable-track{
    height:8px;
    border-radius:6px;
    background:#ddd;
  }
</style>
</head>
<body>

<h2>BB8 Control</h2>

<div class="section">
  <div class="row">
    <span class="pill">Base Speed: <span id="bv">60</span>%</span>
  </div>

  <div class="row bar">
    <button class="small" id="minus">-</button>
    <input id="bs" type="range" min="0" max="100" value="60">
    <button class="small" id="plus">+</button>
  </div>
</div>

<div class="section">
  <h3>Head Servo</h3>
  <div class="row">
    <span class="pill">Position: <span id="sv">90</span>°</span>
  </div>

  <div class="row bar">
    <button class="small" id="sminus">-</button>
    <input id="ss" type="range" min="0" max="180" value="90">
    <button class="small" id="splus">+</button>
  </div>
</div>

<div class="section">
  <h3>Drive Motor</h3>
  <div class="grid3">
    <div></div>
    <button id="up">DRIVE<br>FWD</button>
    <div></div>

    <div></div>
    <button id="driveStop" class="stop">DRIVE<br>STOP</button>
    <div></div>

    <div></div>
    <button id="down">DRIVE<br>BACK</button>
    <div></div>
  </div>
</div>

<div class="section">
  <h3>Spin Motor</h3>
  <div class="grid3">
    <button id="left">SPIN<br>LEFT</button>
    <button id="stop" class="stop">STOP<br>ALL</button>
    <button id="right">SPIN<br>RIGHT</button>
  </div>
</div>

<script>
(function(){
  const bv = document.getElementById('bv');
  const bs = document.getElementById('bs');
  const minus = document.getElementById('minus');
  const plus = document.getElementById('plus');

  const sv = document.getElementById('sv');
  const ss = document.getElementById('ss');
  const sminus = document.getElementById('sminus');
  const splus = document.getElementById('splus');

  let base = 60;
  let servo = 90;

  function clamp(v,min,max){
    return v < min ? min : (v > max ? max : v);
  }

  function send(path){
    fetch(path).catch(() => {});
  }

  function setBase(v, doSend=true){
    v = clamp((v|0), 0, 100);
    base = v;
    bv.textContent = v;
    bs.value = v;
    if(doSend) send('/base?b=' + v);
  }

  function setServo(v, doSend=true){
    v = clamp((v|0), 0, 180);
    servo = v;
    sv.textContent = v;
    ss.value = v;
    if(doSend) send('/servo?pos=' + v);
  }

  bs.addEventListener('input', e => setBase(parseInt(e.target.value || '0'), false));
  bs.addEventListener('change', e => setBase(parseInt(e.target.value || '0'), true));

  ss.addEventListener('input', e => setServo(parseInt(e.target.value || '0'), true));
  ss.addEventListener('change', e => setServo(parseInt(e.target.value || '0'), true));

  minus.addEventListener('click', () => setBase(base - 5, true));
  plus.addEventListener('click', () => setBase(base + 5, true));

  sminus.addEventListener('click', () => setServo(servo - 5, true));
  splus.addEventListener('click', () => setServo(servo + 5, true));

  function bindHold(id, startCmd, stopCmd){
    const el = document.getElementById(id);

    const start = (ev) => {
      ev.preventDefault();
      el.classList.add('pressed');
      send('/cmd?c=' + startCmd + '&b=' + base);
    };

    const stop = (ev) => {
      ev.preventDefault();
      el.classList.remove('pressed');
      send('/cmd?c=' + stopCmd + '&b=' + base);
    };

    el.addEventListener('pointerdown', start, {passive:false});
    el.addEventListener('pointerup', stop, {passive:false});
    el.addEventListener('pointercancel', stop, {passive:false});
    el.addEventListener('pointerleave', stop, {passive:false});
  }

  bindHold('up', 'fwd', 'drive_stop');
  bindHold('down', 'back', 'drive_stop');
  bindHold('left', 'left', 'spin_stop');
  bindHold('right', 'right', 'spin_stop');

  document.getElementById('driveStop').addEventListener('click', ev => {
    ev.preventDefault();
    send('/cmd?c=drive_stop&b=' + base);
  }, {passive:false});

  document.getElementById('stop').addEventListener('click', ev => {
    ev.preventDefault();
    send('/cmd?c=stop&b=' + base);
  }, {passive:false});

  setBase(base, false);
  setServo(servo, false);
})();
</script>

</body>
</html>
)HTML";