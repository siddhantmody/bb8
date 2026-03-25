#pragma once
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1">
<style>
  :root{--btn:70px;--gap:14px}
  *{box-sizing:border-box}
  body{
    font-family:system-ui,-apple-system,Segoe UI,Arial;
    max-width:640px;margin:0 auto;padding:12px;
    user-select:none;-webkit-user-select:none;touch-action:none;
  }
  .row{display:flex;gap:12px;align-items:center;justify-content:center;margin:10px 0}

  .grid{display:grid;grid-template-columns:var(--btn) var(--btn) var(--btn);gap:var(--gap);justify-content:center;margin:12px 0}

  .mode2{display:grid;grid-template-columns:var(--btn) var(--btn);gap:var(--gap);justify-content:center;margin:12px 0}
  .mode3{display:grid;grid-template-columns:var(--btn) var(--btn) var(--btn);gap:var(--gap);justify-content:center;margin:12px 0}

  button{
    width:var(--btn);height:var(--btn);
    border:1px solid #999;border-radius:14px;background:#fff;cursor:pointer;
    font-size:18px;font-weight:600;display:flex;align-items:center;justify-content:center;
  }
  .stop{background:#ffecec;border-color:#f55}
  .small{width:48px;height:40px;font-size:18px}
  .bar{display:flex;align-items:center;gap:8px}
  .pill{background:#eee;border-radius:999px;padding:4px 12px}
  input[type=range]{width:280px;height:28px}
  input[type=range]::-webkit-slider-thumb{width:28px;height:28px;border-radius:50%;background:#666;appearance:none;margin-top:-10px}
  input[type=range]::-webkit-slider-runnable-track{height:8px;border-radius:6px;background:#ddd}
</style></head><body>

<div class="row">
  <span class="pill">Base Speed: <span id="bv">60</span>%</span>
</div>

<div class="row bar">
  <button class="small" id="minus">-</button>
  <input id="bs" type="range" min="0" max="100" value="60">
  <button class="small" id="plus">+</button>
</div>

<div class="grid">
  <div></div>
  <button id="up">UP</button>
  <div></div>

  <button id="left">LEFT</button>
  <button id="stop" class="stop">STOP</button>
  <button id="right">RIGHT</button>

  <div></div>
  <button id="down">DOWN</button>
  <div></div>
</div>

<!-- Row of 2: ATTACK ARM + WALL FOLLOW -->
<div class="mode2">
  <button id="attack">ATTACK ARM</button>
  <button id="wall">WALL FOLLOW</button>
</div>

<!-- Row of 3: NEXUS + MIDDLE TOWER + VIVE -->
<div class="mode3">
  <button id="nexus">NEXUS</button>
  <button id="mid">MIDDLE TOWER</button>
  <button id="vive">VIVE</button>
</div>

<script>
(function(){
  const bv=document.getElementById('bv');
  const bs=document.getElementById('bs');
  const minus=document.getElementById('minus');
  const plus=document.getElementById('plus');

  let base=60, t=null, lastSend=0;
  function clamp(v,min,max){ return v<min?min:(v>max?max:v); }

  function send(path){
    const now=performance.now();
    if(now-lastSend<60){
      clearTimeout(t);
      t=setTimeout(()=>fetch(path).catch(()=>{}),60);
      return;
    }
    lastSend=now;
    fetch(path).catch(()=>{});
  }

  function setBase(v, doSend=true){
    v=clamp((v|0),0,100);
    base=v;
    bv.textContent=v;
    bs.value=v;
    if(doSend) send('/base?b='+v);
  }

  bs.addEventListener('input', e=>setBase(parseInt(e.target.value||'0'), false));
  bs.addEventListener('change', e=>setBase(parseInt(e.target.value||'0'), true));

  minus.addEventListener('click', ()=> setBase(base-5, true));
  plus .addEventListener('click', ()=> setBase(base+5, true));

  function bindTap(id, cmd){
    const el = document.getElementById(id);
    el.addEventListener('click', ev=>{
      ev.preventDefault();
      send('/cmd?c=' + cmd + '&b=' + base);
    }, {passive:false});
  }

  bindTap('up','fwd');
  bindTap('down','back');
  bindTap('left','left');
  bindTap('right','right');

  document.getElementById('stop')
          .addEventListener('click', ()=> send('/cmd?c=stop&b='+base), {passive:false});

  function bindClick(id, cmd){
    const el=document.getElementById(id);
    el.addEventListener('click', ev=>{
      ev.preventDefault();
      send('/cmd?c='+cmd+'&b='+base);
    }, {passive:false});
  }

  bindClick('attack','attack');
  bindClick('wall','wall');
  bindClick('nexus','nexus');
  bindClick('mid','mid');
  bindClick('vive','vive');

  setBase(base, false);
})();
</script>
</body></html>
)HTML";
