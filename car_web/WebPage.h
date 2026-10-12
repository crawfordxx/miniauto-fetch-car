// 控制网页：状态栏、寻物/停止/返回起点、定距直线与定角转向按钮、摇杆
#pragma once
#include <Arduino.h>

static const char kWebPage[] PROGMEM = R"HTML(<!doctype html>
<html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>寻物小车</title>
<style>
:root{color-scheme:light dark;--bg:#f4f4f2;--fg:#1c1c1a;--card:#fff;--line:#d6d6d0;--pad:#e2e2dc;--accent:#2b6cb0;--stop:#c53030;--ok:#2f855a}
@media(prefers-color-scheme:dark){:root{--bg:#141413;--fg:#ececea;--card:#1f1f1d;--line:#3a3a36;--pad:#2c2c28;--accent:#63a4ff;--stop:#fc6b6b;--ok:#5ec48a}}
*{box-sizing:border-box}
html,body{margin:0;background:var(--bg);color:var(--fg);font-family:-apple-system,"PingFang SC",sans-serif}
main{max-width:480px;margin:0 auto;padding:16px;display:flex;flex-direction:column;gap:14px}
h1{font-size:20px;margin:0}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px}
.status{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;font-size:15px}
.status b{font-weight:600}
.row{display:flex;gap:10px}
button{flex:1;font-size:17px;padding:14px 8px;border-radius:12px;border:1px solid var(--line);background:var(--card);color:var(--fg);touch-action:manipulation}
button:active{opacity:.6}
.main{background:var(--accent);border-color:var(--accent);color:#fff}
.stop{background:var(--stop);border-color:var(--stop);color:#fff}
.ret{background:var(--ok);border-color:var(--ok);color:#fff}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
.grid button{font-size:22px}
.label{font-size:13px;opacity:.7;margin:0 0 8px}
select{font-size:16px;padding:6px;border-radius:8px}
#pad{width:min(70vw,240px);aspect-ratio:1;border-radius:50%;background:var(--pad);position:relative;margin:8px auto;touch-action:none}
#knob{width:32%;aspect-ratio:1;border-radius:50%;background:var(--accent);position:absolute;left:34%;top:34%}
</style></head><body><main>
<h1>寻物小车</h1>
<div class="card status">
  <span>状态</span><b id="state">-</b>
  <span>消息</span><span id="msg">-</span>
  <span>电池</span><span id="bat">-</span>
  <span>路线</span><span id="steps">-</span>
</div>

<div class="card">
  <p class="label">按"开始寻物"时，当前位置记为起点</p>
  <div class="row"><button class="main" onclick="cmd('fetch')">开始寻物</button></div>
  <div class="row" style="margin-top:10px">
    <button class="stop" onclick="stopAll()">停止</button>
    <button class="ret" onclick="cmd('return')">返回起点</button>
  </div>
</div>

<div class="card">
  <p class="label">定距直线（按编码器走，计入返回路线）
    <select id="cm"><option>5</option><option selected>10</option><option>20</option><option>30</option></select> 厘米
  </p>
  <div class="grid">
    <button onclick="turn('l')">↺</button><button onclick="step('f')">↑</button><button onclick="turn('r')">↻</button>
    <button onclick="step('l')">←</button><button class="stop" onclick="stopAll()">■</button><button onclick="step('r')">→</button>
    <span></span><button onclick="step('b')">↓</button><span></span>
  </div>
  <p class="label" style="margin-top:8px">↺ ↻ 每次转
    <select id="deg"><option>15</option><option selected>30</option><option>45</option><option>90</option></select> 度；← → 为横向平移
  </p>
</div>

<div class="card">
  <p class="label">机械臂（爪子 90 闭合 ~ 160 张开；升降 58 放下 ~ 132 抬起）</p>
  <div class="row"><button onclick="arm('claw',150)">爪子张开</button><button onclick="arm('claw',100)">爪子合拢</button></div>
  <div class="row" style="margin-top:10px"><button class="stop" onclick="releasePad();cmd('release')">舵机卸力（可用手掰动）</button></div>
  <p class="label" style="margin-top:10px">爪子角度 <b id="clawv">130</b>°</p>
  <input id="claw" type="range" min="90" max="160" step="5" value="130" style="width:100%"
         oninput="$('clawv').textContent=this.value" onchange="arm('claw',this.value)">
  <p class="label" style="margin-top:10px">升降角度 <b id="liftv">58</b>°</p>
  <input id="lift" type="range" min="58" max="132" step="2" value="58" style="width:100%"
         oninput="$('liftv').textContent=this.value" onchange="arm('lift',this.value)">
</div>

<div class="card">
  <p class="label">摇杆（松手即停；摇杆移动不计入返回路线）</p>
  <div id="pad"><div id="knob"></div></div>
  <div class="row"><button id="mode">模式：转向</button></div>
</div>
</main>
<script>
const $=id=>document.getElementById(id);
function show(t){$('msg').textContent=t}
function get(u){return fetch(u).then(r=>r.text()).then(show).catch(()=>show('连接断开'))}
function cmd(c){get('/cmd?c='+c)}
function step(d){get('/cmd?c=step&dir='+d+'&v='+$('cm').value)}
function turn(d){get('/cmd?c=turn&dir='+d+'&v='+$('deg').value)}
function arm(k,v){if(k==="claw"){$("claw").value=v;$("clawv").textContent=v}get("/cmd?c="+k+"&v="+v)}
function stopAll(){releasePad();get('/stop')}
function poll(){fetch('/status').then(r=>r.json()).then(s=>{
  $('state').textContent=s.state;$('msg').textContent=s.msg;
  $('bat').textContent=(s.battery/1000).toFixed(2)+' V';$('steps').textContent=s.steps+' 段'
}).catch(()=>{$('state').textContent='连接断开'})}
setInterval(poll,1000);poll();

const pad=$('pad'),knob=$('knob');let x=0,y=0,strafe=false,active=false,timer=null;
function setKnob(){const r=pad.clientWidth/2,k=knob.clientWidth/2;knob.style.left=(r+x*(r-k)-k)+'px';knob.style.top=(r-y*(r-k)-k)+'px'}
function move(e){const b=pad.getBoundingClientRect(),r=b.width/2,k=knob.clientWidth/2;
  let dx=(e.clientX-b.left-r)/(r-k),dy=-(e.clientY-b.top-r)/(r-k);const n=Math.hypot(dx,dy);if(n>1){dx/=n;dy/=n}x=dx;y=dy;setKnob()}
function sendJoy(){fetch('/joy?x='+x.toFixed(2)+'&y='+y.toFixed(2)+'&m='+(strafe?1:0)).catch(()=>{})}
pad.addEventListener('pointerdown',e=>{active=true;pad.setPointerCapture(e.pointerId);move(e);sendJoy();timer=setInterval(sendJoy,100)});
pad.addEventListener('pointermove',e=>{if(active)move(e)});
function releasePad(){if(!active)return;active=false;clearInterval(timer);x=0;y=0;setKnob();sendJoy()}
pad.addEventListener('pointerup',releasePad);pad.addEventListener('pointercancel',releasePad);
$('mode').onclick=()=>{strafe=!strafe;$('mode').textContent='模式：'+(strafe?'平移':'转向');$('mode').classList.toggle('main',strafe)};
setKnob();
</script></body></html>)HTML";
