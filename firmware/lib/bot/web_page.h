// The page your phone shows at http://192.168.4.1  (plain HTML + JavaScript, no internet needed)
#pragma once
#include <Arduino.h>

static const char WEB_PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>WheelBiped</title>
<style>
 body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:12px;user-select:none;-webkit-user-select:none}
 h1{font-size:18px;margin:0 0 8px}
 .row{display:flex;justify-content:space-between;font-size:14px;margin:4px 0}
 #status{font-weight:bold}
 canvas{background:#000;border-radius:8px;width:100%;height:120px;display:block;margin:8px 0}
 #pad{width:220px;height:220px;border-radius:50%;background:#222;border:2px solid #444;margin:12px auto;position:relative;touch-action:none}
 #knob{width:70px;height:70px;border-radius:50%;background:#3a7bd5;position:absolute;left:75px;top:75px}
 .s{margin:10px 0} .s label{display:flex;justify-content:space-between;font-size:14px}
 input[type=range]{width:100%}
 button{background:#333;color:#eee;border:1px solid #555;border-radius:6px;padding:8px 14px;font-size:14px;margin:4px 4px 0 0}
 small{color:#888}
</style></head><body>
<h1>WheelBiped</h1>
<div class="row"><span id="status">connecting...</span><span>battery <b id="batt">-</b> V</span></div>
<div class="row"><span>tilt <b id="ang">-</b>&deg; (target <span id="tgt">-</span>)</span><span>motor <b id="out">-</b></span></div>
<canvas id="plot" width="600" height="120"></canvas>
<small>white = tilt, blue = target, orange = motor output</small>
<div id="drive"><div id="pad"><div id="knob"></div></div></div>
<div id="legs" class="s" style="display:none"><label>Height (wheel below shafts, mm)<b id="v_height"></b></label>
<input type="range" id="height" min="72" max="145" step="1">
<div class="row"><span>legs R/L offset <b id="dh">-</b> mm</span><span>roll <b id="roll">-</b>&deg;</span></div></div>
<div id="sliders"></div>
<button onclick="fetch('/save').then(()=>alert('Saved to robot'))">Save</button>
<button onclick="fetch('/defaults').then(load)">Defaults</button>
<p><small>Start from the defaults. Fast buzzing/shaking: lower Kd, then Kp. Slow rocking that grows: raise Kd a bit.
Topples slowly before it catches itself: raise Ki. Keeps rolling one way: change Trim, then raise Kv/Kvi. Change one thing at a time.</small></p>
<script>
const P=[['kp','Kp (push back per degree)',0,0.3,0.001],['ki','Ki (main balancing push)',0,8,0.05],
 ['kd','Kd (damping)',0,0.02,0.0001],['trim','Trim (balance point, deg)',-10,10,0.1],
 ['kv','Kv (speed -> lean)',0,20,0.1],['kvi','Kvi (stay in place)',0,10,0.1],
 ['lean','Lean into turns (legs)',0,1.5,0.05],['level','Roll levelling (legs)',0,2,0.05]];
const sl=document.getElementById('sliders');
P.forEach(([k,n,mn,mx,st])=>{sl.insertAdjacentHTML('beforeend',
 `<div class="s"><label>${n}<b id="v_${k}"></b></label><input type="range" id="${k}" min="${mn}" max="${mx}" step="${st}"></div>`);
 const e=document.getElementById(k);e.oninput=()=>{document.getElementById('v_'+k).textContent=e.value;fetch(`/set?${k}=${e.value}`)};});
function load(){fetch('/state').then(r=>r.json()).then(s=>{P.forEach(([k])=>{document.getElementById(k).value=s[k];
 document.getElementById('v_'+k).textContent=s[k]});if(!s.drive)document.getElementById('drive').style.display='none';
 if(s.legs){document.getElementById('legs').style.display='block';hs.value=s.hcmd;v_height.textContent=s.hcmd;}
 else['lean','level'].forEach(k=>document.getElementById(k).parentElement.style.display='none');});}
const hs=document.getElementById('height');hs.oninput=()=>{v_height.textContent=hs.value;fetch(`/set?height=${hs.value}`)};
load();
// live plot
const cv=document.getElementById('plot'),cx=cv.getContext('2d');const H=[];
function draw(){cx.clearRect(0,0,600,120);cx.strokeStyle='#333';cx.beginPath();cx.moveTo(0,60);cx.lineTo(600,60);cx.stroke();
 [['angle','#fff',3],['target','#3a7bd5',3],['out','#f90',50]].forEach(([k,c,sc])=>{cx.strokeStyle=c;cx.beginPath();
 H.forEach((s,i)=>{const y=60-s[k]*sc;i?cx.lineTo(i*4,y):cx.moveTo(0,y)});cx.stroke();});}
function poll(){fetch('/state').then(r=>r.json()).then(s=>{
 document.getElementById('status').textContent=s.armed?'BALANCING':'standby - hold it upright';
 document.getElementById('status').style.color=s.armed?'#4c4':'#fa0';
 ang.textContent=s.angle.toFixed(1);tgt.textContent=s.target.toFixed(1);out.textContent=s.out.toFixed(2);batt.textContent=s.batt.toFixed(2);dh.textContent=s.dh.toFixed(0);roll.textContent=s.roll.toFixed(1);
 H.push(s);if(H.length>150)H.shift();draw();}).catch(()=>{}).finally(()=>setTimeout(poll,100));}
poll();
// joystick
const pad=document.getElementById('pad'),knob=document.getElementById('knob');let jv=0,jt=0,active=false;
function setKnob(x,y){knob.style.left=(75+x*75)+'px';knob.style.top=(75-y*75)+'px';}
function mv(e){const r=pad.getBoundingClientRect(),t=e.touches?e.touches[0]:e;
 let x=(t.clientX-r.left-110)/75,y=-(t.clientY-r.top-110)/75;const m=Math.hypot(x,y);if(m>1){x/=m;y/=m;}
 jv=y;jt=x;setKnob(x,y);e.preventDefault();}
pad.addEventListener('touchstart',e=>{active=true;mv(e)});pad.addEventListener('touchmove',mv);
pad.addEventListener('mousedown',e=>{active=true;mv(e)});window.addEventListener('mousemove',e=>{if(active)mv(e)});
function up(){active=false;jv=0;jt=0;setKnob(0,0);fetch('/cmd?v=0&t=0');}
pad.addEventListener('touchend',up);window.addEventListener('mouseup',()=>{if(active)up()});
setInterval(()=>{if(active)fetch(`/cmd?v=${jv.toFixed(2)}&t=${jt.toFixed(2)}`)},100);
</script></body></html>
)rawliteral";
