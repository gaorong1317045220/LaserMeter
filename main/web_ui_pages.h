#pragma once

// Self-contained web UI pages for the ESP-IDF HTTP server.  The pages use no
// external assets, so they remain usable while connected directly to the
// device access point.
namespace web_ui {

static constexpr char kDashboardHtml[] = R"WEBPAGE(
<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0a0f18">
<title>Laser · 实时控制台</title>
<style>
:root{color-scheme:dark;--bg:#080c13;--panel:#101722;--panel2:#151e2b;--line:#243144;--text:#eef4fb;--muted:#8fa0b5;--cyan:#43d7e8;--blue:#5a8cff;--green:#49d49d;--red:#ff6b79;--amber:#ffbc5e;--r:18px}
*{box-sizing:border-box}
html{background:var(--bg)}
body{margin:0;min-height:100vh;color:var(--text);background:radial-gradient(900px 500px at 75% -20%,#17375c66,transparent),var(--bg);font:14px/1.45 system-ui,-apple-system,"Segoe UI","Microsoft YaHei",sans-serif;-webkit-font-smoothing:antialiased}
button,a{font:inherit}button{cursor:pointer}
.topbar{position:sticky;top:0;z-index:10;height:66px;display:flex;align-items:center;justify-content:space-between;padding:0 max(18px,env(safe-area-inset-left));background:#090e17e8;border-bottom:1px solid #202c3c;backdrop-filter:blur(18px)}
.brand{display:flex;align-items:center;gap:11px;font-weight:720;letter-spacing:.2px}.logo{width:32px;height:32px;display:grid;place-items:center;border-radius:10px;color:#061017;background:linear-gradient(145deg,var(--cyan),#6d91ff);box-shadow:0 0 24px #43d7e844;font-weight:900}.brand small{display:block;color:var(--muted);font-size:11px;font-weight:500;letter-spacing:.8px;text-transform:uppercase}
.nav{display:flex;align-items:center;gap:10px}.link{display:inline-flex;align-items:center;gap:8px;min-height:38px;padding:0 13px;border:1px solid var(--line);border-radius:11px;color:var(--text);background:#131b27;text-decoration:none}.link:hover{border-color:#46627f;background:#182334}
.online{display:flex;align-items:center;gap:7px;padding:8px 11px;border-radius:999px;color:var(--muted);background:#101721;border:1px solid var(--line);font-size:12px}.dot{width:8px;height:8px;border-radius:50%;background:var(--amber);box-shadow:0 0 0 3px #ffbc5e20}.online.ok .dot{background:var(--green);box-shadow:0 0 0 3px #49d49d20}.online.bad .dot{background:var(--red);box-shadow:0 0 0 3px #ff6b7920}
main{width:min(1240px,100%);margin:auto;padding:22px;display:grid;grid-template-columns:minmax(0,1.55fr) minmax(290px,.75fr);gap:18px}
.card{overflow:hidden;border:1px solid var(--line);border-radius:var(--r);background:linear-gradient(155deg,#131b27,#0e151f);box-shadow:0 16px 48px #0004}.cardhead{min-height:55px;display:flex;align-items:center;justify-content:space-between;gap:14px;padding:0 18px;border-bottom:1px solid var(--line)}.cardhead h2{margin:0;font-size:14px;letter-spacing:.2px}.hint{color:var(--muted);font-size:12px}
.camera{grid-row:span 2}.stage{position:relative;display:grid;place-items:center;min-height:300px;aspect-ratio:4/3;background:#020305;overflow:hidden}.stage img{display:block;width:100%;height:100%;object-fit:contain}.live{position:absolute;top:14px;left:14px;display:flex;align-items:center;gap:7px;padding:6px 9px;border-radius:8px;background:#080d14c9;border:1px solid #ffffff18;font-size:11px;font-weight:750;letter-spacing:1px}.live i{width:7px;height:7px;border-radius:50%;background:#ff5265;box-shadow:0 0 10px #ff5265;animation:pulse 1.3s infinite}@keyframes pulse{50%{opacity:.35}}
.actions{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;padding:14px}.btn{min-height:46px;display:flex;align-items:center;justify-content:center;text-decoration:none;border:1px solid #34465d;border-radius:12px;background:#182333;color:var(--text);font-weight:680;transition:.15s transform,.15s border-color,.15s background}.btn:hover{border-color:#597696;background:#1d2c40}.btn:active{transform:scale(.98)}.btn.primary{color:#061019;border-color:transparent;background:linear-gradient(135deg,var(--cyan),#66a0ff)}.btn.busy{opacity:.65;pointer-events:none}.btn.ghost{min-height:36px;padding:0 11px;font-size:12px;background:transparent}
.content{padding:15px}.health{display:grid;grid-template-columns:repeat(3,1fr);gap:9px}.healthitem{padding:12px;border-radius:13px;background:#0b111a;border:1px solid #1d2938}.healthitem span{display:block;margin-bottom:5px;color:var(--muted);font-size:11px}.healthitem strong{font-size:13px}.good{color:var(--green)!important}.danger{color:var(--red)!important}.warn{color:var(--amber)!important}
.metrics{display:grid;grid-template-columns:1fr 1fr;gap:9px}.metric{min-height:82px;padding:13px;border:1px solid #202d3d;border-radius:13px;background:#0c131d}.metric label{display:block;color:var(--muted);font-size:11px;margin-bottom:8px}.metric strong{display:block;font-size:18px;font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.metric small{display:block;color:#667991;margin-top:4px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.wide{grid-column:1/-1}.footerline{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-top:12px;padding-top:12px;border-top:1px solid var(--line);color:var(--muted);font-size:12px}.diag{color:#aab9ca;overflow-wrap:anywhere}
.toast{position:fixed;z-index:20;left:50%;bottom:22px;max-width:min(440px,calc(100% - 30px));padding:11px 15px;border:1px solid #38506b;border-radius:12px;background:#172334;color:#fff;box-shadow:0 14px 40px #0008;transform:translate(-50%,20px);opacity:0;pointer-events:none;transition:.2s}.toast.show{transform:translate(-50%,0);opacity:1}.toast.error{border-color:#7b3441;background:#351820}
@media(max-width:850px){.topbar{height:60px}.brand small{display:none}.online span{display:none}.online{padding:9px}main{grid-template-columns:1fr;padding:14px;gap:14px}.camera{grid-row:auto}.stage{min-height:220px}.side{display:grid;grid-template-columns:1fr 1fr;gap:14px}.side>.card{height:100%}}
@media(max-width:600px){.link{padding:0 11px}.link span{display:none}main{padding:10px}.side{display:grid;grid-template-columns:1fr}.card{border-radius:15px}.stage{min-height:190px}.health{grid-template-columns:repeat(3,1fr)}.healthitem{padding:10px 8px}.metrics{gap:7px}.metric{padding:11px;min-height:76px}.metric strong{font-size:16px}.actions{padding:10px}.cardhead{padding:0 14px}}
</style>
</head>
<body>
<header class="topbar">
  <div class="brand"><div class="logo">L</div><div>Laser Fusion<small>ESP32-S3 control</small></div></div>
  <nav class="nav"><div id="connection" class="online"><i class="dot"></i><span>连接中</span></div><a class="link" href="/files" aria-label="打开文件管理"><b>SD</b><span>文件管理</span></a></nav>
</header>
<main>
  <section class="card camera">
    <div class="cardhead"><h2>实时取景</h2><span id="cameraMeta" class="hint">等待视频流</span></div>
    <div class="stage"><img id="stream" src="/stream" alt="摄像头实时画面"><div class="live"><i></i> LIVE</div></div>
    <div class="actions"><button id="measureBtn" class="btn primary" onclick="command('/api/measure',this,'测量已开始')">开始测量</button><button id="captureBtn" class="btn" onclick="command('/api/capture',this,'拍照请求已发送')">拍照保存</button><a class="btn" href="/jpg?download=1">原始 JPEG</a></div>
  </section>
  <div class="side">
    <section class="card">
      <div class="cardhead"><h2>设备状态</h2><span id="uptime" class="hint">--</span></div>
      <div class="content">
        <div class="health">
          <div class="healthitem"><span>摄像头</span><strong id="cameraState">--</strong></div>
          <div class="healthitem"><span>SD 卡</span><strong id="sdState">--</strong></div>
          <div class="healthitem"><span>I²C</span><strong id="i2cState">--</strong></div>
        </div>
        <div class="footerline"><span id="runtime">等待状态</span><span id="photoCount">照片 --</span></div>
      </div>
    </section>
    <section class="card">
      <div class="cardhead"><h2>关键传感数据</h2><span id="bnoState" class="hint">BNO --</span></div>
      <div class="content">
        <div class="metrics">
          <div class="metric"><label>激光测距</label><strong id="laser">--</strong><small>mm</small></div>
          <div class="metric"><label>IMU Y / P / R</label><strong id="angles">--</strong><small id="confidence">confidence --</small></div>
          <div class="metric"><label>加速度</label><strong id="accel">--</strong><small>m/s² · X / Y / Z</small></div>
          <div class="metric"><label>陀螺仪</label><strong id="gyro">--</strong><small>rad/s · X / Y / Z</small></div>
          <div class="metric wide"><label>位姿 / Quaternion</label><strong id="quat">--</strong><small id="path">trajectory --</small></div>
        </div>
        <div class="footerline"><span id="imuDetail">IMU only</span><button class="btn ghost" onclick="runDiag()">I2C diag</button></div>
        <p id="diag" class="diag hint"></p>
      </div>
    </section>
  </div>
</main>
<div id="toast" class="toast"></div>
<script>
const $=id=>document.getElementById(id),num=(v,n=2)=>Number.isFinite(Number(v))?Number(v).toFixed(n):'--';
let toastTimer,pollTimer;
function toast(message,error=false){const el=$('toast');el.textContent=message;el.className='toast show'+(error?' error':'');clearTimeout(toastTimer);toastTimer=setTimeout(()=>el.className='toast',2400)}
function setHealth(id,ok,yes,no){const el=$(id);el.textContent=ok?yes:no;el.className=ok?'good':'danger'}
function formatUptime(ms){let sec=Math.floor((ms||0)/1000),h=Math.floor(sec/3600),m=Math.floor(sec%3600/60);return h?h+'h '+m+'m':m+'m '+sec%60+'s'}
async function command(url,button,message){button.classList.add('busy');try{const r=await fetch(url,{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);toast(message)}catch(e){toast('操作失败：'+e.message,true)}finally{setTimeout(()=>button.classList.remove('busy'),450)}}
async function runDiag(){const out=$('diag');out.textContent='正在检测…';try{const r=await fetch('/api/i2c_diag',{cache:'no-store'}),j=await r.json();out.textContent=j.detail||j.verdict||'检测完成';toast(j.ok?'I²C 总线正常':'I²C 总线异常',!j.ok)}catch(e){out.textContent='诊断失败：'+e.message}}
function applyState(s){
  $('connection').className='online ok';$('connection').querySelector('span').textContent='设备在线';
  setHealth('cameraState',!!s.camera,'在线','离线');setHealth('sdState',!!s.sd,'已挂载','未挂载');
  const i2c=!!(s.i2c&&s.i2c.sda&&s.i2c.scl);setHealth('i2cState',i2c,'正常','异常');
  $('uptime').textContent='运行 '+formatUptime(s.uptime_ms);$('runtime').textContent=s.runtime?'采集中':'待机';
  $('photoCount').textContent='照片 '+((s.photo&&s.photo.count)||0);$('cameraMeta').textContent=s.camera?'640 × 480 · JPEG':'摄像头离线';
  const laser=s.laser||{};$('laser').textContent=laser.busy?'测量中':(laser.distance_mm??'--');
  const fusion=s.fusion||{},angles=fusion.ypr_rad||[];$("angles").textContent=angles.map(v=>num(v,2)).join(" / ")||"--";$("confidence").textContent="confidence "+num(fusion.confidence,2);$("imuDetail").textContent=fusion.stationary?"IMU stable":"IMU updating";
  const b=s.bno||{},a=b.accel||[],g=b.gyro||[],q=b.quat||[];$("accel").textContent=a.map(v=>num(v,1)).join(" / ")||"--";$("gyro").textContent=g.map(v=>num(v,2)).join(" / ")||"--";$("quat").textContent=q.map(v=>num(v,2)).join(" / ")||"--";$("bnoState").textContent=b.ok?"BNO OK":"BNO error";$("bnoState").className="hint "+(b.ok?"good":"danger");$("path").textContent="origin "+((fusion.position_m||[]).map(v=>num(v,2)).join(" / ")||"--");
  $('measureBtn').classList.toggle('busy',!!laser.busy);
}
async function poll(){try{const r=await fetch('/api/state',{cache:'no-store'});if(!r.ok)throw Error(r.status);applyState(await r.json())}catch(e){$('connection').className='online bad';$('connection').querySelector('span').textContent='连接断开'}finally{pollTimer=setTimeout(poll,600)}}
async function syncTime(){try{await fetch('/api/time?epoch_ms='+Date.now(),{cache:'no-store'})}catch(e){}}
$('stream').addEventListener('load',()=>{$('cameraMeta').textContent='实时视频已连接'});$('stream').addEventListener('error',()=>{$('cameraMeta').textContent='视频流连接失败'});
document.addEventListener('visibilitychange',()=>{if(!document.hidden){clearTimeout(pollTimer);syncTime();poll()}});syncTime();poll();
</script>
</body>
</html>
)WEBPAGE";

static constexpr char kFileManagerHtml[] = R"WEBPAGE(
<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0a0f18">
<title>Laser · SD 文件管理</title>
<style>
:root{color-scheme:dark;--bg:#080c13;--panel:#101722;--line:#243144;--text:#eef4fb;--muted:#8fa0b5;--cyan:#43d7e8;--blue:#628eff;--green:#49d49d;--red:#ff6b79;--r:17px}
*{box-sizing:border-box}html{background:var(--bg)}body{margin:0;min-height:100vh;color:var(--text);background:radial-gradient(800px 480px at 80% -20%,#17375c66,transparent),var(--bg);font:14px/1.45 system-ui,-apple-system,"Segoe UI","Microsoft YaHei",sans-serif;-webkit-font-smoothing:antialiased}button,input,textarea{font:inherit}button{cursor:pointer}
.topbar{position:sticky;top:0;z-index:10;height:66px;display:flex;align-items:center;justify-content:space-between;padding:0 max(18px,env(safe-area-inset-left));background:#090e17e8;border-bottom:1px solid #202c3c;backdrop-filter:blur(18px)}.brand{display:flex;align-items:center;gap:11px;font-weight:720}.back{width:36px;height:36px;display:grid;place-items:center;border:1px solid var(--line);border-radius:11px;color:var(--text);background:#111925;text-decoration:none;font-size:18px}.brand small{display:block;color:var(--muted);font-size:11px;font-weight:500;letter-spacing:.7px;text-transform:uppercase}.sd{display:flex;align-items:center;gap:7px;padding:8px 11px;border:1px solid var(--line);border-radius:999px;color:var(--muted);background:#101721;font-size:12px}.sd i{width:8px;height:8px;border-radius:50%;background:#ffbd5d}.sd.ok i{background:var(--green)}.sd.bad i{background:var(--red)}
main{width:min(1120px,100%);margin:auto;padding:22px}.notice{margin-bottom:12px;padding:10px 13px;border:1px solid #5f4b2a;border-radius:12px;background:#2a2114;color:#e9c98b;font-size:12px}.panel{overflow:hidden;border:1px solid var(--line);border-radius:var(--r);background:linear-gradient(155deg,#131b27,#0e151f);box-shadow:0 18px 55px #0005}.toolbar{display:flex;align-items:center;gap:9px;min-height:64px;padding:11px 15px;border-bottom:1px solid var(--line)}.btn{display:inline-flex;align-items:center;justify-content:center;gap:7px;min-height:40px;padding:0 13px;border:1px solid #34465d;border-radius:11px;background:#172231;color:var(--text);font-weight:650;white-space:nowrap}.btn:hover{border-color:#597696;background:#1c2a3d}.btn:active{transform:scale(.98)}.btn.primary{border-color:transparent;color:#061019;background:linear-gradient(135deg,var(--cyan),#6e99ff)}.btn.danger{color:#ffafb7;border-color:#69323c;background:#29161c}.btn.icon{width:40px;padding:0;font-size:17px}.btn.small{min-height:32px;padding:0 9px;border-radius:9px;font-size:12px;font-weight:600}.spacer{flex:1}.summary{color:var(--muted);font-size:12px;white-space:nowrap}
.pathbar{display:flex;align-items:center;gap:6px;min-height:52px;padding:8px 17px;border-bottom:1px solid var(--line);overflow-x:auto;scrollbar-width:none}.pathbar::-webkit-scrollbar{display:none}.crumb{border:0;background:transparent;color:#a9bdd4;padding:6px 4px;white-space:nowrap}.crumb:hover{color:var(--cyan)}.sep{color:#4c6078}.pathicon{color:var(--cyan);font-weight:800;margin-right:2px}
.listhead,.row{display:grid;grid-template-columns:minmax(180px,1fr) 110px 198px;align-items:center;gap:12px}.listhead{min-height:38px;padding:0 17px;color:#687c94;font-size:11px;text-transform:uppercase;letter-spacing:.8px;border-bottom:1px solid #1c2736}.row{min-height:60px;padding:7px 17px;border-bottom:1px solid #1b2634;transition:.12s background}.row:last-child{border-bottom:0}.row:hover{background:#151f2d}.name{min-width:0;display:flex;align-items:center;gap:12px;border:0;background:none;color:var(--text);padding:0;text-align:left}.fileicon{flex:0 0 auto;width:34px;height:34px;display:grid;place-items:center;border:1px solid #2a3a4f;border-radius:10px;background:#111a26;color:#91a9c4;font-size:16px}.folder .fileicon{border-color:#325268;background:#102430;color:#54d3df}.nameText{min-width:0}.nameText b{display:block;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:13px}.nameText small{display:none;color:var(--muted)}.size{color:var(--muted);font-size:12px;font-variant-numeric:tabular-nums}.rowactions{display:flex;justify-content:flex-end;gap:6px}.empty{padding:80px 20px;text-align:center;color:var(--muted)}.empty b{display:block;margin-bottom:7px;color:var(--text);font-size:17px}.loading{padding:60px;text-align:center;color:var(--muted)}
.overlay{position:fixed;z-index:30;inset:0;display:grid;place-items:center;padding:18px;background:#03060ab8;backdrop-filter:blur(6px)}.overlay[hidden]{display:none}.modal{width:min(860px,100%);max-height:calc(100vh - 36px);display:flex;flex-direction:column;overflow:hidden;border:1px solid #344861;border-radius:18px;background:#111925;box-shadow:0 24px 80px #000b}.modalhead{min-height:59px;display:flex;align-items:center;justify-content:space-between;gap:12px;padding:0 15px;border-bottom:1px solid var(--line)}.modalhead h2{margin:0;font-size:14px}.modalhead p{margin:2px 0 0;max-width:620px;color:var(--muted);font-size:11px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.close{width:36px;height:36px;border:0;border-radius:10px;background:#1b2737;color:#b7c5d5;font-size:20px}.editor{display:block;width:100%;min-height:420px;flex:1;resize:none;padding:16px;border:0;outline:0;color:#e8eef6;background:#080d14;font:13px/1.55 ui-monospace,SFMono-Regular,Consolas,monospace;tab-size:2}.modalfoot{display:flex;align-items:center;gap:9px;padding:11px 15px;border-top:1px solid var(--line)}.modalfoot span{flex:1;color:var(--muted);font-size:11px}
.promptbox{width:min(420px,100%);padding:19px;border:1px solid #344861;border-radius:17px;background:#111925;box-shadow:0 24px 80px #000b}.promptbox h2{margin:0 0 6px;font-size:17px}.promptbox p{margin:0 0 15px;color:var(--muted);font-size:12px}.input{width:100%;height:44px;padding:0 12px;outline:0;border:1px solid #34465d;border-radius:11px;background:#080d14;color:var(--text)}.input:focus{border-color:var(--cyan);box-shadow:0 0 0 3px #43d7e818}.promptactions{display:flex;justify-content:flex-end;gap:8px;margin-top:15px}
.toast{position:fixed;z-index:50;left:50%;bottom:22px;max-width:min(460px,calc(100% - 30px));padding:11px 15px;border:1px solid #38506b;border-radius:12px;background:#172334;color:#fff;box-shadow:0 14px 40px #0008;transform:translate(-50%,20px);opacity:0;pointer-events:none;transition:.2s}.toast.show{transform:translate(-50%,0);opacity:1}.toast.error{border-color:#7b3441;background:#351820}
@media(max-width:720px){.topbar{height:60px}.brand small{display:none}main{padding:10px}.panel{border-radius:15px}.toolbar{flex-wrap:wrap}.summary{width:100%;order:3}.listhead{display:none}.row{grid-template-columns:minmax(0,1fr);gap:5px;padding:10px 13px}.size{display:none}.nameText small{display:block}.rowactions{justify-content:flex-start;padding-left:46px}.btn.small{min-height:34px}.editor{min-height:55vh}.sd span{display:none}.sd{padding:9px}}
@media(max-width:390px){.toolbar .btn span{display:none}.toolbar .btn{width:40px;padding:0}.rowactions{padding-left:0;display:grid;grid-template-columns:repeat(4,1fr)}.rowactions .btn{padding:0 5px}.modalfoot span{display:none}}
</style>
</head>
<body>
<header class="topbar">
  <div class="brand"><a class="back" href="/" aria-label="返回控制台">‹</a><div>SD 文件管理<small>浏览 · 编辑 · 导出</small></div></div>
  <div id="sdStatus" class="sd"><i></i><span>正在读取</span></div>
</header>
<main>
  <div class="notice">设备热点不提供互联网，系统显示“无互联网”是正常的。请选择“仍然使用此 Wi-Fi”，然后从本页面下载文件。</div>
  <section class="panel">
    <div class="toolbar">
      <button id="upBtn" class="btn icon" onclick="goUp()" title="上级目录">↑</button>
      <button class="btn primary" onclick="showNameDialog('mkdir')"><b>＋</b><span>新建文件夹</span></button>
      <button class="btn" onclick="loadFiles()"><b>↻</b><span>刷新</span></button>
      <div class="spacer"></div><span id="summary" class="summary">--</span>
    </div>
    <nav id="breadcrumbs" class="pathbar" aria-label="当前路径"></nav>
    <div class="listhead"><span>名称</span><span>大小</span><span></span></div>
    <div id="fileList"><div class="loading">正在读取 SD 卡…</div></div>
  </section>
</main>

<div id="editorOverlay" class="overlay" hidden>
  <section class="modal" role="dialog" aria-modal="true" aria-labelledby="editorTitle">
    <header class="modalhead"><div><h2 id="editorTitle">文本编辑器</h2><p id="editorPath"></p></div><button class="close" onclick="closeEditor()" aria-label="关闭">×</button></header>
    <textarea id="editorText" class="editor" spellcheck="false"></textarea>
    <footer class="modalfoot"><span>仅支持不超过 64 KiB 的文本文件</span><button class="btn" onclick="closeEditor()">取消</button><button id="saveBtn" class="btn primary" onclick="saveEditor()">保存</button></footer>
  </section>
</div>

<div id="promptOverlay" class="overlay" hidden>
  <form class="promptbox" onsubmit="submitName(event)"><h2 id="promptTitle"></h2><p id="promptHelp"></p><input id="nameInput" class="input" maxlength="96" autocomplete="off"><div class="promptactions"><button type="button" class="btn" onclick="closeNameDialog()">取消</button><button class="btn primary">确认</button></div></form>
</div>
<div id="toast" class="toast"></div>
<script>
const $=id=>document.getElementById(id),enc=encodeURIComponent,MAX_EDIT=65536;
let cwd='/',items=[],editing='',dialogMode='',dialogItem=null,toastTimer,loading=false;
function toast(message,error=false){const el=$('toast');el.textContent=message;el.className='toast show'+(error?' error':'');clearTimeout(toastTimer);toastTimer=setTimeout(()=>el.className='toast',2500)}
function prettySize(n){n=Number(n)||0;if(n<1024)return n+' B';if(n<1048576)return (n/1024).toFixed(n<10240?1:0)+' KiB';if(n<1073741824)return (n/1048576).toFixed(1)+' MiB';return (n/1073741824).toFixed(1)+' GiB'}
function joinPath(base,name){return (base==='/'?'':base)+'/'+name}
function validName(name){return !!name&&name!=='.'&&name!=='..'&&!name.includes('/')&&!name.includes('\\')}
function escapeHtml(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function renderCrumbs(){const nav=$('breadcrumbs'),parts=cwd.split('/').filter(Boolean);let html='<span class="pathicon">SD</span><button class="crumb" onclick="openDir(\'/\')">根目录</button>',path='';for(const part of parts){path+='/'+part;html+='<span class="sep">/</span><button class="crumb" data-path="'+escapeHtml(path)+'">'+escapeHtml(part)+'</button>'}nav.innerHTML=html;nav.querySelectorAll('[data-path]').forEach(b=>b.onclick=()=>openDir(b.dataset.path));$('upBtn').disabled=cwd==='/'}
function iconFor(item){if(item.dir)return '▰';const ext=item.name.split('.').pop().toLowerCase();if(['jpg','jpeg','png','bmp'].includes(ext))return '▧';if(['txt','csv','json','log','md','ini','cfg'].includes(ext))return '≡';return '·'}
function canEdit(item){if(item.dir||item.size>MAX_EDIT)return false;const ext=item.name.includes('.')?item.name.split('.').pop().toLowerCase():'';return ['txt','csv','json','log','md','ini','cfg','xml','html','css','js','c','cpp','h','py',''].includes(ext)}
function renderFiles(){const list=$('fileList'),dirs=items.filter(x=>x.dir).length,files=items.length-dirs,total=items.reduce((n,x)=>n+(x.dir?0:Number(x.size)||0),0);$('summary').textContent=dirs+' 个文件夹 · '+files+' 个文件 · '+prettySize(total);renderCrumbs();if(!items.length){list.innerHTML='<div class="empty"><b>这个文件夹是空的</b>可新建文件夹，或回到设备拍摄照片。</div>';return}items.sort((a,b)=>Number(b.dir)-Number(a.dir)||a.name.localeCompare(b.name));list.innerHTML=items.map((it,i)=>'<div class="row '+(it.dir?'folder':'')+'"><button class="name" data-open="'+i+'"><span class="fileicon">'+iconFor(it)+'</span><span class="nameText"><b>'+escapeHtml(it.name)+'</b><small>'+(it.dir?'文件夹':prettySize(it.size))+'</small></span></button><span class="size">'+(it.dir?'--':prettySize(it.size))+'</span><span class="rowactions">'+(!it.dir&&canEdit(it)?'<button class="btn small" data-edit="'+i+'">编辑</button>':'')+(!it.dir?'<button class="btn small" data-download="'+i+'">下载</button>':'')+'<button class="btn small" data-rename="'+i+'">重命名</button><button class="btn small danger" data-delete="'+i+'">删除</button></span></div>').join('');
  list.querySelectorAll('[data-open]').forEach(b=>b.onclick=()=>{const it=items[+b.dataset.open];it.dir?openDir(it.path):(canEdit(it)?openEditor(it):downloadItem(it))});list.querySelectorAll('[data-edit]').forEach(b=>b.onclick=()=>openEditor(items[+b.dataset.edit]));list.querySelectorAll('[data-download]').forEach(b=>b.onclick=()=>downloadItem(items[+b.dataset.download]));list.querySelectorAll('[data-rename]').forEach(b=>b.onclick=()=>showNameDialog('rename',items[+b.dataset.rename]));list.querySelectorAll('[data-delete]').forEach(b=>b.onclick=()=>deleteItem(items[+b.dataset.delete]));
}
async function responseError(r){try{const j=await r.json();return j.error||('HTTP '+r.status)}catch(e){return 'HTTP '+r.status}}
async function loadFiles(){if(loading)return;loading=true;try{const r=await fetch('/api/files?path='+enc(cwd),{cache:'no-store'});if(!r.ok)throw Error(await responseError(r));const j=await r.json();if(!j.ok)throw Error(j.error||'读取失败');cwd=j.path||cwd;items=j.items||[];$('sdStatus').className='sd ok';$('sdStatus').querySelector('span').textContent='SD 已挂载';renderFiles()}catch(e){items=[];$('sdStatus').className='sd bad';$('sdStatus').querySelector('span').textContent='SD 不可用';$('fileList').innerHTML='<div class="empty"><b>无法读取 SD 卡</b>'+escapeHtml(e.message)+'</div>';$('summary').textContent='读取失败';renderCrumbs();toast(e.message,true)}finally{loading=false}}
function openDir(path){cwd=path||'/';$('fileList').innerHTML='<div class="loading">正在读取…</div>';loadFiles()}function goUp(){if(cwd!=='/'){openDir(cwd.substring(0,cwd.lastIndexOf('/'))||'/')}}
async function fsOp(action,path,dest='',body=null){let url='/api/fs?action='+action+'&path='+enc(path)+(dest?'&dest='+enc(dest):'');const options={method:'POST'};if(body!==null){options.body=body;options.headers={'Content-Type':'text/plain;charset=UTF-8'}}const r=await fetch(url,options);if(!r.ok)throw Error(await responseError(r));const j=await r.json();if(!j.ok)throw Error(j.error||'操作失败');return j}
function showNameDialog(mode,item=null){dialogMode=mode;dialogItem=item;$('promptTitle').textContent=mode==='mkdir'?'新建文件夹':'重命名';$('promptHelp').textContent=mode==='mkdir'?'将在当前目录中创建文件夹。':'输入新的文件或文件夹名称。';$('nameInput').value=item?item.name:'';$('promptOverlay').hidden=false;setTimeout(()=>{$('nameInput').focus();$('nameInput').select()},30)}
function closeNameDialog(){$('promptOverlay').hidden=true;dialogItem=null}
async function submitName(e){e.preventDefault();const name=$('nameInput').value.trim();if(!validName(name)){toast('名称不能为空，且不能包含 / 或 \\',true);return}try{if(dialogMode==='mkdir'){await fsOp('mkdir',joinPath(cwd,name));toast('文件夹已创建')}else if(dialogItem&&name!==dialogItem.name){await fsOp('rename',dialogItem.path,joinPath(cwd,name));toast('重命名完成')}closeNameDialog();await loadFiles()}catch(err){toast(err.message,true)}}
async function deleteItem(item){if(!confirm('确定删除“'+item.name+'”？'+(item.dir?'\n文件夹必须为空。':'')))return;try{await fsOp('delete',item.path);toast('已删除 '+item.name);await loadFiles()}catch(e){toast(e.message,true)}}
function downloadItem(item){window.location.assign('/api/file?download=1&path='+enc(item.path))}
async function openEditor(item){if(item.size>MAX_EDIT){toast('文件超过 64 KiB，无法在线编辑',true);return}try{const r=await fetch('/api/file?path='+enc(item.path),{cache:'no-store'});if(!r.ok)throw Error(await r.text()||('HTTP '+r.status));editing=item.path;$('editorPath').textContent=item.path+' · '+prettySize(item.size);$('editorText').value=await r.text();$('editorOverlay').hidden=false;document.body.style.overflow='hidden';setTimeout(()=>$('editorText').focus(),30)}catch(e){toast('打开失败：'+e.message,true)}}
function closeEditor(){$('editorOverlay').hidden=true;$('editorText').value='';editing='';document.body.style.overflow=''}
async function saveEditor(){if(!editing)return;const text=$('editorText').value,bytes=new TextEncoder().encode(text).length;if(bytes>MAX_EDIT){toast('内容超过 64 KiB，无法保存',true);return}const btn=$('saveBtn');btn.disabled=true;btn.textContent='保存中…';try{await fsOp('save',editing,'',text);toast('文件已保存');closeEditor();await loadFiles()}catch(e){toast('保存失败：'+e.message,true)}finally{btn.disabled=false;btn.textContent='保存'}}
$('promptOverlay').addEventListener('click',e=>{if(e.target===$('promptOverlay'))closeNameDialog()});$('editorOverlay').addEventListener('click',e=>{if(e.target===$('editorOverlay'))closeEditor()});document.addEventListener('keydown',e=>{if(e.key==='Escape'){if(!$('promptOverlay').hidden)closeNameDialog();else if(!$('editorOverlay').hidden)closeEditor()}if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='s'&&!$('editorOverlay').hidden){e.preventDefault();saveEditor()}});fetch('/api/time?epoch_ms='+Date.now(),{cache:'no-store'}).catch(()=>{});loadFiles();
</script>
</body>
</html>
)WEBPAGE";

}  // namespace web_ui
