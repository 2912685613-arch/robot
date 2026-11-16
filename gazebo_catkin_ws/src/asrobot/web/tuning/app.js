'use strict';
const $ = id => document.getElementById(id);
const palette = {teal:'#128f87', blue:'#5b86c7', amber:'#c5a062', violet:'#a282b8', gray:'#b5bfd0', ink:'#7a8b99'};
let server = null, draft = {}, effective = {}, dirty = false, busy = false, seq = 0, rows = [], baseline = null, freeze = false;
let axis = 'distance', windowSeconds = 60, toastTimer, polling = false;
const mainChart = {points:[], plot:null};
const esc = text => String(text ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const number = (v, digits=3) => Number.isFinite(v) ? v.toFixed(digits) : '—';
const unitFor = () => axis === 'yaw' ? '°' : 'm';
function toast(message,error=false) {
  clearTimeout(toastTimer); $('toast').textContent=message; $('toast').className='toast'+(error?' error':'');
  toastTimer=setTimeout(()=>$('toast').classList.add('hidden'),5000);
}
async function request(path, body) {
  const response=await fetch(path,body===undefined ? {} : {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const data=await response.json();
  if(!response.ok) throw new Error(data.error || '请求失败');
  return data;
}
function changed() {
  dirty=Object.keys(draft).some(k=>draft[k]!==effective[k]);
  $('dirtyLabel').textContent=dirty?'有待应用修改':'与节点一致';
  $('dirtyLabel').style.color=dirty?palette.teal:'';
  $('applyButton').disabled=busy || !dirty || !server?.connected;
}
function buildParameters(schema) {
  const groups=[...new Set(schema.map(p=>p.group))];
  $('parameterGroups').innerHTML=groups.map((group,index)=>`<details class="param-group" ${index<3?'open':''}><summary>${esc(group)}</summary>${schema.filter(p=>p.group===group).map(p=>`<label class="param-row" data-search="${esc(p.label+' '+p.key)}"><span class="param-label">${esc(p.label)}<small>${esc(p.key)}</small></span><span class="input-unit"><input class="param-input" type="number" id="param-${p.key}" name="${p.key}" min="${p.min}" max="${p.max}" step="${p.step}" aria-label="${esc(p.label)}"><span>${esc(p.unit)}</span></span></label>`).join('')}</details>`).join('');
  for(const input of document.querySelectorAll('.param-input')) input.addEventListener('input',()=>{
    draft[input.name]=input.value===''?null:Number(input.value);
    input.classList.toggle('dirty',draft[input.name]!==effective[input.name]); changed();
  });
}
function fillParameters(config) {
  draft={...config};
  for(const [key,value] of Object.entries(config)) {
    const input=$('param-'+key); if(input){input.value=value;input.classList.toggle('dirty',value!==effective[key]);}
  }
  changed();
}
function refreshStatus() {
  const latest=server.latest, connected=server.connected;
  $('connectionDot').classList.toggle('live',connected);
  $('connectionText').textContent=connected?'已连接 · motion 遥测':(latest?'遥测已过期':'等待 motion 节点');
  $('modeTag').textContent=server.demo?'DEMO 演示':'ROS 实时';
  $('demoNotice').classList.toggle('hidden',!server.demo);
  $('revision').textContent='v'+server.revision;
  const state=connected?latest.state:'未连接';
  $('stateBadge').textContent=state;
  $('stateBadge').className='status-badge'+(connected&&latest.active?' active':state.includes('LOST')||state.includes('WAIT')?' warning':'');
  $('applyHint').textContent=connected?`修改后点击应用 · 当前生效 v${server.revision}`:'先启动仿真和新编译的 motion 节点';
  for(const id of ['dockButton','resumeButton','resetButton']) $(id).disabled=busy||!connected;
  $('stopButton').disabled=busy;
  $('recordButton').disabled=busy||!connected||!!server.trial;
  $('finishButton').disabled=busy||!server.trial;
  $('recordDot').classList.toggle('on',!!server.trial);
  $('recordText').textContent=server.trial?`${server.trial.label} · 记录中 · ${server.trial.count} 条`:'未在记录 · 可先调整参数';
  $('ageBadge').textContent=connected?`遥测 ${number(server.age,1)} s 前`:'无实时数据';
  $('velocityLive').textContent=connected&&latest.valid?`${number(latest.vz,3)} m/s`:'—';
  $('validText').textContent=connected?(latest.valid?'视觉与反馈有效':`测量不可用 · ${latest.reason}`):'未连接';
  $('storagePath').textContent=server.log_dir;
  $('storagePath').title=server.log_dir;
  $('eventsList').innerHTML=[...server.events].reverse().slice(0,12).map(e=>`<div class="event"><time>${esc(e.wall)}</time><span>${esc(e.message)}</span></div>`).join('');
  changed();
}
function renderMetrics() {
  const trial=server?.trial || server?.history?.[0];
  const m=trial?.metrics?.[axis];
  const put=(id,value,unit,precision=2)=>$(id).innerHTML=Number.isFinite(value)?`${number(value,precision)}<small>${esc(unit)}</small>`:'—';
  put('overshootValue',m?.overshoot,unitFor(),axis==='yaw'?2:4);
  $('overshootDetail').textContent=Number.isFinite(m?.overshoot_pct)?`${number(m.overshoot_pct,1)}% · 相对初始阶跃幅值`:'无有效阶跃，暂不计算百分比';
  put('riseValue',m?.rise_time,'s');put('settlingValue',m?.settling_time,'s');
  $('settlingDetail').textContent=m?.settling_time!=null?(m.provisional?'暂定值，后续超差会撤销':'本段已结束'):(m?.status||'进入误差带并持续稳定');
  put('errorValue',m?.steady_error,unitFor(),axis==='yaw'?2:4);
  $('errorDetail').textContent=m?`末尾窗口均值 · RMSE ${number(m.rmse,axis==='yaw'?2:4)}`:'目标 − 测量；不等同于已稳态';
  $('sampleQuality').textContent=m?`${m.n} 个源采样 · ${m.quality}`:'开始记录后计算指标';
  const titles={distance:'高度阶跃响应',x:'X 方向对准响应',y:'Y 方向对准响应',yaw:'航向相对目标响应'};
  $('responseTitle').textContent=titles[axis];
  $('metricScope').textContent=axis==='distance'?'指标从首次 APPROACH 开始 · 高度为机体原点到标记间距':'指标从记录的首个有效源采样开始 · 目标为零';
  renderHistory();
}
function renderHistory() {
  if(!server?.history?.length) return;
  $('historyBody').innerHTML=server.history.map(t=>{
    const m=t.metrics[axis]; const f=(v,d=2)=>Number.isFinite(v)?number(v,d):'—';
    return `<tr class="${baseline?.id===t.id?'selected':''}"><td>${esc(t.label)}<small>v${esc(t.revision||'—')} · ${t.count} 条采样</small></td><td>${f(m.overshoot,4)} ${unitFor()}<small>${f(m.overshoot_pct,1)}%</small></td><td>${f(m.rise_time)} s</td><td>${f(m.settling_time)} s</td><td>${f(m.steady_error,4)} ${unitFor()}</td><td><button class="text-button reuse" data-id="${esc(t.id)}">复用参数</button><button class="text-button compare" data-id="${esc(t.id)}">对比曲线</button><a class="text-button" href="/api/export.csv?trial=${encodeURIComponent(t.id)}" download="trial_${t.id}.csv">CSV</a></td></tr>`;
  }).join('');
  for(const button of document.querySelectorAll('.reuse')) button.onclick=()=>{
    const trial=server.history.find(t=>t.id===button.dataset.id);fillParameters(trial.config);toast('已填入该次试验参数，尚未应用');
  };
  for(const button of document.querySelectorAll('.compare')) button.onclick=async()=>{
    try {baseline=await request('/api/trial?id='+encodeURIComponent(button.dataset.id));$('baselineLegend').classList.remove('hidden');renderHistory();drawAll();}
    catch(e){toast(e.message,true);}
  };
}
function visibleRows() {
  if(!rows.length)return [];
  const trial=server?.trial||server?.history?.[0];
  const selected=trial&&Number.isFinite(trial.start_t)?rows.filter(r=>r.telemetry.t>=trial.start_t&&(trial.complete?r.telemetry.t<=trial.end_t:true)):rows;
  const end=selected.at(-1)?.telemetry.t||0;
  return windowSeconds ? selected.filter(r=>r.telemetry.t>=end-windowSeconds) : selected;
}
function axisValue(t) { return axis==='yaw'?-t.yaw_error:t[axis]; }
function canvasContext(id) {
  const canvas=$(id), rect=canvas.getBoundingClientRect(), dpr=window.devicePixelRatio||1;
  canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.height*dpr);
  const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);
  return {ctx,w:rect.width,h:rect.height};
}
function plot(id,series,{target=null,tolerance=null,fixedY=null,unit='',main=false}={}) {
  const {ctx,w,h}=canvasContext(id),p={l:51,r:18,t:14,b:29};
  if(w<100||h<50)return;
  const all=series.flatMap(s=>s.points.filter(v=>v.valid&&Number.isFinite(v.y)&&Number.isFinite(v.x)));
  let times=series.flatMap(s=>s.points.map(v=>v.x).filter(Number.isFinite));
  let xmin=times.length?Math.min(...times):0,xmax=times.length?Math.max(...times):60;
  if(xmax-xmin<1)xmax=xmin+1;
  let vals=all.map(v=>v.y);if(Number.isFinite(target))vals.push(target);
  let ymin=fixedY?fixedY[0]:(vals.length?Math.min(...vals):0),ymax=fixedY?fixedY[1]:(vals.length?Math.max(...vals):1);
  if(!fixedY){const span=Math.max(ymax-ymin,unit==='°'?.5:.01);ymin-=span*.15;ymax+=span*.15;}
  const px=x=>p.l+(x-xmin)/(xmax-xmin)*(w-p.l-p.r),py=y=>h-p.b-(y-ymin)/(ymax-ymin)*(h-p.t-p.b);
  ctx.font='10px system-ui';ctx.lineWidth=1;ctx.fillStyle='#91a0ad';ctx.textAlign='right';
  for(let n=0;n<=4;n++){
    const value=ymin+(ymax-ymin)*n/4,y=py(value);
    ctx.strokeStyle='#edf1f5';ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(w-p.r,y);ctx.stroke();
    ctx.fillText(number(value,Math.abs(ymax-ymin)<.1?3:Math.abs(ymax-ymin)<3?2:1),p.l-8,y+3);
  }
  ctx.textAlign='center';
  for(let n=0;n<=5;n++) {const x=xmin+(xmax-xmin)*n/5;ctx.fillText(number(x,0)+' s',px(x),h-9);}
  ctx.textAlign='left';ctx.fillText(unit,5,12);
  if(Number.isFinite(target)){
    if(Number.isFinite(tolerance)) {ctx.fillStyle='#16958a0a';ctx.fillRect(p.l,py(target+tolerance),w-p.l-p.r,py(target-tolerance)-py(target+tolerance));}
    ctx.setLineDash([4,4]);ctx.strokeStyle='#9aabb7';ctx.beginPath();ctx.moveTo(p.l,py(target));ctx.lineTo(w-p.r,py(target));ctx.stroke();ctx.setLineDash([]);
  }
  ctx.save();ctx.beginPath();ctx.rect(p.l,p.t,w-p.l-p.r,h-p.t-p.b);ctx.clip();
  for(const s of series) {
    ctx.strokeStyle=s.color;ctx.lineWidth=s.width||1.8;ctx.setLineDash(s.dash?[4,3]:[]);
    ctx.beginPath();let last=null;
    for(const point of s.points){
      if(!point.valid||!Number.isFinite(point.y)||!Number.isFinite(point.x)){last=null;continue;}
      if(!last||point.x-last.x>.7||point.x<last.x)ctx.moveTo(px(point.x),py(point.y));
      else ctx.lineTo(px(point.x),py(point.y));last=point;
    }
    ctx.stroke();ctx.setLineDash([]);
    const endpoint=s.points.filter(p=>p.valid&&Number.isFinite(p.y)).at(-1);
    if(endpoint){ctx.fillStyle=s.color;ctx.beginPath();ctx.arc(px(endpoint.x),py(endpoint.y),2.5,0,Math.PI*2);ctx.fill();}
  }
  ctx.restore();
  if(main){mainChart.plot={px,py,p,xmin,xmax,w,h};mainChart.points=series[0]?.points||[];}
}
function drawAll() {
  if(freeze||!server)return;
  const data=visibleRows(), t0=(server.trial||server.history[0])?.start_t??rows[0]?.telemetry.t??0;
  const trial=server.trial || server.history[0];
  const target=axis==='distance'?-(trial?.config||server.config).target_z:0;
  const toleranceConfig=trial?.config||server.config;
  const tol=axis==='distance'?toleranceConfig.z_tolerance:axis==='yaw'?toleranceConfig.yaw_tolerance:toleranceConfig.xy_tolerance;
  const points=data.map(r=>({x:r.telemetry.t-t0,y:axisValue(r.telemetry),valid:r.telemetry.valid}));
  let series=[{color:palette.teal,points}];
  if(baseline?.rows?.length){
    const b0=baseline.rows[0].telemetry.t;
    series.push({color:palette.gray,dash:true,points:baseline.rows.map(r=>({x:r.telemetry.t-b0,y:axisValue(r.telemetry),valid:r.telemetry.valid}))});
  }
  plot('responseChart',series,{target,tolerance:tol,unit:unitFor(),main:true});
  $('responseEmpty').classList.toggle('hidden',points.some(p=>p.valid&&Number.isFinite(p.y)));
  plot('velocityChart',[
    {color:palette.blue,points:data.map(r=>({x:r.telemetry.t-t0,y:r.telemetry.vz,valid:r.telemetry.valid}))},
    {color:'#9cabb6',dash:true,points:data.map(r=>({x:r.telemetry.t-t0,y:r.telemetry.target_vz,valid:r.telemetry.active}))}
  ],{unit:'m/s'});
  plot('motorChart',[palette.teal,palette.blue,palette.amber,palette.violet].map((color,i)=>({color,points:data.map(r=>({x:r.telemetry.t-t0,y:r.telemetry.motors?.[i],valid:!!r.telemetry.motors}))})),{unit:'输入',fixedY:[0,100]});
}
async function action(callback,message) {
  if(busy)return;busy=true;refreshStatus();
  try{await callback();toast(message);await poll();}
  catch(e){toast(e.message,true);}
  finally{busy=false;refreshStatus();}
}
async function poll() {
  if(polling)return;polling=true;
  try {
    const next=await request('/api/state?after='+seq);
    if(next.seq<seq){rows=[];seq=0;baseline=null;return;}
    if(!server){buildParameters(next.schema);effective={...next.config};fillParameters(next.config);}
    else if(!dirty && JSON.stringify(effective)!==JSON.stringify(next.config)){effective={...next.config};fillParameters(next.config);}
    server=next;effective={...next.config};seq=next.seq;
    rows.push(...next.rows);if(rows.length>12000)rows.splice(0,rows.length-12000);
    refreshStatus();renderMetrics();drawAll();
  } catch(e){if(server){server.connected=false;refreshStatus();}else $('connectionText').textContent='本地服务未连接';}
  finally{polling=false;}
}
$('parameterSearch').oninput=event=>{
  const q=event.target.value.trim().toLowerCase();
  for(const row of document.querySelectorAll('.param-row')) row.classList.toggle('hidden',!row.dataset.search.toLowerCase().includes(q));
  for(const group of document.querySelectorAll('.param-group')){group.classList.toggle('hidden',![...group.querySelectorAll('.param-row')].some(r=>!r.classList.contains('hidden')));if(q)group.open=true;}
};
$('readButton').onclick=()=>{fillParameters(effective);toast('已读取 motion 确认的生效参数');};
$('defaultButton').onclick=()=>{fillParameters(server.defaults);toast('已填入默认值，尚未应用');};
$('applyButton').onclick=()=>action(async()=>{const result=await request('/api/apply',{config:draft});effective={...result.config};fillParameters(result.config);},'参数已应用，重新开始对接后使用新参数');
for(const [id,command] of Object.entries({dockButton:'dock',stopButton:'stop',resumeButton:'resume',resetButton:'reset'}))
  $(id).onclick=()=>action(()=>request('/api/command',{command}),({dock:'已发送 dock，开始记录',stop:'已停止并暂停',resume:'仿真已恢复，未自动发送 dock',reset:'已重置机器人并暂停'})[command]);
$('recordButton').onclick=()=>action(()=>request('/api/trial/start',{}),'已开始记录');
$('finishButton').onclick=()=>action(()=>request('/api/trial/finish',{}),'记录已结束并保存');
$('axisSelect').onchange=event=>{axis=event.target.value;renderMetrics();drawAll();};
$('windowSelect').onchange=event=>{windowSeconds=Number(event.target.value);drawAll();};
$('freezeChart').onclick=()=>{freeze=!freeze;$('freezeChart').classList.toggle('on',freeze);$('freezeChart').textContent=freeze?'▶':'Ⅱ';if(!freeze)drawAll();toast(freeze?'仅冻结图表，机器人仍按原状态运行':'图表已恢复');};
$('clearBaseline').onclick=()=>{baseline=null;$('baselineLegend').classList.add('hidden');renderHistory();drawAll();};
$('responseChart').addEventListener('mousemove',event=>{
  if(!mainChart.plot)return;const rect=event.target.getBoundingClientRect(),mx=event.clientX-rect.left;
  const p=mainChart.plot;const tx=p.xmin+(mx-p.p.l)/(p.w-p.p.l-p.p.r)*(p.xmax-p.xmin);
  let best=null;for(const v of mainChart.points)if(v.valid&&Number.isFinite(v.y)&&(!best||Math.abs(v.x-tx)<Math.abs(best.x-tx)))best=v;
  if(best){const tt=$('chartTooltip');tt.textContent=`t  ${number(best.x,2)} s\n值 ${number(best.y,4)} ${unitFor()}`;tt.style.left=Math.max(5,Math.min(rect.width-130,mx+12))+'px';tt.style.top='20px';tt.classList.remove('hidden');}
});
$('responseChart').addEventListener('mouseleave',()=>$('chartTooltip').classList.add('hidden'));
new ResizeObserver(()=>drawAll()).observe($('responseChart').parentElement);
for(const id of ['applyButton','dockButton','stopButton','resumeButton','resetButton','recordButton','finishButton'])$(id).disabled=true;
poll();setInterval(()=>{if(!busy)poll();},500);
