'use strict';

const $ = id => document.getElementById(id);
const { clamp, wrapDeg } = ShipModel;
const deg2rad = d => d * Math.PI / 180;

const ui = {
  startBtn:$('startBtn'), pauseBtn:$('pauseBtn'), resetBtn:$('resetBtn'), instantBtn:$('instantBtn'), articleBtn:$('articleBtn'),
  speedRange:$('speedRange'), speedOut:$('speedOut'),
  initialCourse:$('initialCourse'), targetCourse:$('targetCourse'), targetStepTime:$('targetStepTime'), shipSpeed:$('shipSpeed'), duration:$('duration'),
  controllerType:$('controllerType'), kp:$('kp'), ki:$('ki'), kd:$('kd'),
  nomotoK:$('nomotoK'), nomotoT:$('nomotoT'), disturbanceType:$('disturbanceType'), disturbance:$('disturbance'), disturbanceStart:$('disturbanceStart'),
  disturbanceDuration:$('disturbanceDuration'), disturbancePeriod:$('disturbancePeriod'), rudderLimit:$('rudderLimit'), rudderRate:$('rudderRate'),
  runDot:$('runDot'), runStatus:$('runStatus'), simClock:$('simClock'),
  mCourse:$('mCourse'), mTarget:$('mTarget'), mError:$('mError'), mRudder:$('mRudder'), mYaw:$('mYaw'), mDist:$('mDist'),
  qRise:$('qRise'), qOvershoot:$('qOvershoot'), qSettling:$('qSettling'), qMaxDev:$('qMaxDev'), qRecovery:$('qRecovery'), qSteady:$('qSteady'), qMaxRudder:$('qMaxRudder'), qIAE:$('qIAE'),
  compareBtn:$('compareBtn'), articleCompareBtn:$('articleCompareBtn'), csvBtn:$('csvBtn'), pngBtn:$('pngBtn'), compareBody:$('compareBody'),
  verifyBtn:$('verifyBtn'), verifyBody:$('verifyBody'), verifySummary:$('verifySummary'),
  autotuneBtn:$('autotuneBtn'), applyOptimizedBtn:$('applyOptimizedBtn'), compareTuneBtn:$('compareTuneBtn'), autotuneStatus:$('autotuneStatus'),
  autotuneBadge:$('autotuneBadge'), jManual:$('jManual'), jOptimized:$('jOptimized'), jGain:$('jGain'), gainResult:$('gainResult'), tuneBody:$('tuneBody')
};

const canvases = {
  vessel:$('vesselCanvas'), course:$('courseCanvas'), error:$('errorCanvas'), rudder:$('rudderCanvas'), compare:$('compareCanvas')
};

function params(controllerOverride = null) {
  return ShipModel.normalizeParams({
    initialCourse:+ui.initialCourse.value,
    targetCourse:+ui.targetCourse.value,
    targetStepTime:+ui.targetStepTime.value,
    shipSpeed:+ui.shipSpeed.value,
    duration:+ui.duration.value,
    dt:0.05,
    controller:controllerOverride || ui.controllerType.value,
    kp:+ui.kp.value,
    ki:+ui.ki.value,
    kd:+ui.kd.value,
    K:+ui.nomotoK.value,
    T:+ui.nomotoT.value,
    distType:ui.disturbanceType.value,
    distAmp:+ui.disturbance.value,
    distStart:+ui.disturbanceStart.value,
    distDuration:+ui.disturbanceDuration.value,
    distPeriod:+ui.disturbancePeriod.value,
    rudderLimit:+ui.rudderLimit.value,
    rudderRate:+ui.rudderRate.value
  });
}

class Simulator {
  constructor() { this.reset(); }
  reset() {
    const p = params();
    this.state = ShipModel.createState(p);
    this.history = [ShipModel.sampleState(this.state, p)];
    this.running = false;
    this.finished = false;
  }
  step() {
    const p = params();
    if (this.state.t >= p.duration - p.dt / 2) {
      this.running = false;
      this.finished = true;
      updateStatus('Завершено', false);
      return;
    }
    this.state = ShipModel.stepState(this.state, p);
    this.history.push(ShipModel.sampleState(this.state, p));
  }
  loadFinished(history, p) {
    this.history = history;
    const h = history[history.length - 1];
    this.state = {t:h.t, psi:h.psi, r:h.r, delta:h.delta, integral:0, x:h.x, y:h.y, iae:h.iae};
    this.running = false;
    this.finished = true;
    this.lastParams = p;
  }
  get current() { return this.history[this.history.length - 1]; }
}

const sim = new Simulator();
let lastReal = performance.now();

function fmt(v, digits = 2, suffix = '') {
  return Number.isFinite(v) ? `${v.toFixed(digits)}${suffix}` : '—';
}

function updateStatus(text, running) {
  ui.runStatus.textContent = text;
  ui.runDot.classList.toggle('running', running);
}

function updateReadouts() {
  const p = params();
  const h = sim.current || ShipModel.sampleState(ShipModel.createState(p), p);
  ui.simClock.textContent = `t = ${h.t.toFixed(1)} с`;
  ui.mCourse.textContent = `${h.psi.toFixed(2)}°`;
  ui.mTarget.textContent = `${h.target.toFixed(2)}°`;
  ui.mError.textContent = `${h.error.toFixed(2)}°`;
  ui.mRudder.textContent = `${h.delta.toFixed(2)}°`;
  ui.mYaw.textContent = `${h.r.toFixed(4)}°/с`;
  ui.mDist.textContent = `${h.dist.toFixed(1)}°`;

  const q = ShipModel.calculateMetrics(sim.history, p);
  ui.qRise.textContent = fmt(q.riseTime, 1, ' с');
  ui.qOvershoot.textContent = fmt(q.overshoot, 2, '°');
  ui.qSettling.textContent = fmt(q.settlingTime, 1, ' с');
  ui.qMaxDev.textContent = fmt(q.maxDeviation, 2, '°');
  ui.qRecovery.textContent = fmt(q.recoveryTime, 1, ' с');
  ui.qSteady.textContent = fmt(q.steadyError, 2, '°');
  ui.qMaxRudder.textContent = fmt(q.maxRudder, 2, '°');
  ui.qIAE.textContent = fmt(q.iae, 1, ' °·с');
}

function fitCanvas(c) {
  const r = c.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  const w = Math.max(100, Math.round(r.width * dpr)), h = Math.max(80, Math.round(r.height * dpr));
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  return {ctx:c.getContext('2d'), w, h, dpr};
}
function theme(ctx) { ctx.font = `${12 * (window.devicePixelRatio || 1)}px system-ui`; ctx.lineJoin = 'round'; ctx.lineCap = 'round'; }
function drawGrid(ctx, w, h, pad) {
  ctx.strokeStyle = 'rgba(145,168,195,.12)'; ctx.lineWidth = 1;
  for (let i = 0; i <= 5; i++) { const y = pad + (h - 2 * pad) * i / 5; ctx.beginPath(); ctx.moveTo(pad,y); ctx.lineTo(w-pad,y); ctx.stroke(); }
  for (let i = 0; i <= 8; i++) { const x = pad + (w - 2 * pad) * i / 8; ctx.beginPath(); ctx.moveTo(x,pad); ctx.lineTo(x,h-pad); ctx.stroke(); }
}
function seriesChart(canvas, series, opts = {}) {
  const {ctx,w,h,dpr} = fitCanvas(canvas); theme(ctx); ctx.clearRect(0,0,w,h);
  const pad = 40 * dpr; drawGrid(ctx,w,h,pad);
  if (!series.length || !series.some(s => s.data.length)) return;
  const all = series.flatMap(s => s.data.map(d => d[1])).filter(Number.isFinite);
  const xs = series.flatMap(s => s.data.map(d => d[0])).filter(Number.isFinite);
  if (!all.length || !xs.length) return;
  let ymin = opts.ymin ?? Math.min(...all), ymax = opts.ymax ?? Math.max(...all);
  if (Math.abs(ymax - ymin) < 1e-6) { ymin -= 1; ymax += 1; }
  const margin = (ymax - ymin) * .12; ymin -= margin; ymax += margin;
  const xmin = 0, xmax = Math.max(1, opts.xmax ?? Math.max(...xs));
  const X = x => pad + (x - xmin) / (xmax - xmin) * (w - 2 * pad);
  const Y = y => h - pad - (y - ymin) / (ymax - ymin) * (h - 2 * pad);

  (opts.markers || []).forEach(m => {
    if (m.x < xmin || m.x > xmax) return;
    const x = X(m.x); ctx.strokeStyle = m.color || 'rgba(255,255,255,.25)'; ctx.lineWidth = dpr; ctx.setLineDash([4*dpr,4*dpr]);
    ctx.beginPath(); ctx.moveTo(x,pad); ctx.lineTo(x,h-pad); ctx.stroke(); ctx.setLineDash([]);
    if (m.label) { ctx.fillStyle = m.color || '#91a8c3'; ctx.font = `${9*dpr}px system-ui`; ctx.fillText(m.label, x + 4*dpr, pad + 12*dpr); }
  });

  ctx.fillStyle = '#91a8c3'; ctx.font = `${10*dpr}px system-ui`;
  ctx.fillText(`${ymax.toFixed(1)}`, 5*dpr, pad + 4*dpr);
  ctx.fillText(`${ymin.toFixed(1)}`, 5*dpr, h - pad);
  ctx.fillText(`${xmax.toFixed(0)} с`, w - 60*dpr, h - 8*dpr);

  series.forEach(s => {
    ctx.strokeStyle = s.color; ctx.lineWidth = (s.width || 2) * dpr; ctx.setLineDash((s.dash || []).map(v => v*dpr)); ctx.beginPath();
    const stride = Math.max(1, Math.floor(s.data.length / Math.max(1200, w / dpr * 2)));
    let first = true;
    for (let i = 0; i < s.data.length; i += stride) {
      const d = s.data[i], x = X(d[0]), y = Y(d[1]);
      if (first) { ctx.moveTo(x,y); first = false; } else ctx.lineTo(x,y);
    }
    const last = s.data[s.data.length - 1]; if (last) ctx.lineTo(X(last[0]), Y(last[1]));
    ctx.stroke(); ctx.setLineDash([]);
  });
}

function drawVessel() {
  const {ctx,w,h,dpr} = fitCanvas(canvases.vessel); theme(ctx); ctx.clearRect(0,0,w,h);
  const g = ctx.createLinearGradient(0,0,0,h); g.addColorStop(0,'#071a2d'); g.addColorStop(.55,'#092641'); g.addColorStop(1,'#061827'); ctx.fillStyle=g; ctx.fillRect(0,0,w,h);
  ctx.strokeStyle='rgba(76,171,220,.08)'; ctx.lineWidth=1;
  for (let y=0; y<h; y+=24*dpr) { ctx.beginPath(); for (let x=0; x<w; x+=12*dpr) { const yy=y+Math.sin(x/(55*dpr))*3*dpr; x===0?ctx.moveTo(x,yy):ctx.lineTo(x,yy); } ctx.stroke(); }
  const hist = sim.history; if (!hist.length) return; const cur = hist[hist.length-1];
  const range = 220, scale = Math.min(w,h)/(range*1.4), sx=x=>w/2+(x-cur.x)*scale, sy=y=>h/2-(y-cur.y)*scale;
  ctx.strokeStyle='rgba(57,169,255,.85)'; ctx.lineWidth=2*dpr; ctx.beginPath();
  const stride=Math.max(1,Math.floor(hist.length/2500)); let first=true;
  for(let i=0;i<hist.length;i+=stride){const q=hist[i],x=sx(q.x),y=sy(q.y); if(first){ctx.moveTo(x,y);first=false}else ctx.lineTo(x,y);} ctx.lineTo(sx(cur.x),sy(cur.y)); ctx.stroke();

  const p=params(), cx=w/2, cy=h/2, L=120*dpr, currentTarget=ShipModel.referenceAt(cur.t,p), ta=deg2rad(currentTarget);
  ctx.strokeStyle='#65e4c2'; ctx.lineWidth=2*dpr; ctx.setLineDash([8*dpr,7*dpr]); ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(cx+Math.sin(ta)*L,cy-Math.cos(ta)*L); ctx.stroke(); ctx.setLineDash([]);

  if (Math.abs(cur.dist)>.01) {
    ctx.strokeStyle='#ff6b7a'; ctx.lineWidth=3*dpr; ctx.beginPath(); ctx.moveTo(cx-100*dpr,cy+70*dpr); ctx.lineTo(cx-25*dpr,cy+70*dpr); ctx.stroke();
    ctx.fillStyle='#ff6b7a'; ctx.beginPath(); ctx.moveTo(cx-25*dpr,cy+70*dpr); ctx.lineTo(cx-38*dpr,cy+62*dpr); ctx.lineTo(cx-38*dpr,cy+78*dpr); ctx.closePath(); ctx.fill();
    ctx.font=`${11*dpr}px system-ui`; ctx.fillText(`δв = ${cur.dist.toFixed(1)}°`,cx-100*dpr,cy+92*dpr);
  }

  ctx.save(); ctx.translate(cx,cy); ctx.rotate(deg2rad(cur.psi)); ctx.shadowColor='rgba(57,169,255,.55)'; ctx.shadowBlur=18*dpr;
  ctx.fillStyle='#eaf3ff'; ctx.strokeStyle='#39a9ff'; ctx.lineWidth=2*dpr; ctx.beginPath(); ctx.moveTo(0,-30*dpr); ctx.lineTo(12*dpr,-12*dpr); ctx.lineTo(10*dpr,25*dpr); ctx.lineTo(0,31*dpr); ctx.lineTo(-10*dpr,25*dpr); ctx.lineTo(-12*dpr,-12*dpr); ctx.closePath(); ctx.fill(); ctx.stroke();
  ctx.shadowBlur=0; ctx.fillStyle='#0c192b'; ctx.fillRect(-5*dpr,-4*dpr,10*dpr,14*dpr);
  ctx.translate(0,28*dpr); ctx.rotate(deg2rad(cur.delta)); ctx.strokeStyle='#ffc857'; ctx.lineWidth=4*dpr; ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(0,15*dpr); ctx.stroke(); ctx.restore();

  const ccx=w-62*dpr, ccy=64*dpr, rr=35*dpr; ctx.strokeStyle='rgba(234,243,255,.45)'; ctx.lineWidth=1.5*dpr; ctx.beginPath(); ctx.arc(ccx,ccy,rr,0,Math.PI*2); ctx.stroke();
  ctx.fillStyle='#eaf3ff'; ctx.font=`${10*dpr}px system-ui`; ctx.fillText('N',ccx-4*dpr,ccy-rr-5*dpr); ctx.strokeStyle='#39a9ff'; ctx.lineWidth=3*dpr; ctx.beginPath(); ctx.moveTo(ccx,ccy); ctx.lineTo(ccx+Math.sin(deg2rad(cur.psi))*rr*.8,ccy-Math.cos(deg2rad(cur.psi))*rr*.8); ctx.stroke();
}

function drawCharts() {
  const h=sim.history, p=params();
  const markers=[{x:p.targetStepTime,label:'задание курса',color:'rgba(101,228,194,.55)'},{x:p.distStart,label:'возмущение',color:'rgba(255,107,122,.55)'}];
  seriesChart(canvases.course,[{data:h.map(q=>[q.t,q.psi]),color:'#39a9ff'},{data:h.map(q=>[q.t,q.target]),color:'#65e4c2',dash:[7,6]}],{xmax:p.duration,markers});
  seriesChart(canvases.error,[{data:h.map(q=>[q.t,q.error]),color:'#ff6b7a'}],{xmax:p.duration,markers});
  seriesChart(canvases.rudder,[{data:h.map(q=>[q.t,q.delta]),color:'#ffc857'}],{xmax:p.duration,ymin:-p.rudderLimit,ymax:p.rudderLimit,markers});
}
function drawAll(){drawVessel();drawCharts();}

function animate(now) {
  const elapsed=(now-lastReal)/1000; lastReal=now;
  if(sim.running){
    const speed=+ui.speedRange.value; let budget=Math.min(2.0,elapsed*speed);
    while(budget>=params().dt && sim.running){sim.step();budget-=params().dt;}
    updateReadouts(); drawAll();
  }
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);

function resetAll(){sim.reset();updateStatus('Готово',false);updateReadouts();drawAll();}
function instantRun(){
  const p=params(); updateStatus('Расчёт…',true);
  const history=ShipModel.simulate(p); sim.loadFinished(history,p);
  updateStatus('Рассчитано',false); updateReadouts(); drawAll();
}

ui.startBtn.onclick=()=>{if(sim.finished)sim.reset();sim.running=true;updateStatus('Моделирование',true);};
ui.pauseBtn.onclick=()=>{sim.running=false;updateStatus('Пауза',false);};
ui.resetBtn.onclick=resetAll;
ui.instantBtn.onclick=instantRun;
ui.speedRange.oninput=()=>ui.speedOut.textContent=`${ui.speedRange.value}×`;
ui.disturbanceType.onchange=()=>{
  document.querySelectorAll('.pulse-only').forEach(x=>x.style.display=ui.disturbanceType.value==='pulse'?'flex':'none');
  document.querySelectorAll('.sine-only').forEach(x=>x.style.display=ui.disturbanceType.value==='sine'?'flex':'none');
};
window.addEventListener('resize',drawAll);

const articleUI = (controller='PID', distAmp=10, gains=null) => ({
  initialCourse:0,targetCourse:5,targetStepTime:20,shipSpeed:8,duration:600,
  controllerType:controller,kp:gains?.kp ?? 9,ki:controller==='PD'?0:(gains?.ki ?? .08),kd:gains?.kd ?? 120,nomotoK:.185,nomotoT:107.3,
  disturbanceType:'step',disturbance:distAmp,disturbanceStart:220,disturbanceDuration:20,disturbancePeriod:50,rudderLimit:35,rudderRate:5
});
const optGains = ShipModel.OPTIMIZED_PID;

const presets={
  article_manual10:articleUI('PID',10), article_opt10:articleUI('PID',10,optGains), article_opt5:articleUI('PID',5,optGains), article_opt15:articleUI('PID',15,optGains),
  article_pid10:articleUI('PID',10), article_pd10:articleUI('PD',10), article_pid5:articleUI('PID',5), article_pid15:articleUI('PID',15),
  strong:{...articleUI('PID',25,optGains),targetCourse:10,disturbanceStart:180},
  slow:{...articleUI('PID',10,optGains),targetCourse:10,nomotoK:.14,nomotoT:180,rudderRate:4},
  aggressive:{...articleUI('PID',10,{kp:16,ki:.22,kd:230}),targetCourse:10},
  pulse:{...articleUI('PID',15,optGains),targetCourse:10,disturbanceType:'pulse',disturbanceStart:180,disturbanceDuration:35}
};

function applyPreset(q){
  Object.entries(q).forEach(([k,v])=>{if(ui[k])ui[k].value=v;});
  ui.disturbanceType.onchange(); resetAll();
}
document.querySelectorAll('[data-preset]').forEach(b=>b.onclick=()=>applyPreset(presets[b.dataset.preset]));
ui.articleBtn.onclick=()=>{applyPreset(presets.article_opt10);instantRun();};

function batch(controller, baseParams=params()){
  const p=ShipModel.normalizeParams({...baseParams,controller});
  if(controller==='PD') p.ki=0;
  const hist=ShipModel.simulate(p); return {p,hist,q:ShipModel.calculateMetrics(hist,p)};
}

function renderCompare(res,names){
  const colors={P:'#b48cff',PI:'#65e4c2',PD:'#ffc857',PID:'#39a9ff','Ручная':'#ffc857','Авто':'#65e4c2'};
  const p=res[0].p;
  seriesChart(canvases.compare,res.map((r,i)=>({data:r.hist.map(q=>[q.t,q.psi]),color:colors[names[i]],width:2})),{xmax:p.duration,markers:[{x:p.targetStepTime,label:'задание курса',color:'rgba(101,228,194,.55)'},{x:p.distStart,label:'возмущение',color:'rgba(255,107,122,.55)'}]});
  ui.compareBody.innerHTML=res.map((r,i)=>`<tr><td><b style="color:${colors[names[i]]}">${names[i]}</b></td><td>${fmt(r.q.riseTime,1)}</td><td>${fmt(r.q.overshoot,2)}</td><td>${fmt(r.q.maxDeviation,2)}</td><td>${fmt(r.q.recoveryTime,1)}</td><td>${fmt(r.q.steadyError,2)}</td></tr>`).join('');
}
ui.compareBtn.onclick=()=>{const names=['P','PI','PD','PID'];renderCompare(names.map(n=>batch(n)),names);};
ui.articleCompareBtn.onclick=()=>renderTuneComparison();

function batchParams(p){
  const pp=ShipModel.normalizeParams(p), hist=ShipModel.simulate(pp);
  return {p:pp,hist,q:ShipModel.calculateMetrics(hist,pp)};
}
function manualParams(distAmp=10){return ShipModel.normalizeParams({...ShipModel.ARTICLE_BASE,controller:'PID',kp:9,ki:.08,kd:120,distAmp});}
function optimizedParams(distAmp=10){return ShipModel.optimizedArticleParams(distAmp);}
function renderTuneComparison(){
  const res=[batchParams(manualParams(10)),batchParams(optimizedParams(10))];
  renderCompare(res,['Ручная','Авто']);
}
function renderTuneTable(){
  const rows=[];
  for(const d of [5,10,15]){
    for(const [label,p] of [['Ручная',manualParams(d)],['Авто',optimizedParams(d)]]){
      const h=ShipModel.simulate(p),q=ShipModel.calculateMetrics(h,p);
      rows.push(`<tr><td>${d}°</td><td>${label}</td><td>${q.iae.toFixed(1)}</td><td>${q.maxDeviation.toFixed(3)}</td><td>${Number.isFinite(q.recoveryTime)?Math.max(0,q.recoveryTime).toFixed(1):'—'}</td><td>${q.steadyError.toFixed(3)}</td></tr>`);
    }
  }
  ui.tuneBody.innerHTML=rows.join('');
  const jm=PIDOptimizer.objective([9,.08,120]), jo=PIDOptimizer.objective([14,.1775,206.25]);
  ui.jManual.textContent=jm.toFixed(4); ui.jOptimized.textContent=jo.toFixed(4); ui.jGain.textContent=`${((jm-jo)/jm*100).toFixed(1)}%`;
  ui.gainResult.textContent='Kp 14 · Ki 0.1775 · Kd 206.25';
}
function applyOptimized(){
  ui.controllerType.value='PID'; ui.kp.value=14; ui.ki.value=.1775; ui.kd.value=206.25;
  ui.autotuneBadge.textContent='параметры применены'; ui.autotuneBadge.classList.add('ok');
  renderTuneTable(); resetAll();
}
ui.applyOptimizedBtn.onclick=()=>{applyPreset(presets.article_opt10);renderTuneTable();ui.autotuneBadge.textContent='параметры статьи';ui.autotuneBadge.classList.add('ok');};
ui.compareTuneBtn.onclick=()=>{renderTuneComparison();renderTuneTable();};
ui.autotuneBtn.onclick=()=>{
  ui.autotuneBtn.disabled=true; ui.autotuneStatus.className='autotune-status running'; ui.autotuneStatus.textContent='Выполняется робастный поиск по 9 сценариям…';
  setTimeout(()=>{
    const t0=performance.now();
    const r=PIDOptimizer.tune();
    const ms=performance.now()-t0;
    ui.kp.value=r.kp.toFixed(4).replace(/0+$/,'').replace(/\.$/,'');
    ui.ki.value=r.ki.toFixed(4); ui.kd.value=r.kd.toFixed(2);
    ui.controllerType.value='PID';
    ui.autotuneStatus.className='autotune-status done';
    ui.autotuneStatus.textContent=`Готово: ${r.evaluations} оценок за ${ms.toFixed(0)} мс. J: ${r.baselineObjective.toFixed(4)} → ${r.objective.toFixed(4)}.`;
    ui.autotuneBadge.textContent='поиск выполнен'; ui.autotuneBadge.classList.add('ok');
    renderTuneTable(); resetAll(); ui.autotuneBtn.disabled=false;
  },30);
};

function closeEnough(actual, expected, tol){
  if(Number.isNaN(expected)) return Number.isNaN(actual);
  return Number.isFinite(actual) && Math.abs(actual-expected)<=tol;
}
function verifyArticle(){
  const cases=[
    ['Ручной ПИД, δв=10°','manual10',manualParams(10)],
    ['Авто-ПИД, δв=5°','opt5',optimizedParams(5)],
    ['Авто-ПИД, δв=10°','opt10',optimizedParams(10)],
    ['Авто-ПИД, δв=15°','opt15',optimizedParams(15)]
  ];
  let all=true;
  ui.verifyBody.innerHTML=cases.map(([label,key,p])=>{
    const hist=ShipModel.simulate(p), q=ShipModel.calculateMetrics(hist,p), e=ShipModel.AUTOTUNE_REFERENCE[key];
    const ok=closeEnough(q.maxDeviation,e.maxDeviation,.002) && closeEnough(Math.max(0,q.recoveryTime),e.recoveryTime,.06) && closeEnough(q.iae,e.iae,.15);
    all=all&&ok;
    return `<tr><td>${label}</td><td>${q.maxDeviation.toFixed(3)}</td><td>${Number.isFinite(q.recoveryTime)?Math.max(0,q.recoveryTime).toFixed(2):'—'}</td><td>${q.iae.toFixed(1)}</td><td><span class="badge ${ok?'ok':'bad'}">${ok?'совпадает':'расхождение'}</span></td></tr>`;
  }).join('');
  const tune=PIDOptimizer.tune();
  const gainsOK=Math.abs(tune.kp-14)<1e-9 && Math.abs(tune.ki-.1775)<1e-9 && Math.abs(tune.kd-206.25)<1e-9;
  all=all&&gainsOK;
  ui.verifySummary.className=`verification-summary ${all?'success':'failure'}`;
  ui.verifySummary.textContent=all?'✓ Численные опыты и сам алгоритм автонастройки воспроизведены: найдено Kp=14, Ki=0,1775, Kd=206,25.':'Есть расхождение с эталоном новой версии статьи.';
}
ui.verifyBtn.onclick=verifyArticle;

function download(name,text,type='text/plain'){
  const a=document.createElement('a'); a.href=URL.createObjectURL(new Blob([text],{type})); a.download=name; a.click(); setTimeout(()=>URL.revokeObjectURL(a.href),1000);
}
ui.csvBtn.onclick=()=>{
  if(sim.history.length<2)return alert('Сначала запустите или рассчитайте моделирование.');
  const rows=['t_s,course_deg,target_deg,error_deg,yaw_rate_deg_s,rudder_deg,disturbance_deg,x_m,y_m,IAE_deg_s',...sim.history.map(h=>[h.t,h.psi,h.target,h.error,h.r,h.delta,h.dist,h.x,h.y,h.iae].map(v=>Number(v).toFixed(6)).join(','))];
  download('ship_control_experiment.csv',rows.join('\n'),'text/csv');
};
ui.pngBtn.onclick=()=>{const a=document.createElement('a');a.href=canvases.course.toDataURL('image/png');a.download='course_chart.png';a.click();};

ui.disturbanceType.onchange();
ui.speedOut.textContent=`${ui.speedRange.value}×`;
renderTuneTable();
resetAll();
