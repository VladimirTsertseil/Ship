'use strict';
const M=require('./model.js');
const O=require('./optimizer.js');

function near(a,b,tol){return Number.isFinite(a)&&Math.abs(a-b)<=tol;}
let pass=true;
const r=O.tune();
const gainsOK=near(r.kp,14,1e-12)&&near(r.ki,.1775,1e-12)&&near(r.kd,206.25,1e-12);
console.log(`${gainsOK?'PASS':'FAIL'} tune: Kp=${r.kp}, Ki=${r.ki}, Kd=${r.kd}, J=${r.objective.toFixed(9)}, evals=${r.evaluations}`);
pass&&=gainsOK;

const cases=[
  ['manual10', M.normalizeParams({...M.ARTICLE_BASE,controller:'PID',kp:9,ki:.08,kd:120,distAmp:10})],
  ['opt5',M.optimizedArticleParams(5)],['opt10',M.optimizedArticleParams(10)],['opt15',M.optimizedArticleParams(15)]
];
for(const [key,p] of cases){
  const q=M.calculateMetrics(M.simulate(p),p), e=M.AUTOTUNE_REFERENCE[key];
  const ok=near(q.maxDeviation,e.maxDeviation,.002)&&near(Math.max(0,q.recoveryTime),e.recoveryTime,.06)&&near(q.iae,e.iae,.15);
  console.log(`${ok?'PASS':'FAIL'} ${key}: maxDev=${q.maxDeviation.toFixed(6)}, recovery=${Math.max(0,q.recoveryTime).toFixed(2)}, IAE=${q.iae.toFixed(3)}`);
  pass&&=ok;
}
if(!pass) process.exit(1);
console.log('Autotuning and article experiments reproduced successfully.');
