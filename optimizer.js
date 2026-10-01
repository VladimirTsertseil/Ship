'use strict';

(function(root,factory){
  const api=factory();
  if(typeof module==='object' && module.exports) module.exports=api;
  else root.PIDOptimizer=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
  const BOUNDS = Object.freeze({kp:[4,14], ki:[0.03,0.20], kd:[50,220]});
  const BASELINE = Object.freeze({kp:9, ki:0.08, kd:120});
  const ARTICLE_OPTIMUM = Object.freeze({kp:14, ki:0.1775, kd:206.25});
  const WEIGHTS = Object.freeze({iae:0.30,recovery:0.20,maxdev:0.15,rmsRudder:0.10,settling:0.15,overshoot:0.10,worstCase:0.20});

  const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
  function bounded(x){return [clamp(x[0],...BOUNDS.kp),clamp(x[1],...BOUNDS.ki),clamp(x[2],...BOUNDS.kd)];}

  // Компактный расчёт без хранения временного ряда. Дискретная схема совпадает с model.js.
  function evaluateScenario(gains, K, T, distAmp){
    const [kp,ki,kd]=gains;
    const dt=0.05, duration=600, n=Math.round(duration/dt), target=5, stepTime=20, distStart=220;
    const rudderLimit=35, rudderRate=5, tol=0.5, holdSteps=Math.round(30/dt);
    let psi=0,r=0,delta=0,integral=0,iae=0,rudderSq=0,maxRudder=0;
    let maxDev=0, overshoot=0, lastPreViolation=-1;
    let within=0, recovery=NaN;
    let tailSum=0, tailN=0;
    for(let i=0;i<=n;i++){
      const t=i*dt, ref=t>=stepTime?target:0, e=ref-psi, dist=t>=distStart?distAmp:0;
      if(i<n) iae += Math.abs(e)*dt;
      rudderSq += delta*delta; maxRudder=Math.max(maxRudder,Math.abs(delta));
      if(t>=stepTime && t<distStart){
        overshoot=Math.max(overshoot,psi-target);
        if(Math.abs(psi-target)>tol) lastPreViolation=t;
      }
      if(t>=distStart){
        const dev=Math.abs(psi-target); maxDev=Math.max(maxDev,dev);
        if(dev<=tol){
          within++;
          if(!Number.isFinite(recovery) && within>=holdSteps){
            recovery=Math.max(0,t-distStart-30+dt);
          }
        } else within=0;
      }
      if(t>=duration-30){tailSum+=psi;tailN++;}
      if(i===n) break;

      let raw=kp*e+ki*integral-kd*r;
      let command=clamp(raw,-rudderLimit,rudderLimit);
      if(Math.abs(raw)<rudderLimit || Math.sign(e)!==Math.sign(raw)){
        integral += e*dt;
        raw=kp*e+ki*integral-kd*r;
        command=clamp(raw,-rudderLimit,rudderLimit);
      }
      const maxChange=rudderRate*dt;
      const deltaNext=delta+clamp(command-delta,-maxChange,maxChange);
      const yawAccel=(K*(delta+dist)-r)/T;
      const rNext=r+yawAccel*dt;
      const psiNext=psi+r*dt;
      delta=deltaNext;r=rNext;psi=psiNext;
    }
    const settling=lastPreViolation>=stepTime ? lastPreViolation+dt-stepTime : 0;
    const rmsRudder=Math.sqrt(rudderSq/(n+1));
    const steadyError=tailN?tailSum/tailN-target:NaN;
    return {iae,recovery,maxDev,steadyError,rmsRudder,maxRudder,settling,overshoot:Math.max(0,overshoot)};
  }

  function robustScenarios(){
    const K=.185,T=107.3, arr=[];
    for(const d of [5,10,15]){
      arr.push({K,T,d,label:`${d}° / номинал`});
      arr.push({K:K*1.10,T:T*.90,d,label:`${d}° / более маневренная модель`});
      arr.push({K:K*.90,T:T*1.10,d,label:`${d}° / более инерционная модель`});
    }
    return arr;
  }

  function scenarioCost(m){
    const rec=Number.isFinite(m.recovery)?m.recovery:380;
    let j=WEIGHTS.iae*(m.iae/500)+WEIGHTS.recovery*(rec/200)+WEIGHTS.maxdev*(m.maxDev/2)+WEIGHTS.rmsRudder*(m.rmsRudder/10)+WEIGHTS.settling*(m.settling/60)+WEIGHTS.overshoot*(m.overshoot/1);
    if(m.maxRudder>33) j += .2*Math.pow((m.maxRudder-33)/2,2);
    return j;
  }

  function objective(gains, withDetails=false){
    const details=robustScenarios().map(s=>{
      const m=evaluateScenario(gains,s.K,s.T,s.d); return {...s,...m,cost:scenarioCost(m)};
    });
    const costs=details.map(x=>x.cost), mean=costs.reduce((a,b)=>a+b,0)/costs.length, worst=Math.max(...costs);
    const value=mean+WEIGHTS.worstCase*worst;
    return withDetails?{value,mean,worst,details}:value;
  }

  // Детерминированный многоуровневый pattern search. 304 оценки для стандартного запуска.
  function tune(onProgress=null){
    let x=[BASELINE.kp,BASELINE.ki,BASELINE.kd], best=objective(x), evals=1;
    let steps=[2,.04,30];
    const levels=5;
    for(let level=0;level<levels;level++){
      let improved=true;
      while(improved){
        improved=false; let bestX=x.slice();
        for(let a=-1;a<=1;a++) for(let b=-1;b<=1;b++) for(let c=-1;c<=1;c++){
          if(a===0&&b===0&&c===0) continue;
          const y=bounded([x[0]+a*steps[0],x[1]+b*steps[1],x[2]+c*steps[2]]);
          if(y.every((v,i)=>Math.abs(v-x[i])<1e-12)) continue;
          const v=objective(y); evals++;
          if(v<best-1e-10){best=v;bestX=y;improved=true;}
        }
        x=bestX;
        if(onProgress) onProgress({level:level+1,levels,evals,gains:x.slice(),objective:best});
      }
      steps=steps.map(v=>v/2);
    }
    return {kp:x[0],ki:x[1],kd:x[2],objective:best,evaluations:evals,baselineObjective:objective([BASELINE.kp,BASELINE.ki,BASELINE.kd]),details:objective(x,true)};
  }

  return {BOUNDS,BASELINE,ARTICLE_OPTIMUM,WEIGHTS,evaluateScenario,robustScenarios,scenarioCost,objective,tune};
});
