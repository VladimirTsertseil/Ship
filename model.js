'use strict';

(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.ShipModel = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const deg2rad = d => d * Math.PI / 180;
  const wrapDeg = a => {
    let x = a;
    while (x > 180) x -= 360;
    while (x <= -180) x += 360;
    return x;
  };

  const DEFAULTS = Object.freeze({
    initialCourse: 0,
    targetCourse: 5,
    targetStepTime: 20,
    shipSpeed: 8,
    duration: 600,
    dt: 0.05,
    controller: 'PID',
    kp: 9,
    ki: 0.08,
    kd: 120,
    K: 0.185,
    T: 107.3,
    distType: 'step',
    distAmp: 10,
    distStart: 220,
    distDuration: 20,
    distPeriod: 50,
    rudderLimit: 35,
    rudderRate: 5
  });

  function normalizeParams(input = {}) {
    const p = { ...DEFAULTS, ...input };
    p.duration = Math.max(0.1, Number(p.duration));
    p.dt = Math.max(0.001, Number(p.dt));
    p.T = Math.max(0.001, Number(p.T));
    p.rudderLimit = Math.max(0.1, Math.abs(Number(p.rudderLimit)));
    p.rudderRate = Math.max(0.001, Math.abs(Number(p.rudderRate)));
    return p;
  }

  function usesI(controller) { return controller === 'PI' || controller === 'PID'; }
  function usesD(controller) { return controller === 'PD' || controller === 'PID'; }

  function referenceAt(t, p) {
    return t >= p.targetStepTime ? p.targetCourse : p.initialCourse;
  }

  function disturbanceAt(t, p) {
    if (t < p.distStart) return 0;
    if (p.distType === 'step') return p.distAmp;
    if (p.distType === 'pulse') return t <= p.distStart + p.distDuration ? p.distAmp : 0;
    if (p.distType === 'sine') {
      return p.distAmp * Math.sin(2 * Math.PI * (t - p.distStart) / Math.max(1e-6, p.distPeriod));
    }
    return 0;
  }

  function createState(input = {}) {
    const p = normalizeParams(input);
    return {
      t: 0,
      psi: p.initialCourse,
      r: 0,
      delta: 0,
      integral: 0,
      x: 0,
      y: 0,
      iae: 0
    };
  }

  function sampleState(state, input = {}) {
    const p = normalizeParams(input);
    const target = referenceAt(state.t, p);
    const error = wrapDeg(target - state.psi);
    const dist = disturbanceAt(state.t, p);
    return {
      t: state.t,
      psi: state.psi,
      target,
      error,
      r: state.r,
      delta: state.delta,
      dist,
      x: state.x,
      y: state.y,
      iae: state.iae
    };
  }

  function stepState(state, input = {}) {
    const p = normalizeParams(input);
    const dt = p.dt;
    const target = referenceAt(state.t, p);
    const e = wrapDeg(target - state.psi);

    // Та же форма регулятора, что использована в статье:
    // u = Kp*e + Ki*∫e dt - Kd*r.
    // Производная берётся по измеренной угловой скорости, без derivative kick.
    let raw = p.kp * e;
    if (usesI(p.controller)) raw += p.ki * state.integral;
    if (usesD(p.controller)) raw -= p.kd * state.r;
    let command = clamp(raw, -p.rudderLimit, p.rudderLimit);

    // Условное интегрирование (anti-windup), идентично эталонному Python-расчёту.
    let integralNext = state.integral;
    if (usesI(p.controller)) {
      if (Math.abs(raw) < p.rudderLimit || Math.sign(e) !== Math.sign(raw)) {
        integralNext += e * dt;
        raw = p.kp * e + p.ki * integralNext;
        if (usesD(p.controller)) raw -= p.kd * state.r;
        command = clamp(raw, -p.rudderLimit, p.rudderLimit);
      }
    }

    const maxRudderChange = p.rudderRate * dt;
    const deltaNext = state.delta + clamp(command - state.delta, -maxRudderChange, maxRudderChange);

    const disturbance = disturbanceAt(state.t, p);

    // Явный метод Эйлера: в правой части используются значения на текущем шаге.
    const yawAccel = (p.K * (state.delta + disturbance) - state.r) / p.T;
    const rNext = state.r + yawAccel * dt;
    const psiNext = state.psi + state.r * dt;

    // Плоская траектория нужна только для визуализации.
    const v = p.shipSpeed * 0.514444;
    const th = deg2rad(psiNext);
    const xNext = state.x + v * Math.sin(th) * dt;
    const yNext = state.y + v * Math.cos(th) * dt;

    return {
      t: state.t + dt,
      psi: psiNext,
      r: rNext,
      delta: deltaNext,
      integral: integralNext,
      x: xNext,
      y: yNext,
      iae: state.iae + Math.abs(e) * dt
    };
  }

  function simulate(input = {}) {
    const p = normalizeParams(input);
    const steps = Math.round(p.duration / p.dt);
    let state = createState(p);
    const history = [];

    for (let i = 0; i <= steps; i++) {
      history.push(sampleState(state, p));
      if (i < steps) state = stepState(state, p);
    }
    return history;
  }

  function calculateMetrics(history, input = {}, toleranceDeg = 0.5, holdTimeS = 30) {
    const p = normalizeParams(input);
    if (!history || !history.length) {
      return {
        riseTime: NaN, overshoot: NaN, settlingTime: NaN,
        maxDeviation: NaN, recoveryTime: NaN, steadyError: NaN,
        maxRudder: NaN, iae: NaN
      };
    }

    const pre = history.filter(h => h.t >= p.targetStepTime && h.t < p.distStart);
    let overshoot = NaN, riseTime = NaN, settlingTime = NaN;
    if (pre.length) {
      overshoot = Math.max(0, Math.max(...pre.map(h => h.psi)) - p.targetCourse);
      const i10 = pre.findIndex(h => h.psi >= p.initialCourse + 0.1 * (p.targetCourse - p.initialCourse));
      const i90 = pre.findIndex(h => h.psi >= p.initialCourse + 0.9 * (p.targetCourse - p.initialCourse));
      if (i10 >= 0 && i90 >= 0) riseTime = pre[i90].t - pre[i10].t;

      for (let j = 0; j < pre.length; j++) {
        let ok = true;
        for (let k = j; k < pre.length; k++) {
          if (Math.abs(pre[k].psi - p.targetCourse) > toleranceDeg) { ok = false; break; }
        }
        if (ok) { settlingTime = pre[j].t - p.targetStepTime; break; }
      }
    }

    const post = history.filter(h => h.t >= p.distStart);
    let maxDeviation = NaN, recoveryTime = NaN;
    if (post.length) {
      maxDeviation = Math.max(...post.map(h => Math.abs(h.psi - p.targetCourse)));
      const dt = p.dt;
      const window = Math.max(1, Math.round(holdTimeS / dt));
      for (let j = 0; j < Math.max(0, post.length - window); j++) {
        let ok = true;
        for (let k = j; k < j + window; k++) {
          if (Math.abs(post[k].psi - p.targetCourse) > toleranceDeg) { ok = false; break; }
        }
        if (ok) { recoveryTime = post[j].t - p.distStart; break; }
      }
    }

    const tailStart = Math.max(0, history[history.length - 1].t - 30);
    const tail = history.filter(h => h.t >= tailStart);
    const steadyError = tail.length
      ? tail.reduce((s, h) => s + h.psi, 0) / tail.length - p.targetCourse
      : NaN;
    const maxRudder = Math.max(...history.map(h => Math.abs(h.delta)));
    const iae = history[history.length - 1].iae;

    return { riseTime, overshoot, settlingTime, maxDeviation, recoveryTime, steadyError, maxRudder, iae };
  }

  const ARTICLE_BASE = Object.freeze({
    initialCourse: 0,
    targetCourse: 5,
    targetStepTime: 20,
    shipSpeed: 8,
    duration: 600,
    dt: 0.05,
    controller: 'PID',
    kp: 9,
    ki: 0.08,
    kd: 120,
    K: 0.185,
    T: 107.3,
    distType: 'step',
    distAmp: 10,
    distStart: 220,
    distDuration: 20,
    distPeriod: 50,
    rudderLimit: 35,
    rudderRate: 5
  });

  const ARTICLE_REFERENCE = Object.freeze({
    PD10: {
      controller: 'PD', distAmp: 10,
      riseTime: 21.9, overshoot: 0.02037310190570274, settlingTime: 29.1,
      maxDeviation: 1.1156336333798391, recoveryTime: NaN,
      steadyError: 1.1111111111111347, maxRudder: 27.685323144747557
    },
    PID5: {
      controller: 'PID', distAmp: 5,
      maxDeviation: 0.5812392927476466, recoveryTime: 54.05,
      steadyError: 0.02039394383831805
    },
    PID10: {
      controller: 'PID', distAmp: 10,
      riseTime: 18.85, overshoot: 0.5129409711474686, settlingTime: 57.5,
      maxDeviation: 1.0812910253091896, recoveryTime: 114.6,
      steadyError: 0.03813522068566311, maxRudder: 27.872926071493122
    },
    PID15: {
      controller: 'PID', distAmp: 15,
      maxDeviation: 1.58152662131329, recoveryTime: 151.7,
      steadyError: 0.0558764975330206
    }
  });

  function articleParams(controller = 'PID', distAmp = 10) {
    return normalizeParams({ ...ARTICLE_BASE, controller, distAmp, ki: controller === 'PD' ? 0 : ARTICLE_BASE.ki });
  }

  const OPTIMIZED_PID = Object.freeze({kp:14, ki:0.1775, kd:206.25});
  const AUTOTUNE_REFERENCE = Object.freeze({
    objectiveManual: 0.7660057671427452,
    objectiveOptimized: 0.5071339508909866,
    manual10: {maxDeviation:1.0812910253091834,recoveryTime:114.6000000000418,steadyError:0.03813522068567465,iae:260.13831171709705},
    opt5: {maxDeviation:0.3279532379760015,recoveryTime:0,steadyError:0.001677011678935969,iae:148.28772670204205},
    opt10: {maxDeviation:0.6189804792547333,recoveryTime:57.4000000000288,steadyError:0.0032128671758844263,iae:176.38581884730078},
    opt15: {maxDeviation:0.9101953792046347,recoveryTime:84.15000000003488,steadyError:0.004748582734347195,iae:204.48391745456655}
  });

  function optimizedArticleParams(distAmp = 10) {
    return normalizeParams({ ...ARTICLE_BASE, controller:'PID', ...OPTIMIZED_PID, distAmp });
  }

  return {
    DEFAULTS, ARTICLE_BASE, ARTICLE_REFERENCE, OPTIMIZED_PID, AUTOTUNE_REFERENCE,
    clamp, wrapDeg, normalizeParams, referenceAt, disturbanceAt,
    createState, sampleState, stepState, simulate, calculateMetrics, articleParams, optimizedArticleParams
  };
});
