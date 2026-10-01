"""Расчёт показателей качества переходного процесса."""
from __future__ import annotations

import math
from typing import Dict
import numpy as np

from .model import SimulationConfig


def calculate_metrics(
    result: Dict[str, np.ndarray],
    config: SimulationConfig | None = None,
    tolerance_deg: float = 0.5,
    hold_time_s: float = 30.0,
) -> Dict[str, float]:
    """Рассчитать показатели, используемые в статье.

    Возврат после возмущения считается состоявшимся, если ошибка остаётся
    в полосе ±tolerance_deg не менее hold_time_s подряд.
    """
    cfg = config or SimulationConfig()
    t = result["t"]
    y = result["psi_deg"]
    target = cfg.target_deg

    pre_mask = (t >= cfg.target_step_time) & (t < cfg.disturbance_time)
    tp = t[pre_mask]
    yp = y[pre_mask]

    overshoot = max(0.0, float(np.max(yp) - target))

    rise_time = math.nan
    idx10 = np.where(yp >= 0.1 * target)[0]
    idx90 = np.where(yp >= 0.9 * target)[0]
    if idx10.size and idx90.size:
        rise_time = float(tp[idx90[0]] - tp[idx10[0]])

    settling_time = math.nan
    for j in range(len(yp)):
        if np.all(np.abs(yp[j:] - target) <= tolerance_deg):
            settling_time = float(tp[j] - cfg.target_step_time)
            break

    post_mask = t >= cfg.disturbance_time
    tt = t[post_mask]
    yy = y[post_mask]
    max_deviation = float(np.max(np.abs(yy - target)))

    recovery_time = math.nan
    window = max(1, int(round(hold_time_s / cfg.dt)))
    for j in range(max(0, len(yy) - window)):
        if np.all(np.abs(yy[j : j + window] - target) <= tolerance_deg):
            recovery_time = float(tt[j] - cfg.disturbance_time)
            break

    steady_mask = t >= (cfg.t_end - 30.0)
    steady_state_error = float(np.mean(y[steady_mask]) - target)
    rudder_max = float(np.max(np.abs(result["rudder_deg"])))

    return {
        "rise_time_s": rise_time,
        "overshoot_deg": overshoot,
        "settling_time_s": settling_time,
        "max_deviation_after_disturbance_deg": max_deviation,
        "recovery_time_s": recovery_time,
        "steady_state_error_deg": steady_state_error,
        "max_abs_rudder_deg": rudder_max,
    }
