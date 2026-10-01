"""Минимальная проверка того, что расчёт воспроизводит числа из статьи.

Запуск из корня репозитория:
    python tests/test_reproducibility.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ship_course_control.model import PD, PID, SimulationConfig, simulate
from ship_course_control.metrics import calculate_metrics

cfg = SimulationConfig()

EXPECTED = {
    "PD": {
        "rise_time_s": 21.9,
        "overshoot_deg": 0.0203731,
        "settling_time_s": 29.1,
        "max_deviation_after_disturbance_deg": 1.1156336,
        "steady_state_error_deg": 1.1111111,
        "max_abs_rudder_deg": 27.6853231,
    },
    "PID": {
        "rise_time_s": 18.85,
        "overshoot_deg": 0.5129410,
        "settling_time_s": 57.5,
        "max_deviation_after_disturbance_deg": 1.0812910,
        "recovery_time_s": 114.6,
        "steady_state_error_deg": 0.0381352,
        "max_abs_rudder_deg": 27.8729261,
    },
}


def assert_close(actual: float, expected: float, tol: float = 1e-5) -> None:
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol):
        raise AssertionError(f"{actual} != {expected} (tol={tol})")


for key, controller in (("PD", PD), ("PID", PID)):
    result = simulate(controller, disturbance_deg=10.0, config=cfg)
    metrics = calculate_metrics(result, config=cfg)
    for metric, expected in EXPECTED[key].items():
        assert_close(metrics[metric], expected)

pd_metrics = calculate_metrics(simulate(PD, 10.0, cfg), config=cfg)
if not math.isnan(pd_metrics["recovery_time_s"]):
    raise AssertionError("ПД-регулятор не должен возвращаться в полосу ±0,5° при постоянном возмущении 10°")

print("OK: вычислительный эксперимент воспроизводит показатели статьи.")
