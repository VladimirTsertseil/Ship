"""Модель Номото первого порядка и ПД/ПИД-регулятор.

Модель предназначена для воспроизводимого вычислительного эксперимента,
описанного в статье для «Недели науки СПбГМТУ – 2026».

Основное уравнение:
    T * dr/dt + r = K * (delta + disturbance)
    dpsi/dt = r

где r — угловая скорость рыскания, psi — курс, delta — угол руля.
Все внутренние угловые величины хранятся в радианах.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict
import numpy as np


@dataclass(frozen=True)
class SimulationConfig:
    """Параметры объекта и вычислительного эксперимента."""

    nomoto_k: float = 0.185          # 1/с
    nomoto_t: float = 107.3          # с
    dt: float = 0.05                 # с
    t_end: float = 600.0             # с
    target_deg: float = 5.0          # град
    target_step_time: float = 20.0   # с
    disturbance_time: float = 220.0  # с
    rudder_max_deg: float = 35.0     # град
    rudder_rate_deg_s: float = 5.0   # град/с


@dataclass(frozen=True)
class Controller:
    """Коэффициенты регулятора."""

    kp: float
    ki: float
    kd: float
    name: str = "controller"


PD = Controller(kp=9.0, ki=0.0, kd=120.0, name="ПД")
PID = Controller(kp=9.0, ki=0.08, kd=120.0, name="ПИД")


def simulate(
    controller: Controller,
    disturbance_deg: float = 10.0,
    config: SimulationConfig | None = None,
) -> Dict[str, np.ndarray]:
    """Выполнить численное моделирование.

    Используется явный метод Эйлера. Для ПИД-регулятора реализовано
    условное интегрирование (anti-windup): интегратор не накапливает ошибку,
    когда исполнительный механизм насыщен и интегрирование усилило бы насыщение.

    Возмущение задаётся как эквивалентный угол руля. Это не угол ветра/течения,
    а компактный способ представить постоянный внешний разворачивающий момент.
    """
    cfg = config or SimulationConfig()

    n = int(round(cfg.t_end / cfg.dt)) + 1
    t = np.linspace(0.0, cfg.t_end, n)
    psi = np.zeros(n)       # курс, рад
    yaw_rate = np.zeros(n)  # угловая скорость, рад/с
    rudder = np.zeros(n)    # фактический угол руля, рад
    error = np.zeros(n)
    reference = np.zeros(n)
    disturbance = np.zeros(n)

    integral = 0.0
    rudder_max = np.deg2rad(cfg.rudder_max_deg)
    rudder_rate = np.deg2rad(cfg.rudder_rate_deg_s)

    for i in range(n - 1):
        reference[i] = np.deg2rad(cfg.target_deg if t[i] >= cfg.target_step_time else 0.0)
        disturbance[i] = np.deg2rad(disturbance_deg if t[i] >= cfg.disturbance_time else 0.0)
        e = reference[i] - psi[i]
        error[i] = e

        raw = controller.kp * e + controller.ki * integral - controller.kd * yaw_rate[i]
        command = np.clip(raw, -rudder_max, rudder_max)

        # Условное интегрирование для предотвращения интегрального насыщения.
        if abs(raw) < rudder_max or np.sign(e) != np.sign(raw):
            integral += e * cfg.dt
            raw = controller.kp * e + controller.ki * integral - controller.kd * yaw_rate[i]
            command = np.clip(raw, -rudder_max, rudder_max)

        # Ограничение скорости перекладки руля.
        max_change = rudder_rate * cfg.dt
        rudder[i + 1] = rudder[i] + np.clip(command - rudder[i], -max_change, max_change)

        # Модель Номото первого порядка.
        yaw_accel = (
            cfg.nomoto_k * (rudder[i] + disturbance[i]) - yaw_rate[i]
        ) / cfg.nomoto_t
        yaw_rate[i + 1] = yaw_rate[i] + yaw_accel * cfg.dt
        psi[i + 1] = psi[i] + yaw_rate[i] * cfg.dt

    reference[-1] = np.deg2rad(cfg.target_deg)
    disturbance[-1] = np.deg2rad(disturbance_deg)
    error[-1] = reference[-1] - psi[-1]

    return {
        "t": t,
        "psi_deg": np.rad2deg(psi),
        "yaw_rate_deg_s": np.rad2deg(yaw_rate),
        "rudder_deg": np.rad2deg(rudder),
        "error_deg": np.rad2deg(error),
        "reference_deg": np.rad2deg(reference),
        "disturbance_equiv_deg": np.rad2deg(disturbance),
    }
