"""Модель автоматического управления курсом судна для студенческого исследования."""

from .model import SimulationConfig, Controller, simulate
from .metrics import calculate_metrics

__all__ = ["SimulationConfig", "Controller", "simulate", "calculate_metrics"]
