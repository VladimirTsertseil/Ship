"""Наглядная анимация работы системы автоматического управления курсом.

По умолчанию открывает интерактивное окно:
    python visualize.py

Сохранить GIF:
    python visualize.py --save

Выбрать регулятор:
    python visualize.py --controller pd
    python visualize.py --controller pid --save

Важно: положение судна на плоскости — условная кинематическая визуализация.
На научные результаты статьи влияют только курс, угловая скорость и угол руля,
рассчитанные моделью Номото.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Polygon

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ship_course_control.model import PD, PID, SimulationConfig, simulate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Анимация автоматического удержания курса судна")
    parser.add_argument("--controller", choices=("pd", "pid"), default="pid")
    parser.add_argument("--disturbance", type=float, default=10.0, help="Эквивалентное возмущение, град")
    parser.add_argument("--save", action="store_true", help="Сохранить GIF в visualization/")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--duration", type=float, default=20.0, help="Длительность GIF, с")
    return parser.parse_args()


def integrate_visual_track(heading_deg: np.ndarray, dt: float, speed_scale: float = 0.12):
    """Условная плоская траектория только для визуализации."""
    psi = np.deg2rad(heading_deg)
    x = np.zeros_like(psi)
    y = np.zeros_like(psi)
    for i in range(len(psi) - 1):
        x[i + 1] = x[i] + speed_scale * math.sin(psi[i]) * dt
        y[i + 1] = y[i] + speed_scale * math.cos(psi[i]) * dt
    return x, y


def ship_vertices(x: float, y: float, heading_deg: float, length: float = 2.7, width: float = 1.0):
    """Простой силуэт судна; нос направлен по heading_deg."""
    local = np.array([
        [length * 0.60, 0.0],
        [length * 0.12, width * 0.50],
        [-length * 0.45, width * 0.42],
        [-length * 0.55, 0.0],
        [-length * 0.45, -width * 0.42],
        [length * 0.12, -width * 0.50],
    ])
    # В морской конвенции 0° = север, 90° = восток.
    a = math.radians(90.0 - heading_deg)
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return local @ rot.T + np.array([x, y])


def make_animation(controller_key: str, disturbance_deg: float, fps: int, duration: float):
    cfg = SimulationConfig()
    controller = PID if controller_key == "pid" else PD
    result = simulate(controller, disturbance_deg=disturbance_deg, config=cfg)

    t = result["t"]
    heading = result["psi_deg"]
    target = result["reference_deg"]
    rudder = result["rudder_deg"]
    error = result["error_deg"]
    disturbance = result["disturbance_equiv_deg"]
    x, y = integrate_visual_track(heading, cfg.dt)

    n_frames = max(2, int(round(duration * fps)))
    frame_idx = np.linspace(0, len(t) - 1, n_frames).astype(int)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Условная координата X")
    ax.set_ylabel("Условная координата Y")
    ax.set_title(f"Автоматическое удержание курса — {controller.name}-регулятор")
    ax.grid(True, alpha=0.20)

    trail, = ax.plot([], [], linewidth=1.6, label="Траектория")
    target_line, = ax.plot([], [], linestyle="--", linewidth=1.2, label="Заданный курс")
    heading_line, = ax.plot([], [], linewidth=1.4, label="Фактический курс")
    rudder_line, = ax.plot([], [], linewidth=2.0, label="Руль")
    ship = Polygon(ship_vertices(0, 0, 0), closed=True, alpha=0.85)
    ax.add_patch(ship)

    # Компас и информационная панель рисуются в координатах окна, а не мира.
    compass = Circle((0.88, 0.84), 0.085, transform=ax.transAxes, fill=False, linewidth=1.2)
    ax.add_patch(compass)
    ax.text(0.88, 0.94, "N", transform=ax.transAxes, ha="center", va="center", fontsize=9)
    compass_arrow, = ax.plot([], [], transform=ax.transAxes, linewidth=1.8)

    info = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        family="monospace",
        bbox={"boxstyle": "round,pad=0.5", "alpha": 0.80},
    )
    event_text = ax.text(
        0.50,
        0.93,
        "",
        transform=ax.transAxes,
        ha="center",
        va="top",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.4", "alpha": 0.75},
    )

    # Стрелка возмущения (фиксированная в углу кадра).
    disturbance_arrow = ax.annotate(
        "",
        xy=(0.17, 0.17),
        xytext=(0.05, 0.17),
        xycoords=ax.transAxes,
        textcoords=ax.transAxes,
        arrowprops={"arrowstyle": "-|>", "lw": 2.2},
    )
    disturbance_label = ax.text(0.05, 0.20, "", transform=ax.transAxes, fontsize=9)

    ax.legend(loc="lower right")

    window = 15.0

    def update(frame_number: int):
        i = frame_idx[frame_number]
        xi, yi = float(x[i]), float(y[i])
        psi = float(heading[i])
        psi_target = float(target[i])
        delta = float(rudder[i])

        # Камера следует за судном.
        ax.set_xlim(xi - window, xi + window)
        ax.set_ylim(yi - window, yi + window)

        # Хвост траектории: примерно последние 150 секунд модельного времени.
        history_n = int(round(150.0 / cfg.dt))
        j0 = max(0, i - history_n)
        trail.set_data(x[j0:i + 1], y[j0:i + 1])

        ship.set_xy(ship_vertices(xi, yi, psi))

        # Направления заданного и фактического курса.
        arrow_len = 9.0
        a_t = math.radians(90.0 - psi_target)
        a_h = math.radians(90.0 - psi)
        target_line.set_data([xi, xi + arrow_len * math.cos(a_t)], [yi, yi + arrow_len * math.sin(a_t)])
        heading_line.set_data([xi, xi + arrow_len * math.cos(a_h)], [yi, yi + arrow_len * math.sin(a_h)])

        # Руль показан у кормы относительно продольной оси судна.
        stern_x = xi - 1.25 * math.cos(a_h)
        stern_y = yi - 1.25 * math.sin(a_h)
        rudder_angle = a_h + math.radians(delta)
        rudder_len = 1.5
        rudder_line.set_data(
            [stern_x, stern_x - rudder_len * math.cos(rudder_angle)],
            [stern_y, stern_y - rudder_len * math.sin(rudder_angle)],
        )

        # Компас: морская конвенция 0° = север, 90° = восток.
        ca = math.radians(psi)
        cx, cy, cr = 0.88, 0.84, 0.065
        compass_arrow.set_data([cx, cx + cr * math.sin(ca)], [cy, cy + cr * math.cos(ca)])

        info.set_text(
            f"t = {t[i]:6.1f} с\n"
            f"курс      = {psi:6.2f}°\n"
            f"задание   = {psi_target:6.2f}°\n"
            f"ошибка    = {error[i]:6.2f}°\n"
            f"руль      = {delta:6.2f}°\n"
            f"возмущение= {disturbance[i]:6.2f}°"
        )

        if t[i] < cfg.target_step_time:
            event_text.set_text("Исходный курс: 0°")
        elif t[i] < cfg.disturbance_time:
            event_text.set_text(f"Команда: перейти на курс {cfg.target_deg:.0f}°")
        else:
            event_text.set_text("Возмущение включено: регулятор компенсирует увод")

        if t[i] >= cfg.disturbance_time:
            disturbance_arrow.set_visible(True)
            disturbance_label.set_text(f"возмущение {disturbance_deg:.0f}°")
        else:
            disturbance_arrow.set_visible(False)
            disturbance_label.set_text("")

        return (
            trail,
            target_line,
            heading_line,
            rudder_line,
            ship,
            compass_arrow,
            info,
            event_text,
            disturbance_arrow,
            disturbance_label,
        )

    ani = FuncAnimation(fig, update, frames=n_frames, interval=1000 / fps, blit=False, repeat=True)
    plt.tight_layout()
    return fig, ani, controller.name


def main() -> None:
    args = parse_args()
    fig, ani, controller_name = make_animation(args.controller, args.disturbance, args.fps, args.duration)

    if args.save:
        out_dir = ROOT / "visualization"
        out_dir.mkdir(exist_ok=True)
        out_path = out_dir / f"ship_course_{args.controller}.gif"
        print(f"Сохраняю GIF: {out_path}")
        ani.save(out_path, writer=PillowWriter(fps=args.fps), dpi=90)
        print("Готово.")
        plt.close(fig)
    else:
        print(f"Запущена визуализация: {controller_name}-регулятор. Закройте окно для выхода.")
        plt.show()


if __name__ == "__main__":
    main()
