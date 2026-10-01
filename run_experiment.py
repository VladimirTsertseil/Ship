"""Воспроизведение вычислительного эксперимента из статьи.

Запуск:
    python run_experiment.py

Автоматически создаются графики, итоговые таблицы и сырые временные ряды.
"""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ship_course_control.model import PD, PID, SimulationConfig, simulate
from ship_course_control.metrics import calculate_metrics


def fmt(value: float, digits: int = 2) -> str:
    return "—" if math.isnan(value) else f"{value:.{digits}f}"


def save_main_metrics(metrics_by_controller: dict, path: Path) -> None:
    fields = [
        "controller",
        "rise_time_s",
        "overshoot_deg",
        "settling_time_s",
        "max_deviation_after_disturbance_deg",
        "recovery_time_s",
        "steady_state_error_deg",
        "max_abs_rudder_deg",
    ]
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for name, metrics in metrics_by_controller.items():
            writer.writerow({"controller": name, **metrics})


def save_disturbance_series(rows: list[dict], path: Path) -> None:
    fields = [
        "disturbance_equivalent_deg",
        "max_deviation_deg",
        "recovery_time_s",
        "steady_state_error_deg",
    ]
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def save_timeseries(main_results: dict, path: Path) -> None:
    pd = main_results["ПД"]
    pid = main_results["ПИД"]
    fields = [
        "time_s",
        "reference_deg",
        "disturbance_equivalent_deg",
        "pd_heading_deg",
        "pd_error_deg",
        "pd_rudder_deg",
        "pid_heading_deg",
        "pid_error_deg",
        "pid_rudder_deg",
    ]
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for i in range(len(pd["t"])):
            writer.writerow(
                {
                    "time_s": pd["t"][i],
                    "reference_deg": pid["reference_deg"][i],
                    "disturbance_equivalent_deg": pid["disturbance_equiv_deg"][i],
                    "pd_heading_deg": pd["psi_deg"][i],
                    "pd_error_deg": pd["error_deg"][i],
                    "pd_rudder_deg": pd["rudder_deg"][i],
                    "pid_heading_deg": pid["psi_deg"][i],
                    "pid_error_deg": pid["error_deg"][i],
                    "pid_rudder_deg": pid["rudder_deg"][i],
                }
            )


def plot_results(main_results: dict, cfg: SimulationConfig, fig_dir: Path) -> None:
    fig_dir.mkdir(parents=True, exist_ok=True)
    pid = main_results["ПИД"]

    plt.figure(figsize=(7.2, 4.0))
    for name, result in main_results.items():
        plt.plot(result["t"], result["psi_deg"], label=name)
    plt.plot(pid["t"], pid["reference_deg"], "--", label="Заданный курс")
    plt.axvline(cfg.disturbance_time, linestyle=":", linewidth=1, label="Возмущение")
    plt.xlabel("Время, с")
    plt.ylabel("Курс, град")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(fig_dir / "course.png", dpi=300)
    plt.close()

    plt.figure(figsize=(7.2, 4.0))
    for name, result in main_results.items():
        plt.plot(result["t"], result["error_deg"], label=name)
    plt.axvline(cfg.disturbance_time, linestyle=":", linewidth=1)
    plt.xlabel("Время, с")
    plt.ylabel("Ошибка курса, град")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(fig_dir / "error.png", dpi=300)
    plt.close()

    plt.figure(figsize=(7.2, 4.0))
    for name, result in main_results.items():
        plt.plot(result["t"], result["rudder_deg"], label=name)
    plt.axvline(cfg.disturbance_time, linestyle=":", linewidth=1)
    plt.xlabel("Время, с")
    plt.ylabel("Угол руля, град")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(fig_dir / "rudder.png", dpi=300)
    plt.close()


def plot_scheme(fig_dir: Path) -> None:
    """Сохранить структурную схему вычислительной модели."""
    plt.figure(figsize=(7.2, 2.8))
    ax = plt.gca()
    ax.axis("off")
    boxes = [
        (0.03, 0.42, 0.16, 0.24, "Заданный\nкурс"),
        (0.25, 0.42, 0.18, 0.24, "ПД / ПИД\nрегулятор"),
        (0.50, 0.42, 0.17, 0.24, "Рулевой\nпривод"),
        (0.75, 0.42, 0.18, 0.24, "Модель\nНомото"),
    ]
    for x, y, w, h, text in boxes:
        ax.add_patch(plt.Rectangle((x, y), w, h, fill=False, linewidth=1.2))
        ax.text(x + w / 2, y + h / 2, text, ha="center", va="center")
    for x1, x2 in ((0.19, 0.25), (0.43, 0.50), (0.67, 0.75)):
        ax.annotate("", xy=(x2, 0.54), xytext=(x1, 0.54), arrowprops={"arrowstyle": "->", "lw": 1.1})
    ax.plot([0.93, 0.96], [0.54, 0.54], linewidth=1.0)
    ax.plot([0.96, 0.96], [0.54, 0.24], linewidth=1.0)
    ax.plot([0.96, 0.34], [0.24, 0.24], linewidth=1.0)
    ax.annotate("", xy=(0.34, 0.42), xytext=(0.34, 0.24), arrowprops={"arrowstyle": "->", "lw": 1.0})
    ax.text(0.65, 0.12, "обратная связь по курсу и угловой скорости", ha="center", fontsize=9)
    ax.annotate(
        "эквивалентное\nвозмущение",
        xy=(0.84, 0.66),
        xytext=(0.84, 0.93),
        ha="center",
        fontsize=9,
        arrowprops={"arrowstyle": "->", "lw": 1.0},
    )
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.tight_layout()
    plt.savefig(fig_dir / "scheme.png", dpi=300)
    plt.close()


def print_summary(metrics_by_controller: dict, series: list[dict]) -> None:
    print("\nОсновной опыт: эквивалентное возмущение 10°")
    print("-" * 89)
    print(f"{'Регулятор':<10}{'tн, с':>10}{'σ, °':>10}{'tуст, с':>12}{'|e|max, °':>13}{'tвозв, с':>13}{'eуст, °':>11}")
    for name, m in metrics_by_controller.items():
        print(
            f"{name:<10}"
            f"{fmt(m['rise_time_s'], 1):>10}"
            f"{fmt(m['overshoot_deg'], 2):>10}"
            f"{fmt(m['settling_time_s'], 1):>12}"
            f"{fmt(m['max_deviation_after_disturbance_deg'], 2):>13}"
            f"{fmt(m['recovery_time_s'], 1):>13}"
            f"{fmt(m['steady_state_error_deg'], 2):>11}"
        )

    print("\nСерия опытов ПИД-регулятора")
    print("-" * 65)
    for row in series:
        print(
            f"δв={row['disturbance_equivalent_deg']:>2.0f}°: "
            f"макс. отклонение={row['max_deviation_deg']:.2f}°, "
            f"возврат={row['recovery_time_s']:.1f} с, "
            f"eуст={row['steady_state_error_deg']:.2f}°"
        )


if __name__ == "__main__":
    cfg = SimulationConfig()
    fig_dir = ROOT / "figures"
    results_dir = ROOT / "results"
    fig_dir.mkdir(exist_ok=True)
    results_dir.mkdir(exist_ok=True)

    main_results = {
        "ПД": simulate(PD, disturbance_deg=10.0, config=cfg),
        "ПИД": simulate(PID, disturbance_deg=10.0, config=cfg),
    }
    main_metrics = {
        name: calculate_metrics(result, config=cfg)
        for name, result in main_results.items()
    }

    disturbance_series = []
    for d in (5.0, 10.0, 15.0):
        result = simulate(PID, disturbance_deg=d, config=cfg)
        m = calculate_metrics(result, config=cfg)
        disturbance_series.append(
            {
                "disturbance_equivalent_deg": d,
                "max_deviation_deg": m["max_deviation_after_disturbance_deg"],
                "recovery_time_s": m["recovery_time_s"],
                "steady_state_error_deg": m["steady_state_error_deg"],
            }
        )

    save_main_metrics(main_metrics, results_dir / "main_metrics.csv")
    save_disturbance_series(disturbance_series, results_dir / "pid_disturbance_series.csv")
    save_timeseries(main_results, results_dir / "main_timeseries.csv")
    plot_results(main_results, cfg, fig_dir)
    plot_scheme(fig_dir)
    print_summary(main_metrics, disturbance_series)
    print("\nГотово. Графики сохранены в figures/, таблицы и временные ряды — в results/.")
