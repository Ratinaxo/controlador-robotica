#!/usr/bin/env python3
"""Compute and plot navigation metrics for one or two scenarios (Facil vs Dificil)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("Error: matplotlib no esta instalado. Instalalo con: pip install matplotlib", file=sys.stderr)
    sys.exit(1)

from navigation_metrics import compute_metrics
from paths import (
    SCENARIOS_COMPARISON_PNG,
    artifact_paths,
    ensure_output_dir,
    metrics_json_for,
    metrics_png_for,
    trajectory_csv_for,
)

METRIC_LABELS = {
    "time_to_goal_s": "Tiempo hasta meta (s)",
    "planned_length_m": "Longitud planificada (m)",
    "executed_length_m": "Longitud ejecutada (m)",
    "length_delta_m": "Delta longitud (m)",
    "final_position_error_m": "Error posicion final (m)",
}

METRIC_UNITS = {
    "time_to_goal_s": "s",
    "planned_length_m": "m",
    "executed_length_m": "m",
    "length_delta_m": "m",
    "final_position_error_m": "m",
}

METRIC_KEYS = list(METRIC_LABELS.keys())
SCENARIO_COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]


def parse_csv_overrides(pairs: list[str]) -> dict[str, Path]:
    mapping: dict[str, Path] = {}
    for item in pairs:
        if "=" not in item:
            raise argparse.ArgumentTypeError(
                f"Invalid --csv entry {item!r}; expected NAME=path/to/file.csv"
            )
        name, csv_path = item.split("=", 1)
        mapping[name.strip()] = Path(csv_path.strip())
    return mapping


def split_csv_args(csv_args: list[str]) -> tuple[dict[str, Path], Path | None]:
    overrides: dict[str, Path] = {}
    single: Path | None = None
    for item in csv_args:
        if "=" in item:
            overrides.update(parse_csv_overrides([item]))
        else:
            single = Path(item)
    return overrides, single


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compute navigation metrics and generate plots.")
    parser.add_argument("--name", help="World identifier for single-scenario mode")
    parser.add_argument(
        "--compare",
        nargs=2,
        metavar=("NAME_A", "NAME_B"),
        help="Compare two scenarios, e.g. --compare Facil Dificil",
    )
    parser.add_argument(
        "--csv",
        action="append",
        default=[],
        help="Trajectory CSV (single mode) or NAME=PATH pairs (compare mode)",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        default=None,
        help="Output metrics JSON (single mode; default: scripts/output/{name}_metrics.json)",
    )
    parser.add_argument(
        "--output-png",
        type=Path,
        default=None,
        help="Output dashboard PNG (single mode; default: scripts/output/{name}_metrics.png)",
    )
    parser.add_argument(
        "--comparison-png",
        type=Path,
        default=SCENARIOS_COMPARISON_PNG,
        help="Output PNG for --compare mode",
    )
    parser.add_argument("--no-show", action="store_true", help="Do not open plot windows")
    return parser.parse_args()


def resolve_csv(name: str, csv_overrides: dict[str, Path], explicit: Path | None) -> Path:
    if name in csv_overrides:
        return csv_overrides[name]
    if explicit is not None:
        return explicit
    return trajectory_csv_for(name)


def run_single(name: str, csv_path: Path, output_json: Path, output_png: Path, *, show: bool) -> dict:
    artifacts = artifact_paths(name)
    metrics = compute_metrics(
        csv_path,
        artifacts["path_json"],
        grid_json_path=artifacts["grid_json"],
        world=name,
    )
    ensure_output_dir()
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    plot_single_dashboard(metrics, output_png, show=show)
    print(json.dumps(metrics, indent=2))
    print(f"Wrote {output_json}")
    print(f"Wrote {output_png}")
    return metrics


def plot_single_dashboard(metrics: dict, output_png: Path, *, show: bool) -> None:
    world = metrics["world"]
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    fig.suptitle(f"Metricas de navegacion — {world}", fontsize=14)

    ax = axes[0, 0]
    ax.bar(["Planificada", "Ejecutada"], [metrics["planned_length_m"], metrics["executed_length_m"]], color=["#1f77b4", "#d62728"])
    ax.set_ylabel("Longitud (m)")
    ax.set_title("Longitud de ruta")
    ax.grid(axis="y", linestyle="--", alpha=0.4)

    ax = axes[0, 1]
    delta = metrics["length_delta_m"]
    color = "#2ca02c" if abs(delta) < 0.05 else "#ff7f0e"
    ax.bar(["Delta"], [delta], color=color)
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_ylabel("Metros")
    ax.set_title("Diferencia ejecutada - planificada")
    ax.grid(axis="y", linestyle="--", alpha=0.4)

    ax = axes[1, 0]
    time_val = metrics["time_to_goal_s"]
    if time_val is None:
        ax.text(0.5, 0.5, "Meta no alcanzada", ha="center", va="center", transform=ax.transAxes)
        ax.set_axis_off()
    else:
        ax.bar(["Tiempo"], [time_val], color="#9467bd")
        ax.set_ylabel("Segundos")
        ax.set_title("Tiempo hasta meta")
        ax.grid(axis="y", linestyle="--", alpha=0.4)

    ax = axes[1, 1]
    ax.bar(["Error"], [metrics["final_position_error_m"]], color="#8c564b")
    ax.set_ylabel("Metros")
    ax.set_title("Error posicion final (odometria)")
    ax.grid(axis="y", linestyle="--", alpha=0.4)

    fig.tight_layout()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png, dpi=150, bbox_inches="tight")
    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_comparison(all_metrics: list[dict], output_png: Path, *, show: bool) -> None:
    names = [m["world"] for m in all_metrics]
    title = "Comparacion de metricas — " + " vs ".join(names)

    fig, axes = plt.subplots(len(METRIC_KEYS), 1, figsize=(10, 14), sharex=False)
    fig.suptitle(title, fontsize=14)

    for ax, key in zip(axes, METRIC_KEYS):
        values = [float(m[key]) if m.get(key) is not None else 0.0 for m in all_metrics]
        colors = SCENARIO_COLORS[: len(names)]
        ax.bar(names, values, color=colors)
        ax.set_ylabel(METRIC_UNITS[key])
        ax.set_title(METRIC_LABELS[key])
        ax.grid(axis="y", linestyle="--", alpha=0.4)
        if key == "length_delta_m":
            ax.axhline(0.0, color="black", linewidth=0.8)

    fig.tight_layout()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png, dpi=150, bbox_inches="tight")
    if show:
        plt.show()
    else:
        plt.close(fig)


def run_compare(
    names: tuple[str, str],
    csv_overrides: dict[str, Path],
    comparison_png: Path,
    *,
    show: bool,
) -> list[dict]:
    results: list[dict] = []
    for name in names:
        csv_path = resolve_csv(name, csv_overrides, None)
        artifacts = artifact_paths(name)
        metrics = compute_metrics(
            csv_path,
            artifacts["path_json"],
            grid_json_path=artifacts["grid_json"],
            world=name,
        )
        out_json = metrics_json_for(name)
        out_png = metrics_png_for(name)
        ensure_output_dir()
        out_json.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        plot_single_dashboard(metrics, out_png, show=False)
        results.append(metrics)
        print(f"{name}: wrote {out_json} and {out_png}")

    plot_comparison(results, comparison_png, show=show)
    print(f"Wrote {comparison_png}")
    return results


def main() -> int:
    args = parse_args()
    show = not args.no_show
    csv_overrides, single_csv = split_csv_args(args.csv)

    if args.compare:
        name_a, name_b = args.compare
        run_compare(
            (name_a, name_b),
            csv_overrides,
            args.comparison_png,
            show=show,
        )
        return 0

    if not args.name:
        print("Error: provide --name for single mode or --compare NAME_A NAME_B", file=sys.stderr)
        return 1

    csv_path = resolve_csv(args.name, csv_overrides, single_csv)
    output_json = args.output_json or metrics_json_for(args.name)
    output_png = args.output_png or metrics_png_for(args.name)
    run_single(args.name, csv_path, output_json, output_png, show=show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
