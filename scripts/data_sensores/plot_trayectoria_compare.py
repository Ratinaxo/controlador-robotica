#!/usr/bin/env python3
"""Compare executed e-puck trajectory (CSV) against planned A* path in real time."""

from __future__ import annotations

import argparse
import json
import sys
import warnings
from pathlib import Path

warnings.filterwarnings("ignore")

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.colors import ListedColormap
except ImportError:
    print("Error: matplotlib no esta instalado. Instalalo con: pip install matplotlib")
    sys.exit(1)

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS_DIR))

from paths import (
    CONTROLLER_TRAJECTORY_CSV,
    TRAJECTORY_COMPARE_PNG,
    artifact_paths,
)
from plot_style import add_legend_side_arg, legend_loc

DEFAULT_CSV = CONTROLLER_TRAJECTORY_CSV
DEFAULT_OUTPUT = TRAJECTORY_COMPARE_PNG

GRID_SIZE = 20
CELL_SIZE = 0.1
ORIGIN = -1.0
ANIMATION_INTERVAL_MS = 500


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot planned A* path vs executed trajectory.")
    parser.add_argument(
        "--name",
        required=True,
        help="World identifier used to locate grid and path JSON artifacts",
    )
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="Executed trajectory CSV")
    parser.add_argument(
        "--path",
        type=Path,
        default=None,
        help="Planned path JSON (default: scripts/output/{world}_path.json)",
    )
    parser.add_argument(
        "--grid",
        type=Path,
        default=None,
        help="Occupancy grid JSON (default: scripts/output/{world}_grid.json)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Output PNG for --no-realtime mode",
    )
    parser.add_argument(
        "--no-realtime",
        action="store_true",
        help="Render a single snapshot/PNG instead of live animation",
    )
    add_legend_side_arg(parser, default="left")
    return parser.parse_args()


def load_planned_path(path_file: Path) -> tuple[list[tuple[float, float]], tuple[float, float], tuple[float, float], float]:
    data = json.loads(path_file.read_text(encoding="utf-8"))
    planned = [(float(x), float(y)) for x, y in data["path_world"]]
    start = (planned[0][0], planned[0][1])
    goal = (planned[-1][0], planned[-1][1])
    length_m = float(data.get("length_m", 0.0))
    return planned, start, goal, length_m


def load_grid(grid_file: Path) -> list[list[int]] | None:
    if not grid_file.exists():
        return None
    data = json.loads(grid_file.read_text(encoding="utf-8"))
    return data["occupancy"]


def load_start_goal_from_grid(grid_file: Path) -> tuple[tuple[float, float], tuple[float, float]] | None:
    if not grid_file.exists():
        return None
    data = json.loads(grid_file.read_text(encoding="utf-8"))
    if "start_world" not in data or "goal_world" not in data:
        return None
    start = data["start_world"]
    goal = data["goal_world"]
    return (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))


def read_trajectory_csv(csv_file: Path):
    if not csv_file.exists():
        return None

    try:
        import pandas as pd

        df = pd.read_csv(csv_file)
        df.columns = df.columns.str.strip()
        if len(df) < 1:
            return None
        return df
    except Exception:
        return None


def path_length(points: list[tuple[float, float]]) -> float:
    total = 0.0
    for i in range(1, len(points)):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        total += ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    return total


class TrajectoryComparePlot:
    def __init__(
        self,
        planned: list[tuple[float, float]],
        start: tuple[float, float],
        goal: tuple[float, float],
        planned_length_m: float,
        occupancy: list[list[int]] | None,
        csv_file: Path,
        output_file: Path,
        realtime: bool,
        legend_side: str = "left",
    ) -> None:
        self.planned = planned
        self.start = start
        self.goal = goal
        self.planned_length_m = planned_length_m
        self.occupancy = occupancy
        self.csv_file = csv_file
        self.output_file = output_file
        self.realtime = realtime
        self.legend_side = legend_side

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title("Ruta A* vs Trayectoria Ejecutada")
        self._setup_axes()

    def _setup_axes(self) -> None:
        self.ax.set_xlim(ORIGIN, ORIGIN + GRID_SIZE * CELL_SIZE)
        self.ax.set_ylim(ORIGIN, ORIGIN + GRID_SIZE * CELL_SIZE)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.grid(True, linestyle="--", alpha=0.3)

        if self.occupancy is not None:
            cmap = ListedColormap(["#b8e6b8", "#404040"])
            extent = [ORIGIN, ORIGIN + GRID_SIZE * CELL_SIZE, ORIGIN, ORIGIN + GRID_SIZE * CELL_SIZE]
            self.ax.imshow(
                self.occupancy,
                cmap=cmap,
                origin="lower",
                extent=extent,
                vmin=0,
                vmax=1,
                interpolation="nearest",
                alpha=0.85,
            )

        planned_x = [p[0] for p in self.planned]
        planned_y = [p[1] for p in self.planned]
        self.ax.plot(
            planned_x,
            planned_y,
            linestyle="--",
            color="#1f77b4",
            linewidth=2.0,
            label="Ruta A* planificada",
            zorder=3,
        )
        self.ax.scatter(
            [self.start[0]],
            [self.start[1]],
            color="#1a9c2d",
            s=120,
            marker="o",
            edgecolors="black",
            label="Inicio",
            zorder=5,
        )
        self.ax.scatter(
            [self.goal[0]],
            [self.goal[1]],
            color="#d62728",
            s=120,
            marker="X",
            edgecolors="black",
            label="Meta",
            zorder=5,
        )
        (self.executed_line,) = self.ax.plot([], [], color="#d62728", linewidth=2.0, label="Trayectoria ejecutada", zorder=4)
        (self.current_point,) = self.ax.plot([], [], "o", color="#ff7f0e", markersize=8, label="Robot actual", zorder=6)
        self.ax.legend(loc=legend_loc(self.legend_side), fontsize="small")

    def _render_frame(self) -> None:
        df = read_trajectory_csv(self.csv_file)
        if df is None:
            self.ax.set_title(f"Esperando {self.csv_file} ...")
            return

        executed = list(zip(df["x_m"].astype(float), df["y_m"].astype(float)))
        exec_x = [p[0] for p in executed]
        exec_y = [p[1] for p in executed]

        self.executed_line.set_data(exec_x, exec_y)
        self.current_point.set_data([exec_x[-1]], [exec_y[-1]])

        sim_time = float(df["tiempo_s"].iloc[-1])
        waypoint_idx = int(df["waypoint_idx"].iloc[-1])
        estado = str(df["estado"].iloc[-1])
        dist_recorrida = float(df["dist_recorrida_m"].iloc[-1])
        exec_length = path_length(executed)
        delta_length = exec_length - self.planned_length_m

        self.ax.set_title(
            f"t={sim_time:.1f}s | wp={waypoint_idx + 1}/{len(self.planned)} | "
            f"estado={estado} | plan={self.planned_length_m:.2f}m | "
            f"ejec={exec_length:.2f}m | delta={delta_length:+.2f}m"
        )

    def run(self) -> None:
        if self.realtime:
            def update(_frame):
                self._render_frame()
                return self.executed_line, self.current_point

            ani = FuncAnimation(self.fig, update, interval=ANIMATION_INTERVAL_MS, cache_frame_data=False)
            _ = ani
            plt.show()
            return

        self._render_frame()
        self.fig.savefig(self.output_file, dpi=150, bbox_inches="tight")
        print(f"Wrote {self.output_file}")
        if len(read_trajectory_csv(self.csv_file) or []) == 0:
            print("Advertencia: CSV vacio o inexistente; PNG solo muestra ruta planificada.")


def main() -> int:
    args = parse_args()
    artifacts = artifact_paths(args.name)

    if args.path is None:
        args.path = artifacts["path_json"]
    if args.grid is None:
        args.grid = artifacts["grid_json"]

    if not args.path.exists():
        print(f"Error: path JSON not found: {args.path}", file=sys.stderr)
        print(f"Run: uv run python plan_path.py --name {args.name}", file=sys.stderr)
        return 1

    planned, start, goal, planned_length_m = load_planned_path(args.path)
    occupancy = load_grid(args.grid)

    marker_override = load_start_goal_from_grid(args.grid)
    if marker_override is not None:
        start, goal = marker_override

    plot = TrajectoryComparePlot(
        planned=planned,
        start=start,
        goal=goal,
        planned_length_m=planned_length_m,
        occupancy=occupancy,
        csv_file=args.csv,
        output_file=args.output,
        realtime=not args.no_realtime,
        legend_side=args.legend,
    )
    plot.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
