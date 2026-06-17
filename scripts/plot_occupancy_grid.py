#!/usr/bin/env python3
"""Visualize occupancy grid with matplotlib."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
except ImportError:
    print("Error: matplotlib no esta instalado. Instalalo con: pip install matplotlib")
    sys.exit(1)

from occupancy_grid import (
    DEFAULT_DIFFICULTY,
    DEFAULT_SEED,
    DEFAULT_SPEC,
    GridSpec,
    MIN_DIFFICULTY,
    MAX_DIFFICULTY,
    build_occupancy_grid,
    grid_spec_from_dict,
    is_connected,
    load_grid_json,
    parse_cell_arg,
)
from paths import artifact_paths, ensure_output_dir
from plot_style import add_legend_side_arg, legend_loc


def load_path_json(path: str | Path) -> list[tuple[int, int]]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return [tuple(cell) for cell in data["path_cells"]]


def plot_occupancy_grid(
    grid: list[list[int]],
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    title: str,
    path: list[tuple[int, int]] | None = None,
    output_path: str | Path | None = None,
    show: bool = True,
    legend_side: str = "right",
    grid_size: int | None = None,
) -> None:
    size = grid_size if grid_size is not None else len(grid)
    cmap = ListedColormap(["#b8e6b8", "#404040"])
    fig, ax = plt.subplots(figsize=(8, 8))

    ax.imshow(grid, cmap=cmap, origin="lower", vmin=0, vmax=1, interpolation="nearest")
    ax.set_xlim(-0.5, size - 0.5)
    ax.set_ylim(-0.5, size - 0.5)
    ax.set_xticks(range(size))
    ax.set_yticks(range(size))
    ax.set_xlabel("Celda X (cx)")
    ax.set_ylabel("Celda Y (cy)")
    ax.set_title(title)
    ax.grid(True, which="both", linestyle="--", alpha=0.4)
    ax.set_aspect("equal")

    if path:
        path_x = [cx for cx, _ in path]
        path_y = [cy for _, cy in path]
        ax.plot(path_x, path_y, color="#1565c0", linewidth=2.5, alpha=0.9, zorder=3)
        ax.scatter(path_x, path_y, s=18, color="#1565c0", alpha=0.35, zorder=2)

    sx, sy = start
    gx, gy = goal
    ax.plot(sx, sy, marker="o", markersize=12, markerfacecolor="#1f77b4", markeredgecolor="white", linestyle="None", zorder=4)
    ax.plot(gx, gy, marker="o", markersize=12, markerfacecolor="#d62728", markeredgecolor="white", linestyle="None", zorder=4)
    ax.annotate("S", (sx, sy), textcoords="offset points", xytext=(0, 8), ha="center", color="#1f77b4", fontweight="bold", zorder=5)
    ax.annotate("G", (gx, gy), textcoords="offset points", xytext=(0, -14), ha="center", color="#d62728", fontweight="bold", zorder=5)

    legend_handles = [
        plt.Line2D([0], [0], marker="s", color="w", markerfacecolor="#b8e6b8", markersize=10, label="Libre (0)"),
        plt.Line2D([0], [0], marker="s", color="w", markerfacecolor="#404040", markersize=10, label="Pared (1)"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#1f77b4", markersize=10, label="Inicio (S)"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#d62728", markersize=10, label="Meta (G)"),
    ]
    if path:
        legend_handles.append(
            plt.Line2D([0], [0], color="#1565c0", linewidth=2.5, label="Ruta A*")
        )
    ax.legend(handles=legend_handles, loc=legend_loc(legend_side), framealpha=0.9)
    fig.tight_layout()

    if output_path is not None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Wrote {output_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot occupancy grid.")
    parser.add_argument(
        "--name",
        default=None,
        help="World identifier for artifact paths (required with --seed)",
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=None,
        help="Input JSON path (default: scripts/output/{name}_grid.json when --name is set)",
    )
    parser.add_argument(
        "--path",
        type=Path,
        default=None,
        help="Optional path JSON to overlay (default: scripts/output/{name}_path.json when --name is set)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PNG path (default: scripts/output/{name}_grid.png when --name is set)",
    )
    parser.add_argument("--seed", type=int, default=None, help="Regenerate grid from seed instead of JSON")
    parser.add_argument(
        "--start-cell",
        default=None,
        metavar="CX,CY",
        help="Start cell when using --seed (required with --seed)",
    )
    parser.add_argument(
        "--goal-cell",
        default=None,
        metavar="CX,CY",
        help="Goal cell when using --seed (required with --seed)",
    )
    parser.add_argument(
        "--difficulty",
        type=int,
        default=DEFAULT_DIFFICULTY,
        choices=range(MIN_DIFFICULTY, MAX_DIFFICULTY + 1),
        metavar=f"{MIN_DIFFICULTY}-{MAX_DIFFICULTY}",
        help="Maze density when using --seed (1=few walls, 10=many walls)",
    )
    parser.add_argument(
        "--grid-size",
        type=int,
        default=None,
        help=f"Grid size when using --seed (default: {DEFAULT_SPEC.size})",
    )
    parser.add_argument(
        "--cell-size",
        type=float,
        default=None,
        help=f"Cell size in meters when using --seed (default: {DEFAULT_SPEC.cell})",
    )
    parser.add_argument("--no-show", action="store_true", help="Do not open interactive window")
    parser.add_argument("--no-save", action="store_true", help="Do not save PNG file")
    add_legend_side_arg(parser, default="right")
    return parser.parse_args()


def resolve_artifacts(args: argparse.Namespace) -> tuple[str, dict[str, Path]]:
    if args.name is None:
        if args.seed is not None:
            print("Error: --name is required when using --seed", file=sys.stderr)
            sys.exit(1)
        if args.json is None:
            print("Error: --name or --json is required", file=sys.stderr)
            sys.exit(1)
        data = load_grid_json(args.json)
        world = data.get("world", "grid")
        return world, {}

    artifacts = artifact_paths(args.name)
    return artifacts["world"], artifacts


def load_grid_data(
    args: argparse.Namespace,
) -> tuple[list[list[int]], tuple[int, int], tuple[int, int], str, int]:
    world, artifacts = resolve_artifacts(args)

    if args.seed is not None:
        if args.start_cell is None or args.goal_cell is None:
            print("Error: --start-cell and --goal-cell are required when using --seed", file=sys.stderr)
            sys.exit(1)
        grid_size = args.grid_size if args.grid_size is not None else DEFAULT_SPEC.size
        cell_size = args.cell_size if args.cell_size is not None else DEFAULT_SPEC.cell
        try:
            spec = GridSpec(size=grid_size, cell=cell_size)
        except ValueError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            sys.exit(1)
        try:
            start = parse_cell_arg(args.start_cell, "--start-cell", spec)
            goal = parse_cell_arg(args.goal_cell, "--goal-cell", spec)
        except ValueError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            sys.exit(1)
        grid = build_occupancy_grid(args.seed, start=start, goal=goal, difficulty=args.difficulty, spec=spec)
        if not is_connected(grid, start, goal):
            print(
                f"Error: no path from {start} to {goal} (seed={args.seed}, difficulty={args.difficulty})",
                file=sys.stderr,
            )
            sys.exit(1)
        title = (
            f"{world} - Grilla de ocupacion "
            f"({spec.size}x{spec.size}, cell={spec.cell}m, seed={args.seed}, difficulty={args.difficulty})"
        )
        return grid, start, goal, title, spec.size

    json_path = args.json if args.json is not None else artifacts.get("grid_json")
    if json_path is None:
        print("Error: --json or --name is required", file=sys.stderr)
        sys.exit(1)

    if not json_path.exists():
        print(f"Error: no se encontro '{json_path}'.", file=sys.stderr)
        print(
            f"Ejecuta primero: uv run python export_occupancy_grid.py --name {args.name} "
            f"--start-cell CX,CY --goal-cell CX,CY",
            file=sys.stderr,
        )
        sys.exit(1)

    data = load_grid_json(json_path)
    spec = grid_spec_from_dict(data)
    grid = data["occupancy"]
    start = tuple(data["start_cell"])
    goal = tuple(data["goal_cell"])
    seed = data.get("seed", DEFAULT_SEED)
    difficulty = data.get("difficulty", DEFAULT_DIFFICULTY)
    world = data.get("world", world)
    title = (
        f"{world} - Grilla de ocupacion "
        f"({spec.size}x{spec.size}, cell={spec.cell}m, seed={seed}, difficulty={difficulty})"
    )
    return grid, start, goal, title, spec.size


def main() -> int:
    args = parse_args()
    _, artifacts = resolve_artifacts(args)

    if args.json is None and artifacts:
        args.json = artifacts["grid_json"]
    if args.output is None and artifacts:
        args.output = artifacts["grid_png"]

    grid, start, goal, title, grid_size = load_grid_data(args)

    path = None
    if args.path is not None:
        if not args.path.exists():
            print(f"Error: path JSON not found: {args.path}", file=sys.stderr)
            return 1
        path = load_path_json(args.path)
        title = f"{title} + Ruta A* ({len(path)} nodos)"
    elif artifacts and artifacts.get("path_json") and artifacts["path_json"].exists():
        path = load_path_json(artifacts["path_json"])
        title = f"{title} + Ruta A* ({len(path)} nodos)"

    if not args.no_save:
        ensure_output_dir()

    plot_occupancy_grid(
        grid,
        start,
        goal,
        path=path,
        title=title,
        output_path=None if args.no_save else args.output,
        show=not args.no_show,
        legend_side=args.legend,
        grid_size=grid_size,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
