#!/usr/bin/env python3
"""Plan a path with A* from exported occupancy grid."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from astar import astar, path_length_m, path_to_dict
from occupancy_grid import GRID, load_grid_json
from paths import artifact_paths, ensure_output_dir
from plot_style import add_legend_side_arg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plan A* path on exported occupancy grid.")
    parser.add_argument(
        "--name",
        required=True,
        help="World identifier used to locate input/output artifacts",
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=None,
        help="Input occupancy grid JSON (default: scripts/output/{name}_grid.json)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output path JSON (default: scripts/output/{name}_path.json)",
    )
    parser.add_argument(
        "--plot-output",
        type=Path,
        default=None,
        help="Output PNG when --plot is used (default: scripts/output/{name}_path.png)",
    )
    parser.add_argument("--plot", action="store_true", help="Plot grid with planned path")
    parser.add_argument("--no-show", action="store_true", help="Do not open plot window")
    add_legend_side_arg(parser, default="right")
    return parser.parse_args()


def validate_path(
    grid: list[list[int]],
    path_cells: list[tuple[int, int]],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> None:
    if path_cells[0] != start:
        raise ValueError(f"Path must start at {start}, got {path_cells[0]}")
    if path_cells[-1] != goal:
        raise ValueError(f"Path must end at {goal}, got {path_cells[-1]}")

    for cx, cy in path_cells:
        if not (0 <= cx < GRID and 0 <= cy < GRID):
            raise ValueError(f"Path cell {(cx, cy)} is outside grid")
        if grid[cy][cx] != 0:
            raise ValueError(f"Path cell {(cx, cy)} is not free")

    for i in range(1, len(path_cells)):
        prev = path_cells[i - 1]
        curr = path_cells[i]
        if _manhattan_steps(prev, curr) != 1:
            raise ValueError(f"Non-adjacent step from {prev} to {curr}")


def _manhattan_steps(a: tuple[int, int], b: tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def main() -> int:
    args = parse_args()
    artifacts = artifact_paths(args.name)

    if args.json is None:
        args.json = artifacts["grid_json"]
    if args.output is None:
        args.output = artifacts["path_json"]
    if args.plot_output is None:
        args.plot_output = artifacts["path_png"]

    if not args.json.exists():
        print(f"Error: grid JSON not found: {args.json}", file=sys.stderr)
        print(
            f"Run: uv run python export_occupancy_grid.py --name {args.name} "
            f"--start-cell CX,CY --goal-cell CX,CY",
            file=sys.stderr,
        )
        return 1

    data = load_grid_json(args.json)
    grid = data["occupancy"]
    start = tuple(data["start_cell"])
    goal = tuple(data["goal_cell"])
    cell_size = data.get("cell_size_m", 0.1)
    world = data.get("world", args.name)

    path_cells = astar(grid, start, goal)
    if path_cells is None:
        print(f"Error: no path found from {start} to {goal}", file=sys.stderr)
        return 1

    validate_path(grid, path_cells, start, goal)

    ensure_output_dir()
    payload = path_to_dict(
        path_cells,
        world=world,
        source_grid=str(args.json),
        start=start,
        goal=goal,
        cell_size=cell_size,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print(f"World: {world}")
    print(f"Start cell: {start}")
    print(f"Goal cell: {goal}")
    print(f"Path nodes: {len(path_cells)}")
    print(f"Path length: {path_length_m(path_cells, cell_size)} m")
    print(f"Wrote path JSON: {args.output}")

    if args.plot:
        try:
            from plot_occupancy_grid import plot_occupancy_grid
        except ImportError as exc:
            print(f"Error: cannot plot path ({exc})", file=sys.stderr)
            return 1

        title = f"{world} - Ruta A* ({len(path_cells)} nodos, {payload['length_m']} m)"
        plot_occupancy_grid(
            grid,
            start,
            goal,
            path=path_cells,
            title=title,
            output_path=args.plot_output,
            show=not args.no_show,
            legend_side=args.legend,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
