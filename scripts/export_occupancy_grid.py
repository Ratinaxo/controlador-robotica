#!/usr/bin/env python3
"""Generate occupancy grid matrix and export to JSON/CSV."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from occupancy_grid import (
    DEFAULT_DIFFICULTY,
    DEFAULT_SEED,
    GRID,
    MAX_DIFFICULTY,
    MIN_DIFFICULTY,
    build_occupancy_grid,
    cell_to_world,
    count_free_cells,
    count_wall_cells,
    is_connected,
    load_grid_json,
    parse_cell_arg,
    save_grid_csv,
    save_grid_json,
)
from paths import artifact_paths, ensure_output_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate occupancy grid matrix and export to JSON/CSV.")
    parser.add_argument(
        "--name",
        required=True,
        help="World identifier for output files (e.g. laberinto1 -> output/laberinto1_grid.json)",
    )
    parser.add_argument(
        "--start-cell",
        required=True,
        metavar="CX,CY",
        help="Start cell coordinates (e.g. 18,18)",
    )
    parser.add_argument(
        "--goal-cell",
        required=True,
        metavar="CX,CY",
        help="Goal cell coordinates (e.g. 10,0)",
    )
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help="Maze generation seed")
    parser.add_argument(
        "--difficulty",
        type=int,
        default=DEFAULT_DIFFICULTY,
        choices=range(MIN_DIFFICULTY, MAX_DIFFICULTY + 1),
        metavar=f"{MIN_DIFFICULTY}-{MAX_DIFFICULTY}",
        help="Maze density: 1=few walls, 10=many walls",
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=None,
        help="Output JSON path (default: scripts/output/{name}_grid.json)",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Output CSV path (default: scripts/output/{name}_grid.csv)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    artifacts = artifact_paths(args.name)

    if args.json is None:
        args.json = artifacts["grid_json"]
    if args.csv is None:
        args.csv = artifacts["grid_csv"]

    try:
        start = parse_cell_arg(args.start_cell, "--start-cell")
        goal = parse_cell_arg(args.goal_cell, "--goal-cell")
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    if start == goal:
        print("Error: start and goal must be different cells", file=sys.stderr)
        return 1

    grid = build_occupancy_grid(args.seed, start=start, goal=goal, difficulty=args.difficulty)

    if not is_connected(grid, start, goal):
        print(
            f"Error: no path from {start} to {goal} with seed={args.seed}, difficulty={args.difficulty}",
            file=sys.stderr,
        )
        print("Try lowering --difficulty or changing --seed.", file=sys.stderr)
        return 1

    start_world = cell_to_world(start[0], start[1])
    goal_world = cell_to_world(goal[0], goal[1])

    ensure_output_dir()
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    save_grid_json(
        grid,
        args.json,
        world=args.name,
        seed=args.seed,
        start=start,
        goal=goal,
        start_world=start_world,
        goal_world=goal_world,
        difficulty=args.difficulty,
    )
    save_grid_csv(grid, args.csv)

    loaded = load_grid_json(args.json)
    start_center = cell_to_world(start[0], start[1])
    goal_center = cell_to_world(goal[0], goal[1])

    print(f"World: {args.name}")
    print(f"Seed: {args.seed}")
    print(f"Difficulty: {args.difficulty}")
    print(f"Grid: {GRID}x{GRID} (cell={loaded['cell_size_m']} m)")
    print(f"Start cell: {start} (world {start_world}, center {start_center})")
    print(f"Goal cell: {goal} (world {goal_world}, center {goal_center})")
    print(f"Free cells: {count_free_cells(grid)}")
    print(f"Wall cells: {count_wall_cells(grid)}")
    print(f"Connected start->goal: {is_connected(grid, start, goal)}")
    print(f"Wrote JSON: {args.json}")
    print(f"Wrote CSV: {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
