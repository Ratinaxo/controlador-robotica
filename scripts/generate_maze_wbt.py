#!/usr/bin/env python3
"""Build a Webots .wbt world from an exported occupancy grid JSON."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from occupancy_grid import GridSpec, count_wall_cells, grid_spec_from_dict, load_grid_json
from paths import REPO_ROOT, artifact_paths, ensure_output_dir
from wbt_markers import markers_vrml

SNAP = 0.025
WALL_HEIGHT = 0.05
MARKER_Z = 0.005


@dataclass
class WallNode:
    tx: float
    ty: float
    tz: float
    rotation: str | None
    sx: float
    sy: float
    sz: float
    name: str | None = None

    def to_vrml(self) -> str:
        lines = ["Wall {"]
        lines.append(f"  translation {self.tx} {self.ty} {self.tz}")
        if self.rotation:
            lines.append(f"  rotation {self.rotation}")
        if self.name:
            lines.append(f'  name "{self.name}"')
        lines.append(f"  size {self.sx} {self.sy} {self.sz}")
        lines.append("}")
        return "\n".join(lines)


def snap(value: float) -> float:
    return round(round(value / SNAP) * SNAP, 3)


def cell_center(cx: int, cy: int, spec: GridSpec) -> tuple[float, float]:
    x = round(spec.origin + (cx + 0.5) * spec.cell, 3)
    y = round(spec.origin + (cy + 0.5) * spec.cell, 3)
    return x, y


def is_wall_cell(grid: list[list[int]], cx: int, cy: int, spec: GridSpec) -> bool:
    return 0 <= cx < spec.size and 0 <= cy < spec.size and grid[cy][cx] == 1


def build_wall_nodes(
    grid: list[list[int]],
    spec: GridSpec,
    _start: tuple[int, int],
    _goal: tuple[int, int],
) -> list[WallNode]:
    nodes: list[WallNode] = []

    for cy in range(spec.size):
        for cx in range(spec.size):
            if not is_wall_cell(grid, cx, cy, spec):
                continue

            center_x, center_y = cell_center(cx, cy, spec)
            nodes.append(
                WallNode(
                    tx=center_x,
                    ty=center_y,
                    tz=0.0,
                    rotation=None,
                    sx=spec.cell,
                    sy=spec.cell,
                    sz=WALL_HEIGHT,
                    name=f"wall_cell({cx},{cy})",
                )
            )

    return nodes


def print_occupancy(
    grid: list[list[int]],
    start: tuple[int, int],
    goal: tuple[int, int],
    spec: GridSpec,
) -> None:
    print(f"# Occupancy grid (0=free, 1=wall). Size {spec.size}x{spec.size}.")
    print(f"# Start cell {start}, goal cell {goal}")
    for cy in range(spec.size):
        row = []
        for cx in range(spec.size):
            ch = "S" if (cx, cy) == start else "G" if (cx, cy) == goal else str(grid[cy][cx])
            row.append(ch.rjust(1))
        print("".join(row))


def validate_nodes(nodes: list[WallNode]) -> None:
    for node in nodes:
        for value in (node.tx, node.ty, node.tz):
            ratio = round(value / SNAP)
            if abs(value - ratio * SNAP) > 1e-6:
                raise ValueError(f"Value {value} is not a multiple of {SNAP}")


def build_world_file(
    nodes: list[WallNode],
    *,
    world_name: str,
    start_world: tuple[float, float],
    goal_world: tuple[float, float],
    spec: GridSpec,
) -> str:
    start_xyz = (start_world[0], start_world[1], MARKER_Z)
    goal_xyz = (goal_world[0], goal_world[1], MARKER_Z)
    robot_x, robot_y = start_world[0], start_world[1]
    floor = spec.world_span
    tile = spec.cell

    header = f"""#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/apartment_structure/protos/Wall.proto"

WorldInfo {{
}}
Viewpoint {{
  orientation -0.11000835818142425 0.9722126184264838 -0.20664168432953547 1.4348233594531277
  position -0.3623608000878591 0.9324751020350198 1.713356738088449
}}
TexturedBackground {{
  luminosity 10
}}
RectangleArena {{
  floorSize {floor} {floor}
  floorTileSize {tile} {tile}
}}
"""
    walls = "\n".join(node.to_vrml() for node in nodes)
    marker_nodes = markers_vrml(start_xyz, goal_xyz)
    robot = f"""E-puck {{
  translation {robot_x} {robot_y} 0
  rotation 0 0 1 -1.5707953071795862
  controller "controlador_Proyectofinal"
  controllerArgs [
    "--path" "scripts/output/{world_name}_path.json"
    "--csv" "data_sensores/trayectoria_ejecutada.csv"
  ]
}}
"""
    return header + walls + "\n" + marker_nodes + robot


def _reject_worlds_output(path: Path, root: Path) -> None:
    worlds_dir = (root / "worlds").resolve()
    try:
        if path.resolve().is_relative_to(worlds_dir):
            raise SystemExit(
                f"Refusing to write inside worlds/: {path}\n"
                "Use --output under scripts/output/ (default: scripts/output/{world}_generated.wbt)."
            )
    except ValueError:
        pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build Webots .wbt from exported occupancy grid JSON.")
    parser.add_argument(
        "--name",
        required=True,
        help="World identifier used to locate grid JSON and name output .wbt",
    )
    parser.add_argument(
        "--grid",
        type=Path,
        default=None,
        help="Input grid JSON (default: scripts/output/{name}_grid.json)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output .wbt path (default: scripts/output/{name}_generated.wbt)",
    )
    parser.add_argument(
        "--stdout",
        action="store_true",
        help="Print wall VRML nodes to stdout instead of writing --output",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    artifacts = artifact_paths(args.name)
    grid_path = args.grid or artifacts["grid_json"]

    if not grid_path.exists():
        print(f"Error: grid JSON not found: {grid_path.resolve()}", file=sys.stderr)
        print(
            f"Run: uv run python export_occupancy_grid.py --name {args.name} "
            f"--start-cell CX,CY --goal-cell CX,CY",
            file=sys.stderr,
        )
        return 1

    if args.output is None:
        args.output = artifacts["generated_wbt"]

    if not args.stdout:
        _reject_worlds_output(args.output, REPO_ROOT)

    data = load_grid_json(grid_path)
    spec = grid_spec_from_dict(data)
    grid = data["occupancy"]
    start = tuple(data["start_cell"])
    goal = tuple(data["goal_cell"])
    start_world = (float(data["start_world"][0]), float(data["start_world"][1]))
    goal_world = (float(data["goal_world"][0]), float(data["goal_world"][1]))
    world = data.get("world", args.name)

    nodes = build_wall_nodes(grid, spec, start, goal)

    wall_cells = count_wall_cells(grid)
    if len(nodes) != wall_cells:
        raise ValueError(f"Expected {wall_cells} wall nodes, got {len(nodes)}")

    print(
        f"# Built {len(nodes)} full-cell wall nodes from {grid_path.name} "
        f"(grid={spec.size}x{spec.size}, cell={spec.cell}m, walls={wall_cells})",
        file=sys.stderr,
    )

    if args.stdout:
        for node in nodes:
            print(node.to_vrml())
            print()
    else:
        ensure_output_dir()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            build_world_file(
                nodes,
                world_name=world,
                start_world=start_world,
                goal_world=goal_world,
                spec=spec,
            ),
            encoding="utf-8",
        )
        print(f"Wrote {args.output} (world={world}, floor={spec.world_span}m)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
