"""Parse START_MARKER and GOAL_MARKER from Webots .wbt files."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

from occupancy_grid import GRID, world_to_cell

_MARKER_BLOCK = re.compile(
    r"DEF\s+(START_MARKER|GOAL_MARKER)\s+Solid\s*\{.*?translation\s+"
    r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)",
    re.DOTALL,
)


def parse_markers_from_wbt(path: str | Path) -> dict[str, Any]:
    content = Path(path).read_text(encoding="utf-8")
    markers: dict[str, tuple[float, float, float]] = {}

    for match in _MARKER_BLOCK.finditer(content):
        name = match.group(1)
        markers[name] = (float(match.group(2)), float(match.group(3)), float(match.group(4)))

    missing = {"START_MARKER", "GOAL_MARKER"} - set(markers)
    if missing:
        raise ValueError(f"Missing marker definitions in {path}: {', '.join(sorted(missing))}")

    start = markers["START_MARKER"]
    goal = markers["GOAL_MARKER"]
    return {
        "start_world": [start[0], start[1]],
        "goal_world": [goal[0], goal[1]],
        "start_translation": start,
        "goal_translation": goal,
    }


def resolve_start_goal_cells(wbt_path: str | Path) -> tuple[tuple[int, int], tuple[int, int]]:
    markers = parse_markers_from_wbt(wbt_path)
    start_cell = world_to_cell(markers["start_world"][0], markers["start_world"][1])
    goal_cell = world_to_cell(markers["goal_world"][0], markers["goal_world"][1])

    if start_cell is None:
        raise ValueError(
            f"START_MARKER at {markers['start_world']} is outside the {GRID}x{GRID} grid"
        )
    if goal_cell is None:
        raise ValueError(
            f"GOAL_MARKER at {markers['goal_world']} is outside the {GRID}x{GRID} grid"
        )
    return start_cell, goal_cell


def markers_vrml(start_xyz: tuple[float, float, float], goal_xyz: tuple[float, float, float]) -> str:
    sx, sy, sz = start_xyz
    gx, gy, gz = goal_xyz
    return f"""DEF START_MARKER Solid {{
  translation {sx} {sy} {sz}
  rotation 1 0 0 1.5708
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.1 0.85 0.2
        roughness 0.6
        metalness 0
      }}
      geometry Cylinder {{
        height 0.01
        radius 0.08
        subdivision 24
      }}
    }}
  ]
  name "start"
}}
DEF GOAL_MARKER Solid {{
  translation {gx} {gy} {gz}
  rotation 1 0 0 1.5708
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.9 0.15 0.1
        roughness 0.6
        metalness 0
      }}
      geometry Cylinder {{
        height 0.01
        radius 0.08
        subdivision 24
      }}
    }}
  ]
  name "goal"
}}
"""
