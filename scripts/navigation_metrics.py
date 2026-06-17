"""Compute navigation performance metrics from trajectory CSV and path JSON."""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import pandas as pd


def path_length(points: list[tuple[float, float]]) -> float:
    total = 0.0
    for i in range(1, len(points)):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def load_trajectory_csv(csv_path: Path) -> pd.DataFrame:
    if not csv_path.exists():
        raise FileNotFoundError(f"Trajectory CSV not found: {csv_path}")
    df = pd.read_csv(csv_path)
    df.columns = df.columns.str.strip()
    if len(df) < 1:
        raise ValueError(f"Trajectory CSV is empty: {csv_path}")
    return df


def load_path_json(path_json: Path) -> dict[str, Any]:
    if not path_json.exists():
        raise FileNotFoundError(f"Path JSON not found: {path_json}")
    return json.loads(path_json.read_text(encoding="utf-8"))


def goal_from_artifacts(path_data: dict[str, Any], grid_json: Path | None) -> tuple[float, float]:
    path_world = path_data.get("path_world")
    if path_world:
        goal = path_world[-1]
        return float(goal[0]), float(goal[1])

    if grid_json is not None and grid_json.exists():
        grid_data = json.loads(grid_json.read_text(encoding="utf-8"))
        if "goal_world" in grid_data:
            gw = grid_data["goal_world"]
            return float(gw[0]), float(gw[1])

    raise ValueError("Could not determine goal position from path JSON or grid JSON")


def compute_metrics(
    csv_path: Path,
    path_json_path: Path,
    *,
    grid_json_path: Path | None = None,
    world: str | None = None,
) -> dict[str, Any]:
    df = load_trajectory_csv(csv_path)
    path_data = load_path_json(path_json_path)
    goal_x, goal_y = goal_from_artifacts(path_data, grid_json_path)

    fin_rows = df[df["estado"].astype(str) == "FIN"]
    success = len(fin_rows) > 0
    time_to_goal_s = float(fin_rows["tiempo_s"].iloc[0]) if success else None

    executed_points = list(zip(df["x_m"].astype(float), df["y_m"].astype(float)))
    executed_length_m = path_length(executed_points)
    planned_length_m = float(path_data.get("length_m", 0.0))
    length_delta_m = executed_length_m - planned_length_m

    final_row = df.iloc[-1]
    final_x = float(final_row["x_m"])
    final_y = float(final_row["y_m"])
    final_position_error_m = math.hypot(final_x - goal_x, final_y - goal_y)

    return {
        "world": world or path_data.get("world", path_json_path.stem.replace("_path", "")),
        "csv": str(csv_path.resolve()),
        "path_json": str(path_json_path.resolve()),
        "computed_at": datetime.now(timezone.utc).isoformat(),
        "success": success,
        "time_to_goal_s": round(time_to_goal_s, 3) if time_to_goal_s is not None else None,
        "planned_length_m": round(planned_length_m, 4),
        "executed_length_m": round(executed_length_m, 4),
        "length_delta_m": round(length_delta_m, 4),
        "final_position_error_m": round(final_position_error_m, 5),
        "goal_x_m": round(goal_x, 5),
        "goal_y_m": round(goal_y, 5),
        "final_x_m": round(final_x, 5),
        "final_y_m": round(final_y, 5),
    }
