"""Occupancy grid generation with configurable N×N size and cell dimensions."""

from __future__ import annotations

import csv
import json
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from paths import OUTPUT_DIR, REPO_ROOT, SCRIPTS_DIR

PROJECT_ROOT = REPO_ROOT

MIN_GRID_SIZE = 6
ROOM_ORIGIN = 2
ROOM_STEP = 2

START = (10, 19)
GOAL = (10, 0)
DEFAULT_SEED = 42
MIN_DIFFICULTY = 1
MAX_DIFFICULTY = 10
DEFAULT_DIFFICULTY = 5
WORLD_NAME = "MundoFinal1"


@dataclass(frozen=True)
class GridSpec:
    size: int = 20
    cell: float = 0.1

    def __post_init__(self) -> None:
        if self.size < MIN_GRID_SIZE:
            raise ValueError(f"grid size must be >= {MIN_GRID_SIZE}, got {self.size}")
        if self.cell <= 0:
            raise ValueError(f"cell size must be > 0, got {self.cell}")

    @property
    def origin(self) -> float:
        return -(self.size * self.cell) / 2.0

    @property
    def rooms(self) -> int:
        return max(1, (self.size - 2 - ROOM_ORIGIN) // ROOM_STEP + 1)

    @property
    def world_span(self) -> float:
        return self.size * self.cell


DEFAULT_SPEC = GridSpec()
GRID = DEFAULT_SPEC.size
CELL = DEFAULT_SPEC.cell
ORIGIN = DEFAULT_SPEC.origin
ROOMS = DEFAULT_SPEC.rooms


def grid_spec_from_dict(data: dict[str, Any]) -> GridSpec:
    spec = GridSpec(
        size=int(data["grid_size"]),
        cell=float(data["cell_size_m"]),
    )
    stored = data.get("origin_m")
    if stored is not None:
        stored_origin = float(stored[0])
        if abs(stored_origin - spec.origin) > 1e-6:
            print(
                f"[Advertencia] origin_m en JSON ({stored_origin}) difiere del origin "
                f"centrado ({spec.origin}); se usa el valor calculado.",
                file=sys.stderr,
            )
    return spec


def validate_grid_size(size: int) -> int:
    if size < MIN_GRID_SIZE:
        raise ValueError(f"grid size must be >= {MIN_GRID_SIZE}, got {size}")
    return size


def cell_to_world(
    cx: int,
    cy: int,
    spec: GridSpec = DEFAULT_SPEC,
) -> tuple[float, float]:
    x = round(spec.origin + (cx + 0.5) * spec.cell, 3)
    y = round(spec.origin + (cy + 0.5) * spec.cell, 3)
    return x, y


def world_to_cell(
    x: float,
    y: float,
    spec: GridSpec = DEFAULT_SPEC,
) -> tuple[int, int] | None:
    cx = math.floor((x - spec.origin) / spec.cell)
    cy = math.floor((y - spec.origin) / spec.cell)
    if not (0 <= cx < spec.size and 0 <= cy < spec.size):
        return None
    return cx, cy


def _room_cell(ri: int, rj: int) -> tuple[int, int]:
    return ROOM_ORIGIN + ROOM_STEP * ri, ROOM_ORIGIN + ROOM_STEP * rj


def _room_index(cx: int, cy: int, rooms: int) -> tuple[int, int] | None:
    if (cx - ROOM_ORIGIN) % ROOM_STEP != 0 or (cy - ROOM_ORIGIN) % ROOM_STEP != 0:
        return None
    ri = (cx - ROOM_ORIGIN) // ROOM_STEP
    rj = (cy - ROOM_ORIGIN) // ROOM_STEP
    if 0 <= ri < rooms and 0 <= rj < rooms:
        return ri, rj
    return None


def _apply_start_goal(
    grid: list[list[int]],
    start: tuple[int, int],
    goal: tuple[int, int],
    spec: GridSpec,
) -> None:
    sx, sy = start
    gx, gy = goal
    grid[sy][sx] = 0
    grid[gy][gx] = 0

    if sy == spec.size - 1:
        grid[sy - 1][sx] = 0
    elif sy == 0:
        grid[sy + 1][sx] = 0

    if sx == 0:
        grid[sy][sx + 1] = 0
    elif sx == spec.size - 1:
        grid[sy][sx - 1] = 0

    if gy == spec.size - 1:
        grid[gy - 1][gx] = 0
    elif gy == 0:
        grid[gy + 1][gx] = 0

    if gx == 0:
        grid[gy][gx + 1] = 0
    elif gx == spec.size - 1:
        grid[gy][gx - 1] = 0


def _is_border(cx: int, cy: int, spec: GridSpec) -> bool:
    return cx == 0 or cy == 0 or cx == spec.size - 1 or cy == spec.size - 1


def parse_cell_arg(
    value: str,
    name: str,
    spec: GridSpec = DEFAULT_SPEC,
) -> tuple[int, int]:
    parts = value.split(",")
    if len(parts) != 2:
        raise ValueError(f"{name} must be cx,cy (got {value!r})")
    try:
        cx, cy = int(parts[0].strip()), int(parts[1].strip())
    except ValueError as exc:
        raise ValueError(f"{name} must be cx,cy with integers (got {value!r})") from exc
    if not (0 <= cx < spec.size and 0 <= cy < spec.size):
        raise ValueError(f"{name} ({cx},{cy}) is outside grid 0..{spec.size - 1}")
    return cx, cy


def _init_square_grid(spec: GridSpec) -> list[list[int]]:
    grid = [[0 for _ in range(spec.size)] for _ in range(spec.size)]
    for cx in range(spec.size):
        grid[0][cx] = 1
        grid[spec.size - 1][cx] = 1
    for cy in range(spec.size):
        grid[cy][0] = 1
        grid[cy][spec.size - 1] = 1
    return grid


def _apply_perfect_maze(grid: list[list[int]], seed: int, spec: GridSpec) -> None:
    rng = random.Random(seed)
    rooms = spec.rooms

    right = [[True for _ in range(rooms)] for _ in range(rooms - 1)]
    bottom = [[True for _ in range(rooms - 1)] for _ in range(rooms)]

    visited = [[False for _ in range(rooms)] for _ in range(rooms)]
    start_room = _room_index(*_room_cell(rooms // 2, rooms - 1), rooms)
    assert start_room is not None
    stack = [start_room]
    visited[start_room[0]][start_room[1]] = True

    while stack:
        ri, rj = stack[-1]
        neighbors: list[tuple[int, int, str]] = []
        if ri > 0 and not visited[ri - 1][rj]:
            neighbors.append((ri - 1, rj, "left"))
        if ri < rooms - 1 and not visited[ri + 1][rj]:
            neighbors.append((ri + 1, rj, "right"))
        if rj > 0 and not visited[ri][rj - 1]:
            neighbors.append((ri, rj - 1, "top"))
        if rj < rooms - 1 and not visited[ri][rj + 1]:
            neighbors.append((ri, rj + 1, "bottom"))

        if neighbors:
            ni, nj, direction = rng.choice(neighbors)
            if direction == "left":
                right[ri - 1][rj] = False
            elif direction == "right":
                right[ri][rj] = False
            elif direction == "top":
                bottom[ri][rj - 1] = False
            else:
                bottom[ri][rj] = False
            visited[ni][nj] = True
            stack.append((ni, nj))
        else:
            stack.pop()

    for cx in range(spec.size):
        grid[0][cx] = 1
        grid[spec.size - 1][cx] = 1
    for cy in range(spec.size):
        grid[cy][0] = 1
        grid[cy][spec.size - 1] = 1

    for ri in range(rooms):
        for rj in range(rooms):
            cx, cy = _room_cell(ri, rj)
            grid[cy][cx] = 0

    for ri in range(rooms - 1):
        for rj in range(rooms):
            if right[ri][rj]:
                wall_x = ROOM_ORIGIN + ROOM_STEP * ri + 1
                passage_y = ROOM_ORIGIN + ROOM_STEP * rj
                grid[passage_y][wall_x] = 1

    for ri in range(rooms):
        for rj in range(rooms - 1):
            if bottom[ri][rj]:
                wall_y = ROOM_ORIGIN + ROOM_STEP * rj + 1
                passage_x = ROOM_ORIGIN + ROOM_STEP * ri
                grid[wall_y][passage_x] = 1


def _shuffled_cells(rng: random.Random, spec: GridSpec) -> list[tuple[int, int]]:
    cells = [(cx, cy) for cy in range(spec.size) for cx in range(spec.size)]
    rng.shuffle(cells)
    return cells


def _apply_difficulty(
    grid: list[list[int]],
    seed: int,
    start: tuple[int, int],
    goal: tuple[int, int],
    difficulty: int,
    spec: GridSpec,
) -> None:
    if difficulty == DEFAULT_DIFFICULTY:
        return

    rng = random.Random(seed + difficulty * 1000)
    protected = {start, goal}

    if difficulty < DEFAULT_DIFFICULTY:
        p_remove = 1.0 if difficulty == MIN_DIFFICULTY else (DEFAULT_DIFFICULTY - difficulty) / 4.0
        for cx, cy in _shuffled_cells(rng, spec):
            if _is_border(cx, cy, spec) or grid[cy][cx] == 0 or (cx, cy) in protected:
                continue
            if difficulty > MIN_DIFFICULTY and rng.random() >= p_remove:
                continue
            grid[cy][cx] = 0
            if not is_connected(grid, start, goal):
                grid[cy][cx] = 1

    if difficulty > DEFAULT_DIFFICULTY:
        scale = 1.5 if difficulty == MAX_DIFFICULTY else 1.0
        p_add = ((difficulty - DEFAULT_DIFFICULTY) / 5.0) * 0.12 * scale
        for cx, cy in _shuffled_cells(rng, spec):
            if _is_border(cx, cy, spec) or grid[cy][cx] == 1 or (cx, cy) in protected:
                continue
            if rng.random() >= p_add:
                continue
            grid[cy][cx] = 1
            if not is_connected(grid, start, goal):
                grid[cy][cx] = 0


def validate_difficulty(difficulty: int) -> int:
    if not MIN_DIFFICULTY <= difficulty <= MAX_DIFFICULTY:
        raise ValueError(f"difficulty must be between {MIN_DIFFICULTY} and {MAX_DIFFICULTY}, got {difficulty}")
    return difficulty


def build_occupancy_grid(
    seed: int = DEFAULT_SEED,
    start: tuple[int, int] = START,
    goal: tuple[int, int] = GOAL,
    difficulty: int = DEFAULT_DIFFICULTY,
    spec: GridSpec = DEFAULT_SPEC,
) -> list[list[int]]:
    validate_difficulty(difficulty)
    grid = _init_square_grid(spec)
    _apply_start_goal(grid, start, goal, spec)
    _apply_perfect_maze(grid, seed, spec)
    _apply_difficulty(grid, seed, start, goal, difficulty, spec)
    _apply_start_goal(grid, start, goal, spec)
    return grid


def grid_size_from_grid(grid: list[list[int]]) -> int:
    height = len(grid)
    if height == 0:
        raise ValueError("occupancy grid is empty")
    width = len(grid[0])
    if any(len(row) != width for row in grid):
        raise ValueError("occupancy grid rows have inconsistent widths")
    if height != width:
        raise ValueError(f"expected square grid, got {width}x{height}")
    return height


def is_connected(
    grid: list[list[int]],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> bool:
    size = grid_size_from_grid(grid)
    seen = {start}
    stack = [start]
    while stack:
        cx, cy = stack.pop()
        if (cx, cy) == goal:
            return True
        for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < size and 0 <= ny < size and grid[ny][nx] == 0 and (nx, ny) not in seen:
                seen.add((nx, ny))
                stack.append((nx, ny))
    return False


def count_free_cells(grid: list[list[int]]) -> int:
    return sum(cell == 0 for row in grid for cell in row)


def count_wall_cells(grid: list[list[int]]) -> int:
    return sum(cell == 1 for row in grid for cell in row)


def grid_to_dict(
    grid: list[list[int]],
    *,
    world: str,
    seed: int = DEFAULT_SEED,
    start: tuple[int, int] = START,
    goal: tuple[int, int] = GOAL,
    start_world: tuple[float, float] | None = None,
    goal_world: tuple[float, float] | None = None,
    difficulty: int = DEFAULT_DIFFICULTY,
    spec: GridSpec = DEFAULT_SPEC,
) -> dict[str, Any]:
    if start_world is None:
        start_world = cell_to_world(start[0], start[1], spec)
    if goal_world is None:
        goal_world = cell_to_world(goal[0], goal[1], spec)

    return {
        "world": world,
        "seed": seed,
        "difficulty": difficulty,
        "grid_size": spec.size,
        "cell_size_m": spec.cell,
        "origin_m": [spec.origin, spec.origin],
        "start_cell": [start[0], start[1]],
        "goal_cell": [goal[0], goal[1]],
        "start_world": [start_world[0], start_world[1]],
        "goal_world": [goal_world[0], goal_world[1]],
        "occupancy": grid,
    }


def save_grid_json(
    grid: list[list[int]],
    path: str | Path,
    *,
    world: str,
    seed: int = DEFAULT_SEED,
    start: tuple[int, int] = START,
    goal: tuple[int, int] = GOAL,
    start_world: tuple[float, float] | None = None,
    goal_world: tuple[float, float] | None = None,
    difficulty: int = DEFAULT_DIFFICULTY,
    spec: GridSpec = DEFAULT_SPEC,
) -> None:
    payload = grid_to_dict(
        grid,
        world=world,
        seed=seed,
        start=start,
        goal=goal,
        start_world=start_world,
        goal_world=goal_world,
        difficulty=difficulty,
        spec=spec,
    )
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")


def save_grid_csv(grid: list[list[int]], path: str | Path) -> None:
    with Path(path).open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        for row in grid:
            writer.writerow(row)


def load_grid_json(path: str | Path) -> dict[str, Any]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    occupancy = data["occupancy"]
    expected_size = int(data["grid_size"])
    if len(occupancy) != expected_size or any(len(row) != expected_size for row in occupancy):
        raise ValueError(f"Expected occupancy grid of size {expected_size}x{expected_size}")
    return data
