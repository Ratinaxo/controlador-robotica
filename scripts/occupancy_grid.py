"""Occupancy grid for MundoFinal1 (20x20, cell size 0.1 m)."""

from __future__ import annotations

import csv
import json
import math
import random
from pathlib import Path
from typing import Any

from paths import OUTPUT_DIR, REPO_ROOT, SCRIPTS_DIR

PROJECT_ROOT = REPO_ROOT

GRID = 20
CELL = 0.1
ORIGIN = -1.0
START = (10, 19)
GOAL = (10, 0)
DEFAULT_SEED = 42
MIN_DIFFICULTY = 1
MAX_DIFFICULTY = 10
DEFAULT_DIFFICULTY = 5
WORLD_NAME = "MundoFinal1"

ROOMS = 9
ROOM_ORIGIN = 2
ROOM_STEP = 2


def cell_to_world(cx: int, cy: int) -> tuple[float, float]:
    x = round(ORIGIN + (cx + 0.5) * CELL, 3)
    y = round(ORIGIN + (cy + 0.5) * CELL, 3)
    return x, y


def world_to_cell(x: float, y: float) -> tuple[int, int] | None:
    cx = math.floor((x - ORIGIN) / CELL)
    cy = math.floor((y - ORIGIN) / CELL)
    if not (0 <= cx < GRID and 0 <= cy < GRID):
        return None
    return cx, cy


def _room_cell(ri: int, rj: int) -> tuple[int, int]:
    return ROOM_ORIGIN + ROOM_STEP * ri, ROOM_ORIGIN + ROOM_STEP * rj


def _room_index(cx: int, cy: int) -> tuple[int, int] | None:
    if (cx - ROOM_ORIGIN) % ROOM_STEP != 0 or (cy - ROOM_ORIGIN) % ROOM_STEP != 0:
        return None
    ri = (cx - ROOM_ORIGIN) // ROOM_STEP
    rj = (cy - ROOM_ORIGIN) // ROOM_STEP
    if 0 <= ri < ROOMS and 0 <= rj < ROOMS:
        return ri, rj
    return None


def _apply_start_goal(
    grid: list[list[int]],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> None:
    sx, sy = start
    gx, gy = goal
    grid[sy][sx] = 0
    grid[gy][gx] = 0

    if sy == GRID - 1:
        grid[sy - 1][sx] = 0
    elif sy == 0:
        grid[sy + 1][sx] = 0

    if sx == 0:
        grid[sy][sx + 1] = 0
    elif sx == GRID - 1:
        grid[sy][sx - 1] = 0

    if gy == GRID - 1:
        grid[gy - 1][gx] = 0
    elif gy == 0:
        grid[gy + 1][gx] = 0

    if gx == 0:
        grid[gy][gx + 1] = 0
    elif gx == GRID - 1:
        grid[gy][gx - 1] = 0


def _is_border(cx: int, cy: int) -> bool:
    return cx == 0 or cy == 0 or cx == GRID - 1 or cy == GRID - 1


def parse_cell_arg(value: str, name: str) -> tuple[int, int]:
    parts = value.split(",")
    if len(parts) != 2:
        raise ValueError(f"{name} must be cx,cy (got {value!r})")
    try:
        cx, cy = int(parts[0].strip()), int(parts[1].strip())
    except ValueError as exc:
        raise ValueError(f"{name} must be cx,cy with integers (got {value!r})") from exc
    if not (0 <= cx < GRID and 0 <= cy < GRID):
        raise ValueError(f"{name} ({cx},{cy}) is outside grid 0..{GRID - 1}")
    return cx, cy


def _init_square_grid() -> list[list[int]]:
    grid = [[0 for _ in range(GRID)] for _ in range(GRID)]
    for cx in range(GRID):
        grid[0][cx] = 1
        grid[GRID - 1][cx] = 1
    for cy in range(GRID):
        grid[cy][0] = 1
        grid[cy][GRID - 1] = 1
    return grid


def _apply_perfect_maze(grid: list[list[int]], seed: int) -> None:
    rng = random.Random(seed)

    right = [[True for _ in range(ROOMS)] for _ in range(ROOMS - 1)]
    bottom = [[True for _ in range(ROOMS - 1)] for _ in range(ROOMS)]

    visited = [[False for _ in range(ROOMS)] for _ in range(ROOMS)]
    start_room = _room_index(*_room_cell(ROOMS // 2, ROOMS - 1))
    assert start_room is not None
    stack = [start_room]
    visited[start_room[0]][start_room[1]] = True

    while stack:
        ri, rj = stack[-1]
        neighbors: list[tuple[int, int, str]] = []
        if ri > 0 and not visited[ri - 1][rj]:
            neighbors.append((ri - 1, rj, "left"))
        if ri < ROOMS - 1 and not visited[ri + 1][rj]:
            neighbors.append((ri + 1, rj, "right"))
        if rj > 0 and not visited[ri][rj - 1]:
            neighbors.append((ri, rj - 1, "top"))
        if rj < ROOMS - 1 and not visited[ri][rj + 1]:
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

    for cx in range(GRID):
        grid[0][cx] = 1
        grid[GRID - 1][cx] = 1
    for cy in range(GRID):
        grid[cy][0] = 1
        grid[cy][GRID - 1] = 1

    for ri in range(ROOMS):
        for rj in range(ROOMS):
            cx, cy = _room_cell(ri, rj)
            grid[cy][cx] = 0

    for ri in range(ROOMS - 1):
        for rj in range(ROOMS):
            if right[ri][rj]:
                wall_x = ROOM_ORIGIN + ROOM_STEP * ri + 1
                passage_y = ROOM_ORIGIN + ROOM_STEP * rj
                grid[passage_y][wall_x] = 1

    for ri in range(ROOMS):
        for rj in range(ROOMS - 1):
            if bottom[ri][rj]:
                wall_y = ROOM_ORIGIN + ROOM_STEP * rj + 1
                passage_x = ROOM_ORIGIN + ROOM_STEP * ri
                grid[wall_y][passage_x] = 1


def _shuffled_cells(rng: random.Random) -> list[tuple[int, int]]:
    cells = [(cx, cy) for cy in range(GRID) for cx in range(GRID)]
    rng.shuffle(cells)
    return cells


def _apply_difficulty(
    grid: list[list[int]],
    seed: int,
    start: tuple[int, int],
    goal: tuple[int, int],
    difficulty: int,
) -> None:
    if difficulty == DEFAULT_DIFFICULTY:
        return

    rng = random.Random(seed + difficulty * 1000)
    protected = {start, goal}

    if difficulty < DEFAULT_DIFFICULTY:
        p_remove = 1.0 if difficulty == MIN_DIFFICULTY else (DEFAULT_DIFFICULTY - difficulty) / 4.0
        for cx, cy in _shuffled_cells(rng):
            if _is_border(cx, cy) or grid[cy][cx] == 0 or (cx, cy) in protected:
                continue
            if difficulty > MIN_DIFFICULTY and rng.random() >= p_remove:
                continue
            grid[cy][cx] = 0
            if not is_connected(grid, start, goal):
                grid[cy][cx] = 1

    if difficulty > DEFAULT_DIFFICULTY:
        scale = 1.5 if difficulty == MAX_DIFFICULTY else 1.0
        p_add = ((difficulty - DEFAULT_DIFFICULTY) / 5.0) * 0.12 * scale
        for cx, cy in _shuffled_cells(rng):
            if _is_border(cx, cy) or grid[cy][cx] == 1 or (cx, cy) in protected:
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
) -> list[list[int]]:
    validate_difficulty(difficulty)
    grid = _init_square_grid()
    _apply_start_goal(grid, start, goal)
    _apply_perfect_maze(grid, seed)
    _apply_difficulty(grid, seed, start, goal, difficulty)
    _apply_start_goal(grid, start, goal)
    return grid


def is_connected(
    grid: list[list[int]],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> bool:
    seen = {start}
    stack = [start]
    while stack:
        cx, cy = stack.pop()
        if (cx, cy) == goal:
            return True
        for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < GRID and 0 <= ny < GRID and grid[ny][nx] == 0 and (nx, ny) not in seen:
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
) -> dict[str, Any]:
    if start_world is None:
        start_world = cell_to_world(start[0], start[1])
    if goal_world is None:
        goal_world = cell_to_world(goal[0], goal[1])

    return {
        "world": world,
        "seed": seed,
        "difficulty": difficulty,
        "grid_size": GRID,
        "cell_size_m": CELL,
        "origin_m": [ORIGIN, ORIGIN],
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
    if len(occupancy) != GRID or any(len(row) != GRID for row in occupancy):
        raise ValueError(f"Expected occupancy grid of size {GRID}x{GRID}")
    return data
