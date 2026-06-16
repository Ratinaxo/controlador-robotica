"""A* pathfinding on MundoFinal1 occupancy grid."""

from __future__ import annotations

import heapq
from typing import Any

from occupancy_grid import CELL, GRID, cell_to_world


def _manhattan(a: tuple[int, int], b: tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _is_free(grid: list[list[int]], cx: int, cy: int) -> bool:
    return 0 <= cx < GRID and 0 <= cy < GRID and grid[cy][cx] == 0


def astar(
    grid: list[list[int]],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]] | None:
    if not _is_free(grid, start[0], start[1]):
        return None
    if not _is_free(grid, goal[0], goal[1]):
        return None
    if start == goal:
        return [start]

    open_heap: list[tuple[int, int, tuple[int, int]]] = []
    counter = 0
    heapq.heappush(open_heap, (_manhattan(start, goal), counter, start))
    counter += 1

    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    g_score: dict[tuple[int, int], int] = {start: 0}

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            path: list[tuple[int, int]] = []
            node: tuple[int, int] | None = current
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return path

        cx, cy = current
        for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nx, ny = cx + dx, cy + dy
            if not _is_free(grid, nx, ny):
                continue
            neighbor = (nx, ny)
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + _manhattan(neighbor, goal)
                heapq.heappush(open_heap, (f_score, counter, neighbor))
                counter += 1

    return None


def path_to_world(path_cells: list[tuple[int, int]]) -> list[tuple[float, float]]:
    return [cell_to_world(cx, cy) for cx, cy in path_cells]


def path_length_m(path_cells: list[tuple[int, int]], cell_size: float = CELL) -> float:
    if len(path_cells) <= 1:
        return 0.0
    return round((len(path_cells) - 1) * cell_size, 3)


def path_to_dict(
    path_cells: list[tuple[int, int]],
    *,
    world: str,
    source_grid: str,
    start: tuple[int, int],
    goal: tuple[int, int],
    cell_size: float = CELL,
) -> dict[str, Any]:
    path_world = path_to_world(path_cells)
    return {
        "world": world,
        "source_grid": source_grid,
        "start_cell": [start[0], start[1]],
        "goal_cell": [goal[0], goal[1]],
        "path_cells": [[cx, cy] for cx, cy in path_cells],
        "path_world": [[x, y] for x, y in path_world],
        "length_cells": len(path_cells),
        "length_m": path_length_m(path_cells, cell_size),
    }
