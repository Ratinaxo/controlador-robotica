#!/usr/bin/env python3
"""Tests for A* path simplification to straight segment endpoints."""

from __future__ import annotations

from astar import path_length_m, simplify_path


def test_straight_line_collapses_to_endpoints() -> None:
    path = [(5, 0), (6, 0), (7, 0), (8, 0)]
    assert simplify_path(path) == [(5, 0), (8, 0)]


def test_l_shape_keeps_corner() -> None:
    path = [(5, 0), (6, 0), (7, 0), (7, 1)]
    assert simplify_path(path) == [(5, 0), (7, 0), (7, 1)]


def test_single_cell_path() -> None:
    path = [(3, 3)]
    assert simplify_path(path) == [(3, 3)]


def test_length_invariant_after_simplify() -> None:
    path = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (3, 2)]
    segments = simplify_path(path)
    assert path_length_m(path, 0.08) == path_length_m(segments, 0.08)


if __name__ == "__main__":
    test_straight_line_collapses_to_endpoints()
    test_l_shape_keeps_corner()
    test_single_cell_path()
    test_length_invariant_after_simplify()
    print("All simplify_path tests passed.")
