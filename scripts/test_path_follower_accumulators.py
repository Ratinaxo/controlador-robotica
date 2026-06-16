#!/usr/bin/env python3
"""Unit checks for PathFollower deterministic segment/turn accumulators."""

from __future__ import annotations

import math


SEGMENT_TOL = 0.005
TURN_TOL = math.radians(1)


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def segment_axis(p0: tuple[float, float], p1: tuple[float, float]) -> str:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    return "x" if abs(dx) >= abs(dy) else "y"


def segment_heading(p0: tuple[float, float], p1: tuple[float, float]) -> float:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    if abs(dx) >= abs(dy):
        return 0.0 if dx > 0 else math.pi
    return math.pi / 2 if dy > 0 else -math.pi / 2


def axis_distance(pose: tuple[float, float], target: tuple[float, float], axis: str) -> float:
    if axis == "x":
        return target[0] - pose[0]
    return target[1] - pose[1]


class PathFollower:
    def __init__(self, waypoints: list[tuple[float, float]], planned_length_m: float) -> None:
        self.waypoints = waypoints
        self.planned_length_m = planned_length_m
        self.index = 1
        self.state = "GIRAR"
        self.segment_axis = "x"
        self.target_heading = 0.0
        self.segment_driven_m = 0.0
        self.segment_length_m = 0.0
        self.turn_accumulated_rad = 0.0
        self.required_turn_rad = 0.0
        self.prepare_segment(waypoints[0], waypoints[1])

    @property
    def current_waypoint(self) -> tuple[float, float]:
        return self.waypoints[self.index]

    def prepare_segment(self, p0: tuple[float, float], p1: tuple[float, float]) -> None:
        self.segment_axis = segment_axis(p0, p1)
        self.target_heading = segment_heading(p0, p1)

    def begin_advance_segment(self, start: tuple[float, float], target: tuple[float, float], axis: str) -> None:
        self.segment_driven_m = 0.0
        self.segment_length_m = abs(axis_distance(start, target, axis))

    def begin_turn(self, target_heading: float, current_phi: float) -> None:
        self.turn_accumulated_rad = 0.0
        self.required_turn_rad = normalize_angle(target_heading - current_phi)

    def update_segment(self, delta_s: float, phi: float, axis: str) -> None:
        if axis == "x":
            self.segment_driven_m += abs(delta_s * math.cos(phi))
        else:
            self.segment_driven_m += abs(delta_s * math.sin(phi))

    def update_turn(self, delta_phi: float) -> None:
        self.turn_accumulated_rad += delta_phi

    def advance_complete(self) -> bool:
        return self.segment_driven_m >= self.segment_length_m - SEGMENT_TOL

    def turn_complete(self) -> bool:
        if abs(self.required_turn_rad) < TURN_TOL:
            return True
        return abs(self.turn_accumulated_rad) >= abs(self.required_turn_rad) - TURN_TOL


def test_segment_accumulator() -> None:
    waypoints = [(0.85, 0.85), (0.75, 0.85), (0.75, 0.75)]
    follower = PathFollower(waypoints, planned_length_m=0.2)
    follower.begin_advance_segment((0.85, 0.85), (0.75, 0.85), "x")

    assert abs(follower.segment_length_m - 0.1) < 1e-9
    assert not follower.advance_complete()

    steps = 10
    delta = 0.1 / steps
    for _ in range(steps - 1):
        follower.update_segment(delta, math.pi, "x")
        assert not follower.advance_complete()

    follower.update_segment(delta, math.pi, "x")
    assert follower.advance_complete()
    assert follower.segment_driven_m >= 0.1 - SEGMENT_TOL


def test_turn_accumulator() -> None:
    waypoints = [(0.85, 0.85), (0.75, 0.85), (0.75, 0.75)]
    follower = PathFollower(waypoints, planned_length_m=0.2)
    follower.begin_turn(-math.pi / 2, math.pi)

    assert abs(abs(follower.required_turn_rad) - math.pi / 2) < 1e-9
    assert not follower.turn_complete()

    steps = 1000
    delta = (math.pi / 2) / steps
    for _ in range(steps // 2):
        follower.update_turn(delta)
    assert not follower.turn_complete()

    for _ in range(steps // 2):
        follower.update_turn(delta)

    follower.update_turn(delta)
    assert follower.turn_complete()
    assert abs(follower.turn_accumulated_rad) >= math.pi / 2 - TURN_TOL


def test_zero_turn() -> None:
    waypoints = [(0.85, 0.85), (0.75, 0.85)]
    follower = PathFollower(waypoints, planned_length_m=0.1)
    follower.begin_turn(math.pi, math.pi)
    assert follower.turn_complete()


def main() -> int:
    test_segment_accumulator()
    test_turn_accumulator()
    test_zero_turn()
    print("PathFollower accumulator checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
