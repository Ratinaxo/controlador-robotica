"""Shared matplotlib plot styling helpers."""

from __future__ import annotations

import argparse

LEGEND_SIDES = ("left", "right")


def legend_loc(side: str) -> str:
    return "upper left" if side == "left" else "upper right"


def add_legend_side_arg(parser: argparse.ArgumentParser, *, default: str) -> None:
    parser.add_argument(
        "--legend",
        choices=LEGEND_SIDES,
        default=default,
        help="Legend box side (default: %(default)s)",
    )
