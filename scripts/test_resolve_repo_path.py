#!/usr/bin/env python3
"""Tests for controller path resolution (legacy ../../ and repo-relative)."""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
CONTROLLER_DIR = REPO_ROOT / "controllers" / "controlador_Proyectofinal"


def resolve_repo_path(path: Path) -> Path:
    if path.is_absolute():
        return path.resolve()
    if path.parts and path.parts[0] == "..":
        return (CONTROLLER_DIR / path).resolve()
    return (REPO_ROOT / path).resolve()


def test_legacy_controller_relative_path() -> None:
    resolved = resolve_repo_path(Path("../../scripts/output/Facil_path.json"))
    expected = REPO_ROOT / "scripts" / "output" / "Facil_path.json"
    assert resolved == expected.resolve()
    assert resolved.exists(), f"Expected file at {resolved}"


def test_repo_relative_path() -> None:
    resolved = resolve_repo_path(Path("scripts/output/Facil_path.json"))
    expected = REPO_ROOT / "scripts" / "output" / "Facil_path.json"
    assert resolved == expected.resolve()


def test_repo_relative_csv() -> None:
    resolved = resolve_repo_path(Path("data_sensores/trayectoria_ejecutada.csv"))
    expected = REPO_ROOT / "data_sensores" / "trayectoria_ejecutada.csv"
    assert resolved == expected.resolve()


def test_legacy_csv_path() -> None:
    resolved = resolve_repo_path(Path("../../data_sensores/trayectoria_ejecutada.csv"))
    expected = REPO_ROOT / "data_sensores" / "trayectoria_ejecutada.csv"
    assert resolved == expected.resolve()


if __name__ == "__main__":
    test_legacy_controller_relative_path()
    test_repo_relative_path()
    test_repo_relative_csv()
    test_legacy_csv_path()
    print("All resolve_repo_path tests passed.")
