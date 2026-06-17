"""Shared path constants for the MundoFinal pipeline."""

from __future__ import annotations

from pathlib import Path
from typing import TypedDict

SCRIPTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPTS_DIR.parent
OUTPUT_DIR = SCRIPTS_DIR / "output"
DATA_DIR = SCRIPTS_DIR / "data_sensores"

CONTROLLER_TRAJECTORY_CSV = REPO_ROOT / "data_sensores" / "trayectoria_ejecutada.csv"
TRAJECTORY_COMPARE_PNG = DATA_DIR / "trayectoria_compare.png"
SENSORS_CSV = DATA_DIR / "datos_sensores.csv"
SENSORS_REPORT_PNG = DATA_DIR / "reporte_final_sensores.png"
SCENARIOS_COMPARISON_PNG = OUTPUT_DIR / "scenarios_comparison.png"
DATA_SENSORS_DIR = REPO_ROOT / "data_sensores"


def trajectory_csv_for(name: str) -> Path:
    return DATA_SENSORS_DIR / f"{name}_trayectoria.csv"


def metrics_json_for(name: str) -> Path:
    return OUTPUT_DIR / f"{name}_metrics.json"


def metrics_png_for(name: str) -> Path:
    return OUTPUT_DIR / f"{name}_metrics.png"


class ArtifactPaths(TypedDict):
    world: str
    grid_json: Path
    grid_csv: Path
    grid_png: Path
    path_json: Path
    path_png: Path
    generated_wbt: Path
    trajectory_csv: Path


def grid_json_for(name: str) -> Path:
    return OUTPUT_DIR / f"{name}_grid.json"


def grid_csv_for(name: str) -> Path:
    return OUTPUT_DIR / f"{name}_grid.csv"


def grid_png_for(name: str) -> Path:
    return OUTPUT_DIR / f"{name}_grid.png"


def path_json_for(name: str) -> Path:
    return OUTPUT_DIR / f"{name}_path.json"


def path_png_for(name: str) -> Path:
    return OUTPUT_DIR / f"{name}_path.png"


def generated_wbt_for(name: str) -> Path:
    return OUTPUT_DIR / f"{name}_generated.wbt"


def artifact_paths(name: str) -> ArtifactPaths:
    return {
        "world": name,
        "grid_json": grid_json_for(name),
        "grid_csv": grid_csv_for(name),
        "grid_png": grid_png_for(name),
        "path_json": path_json_for(name),
        "path_png": path_png_for(name),
        "generated_wbt": generated_wbt_for(name),
        "trajectory_csv": trajectory_csv_for(name),
    }


def ensure_output_dir() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
