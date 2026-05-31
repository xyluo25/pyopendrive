"""Analyze generated or sample SUMO outputs for the paper workflow."""

from __future__ import annotations

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pyopendrive.sim import analyze_project  # noqa: E402


WORKFLOW_DIR = Path(__file__).resolve().parent
PROJECT_DIR = WORKFLOW_DIR / "generated" / "tempe_baseline"
SAMPLE_OUTPUT_DIR = WORKFLOW_DIR / "outputs"


def main() -> None:
    """Write summary CSVs without requiring CARLA telemetry."""

    project_tripinfo = PROJECT_DIR / "sumo" / "outputs" / "tripinfo.xml"
    project_telemetry = PROJECT_DIR / "carla" / "outputs" / "carla_telemetry.csv"
    project_exists = PROJECT_DIR.exists()
    tripinfo_path = (
        project_tripinfo
        if project_tripinfo.exists()
        else SAMPLE_OUTPUT_DIR / "tripinfo.xml"
    )
    telemetry_path = project_telemetry if project_telemetry.exists() else None
    if not project_exists and telemetry_path is None:
        telemetry_path = SAMPLE_OUTPUT_DIR / "carla_telemetry.csv"

    outputs = analyze_project(
        project_dir=PROJECT_DIR if project_exists else None,
        tripinfo_path=tripinfo_path,
        carla_telemetry_path=telemetry_path,
        skip_carla=telemetry_path is None,
        scenario_id="baseline",
        out_dir=WORKFLOW_DIR / "analysis",
    )
    for name, path in outputs.items():
        print(f"{name}: {path}")


if __name__ == "__main__":
    main()
