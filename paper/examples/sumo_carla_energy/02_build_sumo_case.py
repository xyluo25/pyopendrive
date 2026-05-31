"""Build the SUMO case from the bundled Tempe paper dataset."""

from __future__ import annotations

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pyopendrive.sim.cli import build_sim_project  # noqa: E402


WORKFLOW_DIR = Path(__file__).resolve().parent
DATASET_DIR = WORKFLOW_DIR.parent / "datasets" / "tempe_net"
PROJECT_DIR = WORKFLOW_DIR / "generated" / "tempe_baseline"
SCENARIO_PATH = WORKFLOW_DIR / "configs" / "scenario_baseline.yaml"


def main() -> None:
    """Create SUMO network, route, vehicle type, and config files."""

    result = build_sim_project(
        xodr_path=DATASET_DIR / "tempe.xodr",
        out_dir=PROJECT_DIR,
        scenario_path=SCENARIO_PATH,
        build_sumo=True,
        dry_run=True,
        overwrite=True,
    )
    print(f"SUMO case built under: {result['project_dir']}")


if __name__ == "__main__":
    main()
