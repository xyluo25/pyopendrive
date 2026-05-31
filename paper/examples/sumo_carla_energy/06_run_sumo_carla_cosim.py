"""Build and run the SUMO-CARLA co-simulation helper scripts."""

from __future__ import annotations

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pyopendrive.sim.cli import build_sim_project, run_cosim  # noqa: E402


WORKFLOW_DIR = Path(__file__).resolve().parent
DATASET_DIR = WORKFLOW_DIR.parent / "datasets" / "tempe_net"
PROJECT_DIR = WORKFLOW_DIR / "generated" / "tempe_baseline"
SCENARIO_PATH = WORKFLOW_DIR / "configs" / "scenario_baseline.yaml"


def main() -> None:
    """Create co-simulation scripts and run synchronization if tools exist."""

    build_sim_project(
        xodr_path=DATASET_DIR / "tempe.xodr",
        out_dir=PROJECT_DIR,
        scenario_path=SCENARIO_PATH,
        build_cosim=True,
        dry_run=True,
        overwrite=True,
    )
    run_cosim(PROJECT_DIR)


if __name__ == "__main__":
    main()
