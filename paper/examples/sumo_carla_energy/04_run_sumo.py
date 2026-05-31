"""Run the generated SUMO case when SUMO is installed locally."""

from __future__ import annotations

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pyopendrive.sim.cli import run_sumo  # noqa: E402


WORKFLOW_DIR = Path(__file__).resolve().parent
PROJECT_DIR = WORKFLOW_DIR / "generated" / "tempe_baseline"


def main() -> None:
    """Run SUMO and write the command log under the project output folder."""

    run_sumo(PROJECT_DIR)
    print(f"SUMO outputs written under: {PROJECT_DIR / 'sumo' / 'outputs'}")


if __name__ == "__main__":
    main()
