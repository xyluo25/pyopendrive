"""Load the CARLA OpenDRIVE world when CARLA is running locally."""

from __future__ import annotations

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pyopendrive.sim.cli import run_carla  # noqa: E402


WORKFLOW_DIR = Path(__file__).resolve().parent
PROJECT_DIR = WORKFLOW_DIR / "generated" / "tempe_baseline"


def main() -> None:
    """Run the generated CARLA OpenDRIVE loader."""

    run_carla(PROJECT_DIR)
    print(f"CARLA scripts used from: {PROJECT_DIR / 'carla'}")


if __name__ == "__main__":
    main()
