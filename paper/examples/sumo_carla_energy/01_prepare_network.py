"""Prepare the bundled Tempe OpenDRIVE network for the paper workflow."""

from __future__ import annotations

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pyopendrive.sim import prepare_opendrive_network  # noqa: E402


WORKFLOW_DIR = Path(__file__).resolve().parent
DATASET_DIR = WORKFLOW_DIR.parent / "datasets" / "tempe_net"
PROJECT_DIR = WORKFLOW_DIR / "generated" / "tempe_baseline"
XODR_PATH = DATASET_DIR / "tempe.xodr"


def main() -> None:
    """Copy the paper dataset into a reproducible project directory."""

    prepared_xodr = prepare_opendrive_network(XODR_PATH, PROJECT_DIR, overwrite=True)
    print(f"Prepared OpenDRIVE network: {prepared_xodr}")


if __name__ == "__main__":
    main()
