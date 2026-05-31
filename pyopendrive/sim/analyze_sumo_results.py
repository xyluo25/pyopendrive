"""Backward-compatible wrappers for SUMO result analysis."""

from __future__ import annotations

from pathlib import Path

from .analysis import read_tripinfo, summarize_tripinfo


def analyze_sumo_results(
    tripinfo_path: str | Path,
    *,
    scenario_id: str = "scenario",
) -> dict[str, object]:
    """Read and summarize a SUMO ``tripinfo.xml`` file."""

    return summarize_tripinfo(read_tripinfo(tripinfo_path), scenario_id)


__all__ = ["analyze_sumo_results", "read_tripinfo", "summarize_tripinfo"]
