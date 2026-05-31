"""Scenario comparison helpers for repeated SUMO/CARLA runs."""

from __future__ import annotations

from pathlib import Path

from .analysis import summarize_replicates


def compare_scenarios(
    summary_files: list[str | Path] | None = None,
    out_dir: str | Path | None = None,
) -> dict[str, Path] | None:
    """Combine scenario summary CSV files and write comparison tables.

    The no-argument call is retained as a no-op for compatibility with the
    original placeholder API.
    """

    if summary_files is None or out_dir is None:
        return None
    return summarize_replicates([Path(path) for path in summary_files], out_dir)


__all__ = ["compare_scenarios"]
