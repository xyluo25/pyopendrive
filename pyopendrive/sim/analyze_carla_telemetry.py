"""Backward-compatible wrappers for CARLA telemetry analysis."""

from __future__ import annotations

from pathlib import Path

from .analysis import read_carla_telemetry, summarize_carla_telemetry


def analyze_carla_telemetry(
    telemetry_path: str | Path,
    *,
    scenario_id: str = "scenario",
) -> dict[str, object]:
    """Read and summarize a CARLA telemetry CSV file."""

    return summarize_carla_telemetry(read_carla_telemetry(telemetry_path), scenario_id)


__all__ = [
    "analyze_carla_telemetry",
    "read_carla_telemetry",
    "summarize_carla_telemetry",
]
