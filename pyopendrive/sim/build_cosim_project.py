"""Backward-compatible import wrapper for SUMO-CARLA co-simulation scripts."""

from __future__ import annotations

from .cosim_builder import build_cosim_project

__all__ = ["build_cosim_project"]
