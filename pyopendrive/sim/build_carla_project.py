"""Backward-compatible import wrapper for CARLA project generation."""

from __future__ import annotations

from .carla_builder import build_carla_project

__all__ = ["build_carla_project"]
