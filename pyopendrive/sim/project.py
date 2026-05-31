"""Project directory preparation for SUMO and CARLA simulation studies."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from .config import ScenarioConfig, scenario_to_dict


@dataclass(frozen=True)
class SimulationProjectPaths:
    """Canonical paths created inside a simulation project directory."""

    root: Path
    network_dir: Path
    sumo_dir: Path
    carla_dir: Path
    cosim_dir: Path
    analysis_dir: Path
    input_xodr: Path
    prepared_xodr: Path
    network_summary: Path
    project_config: Path


def make_project_paths(project_dir: str | Path) -> SimulationProjectPaths:
    """Return the standard file and folder layout for a simulation project."""

    root = Path(project_dir)
    network_dir = root / "network"
    return SimulationProjectPaths(
        root=root,
        network_dir=network_dir,
        sumo_dir=root / "sumo",
        carla_dir=root / "carla",
        cosim_dir=root / "cosim",
        analysis_dir=root / "analysis",
        input_xodr=network_dir / "input.xodr",
        prepared_xodr=network_dir / "prepared.xodr",
        network_summary=network_dir / "network_summary.json",
        project_config=root / "project_config.json",
    )


def create_project_directories(
    project_dir: str | Path,
    *,
    overwrite: bool = False,
) -> SimulationProjectPaths:
    """Create the standard project folders.

    Existing files are not deleted. When ``overwrite`` is false, a non-empty
    project directory raises an error so users do not accidentally mix studies.
    """

    paths = make_project_paths(project_dir)
    if paths.root.exists() and any(paths.root.iterdir()) and not overwrite:
        raise FileExistsError(
            f"Project directory already exists and is not empty: {paths.root}. "
            "Pass overwrite=True or choose a new output folder."
        )

    for directory in [
        paths.network_dir,
        paths.sumo_dir,
        paths.carla_dir,
        paths.cosim_dir,
        paths.analysis_dir,
        paths.sumo_dir / "outputs",
        paths.carla_dir / "outputs",
        paths.cosim_dir / "outputs",
        paths.analysis_dir / "figures",
    ]:
        directory.mkdir(parents=True, exist_ok=True)

    return paths


def prepare_opendrive_network(
    xodr_path: str | Path,
    project_dir: str | Path,
    *,
    overwrite: bool = True,
) -> Path:
    """Copy an OpenDRIVE network into a project and write a summary JSON file.

    Args:
        xodr_path: Source ``.xodr`` file.
        project_dir: Simulation project directory.
        overwrite: Whether existing ``input.xodr`` and ``prepared.xodr`` files
            may be replaced.

    Returns:
        Path to the project-ready ``prepared.xodr`` file.
    """

    source_path = Path(xodr_path)
    if not source_path.exists():
        raise FileNotFoundError(f"OpenDRIVE file not found: {source_path}")

    paths = create_project_directories(project_dir, overwrite=True)
    for target_path in [paths.input_xodr, paths.prepared_xodr]:
        if target_path.exists() and not overwrite:
            raise FileExistsError(
                f"OpenDRIVE project file already exists: {target_path}"
            )
        shutil.copy2(source_path, target_path)

    summary = summarize_opendrive_xml(paths.prepared_xodr)
    summary["source_xodr"] = str(source_path.resolve())
    summary["input_xodr"] = str(paths.input_xodr)
    summary["prepared_xodr"] = str(paths.prepared_xodr)
    paths.network_summary.write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )
    return paths.prepared_xodr


def summarize_opendrive_xml(xodr_path: str | Path) -> dict[str, object]:
    """Summarize core OpenDRIVE elements using streaming XML parsing."""

    path = Path(xodr_path)
    counts = {
        "roads": 0,
        "junctions": 0,
        "lane_sections": 0,
        "lanes": 0,
        "signals": 0,
        "objects": 0,
        "controllers": 0,
    }
    header_attrs: dict[str, str] = {}
    road_ids: list[str] = []
    junction_ids: list[str] = []

    for event, elem in ET.iterparse(path, events=("start", "end")):
        if event == "start" and elem.tag == "header":
            header_attrs = dict(elem.attrib)
        if event != "end":
            continue

        if elem.tag == "road":
            counts["roads"] += 1
            road_id = elem.get("id")
            if road_id is not None and len(road_ids) < 20:
                road_ids.append(road_id)
        elif elem.tag == "junction":
            counts["junctions"] += 1
            junction_id = elem.get("id")
            if junction_id is not None and len(junction_ids) < 20:
                junction_ids.append(junction_id)
        elif elem.tag == "laneSection":
            counts["lane_sections"] += 1
        elif elem.tag == "lane":
            counts["lanes"] += 1
        elif elem.tag == "signal":
            counts["signals"] += 1
        elif elem.tag == "object":
            counts["objects"] += 1
        elif elem.tag == "controller":
            counts["controllers"] += 1
        elem.clear()

    return {
        "file": str(path),
        "header": header_attrs,
        "counts": counts,
        "sample_road_ids": road_ids,
        "sample_junction_ids": junction_ids,
    }


def write_project_config(
    project_dir: str | Path,
    scenario_config: ScenarioConfig,
    *,
    xodr_path: str | Path,
    build_sumo: bool,
    build_carla: bool,
    build_cosim: bool,
    dry_run: bool,
) -> Path:
    """Write project metadata needed to reproduce a simulation build."""

    paths = make_project_paths(project_dir)
    payload = {
        "xodr_path": str(Path(xodr_path)),
        "scenario": scenario_to_dict(scenario_config),
        "build_sumo": build_sumo,
        "build_carla": build_carla,
        "build_cosim": build_cosim,
        "dry_run": dry_run,
    }
    paths.project_config.write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )
    return paths.project_config
