"""SUMO project generation utilities for OpenDRIVE-based studies."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import random
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

from .config import ScenarioConfig, resolve_config_path
from .project import make_project_paths

EGO_ADD_MODES = {"add", "added", "user_added", "user-added"}
EGO_SELECT_EXISTING_MODES = {
    "select_existing",
    "select-existing",
    "existing",
    "selected",
}


@dataclass
class ExistingSumoInputs:
    """Existing SUMO inputs discovered beside the source OpenDRIVE file."""

    net_file: Path | None = None
    route_files: list[Path] | None = None
    additional_files: list[Path] | None = None
    trip_files: list[Path] | None = None
    support_files: list[Path] | None = None
    sumocfg_file: Path | None = None
    begin: float | None = None
    end: float | None = None
    step_length: float | None = None


def find_executable(name: str) -> str | None:
    """Return an executable path from ``PATH`` when it is available."""

    return shutil.which(name)


def find_sumo_tool(tool_name: str) -> Path | None:
    """Find a SUMO executable or Python helper using ``SUMO_HOME`` and ``PATH``.

    SUMO installs command-line binaries under ``$SUMO_HOME/bin`` and helper
    scripts such as ``randomTrips.py`` under ``$SUMO_HOME/tools``.
    """

    import os

    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home:
        home_path = Path(sumo_home)
        if tool_name.endswith(".py"):
            helper_path = home_path / "tools" / tool_name
            if helper_path.exists():
                return helper_path
        else:
            executable_path = home_path / "bin" / tool_name
            if executable_path.exists():
                return executable_path
            executable_path = home_path / "bin" / f"{tool_name}.exe"
            if executable_path.exists():
                return executable_path

    path_match = find_executable(tool_name)
    if path_match:
        return Path(path_match)
    return None


def discover_existing_sumo_inputs(
    xodr_path: str | Path | None,
    *,
    sumocfg_path: str | Path | None = None,
) -> ExistingSumoInputs:
    """Find existing SUMO files that belong to an OpenDRIVE dataset.

    SUMO network files do not embed route or additional-file references. This
    helper therefore reads a same-stem ``.sumocfg`` first, then falls back to
    same-folder files such as ``tempe.net.xml``, ``tempe.rou.xml``,
    ``tempe.add.xml``, and ``trips.trips.xml``.
    """

    if xodr_path is None:
        return ExistingSumoInputs()

    source_xodr = Path(xodr_path)
    if not source_xodr.exists():
        return ExistingSumoInputs()

    dataset_dir = source_xodr.parent
    stem = source_xodr.stem
    explicit_sumocfg = Path(sumocfg_path) if sumocfg_path else None
    candidate_sumocfg = explicit_sumocfg or dataset_dir / f"{stem}.sumocfg"
    discovered = ExistingSumoInputs()

    if candidate_sumocfg.exists():
        discovered = _read_sumo_config_inputs(candidate_sumocfg)

    discovered.net_file = discovered.net_file or _existing_path(
        dataset_dir / f"{stem}.net.xml"
    )
    discovered.route_files = _unique_existing_paths(
        *(discovered.route_files or []),
        dataset_dir / f"{stem}.rou.xml",
    )
    discovered.additional_files = _unique_existing_paths(
        *(discovered.additional_files or []),
        dataset_dir / f"{stem}.add.xml",
    )
    discovered.trip_files = _unique_existing_paths(
        *(discovered.trip_files or []),
        dataset_dir / f"{stem}.trips.xml",
        dataset_dir / "trips.trips.xml",
        *sorted(dataset_dir.glob("*.trips.xml")),
    )
    discovered.support_files = _unique_existing_paths(
        *(discovered.support_files or []),
        *(discovered.trip_files or []),
        dataset_dir / f"{stem}.flow.xml",
        dataset_dir / f"{stem}.nod.xml",
        dataset_dir / f"{stem}.edg.xml",
        dataset_dir / f"{stem}.con.xml",
    )
    return discovered


def build_sumo_network(
    prepared_xodr: str | Path,
    net_xml: str | Path,
    options: list[str] | None = None,
    *,
    dry_run: bool = False,
) -> list[str]:
    """Build or plan a SUMO ``.net.xml`` conversion from OpenDRIVE."""

    prepared_xodr_path = Path(prepared_xodr).resolve()
    net_xml_path = Path(net_xml).resolve()
    netconvert_path = find_sumo_tool("netconvert")
    if netconvert_path is None and not dry_run:
        raise RuntimeError(
            "SUMO netconvert was not found. Install SUMO and add its bin "
            "folder to PATH, or run this builder with dry_run=True."
        )

    command = [
        str(netconvert_path or "netconvert"),
        "--opendrive-files",
        str(prepared_xodr_path),
        "--output-file",
        str(net_xml_path),
        "--opendrive.import-all-lanes",
    ]
    command.extend(options or [])

    if dry_run:
        return command

    net_xml_path.parent.mkdir(parents=True, exist_ok=True)
    _run_command(command, cwd=net_xml_path.parent)
    return command


def assign_vehicle_types_to_routes(
    route_file: str | Path,
    ev_share: float,
    seed: int,
) -> None:
    """Assign ``ice_passenger`` or ``ev_passenger`` to trips and vehicles.

    Args:
        route_file: SUMO route file containing ``vehicle`` or ``trip`` records.
        ev_share: Fraction of records that should use the EV type.
        seed: Random seed used for reproducible assignment.
    """

    route_path = Path(route_file)
    if not route_path.exists():
        raise FileNotFoundError(f"SUMO route file not found: {route_path}")

    rng = random.Random(seed)
    tree = ET.parse(route_path)
    root = tree.getroot()
    vehicle_count = 0

    for element in root.iter():
        if element.tag not in {"vehicle", "trip"}:
            continue
        vehicle_type = "ev_passenger" if rng.random() < ev_share else "ice_passenger"
        element.set("type", vehicle_type)
        vehicle_count += 1

    if vehicle_count == 0:
        root.append(ET.Comment("No trip or vehicle records were available to type."))

    _indent_xml(root)
    tree.write(route_path, encoding="utf-8", xml_declaration=True)


def apply_ego_vehicle_config(
    route_file: str | Path,
    scenario_config: ScenarioConfig,
) -> list[str]:
    """Apply the configured ego mode to a SUMO route file.

    ``select_existing`` mode marks vehicles already present in the route file.
    ``add`` mode appends new ego trips or vehicles defined by OD pairs,
    existing route ids, or explicit edge lists.
    """

    ego = scenario_config.ego
    if not ego.enabled:
        return []

    route_path = Path(route_file)
    if not route_path.exists():
        raise FileNotFoundError(f"SUMO route file not found: {route_path}")

    tree = ET.parse(route_path)
    root = tree.getroot()
    existing_vehicle_elements = [
        element for element in root.iter() if element.tag in {"vehicle", "trip"}
    ]
    mode = _normalized_ego_mode(ego.mode)
    ego_ids: list[str] = []

    if mode == "select_existing":
        selected_ids = _select_ego_vehicle_ids(
            existing_vehicle_elements,
            scenario_config,
        )
        for element in existing_vehicle_elements:
            vehicle_id = element.get("id")
            if vehicle_id is None or vehicle_id not in selected_ids:
                continue
            _apply_ego_vehicle_attributes(element, scenario_config)
            ego_ids.append(vehicle_id)
    elif mode != "add":
        raise ValueError(
            "ego.mode must be 'select_existing' or 'add'. "
            f"Received: {scenario_config.ego.mode}"
        )

    for vehicle_config in _added_ego_vehicle_configs(scenario_config):
        appended_id = _append_configured_ego_vehicle(
            root, scenario_config, vehicle_config
        )
        if appended_id:
            ego_ids.append(appended_id)

    tree.write(route_path, encoding="utf-8", xml_declaration=True)
    return _deduplicate_strings(ego_ids)


def build_sumo_project(
    project_dir: str | Path | None = None,
    prepared_xodr: str | Path | None = None,
    scenario_config: ScenarioConfig | None = None,
    *,
    dry_run: bool = False,
) -> dict[str, object] | None:
    """Create a SUMO project directory with network, route, config, and scripts.

    The no-argument call is kept as a harmless no-op for backward compatibility
    with the original placeholder function.
    """

    if project_dir is None or prepared_xodr is None:
        return None

    scenario = scenario_config or ScenarioConfig()
    paths = make_project_paths(project_dir)
    paths.sumo_dir.mkdir(parents=True, exist_ok=True)
    (paths.sumo_dir / "outputs").mkdir(parents=True, exist_ok=True)

    net_xml = paths.sumo_dir / "network.net.xml"
    route_xml = paths.sumo_dir / "routes.rou.xml"
    vtypes_xml = paths.sumo_dir / "vtypes.add.xml"
    sumocfg = paths.sumo_dir / "scenario.sumocfg"
    commands: dict[str, list[str]] = {}
    explicit_sumocfg_file = resolve_config_path(
        scenario.sumo.source_sumocfg_file,
        scenario,
    )
    discovered_inputs = discover_existing_sumo_inputs(
        _source_xodr_from_project(project_dir, prepared_xodr),
        sumocfg_path=explicit_sumocfg_file,
    )
    if not scenario.sumo.auto_discover_source_files:
        discovered_inputs = ExistingSumoInputs()

    source_net_file = (
        resolve_config_path(scenario.sumo.source_net_file, scenario)
        or discovered_inputs.net_file
    )
    if source_net_file is not None:
        _copy_existing_file(source_net_file, net_xml, "SUMO source network")
        commands["network"] = ["copy", str(source_net_file), str(net_xml)]
    else:
        commands["network"] = build_sumo_network(
            prepared_xodr,
            net_xml,
            scenario.sumo.netconvert_options,
            dry_run=dry_run,
        )

    write_vehicle_types(vtypes_xml, scenario)

    additional_files = [vtypes_xml.name]
    copied_additional_files: list[Path] = []
    source_additional_files = _configured_or_discovered_files(
        scenario.sumo.source_additional_file,
        scenario,
        discovered_inputs.additional_files or [],
    )
    for index, source_additional_file in enumerate(source_additional_files):
        copied_additional_file = _copy_source_file_with_name(
            source_additional_file,
            paths.sumo_dir,
            fallback_name=f"dataset_{index}.add.xml",
            label="SUMO source additional file",
        )
        additional_files.append(copied_additional_file.name)
        copied_additional_files.append(copied_additional_file)
    if source_additional_files:
        commands["additional"] = [
            "copy",
            *[str(path) for path in source_additional_files],
            str(paths.sumo_dir),
        ]
    additional_sanitization = sanitize_sumo_additional_files(
        net_xml,
        copied_additional_files,
    )

    configured_route_files = _resolve_config_paths(
        scenario.sumo.source_route_file,
        scenario,
    )
    configured_trip_files = _resolve_config_paths(
        scenario.sumo.source_trip_file,
        scenario,
    )
    source_route_files = (
        configured_route_files
        or discovered_inputs.route_files
        or configured_trip_files
        or discovered_inputs.trip_files
        or []
    )
    copied_support_files = _copy_supporting_sumo_files(
        paths.sumo_dir,
        scenario,
        source_route_files,
        source_additional_files,
        discovered_inputs,
    )
    if copied_support_files:
        commands["support_files"] = [
            "copy",
            *[str(path) for path in copied_support_files],
        ]

    if source_route_files:
        _copy_existing_file(
            source_route_files[0],
            route_xml,
            "SUMO source route or trip file",
        )
        if scenario.sumo.assign_vehicle_types_to_source_routes:
            assign_vehicle_types_to_routes(
                route_xml,
                scenario.fleet.ev_share,
                scenario.random_seed,
            )
        commands["routes"] = ["copy", str(source_route_files[0]), str(route_xml)]
    else:
        commands["routes"] = generate_random_trips(
            paths.sumo_dir,
            scenario,
            dry_run=dry_run,
        )

    ego_vehicle_ids = apply_ego_vehicle_config(route_xml, scenario)
    ego_metadata_path = paths.sumo_dir / "ego_vehicles.json"
    ego_metadata_path.write_text(
        json.dumps(
            {
                "enabled": scenario.ego.enabled,
                "mode": scenario.ego.mode,
                "vehicle_ids": ego_vehicle_ids,
                "vehicle_type_id": scenario.ego.vehicle_type_id,
                "highlight_enabled": scenario.ego.highlight_enabled,
                "highlight_color": scenario.ego.highlight_color,
                "collect_only_ego": scenario.ego.collect_only_ego,
                "analyze_only_ego": scenario.ego.analyze_only_ego,
                "carla_actor_ids": scenario.ego.carla_actor_ids,
                "carla_role_names": scenario.ego.carla_role_names,
                "carla_type_ids": scenario.ego.carla_type_ids,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    write_sumo_config(
        sumocfg,
        scenario,
        additional_files=additional_files,
        begin=_discovered_time_value(
            source_route_files, scenario, discovered_inputs.begin
        ),
        end=_discovered_time_value(source_route_files, scenario, discovered_inputs.end),
        step_length=_discovered_time_value(
            source_route_files,
            scenario,
            discovered_inputs.step_length,
        ),
    )
    write_sumo_run_scripts(paths.sumo_dir)

    plan = {
        "scenario_id": scenario.scenario_id,
        "dry_run": dry_run,
        "commands": commands,
        "files": {
            "net": str(net_xml),
            "routes": str(route_xml),
            "vehicle_types": str(vtypes_xml),
            "sumocfg": str(sumocfg),
            "ego_vehicles": str(ego_metadata_path),
        },
        "ego_vehicle_ids": ego_vehicle_ids,
        "discovered_source_files": _discovered_inputs_to_dict(discovered_inputs),
        "additional_sanitization": additional_sanitization,
    }
    (paths.sumo_dir / "build_sumo_plan.json").write_text(
        json.dumps(plan, indent=2),
        encoding="utf-8",
    )
    _write_command_text(paths.sumo_dir / "planned_commands.txt", commands)
    return plan


def generate_random_trips(
    sumo_dir: Path,
    scenario_config: ScenarioConfig,
    *,
    dry_run: bool,
) -> list[str]:
    """Generate or plan SUMO ``randomTrips.py`` demand creation."""

    random_trips_path = find_sumo_tool("randomTrips.py")
    if random_trips_path is None and not dry_run:
        raise RuntimeError(
            "SUMO randomTrips.py was not found. Set SUMO_HOME to a SUMO "
            "installation or provide sumo.source_route_file in the scenario."
        )

    route_xml = sumo_dir / "routes.rou.xml"
    command = [
        sys.executable,
        str(random_trips_path or "randomTrips.py"),
        "-n",
        "network.net.xml",
        "-r",
        "routes.rou.xml",
        "--begin",
        str(scenario_config.simulation.begin),
        "--end",
        str(scenario_config.simulation.end),
        "--period",
        str(scenario_config.demand.period),
        "--seed",
        str(scenario_config.demand.seed),
    ]
    if scenario_config.demand.validate_routes:
        command.append("--validate")
    command.extend(scenario_config.sumo.random_trips_options)

    if dry_run:
        route_xml.write_text(
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            "<routes>\n"
            "  <!-- Dry run placeholder. Run randomTrips.py to generate demand. -->\n"
            "</routes>\n",
            encoding="utf-8",
        )
        return command

    _run_command(command, cwd=sumo_dir)
    assign_vehicle_types_to_routes(
        route_xml,
        scenario_config.fleet.ev_share,
        scenario_config.random_seed,
    )
    return command


def write_vehicle_types(path: str | Path, scenario_config: ScenarioConfig) -> None:
    """Write ICE and EV vehicle types used by generated or copied routes."""

    fleet = scenario_config.fleet
    root = ET.Element("additional")
    ET.SubElement(
        root,
        "vType",
        {
            "id": "ice_passenger",
            "vClass": "passenger",
            "accel": "2.6",
            "decel": "4.5",
            "sigma": str(fleet.sigma),
            "length": "5.0",
            "maxSpeed": str(fleet.max_speed_mps),
            "emissionClass": fleet.ice_emission_class,
        },
    )
    ev_type = ET.SubElement(
        root,
        "vType",
        {
            "id": "ev_passenger",
            "vClass": "passenger",
            "accel": "2.6",
            "decel": "4.5",
            "sigma": str(fleet.sigma),
            "length": "5.0",
            "maxSpeed": str(fleet.max_speed_mps),
            "emissionClass": fleet.ev_emission_class,
            "mass": "1830",
        },
    )
    for key, value in {
        "has.battery.device": "true",
        "device.battery.capacity": "64000",
        "maximumPower": "150000",
        "frontSurfaceArea": "2.6",
        "airDragCoefficient": "0.35",
        "rotatingMass": "40",
        "rollDragCoefficient": "0.01",
        "constantPowerIntake": "100",
        "propulsionEfficiency": "0.98",
        "recuperationEfficiency": "0.96",
    }.items():
        ET.SubElement(ev_type, "param", {"key": key, "value": value})

    if scenario_config.ego.enabled:
        ego_attributes = {
            "id": scenario_config.ego.vehicle_type_id,
            "vClass": "passenger",
            "accel": "3.0",
            "decel": "4.5",
            "sigma": "0.0",
            "length": "5.0",
            "maxSpeed": str(fleet.max_speed_mps),
        }
        if scenario_config.ego.highlight_enabled:
            ego_attributes["color"] = scenario_config.ego.highlight_color
        ego_attributes.update(
            {
                key: str(value)
                for key, value in scenario_config.ego.vtype_attributes.items()
            }
        )
        ego_attributes["id"] = scenario_config.ego.vehicle_type_id
        ego_type = ET.SubElement(root, "vType", ego_attributes)
        for key, value in scenario_config.ego.vtype_params.items():
            ET.SubElement(ego_type, "param", {"key": str(key), "value": str(value)})

    _indent_xml(root)
    tree = ET.ElementTree(root)
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def write_sumo_config(
    path: str | Path,
    scenario_config: ScenarioConfig,
    *,
    additional_files: list[str],
    begin: float | None = None,
    end: float | None = None,
    step_length: float | None = None,
) -> None:
    """Write the SUMO configuration file for one scenario."""

    begin_value = scenario_config.simulation.begin if begin is None else begin
    end_value = scenario_config.simulation.end if end is None else end
    step_value = (
        scenario_config.simulation.step_length if step_length is None else step_length
    )

    root = ET.Element("configuration")
    input_elem = ET.SubElement(root, "input")
    ET.SubElement(input_elem, "net-file", {"value": "network.net.xml"})
    ET.SubElement(input_elem, "route-files", {"value": "routes.rou.xml"})
    ET.SubElement(
        input_elem,
        "additional-files",
        {"value": ",".join(additional_files)},
    )

    time_elem = ET.SubElement(root, "time")
    ET.SubElement(time_elem, "begin", {"value": str(begin_value)})
    ET.SubElement(time_elem, "end", {"value": str(end_value)})
    ET.SubElement(
        time_elem,
        "step-length",
        {"value": str(step_value)},
    )

    output_elem = ET.SubElement(root, "output")
    if scenario_config.sumo.write_tripinfo:
        ET.SubElement(output_elem, "tripinfo-output", {"value": "outputs/tripinfo.xml"})
    if scenario_config.sumo.write_emissions:
        ET.SubElement(
            output_elem, "emission-output", {"value": "outputs/emissions.xml"}
        )
    if scenario_config.sumo.write_battery:
        ET.SubElement(output_elem, "battery-output", {"value": "outputs/battery.xml"})
    if scenario_config.sumo.write_fcd:
        ET.SubElement(output_elem, "fcd-output", {"value": "outputs/fcd.xml"})
    ET.SubElement(output_elem, "summary-output", {"value": "outputs/summary.xml"})
    ET.SubElement(output_elem, "statistic-output", {"value": "outputs/statistics.xml"})

    processing_elem = ET.SubElement(root, "processing")
    ET.SubElement(processing_elem, "ignore-route-errors", {"value": "true"})
    ET.SubElement(processing_elem, "time-to-teleport", {"value": "-1"})

    _indent_xml(root)
    tree = ET.ElementTree(root)
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def write_sumo_run_scripts(sumo_dir: str | Path) -> None:
    """Write shell and Windows scripts that run the generated SUMO case."""

    directory = Path(sumo_dir)
    (directory / "run_sumo.sh").write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        'cd "$(dirname "$0")"\n'
        "sumo -c scenario.sumocfg\n",
        encoding="utf-8",
    )
    (directory / "run_sumo.bat").write_text(
        "@echo off\r\ncd /d %~dp0\r\nsumo -c scenario.sumocfg\r\n",
        encoding="utf-8",
    )


def _copy_existing_file(source_path: Path, target_path: Path, label: str) -> None:
    if not source_path.exists():
        raise FileNotFoundError(f"{label} not found: {source_path}")
    target_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_path, target_path)


def _select_ego_vehicle_ids(
    route_elements: list[ET.Element],
    scenario_config: ScenarioConfig,
) -> set[str]:
    ego = scenario_config.ego
    selected_ids = {str(vehicle_id) for vehicle_id in ego.vehicle_ids}
    if ego.auto_select_count <= 0:
        return selected_ids

    available_ids = [
        vehicle_id
        for element in route_elements
        if (vehicle_id := element.get("id")) and vehicle_id not in selected_ids
    ]
    if ego.selection_strategy == "random":
        seed = (
            ego.auto_select_seed
            if ego.auto_select_seed is not None
            else scenario_config.random_seed
        )
        random.Random(seed).shuffle(available_ids)

    selected_ids.update(available_ids[: ego.auto_select_count])
    return selected_ids


def _apply_ego_vehicle_attributes(
    element: ET.Element,
    scenario_config: ScenarioConfig,
) -> None:
    element.set("type", scenario_config.ego.vehicle_type_id)
    for key, value in _ego_vehicle_attributes(scenario_config).items():
        element.set(str(key), str(value))


def _added_ego_vehicle_configs(
    scenario_config: ScenarioConfig,
) -> list[dict[str, object]]:
    """Return add-mode ego vehicle specs from all supported config aliases."""

    ego = scenario_config.ego
    if _normalized_ego_mode(ego.mode) != "add":
        return []

    vehicle_configs: list[dict[str, object]] = []
    vehicle_configs.extend(dict(vehicle) for vehicle in ego.added_vehicles)
    vehicle_configs.extend(dict(vehicle) for vehicle in ego.vehicles)
    for od_pair in ego.od_pairs:
        vehicle_configs.append(_normalize_od_pair_config(od_pair))
    for route_vehicle in ego.route_vehicles:
        vehicle_configs.append(_normalize_route_vehicle_config(route_vehicle))
    return vehicle_configs


def _normalize_od_pair_config(vehicle_config: dict[str, object]) -> dict[str, object]:
    """Normalize OD aliases into SUMO's ``from`` and ``to`` attributes."""

    normalized = dict(vehicle_config)
    from_edge = _first_config_value(
        normalized,
        ["from", "from_edge", "origin", "origin_edge"],
    )
    to_edge = _first_config_value(
        normalized,
        ["to", "to_edge", "destination", "destination_edge"],
    )
    if from_edge is not None:
        normalized["from"] = from_edge
    if to_edge is not None:
        normalized["to"] = to_edge
    return normalized


def _normalize_route_vehicle_config(
    vehicle_config: dict[str, object],
) -> dict[str, object]:
    """Normalize route aliases into ``route_edges`` or ``route_id``."""

    normalized = dict(vehicle_config)
    edge_list = _first_config_value(
        normalized,
        ["route_edges", "edge_list", "edges"],
    )
    if edge_list is not None:
        normalized["route_edges"] = edge_list

    route_id = _first_config_value(normalized, ["route_id"])
    if route_id is not None:
        normalized["route_id"] = route_id
    return normalized


def _append_configured_ego_vehicle(
    root: ET.Element,
    scenario_config: ScenarioConfig,
    vehicle_config: dict[str, object],
) -> str | None:
    vehicle_id = str(vehicle_config.get("id") or "").strip()
    if not vehicle_id:
        raise ValueError("Each configured ego vehicle must include a non-empty id.")

    route_edges = _first_config_value(
        vehicle_config,
        ["route_edges", "edge_list", "edges"],
    )
    route_id = _first_config_value(vehicle_config, ["route_id"])
    from_edge = _first_config_value(
        vehicle_config,
        ["from", "from_edge", "origin", "origin_edge"],
    )
    to_edge = _first_config_value(
        vehicle_config,
        ["to", "to_edge", "destination", "destination_edge"],
    )
    attributes = _ego_vehicle_attributes(scenario_config)
    for key, value in vehicle_config.items():
        if key in {
            "route_edges",
            "edge_list",
            "edges",
            "route_id",
            "from_edge",
            "origin",
            "origin_edge",
            "to_edge",
            "destination",
            "destination_edge",
        }:
            continue
        attributes[str(key)] = str(value)

    attributes.setdefault("id", vehicle_id)
    attributes.setdefault("type", scenario_config.ego.vehicle_type_id)
    attributes.setdefault("depart", str(scenario_config.simulation.begin))
    if route_id is not None:
        attributes.setdefault("route", str(route_id))
    if from_edge is not None:
        attributes.setdefault("from", str(from_edge))
    if to_edge is not None:
        attributes.setdefault("to", str(to_edge))

    if route_edges:
        vehicle = ET.SubElement(root, "vehicle", attributes)
        if isinstance(route_edges, list):
            edges_value = " ".join(str(edge) for edge in route_edges)
        else:
            edges_value = str(route_edges)
        ET.SubElement(vehicle, "route", {"edges": edges_value})
    elif "from" in attributes and "to" in attributes:
        ET.SubElement(root, "trip", attributes)
    else:
        ET.SubElement(root, "vehicle", attributes)
    return vehicle_id


def _ego_vehicle_attributes(scenario_config: ScenarioConfig) -> dict[str, str]:
    """Return attributes applied to every ego vehicle or ego trip."""

    attributes = {
        key: str(value) for key, value in scenario_config.ego.vehicle_attributes.items()
    }
    if scenario_config.ego.highlight_enabled:
        attributes.setdefault("color", scenario_config.ego.highlight_color)
    return attributes


def _first_config_value(
    vehicle_config: dict[str, object],
    keys: list[str],
) -> object | None:
    for key in keys:
        value = vehicle_config.get(key)
        if value is None or value == "":
            continue
        if isinstance(value, list) and not value:
            continue
        if isinstance(value, dict) and not value:
            continue
        return value
    return None


def _normalized_ego_mode(raw_mode: str) -> str:
    mode = str(raw_mode).strip().lower()
    if mode in EGO_ADD_MODES:
        return "add"
    if mode in EGO_SELECT_EXISTING_MODES:
        return "select_existing"
    return mode


def _deduplicate_strings(values: list[str]) -> list[str]:
    deduplicated_values: list[str] = []
    seen_values: set[str] = set()
    for value in values:
        if value in seen_values:
            continue
        seen_values.add(value)
        deduplicated_values.append(value)
    return deduplicated_values


def _discovered_time_value(
    source_route_files: list[Path],
    scenario_config: ScenarioConfig,
    discovered_value: float | None,
) -> float | None:
    if not source_route_files or not scenario_config.sumo.use_source_sumocfg_time:
        return None
    return discovered_value


def _copy_source_file_with_name(
    source_path: Path,
    target_dir: Path,
    *,
    fallback_name: str,
    label: str,
) -> Path:
    if not source_path.exists():
        raise FileNotFoundError(f"{label} not found: {source_path}")
    target_dir.mkdir(parents=True, exist_ok=True)
    target_path = target_dir / (source_path.name or fallback_name)
    shutil.copy2(source_path, target_path)
    return target_path


def _copy_supporting_sumo_files(
    sumo_dir: Path,
    scenario_config: ScenarioConfig,
    source_route_files: list[Path],
    source_additional_files: list[Path],
    discovered_inputs: ExistingSumoInputs,
) -> list[Path]:
    """Copy companion SUMO files that support reproducibility but are not inputs."""

    explicit_trip_file = resolve_config_path(
        scenario_config.sumo.source_trip_file,
        scenario_config,
    )
    explicit_sumocfg_file = resolve_config_path(
        scenario_config.sumo.source_sumocfg_file,
        scenario_config,
    )
    input_files = {
        path.resolve()
        for path in [
            *source_route_files,
            *source_additional_files,
            discovered_inputs.net_file,
        ]
        if path is not None
    }
    support_candidates = _unique_existing_paths(
        explicit_trip_file,
        explicit_sumocfg_file,
        discovered_inputs.sumocfg_file,
        *(discovered_inputs.support_files or []),
    )
    copied_files: list[Path] = []
    for source_file in support_candidates:
        if source_file.resolve() in input_files:
            continue
        copied_file = _copy_source_file_with_name(
            source_file,
            sumo_dir,
            fallback_name=source_file.name,
            label="SUMO supporting source file",
        )
        copied_files.append(copied_file)
    return copied_files


def sanitize_sumo_project_additional_files(
    sumo_dir: str | Path,
) -> dict[str, object]:
    """Remove lane-based additional elements that reference missing lanes.

    SUMO stops immediately when an ``additional`` file contains an induction
    loop or lane-area detector for a lane that is not present in the active
    network. Dataset companion ``*.add.xml`` files can become stale after a
    network is rebuilt, so this check keeps copied detector files consistent
    with ``scenario.sumocfg`` before SUMO runs.
    """

    sumo_path = Path(sumo_dir)
    sumocfg_path = sumo_path / "scenario.sumocfg"
    if not sumocfg_path.exists():
        return {
            "status": "skipped_missing_sumocfg",
            "sumocfg": str(sumocfg_path),
            "files": [],
        }

    root = ET.parse(sumocfg_path).getroot()
    input_values = _sumo_config_input_values(root)
    net_file = _first_existing_config_path(
        sumo_path,
        input_values.get("net-file", []),
    )
    additional_files = _resolve_sumo_config_paths(
        sumo_path,
        input_values.get("additional-files", []),
    )
    return sanitize_sumo_additional_files(net_file, additional_files)


def sanitize_sumo_additional_files(
    net_file: str | Path | None,
    additional_files: list[str | Path],
) -> dict[str, object]:
    """Sanitize multiple SUMO additional files against one network file."""

    if net_file is None:
        return {
            "status": "skipped_missing_net_file",
            "net_file": None,
            "files": [],
        }

    net_path = Path(net_file)
    lane_ids = _sumo_net_lane_ids(net_path)
    if not lane_ids:
        return {
            "status": "skipped_no_network_lanes",
            "net_file": str(net_path),
            "files": [],
        }

    file_summaries = [
        sanitize_sumo_additional_file(additional_file, lane_ids)
        for additional_file in additional_files
    ]
    removed_count = sum(
        int(summary.get("removed_count", 0)) for summary in file_summaries
    )
    return {
        "status": "sanitized" if removed_count else "unchanged",
        "net_file": str(net_path),
        "removed_count": removed_count,
        "files": file_summaries,
    }


def sanitize_sumo_additional_file(
    additional_file: str | Path,
    valid_lane_ids: set[str],
) -> dict[str, object]:
    """Remove invalid lane-referencing records from one additional XML file."""

    additional_path = Path(additional_file)
    if not additional_path.exists():
        return {
            "file": str(additional_path),
            "status": "skipped_missing_file",
            "removed_count": 0,
            "missing_lanes": [],
        }

    tree = ET.parse(additional_path)
    root = tree.getroot()
    parent_by_child = {child: parent for parent in root.iter() for child in parent}
    removed_count = 0
    missing_lanes: list[str] = []

    for element in list(root.iter()):
        if element is root:
            continue
        referenced_lanes = _additional_element_lane_references(element)
        if not referenced_lanes:
            continue
        invalid_lanes = [
            lane_id for lane_id in referenced_lanes if lane_id not in valid_lane_ids
        ]
        if not invalid_lanes:
            continue
        parent = parent_by_child.get(element)
        if parent is None:
            continue
        parent.remove(element)
        removed_count += 1
        missing_lanes.extend(invalid_lanes)

    if removed_count:
        _indent_xml(root)
        tree.write(additional_path, encoding="utf-8", xml_declaration=True)

    return {
        "file": str(additional_path),
        "status": "sanitized" if removed_count else "unchanged",
        "removed_count": removed_count,
        "missing_lanes": _deduplicate_strings(missing_lanes)[:20],
    }


def _sumo_net_lane_ids(net_file: Path) -> set[str]:
    if not net_file.exists():
        return set()
    root = ET.parse(net_file).getroot()
    lane_ids: set[str] = set()
    for lane in root.iter("lane"):
        lane_id = lane.get("id")
        if lane_id:
            lane_ids.add(lane_id)
    return lane_ids


def _additional_element_lane_references(element: ET.Element) -> list[str]:
    lane_refs: list[str] = []
    lane_id = element.get("lane")
    if lane_id:
        lane_refs.append(lane_id)

    raw_lanes = element.get("lanes")
    if raw_lanes:
        for lane_part in raw_lanes.replace(",", " ").split():
            if lane_part:
                lane_refs.append(lane_part)
    return lane_refs


def _configured_or_discovered_files(
    configured_value: str | Path | None,
    scenario_config: ScenarioConfig,
    discovered_files: list[Path],
) -> list[Path]:
    configured_paths = _resolve_config_paths(configured_value, scenario_config)
    if configured_paths:
        return configured_paths
    return discovered_files


def _resolve_config_paths(
    configured_value: str | Path | None,
    scenario_config: ScenarioConfig,
) -> list[Path]:
    if configured_value in {None, ""}:
        return []
    raw_parts = str(configured_value).split(",")
    resolved_paths: list[Path] = []
    for raw_part in raw_parts:
        raw_part = raw_part.strip()
        if not raw_part:
            continue
        resolved_path = resolve_config_path(raw_part, scenario_config)
        if resolved_path is not None:
            resolved_paths.append(resolved_path)
    return resolved_paths


def _source_xodr_from_project(
    project_dir: str | Path,
    prepared_xodr: str | Path,
) -> Path | None:
    summary_path = make_project_paths(project_dir).network_summary
    if summary_path.exists():
        try:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            summary = {}
        source_xodr = summary.get("source_xodr")
        if source_xodr and Path(source_xodr).exists():
            return Path(source_xodr)
    prepared_path = Path(prepared_xodr)
    return prepared_path if prepared_path.exists() else None


def _read_sumo_config_inputs(sumocfg_path: Path) -> ExistingSumoInputs:
    root = ET.parse(sumocfg_path).getroot()
    config_dir = sumocfg_path.parent
    discovered = ExistingSumoInputs(sumocfg_file=sumocfg_path)

    input_values = _sumo_config_input_values(root)
    discovered.net_file = _first_existing_config_path(
        config_dir,
        input_values.get("net-file", []),
    )
    discovered.route_files = _resolve_sumo_config_paths(
        config_dir,
        [
            *input_values.get("route-files", []),
            *input_values.get("route-file", []),
        ],
    )
    discovered.additional_files = _resolve_sumo_config_paths(
        config_dir,
        input_values.get("additional-files", []),
    )
    discovered.trip_files = _resolve_sumo_config_paths(
        config_dir,
        [
            *input_values.get("trip-files", []),
            *input_values.get("trip-file", []),
        ],
    )
    discovered.begin = _sumo_config_time_value(root, "begin")
    discovered.end = _sumo_config_time_value(root, "end")
    discovered.step_length = _sumo_config_time_value(root, "step-length")
    return discovered


def _sumo_config_input_values(root: ET.Element) -> dict[str, list[str]]:
    values: dict[str, list[str]] = {}
    for element in root.iter():
        tag_name = _strip_namespace(element.tag)
        if tag_name not in {
            "net-file",
            "route-file",
            "route-files",
            "additional-files",
            "trip-file",
            "trip-files",
        }:
            continue
        raw_value = element.get("value")
        if raw_value:
            values.setdefault(tag_name, []).append(raw_value)
    return values


def _sumo_config_time_value(root: ET.Element, tag_name: str) -> float | None:
    for element in root.iter():
        if _strip_namespace(element.tag) != tag_name:
            continue
        raw_value = element.get("value")
        if raw_value is None:
            return None
        try:
            return float(raw_value)
        except ValueError:
            return None
    return None


def _resolve_sumo_config_paths(config_dir: Path, raw_values: list[str]) -> list[Path]:
    paths: list[Path] = []
    for raw_value in raw_values:
        for part in raw_value.split(","):
            part = part.strip()
            if not part:
                continue
            candidate = Path(part)
            if not candidate.is_absolute():
                candidate = config_dir / candidate
            if candidate.exists():
                paths.append(candidate.resolve())
    return _unique_existing_paths(*paths)


def _first_existing_config_path(
    config_dir: Path,
    raw_values: list[str],
) -> Path | None:
    paths = _resolve_sumo_config_paths(config_dir, raw_values)
    return paths[0] if paths else None


def _existing_path(path: str | Path | None) -> Path | None:
    if path is None:
        return None
    candidate = Path(path)
    return candidate.resolve() if candidate.exists() else None


def _unique_existing_paths(*paths: str | Path | None) -> list[Path]:
    unique_paths: list[Path] = []
    seen_paths: set[Path] = set()
    for path in paths:
        if path is None:
            continue
        candidate = Path(path)
        if not candidate.exists():
            continue
        resolved_path = candidate.resolve()
        if resolved_path in seen_paths:
            continue
        seen_paths.add(resolved_path)
        unique_paths.append(resolved_path)
    return unique_paths


def _discovered_inputs_to_dict(
    discovered_inputs: ExistingSumoInputs,
) -> dict[str, object]:
    return {
        "net_file": str(discovered_inputs.net_file)
        if discovered_inputs.net_file
        else None,
        "route_files": [str(path) for path in discovered_inputs.route_files or []],
        "additional_files": [
            str(path) for path in discovered_inputs.additional_files or []
        ],
        "trip_files": [str(path) for path in discovered_inputs.trip_files or []],
        "support_files": [str(path) for path in discovered_inputs.support_files or []],
        "sumocfg_file": str(discovered_inputs.sumocfg_file)
        if discovered_inputs.sumocfg_file
        else None,
        "begin": discovered_inputs.begin,
        "end": discovered_inputs.end,
        "step_length": discovered_inputs.step_length,
    }


def _strip_namespace(tag_name: str) -> str:
    return tag_name.rsplit("}", 1)[-1]


def _run_command(command: list[str], *, cwd: Path) -> None:
    proc = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        cwd=cwd,
    )
    if proc.returncode != 0:
        detail = "\n".join(part for part in [proc.stdout, proc.stderr] if part)
        raise RuntimeError(
            f"Command failed with exit code {proc.returncode}: {' '.join(command)}\n"
            f"{detail}"
        )


def _write_command_text(path: Path, commands: dict[str, list[str]]) -> None:
    lines = []
    for name, command in commands.items():
        lines.append(f"[{name}]")
        lines.append(" ".join(command))
        lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def _indent_xml(element: ET.Element, level: int = 0) -> None:
    """Pretty-print XML in place using spaces for readability."""

    indent = "\n" + level * "  "
    child_indent = "\n" + (level + 1) * "  "
    children = list(element)
    if children:
        if not element.text or not element.text.strip():
            element.text = child_indent
        for child in children:
            _indent_xml(child, level + 1)
        if not children[-1].tail or not children[-1].tail.strip():
            children[-1].tail = indent
    if level and (not element.tail or not element.tail.strip()):
        element.tail = indent
