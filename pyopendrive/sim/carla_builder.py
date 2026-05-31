"""CARLA OpenDRIVE project generation utilities."""

from __future__ import annotations

import json
from pathlib import Path
import textwrap

from .config import ScenarioConfig
from .project import make_project_paths


def build_carla_project(
    project_dir: str | Path | None = None,
    prepared_xodr: str | Path | None = None,
    scenario_config: ScenarioConfig | None = None,
    *,
    dry_run: bool = False,
) -> dict[str, object] | None:
    """Create CARLA scripts for loading OpenDRIVE and collecting telemetry.

    The generated files do not start a CARLA server. They assume the server is
    already running and fail clearly if the Python API cannot connect.
    """

    if project_dir is None or prepared_xodr is None:
        return None

    scenario = scenario_config or ScenarioConfig()
    paths = make_project_paths(project_dir)
    paths.carla_dir.mkdir(parents=True, exist_ok=True)
    (paths.carla_dir / "outputs").mkdir(parents=True, exist_ok=True)

    load_world_script = paths.carla_dir / "load_opendrive_world.py"
    telemetry_script = paths.carla_dir / "collect_telemetry.py"
    write_load_opendrive_script(load_world_script, scenario)
    write_collect_telemetry_script(telemetry_script, scenario)
    write_carla_run_scripts(paths.carla_dir)

    plan = {
        "scenario_id": scenario.scenario_id,
        "dry_run": dry_run,
        "files": {
            "load_opendrive_world": str(load_world_script),
            "collect_telemetry": str(telemetry_script),
        },
    }
    (paths.carla_dir / "build_carla_plan.json").write_text(
        json.dumps(plan, indent=2),
        encoding="utf-8",
    )
    return plan


def write_load_opendrive_script(
    path: str | Path, scenario_config: ScenarioConfig
) -> None:
    """Write a CARLA client script that loads the prepared OpenDRIVE file."""

    carla = scenario_config.carla
    simulation = scenario_config.simulation
    script = f"""
from pathlib import Path

try:
    import carla
except ImportError as exc:
    raise RuntimeError(
        "The CARLA Python API is required. Add CARLA/PythonAPI/carla to "
        "PYTHONPATH or install a matching carla package."
    ) from exc


PROJECT_DIR = Path(__file__).resolve().parents[1]
XODR_PATH = PROJECT_DIR / "network" / "prepared.xodr"

client = carla.Client({carla.host!r}, {carla.port})
client.set_timeout({carla.timeout})

xodr_content = XODR_PATH.read_text(encoding="utf-8")
params = carla.OpendriveGenerationParameters(
    vertex_distance={carla.vertex_distance},
    max_road_length={carla.max_road_length},
    wall_height={carla.wall_height},
    additional_width={carla.additional_width},
    smooth_junctions={carla.smooth_junctions},
    enable_mesh_visibility=True,
)

world = client.generate_opendrive_world(xodr_content, params)
settings = world.get_settings()
settings.fixed_delta_seconds = {simulation.carla_fixed_delta_seconds}
settings.synchronous_mode = True
world.apply_settings(settings)

print(f"CARLA OpenDRIVE world generated from {{XODR_PATH}}.")
"""
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(_clean_script(script), encoding="utf-8")


def write_collect_telemetry_script(
    path: str | Path, scenario_config: ScenarioConfig
) -> None:
    """Write a CARLA client script that records per-vehicle telemetry to CSV."""

    carla = scenario_config.carla
    simulation = scenario_config.simulation
    ego = scenario_config.ego
    ego_vehicle_ids = sorted(str(vehicle_id) for vehicle_id in ego.vehicle_ids)
    ego_role_names = sorted(
        {*ego_vehicle_ids, *[str(name) for name in ego.carla_role_names]}
    )
    ego_actor_ids = sorted(int(actor_id) for actor_id in ego.carla_actor_ids)
    ego_type_ids = sorted(str(type_id) for type_id in ego.carla_type_ids)
    script = f"""
from __future__ import annotations

import csv
import math
from pathlib import Path

try:
    import carla
except ImportError as exc:
    raise RuntimeError(
        "The CARLA Python API is required. Add CARLA/PythonAPI/carla to "
        "PYTHONPATH or install a matching carla package."
    ) from exc


PROJECT_DIR = Path(__file__).resolve().parents[1]
OUTPUT_PATH = PROJECT_DIR / "carla" / "outputs" / "carla_telemetry.csv"
FIELDNAMES = [
    "frame",
    "time_s",
    "actor_id",
    "type_id",
    "role_name",
    "sumo_id",
    "is_ego",
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
    "vx",
    "vy",
    "vz",
    "ax",
    "ay",
    "az",
    "speed_mps",
    "accel_mps2",
    "jerk_mps3",
]
EGO_ENABLED = {ego.enabled!r}
COLLECT_ONLY_EGO = {ego.collect_only_ego!r}
EGO_VEHICLE_IDS = set({json.dumps(ego_vehicle_ids)})
EGO_ROLE_NAMES = set({json.dumps(ego_role_names)})
EGO_ACTOR_IDS = set({json.dumps(ego_actor_ids)})
EGO_TYPE_IDS = set({json.dumps(ego_type_ids)})


def vector_magnitude(x_value: float, y_value: float, z_value: float) -> float:
    return math.sqrt(x_value * x_value + y_value * y_value + z_value * z_value)


def actor_ego_fields(actor) -> tuple[str, str, bool]:
    attributes = getattr(actor, "attributes", {{}}) or {{}}
    role_name = str(attributes.get("role_name", ""))
    sumo_id = str(
        attributes.get("sumo_id", "")
        or attributes.get("sumo_vehicle_id", "")
        or attributes.get("sumo_actor_id", "")
    )
    is_ego = (
        str(actor.id) in {{str(actor_id) for actor_id in EGO_ACTOR_IDS}}
        or actor.id in EGO_ACTOR_IDS
        or actor.type_id in EGO_TYPE_IDS
        or role_name in EGO_ROLE_NAMES
        or role_name in EGO_VEHICLE_IDS
        or sumo_id in EGO_VEHICLE_IDS
    )
    return role_name, sumo_id, is_ego


client = carla.Client({carla.host!r}, {carla.port})
client.set_timeout({carla.timeout})
world = client.get_world()
OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)

previous_accel_by_actor = {{}}
previous_time_by_actor = {{}}
end_time = {simulation.end}
fixed_step = {simulation.carla_fixed_delta_seconds}

with OUTPUT_PATH.open("w", newline="", encoding="utf-8") as file_obj:
    writer = csv.DictWriter(file_obj, fieldnames=FIELDNAMES)
    writer.writeheader()
    elapsed_time = 0.0
    while elapsed_time <= end_time:
        snapshot = world.tick()
        timestamp = world.get_snapshot().timestamp
        elapsed_time = float(timestamp.elapsed_seconds)
        frame = int(snapshot)
        for actor in world.get_actors().filter("vehicle.*"):
            role_name, sumo_id, is_ego = actor_ego_fields(actor)
            if EGO_ENABLED and COLLECT_ONLY_EGO and not is_ego:
                continue
            transform = actor.get_transform()
            velocity = actor.get_velocity()
            acceleration = actor.get_acceleration()
            speed = vector_magnitude(velocity.x, velocity.y, velocity.z)
            accel = vector_magnitude(acceleration.x, acceleration.y, acceleration.z)
            previous_accel = previous_accel_by_actor.get(actor.id)
            previous_time = previous_time_by_actor.get(actor.id)
            if previous_accel is None or previous_time is None:
                jerk = 0.0
            else:
                time_delta = max(elapsed_time - previous_time, fixed_step)
                jerk = abs(accel - previous_accel) / time_delta
            previous_accel_by_actor[actor.id] = accel
            previous_time_by_actor[actor.id] = elapsed_time
            writer.writerow(
                {{
                    "frame": frame,
                    "time_s": elapsed_time,
                    "actor_id": actor.id,
                    "type_id": actor.type_id,
                    "role_name": role_name,
                    "sumo_id": sumo_id,
                    "is_ego": is_ego,
                    "x": transform.location.x,
                    "y": transform.location.y,
                    "z": transform.location.z,
                    "roll": transform.rotation.roll,
                    "pitch": transform.rotation.pitch,
                    "yaw": transform.rotation.yaw,
                    "vx": velocity.x,
                    "vy": velocity.y,
                    "vz": velocity.z,
                    "ax": acceleration.x,
                    "ay": acceleration.y,
                    "az": acceleration.z,
                    "speed_mps": speed,
                    "accel_mps2": accel,
                    "jerk_mps3": jerk,
                }}
            )

print(f"CARLA telemetry written to {{OUTPUT_PATH}}.")
"""
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(_clean_script(script), encoding="utf-8")


def write_carla_run_scripts(carla_dir: str | Path) -> None:
    """Write scripts that run the generated CARLA client programs."""

    directory = Path(carla_dir)
    (directory / "run_carla_client.sh").write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        'cd "$(dirname "$0")"\n'
        "python load_opendrive_world.py\n"
        "python collect_telemetry.py\n",
        encoding="utf-8",
    )
    (directory / "run_carla_client.bat").write_text(
        "@echo off\r\n"
        "cd /d %~dp0\r\n"
        "python load_opendrive_world.py\r\n"
        "python collect_telemetry.py\r\n",
        encoding="utf-8",
    )


def _clean_script(script: str) -> str:
    return textwrap.dedent(script).lstrip() + "\n"
