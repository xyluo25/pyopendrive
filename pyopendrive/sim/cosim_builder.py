"""SUMO-CARLA co-simulation script generation."""

from __future__ import annotations

import json
from pathlib import Path

from .config import ScenarioConfig
from .project import make_project_paths


def build_cosim_project(
    project_dir: str | Path | None = None,
    prepared_xodr: str | Path | None = None,
    scenario_config: ScenarioConfig | None = None,
    *,
    dry_run: bool = False,
) -> dict[str, object] | None:
    """Create helper scripts for CARLA's SUMO synchronization workflow."""

    if project_dir is None or prepared_xodr is None:
        return None

    scenario = scenario_config or ScenarioConfig()
    paths = make_project_paths(project_dir)
    paths.cosim_dir.mkdir(parents=True, exist_ok=True)
    (paths.cosim_dir / "outputs").mkdir(parents=True, exist_ok=True)

    commands = cosim_commands(scenario)
    write_cosim_scripts(paths.cosim_dir, scenario, commands)
    ego_metadata_path = paths.cosim_dir / "ego_vehicles.json"
    ego_metadata_path.write_text(
        json.dumps(
            {
                "enabled": scenario.ego.enabled,
                "mode": scenario.ego.mode,
                "vehicle_ids": scenario.ego.vehicle_ids,
                "highlight_enabled": scenario.ego.highlight_enabled,
                "highlight_color": scenario.ego.highlight_color,
                "carla_actor_ids": scenario.ego.carla_actor_ids,
                "carla_role_names": scenario.ego.carla_role_names,
                "carla_type_ids": scenario.ego.carla_type_ids,
                "collect_only_ego": scenario.ego.collect_only_ego,
                "analyze_only_ego": scenario.ego.analyze_only_ego,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    plan = {
        "scenario_id": scenario.scenario_id,
        "dry_run": dry_run,
        "commands": commands,
        "ego_vehicle_ids": scenario.ego.vehicle_ids,
        "files": {
            "create_sumo_vtypes": str(paths.cosim_dir / "create_sumo_vtypes.sh"),
            "netconvert_carla": str(paths.cosim_dir / "netconvert_carla.sh"),
            "run_synchronization": str(paths.cosim_dir / "run_synchronization.sh"),
            "ego_vehicles": str(ego_metadata_path),
        },
    }
    (paths.cosim_dir / "build_cosim_plan.json").write_text(
        json.dumps(plan, indent=2),
        encoding="utf-8",
    )
    return plan


def cosim_commands(scenario_config: ScenarioConfig) -> dict[str, list[str]]:
    """Return the planned CARLA co-simulation helper commands."""

    carla = scenario_config.carla
    cosim = scenario_config.cosim
    create_vtypes = [
        "python",
        "$CARLA_HOME/Co-Simulation/Sumo/util/create_sumo_vtypes.py",
        "--carla-host",
        carla.host,
        "--carla-port",
        str(carla.port),
        "--output-file",
        "../sumo/carlavtypes.rou.xml",
    ]
    convert_network = [
        "python",
        "$CARLA_HOME/Co-Simulation/Sumo/util/netconvert_carla.py",
        "../network/prepared.xodr",
        "--output",
        "../sumo/network_carla_sync.net.xml",
    ]
    if cosim.guess_tls:
        convert_network.append("--guess-tls")

    run_sync = [
        "python",
        "$CARLA_HOME/Co-Simulation/Sumo/run_synchronization.py",
        "../sumo/scenario.sumocfg",
    ]
    if cosim.use_sumo_gui:
        run_sync.append("--sumo-gui")
    run_sync.extend(
        [
            "--step-length",
            str(cosim.step_length),
            "--tls-manager",
            cosim.tls_manager,
        ]
    )
    if cosim.sync_vehicle_all:
        run_sync.append("--sync-vehicle-all")

    return {
        "create_sumo_vtypes": create_vtypes,
        "netconvert_carla": convert_network,
        "run_synchronization": run_sync,
    }


def write_cosim_scripts(
    cosim_dir: str | Path,
    scenario_config: ScenarioConfig,
    commands: dict[str, list[str]],
) -> None:
    """Write shell and Windows scripts for each co-simulation helper command."""

    directory = Path(cosim_dir)
    carla_home_default = scenario_config.cosim.carla_home or ""
    command_specs = {
        "create_sumo_vtypes": commands["create_sumo_vtypes"],
        "netconvert_carla": commands["netconvert_carla"],
        "run_synchronization": commands["run_synchronization"],
    }
    for script_name, command in command_specs.items():
        (directory / f"{script_name}.sh").write_text(
            _shell_script(command, carla_home_default),
            encoding="utf-8",
        )
        (directory / f"{script_name}.bat").write_text(
            _batch_script(command, carla_home_default),
            encoding="utf-8",
        )


def _shell_script(command: list[str], carla_home_default: str) -> str:
    command_text = " ".join(_shell_quote(part) for part in command)
    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        'cd "$(dirname "$0")"\n'
        f'CARLA_HOME="${{CARLA_HOME:-{carla_home_default}}}"\n'
        'if [ -z "$CARLA_HOME" ]; then\n'
        '  echo "Set CARLA_HOME to the CARLA installation before running this script."\n'
        "  exit 1\n"
        "fi\n"
        f"{command_text}\n"
    )


def _batch_script(command: list[str], carla_home_default: str) -> str:
    windows_command = " ".join(
        _batch_quote(part.replace("$CARLA_HOME", "%CARLA_HOME%")) for part in command
    )
    return (
        "@echo off\r\n"
        "cd /d %~dp0\r\n"
        f"if not defined CARLA_HOME set CARLA_HOME={carla_home_default}\r\n"
        "if not defined CARLA_HOME (\r\n"
        "  echo Set CARLA_HOME to the CARLA installation before running this script.\r\n"
        "  exit /b 1\r\n"
        ")\r\n"
        f"{windows_command}\r\n"
    )


def _shell_quote(value: str) -> str:
    if value.startswith("$CARLA_HOME"):
        return '"' + value + '"'
    if value.startswith("../") or value.startswith("--"):
        return value
    if " " not in value:
        return value
    return "'" + value.replace("'", "'\"'\"'") + "'"


def _batch_quote(value: str) -> str:
    if value.startswith("--"):
        return value
    if "%CARLA_HOME%" in value or " " in value:
        return f'"{value}"'
    return value
