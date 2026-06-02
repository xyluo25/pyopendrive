"""Command-line interface for PyOpenDRIVE SUMO/CARLA simulation workflows."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

from .carla_builder import build_carla_project
from .config import ScenarioConfig, load_scenario_config, scenario_to_dict
from .cosim_builder import build_cosim_project
from .project import (
    create_project_directories,
    prepare_opendrive_network,
    write_project_config,
)
from .sumo_builder import (
    build_sumo_project,
    find_sumo_tool,
    sanitize_sumo_project_additional_files,
)


def build_sim_project(
    *,
    xodr_path: str | Path,
    out_dir: str | Path,
    scenario_path: str | Path | None = None,
    build_sumo: bool = False,
    build_carla: bool = False,
    build_cosim: bool = False,
    carla_home: str | None = None,
    dry_run: bool = False,
    overwrite: bool = False,
) -> dict[str, object]:
    """Build a reproducible simulation project from one OpenDRIVE network."""

    scenario = (
        load_scenario_config(scenario_path) if scenario_path else ScenarioConfig()
    )
    if carla_home:
        scenario.cosim.carla_home = carla_home
    dependency_messages = dependency_messages_for_build(
        scenario,
        build_sumo=build_sumo,
        build_carla=build_carla,
        build_cosim=build_cosim,
    )

    paths = create_project_directories(out_dir, overwrite=overwrite)
    prepared_xodr = prepare_opendrive_network(xodr_path, paths.root, overwrite=True)
    write_project_config(
        paths.root,
        scenario,
        xodr_path=xodr_path,
        build_sumo=build_sumo,
        build_carla=build_carla,
        build_cosim=build_cosim,
        dry_run=dry_run,
    )

    build_results: dict[str, object] = {
        "project_dir": str(paths.root),
        "prepared_xodr": str(prepared_xodr),
        "scenario": scenario_to_dict(scenario),
        "dependency_messages": dependency_messages,
    }
    if build_sumo:
        sumo_result = build_sumo_project(
            paths.root,
            prepared_xodr,
            scenario,
            dry_run=dry_run,
        )
        build_results["sumo"] = sumo_result
        if isinstance(sumo_result, dict):
            ego_vehicle_ids = sumo_result.get("ego_vehicle_ids")
            if isinstance(ego_vehicle_ids, list):
                scenario.ego.vehicle_ids = [
                    str(vehicle_id) for vehicle_id in ego_vehicle_ids
                ]
    if build_carla:
        build_results["carla"] = build_carla_project(
            paths.root,
            prepared_xodr,
            scenario,
            dry_run=dry_run,
        )
    if build_cosim:
        build_results["cosim"] = build_cosim_project(
            paths.root,
            prepared_xodr,
            scenario,
            dry_run=dry_run,
        )
    build_results["scenario"] = scenario_to_dict(scenario)
    write_project_config(
        paths.root,
        scenario,
        xodr_path=xodr_path,
        build_sumo=build_sumo,
        build_carla=build_carla,
        build_cosim=build_cosim,
        dry_run=dry_run,
    )
    return build_results


def main(argv: list[str] | None = None) -> int:
    """Run the ``pyopendrive-sim`` command-line interface."""

    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "build":
            result = build_sim_project(
                xodr_path=args.xodr,
                out_dir=args.out,
                scenario_path=args.scenario,
                build_sumo=args.build_sumo,
                build_carla=args.build_carla,
                build_cosim=args.build_cosim,
                carla_home=args.carla_home,
                dry_run=args.dry_run,
                overwrite=args.overwrite,
            )
            _print_build_next_steps(result, args)
            return 0
        if args.command == "analyze":
            from .analysis import analyze_project

            outputs = analyze_project(
                project_dir=args.project,
                out_dir=args.out,
                scenario_id=args.scenario,
                tripinfo_path=args.tripinfo,
                carla_telemetry_path=args.carla_telemetry,
                skip_carla=args.sumo_only,
                ego_vehicle_ids=args.ego_vehicle_id,
            )
            print("Analysis files written:")
            for name, path in outputs.items():
                print(f"  {name}: {path}")
            return 0
        if args.command == "sumo-only":
            result = run_sumo_only_workflow(
                xodr_path=args.xodr,
                out_dir=args.out,
                scenario_path=args.scenario,
                gui=args.gui,
                dry_run=args.dry_run,
                overwrite=args.overwrite,
                analysis_out=args.analysis_out,
            )
            _print_sumo_only_next_steps(result, args)
            return 0
        if args.command == "run-sumo":
            run_sumo(args.project, gui=args.gui, extra_args=args.extra_args)
            return 0
        if args.command == "run-carla":
            run_carla(args.project)
            return 0
        if args.command == "run-cosim":
            run_cosim(args.project)
            return 0
    except Exception as exc:
        print(f"pyopendrive-sim error: {exc}", file=sys.stderr)
        return 1

    parser.print_help()
    return 0


def run_sumo_only_workflow(
    *,
    xodr_path: str | Path,
    out_dir: str | Path,
    scenario_path: str | Path | None = None,
    gui: bool = False,
    dry_run: bool = False,
    overwrite: bool = False,
    analysis_out: str | Path | None = None,
) -> dict[str, object]:
    """Build, run, and analyze a SUMO-only project from OpenDRIVE.

    CARLA is not imported, contacted, or analyzed by this workflow.
    """

    if not dry_run:
        _raise_for_missing_dependencies(dependency_messages_for_sumo_run(gui=gui))

    result = build_sim_project(
        xodr_path=xodr_path,
        out_dir=out_dir,
        scenario_path=scenario_path,
        build_sumo=True,
        build_carla=False,
        build_cosim=False,
        dry_run=dry_run,
        overwrite=overwrite,
    )
    if dry_run:
        result["analysis"] = "skipped_dry_run"
        return result

    run_sumo(out_dir, gui=gui)
    analysis_dir = Path(analysis_out) if analysis_out else Path(out_dir) / "analysis"
    from .analysis import analyze_project

    result["analysis"] = {
        name: str(path)
        for name, path in analyze_project(
            project_dir=out_dir,
            out_dir=analysis_dir,
            skip_carla=True,
        ).items()
    }
    return result


def run_sumo(
    project_dir: str | Path,
    *,
    gui: bool = False,
    extra_args: list[str] | None = None,
    clean_outputs: bool = True,
) -> None:
    """Run the generated SUMO scenario and record the command."""

    project_path = Path(project_dir)
    sumo_dir = project_path / "sumo"
    config_path = sumo_dir / "scenario.sumocfg"
    if not config_path.exists():
        raise FileNotFoundError(f"SUMO config not found: {config_path}")

    _raise_for_missing_dependencies(dependency_messages_for_sumo_run(gui=gui))
    sanitization = sanitize_sumo_project_additional_files(sumo_dir)
    removed_count = int(sanitization.get("removed_count", 0))
    if removed_count:
        print(
            "Sanitized SUMO additional files: removed "
            f"{removed_count} lane-based element(s) that referenced missing lanes."
        )
    executable_name = "sumo-gui" if gui else "sumo"
    executable_path = find_sumo_tool(executable_name)
    if executable_path is None:
        raise RuntimeError(
            f"{executable_name} was not found. Install SUMO and add its bin "
            "folder to PATH, or set SUMO_HOME."
        )

    command = [str(executable_path), "-c", "scenario.sumocfg"]
    command.extend(extra_args or [])
    output_dir = sumo_dir / "outputs"
    output_dir.mkdir(parents=True, exist_ok=True)
    if clean_outputs:
        clean_sumo_outputs(sumo_dir)
    (output_dir / "run_sumo_command.json").write_text(
        json.dumps({"command": command}, indent=2),
        encoding="utf-8",
    )
    subprocess.run(command, cwd=sumo_dir, check=True)


def clean_sumo_outputs(sumo_dir: str | Path) -> None:
    """Remove known generated SUMO outputs before starting a new run."""

    directory = Path(sumo_dir)
    output_dir = directory / "outputs"
    output_names = [
        "tripinfo.xml",
        "emissions.xml",
        "battery.xml",
        "fcd.xml",
        "summary.xml",
        "statistics.xml",
        "run_sumo_command.json",
    ]
    for output_name in output_names:
        output_path = output_dir / output_name
        if output_path.exists() and output_path.is_file():
            output_path.unlink()

    for detector_output in directory.glob("output_*.xml"):
        if detector_output.is_file():
            detector_output.unlink()
    null_output = directory / "NULL"
    if null_output.exists() and null_output.is_file():
        null_output.unlink()


def run_carla(project_dir: str | Path) -> None:
    """Run the generated CARLA OpenDRIVE loading script."""

    carla_dir = Path(project_dir) / "carla"
    script_path = carla_dir / "load_opendrive_world.py"
    if not script_path.exists():
        raise FileNotFoundError(f"CARLA load script not found: {script_path}")
    _raise_for_missing_dependencies(dependency_messages_for_carla_run())
    subprocess.run([sys.executable, str(script_path.name)], cwd=carla_dir, check=True)


def run_cosim(project_dir: str | Path) -> None:
    """Run the generated co-simulation synchronization script."""

    cosim_dir = Path(project_dir) / "cosim"
    script_path = (
        cosim_dir / "run_synchronization.bat"
        if os.name == "nt"
        else cosim_dir / "run_synchronization.sh"
    )
    if not script_path.exists():
        raise FileNotFoundError(f"Co-simulation script not found: {script_path}")
    _raise_for_missing_dependencies(dependency_messages_for_cosim_run(project_dir))
    command = (
        ["cmd", "/c", str(script_path)]
        if os.name == "nt"
        else ["bash", str(script_path)]
    )
    subprocess.run(command, cwd=cosim_dir, check=True)


def dependency_messages_for_build(
    scenario_config: ScenarioConfig,
    *,
    build_sumo: bool,
    build_carla: bool,
    build_cosim: bool,
) -> list[str]:
    """Return non-fatal dependency messages for project generation commands."""

    messages: list[str] = []
    if build_sumo:
        messages.extend(
            _sumo_dependency_messages(
                ["sumo", "netconvert", "randomTrips.py"],
                purpose="building or running SUMO projects",
                required_now=False,
            )
        )
    if build_carla:
        messages.extend(_carla_python_dependency_messages(required_now=False))
    if build_cosim:
        sumo_tool = "sumo-gui" if scenario_config.cosim.use_sumo_gui else "sumo"
        messages.extend(
            _sumo_dependency_messages(
                [sumo_tool],
                purpose="running SUMO-CARLA co-simulation",
                required_now=False,
            )
        )
        messages.extend(
            _carla_home_dependency_messages(
                scenario_config.cosim.carla_home,
                required_now=False,
            )
        )
    return messages


def dependency_messages_for_sumo_run(*, gui: bool = False) -> list[str]:
    """Return fatal dependency messages for running a generated SUMO scenario."""

    executable_name = "sumo-gui" if gui else "sumo"
    return _sumo_dependency_messages(
        [executable_name],
        purpose="running SUMO",
        required_now=True,
    )


def dependency_messages_for_carla_run() -> list[str]:
    """Return fatal dependency messages for running generated CARLA scripts."""

    return _carla_python_dependency_messages(required_now=True)


def dependency_messages_for_cosim_run(project_dir: str | Path) -> list[str]:
    """Return fatal dependency messages for running SUMO-CARLA co-simulation."""

    project_config = _read_project_config(project_dir)
    scenario = project_config.get("scenario", {})
    cosim = scenario.get("cosim", {}) if isinstance(scenario, dict) else {}
    use_sumo_gui = True
    carla_home = None
    if isinstance(cosim, dict):
        use_sumo_gui = bool(cosim.get("use_sumo_gui", True))
        carla_home = cosim.get("carla_home")

    sumo_tool = "sumo-gui" if use_sumo_gui else "sumo"
    messages = _sumo_dependency_messages(
        [sumo_tool],
        purpose="running SUMO-CARLA co-simulation",
        required_now=True,
    )
    messages.extend(_carla_home_dependency_messages(carla_home, required_now=True))
    return messages


def _sumo_dependency_messages(
    tool_names: list[str],
    *,
    purpose: str,
    required_now: bool,
) -> list[str]:
    missing_tools = [
        tool_name for tool_name in tool_names if find_sumo_tool(tool_name) is None
    ]
    if not missing_tools:
        return []

    urgency = (
        "required now"
        if required_now
        else "needed before this generated workflow can be run"
    )
    return [
        "Dependency check: SUMO is missing on this operating system. "
        f"Missing tool(s) for {purpose}: {', '.join(missing_tools)}. "
        f"These tool(s) are {urgency}. Install Eclipse SUMO and add its bin "
        "folder to PATH, or set SUMO_HOME so PyOpenDRIVE can find SUMO."
    ]


def _carla_python_dependency_messages(*, required_now: bool) -> list[str]:
    if _python_module_available("carla"):
        return []

    urgency = "required now" if required_now else "needed before running CARLA"
    return [
        "Dependency check: CARLA Python API is missing on this operating system. "
        f"The `carla` Python module is {urgency}. Add CARLA/PythonAPI/carla to "
        "PYTHONPATH or install a matching `carla` package for your CARLA server."
    ]


def _carla_home_dependency_messages(
    configured_carla_home: object,
    *,
    required_now: bool,
) -> list[str]:
    raw_carla_home = str(configured_carla_home or os.environ.get("CARLA_HOME") or "")
    if not raw_carla_home:
        urgency = "required now" if required_now else "needed before co-simulation"
        return [
            "Dependency check: CARLA_HOME is missing on this operating system. "
            f"CARLA_HOME is {urgency} for SUMO-CARLA co-simulation. Set "
            "--carla-home or the CARLA_HOME environment variable to the CARLA "
            "installation folder."
        ]

    carla_home = Path(raw_carla_home)
    if not carla_home.exists():
        return [
            "Dependency check: CARLA_HOME does not exist on this operating system. "
            f"Configured path: {carla_home}. Update --carla-home or CARLA_HOME."
        ]

    missing_helpers = [
        helper_path
        for helper_path in [
            carla_home / "Co-Simulation" / "Sumo" / "util" / "create_sumo_vtypes.py",
            carla_home / "Co-Simulation" / "Sumo" / "util" / "netconvert_carla.py",
            carla_home / "Co-Simulation" / "Sumo" / "run_synchronization.py",
        ]
        if not helper_path.exists()
    ]
    if not missing_helpers:
        return []

    return [
        "Dependency check: CARLA co-simulation helper scripts are missing. "
        "Expected under CARLA_HOME/Co-Simulation/Sumo. Missing file(s): "
        + ", ".join(str(path) for path in missing_helpers)
    ]


def _raise_for_missing_dependencies(messages: list[str]) -> None:
    if messages:
        raise RuntimeError("\n".join(messages))


def _print_dependency_messages(result: dict[str, object]) -> None:
    messages = result.get("dependency_messages", [])
    if not isinstance(messages, list) or not messages:
        return

    for message in messages:
        print(message, file=sys.stderr)


def _python_module_available(module_name: str) -> bool:
    return importlib.util.find_spec(module_name) is not None


def _read_project_config(project_dir: str | Path) -> dict[str, object]:
    config_path = Path(project_dir) / "project_config.json"
    if not config_path.exists():
        return {}
    try:
        payload = json.loads(config_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    return payload if isinstance(payload, dict) else {}


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="pyopendrive-sim",
        description="Build and analyze OpenDRIVE-based SUMO/CARLA simulation projects.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    build_parser = subparsers.add_parser("build", help="Build a simulation project.")
    build_parser.add_argument(
        "--xodr", required=True, help="Input OpenDRIVE .xodr file."
    )
    build_parser.add_argument("--out", required=True, help="Output project directory.")
    build_parser.add_argument("--scenario", help="Scenario JSON/YAML file.")
    build_parser.add_argument("--build-sumo", action="store_true")
    build_parser.add_argument("--build-carla", action="store_true")
    build_parser.add_argument("--build-cosim", action="store_true")
    build_parser.add_argument("--carla-home", help="CARLA installation path.")
    build_parser.add_argument("--dry-run", action="store_true")
    build_parser.add_argument("--overwrite", action="store_true")

    analyze_parser = subparsers.add_parser(
        "analyze", help="Analyze simulation outputs."
    )
    analyze_parser.add_argument("--project", help="Project directory to inspect.")
    analyze_parser.add_argument(
        "--out", required=True, help="Analysis output directory."
    )
    analyze_parser.add_argument("--tripinfo", help="Direct SUMO tripinfo.xml path.")
    analyze_parser.add_argument(
        "--carla-telemetry", help="Direct CARLA telemetry CSV path."
    )
    analyze_parser.add_argument(
        "--scenario", help="Scenario id for direct-file analysis."
    )
    analyze_parser.add_argument(
        "--sumo-only",
        action="store_true",
        help="Analyze SUMO outputs only and ignore CARLA telemetry.",
    )
    analyze_parser.add_argument(
        "--ego-vehicle-id",
        action="append",
        help="Analyze only this ego vehicle id. Repeat for multiple vehicles.",
    )

    sumo_only_parser = subparsers.add_parser(
        "sumo-only",
        help="Build from OpenDRIVE, run SUMO, and analyze SUMO outputs only.",
    )
    sumo_only_parser.add_argument(
        "--xodr", required=True, help="Input OpenDRIVE .xodr file."
    )
    sumo_only_parser.add_argument(
        "--out", required=True, help="Output project directory."
    )
    sumo_only_parser.add_argument("--scenario", help="Scenario JSON/YAML file.")
    sumo_only_parser.add_argument(
        "--analysis-out", help="Analysis output directory. Defaults to OUT/analysis."
    )
    sumo_only_parser.add_argument("--gui", action="store_true")
    sumo_only_parser.add_argument("--dry-run", action="store_true")
    sumo_only_parser.add_argument("--overwrite", action="store_true")

    run_sumo_parser = subparsers.add_parser("run-sumo", help="Run generated SUMO case.")
    run_sumo_parser.add_argument("--project", required=True)
    run_sumo_parser.add_argument("--gui", action="store_true")
    run_sumo_parser.add_argument("extra_args", nargs=argparse.REMAINDER)

    run_carla_parser = subparsers.add_parser(
        "run-carla", help="Load CARLA OpenDRIVE world."
    )
    run_carla_parser.add_argument("--project", required=True)

    run_cosim_parser = subparsers.add_parser("run-cosim", help="Run SUMO-CARLA sync.")
    run_cosim_parser.add_argument("--project", required=True)
    return parser


def _print_build_next_steps(
    result: dict[str, object], args: argparse.Namespace
) -> None:
    _print_dependency_messages(result)
    project_dir = result["project_dir"]
    print(f"Project built: {project_dir}")
    if args.build_sumo:
        print(f"Run SUMO: pyopendrive-sim run-sumo --project {project_dir}")
    if args.build_carla:
        print(f"Load CARLA world: pyopendrive-sim run-carla --project {project_dir}")
    if args.build_cosim:
        print(f"Run co-simulation: pyopendrive-sim run-cosim --project {project_dir}")
    print(
        "Analyze outputs: "
        f"pyopendrive-sim analyze --project {project_dir} --out {project_dir}/analysis"
    )


def _print_sumo_only_next_steps(
    result: dict[str, object], args: argparse.Namespace
) -> None:
    _print_dependency_messages(result)
    project_dir = result["project_dir"]
    if args.dry_run:
        print(f"SUMO-only dry run built: {project_dir}")
        print("Rebuild without --dry-run to execute SUMO.")
        print(
            "After a real SUMO run, analyze outputs with: "
            f"pyopendrive-sim analyze --project {project_dir} "
            f"--out {project_dir}/analysis --sumo-only"
        )
        return
    print(f"SUMO-only workflow completed: {project_dir}")
    print(f"Analysis directory: {args.analysis_out or Path(project_dir) / 'analysis'}")


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
