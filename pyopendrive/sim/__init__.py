# -*- coding:utf-8 -*-
##############################################################
# Created Date: Friday, May 29th 2026
# Contact Info: luoxiangyong01@gmail.com
# Author/Copyright: Mr. Xiangyong Luo
##############################################################


from .build_carla_project import build_carla_project
from .build_cosim_project import build_cosim_project
from .build_sumo_project import build_sumo_project
from .config import EgoVehicleConfig, ScenarioConfig, load_scenario_config
from .project import prepare_opendrive_network

_LAZY_EXPORT_MODULES = {
    "analyze_project": ".analysis",
    "read_carla_telemetry": ".analysis",
    "read_tripinfo": ".analysis",
    "summarize_carla_telemetry": ".analysis",
    "summarize_tripinfo": ".analysis",
    "write_result_figures": ".analysis",
    "write_summary_csv": ".analysis",
    "analyze_carla_telemetry": ".analyze_carla_telemetry",
    "analyze_sumo_results": ".analyze_sumo_results",
    "compare_scenarios": ".compare_scenarios",
}

__all__ = [
    "ScenarioConfig",
    "EgoVehicleConfig",
    "load_scenario_config",
    "prepare_opendrive_network",
    "build_sumo_project",
    "build_carla_project",
    "build_cosim_project",
    "analyze_sumo_results",
    "analyze_carla_telemetry",
    "analyze_project",
    "read_tripinfo",
    "summarize_tripinfo",
    "read_carla_telemetry",
    "summarize_carla_telemetry",
    "write_result_figures",
    "write_summary_csv",
    "compare_scenarios",
]


def __getattr__(name: str) -> object:
    """Load analysis exports only when the caller asks for them."""
    if name not in _LAZY_EXPORT_MODULES:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

    from importlib import import_module

    module = import_module(_LAZY_EXPORT_MODULES[name], __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
