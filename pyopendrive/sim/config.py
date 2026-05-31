"""Scenario configuration models for SUMO and CARLA simulation workflows.

The classes in this module intentionally use only standard-library dataclasses
so that dry-run project generation works before heavy simulator dependencies are
installed. YAML files are supported when PyYAML is available.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields, is_dataclass
import json
from pathlib import Path
from typing import Any


@dataclass
class SimulationTimeConfig:
    """Shared simulation clock settings for SUMO, CARLA, and co-simulation."""

    begin: float = 0.0
    end: float = 3600.0
    step_length: float = 1.0
    carla_fixed_delta_seconds: float = 0.05


@dataclass
class DemandConfig:
    """Traffic demand generation settings."""

    method: str = "random_trips"
    period: float = 1.0
    seed: int = 42
    validate_routes: bool = True


@dataclass
class FleetConfig:
    """Vehicle fleet mix and energy model settings."""

    ev_share: float = 0.0
    ice_share: float = 1.0
    ice_emission_class: str = "HBEFA3/PC_G_EU4"
    ev_emission_class: str = "Energy/unknown"
    max_speed_mps: float = 25.0
    sigma: float = 0.5


@dataclass
class SumoConfig:
    """SUMO network, route, vehicle, and output settings."""

    auto_discover_source_files: bool = True
    use_source_sumocfg_time: bool = True
    assign_vehicle_types_to_source_routes: bool = False
    netconvert_options: list[str] = field(default_factory=list)
    random_trips_options: list[str] = field(default_factory=list)
    write_tripinfo: bool = True
    write_emissions: bool = True
    write_battery: bool = True
    write_fcd: bool = True
    source_net_file: str | None = None
    source_route_file: str | None = None
    source_additional_file: str | None = None
    source_trip_file: str | None = None
    source_sumocfg_file: str | None = None


@dataclass
class CarlaConfig:
    """CARLA OpenDRIVE world generation and client connection settings."""

    host: str = "127.0.0.1"
    port: int = 2000
    timeout: float = 30.0
    vertex_distance: float = 2.0
    max_road_length: float = 50.0
    wall_height: float = 1.0
    additional_width: float = 0.6
    smooth_junctions: bool = True


@dataclass
class CoSimConfig:
    """Settings for CARLA's SUMO synchronization helper scripts."""

    carla_home: str | None = None
    tls_manager: str = "sumo"
    step_length: float = 0.05
    sync_vehicle_all: bool = True
    guess_tls: bool = True
    use_sumo_gui: bool = True


@dataclass
class EgoVehicleConfig:
    """Configuration for one or more ego vehicles.

    The dictionary fields intentionally map directly to SUMO XML attributes so
    users can change accepted SUMO parameters without code changes.
    """

    enabled: bool = False
    mode: str = "select_existing"
    highlight_enabled: bool = True
    highlight_color: str = "0,0,255"
    vehicle_ids: list[str] = field(default_factory=list)
    auto_select_count: int = 0
    selection_strategy: str = "first"
    auto_select_seed: int | None = None
    vehicle_type_id: str = "ego_passenger"
    vtype_attributes: dict[str, str] = field(default_factory=dict)
    vtype_params: dict[str, str] = field(default_factory=dict)
    vehicle_attributes: dict[str, str] = field(default_factory=dict)
    added_vehicles: list[dict[str, Any]] = field(default_factory=list)
    od_pairs: list[dict[str, Any]] = field(default_factory=list)
    route_vehicles: list[dict[str, Any]] = field(default_factory=list)
    vehicles: list[dict[str, Any]] = field(default_factory=list)
    carla_actor_ids: list[int] = field(default_factory=list)
    carla_role_names: list[str] = field(default_factory=list)
    carla_type_ids: list[str] = field(default_factory=list)
    collect_only_ego: bool = True
    analyze_only_ego: bool = True


@dataclass
class ScenarioConfig:
    """Complete configuration for one reproducible simulation scenario."""

    scenario_id: str = "baseline"
    random_seed: int = 42
    simulation: SimulationTimeConfig = field(default_factory=SimulationTimeConfig)
    demand: DemandConfig = field(default_factory=DemandConfig)
    fleet: FleetConfig = field(default_factory=FleetConfig)
    sumo: SumoConfig = field(default_factory=SumoConfig)
    carla: CarlaConfig = field(default_factory=CarlaConfig)
    cosim: CoSimConfig = field(default_factory=CoSimConfig)
    ego: EgoVehicleConfig = field(default_factory=EgoVehicleConfig)
    source_file: str | None = None
    base_dir: Path | None = field(default=None, repr=False, compare=False)


def load_scenario_config(path: str | Path) -> ScenarioConfig:
    """Load a scenario configuration from a JSON, YAML, or YML file.

    Args:
        path: Path to the scenario file.

    Returns:
        A populated :class:`ScenarioConfig`.

    Raises:
        FileNotFoundError: If the configuration file does not exist.
        ValueError: If the file extension or root content is unsupported.
        RuntimeError: If a YAML file is provided and PyYAML is unavailable.
    """

    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(f"Scenario configuration not found: {config_path}")

    suffix = config_path.suffix.lower()
    if suffix == ".json":
        raw_config = json.loads(config_path.read_text(encoding="utf-8"))
    elif suffix in {".yaml", ".yml"}:
        raw_config = _load_yaml_config(config_path)
    else:
        raise ValueError(
            "Scenario files must use .json, .yaml, or .yml extensions. "
            f"Received: {config_path}"
        )

    if raw_config is None:
        raw_config = {}
    if not isinstance(raw_config, dict):
        raise ValueError(
            "Scenario configuration must contain a mapping at the top level. "
            f"Received {type(raw_config).__name__} from {config_path}."
        )

    scenario = scenario_config_from_dict(raw_config)
    scenario.source_file = str(config_path)
    scenario.base_dir = config_path.parent
    return scenario


def scenario_config_from_dict(raw_config: dict[str, Any] | None) -> ScenarioConfig:
    """Create a :class:`ScenarioConfig` from a plain Python dictionary."""

    raw_config = raw_config or {}
    raw_ego_config = raw_config.get("ego", {})
    ego_config = _dataclass_from_mapping(EgoVehicleConfig, raw_ego_config)
    if _should_infer_added_ego_mode(raw_ego_config):
        ego_config.mode = "add"

    scenario = ScenarioConfig(
        scenario_id=str(raw_config.get("scenario_id", "baseline")),
        random_seed=int(raw_config.get("random_seed", 42)),
        simulation=_dataclass_from_mapping(
            SimulationTimeConfig, raw_config.get("simulation", {})
        ),
        demand=_dataclass_from_mapping(DemandConfig, raw_config.get("demand", {})),
        fleet=_dataclass_from_mapping(FleetConfig, raw_config.get("fleet", {})),
        sumo=_dataclass_from_mapping(SumoConfig, raw_config.get("sumo", {})),
        carla=_dataclass_from_mapping(CarlaConfig, raw_config.get("carla", {})),
        cosim=_dataclass_from_mapping(CoSimConfig, raw_config.get("cosim", {})),
        ego=ego_config,
    )
    _normalize_fleet_share(scenario)
    return scenario


def scenario_to_dict(scenario_config: ScenarioConfig) -> dict[str, Any]:
    """Convert a scenario dataclass into JSON-serializable dictionaries."""

    raw = asdict(scenario_config)
    raw.pop("base_dir", None)
    return raw


def resolve_config_path(
    path_value: str | Path | None,
    scenario_config: ScenarioConfig,
) -> Path | None:
    """Resolve a scenario-relative path without assuming the current directory.

    Args:
        path_value: Path value from a scenario file.
        scenario_config: Scenario containing ``base_dir`` from its source file.

    Returns:
        An absolute or current-working-directory-resolved path, or ``None`` when
        ``path_value`` is empty.
    """

    if path_value in {None, ""}:
        return None

    path = Path(path_value)
    if path.is_absolute():
        return path
    if scenario_config.base_dir is not None:
        return (scenario_config.base_dir / path).resolve()
    return path.resolve()


def _load_yaml_config(config_path: Path) -> dict[str, Any] | None:
    try:
        import yaml  # type: ignore
    except ImportError as exc:  # pragma: no cover - depends on environment
        raise RuntimeError(
            "PyYAML is required to read YAML scenario files. Install it with "
            "`pip install PyYAML`, or provide the scenario as JSON."
        ) from exc

    with config_path.open("r", encoding="utf-8") as file_obj:
        return yaml.safe_load(file_obj)


def _dataclass_from_mapping(model_type: type[Any], raw_value: Any) -> Any:
    """Populate a dataclass from known keys and ignore unknown extension keys."""

    if raw_value is None:
        raw_value = {}
    if not isinstance(raw_value, dict):
        raise ValueError(
            f"{model_type.__name__} must be configured with a mapping, "
            f"not {type(raw_value).__name__}."
        )

    values: dict[str, Any] = {
        model_field.name: raw_value[model_field.name]
        for model_field in fields(model_type)
        if model_field.name in raw_value
    }

    instance = model_type(**values)
    if not is_dataclass(instance):
        raise TypeError(f"{model_type.__name__} did not create a dataclass instance.")
    return instance


def _normalize_fleet_share(scenario_config: ScenarioConfig) -> None:
    """Keep ICE and EV shares internally consistent for scenario comparisons."""

    ev_share = max(0.0, min(1.0, float(scenario_config.fleet.ev_share)))
    scenario_config.fleet.ev_share = ev_share
    scenario_config.fleet.ice_share = max(0.0, min(1.0, 1.0 - ev_share))


def _should_infer_added_ego_mode(raw_ego_config: Any) -> bool:
    """Preserve old configs that used ``vehicles`` before explicit modes."""

    if not isinstance(raw_ego_config, dict) or "mode" in raw_ego_config:
        return False

    has_added_vehicle_specs = any(
        raw_ego_config.get(key)
        for key in ["vehicles", "added_vehicles", "od_pairs", "route_vehicles"]
    )
    has_existing_vehicle_specs = bool(
        raw_ego_config.get("vehicle_ids") or raw_ego_config.get("auto_select_count")
    )
    return has_added_vehicle_specs and not has_existing_vehicle_specs
