# AGENTS.md

Project: PyOpenDRIVE TRB SUMO-CARLA Mobility-Energy Workflow
Repository branch: `xyluo25/pyopendrive`, branch `maplibregl`
Primary goal: implement a reproducible Transportation Research Board style workflow that uses PyOpenDRIVE as the OpenDRIVE network preparation layer, SUMO as the network-level mobility and energy simulation engine, and CARLA as the high-fidelity 3D simulation and telemetry engine.

paper title: OpenDRIVE-Based SUMO-CARLA Co-Simulation for Mobility and Energy Assessment in Urban Transportation Networks

## How Codex should use this file

Read this file before making code changes. It converts the TRB paper plan into implementation tasks. Treat each numbered section as both a paper requirement and a software requirement.

When implementing, prefer small, reviewable commits and keep the current PyOpenDRIVE API backward compatible. Do not remove existing public functions unless explicitly asked. Add type hints, docstrings, comments, and clear error messages. Write code that a non-specialist reviewer can understand.

## Key external references

Use these official references when implementing or checking assumptions:

- Codex custom instructions with `AGENTS.md`: https://developers.openai.com/codex/guides/agents-md
- Codex best practices for `AGENTS.md`: https://developers.openai.com/codex/learn/best-practices
- SUMO `netconvert`: https://sumo.dlr.de/docs/netconvert.html
- SUMO TripInfo output: https://sumo.dlr.de/docs/Simulation/Output/TripInfo.html
- SUMO emission output: https://sumo.dlr.de/docs/Simulation/Output/EmissionOutput.html
- SUMO electric vehicle model: https://sumo.dlr.de/docs/Models/Electric.html
- CARLA OpenDRIVE standalone mode: https://carla.readthedocs.io/en/latest/adv_opendrive/
- CARLA SUMO co-simulation: https://carla.readthedocs.io/en/latest/adv_sumo/
- CARLA synchrony and fixed time-step: https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/
- CARLA Python API: https://carla.readthedocs.io/en/latest/python_api/

## Engineering conventions

1. Use Python standard library first. Add new dependencies only when they are clearly necessary.
2. If `pandas`, `matplotlib`, or `PyYAML` already exist in project dependencies, they may be used. If not, keep them optional and provide graceful fallbacks.
3. Avoid `geopandas` unless the user explicitly requests it. Prefer plain GeoJSON dictionaries and simple coordinate arrays for portability.
4. Use `pathlib.Path` for all file paths.
5. Use `subprocess.run(..., check=True, capture_output=True, text=True)` for external SUMO and CARLA helper commands.
6. Provide `--dry-run` mode for build commands. Dry run should create planned command files and configuration files without requiring SUMO or CARLA to be installed.
7. Keep CARLA-dependent tests optional. Skip them when `carla` Python API or a CARLA server is unavailable.
8. Keep SUMO-dependent tests optional. Skip them when `sumo`, `netconvert`, or `SUMO_HOME` is unavailable.
9. Do not hard-code local machine paths. Use CLI arguments, environment variables, or config files.
10. Never include credentials, tokens, private server paths, or unpublished external source code.

## Expected repository additions

Create a package area for simulation workflow code:

```text
pyopendrive/sim/
    __init__.py
    config.py
    project.py
    sumo_builder.py
    carla_builder.py
    cosim_builder.py
    analysis.py
    cli.py
```

Create examples and paper-supporting artifacts:

```text
pyopendrive/paper/examples/sumo_carla_energy/
    README.md
    configs/
        scenario_baseline.yaml
        scenario_ev25.yaml
        scenario_ev50.yaml
        scenario_ev100.yaml
        scenario_eco_speed.yaml
        scenario_signal_optimized.yaml
    scripts/
        01_prepare_network.py
        02_build_sumo_case.py
        03_build_carla_case.py
        04_run_sumo.py
        05_run_carla_telemetry.py
        06_run_sumo_carla_cosim.py
        07_analyze_results.py
    sample_outputs/
        tripinfo_sample.xml
        carla_telemetry_sample.csv
        scenario_summary_sample.csv
```

Create tests:

```text
tests/sim/
    test_config.py
    test_sumo_builder.py
    test_analysis_tripinfo.py
    test_analysis_carla.py
    test_cli.py
```

Update packaging if appropriate:

```toml
[project.scripts]
pyopendrive-sim = "pyopendrive.sim.cli:main"
```

If the project uses another CLI pattern, follow the existing pattern instead of forcing this exact layout.

---

# Step 2. Main paper idea converted to software objective

## Paper idea

Build a TRB-style study around an OpenDRIVE-centered workflow for joint mobility and energy simulation. PyOpenDRIVE prepares and validates the lane-level road network. SUMO runs network-scale mobility, emissions, and EV battery simulation. CARLA runs high-fidelity 3D simulation and vehicle telemetry.

## Implementation objective

Implement a reproducible workflow that can:

1. Load an OpenDRIVE `.xodr` file with PyOpenDRIVE.
2. Inspect and validate core network elements.
3. Generate a SUMO project from the same `.xodr` file.
4. Generate a CARLA OpenDRIVE standalone project from the same `.xodr` file.
5. Generate optional SUMO-CARLA co-simulation commands.
6. Run SUMO simulations when SUMO is available.
7. Collect CARLA telemetry when CARLA is available.
8. Analyze mobility, energy, emission, and trajectory outputs.
9. Export tables and figures for a TRB paper.

## Minimum deliverable

A user should be able to run:

```bash
pyopendrive-sim build \
  --xodr datasets/chatt.xodr \
  --out experiments/chatt_trb \
  --scenario examples/trb_sumo_carla_energy/configs/scenario_ev50.yaml \
  --build-sumo \
  --build-carla \
  --build-cosim \
  --dry-run

pyopendrive-sim analyze \
  --project experiments/chatt_trb \
  --out experiments/chatt_trb/analysis
```

Dry run must work without SUMO or CARLA installed.

---

# Step 3. Proposed contribution statement converted to feature requirements

## Contribution 1: OpenDRIVE-centered network preparation

Implement a network preparation routine that:

- Reads an input `.xodr` file.
- Prints or saves a network summary with roads, junctions, lanes, lane sections, signals, objects, and routing edges when available.
- Saves a cleaned or copied `.xodr` file into the experiment project.
- Optionally produces a GeoJSON summary for web visualization.

Suggested function:

```python
def prepare_opendrive_network(xodr_path: Path, project_dir: Path) -> Path:
    """Load, summarize, validate, and copy or save an OpenDRIVE network.

    Returns the path to the experiment-ready OpenDRIVE file.
    """
```

## Contribution 2: Dual-resolution simulation generation

Implement builders for:

- SUMO network and route files.
- CARLA OpenDRIVE standalone run scripts.
- SUMO-CARLA co-simulation configuration and command scripts.

Suggested classes or functions:

```python
class SumoProjectBuilder:
    def build(self) -> None: ...

class CarlaProjectBuilder:
    def build(self) -> None: ...

class CoSimProjectBuilder:
    def build(self) -> None: ...
```

Use classes only if they simplify state management. Otherwise, use clear module-level functions with typed config objects.

## Contribution 3: Mobility-energy evaluation

Implement analysis utilities that summarize:

- Travel time.
- Waiting time.
- Time loss.
- Throughput.
- Stop count.
- Fuel consumption.
- Electricity consumption.
- Battery state and regenerated energy when available.
- CARLA speed, acceleration, and jerk profiles.

---

# Step 4. Research questions converted to output checks

The implementation should generate data that can answer these research questions.

## RQ1. Can one OpenDRIVE-centered workflow generate consistent SUMO and CARLA simulations from the same lane-level road network?

Required outputs:

- `network_summary.json`
- `sumo/chatt.net.xml` or equivalent SUMO network file
- `carla/load_opendrive_world.py`
- `cosim/run_synchronization.sh` or `.bat`
- Optional `consistency_summary.csv`

## RQ2. How do traffic control, demand, and EV penetration scenarios affect mobility and energy?

Required outputs:

- `analysis/scenario_summary.csv`
- `analysis/mobility_energy_table.csv`
- `analysis/scenario_comparison.csv`
- Optional `analysis/statistical_tests.csv`

## RQ3. What additional insights are obtained by coupling SUMO network-scale outputs with CARLA high-fidelity telemetry?

Required outputs:

- `carla/carla_telemetry.csv`
- `analysis/carla_telemetry_summary.csv`
- Optional `analysis/sumo_carla_trace_comparison.csv`

---

# Step 5. Methodology architecture converted to workflow

Implement this workflow in code and documentation:

```text
OpenDRIVE network
      |
      v
PyOpenDRIVE inspection, editing, validation
      |
      +-------------------------+
      |                         |
      v                         v
SUMO project generation      CARLA project generation
.net.xml, .rou.xml,          OpenDRIVE standalone world
.sumocfg, vTypes             3D actors, sensors, telemetry
      |                         |
      +-----------+-------------+
                  |
                  v
          Optional SUMO-CARLA co-simulation
                  |
                  v
       Mobility, energy, and trajectory outputs
                  |
                  v
       Scenario comparison and TRB tables or figures
```

## Project directory layout produced by `pyopendrive-sim build`

```text
experiments/<project_name>/
    project_config.json
    network/
        input.xodr
        prepared.xodr
        network_summary.json
    sumo/
        network.net.xml
        routes.rou.xml
        vtypes.add.xml
        scenario.sumocfg
        run_sumo.sh
        run_sumo.bat
        outputs/
    carla/
        load_opendrive_world.py
        collect_telemetry.py
        run_carla_client.sh
        run_carla_client.bat
        outputs/
    cosim/
        create_sumo_vtypes.sh
        create_sumo_vtypes.bat
        netconvert_carla.sh
        netconvert_carla.bat
        run_synchronization.sh
        run_synchronization.bat
        outputs/
    analysis/
        scenario_summary.csv
        mobility_energy_table.csv
        figures/
```

---

# Step 6. Additions recommended for PyOpenDRIVE

## New module: `pyopendrive/sim/config.py`

Create typed config models using `dataclasses`.

Recommended dataclasses:

```python
@dataclass
class SimulationTimeConfig:
    begin: float = 0.0
    end: float = 3600.0
    step_length: float = 1.0
    carla_fixed_delta_seconds: float = 0.05

@dataclass
class DemandConfig:
    method: str = "random_trips"
    period: float = 1.0
    seed: int = 42
    validate_routes: bool = True

@dataclass
class FleetConfig:
    ev_share: float = 0.0
    ice_share: float = 1.0
    ice_emission_class: str = "HBEFA3/PC_G_EU4"
    ev_emission_class: str = "Energy/unknown"

@dataclass
class SumoConfig:
    netconvert_options: list[str] = field(default_factory=list)
    random_trips_options: list[str] = field(default_factory=list)
    write_tripinfo: bool = True
    write_emissions: bool = True
    write_battery: bool = True
    write_fcd: bool = True

@dataclass
class CarlaConfig:
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
    carla_home: str | None = None
    tls_manager: str = "sumo"
    step_length: float = 0.05
    sync_vehicle_all: bool = True
    guess_tls: bool = True

@dataclass
class ScenarioConfig:
    scenario_id: str
    random_seed: int
    simulation: SimulationTimeConfig
    demand: DemandConfig
    fleet: FleetConfig
    sumo: SumoConfig
    carla: CarlaConfig
    cosim: CoSimConfig
```

If the project supports Python versions where `list[str]` is not supported, use `List[str]` from `typing`.

## Scenario config loader

Support `.json` without extra dependencies. Support `.yaml` or `.yml` only if `PyYAML` is available. If a user provides YAML and PyYAML is missing, raise a clear error with installation guidance.

Suggested function:

```python
def load_scenario_config(path: Path) -> ScenarioConfig:
    """Load a scenario configuration from JSON or YAML."""
```

## Example scenario schema

```yaml
scenario_id: ev50
random_seed: 812
simulation:
  begin: 0
  end: 3600
  step_length: 1.0
  carla_fixed_delta_seconds: 0.05
demand:
  method: random_trips
  period: 1.0
  seed: 812
  validate_routes: true
fleet:
  ev_share: 0.50
  ice_share: 0.50
  ice_emission_class: HBEFA3/PC_G_EU4
  ev_emission_class: Energy/unknown
sumo:
  netconvert_options: []
  random_trips_options: []
  write_tripinfo: true
  write_emissions: true
  write_battery: true
  write_fcd: true
carla:
  host: 127.0.0.1
  port: 2000
  timeout: 30
  vertex_distance: 2.0
  max_road_length: 50.0
  wall_height: 1.0
  additional_width: 0.6
  smooth_junctions: true
cosim:
  carla_home: null
  tls_manager: sumo
  step_length: 0.05
  sync_vehicle_all: true
  guess_tls: true
```

---

# Step 7. Generate the SUMO simulation

## Required behavior

The SUMO builder must create:

```text
sumo/network.net.xml
sumo/routes.rou.xml
sumo/vtypes.add.xml
sumo/scenario.sumocfg
sumo/run_sumo.sh
sumo/run_sumo.bat
sumo/outputs/
```

## SUMO tool discovery

Implement helper functions:

```python
def find_executable(name: str) -> str | None:
    """Return the executable path if available on PATH."""


def find_sumo_tool(tool_name: str) -> Path | None:
    """Find SUMO tools using SUMO_HOME first, then PATH."""
```

Detection rules:

1. `netconvert` should be found on PATH.
2. `sumo` and `sumo-gui` should be found on PATH.
3. `randomTrips.py` should be found under `$SUMO_HOME/tools/randomTrips.py` when available.
4. In dry run, missing tools should be warnings, not fatal errors.
5. In real run, missing tools should raise `RuntimeError` with clear guidance.

## Convert OpenDRIVE to SUMO network

Preferred command:

```bash
netconvert --opendrive-files network/prepared.xodr -o sumo/network.net.xml
```

If PyOpenDRIVE already provides `xodr_to_net_xml`, use it when available. Otherwise use `netconvert` directly.

Suggested function:

```python
def build_sumo_network(prepared_xodr: Path, net_xml: Path, options: list[str], dry_run: bool) -> list[str]:
    """Build or plan a SUMO network conversion command."""
```

Return the command list for logging and reproducibility.

## Generate demand and routes

For the first implementation, support `random_trips`.

Preferred command:

```bash
python "$SUMO_HOME/tools/randomTrips.py" \
  -n sumo/network.net.xml \
  -r sumo/routes.rou.xml \
  --begin 0 \
  --end 3600 \
  --period 1.0 \
  --seed 42 \
  --validate
```

If `randomTrips.py` is missing, write a placeholder route-generation script and raise a clear error only when the user tries to run it.

## Generate mixed ICE and EV vehicle types

Create `vtypes.add.xml`. Include both ICE and EV types. Use `Energy/unknown` for EV energy tracking. Include battery device parameters for EVs.

Example output:

```xml
<additional>
    <vType id="ice_passenger"
           vClass="passenger"
           accel="2.6"
           decel="4.5"
           sigma="0.5"
           length="5.0"
           maxSpeed="25.0"
           emissionClass="HBEFA3/PC_G_EU4"/>

    <vType id="ev_passenger"
           vClass="passenger"
           accel="2.6"
           decel="4.5"
           sigma="0.5"
           length="5.0"
           maxSpeed="25.0"
           emissionClass="Energy/unknown"
           mass="1830">
        <param key="has.battery.device" value="true"/>
        <param key="device.battery.capacity" value="64000"/>
        <param key="maximumPower" value="150000"/>
        <param key="frontSurfaceArea" value="2.6"/>
        <param key="airDragCoefficient" value="0.35"/>
        <param key="rotatingMass" value="40"/>
        <param key="rollDragCoefficient" value="0.01"/>
        <param key="constantPowerIntake" value="100"/>
        <param key="propulsionEfficiency" value="0.98"/>
        <param key="recuperationEfficiency" value="0.96"/>
    </vType>
</additional>
```

Implementation note: if routes already contain vehicle types, provide a helper to post-process the route file and assign vehicle types by `ev_share` and `random_seed`.

Suggested function:

```python
def assign_vehicle_types_to_routes(route_file: Path, ev_share: float, seed: int) -> None:
    """Assign `ice_passenger` or `ev_passenger` to generated trips or vehicles."""
```

## Generate SUMO config

Create `scenario.sumocfg` with these outputs:

```xml
<configuration>
    <input>
        <net-file value="network.net.xml"/>
        <route-files value="routes.rou.xml"/>
        <additional-files value="vtypes.add.xml"/>
    </input>

    <time>
        <begin value="0"/>
        <end value="3600"/>
        <step-length value="1.0"/>
    </time>

    <output>
        <tripinfo-output value="outputs/tripinfo.xml"/>
        <emission-output value="outputs/emissions.xml"/>
        <battery-output value="outputs/battery.xml"/>
        <summary-output value="outputs/summary.xml"/>
        <statistic-output value="outputs/statistics.xml"/>
        <fcd-output value="outputs/fcd.xml"/>
    </output>

    <processing>
        <ignore-route-errors value="true"/>
        <time-to-teleport value="-1"/>
    </processing>
</configuration>
```

Also allow users to override output options through scenario config.

## Generate run scripts

Create `run_sumo.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
sumo -c scenario.sumocfg
```

Create `run_sumo.bat`:

```bat
@echo off
cd /d %~dp0
sumo -c scenario.sumocfg
```

---

# Step 8. Generate the CARLA simulation

## Required behavior

The CARLA builder must create:

```text
carla/load_opendrive_world.py
carla/collect_telemetry.py
carla/run_carla_client.sh
carla/run_carla_client.bat
carla/outputs/
```

## Load the OpenDRIVE world

Generate a script that:

1. Imports `carla`.
2. Connects to `host` and `port`.
3. Reads `network/prepared.xodr`.
4. Calls `client.generate_opendrive_world(...)`.
5. Applies fixed time-step settings when requested.
6. Prints a clear success message.

Template code to generate:

```python
from pathlib import Path
import carla

XODR_PATH = Path("../network/prepared.xodr")

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(30.0)

xodr_content = XODR_PATH.read_text(encoding="utf-8")

params = carla.OpendriveGenerationParameters(
    vertex_distance=2.0,
    max_road_length=50.0,
    wall_height=1.0,
    additional_width=0.6,
    smooth_junctions=True,
    enable_mesh_visibility=True,
)

world = client.generate_opendrive_world(xodr_content, params)
settings = world.get_settings()
settings.fixed_delta_seconds = 0.05
settings.synchronous_mode = True
world.apply_settings(settings)

print("CARLA OpenDRIVE world generated.")
```

## Collect CARLA telemetry

Generate `collect_telemetry.py` that records:

- `frame`
- `time_s`
- `actor_id`
- `type_id`
- `x`, `y`, `z`
- `roll`, `pitch`, `yaw`
- `vx`, `vy`, `vz`
- `ax`, `ay`, `az`
- `speed_mps`
- `accel_mps2`
- optional `jerk_mps3` computed by actor and time-step

Write output to:

```text
carla/outputs/carla_telemetry.csv
```

Important: CARLA should be used for high-fidelity motion and visualization. Do not claim CARLA is the primary energy model unless a validated CARLA energy model is implemented. Use SUMO as the primary energy and emission source by default.

---

# Step 9. Generate SUMO-CARLA co-simulation

## Required behavior

The co-simulation builder must create:

```text
cosim/create_sumo_vtypes.sh
cosim/create_sumo_vtypes.bat
cosim/netconvert_carla.sh
cosim/netconvert_carla.bat
cosim/run_synchronization.sh
cosim/run_synchronization.bat
cosim/outputs/
```

## Required inputs

- Prepared OpenDRIVE file: `network/prepared.xodr`
- SUMO config: `sumo/scenario.sumocfg`
- CARLA home path from `--carla-home`, `CARLA_HOME`, or scenario config

## CARLA helper scripts

Use CARLA helper scripts when available:

```text
$CARLA_HOME/Co-Simulation/Sumo/util/create_sumo_vtypes.py
$CARLA_HOME/Co-Simulation/Sumo/util/netconvert_carla.py
$CARLA_HOME/Co-Simulation/Sumo/run_synchronization.py
```

## Create SUMO vehicle types compatible with CARLA

Generated command:

```bash
python "$CARLA_HOME/Co-Simulation/Sumo/util/create_sumo_vtypes.py" \
  --carla-host 127.0.0.1 \
  --carla-port 2000 \
  --output-file ../sumo/carlavtypes.rou.xml
```

## Create SUMO network for CARLA synchronization

Generated command:

```bash
python "$CARLA_HOME/Co-Simulation/Sumo/util/netconvert_carla.py" \
  ../network/prepared.xodr \
  --output ../sumo/network_carla_sync.net.xml \
  --guess-tls
```

## Run synchronization

Generated command:

```bash
python "$CARLA_HOME/Co-Simulation/Sumo/run_synchronization.py" \
  ../sumo/scenario.sumocfg \
  --sumo-gui \
  --step-length 0.05 \
  --tls-manager sumo \
  --sync-vehicle-all
```

Implementation notes:

1. Use `--tls-manager sumo` by default for transportation operations studies.
2. Use `--step-length 0.05` by default for high-fidelity synchronization.
3. Provide config options to disable SUMO GUI or use CARLA as traffic-light manager.
4. Co-simulation scripts should not start CARLA automatically unless explicitly requested. Assume the CARLA server is already running.
5. In dry run, write scripts and planned commands only.

---

# Step 10. Experimental scenarios

Create example scenario configs for these cases:

| Scenario            | Description                                                  | Required file                      |
| ------------------- | ------------------------------------------------------------ | ---------------------------------- |
| S0 Baseline         | Existing demand, existing signal timing, mostly ICE vehicles | `scenario_baseline.yaml`         |
| S1 EV25             | 25 percent EV fleet share                                    | `scenario_ev25.yaml`             |
| S2 EV50             | 50 percent EV fleet share                                    | `scenario_ev50.yaml`             |
| S3 EV100            | 100 percent EV fleet share                                   | `scenario_ev100.yaml`            |
| S4 Eco-speed        | Lower max speed or smoother behavior                         | `scenario_eco_speed.yaml`        |
| S5 Signal optimized | Alternative signal timing or coordinated control placeholder | `scenario_signal_optimized.yaml` |

## Scenario defaults

- Baseline EV share: `0.0`
- EV25 EV share: `0.25`
- EV50 EV share: `0.50`
- EV100 EV share: `1.00`
- Simulation duration: `3600` seconds
- SUMO step length: `1.0` second
- CARLA and co-simulation step length: `0.05` second
- Random seeds: start with `42`, then support seed lists for replication

## Multiple seeds

Support this CLI pattern:

```bash
pyopendrive-sim build-batch \
  --xodr datasets/chatt.xodr \
  --out experiments/chatt_trb_batch \
  --scenarios examples/trb_sumo_carla_energy/configs/*.yaml \
  --seeds 1 2 3 4 5 6 7 8 9 10 \
  --build-sumo \
  --dry-run
```

If `build-batch` is too much for the first implementation, document it as future work and implement single-scenario build first.

---

# Step 11. Results to analyze

## SUMO mobility metrics

Parse from `tripinfo.xml`, `statistics.xml`, or both:

- `completed_vehicles`
- `avg_travel_time_s`
- `avg_route_length_m`
- `avg_speed_mps`
- `avg_waiting_time_s`
- `avg_waiting_count`
- `avg_time_loss_s`
- `throughput_veh_per_hour`
- `total_distance_km`

## SUMO energy and emissions metrics

Parse from nested `emissions` and `battery` elements in `tripinfo.xml`, and from `emissions.xml` and `battery.xml` when available:

- `total_fuel_mg`
- `total_co2_mg`
- `total_co_mg`
- `total_hc_mg`
- `total_nox_mg`
- `total_pmx_mg`
- `total_electricity_wh`
- `total_battery_consumed_wh`
- `total_battery_regenerated_wh`
- `avg_final_battery_capacity_wh`
- `electricity_kwh_per_km`
- `co2_g_per_km`

## CARLA telemetry metrics

Parse from `carla_telemetry.csv`:

- `avg_speed_mps`
- `max_speed_mps`
- `avg_accel_mps2`
- `max_accel_mps2`
- `avg_abs_accel_mps2`
- `avg_jerk_mps3`
- `stop_episode_count`
- `vehicle_count`
- `telemetry_duration_s`

## Optional consistency metrics

If vehicle matching exists between SUMO and CARLA:

- `matched_vehicle_count`
- `mean_speed_difference_mps`
- `mean_position_difference_m`
- `route_completion_difference`

If matching does not exist, do not fabricate these metrics. Report them as unavailable.

---

# Step 12. Analysis script and APIs

## Required analysis functions

Implement streaming XML parsing to handle large SUMO outputs.

```python
def read_tripinfo(path: Path) -> list[dict[str, object]]:
    """Read SUMO tripinfo output into row dictionaries."""


def summarize_tripinfo(rows: list[dict[str, object]], scenario_id: str) -> dict[str, object]:
    """Create scenario-level mobility, energy, and emission metrics."""


def read_carla_telemetry(path: Path) -> list[dict[str, object]]:
    """Read CARLA telemetry CSV into row dictionaries."""


def summarize_carla_telemetry(rows: list[dict[str, object]], scenario_id: str) -> dict[str, object]:
    """Create CARLA telemetry summary metrics."""


def write_summary_csv(rows: list[dict[str, object]], path: Path) -> None:
    """Write a list of dictionaries to CSV with stable column order."""
```

If `pandas` is available, helper functions may return DataFrames, but public APIs should still work without pandas.

## XML parser requirements

The `tripinfo.xml` parser must handle rows like this:

```xml
<tripinfo id="veh0" depart="0.00" arrival="300.00" duration="300.00" routeLength="2500.00" waitingTime="20.00" waitingCount="4" timeLoss="50.00" vtype="ev_passenger">
    <emissions CO2_abs="0" fuel_abs="0" electricity_abs="420.5"/>
    <battery actualBatteryCapacity="63500" totalEnergyConsumed="420.5" totalEnergyRegenerated="25.2" depleted="0"/>
</tripinfo>
```

The parser must not fail when nested `emissions` or `battery` elements are missing.

## Analysis CLI

Support:

```bash
pyopendrive-sim analyze --project experiments/chatt_trb --out experiments/chatt_trb/analysis
```

Also support direct file inputs:

```bash
pyopendrive-sim analyze \
  --tripinfo experiments/chatt_trb/sumo/outputs/tripinfo.xml \
  --carla-telemetry experiments/chatt_trb/carla/outputs/carla_telemetry.csv \
  --scenario ev50 \
  --out experiments/chatt_trb/analysis
```

## Required output files

```text
analysis/scenario_summary.csv
analysis/mobility_energy_table.csv
analysis/carla_telemetry_summary.csv
analysis/run_metadata.json
```

Optional files:

```text
analysis/figures/energy_delay_frontier.png
analysis/figures/travel_time_distribution.png
analysis/figures/speed_acceleration_profile.png
analysis/edge_energy_heatmap.geojson
```

---

# Step 13. Statistical analysis

## Multiple simulation runs

Design the code so that 4 scenarios x 10 random seeds can be analyzed.

Recommended project layout:

```text
experiments/chatt_trb_batch/
    baseline_seed01/
    baseline_seed02/
    ev50_seed01/
    ev50_seed02/
    ...
```

## Statistical outputs

Implement a function:

```python
def summarize_replicates(summary_files: list[Path], out_dir: Path) -> None:
    """Combine scenario summaries across random seeds and write confidence intervals."""
```

Output:

```text
analysis/batch_summary.csv
analysis/confidence_intervals.csv
analysis/scenario_pairwise_comparison.csv
```

## Confidence intervals

For each metric by scenario:

```text
mean = average(metric)
std = sample standard deviation(metric)
n = number of seeds
ci95_low = mean - 1.96 * std / sqrt(n)
ci95_high = mean + 1.96 * std / sqrt(n)
```

If `scipy` is available, it is acceptable to use a t-distribution. If not, use the normal approximation and document it.

## Pairwise comparison

Compare each scenario to baseline. If `scipy` is unavailable, write simple differences and percent changes only. If `scipy` is available, include paired t-test or Wilcoxon test as optional outputs.

---

# Step 14. Figures and tables

Generate paper-ready CSV tables first. Figures are optional but useful.

## Required tables

Create these tables as CSV and Markdown:

```text
analysis/table_1_scenarios.csv
analysis/table_2_mobility_results.csv
analysis/table_3_energy_emission_results.csv
analysis/table_4_sumo_carla_consistency.csv
```

## Required figure data

Even if plotting is not implemented, create clean data files:

```text
analysis/figure_4_energy_delay_frontier_data.csv
analysis/figure_5_speed_acceleration_profile_data.csv
```

## Optional figures

If `matplotlib` is available, create:

```text
analysis/figures/energy_delay_frontier.png
analysis/figures/travel_time_distribution.png
analysis/figures/speed_acceleration_profile.png
```

Do not use seaborn. Use one plot per figure. Keep figures simple and reproducible.

---

# Step 15. Structured abstract support

Create a documentation file:

```text
docs/trb_sumo_carla_structured_abstract.md
```

Include this draft and leave placeholders for final numerical results:

```markdown
# Structured Abstract

## Objectives
Urban traffic simulation studies increasingly require integrated assessment of mobility performance and vehicle energy consumption, but preparing consistent networks across microscopic traffic simulators and high-fidelity 3D driving simulators remains difficult. This study develops and evaluates an OpenDRIVE-centered workflow for generating SUMO and CARLA simulations from a shared lane-level road network.

## Methods
The proposed workflow uses PyOpenDRIVE to inspect, edit, validate, and export OpenDRIVE networks. The same network is converted to SUMO for microscopic traffic, emissions, and electric-vehicle battery simulation, and is ingested into CARLA for high-fidelity 3D simulation and vehicle telemetry collection. A real-world urban corridor case study is evaluated under baseline, electrification, eco-speed, and signal-control scenarios using multiple random seeds.

## Findings
Final numerical findings will be populated from `analysis/scenario_summary.csv`, `analysis/mobility_energy_table.csv`, and `analysis/carla_telemetry_summary.csv` after scenario runs are completed.

## Novelty
The study contributes a reproducible OpenDRIVE-based SUMO-CARLA workflow that links lane-level network preparation, co-simulation generation, and joint mobility-energy evaluation.

## Practical Applications
The workflow can support transportation agencies and researchers in evaluating traffic operations, signal strategies, EV adoption impacts, and simulation-ready digital twin scenarios using consistent road network data.
```

Add a small script or documentation note showing how final numbers from CSV outputs should be copied into the abstract.

---

# Step 16. Paper outline support

Create:

```text
docs/trb_sumo_carla_paper_outline.md
```

Include this outline:

```markdown
# TRB Paper Outline

## 1. Introduction
- Mobility-energy assessment need
- Challenge of consistent SUMO and CARLA networks
- Study objectives and contributions

## 2. Background
- OpenDRIVE for lane-level network representation
- SUMO for traffic, emissions, and EV battery simulation
- CARLA for high-fidelity 3D simulation
- Research gap

## 3. Methodology
- PyOpenDRIVE-based network preparation
- SUMO simulation generation
- CARLA simulation generation
- SUMO-CARLA synchronization
- Mobility and energy metrics

## 4. Case Study
- Study network
- Demand generation or calibration
- Scenario design
- Simulation settings

## 5. Results
- Network generation and consistency
- Mobility performance
- Energy and emission performance
- SUMO-CARLA trajectory comparison
- Sensitivity analysis

## 6. Discussion
- Mobility-energy tradeoffs
- Value of dual-resolution simulation
- Implementation considerations

## 7. Conclusions
- Key findings
- Limitations
- Future work
```

This file is not only paper text. It should point to generated outputs that support each section.

---

# Step 17. One-command experiment builder

## Required CLI commands

Implement at least these commands:

```bash
pyopendrive-sim build
pyopendrive-sim run-sumo
pyopendrive-sim analyze
```

Implement these as second priority:

```bash
pyopendrive-sim run-carla
pyopendrive-sim run-cosim
pyopendrive-sim build-batch
pyopendrive-sim analyze-batch
```

## `build` command

Expected arguments:

```bash
pyopendrive-sim build \
  --xodr PATH \
  --out PATH \
  --scenario PATH \
  [--build-sumo] \
  [--build-carla] \
  [--build-cosim] \
  [--carla-home PATH] \
  [--dry-run] \
  [--overwrite]
```

Behavior:

1. Create project directories.
2. Copy or save prepared `.xodr`.
3. Write `project_config.json`.
4. Write `network_summary.json`.
5. Build selected subprojects.
6. Write run scripts.
7. Print next-step commands.

## `run-sumo` command

Expected arguments:

```bash
pyopendrive-sim run-sumo --project PATH [--gui] [--extra-args ...]
```

Behavior:

1. Find `scenario.sumocfg`.
2. Run `sumo` or `sumo-gui`.
3. Write command log to `sumo/outputs/run_sumo_command.json`.
4. Fail with clear error if SUMO is unavailable.

## `run-carla` command

Expected arguments:

```bash
pyopendrive-sim run-carla --project PATH [--host 127.0.0.1] [--port 2000]
```

Behavior:

1. Run `carla/load_opendrive_world.py`.
2. Optionally run telemetry collection.
3. Fail with clear error if CARLA Python API is unavailable or server cannot be reached.

## `run-cosim` command

Expected arguments:

```bash
pyopendrive-sim run-cosim --project PATH [--carla-home PATH] [--sumo-gui]
```

Behavior:

1. Validate CARLA helper script paths.
2. Validate SUMO availability.
3. Run or print synchronization command.
4. Write logs under `cosim/outputs/`.

## `analyze` command

Expected arguments:

```bash
pyopendrive-sim analyze --project PATH --out PATH
```

Behavior:

1. Read SUMO outputs if available.
2. Read CARLA telemetry if available.
3. Write summary CSVs and Markdown tables.
4. Create figures if plotting dependencies are available.
5. Never fail only because CARLA outputs are absent. Instead, mark CARLA metrics as unavailable.

---

# Step 18. Final paper positioning converted to documentation goal

Update `README.md` or create `docs/trb_sumo_carla_workflow.md` with this positioning:

```markdown
This workflow presents an OpenDRIVE-based process for generating consistent SUMO and CARLA simulations and demonstrates its use for joint mobility and energy assessment in an urban transportation network. PyOpenDRIVE is used as the network inspection, editing, and conversion layer; SUMO provides scalable traffic, emission, and electric-vehicle energy simulation; and CARLA provides high-fidelity 3D vehicle simulation and telemetry. The resulting workflow supports reproducible evaluation of traffic control, demand, and electrification scenarios for transportation digital twins.
```

The documentation must include:

1. Installation requirements.
2. Optional SUMO and CARLA requirements.
3. One dry-run example.
4. One SUMO-only example.
5. One CARLA-only example.
6. One SUMO-CARLA co-simulation example.
7. One analysis example.
8. Expected output files.
9. Known limitations.
10. Citation-ready paper description.

---

# Validation and acceptance checks

A task is done only when these checks pass or are clearly documented as skipped.

## Core tests

```bash
python -m pytest tests/sim -q
```

Expected behavior:

- Config loading tests pass.
- SUMO builder dry-run tests pass without SUMO installed.
- Analysis parser tests pass with sample XML and CSV files.
- CLI `--help` tests pass.

## Existing tests

```bash
python -m pytest tests -q
```

If existing tests require external data or tools, document skipped tests and reasons.

## Manual dry-run check

```bash
pyopendrive-sim build \
  --xodr datasets/chatt.xodr \
  --out experiments/chatt_trb \
  --scenario examples/trb_sumo_carla_energy/configs/scenario_baseline.yaml \
  --build-sumo \
  --build-carla \
  --build-cosim \
  --dry-run \
  --overwrite
```

Expected files:

```text
experiments/chatt_trb/project_config.json
experiments/chatt_trb/network/prepared.xodr
experiments/chatt_trb/network/network_summary.json
experiments/chatt_trb/sumo/scenario.sumocfg
experiments/chatt_trb/sumo/vtypes.add.xml
experiments/chatt_trb/carla/load_opendrive_world.py
experiments/chatt_trb/carla/collect_telemetry.py
experiments/chatt_trb/cosim/run_synchronization.sh
experiments/chatt_trb/cosim/run_synchronization.bat
```

## Manual analysis check

Use sample outputs first:

```bash
pyopendrive-sim analyze \
  --tripinfo examples/trb_sumo_carla_energy/sample_outputs/tripinfo_sample.xml \
  --carla-telemetry examples/trb_sumo_carla_energy/sample_outputs/carla_telemetry_sample.csv \
  --scenario sample \
  --out /tmp/pyopendrive_analysis_check
```

Expected files:

```text
/tmp/pyopendrive_analysis_check/scenario_summary.csv
/tmp/pyopendrive_analysis_check/mobility_energy_table.csv
/tmp/pyopendrive_analysis_check/carla_telemetry_summary.csv
/tmp/pyopendrive_analysis_check/run_metadata.json
```

---

# Implementation priority order for Codex

1. Inspect the existing repository structure, `pyproject.toml`, README files, and existing conversion utilities.
2. Add `pyopendrive/sim/config.py` and tests for config loading.
3. Add `pyopendrive/sim/project.py` for directory creation and project metadata.
4. Add `pyopendrive/sim/sumo_builder.py` with dry-run support and tests.
5. Add `pyopendrive/sim/analysis.py` with sample XML and CSV parsing tests.
6. Add `pyopendrive/sim/cli.py` and script entry point.
7. Add CARLA builder scripts with dry-run support.
8. Add co-simulation builder scripts with dry-run support.
9. Add example configs and sample outputs.
10. Add docs for TRB workflow, structured abstract, and paper outline.
11. Run tests and update any failing tests with minimal fixes.

Do not start with CARLA integration. Start with dry-run project generation, SUMO configuration generation, and analysis parsing. Those features are testable without heavy external simulators.

---

# Known limitations to document

1. SUMO and CARLA may interpret OpenDRIVE geometry and traffic lights differently.
2. CARLA OpenDRIVE standalone mode depends on the quality and completeness of the `.xodr` file.
3. SUMO is the primary source for energy and emissions in this workflow.
4. CARLA telemetry is used for motion, 3D visualization, and optional trajectory comparison.
5. EV energy parameters are example values and should be calibrated for a specific vehicle fleet before making policy claims.
6. RandomTrips demand is useful for workflow testing but should be replaced or calibrated with observed demand for a final TRB case study.
7. Co-simulation requires external SUMO and CARLA installations and should be tested manually outside normal CI.

---

# Suggested commit message prefixes

Use these prefixes for clarity:

```text
sim-config: add scenario config models
sim-sumo: add SUMO project builder
sim-carla: add CARLA OpenDRIVE scripts
sim-cosim: add SUMO-CARLA co-simulation scripts
sim-analysis: add mobility and energy analysis
sim-cli: add pyopendrive-sim commands
docs-trb: add TRB workflow documentation
tests-sim: add simulation workflow tests
```
