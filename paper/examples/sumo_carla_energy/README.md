# SUMO-CARLA Energy Workflow Example

This example uses the bundled Tempe network under `paper/examples/datasets/tempe_net`.
No root-level `datasets/` files are required.

## Dry-Run Build

```bash
python -m pyopendrive.sim.cli build
  --xodr paper/examples/datasets/tempe_net/tempe.xodr
  --out paper/examples/sumo_carla_energy/generated/tempe_baseline
  --scenario paper/examples/sumo_carla_energy/configs/scenario_baseline.yaml
  --build-sumo
  --build-carla
  --build-cosim
  --dry-run
  --overwrite
```

The dry run copies the Tempe SUMO files from `paper/examples/datasets/tempe_net`,
writes vehicle-type mixes, and creates CARLA/co-simulation helper scripts without
requiring SUMO or CARLA to be installed.

## SUMO-Only Workflow

Use this path when CARLA is not installed or you only need SUMO mobility and
energy-consumption outputs.

```bash
python -m pyopendrive.sim.cli sumo-only
  --xodr paper/examples/datasets/tempe_net/tempe.xodr
  --out paper/examples/sumo_carla_energy/generated/tempe_sumo_only
  --scenario paper/examples/sumo_carla_energy/configs/scenario_sumo_from_xodr.yaml
  --overwrite
```

The command builds the SUMO project, runs `sumo -c scenario.sumocfg`, and writes
SUMO-only analysis files under `generated/tempe_sumo_only/analysis`. For Tempe,
the builder reads `tempe.sumocfg` and reuses `tempe.net.xml`, `tempe.rou.xml`,
`tempe.add.xml`, and companion trip/flow files from `paper/examples/datasets`
instead of regenerating routes. If no companion files exist, it falls back to
SUMO `netconvert` and `randomTrips.py`. SUMO must be installed and available
through `PATH` or `SUMO_HOME`. CARLA is not imported or analyzed.

Analysis uses `matplotlib` and writes high-resolution 1200-DPI PNG figures under
`generated/tempe_sumo_only/analysis/figures`, including travel time
distributions, route length versus travel time, and mobility-energy summary
plots. If CARLA telemetry is available, CARLA speed and speed-acceleration
figures are added to the same folder.

The example scenario runs a 10-minute Tempe window and disables high-volume
battery, FCD, and detailed SUMO XML outputs so the source-route smoke run
finishes quickly. Set `use_source_sumocfg_time: true` and enable those output
flags in the scenario YAML when the full source SUMO timing and detailed outputs
are needed.

## Ego Vehicle Tutorial

Ego vehicle behavior is controlled in the scenario YAML file, not in the Python
scripts. For the SUMO-only Tempe example, edit:

```text
paper/examples/sumo_carla_energy/configs/scenario_sumo_from_xodr.yaml
```

For the other paper scenarios, edit the matching YAML file in the same folder,
for example `scenario_baseline.yaml`, `scenario_ev25.yaml`, or
`scenario_signal_optimized.yaml`. The code reads the `ego:` block from whichever
scenario file is passed with `--scenario`.

### What The Ego Block Does

The `ego:` block has three jobs:

1. Choose which SUMO vehicles are ego vehicles.
2. Define the SUMO vehicle type and vehicle attributes for those ego vehicles.
3. Highlight ego vehicles in SUMO GUI and route outputs.
4. Tell the analysis and CARLA telemetry collection whether to keep only ego
   vehicles.

The current Tempe SUMO-only config uses this structure:

```yaml
ego:
  enabled: true
  mode: select_existing
  highlight_enabled: true
  highlight_color: "0,0,255"
  vehicle_ids:
    - "141_5269.0"
    - "172_417.0"
  auto_select_count: 0
  selection_strategy: first
  vehicle_type_id: ego_passenger
  vtype_attributes:
    vClass: passenger
    accel: "3.0"
    decel: "4.5"
    sigma: "0.0"
    length: "5.0"
    maxSpeed: "25.0"
  vtype_params: {}
  vehicle_attributes:
    departLane: best
    departSpeed: max
  added_vehicles: []
  od_pairs: []
  route_vehicles: []
  vehicles: []
  carla_actor_ids: []
  carla_role_names: []
  carla_type_ids: []
  collect_only_ego: true
  analyze_only_ego: true
```

When `enabled: true`, the SUMO builder writes
`generated/<project>/sumo/ego_vehicles.json` and changes the selected vehicles
inside `generated/<project>/sumo/routes.rou.xml` to use `type="ego_passenger"`.
When `highlight_enabled: true`, the builder also writes the configured
`highlight_color` onto the ego SUMO `<vType>` and each ego `<vehicle>` or
`<trip>`, so the ego vehicles are visually distinct in SUMO GUI and easy to
find in the route file.
When `analyze_only_ego: true`, `scenario_summary.csv`,
`mobility_energy_table.csv`, and `ego_tripinfo.csv` are calculated only from the
ego vehicles. When `collect_only_ego: true`, the generated CARLA telemetry
script writes only matching ego vehicle actors to `carla_telemetry.csv`.

If CARLA is not installed and you run the `sumo-only` command, CARLA is skipped.
The SUMO analysis still uses the same ego settings and leaves CARLA metrics
unreported.

### Ego Modes

There are two supported ego modes:

- `mode: select_existing`: mark vehicles that are already present in the SUMO
  route file. This mode uses `vehicle_ids`, `auto_select_count`, and
  `selection_strategy`.
- `mode: add`: append new ego vehicles to the SUMO route file. This mode uses
  `od_pairs`, `route_vehicles`, and `added_vehicles`.

Use `select_existing` when the source route file already contains the desired
vehicles. Use `add` when the user wants to place new ego vehicles onto the
network.

### Mode 1: Select Existing SUMO Vehicles By ID

Use `vehicle_ids` when the route file already contains the SUMO vehicles you
want to study. This is the most reproducible option because the same vehicle IDs
are selected every time.

```yaml
ego:
  enabled: true
  mode: select_existing
  vehicle_ids:
    - "141_5269.0"
    - "172_417.0"
  auto_select_count: 0
```

For the Tempe dataset, the builder discovers and reuses:

```text
paper/examples/datasets/tempe_net/tempe.rou.xml
```

To choose different ego vehicles, open that route file and copy the `id` values
from `<vehicle ...>` records. You can also inspect the generated route file
after a build:

```text
paper/examples/sumo_carla_energy/generated/tempe_sumo_only/sumo/routes.rou.xml
```

If the ego vehicles should appear in the SUMO result table, choose vehicles that
depart and arrive inside the configured simulation window. The smoke-run window
is:

```yaml
simulation:
  begin: 27000
  end: 27600
```

The sample IDs `141_5269.0` and `172_417.0` finish inside that 10-minute window,
so they produce rows in:

```text
paper/examples/sumo_carla_energy/generated/tempe_sumo_only/analysis/ego_tripinfo.csv
```

If you choose IDs that do not finish before `simulation.end`, SUMO can still run,
but `ego_tripinfo.csv` may be empty because `tripinfo.xml` only includes
completed trips.

### Mode 1: Auto-Select Existing Ego Vehicles

Use `auto_select_count` when you do not care which specific vehicles are ego
vehicles and only need a fixed number of ego vehicles.

```yaml
ego:
  enabled: true
  mode: select_existing
  vehicle_ids: []
  auto_select_count: 3
  selection_strategy: first
  auto_select_seed: 42
```

Supported selection strategies are:

- `first`: select the first available vehicle records in the route file.
- `random`: shuffle vehicle records using `auto_select_seed` when provided,
  otherwise using `random_seed` from the scenario.

Example random selection:

```yaml
random_seed: 42
ego:
  enabled: true
  mode: select_existing
  vehicle_ids: []
  auto_select_count: 5
  selection_strategy: random
  auto_select_seed: 2026
```

Auto-selection is useful for quick experiments, but it can select vehicles that
do not complete within a short simulation window. For publication runs, prefer
explicit `vehicle_ids` after checking the selected vehicles complete in
`tripinfo.xml`.

### Mode 2: Add New Ego Vehicles

Use `mode: add` when the route file does not already contain the ego vehicle you
need. Each added ego vehicle must include an `id`. The builder appends the new
ego vehicles to `routes.rou.xml`, applies the ego vehicle type, and applies the
highlight color.

There are two recommended add-mode patterns.

The first pattern is an OD pair. SUMO receives a `<trip>` with `from` and `to`,
then SUMO chooses a route across the network when the simulation runs:

```yaml
ego:
  enabled: true
  mode: add
  vehicle_ids: []
  od_pairs:
    - id: ego_od_001
      depart: "27000"
      from: "141_341"
      to: "341_5269"
```

The OD keys also accept readable aliases:

```yaml
ego:
  enabled: true
  mode: add
  od_pairs:
    - id: ego_od_002
      depart: "27000"
      origin_edge: "141_341"
      destination_edge: "341_5269"
```

The second pattern is an explicit route or edge list. Use this when the ego
vehicle must follow a known edge sequence:

```yaml
ego:
  enabled: true
  mode: add
  vehicle_ids: []
  route_vehicles:
    - id: ego_route_001
      depart: "27000"
      route_edges:
        - "141_341"
        - "341_5269"
```

`route_vehicles` also accepts `edges` or `edge_list` as aliases for
`route_edges`.

If you already have a named SUMO `<route id="...">` in the route file, reference
it with `route_id`:

```yaml
ego:
  enabled: true
  mode: add
  route_vehicles:
    - id: ego_named_route_001
      depart: "27000"
      route_id: existing_route_id
```

For advanced cases, use `added_vehicles`. This list passes most keys directly
to SUMO, while still understanding `route_edges`, `edges`, `edge_list`,
`from_edge`, `origin_edge`, `to_edge`, and `destination_edge` aliases:

```yaml
ego:
  enabled: true
  mode: add
  added_vehicles:
    - id: ego_custom_001
      depart: "27000"
      departLane: best
      departSpeed: max
      edges:
        - "141_341"
        - "341_5269"
```

The older `vehicles` list is still accepted for backward compatibility, but new
configs should prefer `od_pairs`, `route_vehicles`, or `added_vehicles` because
their intent is clearer.

### Where To Change SUMO Ego Parameters

SUMO ego vehicle parameters are split into three places so the config stays easy
to edit.

Change `vehicle_type_id` when you want a different SUMO type name:

```yaml
ego:
  vehicle_type_id: ego_passenger
```

Change `highlight_enabled` and `highlight_color` when you want ego vehicles to
stand out in SUMO GUI and in `routes.rou.xml`:

```yaml
ego:
  highlight_enabled: true
  highlight_color: "0,0,255"
```

The color uses SUMO's normal color format, usually `"R,G,B"` or
`"R,G,B,A"`. If `highlight_enabled` is true, the builder sets this color on the
ego `<vType>` and each ego `<vehicle>` or `<trip>`. A per-vehicle `color` value
inside `vehicle_attributes`, `added_vehicles`, `od_pairs`, or `route_vehicles`
can still override the default highlight color for a specific case.

Change `vtype_attributes` for attributes written on the SUMO `<vType>` element:

```yaml
ego:
  vtype_attributes:
    vClass: passenger
    accel: "3.0"
    decel: "4.5"
    sigma: "0.0"
    length: "5.0"
    maxSpeed: "25.0"
```

Common SUMO `<vType>` attributes include `vClass`, `accel`, `decel`, `sigma`,
`length`, `minGap`, `maxSpeed`, `speedFactor`, `speedDev`, `emissionClass`,
`guiShape`, and `color`. The builder passes these through to SUMO; it does not
rename them. Prefer `highlight_color` for normal ego highlighting, and use
`vtype_attributes.color` only when you intentionally want to override that
highlight color at the vehicle-type level.

Change `vtype_params` for SUMO `<param>` children under the ego vehicle type.
This is where energy or battery-model parameters can be placed:

```yaml
ego:
  vtype_params:
    has.battery.device: "true"
    device.battery.capacity: "64000"
    maximumPower: "150000"
    recuperationEfficiency: "0.96"
```

Change `vehicle_attributes` for attributes written on each selected ego
`<vehicle>` or `<trip>` element:

```yaml
ego:
  vehicle_attributes:
    departLane: best
    departSpeed: max
```

Common per-vehicle SUMO attributes include `depart`, `departLane`,
`departPos`, `departSpeed`, `arrivalLane`, `arrivalPos`, `arrivalSpeed`,
`color`, `line`, and `personNumber`. These attributes are applied to every ego
vehicle selected through `vehicle_ids` or `auto_select_count`, and to every
vehicle appended through add mode. For one-off values, place the attribute
directly under a specific item in `od_pairs`, `route_vehicles`, or
`added_vehicles`.

### Where To Change Ego-Only Output Behavior

Use these two flags to decide whether outputs keep all vehicles or only ego
vehicles:

```yaml
ego:
  collect_only_ego: true
  analyze_only_ego: true
```

Set `analyze_only_ego: true` when result CSVs should summarize only ego
vehicles. The analysis output then includes `ego_tripinfo.csv`, and the scenario
summary is calculated from only those ego rows.

Set `analyze_only_ego: false` when the ego vehicles should still be marked in
the generated SUMO project, but the analysis should summarize the full traffic
stream.

Set `collect_only_ego: true` when the generated CARLA telemetry script should
write only ego actors. This keeps co-simulation telemetry files small. Set it to
`false` when you need all CARLA vehicle actors in the telemetry CSV.

### CARLA Ego Matching

In SUMO-only runs, no CARLA matching is needed. In co-simulation runs, the
generated CARLA telemetry script tries to match ego actors using several fields:

```yaml
ego:
  vehicle_ids:
    - "141_5269.0"
  carla_actor_ids: []
  carla_role_names: []
  carla_type_ids: []
```

The script marks a CARLA actor as ego if any of these are true:

- The actor's SUMO id attribute matches a value in `vehicle_ids`.
- The actor `role_name` matches a value in `vehicle_ids` or
  `carla_role_names`.
- The actor id matches a value in `carla_actor_ids`.
- The actor `type_id` matches a value in `carla_type_ids`.

When the project is built with `--build-sumo --build-carla --build-cosim`, the
builder first resolves the SUMO ego vehicle IDs, including add-mode IDs, then
passes those IDs into the generated CARLA and co-simulation files.

Use `carla_role_names` when your CARLA spawning or synchronization setup assigns
stable role names. Use `carla_actor_ids` only for controlled tests where actor
ids are known ahead of time, because actor ids can change between runs. Use
`carla_type_ids` only when the selected CARLA blueprint type uniquely identifies
the ego vehicle; otherwise it may match more vehicles than intended.

### Run And Verify An Ego Configuration

After editing the scenario YAML, rebuild and run the SUMO-only example:

```bash
python -m pyopendrive.sim.cli sumo-only
  --xodr paper/examples/datasets/tempe_net/tempe.xodr
  --out paper/examples/sumo_carla_energy/generated/tempe_sumo_only
  --scenario paper/examples/sumo_carla_energy/configs/scenario_sumo_from_xodr.yaml
  --overwrite
```

Check the generated ego metadata:

```text
paper/examples/sumo_carla_energy/generated/tempe_sumo_only/sumo/ego_vehicles.json
```

Check that the route file contains the ego vehicle type:

```text
paper/examples/sumo_carla_energy/generated/tempe_sumo_only/sumo/routes.rou.xml
```

Look for lines like:

```xml
<vehicle id="141_5269.0" type="ego_passenger" color="0,0,255" ...>
```

Check the ego-only analysis table:

```text
paper/examples/sumo_carla_energy/generated/tempe_sumo_only/analysis/ego_tripinfo.csv
```

Check the scenario summary:

```text
paper/examples/sumo_carla_energy/generated/tempe_sumo_only/analysis/scenario_summary.csv
```

For the provided Tempe ego IDs, the summary should report two completed ego
vehicles for the smoke run. CARLA columns are not analyzed in the SUMO-only
workflow and the summary reports `carla_analysis_status` as skipped.

To analyze an existing project with a temporary ego-id override, use repeated
`--ego-vehicle-id` arguments:

```bash
python -m pyopendrive.sim.cli analyze
  --project paper/examples/sumo_carla_energy/generated/tempe_sumo_only
  --out paper/examples/sumo_carla_energy/generated/tempe_sumo_only/analysis
  --sumo-only
  --ego-vehicle-id 141_5269.0
  --ego-vehicle-id 172_417.0
```

This override is useful for checking different completed vehicles without
editing the YAML. For reproducible runs, put the final IDs back into the
scenario file.

To force route regeneration for a different OpenDRIVE file, set
`auto_discover_source_files: false` under `sumo:` in the scenario YAML. To
analyze an existing SUMO project without CARLA, run:

```bash
python -m pyopendrive.sim.cli analyze
  --project paper/examples/sumo_carla_energy/generated/tempe_sumo_only
  --out paper/examples/sumo_carla_energy/generated/tempe_sumo_only/analysis
  --sumo-only
```

## Step Scripts

```bash
python paper/examples/sumo_carla_energy/01_prepare_network.py
python paper/examples/sumo_carla_energy/02_build_sumo_case.py
python paper/examples/sumo_carla_energy/03_build_carla_case.py
python paper/examples/sumo_carla_energy/07_analyze_results.py
```

`04_run_sumo.py`, `05_run_carla_telemetry.py`, and
`06_run_sumo_carla_cosim.py` require local SUMO and CARLA installations.
