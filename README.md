# pyopendrive

`pyopendrive` is a Python package for reading, inspecting, and converting OpenDRIVE `.xodr` road networks. It wraps the project’s pure-Python OpenDRIVE model and also provides helpers for working with SUMO networks through `sumolib`.

## Features

- Parse OpenDRIVE files into an object model with roads, lanes, junctions, objects, and signals.
- Query geometry, lane sections, road marks, routing graphs, and derived meshes.
- Save loaded maps back to `.xodr`.
- Convert OpenDRIVE maps to SUMO networks and convert SUMO networks back to OpenDRIVE.
- Use the bundled browser viewer for exploring `.xodr` files locally.

## Installation

Install from the repository root:

```powershell
pip install -e .
```

The package depends on `sumolib` for SUMO conversion support. To use the SUMO conversion helpers, you also need the SUMO `netconvert` executable available on `PATH`.

## Quick Start

Load an OpenDRIVE map and inspect its contents:

```python
from pyopendrive import OpenDriveMap

odr_map = OpenDriveMap("datasets/chatt.xodr")

print(len(odr_map.get_roads()))
print(len(odr_map.get_junctions()))
print(odr_map.get_road("1").name)
```

The package also exposes a convenience loader:

```python
from pyopendrive import load

odr_map = load("datasets/chatt.xodr")
```

## Common Tasks

### Save a map back to OpenDRIVE

```python
saved_path = odr_map.save_xodr("output/saved.xodr")
```

### Query lane geometry

```python
road = odr_map.get_road("1")
section = road.get_lanesection(0.0)
lane = section.get_lane(1)

xyz = road.get_xyz(5.0, 1.0, 0.5)
surface_point = road.get_surface_pt(5.0, 1.0)
roadmarks = lane.get_roadmarks(0.0, road.length)
```

### Convert to and from SUMO

```python
from pyopendrive import opendrive_to_sumo_net, sumo_net_to_opendrive_map

sumo_net = opendrive_to_sumo_net("datasets/chatt.xodr", net_file="build/chatt.net.xml")
roundtrip_map = sumo_net_to_opendrive_map(sumo_net, xodr_file="build/chatt_roundtrip.xodr")
```

If `netconvert` is not available, the SUMO helpers will raise an error.

## Bundled Data and Viewer

The repository includes a sample map at [datasets/chatt.xodr](datasets/chatt.xodr) and a simple smoke-test script at [tutorial.py](tutorial.py).

There is also a browser-based viewer in [web](web) that can be opened locally or served from a static HTTP server. See [pyopendrive/__web/README.md](pyopendrive/web/README.md) for viewer instructions.

## Testing

Run the test suite with:

```powershell
pytest
```

The SUMO round-trip tests are skipped automatically when `netconvert` is not installed.

## License

`pyopendrive` is licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for the full text.

The repository also includes third-party assets under [third_party](third_party) with their own notices and licenses.
