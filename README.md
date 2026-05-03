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
from pyopendrive import readXodr

odr_map = readXodr("datasets/chatt.xodr")

print(len(odr_map.getRoads()))
print(len(odr_map.getJunctions()))
print(odr_map.getRoad("1").name)
```

## Common Tasks

### Save a map back to OpenDRIVE

```python
saved_path = odr_map.saveXodr("output/saved.xodr")
```

### Query lane geometry

```python
road = odr_map.getRoad("1")
section = road.get_lanesection(0.0)
lane = section.get_lane(1)

xyz = road.get_xyz(5.0, 1.0, 0.5)
surface_point = road.get_surface_pt(5.0, 1.0)
roadmarks = lane.get_roadmarks(0.0, road.length)
```

### Convert to and from SUMO

```python
from pyopendrive import xodr_to_net_xml, xodr_from_net_xml

xodr_file = "datasets/chatt.xodr"
net_file = "datasets/chatt.net.xml"

sumo_net = xodr_to_net_xml(xodr_file, net_file)
roundtrip_map = xodr_from_net_xml(net_file, xodr_file)

# SUMO Net object as input to save to xodr
# roundtrip_map = xodr_from_net_xml(net=sumo_net, xodr_file="build/chatt_roundtrip.xodr")

```

If `netconvert` is not available, the SUMO helpers will raise an error.

## Bundled Data and Viewer

The repository includes a sample map at [datasets/chatt.xodr](datasets/chatt.xodr) and a simple smoke-test script at [tutorial.py](tutorial.py).

There is also a browser-based viewer in [web](web) that can be opened locally or served from a static HTTP server. See [pyopendrive/web/README.md](pyopendrive/web/README.md) for viewer instructions.

## Testing

Run the test suite with:

```powershell
pytest
```

The SUMO round-trip tests are skipped automatically when `netconvert` is not installed.

## License

`pyopendrive` is licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for the full text.

The repository also includes third-party assets under [third_party](third_party) with their own notices and licenses.
