# pyopendrive

`pyopendrive` is a Python package for reading, inspecting, and converting OpenDRIVE `.xodr` networks. Converted from [libopendrive (C++)](https://github.com/pageldev/libOpenDRIVE) to Python with multiple additions.

## Features

- Parse OpenDRIVE files into an object model with roads, lanes, junctions, objects, and signals.
- Query geometry, lane sections, road marks, routing graphs, and derived meshes.
- Save loaded maps back to `.xodr`.
- Convert OpenDRIVE maps to SUMO networks and convert SUMO networks back to OpenDRIVE.
- Use the bundled browser viewer for exploring `.xodr` files locally.

> [!NOTE]
> This file is adapted from [libopendrive (C++)](https://github.com/pageldev/libOpenDRIVE) with additional modifications
>
> Modifications:
>
> - Converted from C++ to Python. Licensed: Apache 2
> - Refactored APIs for Python package usage.
> - Integrated web viewer. Licensed: Apache 2
> - OpenDrive coordination conversion: LonLat2XY, XY2LonLat
> - OpenDrive to SUMO:  xodr_to_net_xml, xodr_from_net_xml
> - Create OpenDrive Network, edit, inspecting ect...

## Installation

Install from the repository root:

```powershell
pip install -e .
```

The package depends on `sumolib` for SUMO conversion support. To use the SUMO conversion helpers, you also need the SUMO `netconvert` executable available on `PATH`.

## Tutorial

<details open>
<summary><b>Click to expand/collapse Quick Start</b></summary>

### Load an OpenDRIVE map (.xodr) and inspect its contents

```python
import pyopendrive as odr

Map = odr.readXodr("datasets/chatt.xodr")
# Map = odr.OpenDriveMap()
# Map = Map.loadXodr("datasets/chatt.xodr")

print(len(Map.getRoads()))
print(len(Map.getJunctions()))
print(Map.getRoad("1").name)
```

### Convert the network to LonLat and vice verse

```python
import pyopendrive as odr

x = 1377.14000000
y = 221.29000000

# Convert to WGS1984
lon, lat = Map.convertXY2LonLat(x, y)

# Convert back to original coordinate system
x, y = Map.convertLonLat2XY(lon, lat)

```

### Open the Web Viewer

```python
import pyopendrive as odr

# This will open the web viewer to your browser
odr.xodr_web_viewer()

```

### Convert XODR to and from SUMO net xml

```python
import pyopendrive as odr

path_xodr = "datasets/chatt.xodr"
path_net = "datasets/chatt.net.xml"

Map = odr.readXodr(path_xodr)

# Convert OpenDrive to SUMO network
sumo_net = xodr_to_net_xml(xodr_file, path_net)

# Convert OpenDrive from SUMO network file
Map = xodr_from_net_xml(net_file, xodr_file)

```

If `netconvert` is not available, the SUMO helpers will raise an error.

### Save a map back to OpenDRIVE

```python
saved_path = Map.saveXodr("output/saved.xodr")
```

### Query Map Information

```python
import pyopendrive as odr

path_xodr = "datasets/chatt.xodr"

Map = odr.readXodr(path_xodr)

roads = Map.getRoads()
junctions = Map.getJunctions()
graph = Map.getRoutingGraph()

lane_count = len(Map.getLanes())
geometry_count = sum(len(road.ref_line.s0_to_geometry) for road in roads)
object_count = sum(len(road.id_to_object) for road in roads)
signal_count = sum(len(road.id_to_signal) for road in roads)

first_road = roads[0]
s = min(1.0, first_road.length)
xyz = first_road.ref_line.get_xyz(s)
surface = first_road.get_surface_pt(s, 0.0)

print(f"roads={len(roads)}")
print(f"junctions={len(junctions)}")
print(f"lane_sections={sum(len(road.s_to_lanesection) for road in roads)}")
print(f"lanes={lane_count}")
print(f"geometries={geometry_count}")
print(f"objects={object_count}")
print(f"signals={signal_count}")
print(f"routing_edges={len(graph.edges)}")
print(f"sample_ref_xyz={xyz}")
print(f"sample_surface_xyz={surface}")

```

</details>

The repository includes a sample map at [datasets/chatt.xodr](datasets/chatt.xodr) and a simple smoke-test script at [tutorial.py](tutorial.py).

There is also a browser-based viewer in [web](web) that can be opened locally or served from a static HTTP server. See [pyopendrive/web/README.md](pyopendrive/web/README.md) for viewer instructions.

## Testing

Run the test suite with:

```powershell
pytest

# or generate coverage report (pytest-cov requried)
pytest --cov=pyopendrive --cov-report=html  --cov-report=term

# or generate pylint report (pylint required)
pylint . --output=pylint_report.txt
```

The SUMO round-trip tests are skipped automatically when `netconvert` is not installed.

## License

`pyopendrive` is licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for the full text.

The repository also includes third-party assets under [third_party](third_party) with their own notices and licenses.
