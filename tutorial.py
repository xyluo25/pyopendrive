"""Smoke-test the pure-Python OpenDRIVE port with the repository map.

Run from the repository root:

    python -m pyopendrive.example_chatt
"""

from pathlib import Path
try:
    from . import pyopendrive
    from .pyopendrive import OpenDriveMap
except ImportError:
    import pyopendrive
    from pyopendrive import OpenDriveMap


if __name__ == "__main__":
    xodr = Path(__file__).resolve().parent.parent / "chatt.xodr"
    xodr = "./chatt.xodr"
    odr_map = OpenDriveMap(str(xodr))

    roads = odr_map.get_roads()
    junctions = odr_map.get_junctions()
    graph = odr_map.get_routing_graph()

    lane_count = sum(len(section.id_to_lane)
                     for road in roads for section in road.get_lanesections())
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
